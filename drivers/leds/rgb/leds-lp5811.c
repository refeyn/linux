// SPDX-License-Identifier: GPL-2.0-only
/*
 * LED control driver for TI LP5811 Synchronous Boost 4-Channel RGBW LED Driver
 *
 * Copyright 2025 Matthew Joyce <matthew.joyce@refeyn.com>
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/led-class-multicolor.h>
#include <linux/leds.h>
#include <linux/linear_range.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/util_macros.h>

//            ┌────────────────────────────────────────┐
//            │                 LP5811                 │
//            │                                        │
//            │   ┌───────────────────────────────┐    │
//            │   │                               │    │
//            │   │                               │    │
//            │   ▼  ┌────┐     ┌────┐     ┌────┐ │    │
//            │─────►│AEU1├────►│AEU2├────►│AEU3├─┴───►│
//            │      └────┘     └────┘     └────┘      │
//            │                                        │
//            └────────────────────────────────────────┘
//
// ┌──────────────────────────────────────────────────────────────┐
// │                   Animation Engine Unit 1-3                  │
// │                                                              │
// │   ┌─────────────────────────────────────────────────────┐    │
// │   │                                                     │    │
// │   │                                                     │    │
// │   ▼  ┌────┐     ┌────┐     ┌────┐     ┌────┐     ┌────┐ │    │
// │─────►│PWM1│────►│PWM2├────►│PWM3├────►│PWM4├────►│PWM5├─┴───►│
// │      └────┘ T1  └────┘ T2  └────┘ T3  └────┘ T4  └────┘      │
// │                                                              │
// └──────────────────────────────────────────────────────────────┘

// In this driver, brightness corresponds to current, and the pattern corresponds to PWM
// When in blink/pattern mode we use auto control, and use manual control otherwise

// Device registers
#define LP5811_CHIP_EN 0x0
#define LP5811_DEV_CONFIG0 0x1
#define LP5811_DEV_CONFIG0_MAX_CURRENT(x) (x)
#define LP5811_DEV_CONFIG0_VOLTAGE(x) ((x) << 1)
#define LP5811_DEV_CONFIG1 0x2
#define LP5811_DEV_CONFIG3 0x4
#define LP5811_DEV_CONFIG5 0x6
#define LP5811_DEV_CONFIG7 0x8
#define LP5811_DEV_CONFIG11 0xC
#define LP5811_DEV_CONFIG12 0xD

// Command registers
#define LP5811_UPDATE_CMD 0x10
#define LP5811_START_CMD 0x11
#define LP5811_STOP_CMD 0x12
#define LP5811_PAUSE_CMD 0x13
#define LP5811_CONTINUE_CMD 0x14

// LED_EN registers
#define LP5811_LED_EN1 0x20

// Manual DC registers
#define LP5811_MANUAL_DC(x) (0x30 + (x))

// Manual PWM registers
#define LP5811_MANUAL_PWM(x) (0x40 + (x))

// Auto DC registers
#define LP5811_AUTO_DC(x) (0x50 + (x))

// Auto control registers
#define LP5811_LED_PARAMS_LENGTH 0x1A
#define LP5811_AUTO_LED_BASE(x) (0x80 + LP5811_LED_PARAMS_LENGTH * (x))
#define LP5811_PAUSE_TIME_OFFSET 0x0
#define LP5811_PLAYBACK_TIMES_OFFSET 0x1
#define LP5811_AEU_OFFSET(aue) (0x2 + 0x8 * (aue))
#define LP5811_AEU_PWM_OFFSET(aue, x) (LP5811_AEU_OFFSET(aue) + (x))
#define LP5811_AEU_SLOPE_TIME_OFFSET(aue, x) \
	(LP5811_AEU_OFFSET(aue) + 0x5 + (x))
#define LP5811_AEU_PT1_OFFSET 0x7

// Commands
#define LP5811_UPDATE_DATA 0x55
#define LP5811_START_DATA 0xFF
#define LP5811_STOP_DATA 0xAA
#define LP5811_PAUSE_DATA 0x33
#define LP5811_CONTINUE_DATA 0xCC

// Other data
#define LP5811_MAX_LEDS 4
#define LP5811_REG_MAX 0xE7
#define LP5811_LOW_CURRENT_MODE 25500
#define LP5811_HIGH_CURRENT_MODE 51000
#define LP5811_MAX_PATTERN_LEN 12

static const unsigned int aeu_slope_delays[] = { 0,    90,   180,  360,
						 540,  800,  1070, 1520,
						 2060, 2500, 3040, 4020,
						 5010, 5990, 7060, 8050 };
static const struct linear_range voltage_range = { 3000000, 0, 100000, 25 };

static const struct regmap_config lp5811_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = LP5811_REG_MAX,
};

struct lp5811_led {
	struct led_classdev_mc mc;
	struct lp5811_priv *priv;
	enum led_default_state default_state;
};

struct lp5811_priv {
	struct mutex lock;
	struct regmap *regmap;
	unsigned int leds_count;
	unsigned int leds_active;
	unsigned int max_current;
	struct lp5811_led leds[];
};

static int lp5811_set_led_brightness(struct lp5811_priv *priv,
				     unsigned int led_no, unsigned int level)
{
	unsigned int converted_current =
		DIV_ROUND_CLOSEST(level * priv->max_current, LED_FULL);
	return regmap_write(priv->regmap, LP5811_MANUAL_DC(led_no),
			    converted_current) ||
	       regmap_write(priv->regmap, LP5811_MANUAL_PWM(led_no), 0xFF) ||
	       regmap_write(priv->regmap, LP5811_AUTO_DC(led_no),
			    converted_current);
}

static inline int lp5811_turn_off_auto(struct lp5811_priv *priv,
				       struct led_classdev_mc *mccdev)
{
	struct mc_subled *subled;
	int i, ret;
	u32 auto_control;

	ret = regmap_read(priv->regmap, LP5811_DEV_CONFIG3, &auto_control);
	if (ret)
		return ret;

	for (i = 0; i < mccdev->num_colors; i++) {
		subled = mccdev->subled_info + i;
		auto_control &= ~BIT(subled->channel);
	}

	ret = regmap_write(priv->regmap, LP5811_DEV_CONFIG3, auto_control);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, LP5811_UPDATE_CMD, LP5811_UPDATE_DATA);
	if (ret)
		return ret;

	return 0;
}

static int lp5811_mc_brightness_set(struct led_classdev *lcdev,
				    enum led_brightness level)
{
	struct led_classdev_mc *mccdev = lcdev_to_mccdev(lcdev);
	struct lp5811_led *led = container_of(mccdev, struct lp5811_led, mc);
	struct lp5811_priv *priv = led->priv;
	struct mc_subled *subled;
	int i, ret;

	mutex_lock(&priv->lock);

	if (level == LED_OFF) {
		// Deactivate any patterns when brightness is set to zero
		ret = lp5811_turn_off_auto(priv, mccdev);
		if (ret)
			goto out_unlock;
	}

	led_mc_calc_color_components(mccdev, level);

	for (i = 0; i < mccdev->num_colors; i++) {
		u32 brightness;

		subled = mccdev->subled_info + i;
		brightness = min(subled->brightness, lcdev->max_brightness);

		ret = lp5811_set_led_brightness(priv, subled->channel,
						brightness);
		if (ret)
			goto out_unlock;
	}

out_unlock:
	mutex_unlock(&priv->lock);

	return ret;
}

static inline int lp5811_mc_pattern_clear(struct led_classdev *lcdev)
{
	struct led_classdev_mc *mccdev = lcdev_to_mccdev(lcdev);
	struct lp5811_led *led = container_of(mccdev, struct lp5811_led, mc);
	struct lp5811_priv *priv = led->priv;
	int ret;

	mutex_lock(&priv->lock);
	ret = lp5811_turn_off_auto(priv, mccdev);
	mutex_unlock(&priv->lock);

	return ret;
}

static int lp5811_mc_pattern_set(struct led_classdev *lcdev,
				 struct led_pattern *pattern, u32 len,
				 int repeat)
{
	struct led_classdev_mc *mccdev = lcdev_to_mccdev(lcdev);
	struct lp5811_led *led = container_of(mccdev, struct lp5811_led, mc);
	struct lp5811_priv *priv = led->priv;
	struct mc_subled *subled;
	u8 params[LP5811_LED_PARAMS_LENGTH] = { 0 };
	int i, ret, aeu_index, pwm_index, delay_index;
	u32 auto_control;

	if (len >= LP5811_MAX_PATTERN_LEN) {
		return -EINVAL;
	}
	if (len == 0) {
		return lp5811_mc_pattern_clear(lcdev);
	}

	aeu_index = 0;
	pwm_index = 0;
	for (i = 0; i < len; i++) {
		params[LP5811_AEU_PWM_OFFSET(aeu_index, pwm_index)] =
			pattern[i].brightness;

		if (pwm_index == 4) {
			pwm_index = 0;
			++aeu_index;
			params[LP5811_AEU_PWM_OFFSET(aeu_index, pwm_index)] =
				pattern[i].brightness;
		}

		delay_index = find_closest(pattern[i].delta_t, aeu_slope_delays,
					   ARRAY_SIZE(aeu_slope_delays));
		params[LP5811_AEU_SLOPE_TIME_OFFSET(aeu_index, pwm_index / 2)] |=
			delay_index << (4 * (pwm_index % 2));
		pwm_index++;
	}
	for (; pwm_index < 5; ++pwm_index) {
		params[LP5811_AEU_PWM_OFFSET(aeu_index, pwm_index)] =
			pattern[0].brightness;
		// Leave time offset as zero
	}

	// Set number of AEUs in use
	params[LP5811_PLAYBACK_TIMES_OFFSET] =
		(aeu_index << 4) |
		(repeat <= 0 ? 15 : clamp(repeat - 1, 0, 14));

	mutex_lock(&priv->lock);

	for (i = 0; i < mccdev->num_colors; i++) {
		regmap_raw_write(priv->regmap, LP5811_AUTO_LED_BASE(i), &params,
				 LP5811_LED_PARAMS_LENGTH);
	}

	ret = regmap_read(priv->regmap, LP5811_DEV_CONFIG3, &auto_control);
	if (ret)
		goto out_unlock;

	for (i = 0; i < mccdev->num_colors; i++) {
		subled = mccdev->subled_info + i;
		auto_control |= BIT(subled->channel);
	}

	ret = regmap_write(priv->regmap, LP5811_DEV_CONFIG3, auto_control);
	if (ret)
		goto out_unlock;

	ret = regmap_write(priv->regmap, LP5811_UPDATE_CMD, LP5811_UPDATE_DATA);
	if (ret)
		goto out_unlock;

	ret = regmap_write(priv->regmap, LP5811_START_CMD, LP5811_START_DATA);
	if (ret)
		goto out_unlock;

out_unlock:
	mutex_unlock(&priv->lock);

	return ret;
}

static int lp5811_mc_blink_set(struct led_classdev *lcdev,
			       unsigned long *delay_on,
			       unsigned long *delay_off)
{
	struct led_pattern pattern[4];

	if (!*delay_on && !*delay_off)
		*delay_on = *delay_off = 500;

	pattern[0].brightness = LED_FULL;
	pattern[0].delta_t = *delay_on;
	pattern[1].brightness = LED_FULL;
	pattern[1].delta_t = 0;
	pattern[2].brightness = LED_OFF;
	pattern[2].delta_t = *delay_off;
	pattern[3].brightness = LED_OFF;
	pattern[3].delta_t = 0;

	return lp5811_mc_pattern_set(lcdev, pattern, 4, -1);
}

static int lp5811_assign_multicolor_info(struct device *dev,
					 struct lp5811_led *led,
					 struct fwnode_handle *fwnode)
{
	struct lp5811_priv *priv = led->priv;
	struct fwnode_handle *child;
	struct mc_subled *sub_led;
	u32 num_color = 0;
	int ret;

	fwnode_for_each_child_node(fwnode, child) {
		num_color++;
	}

	if (num_color < 2)
		return dev_err_probe(
			dev, -EINVAL,
			"Multicolor must include 2 or more LED channels\n");

	sub_led = devm_kcalloc(dev, num_color, sizeof(*sub_led), GFP_KERNEL);
	if (!sub_led)
		return -ENOMEM;

	num_color = 0;
	fwnode_for_each_child_node(fwnode, child) {
		u32 reg, color;

		ret = fwnode_property_read_u32(child, "reg", &reg);
		if (ret || reg >= LP5811_MAX_LEDS ||
		    priv->leds_active & BIT(reg)) {
			fwnode_handle_put(child);
			return -EINVAL;
		}

		ret = fwnode_property_read_u32(child, "color", &color);
		if (ret) {
			fwnode_handle_put(child);
			return dev_err_probe(
				dev, ret, "LED %d, no color specified\n", reg);
		}

		priv->leds_active |= BIT(reg);
		sub_led[num_color].color_index = color;
		sub_led[num_color].channel = reg;
		++num_color;
	}

	led->mc.num_colors = num_color;
	led->mc.subled_info = sub_led;

	return 0;
}

static int lp5811_init_led_properties(struct device *dev,
				      struct lp5811_led *led,
				      struct led_init_data *init_data)
{
	struct led_classdev *lcdev;
	int ret;

	ret = lp5811_assign_multicolor_info(dev, led, init_data->fwnode);
	if (ret)
		return ret;

	lcdev = &led->mc.led_cdev;
	lcdev->brightness_set_blocking = lp5811_mc_brightness_set;
	lcdev->blink_set = lp5811_mc_blink_set;
	lcdev->pattern_set = lp5811_mc_pattern_set;
	lcdev->pattern_clear = lp5811_mc_pattern_clear;

	lcdev->max_brightness = LED_FULL;
	lcdev->brightness = LED_FULL;

	led->default_state = led_init_default_state_get(init_data->fwnode);

	return 0;
}

static int lp5811_multicolor_led_register(struct device *dev,
					  struct lp5811_led *led,
					  struct led_init_data *init_data)
{
	int ret, i;
	u32 enabled, auto_control;
	struct mc_subled *subled;

	ret = regmap_read(led->priv->regmap, LP5811_LED_EN1, &enabled);
	if (ret)
		return ret;

	ret = regmap_read(led->priv->regmap, LP5811_DEV_CONFIG3, &auto_control);
	if (ret)
		return ret;

	for (i = 0; i < led->mc.num_colors; i++) {
		subled = led->mc.subled_info + i;

		switch (led->default_state) {
		case LEDS_DEFSTATE_ON:
			subled->intensity = led->mc.led_cdev.max_brightness;
			break;
		case LEDS_DEFSTATE_KEEP:
			if (!(enabled & BIT(subled->channel))) {
				subled->intensity = 0;
				break;
			}
			if (auto_control & BIT(subled->channel)) {
				regmap_read(led->priv->regmap,
					    LP5811_AUTO_DC(subled->channel),
					    &subled->intensity);
			} else {
				regmap_read(led->priv->regmap,
					    LP5811_MANUAL_DC(subled->channel),
					    &subled->intensity);
			}
			break;
		default:
			subled->intensity = 0;
		}
	}

	ret = lp5811_mc_brightness_set(&led->mc.led_cdev,
				       led->mc.led_cdev.brightness);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Couldn't set multicolor brightness\n");

	ret = devm_led_classdev_multicolor_register_ext(dev, &led->mc,
							init_data);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Couldn't register multicolor\n");

	return 0;
}

static int lp5811_leds_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct lp5811_priv *priv;
	struct fwnode_handle *child;
	size_t count;
	u32 max_uA, uV;
	unsigned int i = 0, voltage_sel;
	int ret, err;

	count = device_get_child_node_count(dev);
	if (!count || count > LP5811_MAX_LEDS)
		return dev_err_probe(
			dev, -EINVAL,
			"No child node or node count over max LED number %zu\n",
			count);

	priv = devm_kzalloc(dev, struct_size(priv, leds, count), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->leds_count = count;
	mutex_init(&priv->lock);

	priv->regmap = devm_regmap_init_i2c(client, &lp5811_regmap_config);

	if (IS_ERR(priv->regmap)) {
		err = PTR_ERR(priv->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
			err);
		goto fwnode_release;
	}

	ret = of_property_read_u32(dev_of_node(dev), "led-max-microamp",
				   &max_uA);
	if (ret) {
		dev_warn(
			dev,
			"Not specified led-max-microamp, config to the minimum\n");
		max_uA = 0;
	}
	max_uA = clamp(max_uA, 0, LP5811_HIGH_CURRENT_MODE);
	priv->max_current = DIV_ROUND_CLOSEST(max_uA * LED_FULL,
					      max_uA > LP5811_LOW_CURRENT_MODE ?
						      LP5811_HIGH_CURRENT_MODE :
						      LP5811_LOW_CURRENT_MODE);

	ret = of_property_read_u32(dev_of_node(dev), "led-microvolts", &uV);
	if (ret) {
		dev_warn(
			dev,
			"Not specified led-microvolts, config to the minimum\n");
		uV = 0;
	}

	device_for_each_child_node(dev, child) {
		struct lp5811_led *led = priv->leds + i++;
		struct led_init_data init_data = { .fwnode = child };

		led->priv = priv;

		ret = lp5811_init_led_properties(dev, led, &init_data);
		if (ret)
			goto fwnode_release;

		ret = lp5811_multicolor_led_register(dev, led, &init_data);
		if (ret)
			goto fwnode_release;
	}

	linear_range_get_selector_within(&voltage_range, uV, &voltage_sel);

	ret = regmap_write(priv->regmap, LP5811_CHIP_EN, 0x1);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, LP5811_DEV_CONFIG0,
			   LP5811_DEV_CONFIG0_MAX_CURRENT(
				   max_uA > LP5811_LOW_CURRENT_MODE) |
				   LP5811_DEV_CONFIG0_VOLTAGE(voltage_sel));

	if (ret)
		return ret;
	ret = regmap_write(priv->regmap, LP5811_LED_EN1, priv->leds_active);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, LP5811_UPDATE_CMD, LP5811_UPDATE_DATA);
	if (ret)
		return ret;

	return 0;

fwnode_release:
	fwnode_handle_put(child);
	return ret;
}

static const struct of_device_id lp5811_rgbled_device_table[] = {
	{ .compatible = "ti,lp5811" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, lp5811_rgbled_device_table);

static const struct i2c_device_id lp5811_id[] = {
	{ "lp5811", 0 },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, lp5811_id);

static struct i2c_driver lp5811_rgbled_driver = {
    .driver =
        {
            .name = "lp5811",
            .of_match_table = lp5811_rgbled_device_table,
        },
    .probe = lp5811_leds_probe,
    .id_table = lp5811_id,
};
module_i2c_driver(lp5811_rgbled_driver);

MODULE_AUTHOR("Matthew Joyce <matthew.joyce@refeyn.com>");
MODULE_DESCRIPTION("TI LP5811 Synchronous Boost 4-Channel RGBW LED Driver");
MODULE_LICENSE("GPL");
