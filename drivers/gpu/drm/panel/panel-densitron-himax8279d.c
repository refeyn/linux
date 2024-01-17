// SPDX-License-Identifier: GPL-2.0

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_device.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>

struct panel_cmd {
	char cmd;
	char data;
};

struct panel_desc {
	const struct drm_display_mode *display_mode;
	unsigned int bpc;
	unsigned int width_mm;
	unsigned int height_mm;

	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;
	unsigned int lanes;
	const struct panel_cmd *on_cmds;
	unsigned int on_cmds_num;
};

struct panel_info {
	struct drm_panel base;
	struct mipi_dsi_device *link;
	const struct panel_desc *desc;

	struct gpio_desc *reset_gpio;

	bool prepared;
	bool enabled;

	enum drm_panel_orientation orientation;
};

static inline struct panel_info *to_panel_info(struct drm_panel *panel)
{
	return container_of(panel, struct panel_info, base);
}

static int send_mipi_cmds(struct drm_panel *panel, const struct panel_cmd *cmds)
{
	struct panel_info *pinfo = to_panel_info(panel);
	unsigned int i = 0;
	int err;

	for (i = 0; i < pinfo->desc->on_cmds_num; i++) {
		err = mipi_dsi_dcs_write_buffer(pinfo->link, &cmds[i],
						sizeof(struct panel_cmd));

		if (err < 0)
			return err;
	}

	return 0;
}


static int densitron_panel_disable(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int err;
	pr_alert("BOE DISABLE\n");

	if (!pinfo->enabled)
		return 0;

	err = mipi_dsi_dcs_set_display_off(pinfo->link);
	if (err < 0) {
		dev_err(panel->dev, "failed to set display off: %d\n", err);
		return err;
	}

	pinfo->enabled = false;

	return 0;
}

static int densitron_panel_unprepare(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int err;
	pr_alert("BOE UNPREPARE\n");

	if (!pinfo->prepared)
		return 0;

	err = mipi_dsi_dcs_set_display_off(pinfo->link);
	if (err < 0)
		dev_err(panel->dev, "failed to set display off: %d\n", err);

	err = mipi_dsi_dcs_enter_sleep_mode(pinfo->link);
	if (err < 0)
		dev_err(panel->dev, "failed to enter sleep mode: %d\n", err);

	/* sleep_mode_delay: 1ms - 2ms */
	usleep_range(1000, 2000);

	gpiod_set_value(pinfo->reset_gpio, 0);
	pinfo->prepared = false;

	return 0;
}

static int densitron_panel_prepare(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);

	if (pinfo->prepared)
		return 0;
	pr_alert("BOE PREPARE\n");

	/* reset sequence */
	/* T2: 14ms - 15ms */
	usleep_range(14000, 15000);
	gpiod_set_value(pinfo->reset_gpio, 1);

	/* T3: 1ms - 2ms */
	usleep_range(1000, 2000);
	gpiod_set_value(pinfo->reset_gpio, 0);

	/* T4: 1ms - 2ms */
	usleep_range(1000, 2000);
	gpiod_set_value(pinfo->reset_gpio, 1);

	/* T5: 5ms - 6ms */
	usleep_range(5000, 6000);

	pinfo->prepared = true;

	return 0;
}

static int densitron_panel_enable(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int err;

	if (pinfo->enabled)
		return 0;

	// Do reset
	err = mipi_dsi_dcs_soft_reset(pinfo->link);
	if (err < 0) {
		dev_err(panel->dev, "failed to send soft reset: %d\n", err);
		return err;
	}

	usleep_range(5000, 6000);
	pr_alert("BOE ENABLE\n");

	/* send init code */
	err = send_mipi_cmds(panel, pinfo->desc->on_cmds);
	if (err < 0) {
		dev_err(panel->dev, "failed to send DCS Init Code: %d\n", err);
		return err;
	}

	usleep_range(200000, 210000);

	err = mipi_dsi_dcs_exit_sleep_mode(pinfo->link);
	if (err < 0) {
		dev_err(panel->dev, "failed to exit sleep mode: %d\n", err);
		return err;
	}

	/* T6: 120ms - 121ms */
	usleep_range(120000, 121000);

	err = mipi_dsi_dcs_set_display_on(pinfo->link);
	if (err < 0) {
		dev_err(panel->dev, "failed to set display on: %d\n", err);
		return err;
	}
	usleep_range(50000, 51000);

	pinfo->enabled = true;

	return 0;
}

static int densitron_panel_get_modes(struct drm_panel *panel,
			       struct drm_connector *connector)
{
	struct panel_info *pinfo = to_panel_info(panel);
	const struct drm_display_mode *m = pinfo->desc->display_mode;
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, m);
	if (!mode) {
		dev_err(pinfo->base.dev, "failed to add mode %ux%u@%u\n",
			m->hdisplay, m->vdisplay, drm_mode_vrefresh(m));
		return -ENOMEM;
	}

	pr_alert("BOE DISP MODE %u %d", mode->clock, (int) pinfo->orientation);

	drm_mode_set_name(mode);

	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = pinfo->desc->width_mm;
	connector->display_info.height_mm = pinfo->desc->height_mm;
	connector->display_info.bpc = pinfo->desc->bpc;

	/*
	 * TODO: Remove once all drm drivers call
	 * drm_connector_set_orientation_from_panel()
	 */
	drm_connector_set_panel_orientation(connector, pinfo->orientation);

	return 1;
}

static enum drm_panel_orientation densitron_panel_get_orientation(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);

	return pinfo->orientation;
}

static const struct drm_panel_funcs panel_funcs = {
	.disable = densitron_panel_disable,
	.unprepare = densitron_panel_unprepare,
	.prepare = densitron_panel_prepare,
	.enable = densitron_panel_enable,
	.get_modes = densitron_panel_get_modes,
	.get_orientation = densitron_panel_get_orientation,
};

static const struct drm_display_mode default_display_mode = {
	// TODO put real timings in once we have a good signal path
	.clock = 20000,
	.hdisplay = 1200,
	.hsync_start = 1200 + 40,
	.hsync_end = 1200 + 40 + 10,
	.htotal = 1200 + 40 + 10 + 50,
	.vdisplay = 1920,
	.vsync_start = 1920 + 10,
	.vsync_end = 1920 + 10 + 10,
	.vtotal = 1920 + 10 + 10 + 20,
};

/* DMT101F3NMCMU-1A */
static const struct panel_cmd densitron_dmt101f3nmcmu_1a_on_cmds[] = {
	// PAGE 1 (OTP & GOA MUX)
	{ 0xB0, 0x01 }, { 0xC3, 0x0F }, { 0xC4, 0x00 }, { 0xC5, 0x00 },
	{ 0xC6, 0x00 }, { 0xC7, 0x00 }, { 0xC8, 0x0D }, { 0xC9, 0x12 },
	{ 0xCA, 0x11 }, { 0xCD, 0x1D }, { 0xCE, 0x1B }, { 0xCF, 0x0B },
	{ 0xD0, 0x09 }, { 0xD1, 0x07 }, { 0xD2, 0x05 }, { 0xD3, 0x01 },
	{ 0xD7, 0x10 }, { 0xD8, 0x00 }, { 0xD9, 0x00 }, { 0xDA, 0x00 },
	{ 0xDB, 0x00 }, { 0xDC, 0x0E }, { 0xDD, 0x12 }, { 0xDE, 0x11 },
	{ 0xE1, 0x1E }, { 0xE2, 0x1C }, { 0xE3, 0x0C }, { 0xE4, 0x0A },
	{ 0xE5, 0x08 }, { 0xE6, 0x06 }, { 0xE7, 0x02 },
	// PAGE 3 (GOA)
	{ 0xB0, 0x03 }, { 0xBE, 0x03 }, { 0xCC, 0x44 }, { 0xC8, 0x07 },
	{ 0xC9, 0x05 }, { 0xCA, 0x42 }, { 0xCD, 0x3E }, { 0xCF, 0x60 },
	{ 0xD2, 0x04 }, { 0xD3, 0x04 }, { 0xD4, 0x01 }, { 0xD5, 0x01 },
	{ 0xD6, 0x03 }, { 0xD7, 0x04 }, { 0xD9, 0x01 }, { 0xDB, 0x01 },
	{ 0xE4, 0xF0 }, { 0xE5, 0x0A },
	// PAGE 0
	{ 0xB0, 0x00 }, { 0xBD, 0x63 }, { 0xC2, 0x08 }, { 0xC4, 0x10 },
	// PAGE 2 (analog gamma)
	{ 0xB0, 0x02 }, { 0xC0, 0x00 }, { 0xC1, 0x0A }, { 0xC2, 0x20 },
	{ 0xC3, 0x24 }, { 0xC4, 0x23 }, { 0xC5, 0x29 }, { 0xC6, 0x23 },
	{ 0xC7, 0x1C }, { 0xC8, 0x19 }, { 0xC9, 0x17 }, { 0xCA, 0x17 },
	{ 0xCB, 0x18 }, { 0xCC, 0x1A }, { 0xCD, 0x1E }, { 0xCE, 0x20 },
	{ 0xCF, 0x23 }, { 0xD0, 0x07 }, { 0xD1, 0x00 }, { 0xD2, 0x00 },
	{ 0xD3, 0x0A }, { 0xD4, 0x05 }, { 0xD5, 0x1C }, { 0xD6, 0x1A },
	{ 0xD7, 0x13 }, { 0xD8, 0x17 }, { 0xD9, 0x1C }, { 0xDA, 0x19 },
	{ 0xDB, 0x17 }, { 0xDC, 0x17 }, { 0xDD, 0x18 }, { 0xDE, 0x1A },
	{ 0xDF, 0x1E }, { 0xE0, 0x20 }, { 0xE1, 0x23 }, { 0xE2, 0x07 },
};

static const struct panel_desc densitron_dmt101f3nmcmu_1a_panel_desc = {
	.display_mode = &default_display_mode,
	.bpc = 8,
	.width_mm = 135,
	.height_mm = 216,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM,
	.format = MIPI_DSI_FMT_RGB888,
	.lanes = 4,
	.on_cmds = densitron_dmt101f3nmcmu_1a_on_cmds,
	.on_cmds_num = 89,
};

static const struct of_device_id panel_of_match[] = {
	{
		.compatible = "densitron,dmt101f3nmcmu-1a",
		.data = &densitron_dmt101f3nmcmu_1a_panel_desc,
	},
	{
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, panel_of_match);

static int panel_add(struct panel_info *pinfo)
{
	struct device *dev = &pinfo->link->dev;
	int ret;

	pinfo->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(pinfo->reset_gpio)) {
		ret = PTR_ERR(pinfo->reset_gpio);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to get reset gpio: %d\n", ret);
		// return ret;
	}

	ret = of_drm_get_panel_orientation(dev->of_node, &pinfo->orientation);
	if (ret) {
		dev_err(dev, "%pOF: failed to get orientation %d\n", dev->of_node, ret);
		return ret;
	}
	pinfo->orientation = DRM_MODE_PANEL_ORIENTATION_BOTTOM_UP;

	pr_alert("BOE ADD\n");
	drm_panel_init(&pinfo->base, dev, &panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ret = drm_panel_of_backlight(&pinfo->base);
	if (ret)
		return ret;

	drm_panel_add(&pinfo->base);
	pr_alert("BOE ADD END\n");

	return 0;
}

static int panel_probe(struct mipi_dsi_device *dsi)
{
	struct panel_info *pinfo;
	const struct panel_desc *desc;
	int err;

	pinfo = devm_kzalloc(&dsi->dev, sizeof(*pinfo), GFP_KERNEL);
	if (!pinfo)
		return -ENOMEM;

	pr_alert("BOE PROBE\n");
	desc = of_device_get_match_data(&dsi->dev);
	dsi->mode_flags = desc->mode_flags;
	dsi->format = desc->format;
	dsi->lanes = desc->lanes;
	pinfo->desc = desc;

	pinfo->link = dsi;
	mipi_dsi_set_drvdata(dsi, pinfo);

	err = panel_add(pinfo);
	if (err < 0) {
		pr_alert("BOE PROBE: err in add %d\n", err);
		return err;
	}

	err = mipi_dsi_attach(dsi);
	if (err < 0) {
		pr_alert("BOE PROBE: err in attach %d\n", err);
		drm_panel_remove(&pinfo->base);
	}
	pr_alert("BOE PROBE END\n");

	return err;
}

static void panel_remove(struct mipi_dsi_device *dsi)
{
	struct panel_info *pinfo = mipi_dsi_get_drvdata(dsi);
	int err;

	err = densitron_panel_disable(&pinfo->base);
	if (err < 0)
		dev_err(&dsi->dev, "failed to disable panel: %d\n", err);

	err = densitron_panel_unprepare(&pinfo->base);
	if (err < 0)
		dev_err(&dsi->dev, "failed to unprepare panel: %d\n", err);

	err = mipi_dsi_detach(dsi);
	if (err < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", err);

	drm_panel_remove(&pinfo->base);
}

static void panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct panel_info *pinfo = mipi_dsi_get_drvdata(dsi);

	densitron_panel_disable(&pinfo->base);
	densitron_panel_unprepare(&pinfo->base);
}

static struct mipi_dsi_driver panel_driver = {
	.driver = {
		.name = "panel-densitron-himax8279d",
		.of_match_table = panel_of_match,
	},
	.probe = panel_probe,
	.remove = panel_remove,
	.shutdown = panel_shutdown,
};
module_mipi_dsi_driver(panel_driver);

MODULE_AUTHOR("Matthew Joyce <matthew.joyce@refeyn.com>");
MODULE_DESCRIPTION("Densitron Himax8279d driver");
MODULE_LICENSE("GPL v2");
