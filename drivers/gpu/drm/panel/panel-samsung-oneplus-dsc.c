// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 johannSo <jzeug@soppa.de>
// Generated with linux-mdss-dsi-panel-driver-generator from the OnePlus 7T Pro
// (hotdog) vendor device tree (dsi_samsung_oneplus_dsc_cmd):
//   Copyright (c) 2013, The Linux Foundation. All rights reserved.
//
// Samsung AMOLED, 1440x3120 command mode with DSC (8 bpc, 8 bpp, 720x65
// slices, two encoders merged into one DSI link). Shared by the OnePlus 7 Pro
// and 7T Pro.

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/mod_devicetable.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>

#include <video/mipi_display.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct samsung_oneplus_dsc {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct gpio_desc *reset_gpio;
	struct regulator *vddio;
};

static inline
struct samsung_oneplus_dsc *to_samsung_oneplus_dsc(struct drm_panel *panel)
{
	return container_of_const(panel, struct samsung_oneplus_dsc, panel);
}

static void samsung_oneplus_dsc_reset(struct samsung_oneplus_dsc *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(8000, 9000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(5000, 6000);
}

static void samsung_oneplus_dsc_send_pps(struct samsung_oneplus_dsc *ctx,
					 struct mipi_dsi_multi_context *dsi_ctx)
{
	struct drm_dsc_picture_parameter_set pps;

	/*
	 * Downstream's on-sequence carries a fixed PPS as a DCS write (0x0a)
	 * whose rate-control values differ from what msm's DSC encoder uses,
	 * which garbles every slice. Send a real PPS packet packed from the
	 * encoder's config instead, as the sm8150 Pixel 4 panel driver does.
	 */
	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);
	mipi_dsi_picture_parameter_set_multi(dsi_ctx, &pps);
	mipi_dsi_compression_mode_multi(dsi_ctx, true);
}

static int samsung_oneplus_dsc_on(struct samsung_oneplus_dsc *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_compression_mode_multi(&dsi_ctx, true);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc3, 0x00);
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_usleep_range(&dsi_ctx, 10000, 11000);
	mipi_dsi_dcs_set_tear_on_multi(&dsi_ctx, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	mipi_dsi_dcs_set_column_address_multi(&dsi_ctx, 0x0000, 0x059f);
	mipi_dsi_dcs_set_page_address_multi(&dsi_ctx, 0x0000, 0x0c2f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb9,
				     0x01, 0xc0, 0x3c, 0x0b, 0x00, 0x00, 0x00,
				     0x11, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x45);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb5, 0x48);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x1e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5,
				     0x09, 0x10, 0xb4, 0x24, 0xfb);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x37);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc5,
				     0x04, 0xff, 0x00, 0x01, 0x64);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x86);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xeb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0xa5, 0xa5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x5a, 0x5a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb1, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb5, 0xd3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     0x30);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0xa5, 0xa5);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

	return dsi_ctx.accum_err;
}

static int samsung_oneplus_dsc_off(struct samsung_oneplus_dsc *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_usleep_range(&dsi_ctx, 10000, 11000);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

static int samsung_oneplus_dsc_prepare(struct drm_panel *panel)
{
	struct samsung_oneplus_dsc *ctx = to_samsung_oneplus_dsc(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = regulator_enable(ctx->vddio);
	if (ret < 0) {
		dev_err(dev, "Failed to enable vddio: %d\n", ret);
		return ret;
	}
	/* downstream qcom,supply-post-on-sleep = <20> */
	msleep(20);

	samsung_oneplus_dsc_reset(ctx);

	ret = samsung_oneplus_dsc_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		regulator_disable(ctx->vddio);
		return ret;
	}

	{
		struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

		samsung_oneplus_dsc_send_pps(ctx, &dsi_ctx);
		if (dsi_ctx.accum_err) {
			dev_err(dev, "Failed to send PPS: %d\n", dsi_ctx.accum_err);
			return dsi_ctx.accum_err;
		}
	}

	return 0;
}

static int samsung_oneplus_dsc_unprepare(struct drm_panel *panel)
{
	struct samsung_oneplus_dsc *ctx = to_samsung_oneplus_dsc(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	ret = samsung_oneplus_dsc_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_disable(ctx->vddio);

	return 0;
}

static const struct drm_display_mode samsung_oneplus_dsc_mode = {
	.clock = (1440 + 16 + 8 + 8) * (3120 + 4 + 4 + 8) * 90 / 1000,
	.hdisplay = 1440,
	.hsync_start = 1440 + 16,
	.hsync_end = 1440 + 16 + 8,
	.htotal = 1440 + 16 + 8 + 8,
	.vdisplay = 3120,
	.vsync_start = 3120 + 4,
	.vsync_end = 3120 + 4 + 4,
	.vtotal = 3120 + 4 + 4 + 8,
	.width_mm = 71,
	.height_mm = 154,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int samsung_oneplus_dsc_get_modes(struct drm_panel *panel,
					 struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &samsung_oneplus_dsc_mode);
}

static const struct drm_panel_funcs samsung_oneplus_dsc_panel_funcs = {
	.prepare = samsung_oneplus_dsc_prepare,
	.unprepare = samsung_oneplus_dsc_unprepare,
	.get_modes = samsung_oneplus_dsc_get_modes,
};

static int samsung_oneplus_dsc_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

// TODO: Check if /sys/class/backlight/.../actual_brightness actually returns
// correct values. If not, remove this function.
static int samsung_oneplus_dsc_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness_large(dsi, &brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return brightness;
}

static const struct backlight_ops samsung_oneplus_dsc_bl_ops = {
	.update_status = samsung_oneplus_dsc_bl_update_status,
	.get_brightness = samsung_oneplus_dsc_bl_get_brightness,
};

static struct backlight_device *
samsung_oneplus_dsc_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 1023,
		.max_brightness = 1023,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &samsung_oneplus_dsc_bl_ops, &props);
}

static int samsung_oneplus_dsc_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct samsung_oneplus_dsc *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct samsung_oneplus_dsc, panel,
				   &samsung_oneplus_dsc_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->vddio = devm_regulator_get(dev, "vddio");
	if (IS_ERR(ctx->vddio))
		return dev_err_probe(dev, PTR_ERR(ctx->vddio),
				     "Failed to get vddio regulator\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	/* Command mode: no MIPI_DSI_MODE_VIDEO. */
	/*
	 * Downstream sets qcom,mdss-dsc-slice-per-pkt = <2>, both slices of a
	 * line in one packet.
	 */
	dsi->mode_flags = MIPI_DSI_MODE_NO_EOT_PACKET |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS |
			  MIPI_DSI_MODE_DSC_ALL_SLICES_IN_PKT;

	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = samsung_oneplus_dsc_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "Failed to create backlight\n");

	drm_panel_add(&ctx->panel);

	/* This panel only supports DSC; unconditionally enable it */
	dsi->dsc = &ctx->dsc;

	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;

	ctx->dsc.slice_height = 65;
	ctx->dsc.slice_width = 720;
	/*
	 * TODO: hdisplay should be read from the selected mode once
	 * it is passed back to drm_panel (in prepare?)
	 */
	WARN_ON(1440 % ctx->dsc.slice_width);
	ctx->dsc.slice_count = 1440 / ctx->dsc.slice_width;
	ctx->dsc.bits_per_component = 8;
	ctx->dsc.bits_per_pixel = 8 << 4; /* 4 fractional bits */
	ctx->dsc.block_pred_enable = true;

	ret = devm_mipi_dsi_attach(dev, dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void samsung_oneplus_dsc_remove(struct mipi_dsi_device *dsi)
{
	struct samsung_oneplus_dsc *ctx = mipi_dsi_get_drvdata(dsi);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id samsung_oneplus_dsc_of_match[] = {
	{ .compatible = "samsung,oneplus-dsc" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, samsung_oneplus_dsc_of_match);

static struct mipi_dsi_driver samsung_oneplus_dsc_driver = {
	.probe = samsung_oneplus_dsc_probe,
	.remove = samsung_oneplus_dsc_remove,
	.driver = {
		.name = "panel-samsung-oneplus-dsc",
		.of_match_table = samsung_oneplus_dsc_of_match,
	},
};
module_mipi_dsi_driver(samsung_oneplus_dsc_driver);

MODULE_AUTHOR("johannSo <jzeug@soppa.de>");
MODULE_DESCRIPTION("DRM driver for samsung dsc cmd mode oneplus dsi panel");
MODULE_LICENSE("GPL");
