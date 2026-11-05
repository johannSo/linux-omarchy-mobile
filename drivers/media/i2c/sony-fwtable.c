// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sony IMX586 / IMX481 image sensors, with their register tables loaded as
 * firmware.
 *
 * No register settings for these sensors are published under a licence a
 * kernel driver can carry, so this driver ships none. It programs the parts
 * of the register map that Sony's sensors of this generation share (chip ID
 * 0x0016, mode select 0x0100, orientation 0x0101, group hold 0x0104,
 * integration time 0x0202, analogue gain 0x0204, frame length 0x0340, line
 * length 0x0342) and takes everything else -- the global initialisation, and
 * each mode's PLL, readout and timing settings -- from a firmware file that
 * the device's owner generates from their own copy of the phone's vendor
 * software. The file is named by the device tree's "firmware-name".
 *
 * File format (version 1, all fields little-endian):
 *
 *   header (64 bytes)
 *     char   magic[4]       "OMST"
 *     u16    version        1
 *     u16    chip_id        the value of register 0x0016
 *     u32    n_modes        1..16
 *     u32    n_seqs         1..16
 *     u32    init_seq       sequence written after power-on
 *     u32    start_seq      sequence written before streaming, or ~0
 *     u32    stop_seq       sequence written after standby, or ~0
 *     u32    mclk_hz        the input clock the PLL settings expect
 *     u32    lanes          CSI-2 data lanes (D-PHY) or trios (C-PHY)
 *     u32    exposure_min   lines
 *     u32    exposure_margin  lines between frame length and max exposure
 *     u32    again_max      largest analogue gain code (gain 1024/(1024-code))
 *     u32    reserved[4]
 *   n_modes x mode (64 bytes)
 *     u32    width, height  output size
 *     u32    crop_left, crop_top, crop_width, crop_height
 *                           the analogue crop on the pixel array
 *     u32    line_length    pixel clocks per line (register 0x0342)
 *     u32    frame_length   default lines per frame (register 0x0340)
 *     u64    pixel_rate     pixel clocks per second
 *     u64    link_freq      CSI-2 link frequency, Hz, as V4L2 defines it
 *                           for the endpoint's bus (D-PHY or C-PHY)
 *     u32    mbus_code      MEDIA_BUS_FMT_S*10_1X10 without flips
 *     u32    seq            its register sequence
 *     u32    reserved[2]
 *   n_seqs x sequence
 *     u32    count          0..8192
 *     count x { u16 addr; u8 val; u8 flags (0); u32 delay_us (<= 100000) }
 *
 * The tables may not touch mode select (the driver starts and stops the
 * stream itself) or the OTP control and buffer registers (0x0a00-0x0aff),
 * whose write command programs the sensor's one-time-programmable memory.
 * A file that does either is refused before anything is written.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define SFT_REG_CHIP_ID		CCI_REG16(0x0016)
#define SFT_REG_MODE_SELECT	CCI_REG8(0x0100)
#define SFT_MODE_STANDBY	0x00
#define SFT_MODE_STREAMING	0x01
#define SFT_REG_ORIENTATION	CCI_REG8(0x0101)
#define SFT_REG_HOLD		CCI_REG8(0x0104)
#define SFT_REG_EXPOSURE	CCI_REG16(0x0202)
#define SFT_REG_AGAIN		CCI_REG16(0x0204)
#define SFT_REG_FLL		CCI_REG16(0x0340)

#define SFT_FLL_MAX		0xffff
#define SFT_AGAIN_DEN		1024

#define SFT_OTP_FIRST		0x0a00
#define SFT_OTP_LAST		0x0aff

#define SFT_MAX_MODES		16
#define SFT_MAX_SEQS		16
#define SFT_MAX_REGS		8192
#define SFT_MAX_DELAY_US	100000
#define SFT_NO_SEQ		0xffffffff

#define SFT_HDR_SIZE		64
#define SFT_MODE_SIZE		64
#define SFT_REG_SIZE		8

struct sft_chip {
	const char *name;
	const char *model;	/* the subdevice's name, which userspace keys on */
	u16 chip_id;
	u32 native_width;
	u32 native_height;
};

struct sft_reg {
	u16 addr;
	u8 val;
	u32 delay_us;
};

struct sft_seq {
	u32 count;
	struct sft_reg *regs;
};

struct sft_mode {
	u32 width;
	u32 height;
	struct v4l2_rect crop;
	u32 line_length;
	u32 frame_length;
	u64 pixel_rate;
	u64 link_freq;
	u32 mbus_code;
	u32 seq;
	unsigned int link_freq_index;
};

/*
 * In the order the vendor's power-up sequences use: analogue, digital, then
 * the interface. vana2 and vdig2 are the second switch some modules have on
 * each (dummies where a board leaves them out).
 */
static const char * const sft_supply_names[] = {
	"vana",
	"vana2",
	"vdig",
	"vdig2",
	"vif",
};

struct sft {
	struct device *dev;
	const struct sft_chip *chip;
	struct regmap *regmap;
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct clk *mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(sft_supply_names)];

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;

	/* From the firmware file. */
	u16 fw_chip_id;
	u32 mclk_hz;
	u32 lanes;
	u32 exposure_min;
	u32 exposure_margin;
	u32 again_max;
	u32 init_seq;
	u32 start_seq;
	u32 stop_seq;
	unsigned int n_seqs;
	struct sft_seq seqs[SFT_MAX_SEQS];

	/* Modes whose link frequency the board allows. */
	unsigned int n_modes;
	struct sft_mode modes[SFT_MAX_MODES];
	s64 link_freqs[SFT_MAX_MODES];
	unsigned int n_link_freqs;
};

static inline struct sft *to_sft(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sft, sd);
}

/* --- Firmware --- */

static bool sft_bayer_code_ok(u32 code)
{
	return code == MEDIA_BUS_FMT_SRGGB10_1X10 ||
	       code == MEDIA_BUS_FMT_SGRBG10_1X10 ||
	       code == MEDIA_BUS_FMT_SGBRG10_1X10 ||
	       code == MEDIA_BUS_FMT_SBGGR10_1X10;
}

static int sft_parse_seq(struct sft *sensor, const u8 *data, size_t size,
			 size_t *pos, struct sft_seq *seq)
{
	u32 i, count;

	if (*pos + 4 > size)
		return -EINVAL;
	count = get_unaligned_le32(data + *pos);
	*pos += 4;
	if (count > SFT_MAX_REGS || *pos + (size_t)count * SFT_REG_SIZE > size)
		return -EINVAL;

	seq->count = count;
	seq->regs = devm_kcalloc(sensor->dev, max(count, 1U),
				 sizeof(*seq->regs), GFP_KERNEL);
	if (!seq->regs)
		return -ENOMEM;

	for (i = 0; i < count; i++, *pos += SFT_REG_SIZE) {
		const u8 *r = data + *pos;
		struct sft_reg *reg = &seq->regs[i];

		reg->addr = get_unaligned_le16(r);
		reg->val = r[2];
		reg->delay_us = get_unaligned_le32(r + 4);

		if (r[3] != 0 || reg->delay_us > SFT_MAX_DELAY_US) {
			dev_err(sensor->dev, "bad entry %u at 0x%04x\n", i,
				reg->addr);
			return -EINVAL;
		}
		if (reg->addr == 0x0100 ||
		    (reg->addr >= SFT_OTP_FIRST && reg->addr <= SFT_OTP_LAST)) {
			dev_err(sensor->dev,
				"refusing a table that writes 0x%04x\n",
				reg->addr);
			return -EPERM;
		}
	}

	return 0;
}

static int sft_parse_firmware(struct sft *sensor, const struct firmware *fw)
{
	const u8 *d = fw->data;
	size_t size = fw->size, pos;
	struct sft_mode *all;
	u32 n_modes, n_seqs, i;
	int ret;

	if (size < SFT_HDR_SIZE || memcmp(d, "OMST", 4) ||
	    get_unaligned_le16(d + 4) != 1) {
		dev_err(sensor->dev, "not a version 1 table file\n");
		return -EINVAL;
	}

	sensor->fw_chip_id = get_unaligned_le16(d + 6);
	n_modes = get_unaligned_le32(d + 8);
	n_seqs = get_unaligned_le32(d + 12);
	sensor->init_seq = get_unaligned_le32(d + 16);
	sensor->start_seq = get_unaligned_le32(d + 20);
	sensor->stop_seq = get_unaligned_le32(d + 24);
	sensor->mclk_hz = get_unaligned_le32(d + 28);
	sensor->lanes = get_unaligned_le32(d + 32);
	sensor->exposure_min = get_unaligned_le32(d + 36);
	sensor->exposure_margin = get_unaligned_le32(d + 40);
	sensor->again_max = get_unaligned_le32(d + 44);

	if (sensor->fw_chip_id != sensor->chip->chip_id) {
		dev_err(sensor->dev, "tables are for chip 0x%04x, not 0x%04x\n",
			sensor->fw_chip_id, sensor->chip->chip_id);
		return -EINVAL;
	}
	if (!n_modes || n_modes > SFT_MAX_MODES || !n_seqs ||
	    n_seqs > SFT_MAX_SEQS || sensor->init_seq >= n_seqs ||
	    (sensor->start_seq != SFT_NO_SEQ && sensor->start_seq >= n_seqs) ||
	    (sensor->stop_seq != SFT_NO_SEQ && sensor->stop_seq >= n_seqs) ||
	    !sensor->exposure_min || sensor->again_max >= SFT_AGAIN_DEN ||
	    size < SFT_HDR_SIZE + (size_t)n_modes * SFT_MODE_SIZE)
		return -EINVAL;

	all = devm_kcalloc(sensor->dev, n_modes, sizeof(*all), GFP_KERNEL);
	if (!all)
		return -ENOMEM;

	pos = SFT_HDR_SIZE;
	for (i = 0; i < n_modes; i++, pos += SFT_MODE_SIZE) {
		const u8 *m = d + pos;
		struct sft_mode *mode = &all[i];

		mode->width = get_unaligned_le32(m);
		mode->height = get_unaligned_le32(m + 4);
		mode->crop.left = get_unaligned_le32(m + 8);
		mode->crop.top = get_unaligned_le32(m + 12);
		mode->crop.width = get_unaligned_le32(m + 16);
		mode->crop.height = get_unaligned_le32(m + 20);
		mode->line_length = get_unaligned_le32(m + 24);
		mode->frame_length = get_unaligned_le32(m + 28);
		mode->pixel_rate = get_unaligned_le64(m + 32);
		mode->link_freq = get_unaligned_le64(m + 40);
		mode->mbus_code = get_unaligned_le32(m + 48);
		mode->seq = get_unaligned_le32(m + 52);

		if (!mode->width || !mode->height ||
		    mode->width > sensor->chip->native_width ||
		    mode->height > sensor->chip->native_height ||
		    mode->line_length < mode->width ||
		    mode->frame_length <= mode->height ||
		    mode->frame_length > SFT_FLL_MAX ||
		    !mode->pixel_rate || !mode->link_freq ||
		    !sft_bayer_code_ok(mode->mbus_code) ||
		    mode->seq >= n_seqs) {
			dev_err(sensor->dev, "bad mode %u\n", i);
			return -EINVAL;
		}
	}

	for (i = 0; i < n_seqs; i++) {
		ret = sft_parse_seq(sensor, d, size, &pos, &sensor->seqs[i]);
		if (ret) {
			dev_err(sensor->dev, "bad sequence %u: %d\n", i, ret);
			return ret;
		}
	}
	if (pos != size)
		return -EINVAL;
	sensor->n_seqs = n_seqs;

	/* Keep the modes whose link frequency the board lists, once each. */
	{
		struct v4l2_fwnode_endpoint bus = { .bus_type = V4L2_MBUS_UNKNOWN };
		struct fwnode_handle *ep;
		unsigned int j, k;

		ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(sensor->dev),
						     0, 0, 0);
		if (!ep)
			return -ENXIO;
		ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus);
		fwnode_handle_put(ep);
		if (ret)
			return ret;

		if ((bus.bus_type != V4L2_MBUS_CSI2_DPHY &&
		     bus.bus_type != V4L2_MBUS_CSI2_CPHY) ||
		    bus.bus.mipi_csi2.num_data_lanes != sensor->lanes) {
			dev_err(sensor->dev, "tables drive %u lanes, board has %u\n",
				sensor->lanes, bus.bus.mipi_csi2.num_data_lanes);
			v4l2_fwnode_endpoint_free(&bus);
			return -EINVAL;
		}

		for (i = 0; i < n_modes; i++) {
			for (j = 0; j < bus.nr_of_link_frequencies; j++)
				if (bus.link_frequencies[j] == all[i].link_freq)
					break;
			if (j == bus.nr_of_link_frequencies) {
				dev_info(sensor->dev,
					 "mode %ux%u skipped: link %llu Hz not in the device tree\n",
					 all[i].width, all[i].height,
					 all[i].link_freq);
				continue;
			}

			for (k = 0; k < sensor->n_link_freqs; k++)
				if (sensor->link_freqs[k] == all[i].link_freq)
					break;
			if (k == sensor->n_link_freqs)
				sensor->link_freqs[sensor->n_link_freqs++] =
					all[i].link_freq;

			all[i].link_freq_index = k;
			sensor->modes[sensor->n_modes++] = all[i];
		}
		v4l2_fwnode_endpoint_free(&bus);
	}

	if (!sensor->n_modes) {
		dev_err(sensor->dev, "no mode matches the device tree's link frequencies\n");
		return -EINVAL;
	}

	return 0;
}

static int sft_write_seq(struct sft *sensor, u32 index)
{
	const struct sft_seq *seq;
	u32 i;
	int ret = 0;

	if (index == SFT_NO_SEQ)
		return 0;

	seq = &sensor->seqs[index];
	for (i = 0; i < seq->count && !ret; i++) {
		cci_write(sensor->regmap, CCI_REG8(seq->regs[i].addr),
			  seq->regs[i].val, &ret);
		if (seq->regs[i].delay_us)
			fsleep(seq->regs[i].delay_us);
	}

	return ret;
}

/* --- Formats and controls --- */

static u32 sft_flipped_code(struct sft *sensor, u32 code)
{
	/* SRGGB, SGRBG, SGBRG, SBGGR: bit 0 swaps columns, bit 1 rows. */
	static const u32 order[] = {
		MEDIA_BUS_FMT_SRGGB10_1X10,
		MEDIA_BUS_FMT_SGRBG10_1X10,
		MEDIA_BUS_FMT_SGBRG10_1X10,
		MEDIA_BUS_FMT_SBGGR10_1X10,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(order); i++)
		if (order[i] == code)
			break;

	if (sensor->hflip && sensor->hflip->val)
		i ^= 1;
	if (sensor->vflip && sensor->vflip->val)
		i ^= 2;

	return order[i & 3];
}

static const struct sft_mode *sft_find_mode(struct sft *sensor, u32 width,
					    u32 height)
{
	return v4l2_find_nearest_size(sensor->modes, sensor->n_modes, width,
				      height, width, height);
}

static int sft_update_mode_ctrls(struct sft *sensor,
				 const struct sft_mode *mode)
{
	s64 hblank = mode->line_length - mode->width;
	int ret;

	ret = __v4l2_ctrl_s_ctrl(sensor->link_freq, mode->link_freq_index);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_s_ctrl_int64(sensor->pixel_rate, mode->pixel_rate);
	if (ret)
		return ret;

	ret = __v4l2_ctrl_modify_range(sensor->hblank, hblank, hblank, 1,
				       hblank);
	if (ret)
		return ret;

	return __v4l2_ctrl_modify_range(sensor->vblank,
					mode->frame_length - mode->height,
					SFT_FLL_MAX - mode->height, 1,
					mode->frame_length - mode->height);
}

static int sft_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sft *sensor = container_of(ctrl->handler, struct sft, ctrls);
	struct v4l2_subdev_state *state =
		v4l2_subdev_get_locked_active_state(&sensor->sd);
	const struct v4l2_mbus_framefmt *fmt =
		v4l2_subdev_state_get_format(state, 0);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		s64 max = fmt->height + ctrl->val - sensor->exposure_margin;

		ret = __v4l2_ctrl_modify_range(sensor->exposure,
					       sensor->exposure->minimum, max,
					       1, min_t(s64, max,
							sensor->exposure->val));
		if (ret)
			return ret;
	}

	switch (ctrl->id) {
	case V4L2_CID_LINK_FREQ:
	case V4L2_CID_PIXEL_RATE:
	case V4L2_CID_HBLANK:
		/* Follow the mode; nothing to write. */
		return 0;
	}

	if (!pm_runtime_get_if_in_use(sensor->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(sensor->regmap, SFT_REG_AGAIN, ctrl->val, &ret);
		break;
	case V4L2_CID_EXPOSURE:
		cci_write(sensor->regmap, SFT_REG_EXPOSURE, ctrl->val, &ret);
		break;
	case V4L2_CID_VBLANK:
		cci_write(sensor->regmap, SFT_REG_FLL, fmt->height + ctrl->val,
			  &ret);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		cci_write(sensor->regmap, SFT_REG_ORIENTATION,
			  sensor->hflip->val | sensor->vflip->val << 1, &ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(sensor->dev);

	return ret;
}

static const struct v4l2_ctrl_ops sft_ctrl_ops = {
	.s_ctrl = sft_set_ctrl,
};

static int sft_init_controls(struct sft *sensor)
{
	const struct sft_mode *mode = &sensor->modes[0];
	struct v4l2_ctrl_handler *hdl = &sensor->ctrls;
	struct v4l2_fwnode_device_properties props;
	u64 rate_min = U64_MAX, rate_max = 0;
	s64 hblank = mode->line_length - mode->width;
	unsigned int i;
	int ret;

	for (i = 0; i < sensor->n_modes; i++) {
		rate_min = min(rate_min, sensor->modes[i].pixel_rate);
		rate_max = max(rate_max, sensor->modes[i].pixel_rate);
	}

	ret = v4l2_fwnode_device_parse(sensor->dev, &props);
	if (ret)
		return ret;

	v4l2_ctrl_handler_init(hdl, 10);

	sensor->link_freq = v4l2_ctrl_new_int_menu(hdl, &sft_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
						   sensor->n_link_freqs - 1,
						   mode->link_freq_index,
						   sensor->link_freqs);
	sensor->pixel_rate = v4l2_ctrl_new_std(hdl, &sft_ctrl_ops,
					       V4L2_CID_PIXEL_RATE, rate_min,
					       rate_max, 1, mode->pixel_rate);
	sensor->hblank = v4l2_ctrl_new_std(hdl, &sft_ctrl_ops, V4L2_CID_HBLANK,
					   hblank, hblank, 1, hblank);
	sensor->vblank = v4l2_ctrl_new_std(hdl, &sft_ctrl_ops, V4L2_CID_VBLANK,
					   mode->frame_length - mode->height,
					   SFT_FLL_MAX - mode->height, 1,
					   mode->frame_length - mode->height);
	sensor->exposure = v4l2_ctrl_new_std(hdl, &sft_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     sensor->exposure_min,
					     mode->frame_length -
					     sensor->exposure_margin, 1,
					     mode->frame_length -
					     sensor->exposure_margin);
	v4l2_ctrl_new_std(hdl, &sft_ctrl_ops, V4L2_CID_ANALOGUE_GAIN, 0,
			  sensor->again_max, 1, 0);
	sensor->hflip = v4l2_ctrl_new_std(hdl, &sft_ctrl_ops, V4L2_CID_HFLIP,
					  0, 1, 1, 0);
	sensor->vflip = v4l2_ctrl_new_std(hdl, &sft_ctrl_ops, V4L2_CID_VFLIP,
					  0, 1, 1, 0);
	v4l2_ctrl_new_fwnode_properties(hdl, &sft_ctrl_ops, &props);

	if (hdl->error) {
		ret = hdl->error;
		v4l2_ctrl_handler_free(hdl);
		return ret;
	}

	sensor->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	sensor->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	sensor->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	sensor->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	sensor->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	sensor->sd.ctrl_handler = hdl;

	return 0;
}

/* --- Pad operations --- */

static int sft_enum_mbus_code(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_mbus_code_enum *code)
{
	struct sft *sensor = to_sft(sd);

	if (code->index)
		return -EINVAL;

	code->code = sft_flipped_code(sensor, sensor->modes[0].mbus_code);

	return 0;
}

static int sft_enum_frame_size(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	struct sft *sensor = to_sft(sd);
	const struct sft_mode *mode;

	if (fse->index >= sensor->n_modes)
		return -EINVAL;

	mode = &sensor->modes[fse->index];
	if (fse->code != sft_flipped_code(sensor, mode->mbus_code))
		return -EINVAL;

	fse->min_width = mode->width;
	fse->max_width = mode->width;
	fse->min_height = mode->height;
	fse->max_height = mode->height;

	return 0;
}

static void sft_fill_format(struct sft *sensor, const struct sft_mode *mode,
			    struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = sft_flipped_code(sensor, mode->mbus_code);
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int sft_set_pad_format(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_format *fmt)
{
	struct sft *sensor = to_sft(sd);
	const struct sft_mode *mode;

	mode = sft_find_mode(sensor, fmt->format.width, fmt->format.height);
	sft_fill_format(sensor, mode, &fmt->format);

	*v4l2_subdev_state_get_format(state, 0) = fmt->format;
	*v4l2_subdev_state_get_crop(state, 0) = mode->crop;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	if (media_entity_is_streaming(&sd->entity))
		return -EBUSY;

	return sft_update_mode_ctrls(sensor, mode);
}

static int sft_get_selection(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_selection *sel)
{
	struct sft *sensor = to_sft(sd);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(state, 0);
		return 0;
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = sensor->chip->native_width;
		sel->r.height = sensor->chip->native_height;
		return 0;
	}

	return -EINVAL;
}

static int sft_init_state(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state)
{
	struct sft *sensor = to_sft(sd);
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.format.width = sensor->modes[0].width,
		.format.height = sensor->modes[0].height,
	};

	return sft_set_pad_format(sd, state, &fmt);
}

static int sft_enable_streams(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state, u32 pad,
			      u64 streams_mask)
{
	struct sft *sensor = to_sft(sd);
	const struct v4l2_mbus_framefmt *fmt =
		v4l2_subdev_state_get_format(state, 0);
	const struct sft_mode *mode = sft_find_mode(sensor, fmt->width,
						    fmt->height);
	int ret;

	ret = pm_runtime_resume_and_get(sensor->dev);
	if (ret)
		return ret;

	ret = sft_write_seq(sensor, sensor->init_seq);
	if (!ret)
		ret = sft_write_seq(sensor, mode->seq);
	if (ret) {
		dev_err(sensor->dev, "writing the tables failed: %d\n", ret);
		goto err_put;
	}

	ret = __v4l2_ctrl_handler_setup(&sensor->ctrls);
	if (ret)
		goto err_put;

	ret = sft_write_seq(sensor, sensor->start_seq);
	cci_write(sensor->regmap, SFT_REG_MODE_SELECT, SFT_MODE_STREAMING,
		  &ret);
	if (ret)
		goto err_put;

	__v4l2_ctrl_grab(sensor->hflip, true);
	__v4l2_ctrl_grab(sensor->vflip, true);

	return 0;

err_put:
	pm_runtime_put(sensor->dev);
	return ret;
}

static int sft_disable_streams(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state, u32 pad,
			       u64 streams_mask)
{
	struct sft *sensor = to_sft(sd);
	int ret = 0;

	cci_write(sensor->regmap, SFT_REG_MODE_SELECT, SFT_MODE_STANDBY, &ret);
	if (!ret)
		ret = sft_write_seq(sensor, sensor->stop_seq);
	if (ret)
		dev_err(sensor->dev, "stopping the stream failed: %d\n", ret);

	__v4l2_ctrl_grab(sensor->hflip, false);
	__v4l2_ctrl_grab(sensor->vflip, false);

	pm_runtime_put(sensor->dev);

	return 0;
}

static const struct v4l2_subdev_video_ops sft_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops sft_pad_ops = {
	.enum_mbus_code = sft_enum_mbus_code,
	.enum_frame_size = sft_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = sft_set_pad_format,
	.get_selection = sft_get_selection,
	.enable_streams = sft_enable_streams,
	.disable_streams = sft_disable_streams,
};

static const struct v4l2_subdev_ops sft_subdev_ops = {
	.video = &sft_video_ops,
	.pad = &sft_pad_ops,
};

static const struct v4l2_subdev_internal_ops sft_internal_ops = {
	.init_state = sft_init_state,
};

/* --- Power --- */

static int sft_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct sft *sensor = to_sft(sd);
	unsigned int i;
	int ret;

	/* XCLR stays low until every rail and the clock are up. */
	gpiod_set_value_cansleep(sensor->reset_gpio, 1);

	for (i = 0; i < ARRAY_SIZE(sensor->supplies); i++) {
		ret = regulator_enable(sensor->supplies[i].consumer);
		if (ret) {
			dev_err(dev, "enabling %s failed: %d\n",
				sft_supply_names[i], ret);
			goto err_rails;
		}
		usleep_range(1000, 1500);
	}

	ret = clk_prepare_enable(sensor->mclk);
	if (ret)
		goto err_rails;
	usleep_range(1000, 1500);

	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	/* Sony's sensors take up to 10 ms from XCLR to the first access. */
	usleep_range(12000, 13000);

	return 0;

err_rails:
	while (i--)
		regulator_disable(sensor->supplies[i].consumer);
	return ret;
}

static int sft_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct sft *sensor = to_sft(sd);
	unsigned int i = ARRAY_SIZE(sensor->supplies);

	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	usleep_range(1000, 1500);
	clk_disable_unprepare(sensor->mclk);

	while (i--)
		regulator_disable(sensor->supplies[i].consumer);

	return 0;
}

static DEFINE_RUNTIME_DEV_PM_OPS(sft_pm_ops, sft_power_off, sft_power_on,
				 NULL);

/* --- Probe --- */

static int sft_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	const struct firmware *fw;
	const char *fw_name;
	struct sft *sensor;
	unsigned int i;
	u64 id;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->dev = dev;
	sensor->chip = device_get_match_data(dev);

	ret = device_property_read_string(dev, "firmware-name", &fw_name);
	if (ret)
		return dev_err_probe(dev, ret, "no firmware-name\n");

	ret = request_firmware(&fw, fw_name, dev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "no register tables (%s): generate them from the phone's vendor image\n",
				     fw_name);
	ret = sft_parse_firmware(sensor, fw);
	release_firmware(fw);
	if (ret)
		return dev_err_probe(dev, ret, "refusing %s\n", fw_name);

	sensor->mclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(sensor->mclk))
		return dev_err_probe(dev, PTR_ERR(sensor->mclk), "no clock\n");
	if (clk_get_rate(sensor->mclk) != sensor->mclk_hz)
		return dev_err_probe(dev, -EINVAL,
				     "clock is %lu Hz, the tables expect %u\n",
				     clk_get_rate(sensor->mclk),
				     sensor->mclk_hz);

	for (i = 0; i < ARRAY_SIZE(sft_supply_names); i++)
		sensor->supplies[i].supply = sft_supply_names[i];
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(sensor->supplies),
				      sensor->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "no supplies\n");

	sensor->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(sensor->reset_gpio),
				     "no reset GPIO\n");

	v4l2_i2c_subdev_init(&sensor->sd, client, &sft_subdev_ops);
	v4l2_i2c_subdev_set_name(&sensor->sd, client, sensor->chip->model, NULL);
	sensor->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(sensor->regmap))
		return PTR_ERR(sensor->regmap);

	ret = sft_power_on(dev);
	if (ret)
		return ret;

	ret = cci_read(sensor->regmap, SFT_REG_CHIP_ID, &id, NULL);
	if (ret || id != sensor->chip->chip_id) {
		ret = dev_err_probe(dev, ret ?: -ENODEV,
				    "no %s here (ID 0x%04llx)\n",
				    sensor->chip->name, ret ? 0 : id);
		goto err_power_off;
	}

	ret = sft_init_controls(sensor);
	if (ret)
		goto err_power_off;

	sensor->sd.internal_ops = &sft_internal_ops;
	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		goto err_ctrls;

	sensor->sd.state_lock = sensor->ctrls.lock;
	ret = v4l2_subdev_init_finalize(&sensor->sd);
	if (ret)
		goto err_entity;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret)
		goto err_pm;

	pm_runtime_idle(dev);
	dev_info(dev, "%s, %u modes from the tables\n", sensor->chip->name,
		 sensor->n_modes);

	return 0;

err_pm:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	v4l2_subdev_cleanup(&sensor->sd);
err_entity:
	media_entity_cleanup(&sensor->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls);
err_power_off:
	sft_power_off(dev);
	return ret;
}

static void sft_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev)) {
		sft_power_off(&client->dev);
		pm_runtime_set_suspended(&client->dev);
	}
}

static const struct sft_chip sft_imx481 = {
	.name = "IMX481",
	.model = "imx481",
	.chip_id = 0x0481,
	.native_width = 4656,
	.native_height = 3496,
};

static const struct sft_chip sft_imx586 = {
	.name = "IMX586",
	.model = "imx586",
	.chip_id = 0x0586,
	.native_width = 8000,
	.native_height = 6000,
};

static const struct of_device_id sft_of_match[] = {
	{ .compatible = "sony,imx481", .data = &sft_imx481 },
	{ .compatible = "sony,imx586", .data = &sft_imx586 },
	{ }
};
MODULE_DEVICE_TABLE(of, sft_of_match);

static struct i2c_driver sft_driver = {
	.driver = {
		.name = "sony-fwtable",
		.of_match_table = sft_of_match,
		.pm = pm_ptr(&sft_pm_ops),
	},
	.probe = sft_probe,
	.remove = sft_remove,
};
module_i2c_driver(sft_driver);

MODULE_DESCRIPTION("Sony IMX481/IMX586 sensors with register tables from firmware");
MODULE_LICENSE("GPL");
