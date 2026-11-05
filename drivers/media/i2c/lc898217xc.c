// SPDX-License-Identifier: GPL-2.0-only
/*
 * ON Semiconductor LC898217XC lens driver
 *
 * The LC898217XC drives a camera module's focus voice coil. It has 8-bit
 * register addresses; this driver uses two registers, as the stock camera
 * software of the OnePlus 7T Pro does, and writes only the first:
 *   0x84 target position, 16 bits big-endian, 10 bits used
 *   0xb3 status, polled until it reads 0 after power-up
 *
 * The driver powers the actuator while its subdev is open (libcamera keeps
 * it open while the camera is in use) and moves the lens in steps on
 * power-up and back to 0 before power-off, so it doesn't hit its stop.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>

#define LC898217XC_REG_TARGET		CCI_REG16(0x84)
#define LC898217XC_REG_STATUS		CCI_REG8(0xb3)

#define LC898217XC_POS_MAX		1023
/* stock waits 5 ms after the supply, then polls the status for 10 ms */
#define LC898217XC_POWER_DELAY_US	5000
#define LC898217XC_READY_TIMEOUT_US	20000
#define LC898217XC_MOVE_STEP		64
#define LC898217XC_MOVE_DELAY_US	1000

static const char * const lc898217xc_supply_names[] = {
	"vio",
	"vdd",
};

#define LC898217XC_NUM_SUPPLIES	ARRAY_SIZE(lc898217xc_supply_names)

struct lc898217xc {
	struct device *dev;
	struct regmap *regmap;
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *focus;
	struct regulator_bulk_data supplies[LC898217XC_NUM_SUPPLIES];
	u32 pos;	/* the last position written */
};

static inline struct lc898217xc *to_lc898217xc(struct v4l2_subdev *sd)
{
	return container_of(sd, struct lc898217xc, sd);
}

static int lc898217xc_write_pos(struct lc898217xc *lens, u32 pos)
{
	int ret;

	ret = cci_write(lens->regmap, LC898217XC_REG_TARGET, pos, NULL);
	if (!ret)
		lens->pos = pos;

	return ret;
}

/* Move in steps of LC898217XC_MOVE_STEP, LC898217XC_MOVE_DELAY_US apart. */
static int lc898217xc_ramp(struct lc898217xc *lens, u32 to)
{
	int ret;

	while (lens->pos != to) {
		u32 next;

		if (lens->pos < to)
			next = min(lens->pos + LC898217XC_MOVE_STEP, to);
		else
			next = lens->pos > to + LC898217XC_MOVE_STEP ?
			       lens->pos - LC898217XC_MOVE_STEP : to;

		ret = lc898217xc_write_pos(lens, next);
		if (ret)
			return ret;
		usleep_range(LC898217XC_MOVE_DELAY_US,
			     LC898217XC_MOVE_DELAY_US + 100);
	}

	return 0;
}

static int lc898217xc_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct lc898217xc *lens = container_of(ctrl->handler,
					       struct lc898217xc, ctrls);
	int ret;

	if (ctrl->id != V4L2_CID_FOCUS_ABSOLUTE)
		return -EINVAL;

	/* Only while powered: power-up moves to the control's value. */
	if (pm_runtime_get_if_in_use(lens->dev) <= 0)
		return 0;

	ret = lc898217xc_write_pos(lens, ctrl->val);
	pm_runtime_put(lens->dev);

	return ret;
}

static const struct v4l2_ctrl_ops lc898217xc_ctrl_ops = {
	.s_ctrl = lc898217xc_set_ctrl,
};

static int lc898217xc_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct lc898217xc *lens = to_lc898217xc(sd);
	u64 status = 0;
	int ret, err = 0;

	ret = regulator_bulk_enable(LC898217XC_NUM_SUPPLIES, lens->supplies);
	if (ret)
		return ret;

	usleep_range(LC898217XC_POWER_DELAY_US, LC898217XC_POWER_DELAY_US + 500);

	ret = read_poll_timeout(cci_read, err, err || !status, 1000,
				LC898217XC_READY_TIMEOUT_US, false,
				lens->regmap, LC898217XC_REG_STATUS, &status,
				NULL);
	if (err) {
		dev_err(dev, "no answer after power-up: %d\n", err);
		ret = err;
		goto err_power_off;
	}
	if (ret)
		dev_warn(dev, "status still 0x%02llx after power-up\n", status);

	lens->pos = 0;
	ret = lc898217xc_ramp(lens, lens->focus->val);
	if (ret)
		goto err_power_off;

	return 0;

err_power_off:
	regulator_bulk_disable(LC898217XC_NUM_SUPPLIES, lens->supplies);
	return ret;
}

static int lc898217xc_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct lc898217xc *lens = to_lc898217xc(sd);
	int ret;

	ret = lc898217xc_ramp(lens, 0);
	if (ret)
		dev_warn(dev, "moving to rest failed: %d\n", ret);

	return regulator_bulk_disable(LC898217XC_NUM_SUPPLIES, lens->supplies);
}

static int lc898217xc_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return pm_runtime_resume_and_get(sd->dev);
}

static int lc898217xc_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pm_runtime_put(sd->dev);

	return 0;
}

static const struct v4l2_subdev_internal_ops lc898217xc_internal_ops = {
	.open = lc898217xc_open,
	.close = lc898217xc_close,
};

static const struct v4l2_subdev_ops lc898217xc_ops = { };

static int lc898217xc_init_controls(struct lc898217xc *lens)
{
	struct v4l2_ctrl_handler *hdl = &lens->ctrls;

	v4l2_ctrl_handler_init(hdl, 1);
	lens->focus = v4l2_ctrl_new_std(hdl, &lc898217xc_ctrl_ops,
					V4L2_CID_FOCUS_ABSOLUTE, 0,
					LC898217XC_POS_MAX, 1, 0);
	if (hdl->error) {
		int ret = hdl->error;

		v4l2_ctrl_handler_free(hdl);
		return ret;
	}
	lens->sd.ctrl_handler = hdl;

	return 0;
}

static int lc898217xc_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct lc898217xc *lens;
	unsigned int i;
	int ret;

	lens = devm_kzalloc(dev, sizeof(*lens), GFP_KERNEL);
	if (!lens)
		return -ENOMEM;

	lens->dev = dev;
	lens->regmap = devm_cci_regmap_init_i2c(client, 8);
	if (IS_ERR(lens->regmap))
		return PTR_ERR(lens->regmap);

	for (i = 0; i < LC898217XC_NUM_SUPPLIES; i++)
		lens->supplies[i].supply = lc898217xc_supply_names[i];
	ret = devm_regulator_bulk_get(dev, LC898217XC_NUM_SUPPLIES,
				      lens->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get supplies\n");

	v4l2_i2c_subdev_init(&lens->sd, client, &lc898217xc_ops);
	lens->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	lens->sd.internal_ops = &lc898217xc_internal_ops;
	lens->sd.entity.function = MEDIA_ENT_F_LENS;

	ret = lc898217xc_init_controls(lens);
	if (ret)
		return ret;

	ret = media_entity_pads_init(&lens->sd.entity, 0, NULL);
	if (ret)
		goto err_ctrls;

	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev(&lens->sd);
	if (ret)
		goto err_pm;

	return 0;

err_pm:
	pm_runtime_disable(dev);
	media_entity_cleanup(&lens->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&lens->ctrls);
	return ret;
}

static void lc898217xc_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct lc898217xc *lens = to_lc898217xc(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&lens->ctrls);
	media_entity_cleanup(&sd->entity);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		lc898217xc_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(lc898217xc_pm_ops, lc898217xc_power_off,
				 lc898217xc_power_on, NULL);

static const struct of_device_id lc898217xc_of_match[] = {
	{ .compatible = "onnn,lc898217xc" },
	{ }
};
MODULE_DEVICE_TABLE(of, lc898217xc_of_match);

static struct i2c_driver lc898217xc_driver = {
	.probe = lc898217xc_probe,
	.remove = lc898217xc_remove,
	.driver = {
		.name = "lc898217xc",
		.of_match_table = lc898217xc_of_match,
		.pm = pm_ptr(&lc898217xc_pm_ops),
	},
};
module_i2c_driver(lc898217xc_driver);

MODULE_DESCRIPTION("ON Semiconductor LC898217XC lens driver");
MODULE_LICENSE("GPL");
