// SPDX-License-Identifier: GPL-2.0-only
/*
 * Awinic AW8695/AW8697 LRA haptics driver
 *
 * Based on the AW8695 driver Luca Weiss posted to linux-input on 2022-04-08
 * (Message-ID 20220408115311.237039-2-luca.weiss@fairphone.com), itself
 * partially based on Awinic's vendor driver, with the review of that series
 * applied. The AW8697 has the AW8695's register map with its own chip ID.
 *
 * Only RAM mode is used: one period of a sine at the actuator's resonant
 * frequency is written to the chip's SRAM at probe and looped while an
 * FF_RUMBLE effect plays, with the rumble magnitude as the chip's data
 * gain. The resonant frequency is measured by the chip at probe (a short,
 * barely felt drive) and checked against the one the device tree gives.
 * The boost converter stays in bypass: the actuator is driven from VBAT.
 *
 * A playback lasts at least AW869X_MIN_PERIODS periods of f0: user space
 * sends ticks of a few milliseconds (feedbackd's button press is 7 ms),
 * which is about one period, too short for the actuator to move.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/fixp-arith.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define AW8695_CHIPID			0x95
#define AW8697_CHIPID			0x97
#define AW869X_SOFT_RESET		0xaa

/* The waveform library starts here after reset (BASE_ADDRH/L). */
#define AW869X_RAM_BASE_ADDR		0x800
/* RAM mode plays samples at 24 kHz. */
#define AW869X_RAM_RATE_HZ		24000
/* Peak of the generated sine, as in the vendor's and Luca's waveforms. */
#define AW869X_SINE_PEAK		84

#define AW869X_ID			0x00

#define AW869X_SYSST			0x01

#define AW869X_SYSINT			0x02
#define AW869X_SYSINT_BSTERRI		BIT(7)
#define AW869X_SYSINT_OVI		BIT(6)
#define AW869X_SYSINT_UVLI		BIT(5)
#define AW869X_SYSINT_OCDI		BIT(2)
#define AW869X_SYSINT_OTI		BIT(1)
#define AW869X_SYSINT_DONEI		BIT(0)

#define AW869X_SYSINTM			0x03
#define AW869X_SYSINTM_BSTERR		BIT(7)
#define AW869X_SYSINTM_OV		BIT(6)
#define AW869X_SYSINTM_UVLO		BIT(5)
#define AW869X_SYSINTM_FF_AE		BIT(4)
#define AW869X_SYSINTM_FF_AF		BIT(3)
#define AW869X_SYSINTM_OCD		BIT(2)
#define AW869X_SYSINTM_OT		BIT(1)
#define AW869X_SYSINTM_DONE		BIT(0)

#define AW869X_SYSCTRL			0x04
#define AW869X_SYSCTRL_RAMINIT		BIT(5)
#define AW869X_SYSCTRL_PLAY_MODE	GENMASK(3, 2)
#define AW869X_SYSCTRL_PLAY_MODE_RAM	0
#define AW869X_SYSCTRL_PLAY_MODE_CONT	2
#define AW869X_SYSCTRL_BST_BOOST	BIT(1)
#define AW869X_SYSCTRL_STANDBY		BIT(0)

#define AW869X_GO			0x05
#define AW869X_GO_ENABLE		BIT(0)

#define AW869X_WAVSEQ1			0x07
#define AW869X_WAVSEQ2			0x08
#define AW869X_WAVLOOP1			0x0f
#define AW869X_WAVLOOP1_SEQ1		GENMASK(7, 4)
#define AW869X_WAVLOOP1_SEQ2		GENMASK(3, 0)
#define AW869X_WAVLOOP_INFINITE		0xf

#define AW869X_DBGCTRL			0x20
#define AW869X_DBGCTRL_INT_MODE_EDGE	BIT(2)

#define AW869X_DATCTRL			0x2b
#define AW869X_DATCTRL_FC		GENMASK(7, 6)
#define AW869X_DATCTRL_FC_1000HZ	3
#define AW869X_DATCTRL_LPF_ENABLE	BIT(5)

#define AW869X_PWMPRC			0x2d
#define AW869X_PWMPRC_PRC_ENABLE	BIT(7)

#define AW869X_PWMDBG			0x2e
#define AW869X_PWMDBG_PWM_MODE		GENMASK(6, 5)
#define AW869X_PWMDBG_PWM_24K		2

#define AW869X_ANADBG			0x35
#define AW869X_ANADBG_IOC		GENMASK(3, 2)
#define AW869X_ANADBG_IOC_4P65A		3

#define AW869X_ANACTRL			0x36
#define AW869X_ANACTRL_LRA_SRC_REG	BIT(5)

/* Data gain: 0x80 is 1.0 */
#define AW869X_DATDBG			0x39
#define AW869X_GAIN_MAX			0x80

#define AW869X_PRLVL			0x3e
#define AW869X_PRLVL_PR_ENABLE		BIT(7)

#define AW869X_RAMADDRH			0x40
#define AW869X_RAMADDRL			0x41
#define AW869X_RAMDATA			0x42

#define AW869X_GLB_STATE		0x46
#define AW869X_GLB_STATE_MASK		GENMASK(3, 0)
#define AW869X_GLB_STATE_STANDBY	0

#define AW869X_BST_AUTO			0x47
#define AW869X_BST_AUTO_AUTOSW		BIT(2)

#define AW869X_CONT_CTRL		0x48
#define AW869X_CONT_CTRL_CLOSE_PLAYBACK	BIT(3)
#define AW869X_CONT_CTRL_F0_DETECT	BIT(2)

#define AW869X_F_PRE_H			0x49
#define AW869X_F_PRE_L			0x4a

#define AW869X_TRIM_LRA			0x5b

#define AW869X_DETCTRL			0x5f
#define AW869X_DETCTRL_PROTECT_NO_ACTION BIT(5)
#define AW869X_DETCTRL_DIAG_GO		BIT(0)

#define AW869X_ADCTEST			0x66
#define AW869X_ADCTEST_VBAT_HW_COMP	BIT(6)

#define AW869X_F_LRA_F0_H		0x68
#define AW869X_F_LRA_F0_L		0x69

#define AW869X_DRV_LVL			0x7b
#define AW869X_NUM_F0_1			0x7d
#define AW869X_NUM_F0_1_PRE		GENMASK(7, 4)
#define AW869X_NUM_F0_1_WAIT		GENMASK(3, 0)
#define AW869X_NUM_F0_2			0x7e
#define AW869X_NUM_F0_3			0x7f

#define AW869X_MAX_REG			0x7f

/*
 * f0 detection, as the vendor driver and Luca's Fairphone 4 set it up: the
 * chip drives the actuator in CONT mode from a preset f0 and traces the
 * real one. The period counter runs at 384 kHz: f0 [0.1 Hz] = 10^9 /
 * (count * 260).
 */
#define AW869X_F0_COEFF			260
#define AW869X_F0_DRV_LVL		125
#define AW869X_F0_DET_PLAY		5
#define AW869X_F0_DET_WAIT		3
#define AW869X_F0_DET_REPEAT		2
#define AW869X_F0_DET_TRACE		15
/* A measured f0 further than this from the device tree's is not used. */
#define AW869X_F0_TOLERANCE_PCT		10

/* Shortest playback, in periods of f0 (about 23 ms at 175 Hz). */
#define AW869X_MIN_PERIODS		4

/* The SRAM header: a version byte, then each waveform's start and end. */
struct aw869x_sram_header {
	u8 version;
	__be16 start_address;
	__be16 end_address;
} __packed;

struct aw869x {
	struct device *dev;
	struct regmap *regmap;
	struct input_dev *input;
	struct gpio_desc *reset_gpio;
	struct delayed_work play_work;
	u16 level;		/* requested by ff-memless */
	bool playing;		/* the chip plays the sine */
	ktime_t started;
	unsigned int f0_dhz;	/* resonant frequency, 0.1 Hz */
	u8 chip_id;
};

static int aw869x_hw_init(struct aw869x *haptics);
static int aw869x_ram_init(struct aw869x *haptics);

static int aw869x_wait_standby(struct aw869x *haptics, unsigned int sleep_us,
			       unsigned int timeout_us)
{
	unsigned int val;
	int error;

	error = regmap_read_poll_timeout(haptics->regmap, AW869X_GLB_STATE, val,
					 FIELD_GET(AW869X_GLB_STATE_MASK, val) ==
						AW869X_GLB_STATE_STANDBY,
					 sleep_us, timeout_us);
	if (error)
		dev_err(haptics->dev, "did not enter standby: %d\n", error);

	return error;
}

static int aw869x_set_standby(struct aw869x *haptics)
{
	int error;

	error = regmap_set_bits(haptics->regmap, AW869X_SYSINTM, AW869X_SYSINTM_UVLO);
	if (error)
		return error;

	return regmap_set_bits(haptics->regmap, AW869X_SYSCTRL, AW869X_SYSCTRL_STANDBY);
}

/* Leave standby in @play_mode with the boost converter bypassed. */
static int aw869x_set_active(struct aw869x *haptics, unsigned int play_mode)
{
	unsigned int val;
	int error;

	error = regmap_update_bits(haptics->regmap, AW869X_SYSCTRL,
				   AW869X_SYSCTRL_PLAY_MODE | AW869X_SYSCTRL_BST_BOOST |
				   AW869X_SYSCTRL_STANDBY,
				   FIELD_PREP(AW869X_SYSCTRL_PLAY_MODE, play_mode));
	if (error)
		return error;

	/* The interrupt register clears on read: drop a stale UVLO. */
	error = regmap_read(haptics->regmap, AW869X_SYSINT, &val);
	if (error)
		return error;

	return regmap_clear_bits(haptics->regmap, AW869X_SYSINTM, AW869X_SYSINTM_UVLO);
}

/*
 * Clear GO and wait for the chip to finish the period it plays. GO is
 * written, not updated: a read-modify-write would skip the write whenever
 * the bit already reads back clear. Should the chip still not stop, it is
 * reset and set up again, so that the actuator is never left running.
 */
static int aw869x_stop(struct aw869x *haptics)
{
	int error;

	haptics->playing = false;

	error = regmap_write(haptics->regmap, AW869X_GO, 0);
	if (!error)
		error = aw869x_wait_standby(haptics, 1000, 100000);
	if (!error)
		return aw869x_set_standby(haptics);

	dev_err(haptics->dev, "playback did not stop (%d), resetting the chip\n", error);
	error = regmap_write(haptics->regmap, AW869X_ID, AW869X_SOFT_RESET);
	if (error)
		return error;
	usleep_range(2000, 2500);

	error = aw869x_hw_init(haptics);
	if (error)
		return error;

	return aw869x_ram_init(haptics);
}

static unsigned int aw869x_gain(u16 level)
{
	return DIV_ROUND_UP(level * AW869X_GAIN_MAX, 0xffff);
}

static int aw869x_play_sine(struct aw869x *haptics, u16 level)
{
	int error;

	/* Waveform 1, looped until stopped; nothing after it. */
	error = regmap_write(haptics->regmap, AW869X_WAVSEQ1, 1);
	if (error)
		return error;

	error = regmap_write(haptics->regmap, AW869X_WAVSEQ2, 0);
	if (error)
		return error;

	error = regmap_write(haptics->regmap, AW869X_WAVLOOP1,
			     FIELD_PREP(AW869X_WAVLOOP1_SEQ1, AW869X_WAVLOOP_INFINITE));
	if (error)
		return error;

	error = regmap_write(haptics->regmap, AW869X_DATDBG, aw869x_gain(level));
	if (error)
		return error;

	error = aw869x_set_active(haptics, AW869X_SYSCTRL_PLAY_MODE_RAM);
	if (error)
		return error;

	error = regmap_write(haptics->regmap, AW869X_GO, AW869X_GO_ENABLE);
	if (error)
		return error;

	haptics->playing = true;
	haptics->started = ktime_get();

	return 0;
}

static void aw869x_play_work(struct work_struct *work)
{
	struct aw869x *haptics = container_of(to_delayed_work(work),
					      struct aw869x, play_work);
	u16 level = READ_ONCE(haptics->level);
	s64 left_us;
	int error;

	if (level && haptics->playing) {
		/* A new magnitude: change the gain, keep playing. */
		error = regmap_write(haptics->regmap, AW869X_DATDBG, aw869x_gain(level));
	} else if (level) {
		error = aw869x_play_sine(haptics, level);
	} else if (haptics->playing) {
		left_us = AW869X_MIN_PERIODS * 10000000LL / haptics->f0_dhz -
			  ktime_us_delta(ktime_get(), haptics->started);
		if (left_us > 0) {
			queue_delayed_work(system_dfl_wq, &haptics->play_work,
					   usecs_to_jiffies(left_us));
			return;
		}
		error = aw869x_stop(haptics);
	} else {
		return;
	}

	if (error)
		dev_err(haptics->dev, "failed to %s playback: %d\n",
			level ? "start" : "stop", error);
}

static int aw869x_play_effect(struct input_dev *input, void *data,
			      struct ff_effect *effect)
{
	struct aw869x *haptics = input_get_drvdata(input);
	u16 level;

	level = effect->u.rumble.strong_magnitude;
	if (!level)
		level = effect->u.rumble.weak_magnitude;

	WRITE_ONCE(haptics->level, level);
	mod_delayed_work(system_dfl_wq, &haptics->play_work, 0);

	return 0;
}

static void aw869x_close(struct input_dev *input)
{
	struct aw869x *haptics = input_get_drvdata(input);
	int error;

	cancel_delayed_work_sync(&haptics->play_work);
	haptics->level = 0;

	error = aw869x_stop(haptics);
	if (error)
		dev_err(haptics->dev, "failed to stop playback: %d\n", error);
}

static void aw869x_hw_reset(struct aw869x *haptics)
{
	if (!haptics->reset_gpio)
		return;

	gpiod_set_value_cansleep(haptics->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(haptics->reset_gpio, 0);
	usleep_range(3500, 4000);
}

/* Offset calibration of the output stage, as the vendor driver runs it. */
static int aw869x_offset_calibration(struct aw869x *haptics)
{
	unsigned int val;
	int error;

	error = regmap_set_bits(haptics->regmap, AW869X_SYSCTRL, AW869X_SYSCTRL_RAMINIT);
	if (error)
		return error;

	error = regmap_set_bits(haptics->regmap, AW869X_DETCTRL, AW869X_DETCTRL_DIAG_GO);
	if (error)
		return error;

	error = regmap_read_poll_timeout(haptics->regmap, AW869X_DETCTRL, val,
					 !(val & AW869X_DETCTRL_DIAG_GO),
					 10000, 500000);
	if (error)
		return error;

	return regmap_clear_bits(haptics->regmap, AW869X_SYSCTRL, AW869X_SYSCTRL_RAMINIT);
}

/* Have the chip trace the actuator's f0; returns it in 0.1 Hz or an error. */
static int aw869x_detect_f0(struct aw869x *haptics)
{
	unsigned int count, trace_ms, val;
	u8 buf[2];
	int error;

	error = aw869x_stop(haptics);
	if (error)
		return error;

	error = regmap_write(haptics->regmap, AW869X_TRIM_LRA, 0);
	if (error)
		return error;

	error = aw869x_set_active(haptics, AW869X_SYSCTRL_PLAY_MODE_CONT);
	if (error)
		return error;

	error = regmap_update_bits(haptics->regmap, AW869X_CONT_CTRL,
				   AW869X_CONT_CTRL_CLOSE_PLAYBACK | AW869X_CONT_CTRL_F0_DETECT,
				   AW869X_CONT_CTRL_F0_DETECT);
	if (error)
		return error;

	error = regmap_update_bits(haptics->regmap, AW869X_DATCTRL,
				   AW869X_DATCTRL_FC | AW869X_DATCTRL_LPF_ENABLE,
				   FIELD_PREP(AW869X_DATCTRL_FC, AW869X_DATCTRL_FC_1000HZ) |
				   AW869X_DATCTRL_LPF_ENABLE);
	if (error)
		return error;

	error = regmap_set_bits(haptics->regmap, AW869X_ANACTRL, AW869X_ANACTRL_LRA_SRC_REG);
	if (error)
		return error;

	count = 1000000000 / (haptics->f0_dhz * AW869X_F0_COEFF);
	buf[0] = count >> 8;
	buf[1] = count & 0xff;
	error = regmap_bulk_write(haptics->regmap, AW869X_F_PRE_H, buf, 2);
	if (error)
		return error;

	error = regmap_write(haptics->regmap, AW869X_DRV_LVL, AW869X_F0_DRV_LVL);
	if (error)
		return error;

	error = regmap_write(haptics->regmap, AW869X_NUM_F0_1,
			     FIELD_PREP(AW869X_NUM_F0_1_PRE, AW869X_F0_DET_PLAY) |
			     FIELD_PREP(AW869X_NUM_F0_1_WAIT, AW869X_F0_DET_WAIT));
	if (error)
		return error;

	error = regmap_write(haptics->regmap, AW869X_NUM_F0_2, AW869X_F0_DET_REPEAT);
	if (error)
		return error;

	error = regmap_write(haptics->regmap, AW869X_NUM_F0_3, AW869X_F0_DET_TRACE);
	if (error)
		return error;

	error = regmap_read(haptics->regmap, AW869X_SYSINT, &val);
	if (error)
		return error;

	error = regmap_set_bits(haptics->regmap, AW869X_GO, AW869X_GO_ENABLE);
	if (error)
		return error;

	/* The trace takes this many periods of the preset f0. */
	trace_ms = 10000 / haptics->f0_dhz *
		   (AW869X_F0_DET_PLAY + AW869X_F0_DET_WAIT +
		    (AW869X_F0_DET_TRACE + AW869X_F0_DET_WAIT) *
		    (AW869X_F0_DET_REPEAT - 1));
	msleep(trace_ms);

	error = aw869x_wait_standby(haptics, 10000, 500000);
	if (error)
		return error;

	error = regmap_bulk_read(haptics->regmap, AW869X_F_LRA_F0_H, buf, 2);
	if (error)
		return error;
	count = buf[0] << 8 | buf[1];

	error = regmap_update_bits(haptics->regmap, AW869X_CONT_CTRL,
				   AW869X_CONT_CTRL_CLOSE_PLAYBACK | AW869X_CONT_CTRL_F0_DETECT,
				   AW869X_CONT_CTRL_CLOSE_PLAYBACK);
	if (error)
		return error;

	error = aw869x_set_standby(haptics);
	if (error)
		return error;

	if (!count)
		return -ENODATA;

	return 1000000000 / (count * AW869X_F0_COEFF);
}

static int aw869x_hw_init(struct aw869x *haptics)
{
	unsigned int val;
	int error;

	/* Clear the power-on UVLO interrupt. */
	error = regmap_read(haptics->regmap, AW869X_SYSINT, &val);
	if (error)
		return error;

	error = regmap_set_bits(haptics->regmap, AW869X_DBGCTRL, AW869X_DBGCTRL_INT_MODE_EDGE);
	if (error)
		return error;

	/* Report only faults; the boost error is moot with the boost bypassed. */
	error = regmap_write(haptics->regmap, AW869X_SYSINTM,
			     AW869X_SYSINTM_BSTERR | AW869X_SYSINTM_FF_AE |
			     AW869X_SYSINTM_FF_AF | AW869X_SYSINTM_DONE);
	if (error)
		return error;

	error = aw869x_set_standby(haptics);
	if (error)
		return error;

	error = regmap_update_bits(haptics->regmap, AW869X_PWMDBG, AW869X_PWMDBG_PWM_MODE,
				   FIELD_PREP(AW869X_PWMDBG_PWM_MODE, AW869X_PWMDBG_PWM_24K));
	if (error)
		return error;

	error = regmap_update_bits(haptics->regmap, AW869X_ANADBG, AW869X_ANADBG_IOC,
				   FIELD_PREP(AW869X_ANADBG_IOC, AW869X_ANADBG_IOC_4P65A));
	if (error)
		return error;

	/*
	 * The vendor driver turns the chip's own output protections off (they
	 * cut long effects short); over-current and over-temperature stay.
	 */
	error = regmap_set_bits(haptics->regmap, AW869X_DETCTRL,
				AW869X_DETCTRL_PROTECT_NO_ACTION);
	if (error)
		return error;

	error = regmap_clear_bits(haptics->regmap, AW869X_PWMPRC, AW869X_PWMPRC_PRC_ENABLE);
	if (error)
		return error;

	error = regmap_clear_bits(haptics->regmap, AW869X_PRLVL, AW869X_PRLVL_PR_ENABLE);
	if (error)
		return error;

	error = regmap_clear_bits(haptics->regmap, AW869X_BST_AUTO, AW869X_BST_AUTO_AUTOSW);
	if (error)
		return error;

	error = aw869x_offset_calibration(haptics);
	if (error)
		return error;

	return regmap_set_bits(haptics->regmap, AW869X_ADCTEST, AW869X_ADCTEST_VBAT_HW_COMP);
}

/* One period of a sine at f0, at the RAM mode sample rate. */
static int aw869x_ram_init(struct aw869x *haptics)
{
	struct aw869x_sram_header header;
	unsigned int i, samples;
	u8 *wave;
	int error;

	samples = DIV_ROUND_CLOSEST(AW869X_RAM_RATE_HZ * 10, haptics->f0_dhz);
	wave = kmalloc(samples, GFP_KERNEL);
	if (!wave)
		return -ENOMEM;

	for (i = 0; i < samples; i++) {
		s64 v = (s64)fixp_sin32_rad(i, samples) * AW869X_SINE_PEAK;

		wave[i] = (u8)(s8)DIV_S64_ROUND_CLOSEST(v, 0x7fffffff);
	}

	header.version = 0x01;
	header.start_address = cpu_to_be16(AW869X_RAM_BASE_ADDR + sizeof(header));
	header.end_address = cpu_to_be16(AW869X_RAM_BASE_ADDR + sizeof(header) + samples - 1);

	error = regmap_set_bits(haptics->regmap, AW869X_SYSCTRL, AW869X_SYSCTRL_RAMINIT);
	if (error)
		goto out;

	error = regmap_write(haptics->regmap, AW869X_RAMADDRH, AW869X_RAM_BASE_ADDR >> 8);
	if (error)
		goto out;

	error = regmap_write(haptics->regmap, AW869X_RAMADDRL, AW869X_RAM_BASE_ADDR & 0xff);
	if (error)
		goto out;

	error = regmap_noinc_write(haptics->regmap, AW869X_RAMDATA, &header, sizeof(header));
	if (error)
		goto out;

	error = regmap_noinc_write(haptics->regmap, AW869X_RAMDATA, wave, samples);
	if (error)
		goto out;

	error = regmap_clear_bits(haptics->regmap, AW869X_SYSCTRL, AW869X_SYSCTRL_RAMINIT);
out:
	kfree(wave);
	return error;
}

static irqreturn_t aw869x_irq(int irq, void *data)
{
	struct aw869x *haptics = data;
	unsigned int val;
	int error;

	error = regmap_read(haptics->regmap, AW869X_SYSINT, &val);
	if (error) {
		dev_err(haptics->dev, "failed to read SYSINT: %d\n", error);
		return IRQ_NONE;
	}

	if (val & AW869X_SYSINT_OVI)
		dev_err(haptics->dev, "wave data overflow or DPWM DC error\n");
	if (val & AW869X_SYSINT_UVLI)
		dev_err(haptics->dev, "under-voltage lockout\n");
	if (val & AW869X_SYSINT_OCDI)
		dev_err(haptics->dev, "over-current\n");
	if (val & AW869X_SYSINT_OTI)
		dev_err(haptics->dev, "over-temperature\n");

	return val ? IRQ_HANDLED : IRQ_NONE;
}

static bool aw869x_volatile_reg(struct device *dev, unsigned int reg)
{
	return true;
}

static bool aw869x_precious_reg(struct device *dev, unsigned int reg)
{
	return reg == AW869X_SYSINT || reg == AW869X_RAMDATA;
}

static const struct regmap_config aw869x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = AW869X_MAX_REG,
	.volatile_reg = aw869x_volatile_reg,
	.precious_reg = aw869x_precious_reg,
	.cache_type = REGCACHE_NONE,
};

static int aw869x_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct aw869x *haptics;
	unsigned int val, f0_hz;
	int f0, error;

	haptics = devm_kzalloc(dev, sizeof(*haptics), GFP_KERNEL);
	if (!haptics)
		return -ENOMEM;

	haptics->dev = dev;

	error = device_property_read_u32(dev, "awinic,f0-hz", &f0_hz);
	if (error)
		return dev_err_probe(dev, error, "failed to read awinic,f0-hz\n");
	if (f0_hz < 50 || f0_hz > 500)
		return dev_err_probe(dev, -EINVAL, "awinic,f0-hz %u out of range\n", f0_hz);
	haptics->f0_dhz = f0_hz * 10;

	haptics->regmap = devm_regmap_init_i2c(client, &aw869x_regmap_config);
	if (IS_ERR(haptics->regmap))
		return dev_err_probe(dev, PTR_ERR(haptics->regmap),
				     "failed to allocate register map\n");

	haptics->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(haptics->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(haptics->reset_gpio),
				     "failed to get reset gpio\n");

	aw869x_hw_reset(haptics);

	error = regmap_read(haptics->regmap, AW869X_ID, &val);
	if (error)
		return dev_err_probe(dev, error, "failed to read chip ID\n");

	if (val != AW8695_CHIPID && val != AW8697_CHIPID)
		return dev_err_probe(dev, -ENODEV, "unknown chip ID 0x%02x\n", val);
	haptics->chip_id = val;

	error = regmap_write(haptics->regmap, AW869X_ID, AW869X_SOFT_RESET);
	if (error)
		return dev_err_probe(dev, error, "failed to reset the chip\n");
	usleep_range(2000, 2500);

	error = aw869x_hw_init(haptics);
	if (error)
		return dev_err_probe(dev, error, "failed to set the chip up\n");

	f0 = aw869x_detect_f0(haptics);
	if (f0 < 0) {
		dev_warn(dev, "f0 detection failed (%d), using %u Hz\n", f0, f0_hz);
	} else if (abs(f0 - (int)haptics->f0_dhz) * 100 >
		   haptics->f0_dhz * AW869X_F0_TOLERANCE_PCT) {
		dev_warn(dev, "detected f0 %d.%d Hz is far from %u Hz, using %u Hz\n",
			 f0 / 10, f0 % 10, f0_hz, f0_hz);
	} else {
		haptics->f0_dhz = f0;
	}

	error = aw869x_ram_init(haptics);
	if (error)
		return dev_err_probe(dev, error, "failed to write the waveform\n");

	dev_info(dev, "AW86%02x, LRA at %u.%u Hz\n", haptics->chip_id,
		 haptics->f0_dhz / 10, haptics->f0_dhz % 10);

	if (client->irq > 0) {
		error = devm_request_threaded_irq(dev, client->irq, NULL, aw869x_irq,
						  IRQF_ONESHOT, NULL, haptics);
		if (error)
			return dev_err_probe(dev, error, "failed to request interrupt\n");
	}

	INIT_DELAYED_WORK(&haptics->play_work, aw869x_play_work);

	haptics->input = devm_input_allocate_device(dev);
	if (!haptics->input)
		return -ENOMEM;

	haptics->input->name = "aw8695-haptics";
	haptics->input->close = aw869x_close;
	input_set_drvdata(haptics->input, haptics);
	input_set_capability(haptics->input, EV_FF, FF_RUMBLE);

	error = input_ff_create_memless(haptics->input, NULL, aw869x_play_effect);
	if (error)
		return dev_err_probe(dev, error, "failed to create FF device\n");

	error = input_register_device(haptics->input);
	if (error)
		return dev_err_probe(dev, error, "failed to register input device\n");

	i2c_set_clientdata(client, haptics);

	return 0;
}

static int aw869x_suspend(struct device *dev)
{
	struct aw869x *haptics = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&haptics->play_work);
	haptics->level = 0;

	return aw869x_stop(haptics);
}

static DEFINE_SIMPLE_DEV_PM_OPS(aw869x_pm_ops, aw869x_suspend, NULL);

static const struct of_device_id aw869x_of_match[] = {
	{ .compatible = "awinic,aw8695" },
	{ .compatible = "awinic,aw8697" },
	{ }
};
MODULE_DEVICE_TABLE(of, aw869x_of_match);

static struct i2c_driver aw869x_driver = {
	.driver = {
		.name = "aw8695-haptics",
		.of_match_table = aw869x_of_match,
		.pm = pm_sleep_ptr(&aw869x_pm_ops),
	},
	.probe = aw869x_probe,
};
module_i2c_driver(aw869x_driver);

MODULE_DESCRIPTION("Awinic AW8695/AW8697 LRA haptics driver");
MODULE_LICENSE("GPL");
