 /*
  * max1161x.c
  * Copyright (C) 2012- Brent Lu
  *
  * based on linux/drivers/staging/iio/adc/max1363.c
  * Copyright (C) 2008-2010 Jonathan Cameron
  *
  * based on linux/drivers/i2c/chips/max123x
  * Copyright (C) 2002-2004 Stefan Eletzhofer
  *
  * based on linux/drivers/acron/char/pcf8583.c
  * Copyright (C) 2000 Russell King
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License version 2 as
  * published by the Free Software Foundation.
  *
  * max1161x.c
  *
  * Partial support for max1161x family chips.
  *
  *
  * - Control of internal reference.
  */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/fs.h>
#include "max1161x.h"

enum {
	max11612 = 0,
	max11613,
	max11614,
	max11615,
	max11616,
	max11617,
};

/**
 * struct max1161x_chip_info - chip specifc information
 * @mode_list:		array of available scan modes
 * @default_mode:	the scan mode in which the chip starts up
 * @int_vref_mv:	the internal reference voltage
 * @num_modes:		the number of scan modes available
 * @bits:		accuracy of the adc in bits
 */
struct max1161x_chip_info {
	const enum max1161x_scan_mode	*mode_list;
	enum max1161x_scan_mode		default_mode;
	u16				int_vref_mv;
	u8				num_modes;
	u8				bits;
};

/**
 * struct max1161x_state - driver instance specific data
 * @chip_info:		chip model specific constants, available modes etc
 * @current_mode:	the scan mode of this chip
 * @mutex:		the mutex
 * @setupbyte:		cache of current device setup byte
 * @configbyte:		cache of current device config byte
 */
struct max1161x_state {
	const struct max1161x_chip_info	*chip_info;
	enum max1161x_scan_mode		current_mode;
	struct mutex			mutex;
	u8				setupbyte;
	u8				configbyte;
};

static struct i2c_client *max1161x_client;

#define MAX1161X_MODE_SINGLE(_num) {					\
		.conf = MAX1161X_CONFIG_CHANNEL_SEL(_num)		\
			| MAX1161X_CONFIG_SCAN_SINGLE_1			\
			| MAX1161X_CONFIG_SE,				\
		.num_sample = 1,					\
}

#define MAX1161X_MODE_SCAN_TO_CHANNEL(_num) {				\
		.conf = MAX1161X_CONFIG_CHANNEL_SEL(_num)		\
			| MAX1161X_CONFIG_SCAN_TO_CS			\
			| MAX1161X_CONFIG_SE,				\
		.num_sample = _num + 1,					\
}

#define MAX1161X_MODE_SCAN_MID_TO_CHANNEL(_mid, _num) {			\
		.conf = MAX1161X_CONFIG_CHANNEL_SEL(_num)		\
			| MAX1161X_CONFIG_SCAN_MID_TO_CHANNEL		\
			| MAX1161X_CONFIG_SE,				\
		.num_sample = (_num - _mid) + 1,			\
}

#define MAX1161X_MODE_DIFF_SINGLE(_nump, _numm) {			\
		.conf = MAX1161X_CONFIG_CHANNEL_SEL(_nump)		\
			| MAX1161X_CONFIG_SCAN_SINGLE_1			\
			| MAX1161X_CONFIG_DE,				\
		.num_sample = 1,					\
}

#define MAX1161X_MODE_DIFF_SCAN_TO_CHANNEL(_num, _numvals) {		\
		.conf = MAX1161X_CONFIG_CHANNEL_SEL(_num)		\
			| MAX1161X_CONFIG_SCAN_TO_CS			\
			| MAX1161X_CONFIG_DE,				\
		.num_sample = _numvals,					\
}

#define MAX1161X_MODE_DIFF_SCAN_MID_TO_CHANNEL(_num, _numvals) {	\
		.conf = MAX1161X_CONFIG_CHANNEL_SEL(_num)		\
			| MAX1161X_CONFIG_SCAN_MID_TO_CHANNEL		\
			| MAX1161X_CONFIG_SE,				\
		.num_sample = _numvals,					\
}

/**
 * struct max1161x_mode_info - scan mode information
 * @conf:	The corresponding value of the configuration register
 * @num_sample:	Number of sample to read from I2C bus
 */
struct max1161x_mode_info {
	int8_t	conf;
	int8_t	num_sample;
};

static const struct max1161x_mode_info max1161x_mode_table[] = {
	/* All of the single channel options first */
	MAX1161X_MODE_SINGLE(0),
	MAX1161X_MODE_SINGLE(1),
	MAX1161X_MODE_SINGLE(2),
	MAX1161X_MODE_SINGLE(3),
	MAX1161X_MODE_SINGLE(4),
	MAX1161X_MODE_SINGLE(5),
	MAX1161X_MODE_SINGLE(6),
	MAX1161X_MODE_SINGLE(7),
	MAX1161X_MODE_SINGLE(8),
	MAX1161X_MODE_SINGLE(9),
	MAX1161X_MODE_SINGLE(10),
	MAX1161X_MODE_SINGLE(11),

	MAX1161X_MODE_DIFF_SINGLE(0, 1),
	MAX1161X_MODE_DIFF_SINGLE(2, 3),
	MAX1161X_MODE_DIFF_SINGLE(4, 5),
	MAX1161X_MODE_DIFF_SINGLE(6, 7),
	MAX1161X_MODE_DIFF_SINGLE(8, 9),
	MAX1161X_MODE_DIFF_SINGLE(10, 11),
	MAX1161X_MODE_DIFF_SINGLE(1, 0),
	MAX1161X_MODE_DIFF_SINGLE(3, 2),
	MAX1161X_MODE_DIFF_SINGLE(5, 4),
	MAX1161X_MODE_DIFF_SINGLE(7, 6),
	MAX1161X_MODE_DIFF_SINGLE(9, 8),
	MAX1161X_MODE_DIFF_SINGLE(11, 10),

	/* The multichannel scans next */
	MAX1161X_MODE_SCAN_TO_CHANNEL(1),
	MAX1161X_MODE_SCAN_TO_CHANNEL(2),
	MAX1161X_MODE_SCAN_MID_TO_CHANNEL(2, 3),
	MAX1161X_MODE_SCAN_TO_CHANNEL(3),
	MAX1161X_MODE_SCAN_TO_CHANNEL(4),
	MAX1161X_MODE_SCAN_TO_CHANNEL(5),
	MAX1161X_MODE_SCAN_TO_CHANNEL(6),
	MAX1161X_MODE_SCAN_MID_TO_CHANNEL(6, 7),
	MAX1161X_MODE_SCAN_TO_CHANNEL(7),
	MAX1161X_MODE_SCAN_MID_TO_CHANNEL(6, 8),
	MAX1161X_MODE_SCAN_TO_CHANNEL(8),
	MAX1161X_MODE_SCAN_MID_TO_CHANNEL(6, 9),
	MAX1161X_MODE_SCAN_TO_CHANNEL(9),
	MAX1161X_MODE_SCAN_MID_TO_CHANNEL(6, 10),
	MAX1161X_MODE_SCAN_TO_CHANNEL(10),
	MAX1161X_MODE_SCAN_MID_TO_CHANNEL(6, 11),
	MAX1161X_MODE_SCAN_TO_CHANNEL(11),

	MAX1161X_MODE_DIFF_SCAN_TO_CHANNEL(2, 2),
	MAX1161X_MODE_DIFF_SCAN_TO_CHANNEL(4, 3),
	MAX1161X_MODE_DIFF_SCAN_TO_CHANNEL(6, 4),
	MAX1161X_MODE_DIFF_SCAN_MID_TO_CHANNEL(8, 2),
	MAX1161X_MODE_DIFF_SCAN_TO_CHANNEL(8, 5),
	MAX1161X_MODE_DIFF_SCAN_MID_TO_CHANNEL(10, 3),
	MAX1161X_MODE_DIFF_SCAN_TO_CHANNEL(10, 6),
	MAX1161X_MODE_DIFF_SCAN_TO_CHANNEL(3, 2),
	MAX1161X_MODE_DIFF_SCAN_TO_CHANNEL(5, 3),
	MAX1161X_MODE_DIFF_SCAN_TO_CHANNEL(7, 4),
	MAX1161X_MODE_DIFF_SCAN_MID_TO_CHANNEL(9, 2),
	MAX1161X_MODE_DIFF_SCAN_TO_CHANNEL(9, 5),
	MAX1161X_MODE_DIFF_SCAN_MID_TO_CHANNEL(11, 3),
	MAX1161X_MODE_DIFF_SCAN_TO_CHANNEL(11, 6),
};

#define MAX1161X_I2C_NUM_RETRY	10
#define MAX1161X_I2C_DELAY_DFLT	20	/* ms */

static int max1161x_i2c_read(struct i2c_client *client, char *buf, int count)
{
	int ret, tries;

	for (tries = 0, ret = -1;
	     (tries < MAX1161X_I2C_NUM_RETRY) && (ret < 0);
	     tries++) {
		ret = i2c_master_recv(client, buf, count);
		if (ret < 0)
			msleep(MAX1161X_I2C_DELAY_DFLT);
	}

	if (ret < 0)
		dev_err(&client->dev, "Unable to read from I2C, ret %d\n", ret);

	return ret;
}

static int max1161x_i2c_write(struct i2c_client *client, char *buf, int count)
{
	int ret, tries;

	for (tries = 0, ret = -1;
	     (tries < MAX1161X_I2C_NUM_RETRY) && (ret < 0);
	     tries++) {
		ret = i2c_master_send(client, buf, count);
		if (ret < 0)
			msleep(MAX1161X_I2C_DELAY_DFLT);
	}

	if (ret < 0)
		dev_err(&client->dev, "Unable to write to I2C, ret %d\n", ret);

	return ret;
}

static int max1161x_write_basic_config(unsigned char d1, unsigned char d2, int num)
{
	u8 tx_buf[2] = {d1, d2};

	return max1161x_i2c_write(max1161x_client, tx_buf, num);
}

static int max1161x_set_scan_mode(struct max1161x_state *st)
{
	u8 configbyte;

	configbyte = st->configbyte & ~(MAX1161X_CONFIG_CHANNEL_SEL_MASK |
					MAX1161X_CONFIG_SCAN_MASK |
					MAX1161X_CONFIG_SE_DE_MASK);
	configbyte |= max1161x_mode_table[st->current_mode].conf;

	if (configbyte == st->configbyte)
		return 0;

	st->configbyte = configbyte;

	return max1161x_write_basic_config(st->configbyte, 0, 1);
}

static int max1161x_read_channels(enum max1161x_scan_mode mode, u16 *val)
{
	struct max1161x_state *st;
	u8 rxbuf[2 * MAX1161X_CHANNEL_MAX];
	int ret = 0, i;

	BUG_ON(max1161x_client == NULL);
	st = i2c_get_clientdata(max1161x_client);

	mutex_lock(&st->mutex);

	/* Check to see if current scan mode is correct */
	if (st->current_mode != mode) {
		/* Update scan mode if needed */
		st->current_mode = mode;
		ret = max1161x_set_scan_mode(st);
		if (ret < 0)
			goto error_ret;
	}
	if (st->chip_info->bits > 8) {
		/* Get reading */
		ret = max1161x_i2c_read(max1161x_client, rxbuf,
					max1161x_mode_table[mode].num_sample * 2);
		if (ret < 0)
			goto error_ret;
		for (i = 0; i < max1161x_mode_table[mode].num_sample; i++)
			val[i] = (u16)(rxbuf[(i * 2) + 1]) |
				 ((u16)(rxbuf[i * 2] & 0x0F)) << 8;
	} else {
		/* Get reading */
		ret = max1161x_i2c_read(max1161x_client, rxbuf,
					max1161x_mode_table[mode].num_sample);
		if (ret < 0)
			goto error_ret;
		for (i = 0; i < max1161x_mode_table[mode].num_sample; i++)
			val[i] = rxbuf[i];
	}

error_ret:
	mutex_unlock(&st->mutex);
	return ret;
}

static const enum max1161x_scan_mode max11612_mode_list[] = {
	_s0, _s1, _s2, _s3,
	s0to1, s0to2, s0to3,
	s2to3,
	d0m1, d2m3, d1m0, d3m2,
	d0m1to2m3, d1m0to3m2,
};

static const enum max1161x_scan_mode max11614_mode_list[] = {
	_s0, _s1, _s2, _s3, _s4, _s5, _s6, _s7,
	s0to1, s0to2, s0to3, s0to4, s0to5, s0to6, s0to7,
	s6to7,
	d0m1, d2m3, d4m5, d6m7,
	d1m0, d3m2, d5m4, d7m6,
	d0m1to2m3, d0m1to4m5, d0m1to6m7,
	d1m0to3m2, d1m0to5m4, d1m0to7m6,
};

static const enum max1161x_scan_mode max11616_mode_list[] = {
	_s0, _s1, _s2, _s3, _s4, _s5, _s6, _s7, _s8, _s9, _s10, _s11,
	s0to1, s0to2, s0to3, s0to4, s0to5, s0to6,
	s0to7, s0to8, s0to9, s0to10, s0to11,
	d0m1, d2m3, d4m5, d6m7, d8m9, d10m11,
	d1m0, d3m2, d5m4, d7m6, d9m8, d11m10,
	d0m1to2m3, d0m1to4m5, d0m1to6m7, d0m1to8m9, d0m1to10m11,
	d1m0to3m2, d1m0to5m4, d1m0to7m6, d1m0to9m8, d1m0to11m10,
	s6to7, s6to8, s6to9, s6to10, s6to11,
	d6m7to8m9, d6m7to10m11, d7m6to9m8, d7m6to11m10,
};

static const struct max1161x_chip_info max1161x_chip_info_tbl[] = {
	[max11612] = {
		.bits = 12,
		.int_vref_mv = 4096,
		.mode_list = max11612_mode_list,
		.num_modes = ARRAY_SIZE(max11612_mode_list),
		.default_mode = s0to3,
	},
	[max11613] = {
		.bits = 12,
		.int_vref_mv = 2048,
		.mode_list = max11612_mode_list,
		.num_modes = ARRAY_SIZE(max11612_mode_list),
		.default_mode = s0to3,
	},
	[max11614] = {
		.bits = 12,
		.int_vref_mv = 4096,
		.mode_list = max11614_mode_list,
		.num_modes = ARRAY_SIZE(max11614_mode_list),
		.default_mode = s0to7,
	},
	[max11615] = {
		.bits = 12,
		.int_vref_mv = 2048,
		.mode_list = max11614_mode_list,
		.num_modes = ARRAY_SIZE(max11614_mode_list),
		.default_mode = s0to7,
	},
	[max11616] = {
		.bits = 12,
		.int_vref_mv = 4098,
		.mode_list = max11616_mode_list,
		.num_modes = ARRAY_SIZE(max11616_mode_list),
		.default_mode = s0to11,
	},
	[max11617] = {
		.bits = 12,
		.int_vref_mv = 2048,
		.mode_list = max11616_mode_list,
		.num_modes = ARRAY_SIZE(max11616_mode_list),
		.default_mode = s0to11,
	},
};

static ssize_t max1161x_reg_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct max1161x_state *st;
	int ret = 0;

	BUG_ON(max1161x_client == NULL);
	st = i2c_get_clientdata(max1161x_client);

	mutex_lock(&st->mutex);
	/* There is no way to read them back from chip so just print the variable */
	ret += sprintf(buf + ret, "Register:\n");
	ret += sprintf(buf + ret, "  SETUP  =0x%02X\n", st->setupbyte);
	ret += sprintf(buf + ret, "  CONFIG =0x%02X\n", st->configbyte);
	mutex_unlock(&st->mutex);

	return ret;
}

static ssize_t max1161x_reg_store(struct device *dev,
				  struct device_attribute *attr, const char *buf,
				  size_t count)
{
	struct max1161x_state *st;
	u32 setupbyte, configbyte;
	int ret;

	BUG_ON(max1161x_client == NULL);
	st = i2c_get_clientdata(max1161x_client);

	if (sscanf(buf, "%x %x", &setupbyte, &configbyte) <= 0) {
		dev_err(&max1161x_client->dev, "Unable to parse user data\n");
		return -EINVAL;
	}

	dev_err(&max1161x_client->dev, "setupbyte %d, configbyte %d\n", setupbyte,
		configbyte);

	mutex_lock(&st->mutex);
	st->setupbyte = (u8)MAX1161X_SETUP_BYTE(setupbyte);
	st->configbyte = (u8)MAX1161X_CONFIG_BYTE(configbyte);

	ret = max1161x_write_basic_config(st->setupbyte, st->configbyte, 2);
	mutex_unlock(&st->mutex);

	if (ret < 0) {
		dev_err(&max1161x_client->dev, "Unable to write data, ret %d\n", ret);
		return -EIO;
	}

	return count;
}

static DEVICE_ATTR(max1161x_reg, 0644, max1161x_reg_show, max1161x_reg_store);

static const char *max1161x_scan_mode_text[] = {
	/* Single read of a single channel */
	"_s0", "_s1", "_s2", "_s3", "_s4", "_s5", "_s6", "_s7", "_s8", "_s9", "_s10", "_s11",
	/* Differential single read */
	"d0m1", "d2m3", "d4m5", "d6m7", "d8m9", "d10m11",
	"d1m0", "d3m2", "d5m4", "d7m6", "d9m8", "d11m10",
	/* Scan to channel and mid to channel where overlapping */
	"s0to1", "s0to2", "s2to3", "s0to3", "s0to4", "s0to5", "s0to6",
	"s6to7", "s0to7", "s6to8", "s0to8", "s6to9",
	"s0to9", "s6to10", "s0to10", "s6to11", "s0to11",
	/* Differential scan to channel and mid to channel where overlapping */
	"d0m1to2m3", "d0m1to4m5", "d0m1to6m7", "d6m7to8m9",
	"d0m1to8m9", "d6m7to10m11", "d0m1to10m11", "d1m0to3m2",
	"d1m0to5m4", "d1m0to7m6", "d7m6to9m8", "d1m0to9m8",
	"d7m6to11m10", "d1m0to11m10",
};

static int max1161x_validate_mode(struct max1161x_state *st,
				  enum max1161x_scan_mode mode)
{
	int i;

	for (i = 0; i < st->chip_info->num_modes; i++) {
		if (st->chip_info->mode_list[i] == mode)
			return 0;
	}

	return -1;
}

static ssize_t max1161x_channel_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct max1161x_state *st;
	u16 data[MAX1161X_CHANNEL_MAX];
	int ret, i;

	BUG_ON(max1161x_client == NULL);
	st = i2c_get_clientdata(max1161x_client);

	ret = max1161x_read_channels(st->current_mode, data);
	if (ret < 0) {
		dev_err(&max1161x_client->dev, "Unable to write data, ret %d\n", ret);
		return -EIO;
	}

	ret = sprintf(buf, "Scan mode %s:", max1161x_scan_mode_text[st->current_mode]);
	for (i = 0; i < max1161x_mode_table[st->current_mode].num_sample; i++)
		ret += sprintf(buf + ret, " 0x%02X", data[i]);
	ret += sprintf(buf + ret, "\n");

	return ret;
}

static ssize_t max1161x_channel_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct max1161x_state *st;
	enum max1161x_scan_mode mode;
	char *tmp;
	int ret;

	BUG_ON(max1161x_client == NULL);
	st = i2c_get_clientdata(max1161x_client);

	/* remove new line character */
	for (tmp = buf; *tmp != '\0'; tmp++)
		if ((*tmp == '\r') || (*tmp == '\n')) {
			*tmp = '\0';
			break;
		}

	for (mode = 0; mode < (sizeof(max1161x_scan_mode_text) / sizeof(char *)); mode++) {
		if (strcmp(buf, max1161x_scan_mode_text[mode]) != 0)
			continue;

		ret = max1161x_validate_mode(st, mode);
		if (ret < 0) {
			dev_err(&max1161x_client->dev, "Invalid mode %s\n", buf);
			return -EINVAL;
		}

		mutex_lock(&st->mutex);
		if (st->current_mode != mode) {
			/* Update scan mode if needed */
			st->current_mode = mode;
			ret = max1161x_set_scan_mode(st);
			if (ret < 0) {
				mutex_unlock(&st->mutex);
				dev_err(&max1161x_client->dev, "Unable to write data, ret %d\n", ret);
				return -EIO;
			}
		}
		mutex_unlock(&st->mutex);
		return count;
	}

	dev_err(&max1161x_client->dev, "Unknown mode %s\n", buf);
	return -EINVAL;
}

static DEVICE_ATTR(max1161x_channel, 0644, max1161x_channel_show, max1161x_channel_store);

static long max1161x_adc_ioctl(struct file *filp, unsigned int cmd,
			       unsigned long arg)
{
	struct max1161x_state *st;
	struct max1161x_adc_user_parms par;
	int ret;

	ret = copy_from_user(&par, (void __user *)arg, sizeof(par));
	if (ret) {
		dev_dbg(&max1161x_client->dev, "copy_from_user: %d\n", ret);
		return -EACCES;
	}

	BUG_ON(max1161x_client == NULL);
	st = i2c_get_clientdata(max1161x_client);

	switch (cmd) {
	case MAX1161X_ADC_IOCX_ADC_SETUP_READ: {
		mutex_lock(&st->mutex);
		par.setup.reference = st->setupbyte & MAX1161X_SETUP_SEL_MASK;
		par.setup.external_clock = (st->setupbyte & MAX1161X_SETUP_EXT_CLOCK) ? 1 : 0;
		par.setup.bipolar = (st->setupbyte & MAX1161X_SETUP_BIPOLAR) ? 1 : 0;
		mutex_unlock(&st->mutex);
		break;
	}
	case MAX1161X_ADC_IOCX_ADC_SETUP_WRITE: {
		u8 setupbyte;

		setupbyte = (par.setup.reference & MAX1161X_SETUP_SEL_MASK) |
			    (par.setup.external_clock ? MAX1161X_SETUP_EXT_CLOCK : MAX1161X_SETUP_INT_CLOCK) |
			    (par.setup.bipolar ? MAX1161X_SETUP_BIPOLAR : MAX1161X_SETUP_UNIPOLAR) |
			    MAX1161X_SETUP_NORESET;
		setupbyte = MAX1161X_SETUP_BYTE(setupbyte);

		mutex_lock(&st->mutex);
		if (st->setupbyte != setupbyte) {
			st->setupbyte = setupbyte;
			ret = max1161x_write_basic_config(st->setupbyte, 0, 1);
			if (ret < 0) {
				mutex_unlock(&st->mutex);
				return -EIO;
			}
		}
		mutex_unlock(&st->mutex);
		break;
	}
	case MAX1161X_ADC_IOCX_ADC_RAW_READ: {
		ret = max1161x_validate_mode(st, par.raw_read.mode);
		if (ret < 0)
			return -EINVAL;

		ret = max1161x_read_channels(par.raw_read.mode, par.raw_read.result);
		if (ret < 0)
			return -EIO;
		break;
	}
	default:
		return -EINVAL;
	}

	ret = copy_to_user((void __user *)arg, &par, sizeof(par));
	if (ret) {
		dev_dbg(&max1161x_client->dev, "copy_to_user: %d\n", ret);
		return -EACCES;
	}

	return 0;
}

static const struct file_operations max1161x_adc_fileops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = max1161x_adc_ioctl
};

static struct miscdevice max1161x_adc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "max1161x-adc",
	.fops = &max1161x_adc_fileops
};

static int __devinit max1161x_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	int ret;
	struct max1161x_state *st;

	st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (st == NULL)
		return -ENOMEM;

	max1161x_client = client;

	i2c_set_clientdata(max1161x_client, st);

	mutex_init(&st->mutex);

	st->chip_info = &max1161x_chip_info_tbl[id->driver_data];
	st->current_mode = st->chip_info->default_mode;
	st->setupbyte = MAX1161X_SETUP_AIN3_IS_AIN3_REF_IS_VDD |
			MAX1161X_SETUP_POWER_UP_INT_REF|
			MAX1161X_SETUP_INT_CLOCK |
			MAX1161X_SETUP_UNIPOLAR |
			MAX1161X_SETUP_NORESET;
	st->setupbyte = MAX1161X_SETUP_BYTE(st->setupbyte);
	st->configbyte = max1161x_mode_table[st->current_mode].conf;
	st->configbyte = MAX1161X_CONFIG_BYTE(st->configbyte);

	ret = max1161x_write_basic_config(st->setupbyte, st->configbyte, 2);
	if (ret < 0)
		goto error_free_device;

	ret = misc_register(&max1161x_adc_device);
	if (ret) {
		dev_err(&client->dev, "Unable to register misc_device, ret %d\n", ret);
		goto error_misc;
	}

	ret = device_create_file(&client->dev, &dev_attr_max1161x_reg);
	if (ret)
		dev_err(&client->dev, "Unable to register devive %s, ret %d\n",
			dev_attr_max1161x_reg.attr.name, ret);
	ret = device_create_file(&client->dev, &dev_attr_max1161x_channel);
	if (ret)
		dev_err(&client->dev, "Unable to register devive %s, ret %d\n",
			dev_attr_max1161x_channel.attr.name, ret);

	return 0;

error_misc:
	misc_deregister(&max1161x_adc_device);
error_free_device:
	kfree(st);
	i2c_set_clientdata(max1161x_client, NULL);
	max1161x_client = NULL;

	return ret;
}

static int __devexit max1161x_remove(struct i2c_client *client)
{
	struct max1161x_state *st = i2c_get_clientdata(client);

	device_remove_file(&client->dev, &dev_attr_max1161x_reg);
	device_remove_file(&client->dev, &dev_attr_max1161x_channel);

	misc_deregister(&max1161x_adc_device);

	kfree(st);
	max1161x_client = NULL;

	return 0;
}

static const struct i2c_device_id max1161x_id[] = {
	{ "max11612", max11612 },
	{ "max11613", max11613 },
	{ "max11614", max11614 },
	{ "max11615", max11615 },
	{ "max11616", max11616 },
	{ "max11617", max11617 },
	{}
};

MODULE_DEVICE_TABLE(i2c, max1161x_id);

static struct i2c_driver max1161x_driver = {
	.driver = {
		.name = "max1161x",
		.owner = THIS_MODULE,
	},
	.probe = max1161x_probe,
	.remove = __devexit_p(max1161x_remove),
	.id_table = max1161x_id,
};

static __init int max1161x_init(void)
{
	return i2c_add_driver(&max1161x_driver);
}

static __exit void max1161x_exit(void)
{
	i2c_del_driver(&max1161x_driver);
}

MODULE_AUTHOR("Brent Lu <brent.wy.lu@enskytech.com>");
MODULE_DESCRIPTION("Maxim 1161X ADC family");
MODULE_LICENSE("GPL v2");

module_init(max1161x_init);
module_exit(max1161x_exit);
