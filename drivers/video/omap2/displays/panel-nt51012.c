/*
 * Novatek NT51012 MIPI DSI driver
 *
 * Copyright (C) Texas Instruments
 * Author: Subbaraman Narayanamurthy <x0164410@ti.com>
 *
 * Based on original version from Jerry Alexander <x0135174@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//#define DEBUG

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/i2c.h>

#include <video/omapdss.h>
#include <video/omap-panel-generic.h>

#include "panel-nt51012.h"
#include <mach/bowser_idme_init.h>


static unsigned char first_suspend = 1;
static unsigned char panel_err_count = 0;

static unsigned char color_sat_enabled = 0;
static unsigned char gamma_ctrl = 0;
static unsigned char pwm_step_ctrl = 0;
static unsigned char pwm_frame_ctrl = 0;
static unsigned char post_evt3 = 0;

extern char idme_get_board_type_string(void);

static struct omap_video_timings nt51012_timings = {
	.x_res		= NT51012_WIDTH,
	.y_res		= NT51012_HEIGHT,
	.pixel_clock	= NT51012_PCLK,
	.hfp		= NT51012_HFP,
	.hsw            = NT51012_HSW,
	.hbp            = NT51012_HBP,
	.vfp            = NT51012_VFP,
	.vsw            = NT51012_VSW,
	.vbp            = NT51012_VBP,
};

/* device private data structure */
struct nt51012_data {
	struct mutex lock;

	struct omap_dss_device *dssdev;

	struct workqueue_struct *workq;
	struct delayed_work reset_work;

	int channel0;
	int channel1;
	char cabc_mode;
};

static DEVICE_ATTR(cabc_mode, S_IRUGO | S_IWUSR, nt51012_show_cabc_mode,
		nt51012_set_cabc_mode);

static DEVICE_ATTR(cabc_modes, S_IRUGO,
		nt51012_show_cabc_modes, NULL);

static DEVICE_ATTR(gamma_ctrl, S_IRUGO | S_IWUSR, nt51012_show_gamma_ctrl,
	nt51012_set_gamma_ctrl);

static DEVICE_ATTR(pwm_step_ctrl, S_IRUGO | S_IWUSR, nt51012_show_pwm_step_ctrl,
	nt51012_set_pwm_step_ctrl);

static DEVICE_ATTR(pwm_frame_ctrl, S_IRUGO | S_IWUSR, nt51012_show_pwm_frame_ctrl,
	nt51012_set_pwm_frame_ctrl);

static struct attribute *nt51012_attrs[] = {
	&dev_attr_cabc_mode.attr,
	&dev_attr_cabc_modes.attr,
	&dev_attr_gamma_ctrl.attr,
	&dev_attr_pwm_step_ctrl.attr,
	&dev_attr_pwm_frame_ctrl.attr,
	NULL,
};

static struct attribute_group nt51012_attr_group = {
	.attrs = nt51012_attrs,
};

static const char *cabc_modes[] = {
	"moving-image",
	"still-image",
	"ui",
	"off",
};

static ssize_t nt51012_show_cabc_mode(struct omap_dss_device *dssdev,
		struct device_attribute *attr,
		char *buf)
{
	struct nt51012_data *d2d = dev_get_drvdata(&dssdev->dev);
	const char *mode_str;
	char mode;
	int len;

	mode = d2d->cabc_mode;

	if (mode >= 0 && mode < ARRAY_SIZE(cabc_modes))
		mode_str = cabc_modes[mode];
	else
		mode_str = "unknown";

	len = snprintf(buf, PAGE_SIZE, "%s\n", mode_str);

	return len < PAGE_SIZE - 1 ? len : PAGE_SIZE - 1;
}

static ssize_t nt51012_set_cabc_mode(struct omap_dss_device *dssdev,
		struct device_attribute *attr,
		const char *buf, ssize_t count)
{
	struct nt51012_data *d2d = dev_get_drvdata(&dssdev->dev);
	char i;
	int r = 0;

	mutex_lock(&d2d->lock);

	for (i = 0; i < ARRAY_SIZE(cabc_modes); i++) {
		if (sysfs_streq(cabc_modes[i], buf))
			break;
	}

	if (i == ARRAY_SIZE(cabc_modes))
	{
		mutex_unlock(&d2d->lock);
		return -EINVAL;
	}

	if(d2d->cabc_mode == i)
	{
		mutex_unlock(&d2d->lock);
		return -EINVAL;
	}
	else
	{
		if(dssdev->state == OMAP_DSS_DISPLAY_SUSPENDED)
		{
			d2d->cabc_mode = i;
			mutex_unlock(&d2d->lock);
			return r;
		}

		dsi_bus_lock(dssdev);

		/* NT51012 control register 0xB0 has CABC_ENB[7:6] */
		r = dsi_vc_dcs_write_1(dssdev, 1, 0xB0, (i<<6)|0x3E);

		dsi_bus_unlock(dssdev);

		if(r)
		{
			mutex_unlock(&d2d->lock);
			return -EINVAL;
		}
		else
		{
			d2d->cabc_mode = i;
		}
	}

	mutex_unlock(&d2d->lock);
	return count;
}

static ssize_t nt51012_show_cabc_modes(struct omap_dss_device *dssdev,
		struct device_attribute *attr,
		char *buf)
{
	int len;
	int i;

	for (i = 0, len = 0;
	     len < PAGE_SIZE && i < ARRAY_SIZE(cabc_modes); i++)
		len += snprintf(&buf[len], PAGE_SIZE - len, "%s%s%s",
			i ? " " : "", cabc_modes[i],
			i == ARRAY_SIZE(cabc_modes) - 1 ? "\n" : "");

	return len < PAGE_SIZE ? len : PAGE_SIZE - 1;
}

static ssize_t nt51012_set_gamma_ctrl(struct omap_dss_device *dssdev,
		struct device_attribute *attr,
		const char *buf, ssize_t count)
{
	struct nt51012_data *d2d = dev_get_drvdata(&dssdev->dev);
	int value;
	int r = 0;

	if (sscanf(buf, "%d", &value) != 1)
		return -EINVAL;

	mutex_lock(&d2d->lock);

	if(dssdev->state == OMAP_DSS_DISPLAY_SUSPENDED)
	{
		mutex_unlock(&d2d->lock);
		return r;
	}

	dsi_bus_lock(dssdev);

	gamma_ctrl = value & 0xFF;

	r = dsi_vc_dcs_write_1(dssdev, 1, 0xF5, gamma_ctrl);

	dsi_bus_unlock(dssdev);

	if(r)
	{
		mutex_unlock(&d2d->lock);
		return -EINVAL;
	}

	mutex_unlock(&d2d->lock);

	return count;
}

static ssize_t nt51012_show_gamma_ctrl(struct omap_dss_device *dssdev,
		struct device_attribute *attr,
		char *buf)
{
	int len;
	len = snprintf(buf, PAGE_SIZE, "%x\n", gamma_ctrl);
	return len < PAGE_SIZE - 1 ? len : PAGE_SIZE - 1;
}

static ssize_t nt51012_set_pwm_step_ctrl(struct omap_dss_device *dssdev,
		struct device_attribute *attr,
		const char *buf, ssize_t count)
{
	struct nt51012_data *d2d = dev_get_drvdata(&dssdev->dev);
	int value;
	int r = 0;

	if (sscanf(buf, "%d", &value) != 1)
		return -EINVAL;

	mutex_lock(&d2d->lock);

	if(dssdev->state == OMAP_DSS_DISPLAY_SUSPENDED)
	{
		mutex_unlock(&d2d->lock);
		return r;
	}

	dsi_bus_lock(dssdev);

	pwm_step_ctrl = value & 0xFF;

	r = dsi_vc_dcs_write_1(dssdev, 1, 0xC3, pwm_step_ctrl);

	dsi_bus_unlock(dssdev);

	if(r)
	{
		mutex_unlock(&d2d->lock);
		return -EINVAL;
	}

	mutex_unlock(&d2d->lock);

	return count;
}

static ssize_t nt51012_show_pwm_step_ctrl(struct omap_dss_device *dssdev,
		struct device_attribute *attr,
		char *buf)
{
	int len;
	len = snprintf(buf, PAGE_SIZE, "%x\n", pwm_step_ctrl);
	return len < PAGE_SIZE - 1 ? len : PAGE_SIZE - 1;
}

static ssize_t nt51012_set_pwm_frame_ctrl(struct omap_dss_device *dssdev,
		struct device_attribute *attr,
		const char *buf, ssize_t count)
{
	struct nt51012_data *d2d = dev_get_drvdata(&dssdev->dev);
	int value;
	int r = 0;

	if (sscanf(buf, "%d", &value) != 1)
		return -EINVAL;

	mutex_lock(&d2d->lock);

	if(dssdev->state == OMAP_DSS_DISPLAY_SUSPENDED)
	{
		mutex_unlock(&d2d->lock);
		return r;
	}

	dsi_bus_lock(dssdev);

	pwm_frame_ctrl = value & 0xFF;

	r = dsi_vc_dcs_write_1(dssdev, 1, 0xC6, pwm_frame_ctrl);

	dsi_bus_unlock(dssdev);

	if(r)
	{
		mutex_unlock(&d2d->lock);
		return -EINVAL;
	}

	mutex_unlock(&d2d->lock);

	return count;
}

static ssize_t nt51012_show_pwm_frame_ctrl(struct omap_dss_device *dssdev,
		struct device_attribute *attr,
		char *buf)
{
	int len;
	len = snprintf(buf, PAGE_SIZE, "%x\n", pwm_frame_ctrl);
	return len < PAGE_SIZE - 1 ? len : PAGE_SIZE - 1;
}

static struct panel_board_data *get_board_data(struct omap_dss_device *dssdev)
{
	return (struct panel_board_data *)dssdev->data;
}

static void nt51012_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static void nt51012_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
}

static int nt51012_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	if (nt51012_timings.x_res != timings->x_res ||
			nt51012_timings.y_res != timings->y_res ||
			nt51012_timings.pixel_clock != timings->pixel_clock ||
			nt51012_timings.hsw != timings->hsw ||
			nt51012_timings.hfp != timings->hfp ||
			nt51012_timings.hbp != timings->hbp ||
			nt51012_timings.vsw != timings->vsw ||
			nt51012_timings.vfp != timings->vfp ||
			nt51012_timings.vbp != timings->vbp)
		return -EINVAL;

	return 0;
}

static void nt51012_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	*xres = nt51012_timings.x_res;
	*yres = nt51012_timings.y_res;
}

static int nt51012_lcd_enable(struct omap_dss_device *dssdev)
{
	struct panel_board_data *board_data = get_board_data(dssdev);

	if (board_data->lcd_en_gpio == -1)
		return -1;

	gpio_set_value(board_data->lcd_en_gpio, 1);
	gpio_set_value(board_data->cabc_en_gpio,1);
	dev_dbg(&dssdev->dev, "nt51012_lcd_enable\n");

	return 0;
}

static int nt51012_lcd_disable(struct omap_dss_device *dssdev)
{
	struct panel_board_data *board_data = get_board_data(dssdev);

	if (board_data->lcd_en_gpio == -1)
		return -1;

	gpio_set_value(board_data->lcd_en_gpio, 0);
	gpio_set_value(board_data->cabc_en_gpio,0);
	dev_dbg(&dssdev->dev, "nt51012_lcd_disable\n");

	return 0;
}

static int nt51012_probe(struct omap_dss_device *dssdev)
{
	struct nt51012_data *d2d;
	int r = 0;
	char *board_type_str = idme_get_board_type_string();

	dev_dbg(&dssdev->dev, "nt51012_probe\n");
	printk(KERN_INFO "******** board type = %s ********\n", board_type_str);

	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.timings = nt51012_timings;
	dssdev->ctrl.pixel_size = 24;
	dssdev->panel.acbi = 0;
	dssdev->panel.acb = 40;
	dssdev->panel.width_in_um = 94200;
	dssdev->panel.height_in_um = 150720;

	d2d = kzalloc(sizeof(*d2d), GFP_KERNEL);
	if (!d2d) {
		r = -ENOMEM;
		goto err;
	}

	d2d->dssdev = dssdev;

#if defined (CONFIG_BACKLIGHT_LP855X)
	d2d->cabc_mode = 3; /* CABC off */
#else
	d2d->cabc_mode = 0; /* Moving image */
#endif

	if (!(!(strcmp(board_type_str, "Tate PreEVT2.1")) ||
	     !(strcmp(board_type_str, "Tate EVT2.1")))) {
		printk(KERN_INFO "EVT3.0 or above identified\n");
		post_evt3 = 1;
	}

	mutex_init(&d2d->lock);

	d2d->workq = create_singlethread_workqueue("panel_rst");
	if(d2d->workq == NULL) {
		dev_err(&dssdev->dev, "Cannot create ESD reset workq\n");
		r = -ENOMEM;
		goto err_mem;
	}

	INIT_DELAYED_WORK_DEFERRABLE(&d2d->reset_work, panel_reset_work);

	dev_set_drvdata(&dssdev->dev, d2d);

	r = omap_dsi_request_vc(dssdev, &d2d->channel0);
	if (r)
	{
		dev_err(&dssdev->dev, "failed to get virtual channel0\n");
		goto err_wq;
	}

	r = omap_dsi_set_vc_id(dssdev, d2d->channel0, 0);
	if (r)
	{
		dev_err(&dssdev->dev, "failed to set VC_ID0\n");
		goto err_vc0;
	}

	r = omap_dsi_request_vc(dssdev, &d2d->channel1);
	if (r)
	{
		dev_err(&dssdev->dev, "failed to get virtual channel1\n");
		goto err_vc0;
	}

	r = omap_dsi_set_vc_id(dssdev, d2d->channel1, 0);
	if (r)
	{
		dev_err(&dssdev->dev, "failed to set VC_ID1\n");
		goto err_vc1;
	}

	r = sysfs_create_group(&dssdev->dev.kobj, &nt51012_attr_group);
	if (r) {
		dev_err(&dssdev->dev, "failed to create sysfs file\n");
	}


	dev_dbg(&dssdev->dev, "nt51012_probe done\n");
	printk("nt51012_probe done\n");

	return 0;

err_vc1:
	omap_dsi_release_vc(dssdev, d2d->channel1);
err_vc0:
	omap_dsi_release_vc(dssdev, d2d->channel0);
err_wq:
	destroy_workqueue(d2d->workq);
err_mem:
	kfree(d2d);
err:
	return r;
}

static void nt51012_remove(struct omap_dss_device *dssdev)
{
	struct nt51012_data *d2d = dev_get_drvdata(&dssdev->dev);

	omap_dsi_release_vc(dssdev, d2d->channel0);
	omap_dsi_release_vc(dssdev, d2d->channel1);

	sysfs_remove_group(&dssdev->dev.kobj, &nt51012_attr_group);

	nt51012_cancel_reset_work(dssdev);
	destroy_workqueue(d2d->workq);

	kfree(d2d);
}

static void nt51012_config(struct omap_dss_device *dssdev)
{
	struct nt51012_data *d2d = dev_get_drvdata(&dssdev->dev);
	u8 term_res_value = 0;
	u8 data = 0;
	int r = 0;

	// Termination resistance configuration @ TCON
	// 0x0F - 100ohm/100ohm
	// 0x07 - 100ohm/open
	// 0x0B - open/100ohm
	// 0x0D - 200ohm/200ohm

	term_res_value = 0x0B; // set it to open_100ohm

	//soft reset
	dsi_vc_dcs_write_1(dssdev, 0, 0x01, 0x00);
	msleep(20);  //need atleast 20ms after reset

	dsi_vc_dcs_write_1(dssdev, 0, 0xAE, term_res_value);

	//printk("test mode1\n");
	dsi_vc_dcs_write_1(dssdev, 0, 0xEE, 0xEA);
	//printk("test mode2\n");
	dsi_vc_dcs_write_1(dssdev, 0, 0xEF, 0x5F);
	//printk("bias\n");
	dsi_vc_dcs_write_1(dssdev, 0, 0xF2, 0x68);

	if (post_evt3) {

		printk("Applying Vendor specific settings for EVT3.0 Configuration and above\n");

		/* Read vendor id to set parameters for different panels */
		r = dsi_vc_dcs_read(dssdev, 0, 0xFC, &data, 1);
		if(r < 0)
			dev_err(&dssdev->dev,"dsi read err\n");
		else
			dev_dbg(&dssdev->dev,"data read: %x\n", data);

		/* Read bits [5:4] for vendor id */
		data = (data >> 4) & 0x03;

		printk("Vendor id: %d\n", data);

		switch(data)
		{
			case 1:
				/* CMI */
				dsi_vc_dcs_write_1(dssdev, 0, 0xB1, 0xFF);
				/* TCLV_OE timing adjust for Gate Pulse Modulation Function */
				dsi_vc_dcs_write_1(dssdev, 0, 0xB2, 0x7F);
				/* Differential impedance selection */
				dsi_vc_dcs_write_1(dssdev, 0, 0xAE, term_res_value);
				break;
			case 2:
				/* LGD */
				/* TCLV_OE timing adjust for Gate Pulse Modulation Function */
				dsi_vc_dcs_write_1(dssdev, 0, 0xB2, 0x7D);
				/* Gate OE width control which secures data charging time margin */
				dsi_vc_dcs_write_1(dssdev, 0, 0xB6, 0x18);
				/* AVDDG off */
				dsi_vc_dcs_write_1(dssdev, 0, 0xD2, 0x64);
				/* Differential impedance selection */
				dsi_vc_dcs_write_1(dssdev, 0, 0xAE, term_res_value);
				break;
			case 3:
				/* PLD */
				/* Differential impedance selection */
				dsi_vc_dcs_write_1(dssdev, 0, 0xAE, term_res_value);
				/* Selection of amplifier */
				dsi_vc_dcs_write_1(dssdev, 0, 0xBE, 0x02);
				/* Adjust drive timing 1 */
				dsi_vc_dcs_write_1(dssdev, 0, 0xB5, 0x90);
				/* Adjust drive timing 2 */
				dsi_vc_dcs_write_1(dssdev, 0, 0xB6, 0x09);
				break;
			case 0:
			default:
				/* TCLV_OE timing adjust for Gate Pulse Modulation Function */
				dsi_vc_dcs_write_1(dssdev, 0, 0xB2, 0x7D);
				/* Gate OE width control which secures data charging time margin */
				dsi_vc_dcs_write_1(dssdev, 0, 0xB6, 0x18);
				/* AVDDG off */
				dsi_vc_dcs_write_1(dssdev, 0, 0xD2, 0x64);
				/* Differential impedance selection */
				dsi_vc_dcs_write_1(dssdev, 0, 0xAE, term_res_value);
				break;
		}
	} else {
		printk("Not Applying Panel Vendor related settings for boards less than EVT3.0 Configuration\n");
	}

	/* Color saturation */
	if(color_sat_enabled)
	{
		dsi_vc_dcs_write_1(dssdev, 0, 0xB3, 0x03);

		dsi_vc_dcs_write_1(dssdev, 0, 0xC8, 0x04);
	}

	dsi_vc_dcs_write_1(dssdev, 0, 0xEE, 0x00);

	dsi_vc_dcs_write_1(dssdev, 0, 0xEF, 0x00);

	/* CABC settings suggested by Rajeev and Novatek */
	/* Gamma control */
	dsi_vc_dcs_write_1(dssdev, 0, 0xF5, 0xF0);

	/* PWM step control */
	dsi_vc_dcs_write_1(dssdev, 0, 0xC3, 0x41);

	/* PWM frame control */
	dsi_vc_dcs_write_1(dssdev, 0, 0xC6, 0x01);

	/* Read back values and store it*/
	if (post_evt3) {
		printk("Applying CABC settings for EVT3.0 Configuration and above\n");
		r = dsi_vc_dcs_read(dssdev, 0, 0xF5, &gamma_ctrl, 1);
		if(r < 0)
			printk("dsi read err\n");
		else {
			r = dsi_vc_dcs_read(dssdev, 0, 0xC3, &pwm_step_ctrl, 1);
			if(r < 0)
				printk("dsi read err\n");

			r = dsi_vc_dcs_read(dssdev, 0, 0xC6, &pwm_frame_ctrl, 1);
			if(r < 0)
				printk("dsi read err\n");
		}
	} else {
		printk("Not Applying CABC settings for boards less than EVT3.0 Configuration\n");
	}
	/* NT51012 control register 0xB0 has CABC_ENB[7:6] */
	dsi_vc_dcs_write_1(dssdev, 0, 0xB0, (d2d->cabc_mode<<6)|0x3E);
}

DECLARE_WAIT_QUEUE_HEAD(panel_init_queue);
DECLARE_WAIT_QUEUE_HEAD(panel_fini_queue);
int nt51012_panel_enabled = 0;
int lp855x_bl_off = 0;

static int nt51012_power_on(struct omap_dss_device *dssdev)
{
	struct nt51012_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r;

	/* At power on the first vsync has not been received yet */
	dssdev->first_vsync = false;

	dev_dbg(&dssdev->dev, "power_on\n");

	if (dssdev->platform_enable)
		dssdev->platform_enable(dssdev);

	r = omapdss_dsi_display_enable(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "failed to enable DSI\n");
		goto err_disp_enable;
	}

	/* first_suspend flag is to ignore suspend one time during init process
	we use this flag to re-init the TCON once by disable/enable panel
	voltage. From the next time suspend will help doing this */

	if (!dssdev->skip_init) {

		if (first_suspend) {
			/* Send sleep in command to the TCON */
			dsi_vc_dcs_write_1(dssdev, 1, 0x11, 0x01);
			msleep(160);
			/* Turn off the panel voltage first and wait for sometime */
			nt51012_lcd_disable(dssdev);
			msleep(100);
		}

		/* enable lcd */
		r = nt51012_lcd_enable(dssdev);

		/* Wait till TCON gets ready */
		msleep(100);

		/* do extra job to match kozio registers (???) */
		dsi_videomode_panel_preinit(dssdev);

		if (first_suspend)
			nt51012_config(dssdev);

		/* Go to HS mode after sending all the DSI commands in LP mode */
		omapdss_dsi_vc_enable_hs(dssdev, d2d->channel0, true);
		omapdss_dsi_vc_enable_hs(dssdev, d2d->channel1, true);

		/* 0x0e - 16bit
		* 0x1e - packed 18bit
		* 0x2e - unpacked 18bit
		* 0x3e - 24bit
		*/

		dsi_video_mode_enable(dssdev, 0x3e);
	}

	/* LCD is enabled, so wake up the Backlight power-on. */
	nt51012_panel_enabled = 1;

	if (dssdev->skip_init)
		dssdev->skip_init = false;

	wake_up(&panel_init_queue);
	if(r) {
		dev_err(&dssdev->dev, "failed to enable LCD\n");
		goto err_write_init;
	}

	dev_dbg(&dssdev->dev, "power_on done\n");

	return r;

err_write_init:
	omapdss_dsi_display_disable(dssdev, false, false);
err_disp_enable:
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	return r;
}

static void nt51012_power_off(struct omap_dss_device *dssdev)
{
	int r;

	/* wait until backlight turned off */
	wait_event_timeout(panel_fini_queue, lp855x_bl_off , msecs_to_jiffies(100) );
	lp855x_bl_off  = 0;

	dsi_video_mode_disable(dssdev);

	/* Send sleep in command to the TCON */
	dsi_vc_dcs_write_1(dssdev, 1, 0x11, 0x01);
	msleep(160);

	omapdss_dsi_display_disable(dssdev, true, false);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	/* disable lcd */
	r = nt51012_lcd_disable(dssdev);
	nt51012_panel_enabled = 0;
	if(r) {
		dev_err(&dssdev->dev, "failed to disable LCD\n");
	}
	return;
}

static void nt51012_disable(struct omap_dss_device *dssdev)
{
	struct nt51012_data *d2d = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "disable\n");

	mutex_lock(&d2d->lock);

	nt51012_cancel_reset_work(dssdev);

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		dsi_bus_lock(dssdev);

		nt51012_power_off(dssdev);

		dsi_bus_unlock(dssdev);
	}

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

	mutex_unlock(&d2d->lock);

	return;
}

static int nt51012_enable(struct omap_dss_device *dssdev)
{
	struct nt51012_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r = 0;

	dev_dbg(&dssdev->dev, "enable\n");

	mutex_lock(&d2d->lock);

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
	{
		mutex_unlock(&d2d->lock);
		return -EINVAL;
	}

	dsi_bus_lock(dssdev);

	r = nt51012_power_on(dssdev);

	dsi_bus_unlock(dssdev);

	if (r) {
		dev_dbg(&dssdev->dev, "enable failed\n");
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	} else {
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	}

	mutex_unlock(&d2d->lock);

	return r;
}

static void nt51012_cancel_reset_work(struct omap_dss_device *dssdev)
{
	struct nt51012_data *d2d = dev_get_drvdata(&dssdev->dev);

	if (!post_evt3)
		return;
	cancel_delayed_work(&d2d->reset_work);
}

static void nt51012_queue_reset_work(struct omap_dss_device *dssdev)
{
	struct nt51012_data *d2d = dev_get_drvdata(&dssdev->dev);

	if (!post_evt3)
		return;
	queue_delayed_work(d2d->workq, &d2d->reset_work, msecs_to_jiffies(500));
}

static void panel_reset_work(struct work_struct *work)
{
	struct nt51012_data *d2d = container_of(work, struct nt51012_data, reset_work.work);
	struct omap_dss_device *dssdev = d2d->dssdev;
	int r = 0;
	u8 data;
	static volatile int  inreset = 0;

	if (!inreset) {
		inreset=1;

		if (!post_evt3)
			return;

		msleep(300);

		printk("- reset panel\n");

		mutex_lock(&d2d->lock);

		dsi_bus_lock(dssdev);

		r = nt51012_reset(dssdev);
		if(r < 0) {
			printk("Panel NoAck & Error in panel_reset\n");
		}
		else {
			printk("Panel NoAck Resetted successfully\n");
		}


		dsi_bus_unlock(dssdev);


		mutex_unlock(&d2d->lock);
		inreset=0;
	}

	return;
}

int nt51012_reset_isr(struct omap_dss_device *dssdev)
{
	if (dssdev) {
		nt51012_queue_reset_work(dssdev);
	}
	return 0;
}

int nt51012_reset(struct omap_dss_device *dssdev)
{
	struct nt51012_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r = 0;


	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		r = -EINVAL;
		goto err;
	}

	dsi_video_mode_disable(dssdev);

	/* Send sleep in command to the TCON */
	r = dsi_vc_dcs_write_1(dssdev, 1, 0x11, 0x01);
	if(r < 0) {
		dev_err(&dssdev->dev, "error in sending sleep in command\n");
	}

	omapdss_dsi_display_disable(dssdev, true, false);


	/* disable lcd */
	r = nt51012_lcd_disable(dssdev);

	if(r) {
		dev_err(&dssdev->dev, "failed to disable LCD\n");
	}

	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

	msleep(500);

	r = omapdss_dsi_display_reset(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "failed to enable DSI\n");
		goto err;
	}

	/* enable lcd */
	r = nt51012_lcd_enable(dssdev);

	/* Wait till TCON gets ready */
	msleep(100);

	dsi_videomode_panel_preinit(dssdev);

	nt51012_config(dssdev);

	/* Go to HS mode after sending all the DSI commands in LP mode */
	omapdss_dsi_vc_enable_hs(dssdev, d2d->channel0, true);
	omapdss_dsi_vc_enable_hs(dssdev, d2d->channel1, true);

	dsi_video_mode_enable(dssdev, 0x3e);
	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;


err:
	return r;
}

static int nt51012_suspend(struct omap_dss_device *dssdev)
{
	struct nt51012_data *nd = dev_get_drvdata(&dssdev->dev);
	int r;

	/* This suspend is part of fb_blank from HWC? Ignore this as it
	interrupts the panel init process */

	mutex_lock(&nd->lock);

	if(first_suspend)
	{
		dev_dbg(&dssdev->dev, "not suspending now\n");
		first_suspend = 0;
		mutex_unlock(&nd->lock);
		return 0;
	}
	dev_dbg(&dssdev->dev, "suspend\n");

	nt51012_cancel_reset_work(dssdev);

	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE) {
		r = -EINVAL;
		goto err;
	}

	dsi_bus_lock(dssdev);

	nt51012_power_off(dssdev);

	dsi_bus_unlock(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

	mutex_unlock(&nd->lock);

	return 0;
err:
	mutex_unlock(&nd->lock);
	return r;
}


static int nt51012_resume(struct omap_dss_device *dssdev)
{
	struct nt51012_data *nd = dev_get_drvdata(&dssdev->dev);
	int r;

	mutex_lock(&nd->lock);

	if(first_suspend)
	{
		dev_dbg(&dssdev->dev, "not resuming now\n");
		first_suspend = 0;
		mutex_unlock(&nd->lock);
		return 0;
	}
	dev_dbg(&dssdev->dev, "resume\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED) {
		r = -EINVAL;
		goto err;
	}

	dsi_bus_lock(dssdev);

	r = nt51012_power_on(dssdev);

	dsi_bus_unlock(dssdev);

	if (r) {
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	} else {
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	}

	mutex_unlock(&nd->lock);

	return r;
err:
	mutex_unlock(&nd->lock);
	return r;
}

static struct omap_dss_driver nt51012_driver = {
	.probe		= nt51012_probe,
	.remove		= nt51012_remove,

	.enable		= nt51012_enable,
	.disable	= nt51012_disable,
	.suspend	= nt51012_suspend,
	.resume		= nt51012_resume,

	.get_resolution	= nt51012_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	.get_timings	= nt51012_get_timings,
	.set_timings	= nt51012_set_timings,
	.check_timings	= nt51012_check_timings,
	.reset_panel	= nt51012_reset_isr,

	.driver         = {
		.name   = "nt51012",
		.owner  = THIS_MODULE,
	},
};

static int __init nt51012_init(void)
{
	int r;

	r = omap_dss_register_driver(&nt51012_driver);
	if (r < 0) {
		printk("nt51012 driver registration failed\n");
		return r;
	}

	return 0;
}

static void __exit nt51012_exit(void)
{
	omap_dss_unregister_driver(&nt51012_driver);
}

module_init(nt51012_init);
module_exit(nt51012_exit);

MODULE_AUTHOR("Subbaraman Narayanamurthy <x0164410@ti.com>");
MODULE_DESCRIPTION("NT51012 MIPI DSI Driver");
MODULE_LICENSE("GPL");
