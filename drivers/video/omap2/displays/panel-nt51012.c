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

#define DEBUG

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

	int channel0;
	int channel1;
};

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
	dev_dbg(&dssdev->dev, "nt51012_lcd_enable\n");

	return 0;
}

static int nt51012_lcd_disable(struct omap_dss_device *dssdev)
{
	struct panel_board_data *board_data = get_board_data(dssdev);

	if (board_data->lcd_en_gpio == -1)
		return -1;

	gpio_set_value(board_data->lcd_en_gpio, 0);
	dev_dbg(&dssdev->dev, "nt51012_lcd_disable\n");

	return 0;
}

static int nt51012_probe(struct omap_dss_device *dssdev)
{
	struct nt51012_data *d2d;
	int r = 0;

	dev_dbg(&dssdev->dev, "nt51012_probe\n");

	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.timings = nt51012_timings;
	dssdev->ctrl.pixel_size = 24;
	dssdev->panel.acbi = 0;
	dssdev->panel.acb = 40;

	d2d = kzalloc(sizeof(*d2d), GFP_KERNEL);
	if (!d2d) {
		r = -ENOMEM;
		return r;
	}

	d2d->dssdev = dssdev;

	mutex_init(&d2d->lock);

	dev_set_drvdata(&dssdev->dev, d2d);

	r = omap_dsi_request_vc(dssdev, &d2d->channel0);
	if (r)
		dev_err(&dssdev->dev, "failed to get virtual channel0\n");

	r = omap_dsi_set_vc_id(dssdev, d2d->channel0, 0);
	if (r)
		dev_err(&dssdev->dev, "failed to set VC_ID0\n");

	r = omap_dsi_request_vc(dssdev, &d2d->channel1);
	if (r)
		dev_err(&dssdev->dev, "failed to get virtual channel1\n");

	r = omap_dsi_set_vc_id(dssdev, d2d->channel1, 0);
	if (r)
		dev_err(&dssdev->dev, "failed to set VC_ID1\n");

	dev_dbg(&dssdev->dev, "nt51012_probe done\n");
	printk("nt51012_probe done\n");

	/* do I need an err and kfree(d2d) */
	return r;
}

static void nt51012_remove(struct omap_dss_device *dssdev)
{
	struct nt51012_data *d2d = dev_get_drvdata(&dssdev->dev);

	omap_dsi_release_vc(dssdev, d2d->channel0);
	omap_dsi_release_vc(dssdev, d2d->channel1);

	kfree(d2d);
}

static void nt51012_config(struct omap_dss_device *dssdev)
{

	//printk("+++++++++++++ nt51012_config +++++++++++++++\n");

	//soft reset
	//printk("software reset\n");
	dsi_vc_dcs_write_1(dssdev, 0, 0x01, 0x00);
	msleep(20);  //need more than 20ms

	//printk("bist mode\n");
	//dsi_vc_dcs_write_1(dssdev, 0, 0xB1, 0xEF);
        
	//printk("swing double mode\n");
	dsi_vc_dcs_write_1(dssdev, 0, 0xAE, 0x0D);

	//printk("test mode1\n");
	dsi_vc_dcs_write_1(dssdev, 0, 0xEE, 0xEA);
	//printk("test mode2\n");
	dsi_vc_dcs_write_1(dssdev, 0, 0xEF, 0x5F);
	//printk("bias\n");
	dsi_vc_dcs_write_1(dssdev, 0, 0xF2, 0x28);

	//ken add DSI PWM fixed at 70% duty cycle
	//need to modify as flexible PWM duty cycle switching
	printk("CABC Moving-Mode (Moving Image Mode) 70 percent\n"); 
	dsi_vc_dcs_write_1(dssdev, 0, 0xB0, 0x3E);
	//end
        
}

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

	omapdss_dsi_vc_enable_hs(dssdev, d2d->channel0, true);

	/* enable lcd */
	r = nt51012_lcd_enable(dssdev);
	if(r) {
		dev_err(&dssdev->dev, "failed to enable LCD\n");
		goto err_write_init;
	}

	/* Wait till TCON gets ready */
	msleep(100);

	/* do extra job to match kozio registers (???) */
	dsi_videomode_panel_preinit(dssdev);

	nt51012_config(dssdev);

	/* Need to wait a certain time - Toshiba Bridge Constraint */
	/* msleep(400); */

	omapdss_dsi_vc_enable_hs(dssdev, d2d->channel1, true);

	/* 0x0e - 16bit
	 * 0x1e - packed 18bit
	 * 0x2e - unpacked 18bit
	 * 0x3e - 24bit
	 */

	dsi_video_mode_enable(dssdev, 0x3e);

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
	dsi_video_mode_disable(dssdev);

	omapdss_dsi_display_disable(dssdev, false, false);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	/* disable lcd */
	r = nt51012_lcd_disable(dssdev);
	if(r) {
		dev_err(&dssdev->dev, "failed to disable LCD\n");
	}
}

static void nt51012_disable(struct omap_dss_device *dssdev)
{
	struct nt51012_data *d2d = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "disable\n");

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		mutex_lock(&d2d->lock);
		dsi_bus_lock(dssdev);

		nt51012_power_off(dssdev);

		dsi_bus_unlock(dssdev);
		mutex_unlock(&d2d->lock);
	}

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int nt51012_enable(struct omap_dss_device *dssdev)
{
	struct nt51012_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r = 0;

	dev_dbg(&dssdev->dev, "enable\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
		return -EINVAL;

	mutex_lock(&d2d->lock);
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

static int nt51012_suspend(struct omap_dss_device *dssdev)
{
	/* Disable the panel and return 0;*/
	nt51012_disable(dssdev);
	return 0;
}

static struct omap_dss_driver nt51012_driver = {
	.probe		= nt51012_probe,
	.remove		= nt51012_remove,

	.enable		= nt51012_enable,
	.disable	= nt51012_disable,
	.suspend	= nt51012_suspend,
	.resume		= NULL,

	.get_resolution	= nt51012_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	.get_timings	= nt51012_get_timings,
	.set_timings	= nt51012_set_timings,
	.check_timings	= nt51012_check_timings,

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
