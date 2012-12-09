/*
 * Toshiba TC358765 DSI-to-LVDS chip driver
 *
 * Copyright (C) Texas Instruments
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
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

#include "panel-tc358765.h"

static struct omap_video_timings tc358765_timings = {
	.x_res		= TC358765_WIDTH,
	.y_res		= TC358765_HEIGHT,
	.pixel_clock	= TC358765_PCLK,
	.hfp		= TC358765_HFP,
	.hsw            = TC358765_HSW,
	.hbp            = TC358765_HBP,
	.vfp            = TC358765_VFP,
	.vsw            = TC358765_VSW,
	.vbp            = TC358765_VBP,
};

/* device private data structure */
struct tc358765_data {
	struct mutex lock;

	struct omap_dss_device *dssdev;

	int channel0;
	int channel1;
};

static struct panel_board_data *get_board_data(struct omap_dss_device *dssdev)
{
	return (struct panel_board_data *)dssdev->data;
}

static int tc358765_read_register(struct omap_dss_device *dssdev, u16 reg)
{
	struct tc358765_data *d2d = dev_get_drvdata(&dssdev->dev);
	u8 buf[4];
	u32 val;
	int r;

	r = dsi_vc_gen_read_2(dssdev, d2d->channel1, reg, buf, 4);
	if (r < 0) {
		dev_err(&dssdev->dev, "gen read failed\n");
		return r;
	}

	val = buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24);
	dev_dbg(&dssdev->dev, "reg read %x, val=%08x\n", reg, val);
	return 0;
}

static int tc358765_write_register(struct omap_dss_device *dssdev, u16 reg,
		u32 value)
{
	struct tc358765_data *d2d = dev_get_drvdata(&dssdev->dev);
	u8 buf[6];
	int r;

	buf[0] = (reg >> 0) & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	buf[2] = (value >> 0) & 0xff;
	buf[3] = (value >> 8) & 0xff;
	buf[4] = (value >> 16) & 0xff;
	buf[5] = (value >> 24) & 0xff;

	r = dsi_vc_gen_write_nosync(dssdev, d2d->channel1, buf, 6);
	if (r)
		dev_err(&dssdev->dev, "reg write reg(%x) val(%x) failed: %d\n",
			       reg, value, r);
	return r;
}



static void tc358765_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static void tc358765_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
}

static int tc358765_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	if (tc358765_timings.x_res != timings->x_res ||
			tc358765_timings.y_res != timings->y_res ||
			tc358765_timings.pixel_clock != timings->pixel_clock ||
			tc358765_timings.hsw != timings->hsw ||
			tc358765_timings.hfp != timings->hfp ||
			tc358765_timings.hbp != timings->hbp ||
			tc358765_timings.vsw != timings->vsw ||
			tc358765_timings.vfp != timings->vfp ||
			tc358765_timings.vbp != timings->vbp)
		return -EINVAL;

	return 0;
}

static void tc358765_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
	*xres = tc358765_timings.x_res;
	*yres = tc358765_timings.y_res;
}

static int tc358765_hw_reset(struct omap_dss_device *dssdev)
{
	struct panel_board_data *board_data = get_board_data(dssdev);

	if (board_data == NULL || board_data->reset_gpio == -1)
		return 0;

	gpio_set_value(board_data->reset_gpio, 1);
	udelay(100);
	/* reset the panel */
	gpio_set_value(board_data->reset_gpio, 0);
	/* assert reset */
	udelay(100);
	gpio_set_value(board_data->reset_gpio, 1);

	/* wait after releasing reset */
	msleep(100);

	return 0;
}

static int tc358765_lcd_enable(struct omap_dss_device *dssdev)
{
	struct panel_board_data *board_data = get_board_data(dssdev);

	if (board_data->lcd_en_gpio == -1)
		return -1;

	gpio_set_value(board_data->lcd_en_gpio, 1);
	dev_dbg(&dssdev->dev, "tc358765_lcd_enable\n");

	return 0;
}

static int tc358765_lcd_disable(struct omap_dss_device *dssdev)
{
	struct panel_board_data *board_data = get_board_data(dssdev);

	if (board_data->lcd_en_gpio == -1)
		return -1;

	gpio_set_value(board_data->lcd_en_gpio, 0);
	dev_dbg(&dssdev->dev, "tc358765_lcd_disable\n");

	return 0;
}

static int tc358765_probe(struct omap_dss_device *dssdev)
{
	struct tc358765_data *d2d;
	int r = 0;

	dev_dbg(&dssdev->dev, "tc358765_probe\n");

	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.timings = tc358765_timings;
#if defined (CONFIG_PANEL_SAMSUNG_LTL089CL01)
	dssdev->ctrl.pixel_size = 18;
#else
	dssdev->ctrl.pixel_size = 24;
#endif
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

	dev_dbg(&dssdev->dev, "tc358765_probe done\n");

	/* do I need an err and kfree(d2d) */
	return r;
}

static void tc358765_remove(struct omap_dss_device *dssdev)
{
	struct tc358765_data *d2d = dev_get_drvdata(&dssdev->dev);

	omap_dsi_release_vc(dssdev, d2d->channel0);
	omap_dsi_release_vc(dssdev, d2d->channel1);

	kfree(d2d);
}

static struct
{
	u16 reg;
	u32 data;
} tc358765_init_seq[] = {
	/* this register setting is required only if host wishes to
	 * perform DSI read transactions
	 */
#if defined (CONFIG_PANEL_SAMSUNG_LTL089CL01)
	{ PPI_TX_RX_TA, 0x0008000B },
	/* SYSLPTX Timing Generation Counter */
	{ PPI_LPTXTIMECNT, 0x00000007 },
	/* D*S_CLRSIPOCOUNT = [(THS-SETTLE + THS-ZERO) / HS_byte_clock_period ] */
	{ PPI_D0S_CLRSIPOCOUNT, 0x00000007 },
	{ PPI_D1S_CLRSIPOCOUNT, 0x00000007 },
	{ PPI_D2S_CLRSIPOCOUNT, 0x00000007 },
	{ PPI_D3S_CLRSIPOCOUNT, 0x00000007 },
#else
	/* SYSLPTX Timing Generation Counter */
	{ PPI_LPTXTIMECNT, 0x00000004 },
	/* D*S_CLRSIPOCOUNT = [(THS-SETTLE + THS-ZERO) / HS_byte_clock_period ] */
	{ PPI_D0S_CLRSIPOCOUNT, 0x00000003 },
	{ PPI_D1S_CLRSIPOCOUNT, 0x00000003 },
	{ PPI_D2S_CLRSIPOCOUNT, 0x00000003 },
	{ PPI_D3S_CLRSIPOCOUNT, 0x00000003 },
#endif
	/* SpeedLaneSel == HS4L */
	{ DSI_LANEENABLE, 0x0000001F },
	/* SpeedLaneSel == HS4L */
	{ PPI_LANEENABLE, 0x0000001F },
	/* Changed to 1 */
	{ PPI_STARTPPI, 0x00000001 },
	/* Changed to 1 */
	{ DSI_STARTDSI, 0x00000001 },

#if defined (CONFIG_PANEL_SAMSUNG_LTL089CL01)
	{ LVPHY0, 0x00448006 },
	{ LVPHY0, 0x00048006 },
	{ LVMX0003, 0x05040302 },
	{ LVMX0407, 0x0A070106 },
	{ LVMX0811, 0x09080C0B },
	{ LVMX1215, 0x120F0E0D },
	{ LVMX1619, 0x14131110 },
	{ LVMX2023, 0x1B171615 },
	{ LVMX2427, 0x001A1918 },
	{ VPCTRL, 0x00000021 },
	{ HTIM1, 0x00F80006 },
	{ VTIM1, 0x00060006 },
	{ LVCFG, 0x00000103 },
#else
	/* configure D2L on-chip PLL */
	{ LVPHY1, 0x00000000 },
	/* set frequency range allowed and clock/data lanes */
	{ LVPHY0, 0x00044006 },

	/* configure D2L chip LCD Controller configuration registers */
	{ LVMX0003, 0x03020100 },
	{ LVMX0407, 0x08050704 },
	{ LVMX0811, 0x0F0E0A09 },
	{ LVMX1215, 0x100D0C0B },
	{ LVMX1619, 0x12111716 },
	{ LVMX2023, 0x1B151413 },
	{ LVMX2427, 0x061A1918 },
	{ VPCTRL, 0x00000120 },
	{ HTIM1, ((TC358765_HBP << 16) | TC358765_HSW)},
	{ HTIM2, ((TC358765_HFP << 16) | TC358765_WIDTH)},
	{ VTIM1, ((TC358765_VBP << 16) | TC358765_VSW)},
	{ VTIM2, ((TC358765_VFP << 16) | TC358765_HEIGHT)},
	{ LVCFG, 0x00000001 },
#endif

	/* Issue a soft reset to LCD Controller for a clean start */
	{ SYSRST, 0x00000004 },
	{ VFUEN, 0x00000001 },
};

static int tc358765_write_init_config(struct omap_dss_device *dssdev)
{
	int i;
	int r;

	for (i = 0; i < ARRAY_SIZE(tc358765_init_seq); ++i) {
		u16 reg = tc358765_init_seq[i].reg;
		u32 data = tc358765_init_seq[i].data;

		r = tc358765_write_register(dssdev, reg, data);
		if (r) {
			dev_err(&dssdev->dev,
					"failed to write initial config (write) %d\n", i);
			return r;
		}

#if defined (CONFIG_PANEL_SAMSUNG_LTL089CL01)
		if(reg == LVPHY0)
			mdelay(1);
#endif

	}

#if !defined (CONFIG_PANEL_SAMSUNG_LTL089CL01)
	tc358765_write_register(dssdev, HTIM2,
		(TC358765_HFP << 16) | board_data->x_res);
	tc358765_write_register(dssdev, VTIM2,
		(TC358765_VFP << 16) | board_data->y_res);
#endif

	return 0;
}

static int tc358765_power_on(struct omap_dss_device *dssdev)
{
	struct tc358765_data *d2d = dev_get_drvdata(&dssdev->dev);
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
	r = tc358765_lcd_enable(dssdev);
	if(r) {
		dev_err(&dssdev->dev, "failed to enable LCD\n");
		goto err_write_init;
	}

	/* reset tc358765 bridge */
	tc358765_hw_reset(dssdev);

	/* do extra job to match kozio registers (???) */
	dsi_videomode_panel_preinit(dssdev);

	/* Need to wait a certain time - Toshiba Bridge Constraint */
	/* msleep(400); */

	/* configure D2L chip DSI-RX configuration registers */
	r = tc358765_write_init_config(dssdev);
	if (r)
		goto err_write_init;

	omapdss_dsi_vc_enable_hs(dssdev, d2d->channel1, true);

	/* 0x0e - 16bit
	 * 0x1e - packed 18bit
	 * 0x2e - unpacked 18bit
	 * 0x3e - 24bit
	 */
#if defined (CONFIG_PANEL_SAMSUNG_LTL089CL01)
	dsi_video_mode_enable(dssdev, 0x1e);
#else
	dsi_video_mode_enable(dssdev, 0x3e);
#endif

	dev_dbg(&dssdev->dev, "power_on done\n");

	return r;

err_write_init:
	omapdss_dsi_display_disable(dssdev, false, false);
err_disp_enable:
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	return r;
}

static void tc358765_power_off(struct omap_dss_device *dssdev)
{
	int r;
	dsi_video_mode_disable(dssdev);

	omapdss_dsi_display_disable(dssdev, false, false);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	/* disable lcd */
	r = tc358765_lcd_disable(dssdev);
	if(r) {
		dev_err(&dssdev->dev, "failed to disable LCD\n");
	}
}

static void tc358765_disable(struct omap_dss_device *dssdev)
{
	struct tc358765_data *d2d = dev_get_drvdata(&dssdev->dev);

	dev_dbg(&dssdev->dev, "disable\n");

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
		mutex_lock(&d2d->lock);
		dsi_bus_lock(dssdev);

		tc358765_power_off(dssdev);

		dsi_bus_unlock(dssdev);
		mutex_unlock(&d2d->lock);
	}

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int tc358765_enable(struct omap_dss_device *dssdev)
{
	struct tc358765_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r = 0;

	dev_dbg(&dssdev->dev, "enable\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
		return -EINVAL;

	mutex_lock(&d2d->lock);
	dsi_bus_lock(dssdev);

	r = tc358765_power_on(dssdev);

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

static int tc358765_suspend(struct omap_dss_device *dssdev)
{
	/* Disable the panel and return 0;*/
	tc358765_disable(dssdev);
	return 0;
}

static struct omap_dss_driver tc358765_driver = {
	.probe		= tc358765_probe,
	.remove		= tc358765_remove,

	.enable		= tc358765_enable,
	.disable	= tc358765_disable,
	.suspend	= tc358765_suspend,
	.resume		= NULL,

	.get_resolution	= tc358765_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	.get_timings	= tc358765_get_timings,
	.set_timings	= tc358765_set_timings,
	.check_timings	= tc358765_check_timings,

	.driver         = {
		.name   = "tc358765",
		.owner  = THIS_MODULE,
	},
};

static int __init tc358765_init(void)
{
	int r;

	r = omap_dss_register_driver(&tc358765_driver);
	if (r < 0) {
		printk("tc358765 driver registration failed\n");
		return r;
	}

	return 0;
}

static void __exit tc358765_exit(void)
{
	omap_dss_unregister_driver(&tc358765_driver);
}

module_init(tc358765_init);
module_exit(tc358765_exit);

MODULE_AUTHOR("Tomi Valkeinen <tomi.valkeinen@ti.com>");
MODULE_DESCRIPTION("TC358765 DSI-2-LVDS Driver");
MODULE_LICENSE("GPL");
