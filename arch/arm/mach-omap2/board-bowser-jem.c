/*
 * Board support file for OMAP4430 SDP.
 *
 * Copyright (C) 2009 Texas Instruments
 *
 * Author: Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * Based on mach-omap2/board-3430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h>
#include <linux/usb/otg.h>
#include <linux/spi/spi.h>
#include <linux/hwspinlock.h>
#include <linux/i2c/twl.h>
#include <linux/i2c/twl6030-gpadc.h>
#include <linux/gpio_keys.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>
#include <linux/omapfb.h>
#include <linux/twl6040-vib.h>
#include <linux/memblock.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#include <linux/bwan.h>
#include <linux/qmi.h>
#include <linux/delay.h>

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <mach/emif.h>
#include <mach/lpddr2-elpida.h>
#include <mach/dmm.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/omap4-keypad.h>
#include <plat/omap_apps_brd_id.h>
#include <plat/omap-serial.h>
#include <plat/remoteproc.h>
#include <video/omapdss.h>
#include <video/omap-panel-nokia-dsi.h>
#include <plat/vram.h>
#include <plat/omap-pm.h>
#include <plat/board-bowser-bluetooth.h>

#include <mach/bowser_idme_init.h>

/*SW5, Jamestsai, 1213, enable cypress/atmel{*/
#if defined(CONFIG_TOUCHSCREEN_CYPRESS_TTSP)
    #include <linux/input/touch_platform.h>
#endif
#if defined(CONFIG_TOUCHSCREEN_ATMEL_MXT)
    #include <linux/i2c/atmel_mxt_ts.h>
#endif
/*}SW5, Jamestsai, 1213, enable cypress/atmel*/

#include <sound/wm8962.h>

#include "board-bowser.h"
#include <mach/omap4_ion.h>
#include "mux.h"
#include "hsmmc.h"
#include "timer-gp.h"
#include "control.h"
#include "common-board-devices.h"
#include "pm.h"
#include "prm-regbits-44xx.h"
#include "prm44xx.h"
#include "omap_ram_console.h"

// Anvoi, 2011/12/14, Porting Light sensor driver to ICS
#ifdef CONFIG_INPUT_MAX44007
#include <linux/input/max44007.h>
#endif
// Anvoi, 2011/12/14, Porting Light sensor driver to ICS

/* JossCheng, 20111229, Rohm BU52061NVX hall sensor {*/
#ifdef CONFIG_INPUT_BU52061_HALLSENSOR
#include <linux/input/bu52061.h>
#endif
/* JossCheng, 20111229, Rohm BU52061NVX hall sensor }*/

#include <plat/omap-serial.h>
/* JossCheng, 20111221, Porting gyro sensor and e-compass {*/
#ifdef CONFIG_MPU_SENSORS_MPU6050B1
#include <linux/mpu.h>
#define GPIO_GRYO               4
#endif
/* JossCheng, 20111221, Porting gyro sensor and e-compass }*/
/* JossCheng, 20111229, Rohm BU52061NVX hall sensor {*/
#ifdef CONFIG_INPUT_BU52061_HALLSENSOR
#define HALL_EFFECT 0  // Hall sensor output pin -- gpio_wk0
#endif
/* JossCheng, 20111229, Rohm BU52061NVX hall sensor }*/

/* DeepakB DYNAMIC DDR Detection changes */
#define  SAMSUNG_SDRAM 		0x1
#define  ELPIDA_SDRAM  		0x3
#define  HYNIX_SDRAM		0x6
#define  SDRAM_DENSITY_MASK    0x3C
#define  SDRAM_DENSITY_2CS     0x14
#define  SDRAM_DENSITY_1CS     0x18

#define GPIO_WM8962_IRQ	        	26

/* PWM2 and TOGGLE3 register offsets */
#define LED_PWM2ON		0x03
#define LED_PWM2OFF		0x04
#define TWL6030_TOGGLE3		0x92

#define TPS62361_GPIO   7

// Anvoi, 2011/12/14, Porting Light sensor driver to ICS
#ifdef CONFIG_INPUT_MAX44007
#define GPIO_MAX44007_IRQ       36
#endif
// Anvoi, 2011/12/14, Porting Light sensor driver to ICS

#define GPIO_BTGPS_RST_N	53
#define GPIO_BTGPS_EN 		46
#define GPIO_BT_WAKE      	49
#define GPIO_BT_HOST_WAKE   82	// Connect to G01_HOST_WAKE of BCM2076

#define MMC_VDD_ANY       0x00FFFF80
/* 20111220, Robert Hu, for JEM SMB347 Charger { */
#if defined(CONFIG_CHARGER_SMB347)
#include <linux/power/smb347.h>
#define GPIO_SMB347_IRQ		2
#endif
/* 20111220, Robert Hu, for JEM SMB347 Charger } */
#define OMAP_HDMI_HPD_ADDR	0x4A100098
#define OMAP_HDMI_PULLTYPE_MASK	0x00000010


#define GPADC_CHANNEL_TOUCH_VENDOR  4  /* use gpadc channel 4 for touch vendor detection */

#define USB_EXT_PWR_EN 61
#define USB_NRESET 62

/* WAN specific GPIOs */
#define GPIO_WAN_FW_RDY		16	/* WAN_FW_RDY */
#define GPIO_WAN_SIM_PRESENT	17	/* SIM_PRESENT */
#define GPIO_WAN_ON		18	/* WAN_ON_OFF */
#define GPIO_WAN_SHUTDOWN	19	/* WAN_RST_N */
#define GPIO_WAN_USB_EN		20	/* WAN_USB_EN */

#define GPIO_REMOTE_WAKEUP	3	/* USB remote wakeup */

/*SW5, Anvoi, 20111215, Config key VolumeUp/VolumeDown{*/				
/* GPIO_KEY for Bowser */
/* Config VolumeUp : GPIO_WK1 , VolumeDown : GPIO_50 */
static struct gpio_keys_button bowser_gpio_keys_buttons[] = {
	[0] = {
		.code			= KEY_VOLUMEUP,
		.gpio			= 1,
		.desc			= "SW1",
		.active_low		= 1,
		.wakeup			= 0,
		.debounce_interval	= 5,
	},
	[1] = {
		.code			= KEY_VOLUMEDOWN,
		.gpio			= 50,
		.desc			= "SW3",
		.active_low		= 1,
		.wakeup			= 0,
		.debounce_interval	= 0,
		},
	};

static struct gpio_keys_platform_data bowser_gpio_keys = {
	.buttons		= bowser_gpio_keys_buttons,
	.nbuttons		= ARRAY_SIZE(bowser_gpio_keys_buttons),
	.rep			= 0,
};

static struct platform_device bowser_gpio_keys_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.dev		= {
		.platform_data	= &bowser_gpio_keys,
	},
};

static struct bwan_platform_data bowser_wan_data = {
	.init			=	NULL,
	.wan_on_gpio		=	GPIO_WAN_ON,
	.wan_shutdown_gpio	=	GPIO_WAN_SHUTDOWN,
	.usb_en_gpio		=	GPIO_WAN_USB_EN,
	.fw_rdy_gpio		=	GPIO_WAN_FW_RDY,
	.sim_present_gpio	=	GPIO_WAN_SIM_PRESENT,
};

static struct platform_device bowser_wan = {
	.name		=	"bwan",
	.id		=	-1,
	.dev		=	{
					.platform_data = &bowser_wan_data,
	},
};

static struct qmi_platform_data qmi_data = {
	.init			=	NULL,
	.remote_wakeup_gpio	=	GPIO_REMOTE_WAKEUP,
};

static struct platform_device qmi = {
	.name		=	"qmi",
	.id		=	-1,
	.dev		=	{
					.platform_data = &qmi_data,
	},
};

/*SW5, Anvoi, 20111215, Config key VolumeUp/VolumeDown }*/
#define OMAP4SDP_MDM_PWR_EN_GPIO	157

// BokeeLi, 2011/12/14, Porting proximity driver
#ifdef CONFIG_INPUT_PIC12LF1822_PROXIMITY
#define GPIO_PIC12LF1822_PROXIMITY_IRQ	85
#define PIC12LF1822_PROXIMITY_NAME	"PIC12LF1822_Prox"
#endif
// BokeeLi, 2011/12/14, Porting proximity driver
/*SW5, Jamestsai, 1213, enable cypress{*/
/* same for both Atmel and Cypress */
/* Note these must be same as in the cypress file board-touch-cyttsp4_core.c */
#if defined(CONFIG_TOUCHSCREEN_CYPRESS_TTSP) || (CONFIG_TOUCHSCREEN_ATMEL_MXT)
#define GPIO_TOUCH_RESET 23
#define GPIO_TOUCH_IRQ   24
#endif
/*}SW5, Jamestsai, 1213, enable cypress*/

static struct twl4030_madc_platform_data twl6030_madc = {
	.irq_line = -1,
	.features = TWL6032_SUBCLASS,
};

static struct platform_device twl6030_madc_device = {
	.name   = "twl6030_madc",
	.id = -1,
	.dev	= {
		.platform_data	= &twl6030_madc,
	},
};

/* JossCheng, 20111229, Rohm BU52061NVX hall sensor {*/
#ifdef CONFIG_INPUT_BU52061_HALLSENSOR
static int BU52061_init_irq(void)
{
        int ret = 0;
        printk("BU52061_init_irq\n");
        ret = gpio_request(HALL_EFFECT, "hall_sensor_status");
        if (ret) {
               printk(KERN_ERR"BU52061 gpio_request failed, return: %d\n", ret);
               goto err_request_gpio;
        }

        ret = gpio_direction_input(HALL_EFFECT);
        if (ret) {
                printk(KERN_ERR"set BU52061_irq gpio's direction failed, return: %d\n", ret);
				goto err_request_gpio;
        }
		
		return ret;
err_request_gpio:
        gpio_free(HALL_EFFECT);
        return ret;
}

static struct _bu52061_platform_data bu52061_data = {
        .init_irq       = BU52061_init_irq,
        .irq = OMAP_GPIO_IRQ(HALL_EFFECT),
};

static struct platform_device bu52061_platform_device = {
        .name = "bu52061",
        .id       = -1,
        .dev  = {
                .platform_data = &bu52061_data,
        },
};
#endif
/* JossCheng, 20111229, Rohm BU52061NVX hall sensor }*/

static struct platform_device *bowser_devices[] __initdata = {
/*SW5, Anvoi, 20111215, Config key VolumeUp/VolumeDown{*/
	&bowser_gpio_keys_device,
	&bowser_wan,
	&qmi,
/*SW5, Anvoi, 20111215, Config key VolumeUp/VolumeDown{*/
/* JossCheng, 20111229, Rohm BU52061NVX hall sensor {*/
#ifdef CONFIG_INPUT_BU52061_HALLSENSOR
	&bu52061_platform_device,

#endif
/* JossCheng, 20111229, Rohm BU52061NVX hall sensor }*/
};

static struct omap_board_config_kernel sdp4430_config[] __initdata = {
};

static void __init omap_4430sdp_init_early(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(1);
#endif
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_UTMI,
	.mode			= MUSB_OTG,
	.power			= 100,
};

static struct twl4030_usb_data omap4_usbphy_data = {
	.phy_init	= omap4430_phy_init,
	.phy_exit	= omap4430_phy_exit,
	.phy_power	= omap4430_phy_power,
	.phy_set_clock	= omap4430_phy_set_clk,
	.phy_suspend	= omap4430_phy_suspend,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 2,
		.caps		=  MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA | MMC_CAP_1_8V_DDR,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable   = true,
		.ocr_mask	= MMC_VDD_29_30,
		.no_off_init	= true,
	},
	{
		.name           = "bcmdhd_wlan",
		.mmc		= 5,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA, // | MMC_CAP_1_8V_DDR,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195 | MMC_VDD_20_21,
		.nonremovable	= false,
		.mmc_data	= &bowser_wifi_data,
	},
	{}	/* Terminator */
};

static struct regulator_consumer_supply sdp4430_vaux_supply[] = {
	{
		.supply = "vmmc",
		.dev_name = "omap_hsmmc.1",
	},
};
static struct regulator_consumer_supply sdp4430_vmmc_supply[] = {
	{
		.supply = "vmmc",
		.dev_name = "omap_hsmmc.0",
	},
};
static struct regulator_consumer_supply sdp4430_vcxio_supply[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi1"),
};

static struct regulator_consumer_supply sdp4430_cam2_supply[] = {
	{
		.supply = "cam2pwr",
	},
};

static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		ret = twl6030_mmc_card_detect_config();
		if (ret)
			pr_err("Failed configuring MMC1 card detect\n");
		pdata->slots[0].card_detect_irq = TWL6030_IRQ_BASE +
						MMCDETECT_INTR_OFFSET;
		pdata->slots[0].card_detect = twl6030_mmc_card_detect;
	}

	return ret;
}

static __init void omap4_twl6030_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev) {
		pr_err("Failed %s\n", __func__);
		return;
	}
	pdata = dev->platform_data;
	pdata->init =	omap4_twl6030_hsmmc_late_init;
}

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	struct omap2_hsmmc_info *c;

	omap2_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++)
		omap4_twl6030_hsmmc_set_late_init(c->dev);

	return 0;
}

static struct regulator_init_data sdp4430_vaux1 = {
	.constraints = {
		.min_uV			= 1000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on	= true,
		.state_mem = {
				.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = sdp4430_vaux_supply,
};

static struct regulator_consumer_supply bowser_ldo4_supply[] = {
        REGULATOR_SUPPLY("av-switch", "soc-audio"),
        {
                .supply = "ldo4",
        },
};

static struct regulator_init_data sdp4430_vaux2 = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
				.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
	.num_consumer_supplies  = 2,
	.consumer_supplies      = bowser_ldo4_supply,

};

static struct regulator_init_data sdp4430_vaux3 = {
	.constraints = {
		.min_uV			= 2800000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
				.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = sdp4430_cam2_supply,
};

/* VMMC1 for MMC1 card */
static struct regulator_init_data sdp4430_vmmc = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
				.enabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = sdp4430_vmmc_supply,
};

static struct regulator_init_data sdp4430_vpp = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 2500000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
				.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
};

#if defined(CONFIG_CHARGER_SMB347)
static struct regulator_consumer_supply bowser_ldo7_supply[] = {
	{
		.supply = "smb347_ldo7",
	},
};
#endif

static struct regulator_init_data sdp4430_vusim = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 2900000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
				.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
#if defined(CONFIG_CHARGER_SMB347)
	.num_consumer_supplies  = ARRAY_SIZE(bowser_ldo7_supply),
	.consumer_supplies      = bowser_ldo7_supply,
#endif
};

static struct regulator_init_data sdp4430_vana = {
	.constraints = {
		.min_uV			= 2100000,
		.max_uV			= 2100000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on	= true,
		.state_mem = {
				.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data sdp4430_vcxio = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.always_on	= true,
	},
	.num_consumer_supplies	= ARRAY_SIZE(sdp4430_vcxio_supply),
	.consumer_supplies	= sdp4430_vcxio_supply,
};

static struct regulator_consumer_supply sdp4430_vdac_supply[] = {
	{
		.supply = "hdmi_vref",
	},
};

static struct regulator_init_data sdp4430_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = ARRAY_SIZE(sdp4430_vdac_supply),
	.consumer_supplies      = sdp4430_vdac_supply,
};

static struct regulator_init_data sdp4430_vusb = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 =	REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
		.state_mem = {
				.disabled	= true,
		},
		.initial_state		= PM_SUSPEND_MEM,
	},
};

static struct regulator_init_data sdp4430_clk32kg = {
	.constraints = {
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.always_on		= true,
	},
};

static void omap4_audio_conf(void)
{
	/* twl6040 naudint */
	omap_mux_init_signal("sys_nirq2.sys_nirq2", \
		OMAP_PIN_INPUT_PULLUP);
}

static struct wm8962_pdata wm8962_pdata = {
	.gpio_init = {
		[1] = WM8962_GPIO_FN_IRQ,              /* GPIO2 is IRQ */
		[4] = 0x8000 | WM8962_GPIO_FN_DMICDAT, /* GPIO5 DMICDAT */
		[5] = WM8962_GPIO_FN_DMICCLK,          /* GPIO6 DMICCLK */
	},
	.mic_cfg = 1, /*MICBIAS high, threasholds minimum*/
};

static struct twl4030_codec_audio_data twl6040_audio = {
	/* single-step ramp for headset and handsfree */
	.hs_left_step	= 0x0f,
	.hs_right_step	= 0x0f,
	.hf_left_step	= 0x1d,
	.hf_right_step	= 0x1d,
};

static struct twl4030_codec_vibra_data twl6040_vibra = {
	.max_timeout	= 15000,
	.initial_vibrate = 0,
};

static struct twl4030_codec_data twl6040_codec = {
	.audio		= &twl6040_audio,
	.vibra		= &twl6040_vibra,
	.audpwron_gpio	= 127,
	.naudint_irq	= OMAP44XX_IRQ_SYS_2N,
	.irq_base	= TWL6040_CODEC_IRQ_BASE,
};

static struct twl4030_platform_data sdp4430_twldata = {
	.irq_base	= TWL6030_IRQ_BASE,
	.irq_end	= TWL6030_IRQ_END,

	/* TWL6030 regulators at OMAP443X/446X based SOMs */
	.vmmc		= &sdp4430_vmmc,
	.vpp		= &sdp4430_vpp,
	.vusim		= &sdp4430_vusim,
	.vana		= &sdp4430_vana,
	.vcxio		= &sdp4430_vcxio,
	.vdac		= &sdp4430_vdac,
	.vusb		= &sdp4430_vusb,
	.vaux1		= &sdp4430_vaux1,
	.vaux2		= &sdp4430_vaux2,
	.vaux3		= &sdp4430_vaux3,

	/* TWL6032 regulators at OMAP447X based SOMs */
	.ldo1		= &sdp4430_vpp,
	.ldo2		= &sdp4430_vaux1,
	.ldo3		= &sdp4430_vaux3,
	.ldo4		= &sdp4430_vaux2,
	.ldo5		= &sdp4430_vmmc,
	.ldo6		= &sdp4430_vcxio,
	.ldo7		= &sdp4430_vusim,
	.ldoln		= &sdp4430_vdac,
	.ldousb		= &sdp4430_vusb,

	/* TWL6030/6032 common resources */	
	.clk32kg	= &sdp4430_clk32kg,

	/* children */	
	.usb		= &omap4_usbphy_data,
	.madc		= &twl6030_madc,
};

// Anvoi, 2011/12/14, Porting Light sensor driver to ICS
#ifdef CONFIG_INPUT_MAX44007
static struct MAX44007PlatformData max44007_pdata = {
        .placeHolder = 0x1234,
};
#endif
// Anvoi, 2011/12/14, Porting Light sensor driver to ICS

/*SW5, Jamestsai, 1213, enable cypress/atmel{*/
#if defined(CONFIG_TOUCHSCREEN_ATMEL_MXT)


int mxt_get_sensor_vendor(void)
{
        int val = -1;

#if 1
	struct twl6030_gpadc_request req;

	req.channels = (1 << GPADC_CHANNEL_TOUCH_VENDOR);
	req.method = TWL6030_GPADC_SW2;
	req.active = 0;
	req.func_cb = NULL;
	val = twl6030_gpadc_conversion(&req);
        if (val < 0) { /* failure */
                printk("Atmel touch sensor vendor get gpadc value failure %d\n", val);
        }
	else {
		val = req.rbuf[GPADC_CHANNEL_TOUCH_VENDOR];
                printk("Atmel touch sensor vendor get gpadc value %d\n", val);
                if (val < 1700) return ATMEL_TOUCH_SENSOR_VENDOR_TPK; /* this value is for compatible with tate pre-EVT2.0 devices */
                else if (val < 1900) return ATMEL_TOUCH_SENSOR_VENDOR_TPK;
                else if (val < 2100) return ATMEL_TOUCH_SENSOR_VENDOR_WINTEK;
                else if (val < 2300) return ATMEL_TOUCH_SENSOR_VENDOR_CMI;
                else if (val < 2500) return ATMEL_TOUCH_SENSOR_VENDOR_HANNSTOUCH;
                else return -1;
        }
#else
	val = ATMEL_TOUCH_SENSOR_VENDOR_TPK;
#endif
        return val;
}


static struct mxt_platform_data __initdata lg_touch_data = { /* atmel */
        .x_line = 24,
        .y_line = 32,
        .x_size = 1024,
        .y_size = 768,
        .voltage = 0,
        .blen = 144,   /* gain value */
        .threshold = 65,
        .orient = MXT_ROTATED_90,
        .irqflags = IRQF_TRIGGER_FALLING,
};

static struct mxt_platform_data __initdata samsung_touch_data = { /* atmel */
        .x_line = 24,
        .y_line = 32,
        .x_size = 1920,
        .y_size = 1200,
        .voltage = 0,
        .blen = 144,   /* gain value */
        .threshold = 50,
        .orient = 0, // This is set to 0 so the touch driver don't switch x & y when reporting to upper layer.
        .irqflags = IRQF_TRIGGER_FALLING,
};

static struct mxt_platform_data __initdata novatek_touch_data = { /* for tate */
        .x_line = 24,
        .y_line = 32,
        .x_size = 800,
        .y_size = 1280,
        .voltage = 0,
        .blen = 144,   /* gain value */
        .threshold = 50,
        .orient = 0, // This is set to 0 so the touch driver don't switch x & y when reporting to upper layer.
        .irqflags = IRQF_TRIGGER_FALLING,
	.read_chg = NULL,
        .read_sensor_vendor = mxt_get_sensor_vendor,
};


#endif

#if defined(CONFIG_TOUCHSCREEN_CYPRESS_TTSP)
extern struct touch_platform_data cyttsp4_i2c_touch_platform_data;
#endif

static struct i2c_board_info __initdata sdp4430_i2c_2_boardinfo[] = {
#if defined(CONFIG_TOUCHSCREEN_ATMEL_MXT)
        {
#if defined(CONFIG_PANEL_SAMSUNG_LTL089CL01)
                I2C_BOARD_INFO("atmel_mxt_ts", 0x4c), /* touch IC is in display board */
                .irq = OMAP_GPIO_IRQ(GPIO_TOUCH_IRQ),
                .platform_data = &samsung_touch_data,
#elif defined(CONFIG_PANEL_NT51012_LG)        /* tate */
                I2C_BOARD_INFO("atmel_mxt_ts", 0x4c),
                .irq = OMAP_GPIO_IRQ(GPIO_TOUCH_IRQ),
                .platform_data = &novatek_touch_data,
#else // LG panel
                I2C_BOARD_INFO("atmel_mxt_ts", 0x4d),
                .irq = OMAP_GPIO_IRQ(GPIO_TOUCH_IRQ),
                .platform_data = &lg_touch_data,
#endif
        },
#endif

#if defined(CONFIG_TOUCHSCREEN_CYPRESS_TTSP)
        {
                I2C_BOARD_INFO(CY_I2C_NAME, CY_I2C_TCH_ADR),
                .irq = OMAP_GPIO_IRQ(GPIO_TOUCH_IRQ),
                .platform_data = &cyttsp4_i2c_touch_platform_data,
        },
#endif

};
/*}SW5, Jamestsai, 1213, enable cypress/atmel*/

/* JossCheng, 20111221, Porting gyro sensor and e-compass {*/
#ifdef CONFIG_MPU_SENSORS_MPU6050B1
static struct mpu_platform_data mpu_gyro_data = {
	.int_config  = 0x10,
	.orientation = {   0,  -1,  0,
			   1,  0,  0,
			   0,  0,  1 },
};
static struct ext_slave_platform_data mpu_compass_data = {
	.bus         = EXT_SLAVE_BUS_SECONDARY,
	.orientation = { 0, -1, 0,
			 1, 0, 0,
			 0, 0, 1 },
};

static void mpu6050b1_init(void)
{
	gpio_request(GPIO_GRYO, "MPUIRQ");
	gpio_direction_input(GPIO_GRYO);
}
#endif
/* JossCheng, 20111221, Porting gyro sensor and e-compass }*/

#if defined(CONFIG_CHARGER_SMB347)
static struct smb347_platform_data smb347_pdata = {
	.regulator_name = "smb347_ldo7",
};
#endif

static struct i2c_board_info __initdata sdp4430_i2c_3_boardinfo[] = {

/* 20111214, Robert Hu, for JEM gasgauge { */
#if defined(CONFIG_BATTERY_BQ27541)
    {
        I2C_BOARD_INFO("bq27541", 0x55),
    },
#endif
/* 20111214, Robert Hu, for JEM gasgauge } */ 

/* 20111220, Robert Hu, for JEM SMB347 Charger { */
#if defined(CONFIG_CHARGER_SMB347)
	{
		I2C_BOARD_INFO("smb347", 0x5F),
		.platform_data = &smb347_pdata,
		.irq = OMAP_GPIO_IRQ(GPIO_SMB347_IRQ),
	},
#endif
/* 20111220, Robert Hu, for JEM SMB347 Charger } */   
        {
                I2C_BOARD_INFO("wm8962", 0x1a),
                .platform_data = &wm8962_pdata,
                .irq = OMAP_GPIO_IRQ(GPIO_WM8962_IRQ),    /* GPIO26 */
        },
};

static struct i2c_board_info __initdata sdp4430_i2c_4_boardinfo[] = {

// BokeeLi, 2011/12/14, Porting proximity driver
#ifdef CONFIG_INPUT_PIC12LF1822_PROXIMITY
	{
		I2C_BOARD_INFO(PIC12LF1822_PROXIMITY_NAME, 0x0d),
//		.platform_data = &pic12lf1822_proximity_pdata,
		.irq = OMAP_GPIO_IRQ(GPIO_PIC12LF1822_PROXIMITY_IRQ), /* GPIO85 */
	},
#endif
// BokeeLi, 2011/12/14, Porting proximity driver

/* JossCheng, 20111221, Porting gyro sensor and e-compass {*/
#ifdef CONFIG_MPU_SENSORS_MPU6050B1
	{
		I2C_BOARD_INFO("mpu6050", 0x68),
		.irq = OMAP_GPIO_IRQ(GPIO_GRYO),
		.platform_data = &mpu_gyro_data,
	},
	{
		I2C_BOARD_INFO("ami306", 0x0F),
		.irq = OMAP_GPIO_IRQ(GPIO_GRYO),
		.platform_data = &mpu_compass_data,
	},
#endif
/* JossCheng, 20111221, Porting gyro sensor and e-compass }*/


// Anvoi, 2011/12/14, Porting Light sensor driver to ICS

#ifdef CONFIG_INPUT_MAX44007
        {
                I2C_BOARD_INFO(MAX44007_NAME, 0x4A),
                .platform_data = &max44007_pdata,
                .irq = OMAP_GPIO_IRQ(GPIO_MAX44007_IRQ),    /* GPIO36 */
        },
#endif
// Anvoi, 2011/12/14, Porting Light sensor driver to ICS

#ifdef CONFIG_MAX1161X
	/* Maxim MAX11613 ADC */
	{
		I2C_BOARD_INFO("max11613", 0x34),
	},
#endif
};

// Anvoi, 2011/12/14, Porting Light sensor driver to ICS
static void omap4_als_init(void)
{
#ifdef CONFIG_INPUT_MAX44007
        /* max44007 gpio */
        printk("MAX44007: Pulling up IRQ line\n");
        omap_mux_init_gpio(GPIO_MAX44007_IRQ, \
                OMAP_PIN_INPUT_PULLUP);
#endif				
}
// Anvoi, 2011/12/14, Porting Light sensor driver to ICS


static void __init blaze_pmic_mux_init(void)
{

	omap_mux_init_signal("sys_nirq1", OMAP_PIN_INPUT_PULLUP |
						OMAP_WAKEUP_EN);
}

static void __init omap_i2c_hwspinlock_init(int bus_id, int spinlock_id,
				struct omap_i2c_bus_board_data *pdata)
{
	/* spinlock_id should be -1 for a generic lock request */
	if (spinlock_id < 0)
		pdata->handle = hwspin_lock_request();
	else
		pdata->handle = hwspin_lock_request_specific(spinlock_id);

	if (pdata->handle != NULL) {
		pdata->hwspin_lock_timeout = hwspin_lock_timeout;
		pdata->hwspin_unlock = hwspin_unlock;
	} else {
		pr_err("I2C hwspinlock request failed for bus %d\n", \
								bus_id);
	}
}

static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_1_bus_pdata;
static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_3_bus_pdata;
static struct omap_i2c_bus_board_data __initdata sdp4430_i2c_4_bus_pdata;

static int __init omap4_i2c_init(void)
{

	omap_i2c_hwspinlock_init(1, 0, &sdp4430_i2c_1_bus_pdata);
	omap_i2c_hwspinlock_init(2, 1, &sdp4430_i2c_2_bus_pdata);
	omap_i2c_hwspinlock_init(3, 2, &sdp4430_i2c_3_bus_pdata);
	omap_i2c_hwspinlock_init(4, 3, &sdp4430_i2c_4_bus_pdata);

	omap_register_i2c_bus_board_data(1, &sdp4430_i2c_1_bus_pdata);
	omap_register_i2c_bus_board_data(2, &sdp4430_i2c_2_bus_pdata);
	omap_register_i2c_bus_board_data(3, &sdp4430_i2c_3_bus_pdata);
	omap_register_i2c_bus_board_data(4, &sdp4430_i2c_4_bus_pdata);

	omap4_pmic_init("twl6030", &sdp4430_twldata);
	/*SW5, Jamestsai, 1213, enable cypress{*/
	//omap_register_i2c_bus(2, 400, NULL, 0);
	omap_register_i2c_bus(2, 400, sdp4430_i2c_2_boardinfo, 
				ARRAY_SIZE(sdp4430_i2c_2_boardinfo));
	/*}SW5, Jamestsai, 1213, enable cypress*/
        /* JossCheng, 20111221, change clock of i2c bus3 to be 400K Hz { */
	omap_register_i2c_bus(3, 400, sdp4430_i2c_3_boardinfo,
				ARRAY_SIZE(sdp4430_i2c_3_boardinfo));
        /* JossCheng, 20111221, change clock of i2c bus3 to be 400K Hz } */
	omap_register_i2c_bus(4, 400, sdp4430_i2c_4_boardinfo,
				ARRAY_SIZE(sdp4430_i2c_4_boardinfo));

#ifdef CONFIG_MACH_OMAP4_BOWSER_SUBTYPE_JEM
	omap2_i2c_pullup(3, I2C_PULLUP_STD_860_OM_FAST_500_OM);
#endif

	/*
	 * This will allow unused regulator to be shutdown. This flag
	 * should be set in the board file. Before regulators are registered.
	 */
	regulator_has_full_constraints();

	/*
	 * Drive MSECURE high for TWL6030/6032 write access.
	 */
	omap_mux_init_signal("fref_clk3_req.gpio_wk30", OMAP_PIN_OUTPUT); 
	gpio_request(30, "msecure");
	gpio_direction_output(30, 1);

	return 0;
}

static bool enable_suspend_off = true;
module_param(enable_suspend_off, bool, S_IRUSR | S_IRGRP | S_IROTH);

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {

//raviks added as per bowser3
#if 0
	/* WLAN IRQ - GPIO 53 */
	OMAP4_MUX(GPMC_NCS3, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* WLAN_EN - GPIO 43 */
	OMAP4_MUX(GPMC_A19, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* WLAN SDIO: MMC5 CMD */
	OMAP4_MUX(SDMMC5_CMD, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* WLAN SDIO: MMC5 CLK */
	OMAP4_MUX(SDMMC5_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* WLAN SDIO: MMC5 DAT[0-3] */
	OMAP4_MUX(SDMMC5_DAT0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
#endif

#ifdef CONFIG_INPUT_PIC12LF1822_PROXIMITY
	/* PROXIMITY IRQ - GPIO 85 */
	OMAP4_MUX(USBB1_ULPITLL_STP, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
#endif
#ifdef CONFIG_TOUCHSCREEN_ATMEL_MXT
	/* TOUCH IRQ - GPIO 24 */
	OMAP4_MUX(DPM_EMU13, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
#endif
	OMAP4_MUX(DPM_EMU18, OMAP_MUX_MODE1 | OMAP_PULL_ENA),/* LCDBKL_PWM, GPTimer 10 */
	{ .reg_offset = OMAP_MUX_TERMINATOR },

};

/*
 * LPDDR2 Configeration Data:
 * The memory organisation is as below :
 *	EMIF1 - CS0 -	2 Gb
 *		CS1 -	2 Gb
 *	EMIF2 - CS0 -	2 Gb
 *		CS1 -	2 Gb
 *	--------------------
 *	TOTAL -		8 Gb
 *
 * Same devices installed on EMIF1 and EMIF2
 */
static __initdata struct emif_device_details emif_devices_1cs = {
	.cs0_device = &lpddr2_elpida_4G_S4_dev,
	.cs1_device = 0
};

static __initdata struct emif_device_details emif_devices_2cs = {
	.cs0_device = &lpddr2_elpida_2G_S4_dev,
	.cs1_device = &lpddr2_elpida_2G_S4_dev
};

#else
#define board_mux	NULL
#define board_wkup_mux NULL
#endif

static struct omap_device_pad blaze_uart1_pads[] __initdata = {
	{
		.name	= "uart1_cts.uart1_cts",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart1_rts.uart1_rts",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart1_tx.uart1_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart1_rx.uart1_rx",
		.flags	= OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
		.idle	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
};


static struct omap_device_pad blaze_uart2_pads[] __initdata = {
	{
		.name	= "uart2_cts.uart2_cts",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,

	},
	{
		.name	= "uart2_rts.uart2_rts",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart2_tx.uart2_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart2_rx.uart2_rx",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
};

static struct omap_device_pad blaze_uart3_pads[] __initdata = {
	{
		.name	= "uart3_cts_rctx.uart3_cts_rctx",
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart3_rts_sd.uart3_rts_sd",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart3_tx_irtx.uart3_tx_irtx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart3_rx_irrx.uart3_rx_irrx",
		.flags	= OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
		.idle	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
};

static struct omap_device_pad blaze_uart4_pads[] __initdata = {
	{
		.name	= "uart4_tx.uart4_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart4_rx.uart4_rx",
		.flags	= OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
		.idle	= OMAP_PIN_INPUT | OMAP_MUX_MODE0,
	},
};

static struct omap_uart_port_info blaze_uart_info_uncon __initdata = {
	.use_dma	= 0,
	.auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
        .wer = 0,
};

static struct omap_uart_port_info blaze_uart2_info __initdata = {
	.use_dma	= 0,
	.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
	.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
	.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
	.auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
	.wer = (OMAP_UART_WER_TX | OMAP_UART_WER_RX | OMAP_UART_WER_CTS),
	.wake_peer      = bcm_bt_lpm_exit_lpm_locked,
};

static struct omap_uart_port_info blaze_uart3_info __initdata = {
	.use_dma	= 0,
	.auto_sus_timeout = DEFAULT_AUTOSUSPEND_DELAY,
	.wer = (OMAP_UART_WER_TX | OMAP_UART_WER_RX | OMAP_UART_WER_CTS),
};

static void SetupUartSuspend()
{
	if( idme_query_board_type( IDME_BOARD_TYPE_TATE_EVT2 ) )	
	{
		// disable uart suspend
		printk("uart auto suspend - disabled\n");
		blaze_uart2_info.auto_sus_timeout = -1;
	}
	else
	{
		printk("uart auto suspend - enabled\n");
	}
}

static inline void __init board_serial_init(void)
{
	// Check for board compatibility.
	SetupUartSuspend();

	omap_serial_init_port_pads(0, blaze_uart1_pads,
		ARRAY_SIZE(blaze_uart1_pads), &blaze_uart_info_uncon);
	omap_serial_init_port_pads(1, blaze_uart2_pads,
		ARRAY_SIZE(blaze_uart2_pads), &blaze_uart2_info);
	omap_serial_init_port_pads(2, blaze_uart3_pads,
		ARRAY_SIZE(blaze_uart3_pads), &blaze_uart3_info);
	omap_serial_init_port_pads(3, blaze_uart4_pads,
		ARRAY_SIZE(blaze_uart4_pads), &blaze_uart_info_uncon);
}

static void omap4_sdp4430_bt_mux_init(void)
{
	// Bluetooth
	omap_mux_init_gpio(GPIO_BT_HOST_WAKE,
			OMAP_PIN_INPUT | OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_WAKEUPENABLE | OMAP_WAKEUP_EVENT);
	omap_mux_init_gpio(GPIO_BTGPS_RST_N, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(GPIO_BTGPS_EN, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(GPIO_BT_WAKE, OMAP_PIN_OUTPUT);
}

int bowser_bluetooth_irq_num = -1;
EXPORT_SYMBOL(bowser_bluetooth_irq_num);

/* bcm2076 lpm support */
static struct platform_device *reg_pdev = NULL;
static struct platform_device bcm2076_bluetooth_device = {
	.name = "bcm2076_bluetooth",
	.id = -1,
};

static void omap4_sdp4430_bt_init(void)
{
	int ret = 0;
	omap4_sdp4430_bt_mux_init();

	/* BT + GPS related GPIOs */
	if (gpio_request(GPIO_BTGPS_RST_N, "bcm2076") ||
	    gpio_direction_output(GPIO_BTGPS_RST_N, 1))
		pr_err("Error in initializing Bluetooth chip reset gpio: Unable to set direction. \n");

	gpio_set_value(GPIO_BTGPS_RST_N, 1);

	if (gpio_request(GPIO_BTGPS_EN, "bcm2076") ||
	    gpio_direction_output(GPIO_BTGPS_EN, 1))
		pr_err("Error in initializing Bluetooth power gpio: Unable to set direction.\n");

	gpio_set_value(GPIO_BTGPS_EN, 1);

	// Disable Internal Pull up after the required 100msec delay after BTGPS_EN is asserted
	msleep(120);
	omap_mux_init_gpio(GPIO_BT_HOST_WAKE,
			OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE | OMAP_WAKEUP_EVENT);


	if (gpio_request(GPIO_BT_WAKE, "bcm2076") ||
	    gpio_direction_output(GPIO_BT_WAKE, 1))
		pr_err("Error in initializing Bluetooth reset gpio: unable to set direction.\n");

	gpio_set_value(GPIO_BT_WAKE, 1);

	ret = platform_device_register(&bcm2076_bluetooth_device);
	if (ret)
	{
		pr_err("could not register bcm2076_bluetooth_device!\n");
	}

	bowser_bluetooth_irq_num = OMAP_GPIO_IRQ(GPIO_BT_HOST_WAKE);
	printk("Configured GPIO_%d as bluetooth host wake interrupt, IRQ num = %d\n", GPIO_BT_HOST_WAKE, bowser_bluetooth_irq_num);
}

// BokeeLi, 2011/12/14, Porting proximity driver
#ifdef CONFIG_INPUT_PIC12LF1822_PROXIMITY
static void omap4_proximity_init(void)
{
	int error;

	/* pic12lf1822 gpio */
	printk("PIC12LF1822: Configuring IRQ line\n");
	omap_mux_init_gpio(GPIO_PIC12LF1822_PROXIMITY_IRQ, OMAP_PIN_INPUT_PULLUP);

	error = gpio_request(GPIO_PIC12LF1822_PROXIMITY_IRQ, "proximity");
	if (error < 0) {
		pr_err("%s:failed to request GPIO %d, error %d\n",
			__func__, GPIO_PIC12LF1822_PROXIMITY_IRQ, error);
		return;
	}

	error = gpio_direction_input(GPIO_PIC12LF1822_PROXIMITY_IRQ);
	if (error < 0) {
		pr_err("%s: GPIO configuration failed: GPIO %d,error %d\n",
			__func__, GPIO_PIC12LF1822_PROXIMITY_IRQ, error);
		gpio_free(GPIO_PIC12LF1822_PROXIMITY_IRQ);
	}
}
#endif
// BokeeLi, 2011/12/14, Porting proximity driver
/*SW5, Jamestsai, 1213, enable cypress{*/
#if defined(CONFIG_TOUCHSCREEN_CYPRESS_TTSP) || (CONFIG_TOUCHSCREEN_ATMEL_MXT)
static void omap4_touch_init(void)
{
        int error;

        /* atmel touch gpio IRQ */
        omap_mux_init_gpio(GPIO_TOUCH_IRQ, OMAP_PIN_INPUT_PULLUP);

        error = gpio_request(GPIO_TOUCH_IRQ, "touchscreen");
        if (error < 0) {
                pr_err("%s:failed to request GPIO %d, error %d\n",
                        __func__, GPIO_TOUCH_IRQ, error);
                return;
        }

        error = gpio_direction_input(GPIO_TOUCH_IRQ);
        if (error < 0) {
                pr_err("%s: GPIO configuration failed: GPIO %d,error %d\n",
                        __func__, GPIO_TOUCH_IRQ, error);
                gpio_free(GPIO_TOUCH_IRQ);
        }

        /* atmel touch gpio RESET */
        omap_mux_init_gpio(GPIO_TOUCH_RESET, OMAP_PIN_OUTPUT);

        error = gpio_request(GPIO_TOUCH_RESET, "touchscreen");
        if (error < 0) {
                pr_err("%s:failed to request GPIO %d, error %d\n",
                        __func__, GPIO_TOUCH_RESET, error);
                return;
        }

        error = gpio_direction_output(GPIO_TOUCH_RESET, 1);
        if (error < 0) {
                pr_err("%s: GPIO configuration failed: GPIO %d,error %d\n",
                        __func__, GPIO_TOUCH_RESET, error);
                gpio_free(GPIO_TOUCH_RESET);
        }

}
#endif
/*}SW5, Jamestsai, 1213, enable cypress*/

#ifdef CONFIG_MACH_OMAP4_BOWSER_SUBTYPE_JEM
static struct platform_device jem_devmodel_device = {
	.name		= "jem_devmodel",
	.id		= -1,
};

int __init jem_dev_init(void)
{
	platform_device_register(&jem_devmodel_device);
	return 0;
}
#endif

#if defined(CONFIG_USB_EHCI_HCD_OMAP) || defined(CONFIG_USB_OHCI_HCD_OMAP3)
static const struct usbhs_omap_board_data usbhs_bdata __initconst = {
	.port_mode[0] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[1] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset  = false,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = USB_NRESET,
	.reset_gpio_port[2]  = -EINVAL
};

static struct gpio bowser_ehci_gpios[] __initdata = {
	{USB_EXT_PWR_EN,	GPIOF_OUT_INIT_LOW,  "ext_boost_en"  },
	{ USB_NRESET,	GPIOF_OUT_INIT_LOW,  "Phy_nreset" },
	{ WAN_USB_RESUME_GPIO, GPIOF_OUT_INIT_LOW, "USB_Resume"},
};


static void __init omap4_ehci_ohci_init(void)
{

{
	int ret;
	struct clk *phy_ref_clk;

	if( board_has_wan() ) {

		/* disable external boost / Reset External PHY */
		ret = gpio_request_array(bowser_ehci_gpios,
					 ARRAY_SIZE(bowser_ehci_gpios));
		if (ret) {
			pr_err("Unable to initialize EHCI power/reset\n");
			return;
		}

		printk("USB PHY external auxclk3 is turned ON for a WAN board or a Bowser5\n");

		/* FREF_CLK3 provides the 19.2 MHz reference clock to the PHY */
		phy_ref_clk = clk_get(NULL, "auxclk3_ck");
		if (IS_ERR(phy_ref_clk)) {
			pr_err("Cannot request auxclk3\n");
			return;
		}
		clk_set_rate(phy_ref_clk, 19200000);
		clk_enable(phy_ref_clk);


		gpio_export(USB_EXT_PWR_EN, 0);
		gpio_set_value(WAN_USB_RESUME_GPIO, 0);

		usbhs_init(&usbhs_bdata);

		/* enable power to external Boost for VBUS */
		gpio_set_value(USB_EXT_PWR_EN, 1);
	}
	else {
		printk("USB PHY external auxclk3 is not turned ON \n");
	}
}
	return;

}

#else
static void __init omap4_ehci_ohci_init(void){}
#endif


static void __init omap_bowser_fixup(struct machine_desc *desc,
                                     struct tag *tags,
                                     char **cmdline, struct meminfo *mi)
{
        pr_err("omap_bowser_fixup\n");
        bowser_init_idme();
}

static void lpddr_init()
{
	int sd_vendor = omap_sdram_vendor();

	if ((sd_vendor == SAMSUNG_SDRAM) || (sd_vendor == ELPIDA_SDRAM) ||
		(sd_vendor == HYNIX_SDRAM)) {
		if ((omap_sdram_density() & SDRAM_DENSITY_MASK) == SDRAM_DENSITY_2CS) {
			omap_emif_setup_device_details(&emif_devices_2cs, &emif_devices_2cs);
			printk(KERN_INFO "** 2cs ddr detected\n");
			return;
		}
		omap_emif_setup_device_details(&emif_devices_1cs, &emif_devices_1cs);
		printk(KERN_INFO "** 1cs ddr detected **\n");
	}
	else
		printk(KERN_INFO "** DDR vendor_id: %d doesn't supported**\n", sd_vendor);
}

static void __init omap_4430sdp_init(void)
{
	int package = OMAP_PACKAGE_CBS;

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;
	omap4_mux_init(board_mux, NULL, package);

	lpddr_init();

	omap_board_config = sdp4430_config;
	omap_board_config_size = ARRAY_SIZE(sdp4430_config);

	omap_init_board_version(0);

	omap4_create_board_props();
	blaze_pmic_mux_init();
	
	omap4_i2c_init();
	/* JossCheng, 20111221, Porting gyro sensor and e-compass {*/	
#ifdef CONFIG_MPU_SENSORS_MPU6050B1
	mpu6050b1_init();
#endif
	/* JossCheng, 20111221, Porting gyro sensor and e-compass }*/
	//blaze_sensor_init();
// Anvoi, 2011/12/14, Porting Light sensor driver to ICS
	omap4_als_init();
// Anvoi, 2011/12/14, Porting Light sensor driver to ICS
	//blaze_touch_init();
	omap4_register_ion();
/*SW5, Anvoi, 20111215, Config key VolumeUp/VolumeDown{*/	
	platform_add_devices(bowser_devices, ARRAY_SIZE(bowser_devices));
/*SW5, Anvoi, 20111215, Config key VolumeUp/VolumeDown}*/
	board_serial_init();
	omap4_twl6030_hsmmc_init(mmc);
	bowser_wifi_init();
	omap4_sdp4430_bt_init();

	omap4_ehci_ohci_init();
	usb_musb_init(&musb_board_data);

// BokeeLi, 2011/12/14, Porting proximity driver
#ifdef CONFIG_INPUT_PIC12LF1822_PROXIMITY
	omap4_proximity_init();
#endif
// BokeeLi, 2011/12/14, Porting proximity driver

/*SW5, Jamestsai, 1213, enable cypress{*/
#if defined(CONFIG_TOUCHSCREEN_CYPRESS_TTSP) || (CONFIG_TOUCHSCREEN_ATMEL_MXT)
	omap4_touch_init();
#endif
/*}SW5, Jamestsai, 1213, enable cypress*/

#ifdef CONFIG_MACH_OMAP4_BOWSER_SUBTYPE_JEM
	jem_dev_init();
#endif

	omap_dmm_init();

	//omap_4430sdp_display_init();
	bowser_panel_init();


	omap_enable_smartreflex_on_init();
        if (enable_suspend_off)
                omap_pm_enable_off_mode();

}

static void __init omap_4430sdp_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}
static void __init omap_4430sdp_reserve(void)
{
	omap_init_ram_size();

#ifdef CONFIG_ION_OMAP
	bowser_android_display_setup(get_omap_ion_platform_data());
	omap_ion_init();
#else
	bowser_android_display_setup(NULL);
#endif

	omap_ram_console_init(OMAP_RAM_CONSOLE_START_DEFAULT,
			OMAP_RAM_CONSOLE_SIZE_DEFAULT);

	/* do the static reservations first */
	memblock_remove(PHYS_ADDR_SMC_MEM, PHYS_ADDR_SMC_SIZE);
	memblock_remove(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE);
	/* ipu needs to recognize secure input buffer area as well */
	omap_ipu_set_static_mempool(PHYS_ADDR_DUCATI_MEM, PHYS_ADDR_DUCATI_SIZE +
					OMAP4_ION_HEAP_SECURE_INPUT_SIZE +
					OMAP4_ION_HEAP_SECURE_OUTPUT_WFDHDCP_SIZE);

	omap_reserve();
}

MACHINE_START(OMAP4_BOWSER, "OMAP4 Bowser board")
	/* Maintainer: Santosh Shilimkar - Texas Instruments Inc */
	.boot_params	= 0x80000100,
	.reserve	= omap_4430sdp_reserve,
	.map_io		= omap_4430sdp_map_io,
        .fixup          = omap_bowser_fixup,
	.init_early	= omap_4430sdp_init_early,
	.init_irq	= gic_init_irq,
	.init_machine	= omap_4430sdp_init,
	.timer		= &omap_timer,
MACHINE_END
