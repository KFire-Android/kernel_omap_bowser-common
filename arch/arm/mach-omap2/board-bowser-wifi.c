#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h>

#include <linux/hwspinlock.h>

#include <linux/gpio_keys.h>
#include <linux/memblock.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>

#include <mach/hardware.h>
#include <mach/omap4-common.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/mmc.h>
//#include <plat/omap-pm.h>

#include <mach/bowser_idme_init.h>

#include "board-bowser.h"
#include "control.h"
#include "mux.h"

#define GPIO_WLAN_EN		43
#define GPIO_WLAN_HOST_WAKE 44	// Connect to GPIO_1 of BCM43239

#ifdef CONFIG_DHD_USE_STATIC_BUF

#define PREALLOC_WLAN_NUMBER_OF_SECTIONS	4
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS		160
#define PREALLOC_WLAN_SECTION_HEADER		24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)

#define WLAN_SKB_BUF_NUM	16

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

typedef struct wifi_mem_prealloc_struct {
	void *mem_ptr;
	unsigned long size;
} wifi_mem_prealloc_t;

static wifi_mem_prealloc_t wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};

static void *bowser_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_NUMBER_OF_SECTIONS)
		return wlan_static_skb;
	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;
	if (wifi_mem_array[section].size < size)
		return NULL;
	return wifi_mem_array[section].mem_ptr;
}

#endif //CONFIG_DHD_USE_STATIC_BUF

int __init bowser_wifi_mem_init(void)
{
#ifdef CONFIG_DHD_USE_STATIC_BUF
	int i;

	for(i=0;( i < WLAN_SKB_BUF_NUM );i++) {
		if (i < (WLAN_SKB_BUF_NUM/2))
			wlan_static_skb[i] = dev_alloc_skb(4096);
		else
			wlan_static_skb[i] = dev_alloc_skb(8192);
	}
	for(i=0;( i < PREALLOC_WLAN_NUMBER_OF_SECTIONS );i++) {
		wifi_mem_array[i].mem_ptr = kmalloc(wifi_mem_array[i].size,
							GFP_KERNEL);
		if (wifi_mem_array[i].mem_ptr == NULL)
			return -ENOMEM;
	}
#endif

	return 0;
}

#if 0
static struct resource bowser_wifi_resources[] = {
	[0] = {
		.name		= "bcmdhd_wlan_irq",
		.start		= OMAP_GPIO_IRQ(GPIO_WLAN_IRQ),
		.end		= OMAP_GPIO_IRQ(GPIO_WLAN_IRQ),
		.flags          = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};
#endif

static int bowser_wifi_cd = 0; /* WIFI virtual 'card detect' status */
static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

static int bowser_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;

	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;

	return 0;
}

static unsigned int bowser_wifi_status(struct device *dev)
{
	return bowser_wifi_cd;
}

struct mmc_platform_data bowser_wifi_data = {
	.ocr_mask		= MMC_VDD_165_195 | MMC_VDD_20_21,
	.built_in		= 1,
	.status			= bowser_wifi_status,
	.card_present		= 0,
	.register_status_notify	= bowser_wifi_status_register,
};

static int bowser_wifi_set_carddetect(int val)
{
	pr_debug("%s: %d\n", __func__, val);
	bowser_wifi_cd = val;

	if (wifi_status_cb) {
		wifi_status_cb(val, wifi_status_cb_devid);
	} else
		pr_warning("%s: Nobody to notify\n", __func__);

	return 0;
}

static int bowser_wifi_power_state;
static int bowser_wifi_reset_state;

static int bowser_wifi_power(int on)
{
	printk("%s: Powering %s wifi\n", __FUNCTION__, (on ? "on" : "off"));

	if (on == bowser_wifi_power_state)
		return 0;

	// KS:
	// GPIO_WLAN_EN connects to both WL_REG_ON and WL_RST_N, within the wifi module.
	// GPIO_WLAN_EN: 1 => turn on regulator, release reset
	//               0 => turn off regulator, assert reset
	gpio_set_value(GPIO_WLAN_EN, on);
	if (on) {
		// KS: minimum delay =
		//     1.5ms (internal PMU) +
		//     104ms (internal POR) +
		//     4ms (device responds to host F0 read) +
		//     8ms (reference clock is up, PLL register accessible) +

		msleep(150);
	}

	bowser_wifi_power_state = on;
	return 0;
}

static int bowser_wifi_reset(int on)
{
	printk("%s: %d\n", __FUNCTION__, on);
	bowser_wifi_reset_state = on;
	
	//bowser_wifi_power(on);
	return 0;
}

static int char2int(unsigned char c)
{
	if (('0' <= c) && (c <= '9')) {
		return c - '0';
	}
	else if (('A' <= c) && (c <= 'F')) {
		return c - 'A' + 10;
	}
	else if (('a' <= c) && (c <= 'f')) {
		return c - 'a' + 10;
	}

	return 0;
}

static int bowser_wifi_get_mac_addr(unsigned char *buf)
{
	unsigned char mac_addr[6];
	int hexIdx,charIdx;

	if (!buf)
		return -EFAULT;

	for (charIdx = 0, hexIdx = 0; charIdx < 12; hexIdx++) {
		mac_addr[hexIdx] = char2int(system_mac_addr[charIdx++]);
		mac_addr[hexIdx] = mac_addr[hexIdx] << 4;
		mac_addr[hexIdx] = mac_addr[hexIdx] + char2int(system_mac_addr[charIdx++]);
	}

	memcpy(buf, mac_addr, 6);
	return 0;
}

/* Customized Locale table : OPTIONAL feature */
#define WLC_CNTRY_BUF_SZ	4
typedef struct cntry_locales_custom {
	char iso_abbrev[WLC_CNTRY_BUF_SZ];
	char custom_locale[WLC_CNTRY_BUF_SZ];
	int  custom_locale_rev;
} cntry_locales_custom_t;

static cntry_locales_custom_t bowser_wifi_translate_custom_table[] = {
/* Table should be filled out based on custom platform regulatory requirement */
	{"",   "XZ", 10}, /* universal */
	{"US", "US", 69}, /* input ISO "US" to : US regrev 69 */
	{"CA", "US", 69}, /* input ISO "CA" to : US regrev 69 */
	{"EU", "EU", 5},  /* European union countries */
	{"AT", "EU", 5},
	{"BE", "EU", 5},
	{"BG", "EU", 5},
	{"CY", "EU", 5},
	{"CZ", "EU", 5},
	{"DK", "EU", 5},
	{"EE", "EU", 5},
	{"FI", "EU", 5},
	{"FR", "EU", 5},
	{"DE", "EU", 5},
	{"GR", "EU", 5},
	{"HU", "EU", 5},
	{"IE", "EU", 5},
	{"IT", "EU", 5},
	{"LV", "EU", 5},
	{"LI", "EU", 5},
	{"LT", "EU", 5},
	{"LU", "EU", 5},
	{"MT", "EU", 5},
	{"NL", "EU", 5},
	{"PL", "EU", 5},
	{"PT", "EU", 5},
	{"RO", "EU", 5},
	{"SK", "EU", 5},
	{"SI", "EU", 5},
	{"ES", "EU", 5},
	{"SE", "EU", 5},
	{"GB", "EU", 5},  /* input ISO "GB" to : EU regrev 05 */
	{"IL", "IL", 0},
	{"CH", "CH", 0},
	{"TR", "TR", 0},
	{"NO", "NO", 0},
	{"KR", "XY", 3},
	{"AU", "XY", 3},
	{"CN", "XY", 3},  /* input ISO "CN" to : XY regrev 03 */
	{"TW", "XY", 3},
	{"AR", "XY", 3},
	{"MX", "XY", 3}
};

static void *bowser_wifi_get_country_code(char *ccode)
{
	int size = ARRAY_SIZE(bowser_wifi_translate_custom_table);
	int i;

	if (!ccode)
		return NULL;
	printk("%s: Country code: %s\n", __func__, ccode);

	for (i = 0; i < size; i++)
		if (strcmp(ccode, bowser_wifi_translate_custom_table[i].iso_abbrev) == 0)
			return &bowser_wifi_translate_custom_table[i];
	return &bowser_wifi_translate_custom_table[0];
}

static struct wifi_platform_data bowser_wifi_control = {
	.set_power      = bowser_wifi_power,
	.set_reset      = bowser_wifi_reset,
	.set_carddetect = bowser_wifi_set_carddetect,
#ifdef CONFIG_DHD_USE_STATIC_BUF
	.mem_prealloc	= bowser_wifi_mem_prealloc,
#else
	.mem_prealloc	= NULL,
#endif
	.get_mac_addr	= bowser_wifi_get_mac_addr,
	.get_country_code = bowser_wifi_get_country_code,
};

struct platform_device bowser_wifi_device = {
		.name = "bcmdhd_wlan",
		.id             = 1,
		.dev            = {
        		.platform_data = &bowser_wifi_control,
		},
};

int bowser_wifi_irq_num = -1;
EXPORT_SYMBOL(bowser_wifi_irq_num);

void bowser_wifi_mux_init(void)
{
#ifndef CONFIG_MACH_OMAP4_BOWSER_SUBTYPE_TATE
	u32 val = 0;
#endif

	omap_mux_init_gpio(GPIO_WLAN_HOST_WAKE,
			OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE | OMAP_WAKEUP_EVENT);

	omap_mux_init_gpio(GPIO_WLAN_EN, OMAP_PIN_OUTPUT);

	omap_mux_init_signal("sdmmc5_cmd.sdmmc5_cmd",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_clk.sdmmc5_clk",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat0.sdmmc5_dat0",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat1.sdmmc5_dat1",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat2.sdmmc5_dat2",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("sdmmc5_dat3.sdmmc5_dat3",
				OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);


#ifndef CONFIG_MACH_OMAP4_BOWSER_SUBTYPE_TATE
	/* Change slew rate on SDIO Signals */

	val = omap4_ctrl_pad_readl(OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_SMART3IO_PADCONF_1);
	val |= (3 << 30);
	omap4_ctrl_pad_writel(val, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_SMART3IO_PADCONF_1);

	val = omap4_ctrl_pad_readl(OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_SMART3IO_PADCONF_2);
	val |= (1 << 15);
	omap4_ctrl_pad_writel(val, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_SMART3IO_PADCONF_2);
#endif

	/* WIFI related GPIOs */
	if (gpio_request(GPIO_WLAN_EN, "bcm43239") ||
	    gpio_direction_output(GPIO_WLAN_EN, 1))
		pr_err("Error in initializing Wifi chip enable gpio: unable to set direction.\n");

	gpio_set_value(GPIO_WLAN_EN, 0);

	/* Wifi IRQ and Host Wake */
	bowser_wifi_irq_num = OMAP_GPIO_IRQ(GPIO_WLAN_HOST_WAKE);
	printk("%s: Configured GPIO_%d as wifi host wake interrupt, IRQ num = %d\n", 
			__FUNCTION__, GPIO_WLAN_HOST_WAKE, bowser_wifi_irq_num);
}

int __init bowser_wifi_init(void)
{
	int rc;
	
	bowser_wifi_mux_init();
	bowser_wifi_mem_init();
	
	rc = platform_device_register(&bowser_wifi_device);
	
	return rc;
}

