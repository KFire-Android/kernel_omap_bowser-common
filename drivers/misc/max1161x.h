#ifndef _MAX1161X_H_
#define _MAX1161X_H_

/* Setup byte */
#define MAX1161X_SETUP_BYTE(a) ((a) | 0x80)
/* SEL2/SEL1/SEL0 */
#define MAX1161X_SETUP_SEL_MASK			0x70
#define MAX1161X_SETUP_AIN3_IS_AIN3_REF_IS_VDD	0x00
#define MAX1161X_SETUP_AIN3_IS_REF_EXT_TO_REF	0x20
#define MAX1161X_SETUP_AIN3_IS_AIN3_REF_IS_INT	0x40
#define MAX1161X_SETUP_AIN3_IS_REF_REF_IS_INT	0x60
#define MAX1161X_SETUP_POWER_DOWN_INT_REF	0x00
#define MAX1161X_SETUP_POWER_UP_INT_REF		0x10
/* CLK */
#define MAX1161X_SETUP_INT_CLOCK		0x00
#define MAX1161X_SETUP_EXT_CLOCK		0x08
/* BIP/UNI */
#define MAX1161X_SETUP_UNIPOLAR			0x00
#define MAX1161X_SETUP_BIPOLAR			0x04
/* RST */
#define MAX1161X_SETUP_RESET			0x00
#define MAX1161X_SETUP_NORESET			0x02

/* Configuration byte */
#define MAX1161X_CONFIG_BYTE(a) ((a))
/* SCAN1/SCAN0 */
#define MAX1161X_CONFIG_SCAN_MASK		0x60
#define MAX1161X_CONFIG_SCAN_TO_CS		0x00
#define MAX1161X_CONFIG_SCAN_SINGLE_8		0x20
#define MAX1161X_CONFIG_SCAN_MID_TO_CHANNEL	0x40
#define MAX1161X_CONFIG_SCAN_SINGLE_1		0x60
/* CS3/CS2/CS1/CS0 */
#define MAX1161X_CONFIG_CHANNEL_SEL_MASK	0x1E
#define MAX1161X_CONFIG_CHANNEL_SEL(a)		((a) << 1)
/* SGL/DIF */
#define MAX1161X_CONFIG_SE_DE_MASK		0x01
#define MAX1161X_CONFIG_DE			0x00
#define MAX1161X_CONFIG_SE			0x01

/* This must be maintained along side the max1161x_mode_table in max1161x.c */
enum max1161x_scan_mode {
	/* Single read of a single channel */
	_s0, _s1, _s2, _s3, _s4, _s5, _s6, _s7, _s8, _s9, _s10, _s11,
	/* Differential single read */
	d0m1, d2m3, d4m5, d6m7, d8m9, d10m11,
	d1m0, d3m2, d5m4, d7m6, d9m8, d11m10,
	/* Scan to channel and mid to channel where overlapping */
	s0to1, s0to2, s2to3, s0to3, s0to4, s0to5, s0to6,
	s6to7, s0to7, s6to8, s0to8, s6to9,
	s0to9, s6to10, s0to10, s6to11, s0to11,
	/* Differential scan to channel and mid to channel where overlapping */
	d0m1to2m3, d0m1to4m5, d0m1to6m7, d6m7to8m9,
	d0m1to8m9, d6m7to10m11, d0m1to10m11, d1m0to3m2,
	d1m0to5m4, d1m0to7m6, d7m6to9m8, d1m0to9m8,
	d7m6to11m10, d1m0to11m10,
};

#define MAX1161X_CHANNEL_MAX	12

#define MAX1161X_ADC_IOC_MAGIC '`'
#define MAX1161X_ADC_IOCX_ADC_SETUP_READ	_IO(MAX1161X_ADC_IOC_MAGIC, 0)
#define MAX1161X_ADC_IOCX_ADC_SETUP_WRITE	_IO(MAX1161X_ADC_IOC_MAGIC, 1)
#define MAX1161X_ADC_IOCX_ADC_RAW_READ		_IO(MAX1161X_ADC_IOC_MAGIC, 2)

struct max1161x_setup {
	int reference;
	int external_clock;
	int bipolar;
};

struct max1161x_raw_read {
	int mode;
	u16 result[MAX1161X_CHANNEL_MAX];
};

struct max1161x_adc_user_parms {
	union {
		struct max1161x_setup		setup;
		struct max1161x_raw_read	raw_read;
	};
};

#endif /* _MAX1161X_H_ */
