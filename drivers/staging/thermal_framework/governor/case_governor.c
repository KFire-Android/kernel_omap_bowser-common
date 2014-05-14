/*
 * drivers/thermal_framework/governor/case_governor.c
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Sebastien Sabatier <s-sabatier1@ti.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include <linux/err.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/debugfs.h>

#include <linux/thermal_framework.h>
#include <plat/cpu.h>

#define THERMAL_PREFIX "Thermal Policy: case_governor: "
#define THERMAL_INFO(fmt, args...) do { printk(KERN_INFO THERMAL_PREFIX fmt, ## args); } while(0)

#ifdef THERMAL_DEBUG
#define THERMAL_DBG(fmt, args...) do { printk(KERN_DEBUG THERMAL_PREFIX fmt, ## args); } while(0)
#else
#define THERMAL_DBG(fmt, args...) do {} while(0)
#endif

/* System/Case Thermal thresholds */
#define SYS_THRESHOLD_HOT		61500
#define SYS_THRESHOLD_COLD		57500
#define CPU_THRESHOLD_COLD		59500
#define SYS_THRESHOLD_HOT_INC		500
#define INIT_COOLING_LEVEL		0
#define CASE_SUBZONES_NUMBER		4
#define CASE_MAX_COOLING_ACTION		(CASE_SUBZONES_NUMBER + 4 + (omap4_has_mpu_1_5ghz()? 1:0) )
int case_subzone_number = CASE_SUBZONES_NUMBER;
static int sys_threshold_hot = SYS_THRESHOLD_HOT;
static int sys_threshold_cold = SYS_THRESHOLD_COLD;
static int cpu_threshold_cold = CPU_THRESHOLD_COLD;
static int thot = SYS_THRESHOLD_COLD;
static int tcold = SYS_THRESHOLD_COLD;

struct case_governor {
	struct thermal_dev *temp_sensor;
	int cooling_level;
};

static struct thermal_dev *therm_fw;
static struct case_governor *case_gov;
static int case_thermal_shutdown_enabled = 1;

static void case_reached_max_state(struct list_head *cooling_list, int temp)
{
	/* invoke charger cooling device with cooling level 0 to reset charger limit to max  */
	/* So, device can be charged during powered off state caused by thermal policy */
	THERMAL_INFO("!!! Thermal Shutdown!!!, reset charger current to MAX");
	thermal_device_call_dev(cooling_list, "charger_cooling" , cool_device, 0);

	/* we have done everything that could be done so far, giving up */
	orderly_poweroff(true);
}

/**
 * DOC: Introduction
 * =================
 * The SYSTEM Thermal governor maintains the policy for the SYSTEM
 * temperature (PCB and/or case). The main goal of the governor is to
 * get the temperature from a thermal sensor and calculate the current
 * temperature of the case. When the case temperature is above the hot
 * limit, then the SYSTEM Thermal governor will cool the system by
 * throttling CPU frequency.
 * When the temperature is below the cold limit, then the SYSTEM Thermal
 * The SYSTEM Thermal governor may use 3 different sensors :
 * - CPU (OMAP) on-die temperature sensor (if there is no PCB sensor)
 * - PCB sensor located closed to CPU die
 * - Dedicated sensor for case temperature management
 * To take into account the response delay between the case temperature
 * and the temperature from one of these sensors, the sensor temperature
 * should be averaged.
 *
 * @cooling_list: The list of cooling devices available to cool the zone
 * @temp:	Temperature (average on-die CPU temp or PCB temp sensor)
*/

static int case_thermal_manager(struct list_head *cooling_list, int temp)
{
	if (temp == 0)
		return 0;
	THERMAL_DBG(" temp %d, tcold %d, thot %d, level %d\n",temp, tcold, thot, case_gov->cooling_level);
	/* Temperature just crossed below cpu_threshold_cold, but
	 * still temperature has not reached sys_threshold_cold yet.
	 * Restore CPU max freq earlier than other cooling devices.
	 */
	if ( (tcold == cpu_threshold_cold) && (temp < sys_threshold_hot) && (thot >= sys_threshold_hot) ) {
		thot = sys_threshold_hot;
		tcold = sys_threshold_cold ;
		/* update cooling level */
		case_gov->cooling_level = (cpu_threshold_cold - sys_threshold_cold)/ SYS_THRESHOLD_HOT_INC;

		/* invoke cpu cooling device with cooling level 0 */
		thermal_device_call_dev(cooling_list, "cpufreq_cooling.1" , cool_device, 0);
		/* invoke charger cooling device with cooling level 2, so device gets charged */
		/* otherwise battery won't be charged until temp drops to t_cold which is 58.*/
		thermal_device_call_dev(cooling_list, "charger_cooling" , cool_device, 2);
		/* update tcold and thot on temp_sensor */
		thermal_device_call(case_gov->temp_sensor, set_temp_thresh, tcold  , thot);

		/* case_report_temp() at case_report_temp.c set hot_event to 0
		 * once temp crosses down tcold which is cpu_threshold_cold.
		 * we have to override hot_event to 1, so still case thermal manager
		 * remains active to detect temp crosses down new tcold
		 * which would be sys_threshold_cold. */
		thermal_device_call(case_gov->temp_sensor, set_hot_event, 1);

		THERMAL_DBG("%s: step up CPU max freq to default max freq, tcold set to %d, thot set to %d",
			__func__, tcold, thot);

		/* Other cooling devices are still under thermal policy 
		 * because temp has not reached sys_threshold_cold, so skip the rest of code.
		 */
		return 0;
	}

	if (temp >= sys_threshold_hot) {
		case_gov->cooling_level++;
		thot += SYS_THRESHOLD_HOT_INC;
		pr_info("%s:syst temp >= thot thot set to %d", __func__, thot);
		if (case_gov->cooling_level > CASE_MAX_COOLING_ACTION && case_thermal_shutdown_enabled )
			case_reached_max_state(cooling_list, temp);
	} else if (temp < sys_threshold_cold) {
		case_gov->cooling_level = INIT_COOLING_LEVEL;
		/* We want to be notified on the first subzone */
		thot = sys_threshold_cold;
		pr_info("%s: syst temp =< tcold thot set to %d", __func__,
									thot);
	} else {
		case_gov->cooling_level++;
		thot = sys_threshold_cold +
			((sys_threshold_hot - sys_threshold_cold) /
			 case_subzone_number) *
			case_gov->cooling_level;
		pr_info("%s: sys_threshold_hot >= syst temp "
				">= tcold thot set to %d",
				__func__, thot);
	}
	thermal_device_call_all(cooling_list, cool_device,
						case_gov->cooling_level);

	if ( (thot  > sys_threshold_hot) && (temp >= cpu_threshold_cold) ) {
		THERMAL_DBG("%s: thot  > sys_threshold_hot  && temp >= cpu_threshold_coldeshold_hot, set tcold to cpu_threshold_cold %d", \
				__func__, cpu_threshold_cold);
		tcold = cpu_threshold_cold;
	}
	else {
		THERMAL_DBG("%s: set tcold to sys_threshold_cold %d", __func__, sys_threshold_cold );
		tcold = sys_threshold_cold;
	}

	thermal_device_call(case_gov->temp_sensor, set_temp_thresh,
								tcold, thot);
	return 0;
}

static int case_process_temp(struct thermal_dev *gov,
				struct list_head *cooling_list,
				struct thermal_dev *temp_sensor,
				int temp)
{
	int ret;

	case_gov->temp_sensor = temp_sensor;
	ret = case_thermal_manager(cooling_list, temp);

	return ret;
}

static int option_get(void *data, u64 *val)
{
	u32 *option = data;

	*val = *option;

	return 0;
}

static int option_set(void *data, u64 val)
{
	u32 *option = data;

	*option = val;

	/* reset intermediate thot & tcold as the sys constraint has changed */
	tcold = sys_threshold_cold;
	thot = sys_threshold_cold;

	thermal_device_call(case_gov->temp_sensor, set_temp_thresh,
				tcold, thot);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(case_fops, option_get, option_set, "%llu\n");

static int case_register_debug_entries(struct thermal_dev *gov,
					struct dentry *d)
{
	(void) debugfs_create_file("case_subzone_number",
			S_IRUGO | S_IWUSR, d, &case_subzone_number,
			&case_fops);
	(void) debugfs_create_file("sys_threshold_hot",
			S_IRUGO | S_IWUSR, d, &sys_threshold_hot,
			&case_fops);
	(void) debugfs_create_file("sys_threshold_cold",
			S_IRUGO | S_IWUSR, d, &sys_threshold_cold,
			&case_fops);
	(void) debugfs_create_file("cpu_threshold_cold",
			S_IRUGO | S_IWUSR, d, &cpu_threshold_cold,
			&case_fops);
	return 0;
}

static struct thermal_dev_ops case_gov_ops = {
	.process_temp = case_process_temp,
#ifdef CONFIG_THERMAL_FRAMEWORK_DEBUG
	.register_debug_entries = case_register_debug_entries,
#endif
};

static int __init thermal_shutdown_disable(char *str)
{
	case_thermal_shutdown_enabled = 0;
	THERMAL_INFO("Thermal Shutdown Disabled!!!");
        return 0;
}


__setup("case_thermal_shutdown_disable", thermal_shutdown_disable);


static int __init case_governor_init(void)
{
	struct thermal_dev *thermal_fw;

	case_gov = kzalloc(sizeof(struct case_governor), GFP_KERNEL);
	if (!case_gov) {
		pr_err("%s:Cannot allocate memory\n", __func__);
		return -ENOMEM;
	}

	thermal_fw = kzalloc(sizeof(struct thermal_dev), GFP_KERNEL);
	if (thermal_fw) {
		thermal_fw->name = "case_governor";
		thermal_fw->domain_name = "case";
		thermal_fw->dev_ops = &case_gov_ops;
		thermal_governor_dev_register(thermal_fw);
		therm_fw = thermal_fw;
	} else {
		pr_err("%s: Cannot allocate memory\n", __func__);
		kfree(case_gov);
		return -ENOMEM;
	}

	case_gov->cooling_level = INIT_COOLING_LEVEL;

	return 0;
}

static void __exit case_governor_exit(void)
{
	thermal_governor_dev_unregister(therm_fw);
	kfree(therm_fw);
	kfree(case_gov);
}

module_init(case_governor_init);
module_exit(case_governor_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("System/Case thermal governor");
MODULE_LICENSE("GPL");
