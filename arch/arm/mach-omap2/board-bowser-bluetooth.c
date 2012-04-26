/*
 * Bluetooth Broadcomm  and low power control via GPIO
 *
 *  Copyright (C) 2011 Google, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/rfkill.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <asm/mach-types.h>
#include <plat/serial.h>
#include <plat/omap-serial.h>
#include <plat/board-bowser-bluetooth.h>
#include <linux/regulator/driver.h>

// GPIO settings
#define BT_REG_GPIO 46
//att #define BT_RESET_GPIO 53

#define BT_WAKE_GPIO 49
#define BT_HOST_WAKE_GPIO 82

//The BT UART is set to UART2 on this platform (defined in plat/omap-serial.h)

#define BT_HOST_WAKE_INT_ENABLED	1
extern int bowser_bluetooth_irq_num;	//from board-xxxx.c

static struct rfkill *bt_rfkill;
static struct regulator *clk32kaudio_reg;
static bool bt_enabled;
static bool host_wake_uart_enabled;
static bool wake_uart_enabled;

struct bcm_bt_lpm {
	int wake;
	int host_wake;

	struct hrtimer enter_lpm_timer;
	ktime_t enter_lpm_delay;

	struct uart_port *uport;

	struct wake_lock wake_lock;
	char wake_lock_name[100];
} bt_lpm;

static int bcm2076_bt_rfkill_set_power(void *data, bool blocked)
{
	pr_debug( "%s calling gpio_set_value(BT_REG_GPIO,%d)\n",__FUNCTION__,!blocked);

	// rfkill_ops callback. Turn transmitter on when blocked is false
	if (!blocked) {
		printk("bcm2076_bt_rfkill_set_power(On)\n");
		if (clk32kaudio_reg && !bt_enabled)
			regulator_enable(clk32kaudio_reg);

		gpio_set_value(BT_REG_GPIO, 1);
//att BT_RESET_GPIO is tied to BT_REG_GPIO so no need to set this
//att		gpio_set_value(BT_RESET_GPIO, 1);

	} else {
		printk("bcm2076_bt_rfkill_set_power(Off)\n");
//att BT_RESET_GPIO is tied to BT_REG_GPIO so no need to set this
//att		gpio_set_value(BT_RESET_GPIO, 0);
		gpio_set_value(BT_REG_GPIO, 0);
		if (clk32kaudio_reg && bt_enabled)
			regulator_disable(clk32kaudio_reg);
	}

	bt_enabled = !blocked;

	return 0;
}

static const struct rfkill_ops bcm2076_bt_rfkill_ops = {
	.set_block = bcm2076_bt_rfkill_set_power,
};

static void set_wake_locked(int wake)
{
//pr_debug( "bbbluetooth.c:%s wake=%d TOP\n",__FUNCTION__,wake);
	bt_lpm.wake = wake;

	if (!wake)
		wake_unlock(&bt_lpm.wake_lock);

	if (!wake_uart_enabled && wake)
	{
		pr_debug( "%s !wake_uart_enabled && wake, calling omap_serial_ext_uart_enable\n",__FUNCTION__);
		omap_serial_ext_uart_enable(UART2);

		//Mux back GPIO PULL UP pin in RTS pin
		pr_debug("** set_wake_locked ** changing RTS pin from GPIO back to RTS\n");
		omap_rts_mux_write(0, UART2);
	}

// DOWN_STREAM_BT_WAKE_AND_UART_CONTROL_FIX ++
// Control BT_WAKE assertion/de-assertion in down-stream path only
#if 0
	pr_debug( "bbbluetooth.c:%s setting BT_WAKE(%s)\n",__FUNCTION__,wake ? "high":"low");
	gpio_set_value(BT_WAKE_GPIO, wake);
#endif
// DOWN_STREAM_BT_WAKE_AND_UART_CONTROL_FIX --

	if (wake_uart_enabled && !wake)
	{
		//Mux RTS pin into GPIO PULL UP pin (this flows off the 2076)
		pr_debug("** set_wake_locked ** changing RTS pin to GPIO pulled up\n");
		omap_rts_mux_write(MUX_PULL_UP, UART2);

		pr_debug( "%s wake_uart_enabled && !wake, calling omap_serial_ext_uart_disable\n",__FUNCTION__);
		omap_serial_ext_uart_disable(UART2);
	}
	wake_uart_enabled = wake;

//pr_debug( "bbbluetooth.c:%s wake_uart_enabled=%d END\n",__FUNCTION__,wake_uart_enabled);
}

static enum hrtimer_restart enter_lpm(struct hrtimer *timer) {
	unsigned long flags;

	pr_debug( "bbbluetooth.c:%s TOP uport=0x%x\n",__FUNCTION__,bt_lpm.uport);

// DOWN_STREAM_BT_WAKE_AND_UART_CONTROL_FIX ++
	gpio_set_value(BT_WAKE_GPIO, 0);
	pr_debug( "bbbluetooth.c:%s set BT_WAKE(low)\n",__FUNCTION__);
// DOWN_STREAM_BT_WAKE_AND_UART_CONTROL_FIX --

	spin_lock_irqsave(&bt_lpm.uport->lock, flags);
// DOWN_STREAM_BT_WAKE_AND_UART_CONTROL_FIX ++
	if (bt_lpm.host_wake)
	{
		hrtimer_start(&bt_lpm.enter_lpm_timer, bt_lpm.enter_lpm_delay,
			HRTIMER_MODE_REL);
	}
	else
	{
		set_wake_locked(0);
	}
// DOWN_STREAM_BT_WAKE_AND_UART_CONTROL_FIX --
	spin_unlock_irqrestore(&bt_lpm.uport->lock, flags);

	pr_debug( "bbbluetooth.c:%s END\n",__FUNCTION__);
	return HRTIMER_NORESTART;
}

void bcm_bt_lpm_exit_lpm_locked(struct uart_port *uport) {
	bt_lpm.uport = uport;

//pr_debug( "bbbluetooth.c:%s TOP uport=0x%x\n",__FUNCTION__,uport);

	hrtimer_try_to_cancel(&bt_lpm.enter_lpm_timer);

// DOWN_STREAM_BT_WAKE_AND_UART_CONTROL_FIX ++
// Assert BT_WAKE explicitly
	pr_debug( "bbbluetooth.c:%s setting BT_WAKE(high)\n",__FUNCTION__);
	gpio_set_value(BT_WAKE_GPIO, 1);
// DOWN_STREAM_BT_WAKE_AND_UART_CONTROL_FIX --
	set_wake_locked(1);

	hrtimer_start(&bt_lpm.enter_lpm_timer, bt_lpm.enter_lpm_delay,
		HRTIMER_MODE_REL);

	pr_debug( "bbbluetooth.c:%s called hrtimer_start(1 sec), END\n",__FUNCTION__);
}
EXPORT_SYMBOL(bcm_bt_lpm_exit_lpm_locked);

static void update_host_wake_locked(int host_wake)
{
	pr_debug( "%s TOP, host_wake=%d\n",__FUNCTION__,host_wake);
	if (host_wake == bt_lpm.host_wake)
	{
		pr_debug( "%s (host_wake==bt_lpm.host_wake), returning early\n",__FUNCTION__);
		return;
	}

	bt_lpm.host_wake = host_wake;

	if (host_wake) {
		wake_lock(&bt_lpm.wake_lock);

		if (!host_wake_uart_enabled)
		{
			omap_serial_ext_uart_enable(UART2);
			pr_debug( "%s !host_wake_uart_enabled, called omap_serial_ext_uart_enable\n",__FUNCTION__);

			//Mux back GPIO PULL UP pin in RTS pin
			pr_debug("** update_host_wake_locked ** changing RTS pin from GPIO back to RTS\n");
			omap_rts_mux_write(0, UART2);
		}
	} else {
		if (host_wake_uart_enabled)
		{
			//Mux RTS pin into GPIO PULL UP pin (this flows off the 2076)
			pr_debug("** update_host_wake_locked ** changing RTS pin to GPIO pulled up\n");
			omap_rts_mux_write(MUX_PULL_UP, UART2);

			omap_serial_ext_uart_disable(UART2);
			pr_debug( "%s host_wake_uart_enabled, called omap_serial_ext_uart_disable\n",__FUNCTION__);
		}

		// Take a timed wakelock, so that upper layers can take it.
		// The chipset deasserts the hostwake lock, when there is no
		// more data to send.
		wake_lock_timeout(&bt_lpm.wake_lock, HZ/2);
	}

	host_wake_uart_enabled = host_wake;
	pr_debug( "%s host_wake_uart_enabled = host_wake = %d\n",__FUNCTION__,host_wake);
}

static irqreturn_t host_wake_isr(int irq, void *dev)
{
	int host_wake;
	unsigned long flags;

	pr_debug( "%s TOP\n",__FUNCTION__);
	host_wake = gpio_get_value(BT_HOST_WAKE_GPIO);

	pr_debug( "%s host_wake = %d\n",__FUNCTION__,host_wake);

	irq_set_irq_type(irq, host_wake ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);

	if (!bt_lpm.uport) {
		bt_lpm.host_wake = host_wake;
		return IRQ_HANDLED;
	}

	spin_lock_irqsave(&bt_lpm.uport->lock, flags);
	update_host_wake_locked(host_wake);
	spin_unlock_irqrestore(&bt_lpm.uport->lock, flags);

	return IRQ_HANDLED;
}

static int bcm_bt_lpm_init(struct platform_device *pdev)
{
	int irq;
	int ret;
	int rc;

	pr_debug( "%s TOP\n",__FUNCTION__);

#if 0	//att XXX already done in board config
	rc = gpio_request(BT_WAKE_GPIO, "bcm2076_wake_gpio");
	if (unlikely(rc)) {
		return rc;
	}
#endif 	//att XXX already done in board config

#if BT_HOST_WAKE_INT_ENABLED
	pr_debug( "%s about to get the BT_HOST_WAKE GPIO\n",__FUNCTION__);
	rc = gpio_request(BT_HOST_WAKE_GPIO, "bcm2076_host_wake_gpio");
	if (unlikely(rc)) {
	pr_debug( "%s failed to get the BT_HOST_WAKE GPIO\n",__FUNCTION__);
		gpio_free(BT_WAKE_GPIO);
		return rc;
	}
#endif //BT_HOST_WAKE_INT_ENABLED

	hrtimer_init(&bt_lpm.enter_lpm_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	bt_lpm.enter_lpm_delay = ktime_set(1, 0);  /* 1 sec */
	bt_lpm.enter_lpm_timer.function = enter_lpm;

	bt_lpm.host_wake = 0;

#if BT_HOST_WAKE_INT_ENABLED
	pr_debug( "%s about to connect BT_HOST_WAKE_GPIO irq/isr\n",__FUNCTION__);

	//TI stores this in the board-config
	pr_debug( "%s using bowser_bluetooth_irq_num set in board config\n",__FUNCTION__);
	irq = bowser_bluetooth_irq_num;

	pr_debug( "%s about to call request_irq(host_wake_isr)\n",__FUNCTION__);
	ret = request_irq(irq, host_wake_isr, IRQF_TRIGGER_HIGH,
			  "bt host_wake", NULL);
	if (ret) {
		pr_debug( "%s failed to connect BT_HOST_WAKE_GPIO irq\n",__FUNCTION__);
		gpio_free(BT_WAKE_GPIO);
		gpio_free(BT_HOST_WAKE_GPIO);
		return ret;
	}
	pr_debug( "%s enabled BT_HOST_WAKE isr\n",__FUNCTION__);

#if 0	//TI says this is not needed, set in board config
	pr_debug( "%s about to set BT_HOST_WAKE irq\n",__FUNCTION__);
	ret = irq_set_irq_wake(irq, 1);
	if (ret) {
		pr_debug( "%s failed to set BT_HOST_WAKE irq\n",__FUNCTION__);
		gpio_free(BT_WAKE_GPIO);
		gpio_free(BT_HOST_WAKE_GPIO);
		return ret;
	}
#endif	//TI says this is not needed, set in board config

//	pr_debug( "%s skipping setting the directions of BT_HOST WAKE\n",__FUNCTION__);
//	gpio_direction_input(BT_HOST_WAKE_GPIO);
#else	//att XXX disabling this until TI enables it
	pr_debug( "%s skipping enabling the BT_HOST_WAKE irq\n",__FUNCTION__);
#endif //BT_HOST_WAKE_INT_ENABLED

	snprintf(bt_lpm.wake_lock_name, sizeof(bt_lpm.wake_lock_name),
			"BTLowPower");
	wake_lock_init(&bt_lpm.wake_lock, WAKE_LOCK_SUSPEND,
			 bt_lpm.wake_lock_name);

	return 0;
}

static int bcm2076_bluetooth_probe(struct platform_device *pdev)
{
	int rc = 0;
	int ret = 0;

pr_debug( "%s: TOP\n",__FUNCTION__);

#if 1
	pr_debug( "%s: skipping gpio_request(BT_RESET and BT_REG_GPIO). was done in boardconfig\n",__FUNCTION__);
#else
	rc = gpio_request(BT_RESET_GPIO, "bcm2076");
	if (unlikely(rc)) {
		return rc;
	}

	rc = gpio_request(BT_REG_GPIO, "bcm2076");
	if (unlikely(rc)) {
		gpio_free(BT_RESET_GPIO);
		return rc;
	}
#endif //att already done in board config

#if 1
	pr_debug( "%s: clk32kaudio always on. No need to call regulator_get()\n",__FUNCTION__);
#else
	clk32kaudio_reg = regulator_get(0, "clk32kaudio");
	if (IS_ERR(clk32kaudio_reg)) {
		pr_err("clk32kaudio reg not found!\n");
		clk32kaudio_reg = NULL;
	}
#endif

#if 1
	pr_debug( "%s skipping powering on BT_REG_GPIO, already done in board config\n",__FUNCTION__);
#else
	gpio_direction_output(BT_REG_GPIO, 1);
	gpio_direction_output(BT_RESET_GPIO, 1);
#endif

	bt_rfkill = rfkill_alloc("bcm2076 Bluetooth", &pdev->dev,
				RFKILL_TYPE_BLUETOOTH, &bcm2076_bt_rfkill_ops,
				NULL);

	if (unlikely(!bt_rfkill)) {
		pr_debug( "%s: rfkill_alloc('bcm2076_Bluetooth') failed, bt_rfkill=%d\n",__FUNCTION__,(int)bt_rfkill);
		return -ENOMEM;
	}

	rc = rfkill_register(bt_rfkill);
	if (unlikely(rc)) {
		pr_debug( "%s: rfkill_register(bt_rfkill) failed, rc=%d\n",__FUNCTION__,rc);
		rfkill_destroy(bt_rfkill);
		return -1;
	}

	rfkill_set_states(bt_rfkill, true, false);

	bcm2076_bt_rfkill_set_power(NULL, true);

	ret = bcm_bt_lpm_init(pdev);
	if (ret) {
		pr_debug( "%s: bcm_bt_lpm_init failed, ret=%d\n",__FUNCTION__,ret);
		rfkill_unregister(bt_rfkill);
		rfkill_destroy(bt_rfkill);
	}

	pr_debug( "%s: END ret = %d\n",__FUNCTION__,ret);

	return ret;
}

static int bcm2076_bluetooth_remove(struct platform_device *pdev)
{
	rfkill_unregister(bt_rfkill);
	rfkill_destroy(bt_rfkill);

	if (clk32kaudio_reg)
		regulator_put(clk32kaudio_reg);

	wake_lock_destroy(&bt_lpm.wake_lock);
	return 0;
}

int bcm2076_bluetooth_suspend(struct platform_device *pdev, pm_message_t state)
{
#if BT_HOST_WAKE_INT_ENABLED
	int irq = gpio_to_irq(BT_HOST_WAKE_GPIO);
	int host_wake;

	disable_irq(irq);
	host_wake = gpio_get_value(BT_HOST_WAKE_GPIO);

	if (host_wake) {
		enable_irq(irq);
		return -EBUSY;
	}

	//Mux RTS pin into GPIO PULL UP pin (this flows off the 2076)
	pr_debug("** suspend ** changing RTS pin to GPIO pulled up\n");
	omap_rts_mux_write(MUX_PULL_UP, UART2);

#else
	pr_debug( "%s: BT_HOST_WAKE disabled\n",__FUNCTION__);
#endif //BT_HOST_WAKE_INT_ENABLED

	return 0;
}

int bcm2076_bluetooth_resume(struct platform_device *pdev)
{
#if BT_HOST_WAKE_INT_ENABLED
	int irq = gpio_to_irq(BT_HOST_WAKE_GPIO);
	enable_irq(irq);
#else
	pr_debug( "%s: BT_HOST_WAKE disabled\n",__FUNCTION__);
#endif //BT_HOST_WAKE_INT_ENABLED

	//Mux back GPIO PULL UP pin in RTS pin
	pr_debug("** resume ** changing RTS pin from GPIO back to RTS\n");
	omap_rts_mux_write(0, UART2);

	return 0;
}

static struct platform_driver bcm2076_bluetooth_platform_driver = {
	.probe = bcm2076_bluetooth_probe,
	.remove = bcm2076_bluetooth_remove,
	.suspend = bcm2076_bluetooth_suspend,
	.resume = bcm2076_bluetooth_resume,
	.driver = {
		   .name = "bcm2076_bluetooth",
		   .owner = THIS_MODULE,
		   },
};

static int __init bcm2076_bluetooth_init(void)
{
	bt_enabled = false;
	return platform_driver_register(&bcm2076_bluetooth_platform_driver);
}

static void __exit bcm2076_bluetooth_exit(void)
{
	platform_driver_unregister(&bcm2076_bluetooth_platform_driver);
}


module_init(bcm2076_bluetooth_init);
module_exit(bcm2076_bluetooth_exit);

MODULE_ALIAS("platform:bcm2076");
MODULE_DESCRIPTION("bcm2076_bluetooth");
MODULE_AUTHOR("Jaikumar Ganesh <jaikumar@google.com>");
MODULE_LICENSE("GPL");
