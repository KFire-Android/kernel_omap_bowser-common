/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * based on gpio_key.c
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/i2c/twl.h>
#include <linux/suspend.h>
#include <linux/completion.h>
#include "../../kernel/power/power.h"

#undef DEBUGD
#define DEBUGD
#ifdef DEBUGD
#define dprintk(x...) printk(x)
#else
#define dprintk(x...)
#endif



struct gpio_button_data {
	struct gpio_keys_button *button;
	struct input_dev *input;
	struct work_struct work;
};

struct gpio_keys_drvdata {
	struct input_dev *input;
	unsigned int n_buttons;
	struct gpio_button_data data[0];
};


static bool bReprtingEvent = false;
static struct completion hall_completion;
static bool bWakebyHall = false;
static bool bSuspend = false;

static void gpio_keys_report_event(struct gpio_button_data *bdata)
{
	struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned int type = button->type ?: EV_KEY;


	init_completion(&hall_completion);
	bReprtingEvent = true;
        input_event(input, type, button->code, 1);
        input_sync(input);
        mdelay(300);

	input_event(input, type, button->code, 0);
	input_sync(input);

	dprintk("%s function:::::send >> code==%d\n", __func__, KEY_POWER); //debug

    complete(&hall_completion);

}


static void gpio_keys_work_func(struct work_struct *work)
{
	struct gpio_button_data *bdata =
		container_of(work, struct gpio_button_data, work);
	struct gpio_keys_button *button = bdata->button;
	int gpio_val = 0;

    if (bReprtingEvent)
            wait_for_completion(&hall_completion);

	gpio_val = gpio_get_value(button->gpio);

	if ((get_suspend_state()==PM_SUSPEND_ON) ^ gpio_val) {
		if(!gpio_val)
			gpio_keys_report_event(bdata);
		else if (bSuspend)
			bWakebyHall = true;
		else
			gpio_keys_report_event(bdata);
	}



    bReprtingEvent = false;
}

static irqreturn_t hall_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;
	struct gpio_keys_button *button = bdata->button;
	dprintk("Hall ISR\n");
	//bWakebyHall = true;

	BUG_ON(irq != gpio_to_irq(button->gpio));
	schedule_work(&bdata->work);
	return IRQ_HANDLED;
}

static int __devinit gpio_keys_setup_key(struct platform_device *pdev,
					 struct gpio_button_data *bdata,
					 struct gpio_keys_button *button)
{
	const char *desc = button->desc ? button->desc : "gpio_keys";
	struct device *dev = &pdev->dev;
	unsigned long irqflags;
	int irq, error;

	INIT_WORK(&bdata->work, gpio_keys_work_func);

	error = gpio_request(button->gpio, desc);
	dprintk("HAL GPIO = %d \n", button->gpio);
	if (error < 0) {
		dev_err(dev, "failed to request GPIO %d, error %d\n",
			button->gpio, error);
		goto fail2;
	}

	error = gpio_direction_input(button->gpio);
	if (error < 0) {
		dev_err(dev, "failed to configure"
			" direction for GPIO %d, error %d\n",
			button->gpio, error);
		goto fail3;
	}

	irq = gpio_to_irq(button->gpio);
	if (irq < 0) {
		error = irq;
		dev_err(dev, "Unable to get irq number for GPIO %d, error %d\n",
			button->gpio, error);
		goto fail3;
	}

	irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
	/*
	 * If platform has specified that the button can be disabled,
	 * we don't want it to share the interrupt line.
	 */
	if (!button->can_disable)
		irqflags |= IRQF_SHARED;


	error = request_irq(irq, hall_isr, irqflags, desc, bdata);
	if (error) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			irq, error);
		goto fail3;
	}

	return 0;

fail3:
	gpio_free(button->gpio);
fail2:
	return error;
}

static ssize_t gpio_status_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
	int gpio_status = 0;
	gpio_status = gpio_get_value(0);
	return sprintf(buf, "%d\n", gpio_status);
}
static struct kobj_attribute gpio_status_attribute = (struct kobj_attribute)__ATTR(gpio_status,0444,gpio_status_show,NULL);



static int __devinit tate_hall_probe(struct platform_device *pdev)
{

	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_keys_drvdata *ddata;
	struct device *dev = &pdev->dev;
	struct input_dev *input;
	int error;
	int wakeup = 0;
	struct gpio_keys_button *button;
	struct gpio_button_data *bdata;
	unsigned int type;

	dprintk("%s start.\n", __func__);

	//omap_writel(0x411b411b,0x4a31e040);



	ddata = kzalloc(sizeof(struct gpio_keys_drvdata) +
			sizeof(struct gpio_button_data),
			GFP_KERNEL);
	input = input_allocate_device();
	if (!ddata || !input) {
		dev_err(dev, "failed to allocate state\n");
		error = -ENOMEM;
		goto fail1;
	}

	ddata->input = input;

	platform_set_drvdata(pdev, ddata);

	input->name = pdev->name;
	input->dev.parent = &pdev->dev;

	button = pdata->buttons; //original setting
	bdata = ddata->data;
	type = button->type ?: EV_KEY;

	bdata->input = input;
	bdata->button = button;

	error = gpio_keys_setup_key(pdev, bdata, button);
	if (error)
		goto fail2;

	if (button->wakeup)
		wakeup = 1;

	input_set_capability(input, type, button->code);

	error = sysfs_create_file(&pdev->dev.kobj,&gpio_status_attribute.attr);
	if (error) {
		dev_err(dev, "Unable to export keys/switches, error: %d\n",
			error);
		goto fail2;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
			error);
		goto fail3;
	}



	/* get current state of buttons */
	gpio_keys_report_event(ddata->data);

	input_sync(input);

	device_init_wakeup(&pdev->dev, wakeup);
	dprintk("%s end.\n", __func__);
	return 0;



 fail3:
	sysfs_remove_file(&pdev->dev.kobj, &gpio_status_attribute.attr);
 fail2:
	free_irq(gpio_to_irq(pdata->buttons->gpio), ddata->data);
	cancel_work_sync(&ddata->data->work);
	gpio_free(pdata->buttons->gpio);

	platform_set_drvdata(pdev, NULL);
 fail1:
	input_free_device(input);
	kfree(ddata);

	return error;
}

static int __devexit tate_hall_remove(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;
	int irq;

	sysfs_remove_file(&pdev->dev.kobj, &gpio_status_attribute.attr);

	device_init_wakeup(&pdev->dev, 0);

	irq = gpio_to_irq(pdata->buttons->gpio);
	free_irq(irq, ddata->data);
	cancel_work_sync(&ddata->data->work);
	gpio_free(pdata->buttons->gpio);

	input_unregister_device(input);

	return 0;
}


#ifdef CONFIG_PM
static int gpio_keys_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;

	if (device_may_wakeup(&pdev->dev)) {
		struct gpio_keys_button *button = pdata->buttons;
		if (button->wakeup) {
			int irq = gpio_to_irq(button->gpio);
			enable_irq_wake(irq);
		}
	}
	bSuspend = true;
	return 0;
}

static int gpio_keys_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;

	struct gpio_keys_button *button = pdata->buttons;
	if (button->wakeup && device_may_wakeup(&pdev->dev)) {
		int irq = gpio_to_irq(button->gpio);
		disable_irq_wake(irq);
	}
	printk("HALL RESUME\n");
	if(bWakebyHall) {
	gpio_keys_report_event(ddata->data);
	input_sync(ddata->input);
	bWakebyHall = false;
	}
	bSuspend = false;
	return 0;
}

static const struct dev_pm_ops gpio_keys_pm_ops = {
	.suspend	= gpio_keys_suspend,
	.resume		= gpio_keys_resume,
};
#endif

static struct platform_driver hall_device_driver = {
	.probe		= tate_hall_probe,
	.remove		= tate_hall_remove,
	.driver		= {
		.name	= "hall",
		.owner	= THIS_MODULE,
#if 1
		.pm	= &gpio_keys_pm_ops,
#endif
	}
};

static int __init hall_sensors_init(void)
{
	return platform_driver_register(&hall_device_driver);
}

static void __exit hall_sensors_exit(void)
{
	platform_driver_unregister(&hall_device_driver);
}

module_init(hall_sensors_init);
module_exit(hall_sensors_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Quanta");
MODULE_DESCRIPTION("Hall sensor driver for Tate board");

