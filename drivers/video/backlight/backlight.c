/*
 * Backlight Lowlevel Control Abstraction
 *
 * Copyright (C) 2003,2004 Hewlett-Packard Company
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/backlight.h>
#include <linux/notifier.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/slab.h>
#include <linux/thermal_framework.h>

#ifdef CONFIG_PMAC_BACKLIGHT
#include <asm/backlight.h>
#endif

/* JossCheng, 20111222, LCM CABC control { */
#include <linux/gpio.h>
#include <linux/delay.h>
#define BKL_CABC_APS_EN 37
/* JossCheng, 20111222, LCM CABC control } */

#define COOLING_STEP		10
static const char const *backlight_types[] = {
	[BACKLIGHT_RAW] = "raw",
	[BACKLIGHT_PLATFORM] = "platform",
	[BACKLIGHT_FIRMWARE] = "firmware",
};
#define THERMAL_PREFIX "Thermal Policy: brightness agent: "
#define THERMAL_INFO(fmt, args...) do { printk(KERN_INFO THERMAL_PREFIX fmt, ## args); } while(0)

#ifdef THERMAL_DEBUG
#define THERMAL_DBG(fmt, args...) do { printk(KERN_DEBUG THERMAL_PREFIX fmt, ## args); } while(0)
#else
#define THERMAL_DBG(fmt, args...) do {} while(0)
#endif

#if defined(CONFIG_FB) || (defined(CONFIG_FB_MODULE) && \
			   defined(CONFIG_BACKLIGHT_CLASS_DEVICE_MODULE))
/* This callback gets called when something important happens inside a
 * framebuffer driver. We're looking if that important event is blanking,
 * and if it is, we're switching backlight power as well ...
 */
static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct backlight_device *bd;
	struct fb_event *evdata = data;

	/* If we aren't interested in this event, skip it immediately ... */
	if (event != FB_EVENT_BLANK && event != FB_EVENT_CONBLANK)
		return 0;

	bd = container_of(self, struct backlight_device, fb_notif);
	mutex_lock(&bd->ops_lock);
	if (bd->ops)
		if (!bd->ops->check_fb ||
		    bd->ops->check_fb(bd, evdata->info)) {
			bd->props.fb_blank = *(int *)evdata->data;
			if (bd->props.fb_blank == FB_BLANK_UNBLANK)
				bd->props.state &= ~BL_CORE_FBBLANK;
			else
				bd->props.state |= BL_CORE_FBBLANK;
			backlight_update_status(bd);
		}
	mutex_unlock(&bd->ops_lock);
	return 0;
}

static int backlight_register_fb(struct backlight_device *bd)
{
	memset(&bd->fb_notif, 0, sizeof(bd->fb_notif));
	bd->fb_notif.notifier_call = fb_notifier_callback;

	return fb_register_client(&bd->fb_notif);
}

static void backlight_unregister_fb(struct backlight_device *bd)
{
	fb_unregister_client(&bd->fb_notif);
}
#else
static inline int backlight_register_fb(struct backlight_device *bd)
{
	return 0;
}

static inline void backlight_unregister_fb(struct backlight_device *bd)
{
}
#endif /* CONFIG_FB */

static void backlight_generate_event(struct backlight_device *bd,
				     enum backlight_update_reason reason)
{
	char *envp[2];

	switch (reason) {
	case BACKLIGHT_UPDATE_SYSFS:
		envp[0] = "SOURCE=sysfs";
		break;
	case BACKLIGHT_UPDATE_HOTKEY:
		envp[0] = "SOURCE=hotkey";
		break;
	default:
		envp[0] = "SOURCE=unknown";
		break;
	}
	envp[1] = NULL;
	kobject_uevent_env(&bd->dev.kobj, KOBJ_CHANGE, envp);
	sysfs_notify(&bd->dev.kobj, NULL, "actual_brightness");
}

static ssize_t backlight_show_power(struct device *dev,
		struct device_attribute *attr,char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);

	return sprintf(buf, "%d\n", bd->props.power);
}

static ssize_t backlight_store_power(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct backlight_device *bd = to_backlight_device(dev);
	unsigned long power;

	rc = strict_strtoul(buf, 0, &power);
	if (rc)
		return rc;

	rc = -ENXIO;
	mutex_lock(&bd->ops_lock);
	if (bd->ops) {
		pr_debug("backlight: set power to %lu\n", power);
		if (bd->props.power != power) {
			bd->props.power = power;
			backlight_update_status(bd);
		}
		rc = count;
	}
	mutex_unlock(&bd->ops_lock);

	return rc;
}

static ssize_t backlight_show_brightness(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);

	return sprintf(buf, "%d\n", bd->props.brightness);
}

static ssize_t backlight_store_brightness(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct backlight_device *bd = to_backlight_device(dev);
	unsigned long brightness;

	rc = strict_strtoul(buf, 0, &brightness);
	if (rc)
		return rc;

	rc = -ENXIO;

	mutex_lock(&bd->ops_lock);
	if (bd->ops) {
		if (brightness > bd->props.max_brightness ||
				/* account for thermal limits */
				brightness > bd->props.max_thermal_brightness)
			rc = -EINVAL;
		else {
			pr_debug("backlight: set brightness to %lu\n",
				 brightness);
			bd->props.brightness = brightness;
		        backlight_update_status(bd);
		        rc = count;
	        }
	}
	/* save requested brightness even though it is greater than max_thermal_brightness */
	/* so, when system cools down, brightness will be set to saved_brightness. */
	bd->props.saved_brightness = brightness & bd->props.max_brightness;
	mutex_unlock(&bd->ops_lock);

	backlight_generate_event(bd, BACKLIGHT_UPDATE_SYSFS);

	return rc;
}

static ssize_t backlight_show_type(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);

	return sprintf(buf, "%s\n", backlight_types[bd->props.type]);
}

static ssize_t backlight_show_max_brightness(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);

	return sprintf(buf, "%d\n", bd->props.max_brightness);
}

static ssize_t backlight_show_max_thermal_brightness(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);

	return sprintf(buf, "%d\n", bd->props.max_thermal_brightness);
}

static ssize_t backlight_show_saved_brightness(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct backlight_device *bd = to_backlight_device(dev);

	return sprintf(buf, "%d\n", bd->props.saved_brightness);
}

static ssize_t backlight_show_actual_brightness(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int rc = -ENXIO;
	struct backlight_device *bd = to_backlight_device(dev);

	mutex_lock(&bd->ops_lock);
	if (bd->ops && bd->ops->get_brightness)
		rc = sprintf(buf, "%d\n", bd->ops->get_brightness(bd));
	mutex_unlock(&bd->ops_lock);

	return rc;
}

/* JossCheng, 20111222, LCM CABC control { */
static ssize_t backlight_show_cabc(struct device *dev,
		struct device_attribute *attr,char *buf)
{
	int val;

	val = gpio_get_value(BKL_CABC_APS_EN);
	return sprintf(buf, "%d\n", val);
}

static ssize_t backlight_store_cabc(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc, cabc_gpio_value;
	unsigned long cabc;

	rc = strict_strtoul(buf, 0, &cabc);
	if (rc)
		return rc;

	rc = -ENXIO;

	cabc_gpio_value = gpio_get_value(BKL_CABC_APS_EN);
	if (cabc && (cabc_gpio_value == 0)){
		gpio_direction_output(BKL_CABC_APS_EN, 1);
		printk(KERN_DEBUG "cabc_enable\n");
		}
	else if(!cabc && (cabc_gpio_value == 1)){
		gpio_direction_output(BKL_CABC_APS_EN, 0);
		printk(KERN_DEBUG "cabc_disable\n");
		}
	rc = count;
	return rc;
}
/* JossCheng, 20111222, LCM CABC control } */

static struct class *backlight_class;

static int backlight_suspend(struct device *dev, pm_message_t state)
{
	struct backlight_device *bd = to_backlight_device(dev);

	mutex_lock(&bd->ops_lock);
	if (bd->ops && bd->ops->options & BL_CORE_SUSPENDRESUME) {
		bd->props.state |= BL_CORE_SUSPENDED;
		backlight_update_status(bd);
	}
	mutex_unlock(&bd->ops_lock);

	return 0;
}

static int backlight_resume(struct device *dev)
{
	struct backlight_device *bd = to_backlight_device(dev);

	mutex_lock(&bd->ops_lock);
	if (bd->ops && bd->ops->options & BL_CORE_SUSPENDRESUME) {
		bd->props.state &= ~BL_CORE_SUSPENDED;
		backlight_update_status(bd);
	}
	mutex_unlock(&bd->ops_lock);

	return 0;
}

static void bl_device_release(struct device *dev)
{
	struct backlight_device *bd = to_backlight_device(dev);
	kfree(bd);
}

static struct device_attribute bl_device_attributes[] = {
	__ATTR(bl_power, 0644, backlight_show_power, backlight_store_power),
	__ATTR(brightness, 0644, backlight_show_brightness,
		     backlight_store_brightness),
	__ATTR(actual_brightness, 0444, backlight_show_actual_brightness,
		     NULL),
	__ATTR(max_brightness, 0444, backlight_show_max_brightness, NULL),
	__ATTR(type, 0444, backlight_show_type, NULL),
	/* JossCheng, 20111222, LCM CABC control { */
	__ATTR(cabc_enable, 0644, backlight_show_cabc, backlight_store_cabc),	
	/* JossCheng, 20111222, LCM CABC control } */
	__ATTR(max_thermal_brightness, 0444, backlight_show_max_thermal_brightness, NULL),
	__ATTR(saved_brightness, 0444, backlight_show_saved_brightness, NULL),
	__ATTR_NULL,
};

/**
 * backlight_force_update - tell the backlight subsystem that hardware state
 *   has changed
 * @bd: the backlight device to update
 *
 * Updates the internal state of the backlight in response to a hardware event,
 * and generate a uevent to notify userspace
 */
void backlight_force_update(struct backlight_device *bd,
			    enum backlight_update_reason reason)
{
	mutex_lock(&bd->ops_lock);
	if (bd->ops && bd->ops->get_brightness)
		bd->props.brightness = bd->ops->get_brightness(bd);
	mutex_unlock(&bd->ops_lock);
	backlight_generate_event(bd, reason);
}
EXPORT_SYMBOL(backlight_force_update);

static int backlight_apply_cooling(struct thermal_dev *dev,
				int level)
{
	struct backlight_device *bd = to_backlight_device(dev->dev);
	int brightness;
	int percent;
	static int previous_cooling_level = 0, new_cooling_level = 0;

	/* transform into percentage */
	percent = thermal_cooling_device_reduction_get(dev, level);
	if (percent < 0 || percent > 100)
		return -EINVAL;
	brightness = (bd->props.max_brightness * percent) / 100;

	mutex_lock(&bd->ops_lock);
	new_cooling_level = level;
	THERMAL_DBG("%s: previous_cooling_level %d, new_cooling_level %d , percent %d, currentbl %d, saved bl %d, thermalmax %lu " ,
	       __FUNCTION__, previous_cooling_level, new_cooling_level, percent,bd->props.brightness, bd->props.saved_brightness, brightness );
	if (bd->ops) {
		bd->props.max_thermal_brightness = brightness;
		if ((new_cooling_level == 0) && (previous_cooling_level == 0) )  {
			/* reached level 0 without hitting throttling down the brightness */
			/* so no reason to change brightness */
			goto exit;

		} else if ( (new_cooling_level > previous_cooling_level) && (bd->props.brightness < brightness) ) {
			/* device is heating up, but new max thermal brightness
			 * is greater than current brightness.
			 * so, we don't need to update the brightness.
			 */
			THERMAL_DBG("%s: device is heating up, but no change is brightness. ",__FUNCTION__);
			goto exit;
		}
		else if ( (new_cooling_level < previous_cooling_level) && (bd->props.saved_brightness < brightness)) {
			/* devices is cooling down, and new max thermal brightness
			 * is greater than previously saved brightness.
			 * So, we need to increase brightness to the saved value not thermal max.
			 */
			bd->props.brightness = bd->props.saved_brightness;
			THERMAL_DBG("%s device is cooling down, restore saved brightness",__FUNCTION__);
		}
		else {
			THERMAL_INFO("brightness transition from %d to %d", bd->props.brightness, brightness);
			bd->props.brightness = brightness;
		}
		backlight_update_status(bd);
		backlight_generate_event(bd, BACKLIGHT_UPDATE_SYSFS);
	}
exit:
	mutex_unlock(&bd->ops_lock);
	previous_cooling_level = new_cooling_level;
	return 0;
}

static struct thermal_dev_ops backlight_cooling_ops = {
	.cool_device = backlight_apply_cooling,
};

static struct thermal_dev case_thermal_dev = {
	.name		= "backlight_cooling",
	.domain_name	= "case",
	.dev_ops	= &backlight_cooling_ops,
};

/**
 * backlight_device_register - create and register a new object of
 *   backlight_device class.
 * @name: the name of the new object(must be the same as the name of the
 *   respective framebuffer device).
 * @parent: a pointer to the parent device
 * @devdata: an optional pointer to be stored for private driver use. The
 *   methods may retrieve it by using bl_get_data(bd).
 * @ops: the backlight operations structure.
 *
 * Creates and registers new backlight device. Returns either an
 * ERR_PTR() or a pointer to the newly allocated device.
 */
struct backlight_device *backlight_device_register(const char *name,
	struct device *parent, void *devdata, const struct backlight_ops *ops,
	const struct backlight_properties *props)
{
	struct backlight_device *new_bd;
	struct thermal_dev *tdev;
	int rc;

	pr_debug("backlight_device_register: name=%s\n", name);

	new_bd = kzalloc(sizeof(struct backlight_device), GFP_KERNEL);
	if (!new_bd)
		return ERR_PTR(-ENOMEM);

	tdev = kzalloc(sizeof(struct thermal_dev), GFP_KERNEL);
	if (!tdev) {
		kfree(new_bd);
		return ERR_PTR(-ENOMEM);
	}

	mutex_init(&new_bd->update_lock);
	mutex_init(&new_bd->ops_lock);

	new_bd->dev.class = backlight_class;
	new_bd->dev.parent = parent;
	new_bd->dev.release = bl_device_release;
	dev_set_name(&new_bd->dev, name);
	dev_set_drvdata(&new_bd->dev, devdata);

	/* Set default properties */
	if (props) {
		memcpy(&new_bd->props, props,
		       sizeof(struct backlight_properties));
		if (props->type <= 0 || props->type >= BACKLIGHT_TYPE_MAX) {
			WARN(1, "%s: invalid backlight type", name);
			new_bd->props.type = BACKLIGHT_RAW;
		}
	} else {
		new_bd->props.type = BACKLIGHT_RAW;
	}

	rc = device_register(&new_bd->dev);
	if (rc) {
		kfree(new_bd);
		kfree(tdev);
		return ERR_PTR(rc);
	}

	rc = backlight_register_fb(new_bd);
	if (rc) {
		kfree(tdev);
		device_unregister(&new_bd->dev);
		return ERR_PTR(rc);
	}

	new_bd->ops = ops;

#ifdef CONFIG_PMAC_BACKLIGHT
	mutex_lock(&pmac_backlight_mutex);
	if (!pmac_backlight)
		pmac_backlight = new_bd;
	mutex_unlock(&pmac_backlight_mutex);
#endif

	memcpy(tdev, &case_thermal_dev, sizeof(struct thermal_dev));
	tdev->dev = &new_bd->dev;
	rc = thermal_cooling_dev_register(tdev);
	if (rc < 0) {
		device_unregister(&new_bd->dev);
		kfree(tdev);
		kfree(new_bd);
		return ERR_PTR(rc);
	}
	new_bd->tdev = tdev;
	new_bd->props.max_thermal_brightness = new_bd->props.max_brightness;
	new_bd->props.saved_brightness = 0;

	return new_bd;
}
EXPORT_SYMBOL(backlight_device_register);

/**
 * backlight_device_unregister - unregisters a backlight device object.
 * @bd: the backlight device object to be unregistered and freed.
 *
 * Unregisters a previously registered via backlight_device_register object.
 */
void backlight_device_unregister(struct backlight_device *bd)
{
	if (!bd)
		return;

#ifdef CONFIG_PMAC_BACKLIGHT
	mutex_lock(&pmac_backlight_mutex);
	if (pmac_backlight == bd)
		pmac_backlight = NULL;
	mutex_unlock(&pmac_backlight_mutex);
#endif
	mutex_lock(&bd->ops_lock);
	bd->ops = NULL;
	mutex_unlock(&bd->ops_lock);

	backlight_unregister_fb(bd);
	device_unregister(&bd->dev);
	thermal_cooling_dev_unregister(bd->tdev);
}
EXPORT_SYMBOL(backlight_device_unregister);

static void __exit backlight_class_exit(void)
{
	class_destroy(backlight_class);
	gpio_free(BKL_CABC_APS_EN);
}

static int __init backlight_class_init(void)
{
	int error;

	backlight_class = class_create(THIS_MODULE, "backlight");
	if (IS_ERR(backlight_class)) {
		printk(KERN_WARNING "Unable to create backlight class; errno = %ld\n",
				PTR_ERR(backlight_class));
		return PTR_ERR(backlight_class);
	}

	backlight_class->dev_attrs = bl_device_attributes;
	backlight_class->suspend = backlight_suspend;
	backlight_class->resume = backlight_resume;
	error = gpio_request(BKL_CABC_APS_EN, "cabc");
	if (error < 0) {
		pr_err("%s:failed to request GPIO %d, error %d\n",
			__func__, BKL_CABC_APS_EN, error);
		return error;
	}
	return 0;
}

/*
 * if this is compiled into the kernel, we need to ensure that the
 * class is registered before users of the class try to register lcd's
 */
postcore_initcall(backlight_class_init);
module_exit(backlight_class_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jamey Hicks <jamey.hicks@hp.com>, Andrew Zabolotny <zap@homelink.ru>");
MODULE_DESCRIPTION("Backlight Lowlevel Control Abstraction");
