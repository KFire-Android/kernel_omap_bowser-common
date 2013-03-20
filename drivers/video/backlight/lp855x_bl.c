/*
 * TI LP855x Backlight Driver
 *
 *			Copyright (C) 2011 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/lp855x.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define BRIGHTNESS_CTRL	(0x00)
#define DEVICE_CTRL	(0x01)
#define EEPROM_ADDR0	(0xA0)

#ifdef CONFIG_DEBUG_FS
struct debug_dentry {
	struct dentry *dir;
	struct dentry *reg;
	struct dentry *chip;
	struct dentry *blmode;
};
#endif

struct lp855x {
	const char *chipid;
	struct i2c_client *client;
	struct backlight_device *bl;
	struct device *dev;
	struct mutex xfer_lock;
	struct lp855x_platform_data *pdata;
	u8 last_brightness;
#ifdef CONFIG_DEBUG_FS
	struct debug_dentry dd;
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
       struct early_suspend early_suspend;
#endif
};

#ifdef CONFIG_HAS_EARLYSUSPEND
	static void lp855x_early_suspend(struct early_suspend *es);
	static void lp855x_late_resume(struct early_suspend *es);
#endif

static int lp855x_i2c_read(struct lp855x *lp, u8 reg, u8 *data, u8 len)
{
	s32 ret;

	mutex_lock(&lp->xfer_lock);
	ret = i2c_smbus_read_i2c_block_data(lp->client, reg, len, data);
	mutex_unlock(&lp->xfer_lock);

	return (ret != len) ? -EIO : 0;
}

static int lp855x_i2c_write(struct lp855x *lp, u8 reg, u8 *data, u8 len)
{
	s32 ret;

	mutex_lock(&lp->xfer_lock);
	ret = i2c_smbus_write_i2c_block_data(lp->client, reg, len, data);
	mutex_unlock(&lp->xfer_lock);

	return ret;
}

static inline int lp855x_read_byte(struct lp855x *lp, u8 reg, u8 *data)
{
	return lp855x_i2c_read(lp, reg, data, 1);
}

static inline int lp855x_write_byte(struct lp855x *lp, u8 reg, u8 data)
{
	u8 written = data;
	return lp855x_i2c_write(lp, reg, &written, 1);
}

#ifdef CONFIG_DEBUG_FS
static int lp855x_dbg_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t lp855x_help_register(struct file *file, char __user *userbuf,
				    size_t count, loff_t *ppos)
{
	char buf[320];
	unsigned int len;
	const char *help = "\n How to read/write LP855x registers\n\n \
	(example) To read 0x00 register,\n \
	echo 0x00 r > /sys/kernel/debug/lp855x/registers\n \
	To write 0xff into 0x1 address,\n \
	echo 0x00 0xff w > /sys/kernel/debug/lp855x/registers \n \
	To dump values from 0x00 to 0x06 address,\n \
	echo 0x00 0x06 d > /sys/kernel/debug/lp855x/registers\n";

	len = snprintf(buf, sizeof(buf), "%s\n", help);
	if (len > sizeof(buf))
		len = sizeof(buf);

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static char *lp855x_parse_register_cmd(const char *cmd, u8 *byte)
{
	char tmp[10];
	char *blank;
	unsigned long arg;

	blank = strchr(cmd, ' ');
	memset(tmp, 0x0, sizeof(tmp));
	memcpy(tmp, cmd, blank - cmd);

	if (strict_strtol(tmp, 16, &arg) < 0)
		return NULL;

	*byte = arg;
	return blank;
}

static ssize_t lp855x_ctrl_register(struct file *file,
				    const char __user *userbuf, size_t count,
				    loff_t *ppos)
{
	char mode, buf[20];
	char *pos, *pos2;
	u8 i, arg1, arg2, val;
	struct lp855x *lp = file->private_data;

	if (copy_from_user(buf, userbuf, min(count, sizeof(buf))))
		return -EFAULT;

	mode = buf[count - 2];
	switch (mode) {
	case 'r':
		if (!lp855x_parse_register_cmd(buf, &arg1))
			return -EINVAL;

		lp855x_read_byte(lp, arg1, &val);
		dev_info(lp->dev, "Read [0x%.2x] = 0x%.2x\n", arg1, val);
		break;
	case 'w':
		pos = lp855x_parse_register_cmd(buf, &arg1);
		if (!pos)
			return -EINVAL;
		pos2 = lp855x_parse_register_cmd(pos + 1, &arg2);
		if (!pos2)
			return -EINVAL;

		lp855x_write_byte(lp, arg1, arg2);
		dev_info(lp->dev, "Written [0x%.2x] = 0x%.2x\n", arg1, arg2);
		break;
	case 'd':
		pos = lp855x_parse_register_cmd(buf, &arg1);
		if (!pos)
			return -EINVAL;
		pos2 = lp855x_parse_register_cmd(pos + 1, &arg2);
		if (!pos2)
			return -EINVAL;

		for (i = arg1; i <= arg2; i++) {
			lp855x_read_byte(lp, i, &val);
			dev_info(lp->dev, "Read [0x%.2x] = 0x%.2x\n", i, val);
		}
		break;
	default:
		break;
	}

	return count;
}

static ssize_t lp855x_get_chipid(struct file *file, char __user *userbuf,
				 size_t count, loff_t *ppos)
{
	struct lp855x *lp = file->private_data;
	char buf[10];
	unsigned int len;

	len = snprintf(buf, sizeof(buf), "%s\n", lp->chipid);

	if (len > sizeof(buf))
		len = sizeof(buf);

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t lp855x_get_bl_mode(struct file *file, char __user *userbuf,
				  size_t count, loff_t *ppos)
{
	char buf[20];
	unsigned int len;
	char *strmode = NULL;
	struct lp855x *lp = file->private_data;
	enum lp855x_brightness_ctrl_mode mode = lp->pdata->mode;

	if (mode == PWM_BASED)
		strmode = "pwm based";
	else if (mode == REGISTER_BASED)
		strmode = "register based";

	len = snprintf(buf, sizeof(buf), "%s\n", strmode);

	if (len > sizeof(buf))
		len = sizeof(buf);

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

#define LP855X_DBG_ENTRY(name, pread, pwrite) \
static const struct file_operations dbg_##name##_fops = { \
	.open = lp855x_dbg_open, \
	.read = pread, \
	.write = pwrite, \
	.owner = THIS_MODULE, \
	.llseek = default_llseek, \
}

LP855X_DBG_ENTRY(registers, lp855x_help_register, lp855x_ctrl_register);
LP855X_DBG_ENTRY(chip, lp855x_get_chipid, NULL);
LP855X_DBG_ENTRY(blmode, lp855x_get_bl_mode, NULL);

static void lp855x_create_debugfs(struct lp855x *lp)
{
	struct debug_dentry *dd = &lp->dd;

	dd->dir = debugfs_create_dir("lp855x", NULL);

	dd->reg = debugfs_create_file("registers", S_IWUSR | S_IRUGO,
				      dd->dir, lp, &dbg_registers_fops);

	dd->chip = debugfs_create_file("chip_id", S_IRUGO,
				       dd->dir, lp, &dbg_chip_fops);

	dd->blmode = debugfs_create_file("bl_ctl_mode", S_IRUGO,
					 dd->dir, lp, &dbg_blmode_fops);
}

static void lp855x_remove_debugfs(struct lp855x *lp)
{
	struct debug_dentry *dd = &lp->dd;

	debugfs_remove(dd->blmode);
	debugfs_remove(dd->chip);
	debugfs_remove(dd->reg);
	debugfs_remove(dd->dir);
}
#else
static inline void lp855x_create_debugfs(struct lp855x *lp)
{
	return;
}

static inline void lp855x_remove_debugfs(struct lp855x *lp)
{
	return;
}
#endif

static int lp855x_is_valid_rom_area(struct lp855x *lp, u8 addr)
{
	const char *id = lp->chipid;
	u8 start, end;

	if (strstr(id, "lp8550") || strstr(id, "lp8551")
	    || strstr(id, "lp8552") || strstr(id, "lp8553")) {
		start = EEPROM_START;
		end = EEPROM_END;
	} else if (strstr(id, "lp8556")) {
		start = EPROM_START;
		end = EPROM_END;
	}

	return (addr >= start && addr <= end) ? 1 : 0;
}

static void lp855x_init_device(struct lp855x *lp)
{
	u8 val, addr;
	int i, ret = 0;
	struct lp855x_platform_data *pd = lp->pdata;

	lp->last_brightness = 0;

	if (pd->gpio_en >= 0) {
		ret = gpio_request(pd->gpio_en, "backlight_lp855x_gpio");
		if (ret != 0) {
			pr_err("%s:%s backlight gpio request failed\n",__FILE__, __FUNCTION__);
	                pd->gpio_en = -EINVAL; /* serve as a flag as well */
		}
	}
	gpio_direction_output(pd->gpio_en, 1);
	msleep(20);

	/* Setting BRIGHTNESS_CTRL and DEVICE_CTRL to default mode */
	val = 0;
	ret = lp855x_write_byte(lp, BRIGHTNESS_CTRL, val);

	ret |= lp855x_write_byte(lp, DEVICE_CTRL, val);

	/* Use the initial brightness to set current */
	val = pd->initial_brightness;
	ret |= lp855x_write_byte(lp, EEPROM_ADDR0, val);

	if (pd->load_new_rom_data && pd->size_program) {
		for (i = 0; i < pd->size_program; i++) {
			addr = pd->rom_data[i].addr;
			val = pd->rom_data[i].val;
			if (!lp855x_is_valid_rom_area(lp, addr))
				continue;

			ret |= lp855x_write_byte(lp, addr, val);
		}
	}

	if (ret)
		dev_err(lp->dev, "i2c write err\n");
}

/* LCD and Back light ordering.... */
extern wait_queue_head_t panel_init_queue;
extern wait_queue_head_t panel_fini_queue;
extern int nt51012_panel_enabled;
extern int lp855x_bl_off;
static int lp855x_bl_update_status(struct backlight_device *bl)
{
	u8 addr, val;
	int i, ret = 0;
	struct lp855x *lp = bl_get_data(bl);
	struct lp855x_platform_data *pd = lp->pdata;
	enum lp855x_brightness_ctrl_mode mode = lp->pdata->mode;
	int gpio_en = lp->pdata->gpio_en;
	u8 last_brightness = lp->last_brightness;
	u8 brightness = bl->props.brightness;


	if (bl->props.state & BL_CORE_SUSPENDED)
		bl->props.brightness = 0;

	if ((!last_brightness) && (brightness)) {
		/* Power Up */
		/* wait until LCD is panel is ready */
		wait_event_timeout(panel_init_queue, nt51012_panel_enabled, msecs_to_jiffies(500));

		if (nt51012_panel_enabled == 0) {
			/* wait_event_timeout timed out! 
			 * It is possible that nt51012_panel_enable is not set to zero at 
			 * panel driver nt51012_power_off(). In that case, we need to turn BL back on.
			 */
			nt51012_panel_enabled = 1;
			goto exit;
		}

		/* to prevent BL remains on while DSSCOMP: blanked screen */
		nt51012_panel_enabled = 0;
		gpio_set_value(gpio_en, 1);

		msleep(20);

		if (pd->load_new_rom_data && pd->size_program) {
			for (i = 0; i < pd->size_program; i++) {
				addr = pd->rom_data[i].addr;
				val = pd->rom_data[i].val;
				if (!lp855x_is_valid_rom_area(lp, addr))
					continue;

				ret |= lp855x_write_byte(lp, addr, val);
			}
		}
		if(ret)
			dev_err(lp->dev, "i2c write err\n");

	}
	else if ((last_brightness) && (!brightness)) {
		/* Power Down */
		gpio_set_value(gpio_en, 0);
		/*backlight is off, so wake up lcd disable */
		lp855x_bl_off  = 1;
		wake_up(&panel_fini_queue);
	}
	else if ( last_brightness == brightness) {
		/*nothing to do*/
		goto exit;
	}

	if (mode == PWM_BASED) {
		struct lp855x_pwm_data *pd = &lp->pdata->pwm_data;
		int br = bl->props.brightness;
		int max_br = bl->props.max_brightness;

		if (pd->pwm_set_intensity)
			pd->pwm_set_intensity(br, max_br);

	} else if (mode == REGISTER_BASED) {
		u8 val = bl->props.brightness;
		lp855x_write_byte(lp, EEPROM_ADDR0, val);
	}

	lp->last_brightness = bl->props.brightness;
exit:
	return (bl->props.brightness);
}

static int lp855x_bl_get_brightness(struct backlight_device *bl)
{
	struct lp855x *lp = bl_get_data(bl);
	enum lp855x_brightness_ctrl_mode mode = lp->pdata->mode;

	if (mode == PWM_BASED) {
		struct lp855x_pwm_data *pd = &lp->pdata->pwm_data;
		int max_br = bl->props.max_brightness;

		if (pd->pwm_get_intensity)
			bl->props.brightness = pd->pwm_get_intensity(max_br);

	} else if (mode == REGISTER_BASED) {
		u8 val = 0;

		lp855x_read_byte(lp, EEPROM_ADDR0, &val);
		bl->props.brightness = val;
	}

	return (bl->props.brightness);
}

static const struct backlight_ops lp855x_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = lp855x_bl_update_status,
	.get_brightness = lp855x_bl_get_brightness,
};

static int lp855x_backlight_register(struct lp855x *lp)
{
	struct backlight_device *bl;
	struct backlight_properties props;
	const char *name = lp->pdata->name;

	if (!name)
		return -ENODEV;

	props.brightness = lp->pdata->initial_brightness;
	props.max_brightness =
		(lp->pdata->max_brightness < lp->pdata->initial_brightness) ?
		255 : lp->pdata->max_brightness;
	props.type = BACKLIGHT_RAW;

	bl = backlight_device_register(name, lp->dev, lp,
				       &lp855x_bl_ops, &props);
	if (IS_ERR(bl))
		return -EIO;

	lp->bl = bl;

	return 0;
}

static void lp855x_backlight_unregister(struct lp855x *lp)
{
	if (lp->bl)
		backlight_device_unregister(lp->bl);
}

static int lp855x_probe(struct i2c_client *cl, const struct i2c_device_id *id)
{
	struct lp855x *lp;
	struct lp855x_platform_data *pdata = cl->dev.platform_data;
	int ret;

	if (!i2c_check_functionality(cl->adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
		goto err_io;

	lp = kzalloc(sizeof(struct lp855x), GFP_KERNEL);
	if (!lp)
		goto err_mem;

	lp->client = cl;
	lp->dev = &cl->dev;
	lp->pdata = pdata;
	lp->chipid = id->name;
	lp->last_brightness = 0;
	i2c_set_clientdata(cl, lp);

	mutex_init(&lp->xfer_lock);


	lp855x_init_device(lp);
	ret = lp855x_backlight_register(lp);
	if (ret)
		goto err_dev;

	backlight_update_status(lp->bl);
	lp855x_create_debugfs(lp);

#ifdef CONFIG_HAS_EARLYSUSPEND
	lp->early_suspend.suspend = lp855x_early_suspend;
	lp->early_suspend.resume = lp855x_late_resume;
	register_early_suspend(&lp->early_suspend);
#endif

	return ret;

err_io:
	return -EIO;
err_mem:
	return -ENOMEM;
err_dev:
	dev_err(lp->dev, "can not register backlight device. errcode = %d\n",
		ret);
	kfree(lp);
	return ret;
}

static int __devexit lp855x_remove(struct i2c_client *cl)
{
	struct lp855x *lp = i2c_get_clientdata(cl);

	lp->bl->props.brightness = 0;
	backlight_update_status(lp->bl);
	lp855x_remove_debugfs(lp);
	lp855x_backlight_unregister(lp);
	kfree(lp);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&lp->early_suspend);
#endif

	return 0;
}

#ifdef CONFIG_PM
static int lp855x_suspend(struct i2c_client *client,pm_message_t mesg)
{
	struct lp855x *lp = i2c_get_clientdata(client);

	mutex_lock(&lp->bl->update_lock);

	lp->last_brightness = lp->bl->props.brightness;
	lp->bl->props.brightness = 0;
	mutex_unlock(&lp->bl->update_lock);
	backlight_update_status(lp->bl);

	if (lp->pdata->gpio_en == -1)
		return -1;

	gpio_set_value(lp->pdata->gpio_en, 0);

	return 0;
}

static int lp855x_resume(struct i2c_client *client)
{
	u8 val, addr;
	int i, ret = 0;
	struct lp855x *lp = i2c_get_clientdata(client);
	struct lp855x_platform_data *pd = lp->pdata;

	if (lp->pdata->gpio_en == -1)
		return -1;

	gpio_set_value(lp->pdata->gpio_en, 1);
	msleep(20);

	if (pd->load_new_rom_data && pd->size_program) {
		for (i = 0; i < pd->size_program; i++) {
			addr = pd->rom_data[i].addr;
			val = pd->rom_data[i].val;
			if (!lp855x_is_valid_rom_area(lp, addr))
				continue;

			ret |= lp855x_write_byte(lp, addr, val);
		}
	}

	if (ret)
		dev_err(lp->dev, "i2c write err\n");

	mutex_lock(&lp->bl->update_lock);
	lp->bl->props.brightness = lp->last_brightness;

	mutex_unlock(&lp->bl->update_lock);
	backlight_update_status(lp->bl);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void lp855x_early_suspend(struct early_suspend *es)
{

	struct lp855x *lp;
	lp = container_of(es, struct lp855x, early_suspend);

	if (lp855x_suspend(lp->client, PMSG_SUSPEND) != 0)
		dev_err(lp->dev, "%s: failed\n", __func__);

}

static void lp855x_late_resume(struct early_suspend *es)
{

	struct lp855x *lp;
	lp = container_of(es, struct lp855x, early_suspend);

	if (lp855x_resume(lp->client) != 0)
		dev_err(lp->dev, "%s: failed\n", __func__);
}

#else
static const struct dev_pm_ops lp855x_pm_ops = {
	.suspend	= lp855x_suspend,
	.resume		= lp855x_resume,
};
#endif
#endif

static const struct i2c_device_id lp855x_ids[] = {
	{"lp8550", LP8550},
	{"lp8551", LP8551},
	{"lp8552", LP8552},
	{"lp8553", LP8553},
	{"lp8556", LP8556},
};


static struct i2c_driver lp855x_driver = {
	.driver = {
		   .name = "lp855x",
		   },
	.probe = lp855x_probe,
#ifndef CONFIG_HAS_EARLYSUSPEND
		.pm	= &lp855x_pm_ops,
#endif
	.remove = __devexit_p(lp855x_remove),
	.id_table = lp855x_ids,
};

static int __init lp855x_init(void)
{
	return i2c_add_driver(&lp855x_driver);
}

static void __exit lp855x_exit(void)
{
	i2c_del_driver(&lp855x_driver);
}

module_init(lp855x_init);
module_exit(lp855x_exit);

MODULE_DESCRIPTION("Texas Instruments LP855x Backlight driver");
MODULE_AUTHOR("Milo Kim <milo.kim@ti.com>, Dainel Jeong <daniel.jeong@ti.com>");
MODULE_LICENSE("GPL");
