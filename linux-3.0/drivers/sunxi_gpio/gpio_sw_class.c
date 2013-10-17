/*
 * 2011-2012
 * panlong <panlong@allwinnertech.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <mach/sys_config.h>
#include "gpio_sw.h"
#include "gpio_sw_class.h"
static struct class *gpio_sw_class;

static ssize_t mul_sel_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gpio_sw_classdev *gpio_sw_cdev = dev_get_drvdata(dev);
	gpio_sw_cdev->mul_sel = mul_sel_get(gpio_sw_cdev);
	return sprintf(buf, "%u\n", gpio_sw_cdev->mul_sel);
}

static ssize_t pull_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gpio_sw_classdev *gpio_sw_cdev = dev_get_drvdata(dev);
	gpio_sw_cdev->pull = pull_get(gpio_sw_cdev);
	return sprintf(buf, "%u\n", gpio_sw_cdev->pull);
}

static ssize_t drv_level_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gpio_sw_classdev *gpio_sw_cdev = dev_get_drvdata(dev);
	gpio_sw_cdev->drv_level = drv_level_get(gpio_sw_cdev);
	return sprintf(buf, "%u\n", gpio_sw_cdev->drv_level);
}

static ssize_t data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gpio_sw_classdev *gpio_sw_cdev = dev_get_drvdata(dev);
	gpio_sw_cdev->data = data_get(gpio_sw_cdev);
	return sprintf(buf, "%u\n", gpio_sw_cdev->data);
}

static ssize_t mul_sel_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct gpio_sw_classdev *gpio_sw_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	char *after;
	int in_out = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;
		mul_sel_set(gpio_sw_cdev, in_out);
	}

	return ret;
}

static ssize_t pull_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct gpio_sw_classdev *gpio_sw_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	char *after;
	int pull = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;
		pull_set(gpio_sw_cdev, pull);
	}

	return ret;
}

static ssize_t drv_level_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct gpio_sw_classdev *gpio_sw_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	char *after;
	int drv_level = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;

		drv_level_set(gpio_sw_cdev, drv_level);
	}

	return ret;
}

static ssize_t data_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct gpio_sw_classdev *gpio_sw_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	char *after;
	int data = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;
		data_set(gpio_sw_cdev, data);
	}

	return ret;
}

void gpio_sw_classdev_suspend(struct gpio_sw_classdev *gpio_sw_cdev)
{
	gpio_sw_cdev->flags |= SW_GPIO_SUSPENDED;
	GPIO_SW_DEBUG("gpio_sw_classdev_suspend OK!");
}
EXPORT_SYMBOL_GPL(gpio_sw_classdev_suspend);

void gpio_sw_classdev_resume(struct gpio_sw_classdev *gpio_sw_cdev)
{
	GPIO_SW_DEBUG("gpio_sw_classdev_resume OK!");
	gpio_sw_cdev->flags &= ~SW_GPIO_SUSPENDED;
}
EXPORT_SYMBOL_GPL(gpio_sw_classdev_resume);

static int gpio_sw_suspend(struct device *dev, pm_message_t state)
{
	struct gpio_sw_classdev *gpio_sw_cdev = dev_get_drvdata(dev);
		GPIO_SW_DEBUG("gpio class suspend \n");
	if (gpio_sw_cdev->flags & SW_GPIO_CORE_SUSPENDED)
		gpio_sw_classdev_suspend(gpio_sw_cdev);

	return 0;
}

static int gpio_sw_resume(struct device *dev)
{
	struct gpio_sw_classdev *gpio_sw_cdev = dev_get_drvdata(dev);
		GPIO_SW_DEBUG("gpio class resume \n");
	if (gpio_sw_cdev->flags & SW_GPIO_CORE_SUSPENDED)
		gpio_sw_classdev_resume(gpio_sw_cdev);

	return 0;
}

static const struct {
	const char *name;
	u32 flags;
}trigger_types[] = {
	{ "positive",	SW_GPIO_TRIGER_POSITIVE },
	{ "negative",	SW_GPIO_TRIGER_NEGATIVE},
	{ "high",		SW_GPIO_TRIGER_HIGH },
	{ "low",		SW_GPIO_TRIGER_LOW },
	{ "double",		SW_GPIO_TRIGER_DOUBLE},
};

static ssize_t trigger_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gpio_sw_classdev *gpio_sw_cdev = dev_get_drvdata(dev);
	int i,len=0;

	if (!gpio_sw_cdev->irq)
		len += sprintf(buf+len, "[none] ");
	else
		len += sprintf(buf+len, "none ");

	for (i = 0; i < ARRAY_SIZE(trigger_types); i++){
		if(gpio_sw_cdev->irq_type == trigger_types[i].flags)
		len += sprintf(buf+len, "[%s] ", trigger_types[i].name);
		else
		len += sprintf(buf+len, "%s ", trigger_types[i].name);
	}

	len += sprintf(buf+len, "\n");
	return  len;
}

static ssize_t trigger_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct gpio_sw_classdev *gpio_sw_cdev = dev_get_drvdata(dev);
	int i,ret=-1;

	for (i = 0; i < ARRAY_SIZE(trigger_types); i++){
		GPIO_SW_DEBUG("trigger_types[i].name is %s\n",trigger_types[i].name);
		GPIO_SW_DEBUG("trigger_types[i].name is %s\n",buf);
		if(sysfs_streq(buf,trigger_types[i].name)){
		ret = set_trigger_types(gpio_sw_cdev, trigger_types[i].flags);
		gpio_sw_cdev->irq_type=trigger_types[i].flags;
		break;
		}
	}
	GPIO_SW_DEBUG("loop is %d\n",ARRAY_SIZE(trigger_types));
	return size;
}

static struct device_attribute gpio_sw_class_attrs[] = {
	__ATTR(mul_sel, 0644, mul_sel_show, mul_sel_store),
	__ATTR(pull, 0644, pull_show, pull_store),
	__ATTR(drv_level, 0644, drv_level_show, drv_level_store),
	__ATTR(data, 0644, data_show, data_store),
	__ATTR(trigger, 0644,trigger_show,trigger_store),
	__ATTR_NULL,
};

int gpio_sw_classdev_register(struct device *parent, struct gpio_sw_classdev *gpio_sw_cdev)
{
	gpio_sw_cdev->dev = device_create(gpio_sw_class, parent, 0, gpio_sw_cdev,
				      "%s", gpio_sw_cdev->name);
	if (IS_ERR(gpio_sw_cdev->dev))
		return PTR_ERR(gpio_sw_cdev->dev);
	down_write(&gpio_sw_list_lock);
	list_add_tail(&gpio_sw_cdev->node, &gpio_sw_list);
	up_write(&gpio_sw_list_lock);
	printk(KERN_DEBUG "Registered gpio_sw device: %s\n",
			gpio_sw_cdev->name);
	return 0;
}
EXPORT_SYMBOL_GPL(gpio_sw_classdev_register);

void gpio_sw_classdev_unregister(struct gpio_sw_classdev *gpio_sw_cdev)
{
	device_unregister(gpio_sw_cdev->dev);
	down_write(&gpio_sw_list_lock);
	list_del(&gpio_sw_cdev->node);
	up_write(&gpio_sw_list_lock);
}
EXPORT_SYMBOL_GPL(gpio_sw_classdev_unregister);

static int __init gpio_sw_init(void)
{
	gpio_sw_class = class_create(THIS_MODULE, "gpio_sw");
	if (IS_ERR(gpio_sw_class))
		return PTR_ERR(gpio_sw_class);
	gpio_sw_class->suspend		= gpio_sw_suspend;
	gpio_sw_class->resume		= gpio_sw_resume;
	gpio_sw_class->dev_attrs 	= gpio_sw_class_attrs;
	return 0;
}

static void __exit gpio_sw_exit(void)
{
	class_destroy(gpio_sw_class);
}

subsys_initcall(gpio_sw_init);
module_exit(gpio_sw_exit);

MODULE_AUTHOR("panlong <panlong@allwinnertech.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("sw_gpio Class Interface");
