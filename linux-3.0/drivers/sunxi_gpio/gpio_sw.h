/*
 * 2011-2012
 * panlong <panlong@allwinnertech.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef GPIO_SW_H_INCLUDED
#define GPIO_SW_H_INCLUDED
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/rwsem.h>
#include <linux/timer.h>
#define GPIO_TEST_BASE 0xf1c20800
#define GPIO_INT_BASE 0xf1c20400

struct gpio_sw_platdata {

	unsigned int		flags;
	char				name[16];

};
struct gpio_sw_classdev{
	const char	*name;
	u32	pio_hdle;
	int	port;					/*GPIO使用的端口号 1:PA 2:PB .... */
	int port_num;				/*GPIO在当前端口的序号(第几个引脚)  */
	int mul_sel;				/*GPIO的功能选择 0：输入  1：输出 */
	int pull;					/*GPIO的内置电阻状态 0代表高阻，1代表上拉，2代表下拉 */
	int drv_level;				/*GPIO的驱动能力 有0到3四个等级 */
	int data;					/*GPIO的电平 */
	int flags;
	char irq;
	char irq_num;
	u8 irq_type;

	#define SW_GPIO_TRIGER_POSITIVE		0x0
	#define SW_GPIO_TRIGER_NEGATIVE		0x1
	#define SW_GPIO_TRIGER_HIGH			0x2
	#define SW_GPIO_TRIGER_LOW			0x3
	#define SW_GPIO_TRIGER_DOUBLE		0x4

	#define SW_GPIO_SUSPENDED		(1 << 0)
	#define SW_GPIO_CORE_SUSPENDED		(1 << 16)
	int		(*gpio_sw_cfg_set)(struct gpio_sw_classdev *gpio_sw_cdev,
					int  mul_sel);		/*设置gpio的输入输出状态 */
	int		(*gpio_sw_pull_set)(struct gpio_sw_classdev *gpio_sw_cdev,
					int  pull);			/*设置gpio的电阻是上拉还是高阻或者下拉 */
	int		(*gpio_sw_data_set)(struct gpio_sw_classdev *gpio_sw_cdev,
					int  data);			/*设置gpio的输出电平*/
	int		(*gpio_sw_drv_level_set)(struct gpio_sw_classdev *gpio_sw_cdev,
					int  drv_level);	/*设置gpio的驱动等级 */
	int		(*gpio_sw_cfg_get)(struct gpio_sw_classdev *gpio_sw_cdev);
										/*获取gpio的输入输出 */
	int		(*gpio_sw_pull_get)(struct gpio_sw_classdev *gpio_sw_cdev);
										/*获取gpio的电阻是上拉还是高阻或者下拉 */
	int		(*gpio_sw_data_get)(struct gpio_sw_classdev *gpio_sw_cdev);
										/*获取pio的输出电平*/
	int		(*gpio_sw_drv_level_get)(struct gpio_sw_classdev *gpio_sw_cdev);
										/*获取gpio的驱动等级 */
	struct device		*dev;
	struct list_head	 node;

};

extern void gpio_sw_classdev_suspend(struct gpio_sw_classdev *gpio_sw_cdev);
extern void gpio_sw_classdev_resume(struct gpio_sw_classdev *gpio_sw_cdev);

extern int gpio_sw_classdev_register(struct device *parent,
				struct gpio_sw_classdev *gpio_sw_cdev);
extern void gpio_sw_classdev_unregister(struct gpio_sw_classdev *gpio_sw_cdev);


#endif
