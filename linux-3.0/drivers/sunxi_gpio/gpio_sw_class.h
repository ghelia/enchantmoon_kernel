/*
 * 2011-2012
 * panlong <panlong@allwinnertech.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef GPIO_SW_CLASS_H_INCLUDED
#define GPIO_SW_CLASS_H_INCLUDED

#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/rwsem.h>
#include <linux/timer.h>
#include <asm/io.h>
#include "gpio_sw.h"
#define	GPIO_ICR	0x200
#if 0
#define GPIO_SW_DEBUG(fmt...) printk(fmt)
#else
#define GPIO_SW_DEBUG(fmt...) do{} while (0)
#endif

extern struct rw_semaphore gpio_sw_list_lock;
extern struct list_head gpio_sw_list;
int mul_sel_get( struct gpio_sw_classdev *gpio_sw_cdev)
{
	int ret;
	GPIO_SW_DEBUG(" entering mul_sel_get ! \n  ");
	ret = gpio_sw_cdev->gpio_sw_cfg_get(gpio_sw_cdev);
	GPIO_SW_DEBUG(" leaving mul_sel_get ! \n  ");
	return ret;
}

int pull_get(struct gpio_sw_classdev *gpio_sw_cdev)
{
	int ret;
	GPIO_SW_DEBUG(" entering pull_get ! \n  ");
	ret = gpio_sw_cdev->gpio_sw_pull_get(gpio_sw_cdev);
	GPIO_SW_DEBUG(" leaving pull_get ! \n  ");
	return ret;
}

int data_get(struct gpio_sw_classdev *gpio_sw_cdev)
{
	int ret ;
	unsigned int data;
	GPIO_SW_DEBUG(" entering data_get ! \n  ");
	ret = gpio_sw_cdev->gpio_sw_data_get(gpio_sw_cdev);
	data = __raw_readl(GPIO_TEST_BASE + 0x34 ) ;
	GPIO_SW_DEBUG("data is %x ", data);
	GPIO_SW_DEBUG(" leaving data_get ! \n  ");
	return ret;
}

int drv_level_get(struct gpio_sw_classdev *gpio_sw_cdev)
{
	int ret;
	GPIO_SW_DEBUG(" entering drv_level_get ! \n  ");
	ret = gpio_sw_cdev->gpio_sw_drv_level_get(gpio_sw_cdev);
	GPIO_SW_DEBUG("drv_level_get is %d",ret);
	GPIO_SW_DEBUG(" leaving drv_level_get ! \n  ");
	return ret;
}

void mul_sel_set(struct gpio_sw_classdev *gpio_sw_cdev,
					int  mul_sel)
{
	GPIO_SW_DEBUG("  gpio_sw_cdev->mul_sel = %d  \n ", gpio_sw_cdev->mul_sel);
	GPIO_SW_DEBUG("  mul_sel = %d  \n ",mul_sel );
	if ( mul_sel > 1){
		GPIO_SW_DEBUG(" mul_sel max number is 1 ! \n  ");
		return ;
		}else if ( mul_sel < 0){
		GPIO_SW_DEBUG(" mul_sel min number is  0! \n ");
		return ;
		}else {
		GPIO_SW_DEBUG("  set mul_sel = %d now ! \n ",mul_sel);
		gpio_sw_cdev->mul_sel = mul_sel;
	}
	GPIO_SW_DEBUG("  gpio_sw_cdev->mul_sel = %d now ! \n ",gpio_sw_cdev->mul_sel);
	if (!(gpio_sw_cdev->flags & SW_GPIO_SUSPENDED))
		gpio_sw_cdev->gpio_sw_cfg_set(gpio_sw_cdev, mul_sel);
}

void pull_set(struct gpio_sw_classdev *gpio_sw_cdev,
					int  pull)
{
	GPIO_SW_DEBUG("  gpio_sw_cdev->pull = %d  \n ",gpio_sw_cdev->pull);
	GPIO_SW_DEBUG("  pull = %d  \n ",pull );
	if ( pull > 2){
		GPIO_SW_DEBUG(" pull max number is 2 ! \n  ");
		return ;
		}else if ( pull < 0){
		GPIO_SW_DEBUG(" pull min number is 0 ! \n  ");
		return ;
		}else {
		GPIO_SW_DEBUG("  set pull = %d now ! \n ",pull);
		gpio_sw_cdev->pull = pull;
	}
	GPIO_SW_DEBUG("  gpio_sw_cdev->pull = %d now ! \n ",gpio_sw_cdev->pull);
	if (!(gpio_sw_cdev->flags & SW_GPIO_SUSPENDED))
		gpio_sw_cdev->gpio_sw_pull_set(gpio_sw_cdev, pull);
}

void drv_level_set(struct gpio_sw_classdev *gpio_sw_cdev,
					int  drv_level)
{
	GPIO_SW_DEBUG("  gpio_sw_cdev->drv_level = %d \n ",gpio_sw_cdev->drv_level);
	GPIO_SW_DEBUG("  drv_level = %d  \n ",drv_level);
	if ( drv_level > 3){
		GPIO_SW_DEBUG(" drv_level max number is 2 ! \n  ");
		return ;
		}else if ( drv_level < 0){
		GPIO_SW_DEBUG(" drv_level min number is 0 ! \n  ");
		return ;
		}else {
		GPIO_SW_DEBUG("  set drv_level = %d now ! \n ",drv_level);
		gpio_sw_cdev->drv_level = drv_level;
	}
	GPIO_SW_DEBUG("  gpio_sw_cdev->drv_level = %d now ! \n ",gpio_sw_cdev->drv_level);
	if (!(gpio_sw_cdev->flags & SW_GPIO_SUSPENDED))
		gpio_sw_cdev->gpio_sw_drv_level_set(gpio_sw_cdev, drv_level);
}

void data_set(struct gpio_sw_classdev *gpio_sw_cdev,
					int  data)
{
	GPIO_SW_DEBUG("  gpio_sw_cdev->data = %d  \n ",gpio_sw_cdev->data);
	GPIO_SW_DEBUG("  data = %d  \n ",data );
	if ( data > 1){
		GPIO_SW_DEBUG(" data max number is 2 ! \n  ");
		return ;
		}else if ( data < 0){
		GPIO_SW_DEBUG(" data min number is 0 ! \n  ");
		return ;
		}else {
		GPIO_SW_DEBUG("  set data = %d now ! \n ",data);
		gpio_sw_cdev->data = data;
	}
	GPIO_SW_DEBUG("  gpio_sw_cdev->data = %d now ! \n ",gpio_sw_cdev->data);
	if (!(gpio_sw_cdev->flags & SW_GPIO_SUSPENDED))
		gpio_sw_cdev->gpio_sw_data_set(gpio_sw_cdev, data);
}

int set_trigger_types(struct gpio_sw_classdev *gpio_sw_cdev,
					u32  flags)
{

	int flags_tmp,icr_num,icr_area,tmp=0xf;
	GPIO_SW_DEBUG("flags is %d\n",flags);
	if(!gpio_sw_cdev->irq_num){
	GPIO_SW_DEBUG("this pin`s irq is disable \n");
	return 1;
	}
	icr_area	= (gpio_sw_cdev->irq_num - 1) / 8;
	icr_num		= (gpio_sw_cdev->irq_num - 1) % 8;
	flags_tmp	= __raw_readl(GPIO_TEST_BASE + GPIO_ICR + icr_area*0x4);
	tmp = ~ (tmp << icr_num * 4);
	__raw_writel((flags << icr_num * 4) | (flags_tmp & tmp),GPIO_TEST_BASE + GPIO_ICR + icr_area*0x4);
	GPIO_SW_DEBUG("flags_tmp is %x\nnow is %x\nwrite is %x\n",flags_tmp,__raw_readl(GPIO_TEST_BASE + GPIO_ICR + icr_area*0x4),((flags << icr_num * 4) | (flags_tmp & tmp)));

	return 0;
}
#endif
