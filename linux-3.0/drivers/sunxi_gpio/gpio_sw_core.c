/*
 * 2011-2012
 * panlong <panlong@allwinnertech.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/rwsem.h>

DECLARE_RWSEM(gpio_sw_list_lock);
EXPORT_SYMBOL_GPL(gpio_sw_list_lock);

LIST_HEAD(gpio_sw_list);
EXPORT_SYMBOL_GPL(gpio_sw_list);