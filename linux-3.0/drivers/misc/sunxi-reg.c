/* driver/misc/sunxi-reg.c
 *
 *  Copyright (C) 2011 Allwinner Technology Co.Ltd
 *  Tom Cubie <tangliang@allwinnertech.com>
 *  update by panlong <panlong@allwinnertech.com> , 2012-4-19 15:39
 *
 *  www.allwinnertech.com
 *
 *  User access to the registers driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/sysdev.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/device.h>

#undef DEBUG_SUNXI

#ifdef DEBUG_SUNXI
#define sunxi_reg_dbg(x...) printk(x)
#else
#define sunxi_reg_dbg(x...)
#endif


static unsigned long 	global_address;
static unsigned int		global_size=1;

static void read_reg(unsigned long addr, int n, unsigned long reg[])
{
	int i;
	for(i=0;i<n;i++)
	reg[i]=readl(addr+i*4);
}

static void write_reg(unsigned long addr, unsigned int value)
{
	writel(value,addr);
}

static ssize_t sunxi_reg_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int i,len=0;
	unsigned long tmp_buff[512];
	if((global_address >= 0xf0000000) && (global_address <= 0xffffffff)){
		read_reg(global_address,global_size,tmp_buff);
		len += sprintf(buf+len,"address\t\t00\t\t04\t\t08\t\t0c\n");
		for(i=0;i<global_size;i++){
			if((i+3) < global_size){
				len += sprintf(buf+len, "0x%.8lx:\t%.8lx\t%.8lx\t%.8lx\t%.8lx\n",global_address+i*4,tmp_buff[i],tmp_buff[i+1],tmp_buff[i+2],tmp_buff[i+3]);
				i+=3;
			}else{
				switch(global_size - i){
					case 1:{
						len += sprintf(buf+len,"0x%.8lx:\t%.8lx\n",global_address+i*4,tmp_buff[i]);
						return  len;
					}
					case 2:{
						len += sprintf(buf+len,"0x%.8lx:\t%.8lx\t%.8lx\n",global_address+i*4,tmp_buff[i],tmp_buff[i+1]);
						return  len;
					}
					case 3:{
						len += sprintf(buf+len,"0x%.8lx:\t%.8lx\t%.8lx\t%.8lx\n",global_address+i*4,tmp_buff[i],tmp_buff[i+1],tmp_buff[i+2]);
						return  len;
					}
				}
			}
		}
	}else{
	printk("the address is invalid!\n");
	}
	return  len;
}

static ssize_t sunxi_reg_addr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0x%lx\n", global_address);
}

static ssize_t sunxi_reg_size_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", global_size);
}

static ssize_t sunxi_reg_addr_store(struct device *dev,struct device_attribute *attr,
		const char *buf, size_t size)
{
	char *after;
	unsigned long address;
	address = simple_strtoull(buf, &after, 16);
	if((address >= 0xf0000000) && (address <= 0xffffffff))
		global_address	= address;
	else{
		printk("Please set valid address(0xf0000000-0xffffffff) first\n");
		return -1;
		}
	return size;
}

static ssize_t sunxi_reg_size_store(struct device *dev,struct device_attribute *attr,
		const char *buf, size_t size)
{
	char *after;
	global_size = simple_strtoul(buf, &after, 10);
	if(!global_size)
			global_size	= 1;
	return size;
}

static ssize_t sunxi_reg_value_store(struct device *dev,struct device_attribute *attr,
		const char *buf, size_t size)
{
	char *after;
	//unsigned int	value;
	int ret=-1,i=0,j=9,k=0,writing=0;
	unsigned long tmp_buff[504];
	char tmp_value[8];
	char *p;
	p=(char *)buf;
	while(*p){
		if(((*p >= '0')&&(*p <= '9')) || ((*p >= 'a')&&(*p <= 'f'))|| ((*p >= 'A')&&(*p <= 'F'))){
			writing=1;
			j--;
			//printk("j is %d\n",j);
			if(!j){
				printk("bits too many!\n");
				return ret;
			}
			tmp_value[i++]	= *p;
			p++;
		}else{
			if(writing){
					if((j<9) && (j>0)){
						j=9;
						tmp_buff[k++]=simple_strtoul(tmp_value, &after, 16);
						while(i)
							tmp_value[--i]=0;
						p++;
					}else if(j == 9){
					p++;
					}else{
						/*don`t execute this code forever unless mem broken*/
						printk("the value of j is err!");
					}
				}else{
				p++;
			}
		}
	}

	if(k != global_size){
		printk("the input size is not equal expect size!\n");
		return ret;
	}
	/*
	for(i=0;i<global_size;i++)
		printk("a[%d] is %.8lx\n",i,tmp_buff[i]);
	*/
	for(i=0;i<global_size;i++)
		write_reg(global_address+i,tmp_buff[i]);

	return size;
}

static DEVICE_ATTR(value, S_IRUGO|S_IWUSR|S_IWGRP,
		sunxi_reg_value_show, sunxi_reg_value_store);
static DEVICE_ATTR(address, S_IRUGO|S_IWUSR|S_IWGRP,
		sunxi_reg_addr_show, sunxi_reg_addr_store);
static DEVICE_ATTR(size, S_IRUGO|S_IWUSR|S_IWGRP,
		sunxi_reg_size_show, sunxi_reg_size_store);

static struct attribute *sunxi_reg_attributes[] = {
	&dev_attr_value.attr,
	&dev_attr_address.attr,
	&dev_attr_size.attr,
	NULL
};

static struct attribute_group sunxi_reg_attribute_group = {
	.name = "rw",
	.attrs = sunxi_reg_attributes
};

static struct miscdevice sunxi_reg_dev = {
	.minor =	MISC_DYNAMIC_MINOR,
	.name =		"sunxi-reg",
};

static int __init sunxi_reg_init(void) {
	int err;

	pr_info("sunxi debug register driver init\n");

	err = misc_register(&sunxi_reg_dev);
	if(err) {
		pr_err("%s register sunxi debug register driver as misc device error\n", __FUNCTION__);
		goto exit;
	}

	err=sysfs_create_group(&sunxi_reg_dev.this_device->kobj,
						 &sunxi_reg_attribute_group);
	if(err){
	pr_err("%s sysfs_create_group  error\n", __FUNCTION__);
	}
exit:
	return err;
}

static void __exit sunxi_reg_exit(void) {

	sunxi_reg_dbg("Bye, sunxi_reg exit\n");
	misc_deregister(&sunxi_reg_dev);
	sysfs_remove_group(&sunxi_reg_dev.this_device->kobj,
						 &sunxi_reg_attribute_group);
}

module_init(sunxi_reg_init);
module_exit(sunxi_reg_exit);

MODULE_DESCRIPTION("a simple sunxi register driver");
MODULE_AUTHOR("Tom Cubie");
MODULE_LICENSE("GPL");

