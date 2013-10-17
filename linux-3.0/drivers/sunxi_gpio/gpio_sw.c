/*
 * 2011-2012
 * panlong <panlong@allwinnertech.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <mach/sys_config.h>
#include <asm/io.h>
#include "gpio_sw.h"
#define mul_sel_1	1
#define pull_2 		2
#define drv_level_3	3
#define data_4		4
#define port_5		5
#define port_num_6	6
#define REG_RD(fmt...)	__raw_readl(fmt)

#if 0
#define GPIO_SW_DEBUG(fmt...) printk(fmt)
#else
#define GPIO_SW_DEBUG(fmt...) do{} while (0)
#endif

#define GPIO_IRQ  28

int all_irq_enable	= 0;
static struct platform_device gpio_sw_dev[256];
static struct gpio_sw_platdata pdatesw[256];
/*
void sunxi_gpio_do_tasklet(unsigned long data)
{
	printk("this is irp donw dispuse !\n");
}
DECLARE_TASKLET(sunxi_tasklet,sunxi_gpio_do_tasklet,0);
*/
irqreturn_t sunxi_interrupt(int irq,void *dev_id)
{
	unsigned int PIC, PIS,tmp;
	int i = 0;
	PIC = REG_RD(GPIO_TEST_BASE + 0x210 ) ;
	PIS = REG_RD(GPIO_TEST_BASE + 0x214 ) ;
	tmp	= PIS;

	while(tmp){
		if(tmp & 0x1){
		/*if (tmp & 0x1) is true, the i represent NO.i EINT interrupt take place.
		you can through the value of i to decide to do what*/
		printk("this is NO.%d gpio INT \n",i);
		}
		tmp >>= 1;
		i++;
	}

	GPIO_SW_DEBUG("0 PIC is %x \n PIS is %x \n",PIC,PIS);
	__raw_writel(PIS, GPIO_TEST_BASE + 0x214);
	GPIO_SW_DEBUG("1 PIC is %x \n PIS is %x \n",PIC,PIS);

	/*this is a interface to connect interrupt top half and bottom half,if want to use bottom half,you can open fanctions sunxi_gpio_do_tasklet and tasklet_schedule*/

	/*
	tasklet_schedule(&sunxi_tasklet);
	*/
	return IRQ_HANDLED;
}

int get_gpio_member_value( u32 gpio_hd , char *name , int member_name )
{
	user_gpio_set_t  gpio_info[1];
	int  ret;
    GPIO_SW_DEBUG("fetch gpio_hd is %x \n",gpio_hd);
	GPIO_SW_DEBUG("fetch name is %s \n",name);
    ret = gpio_get_one_pin_status(gpio_hd, gpio_info, name, 1);
    if(ret < 0){
        GPIO_SW_DEBUG("fetch gpio infomation fail \n");
	}
	else{
        GPIO_SW_DEBUG("fetch gpio infomation ok \n");
	}
	GPIO_SW_DEBUG("fetch ret0 is %d \n",ret);
	switch ( member_name )
		{
		case mul_sel_1:
		ret = gpio_info->mul_sel;
		break;
		case pull_2:	
		ret = gpio_info->pull;
		break;
		case drv_level_3:		
		ret = gpio_info->drv_level;
		break;
		case data_4:
		ret = gpio_info->data;
		break;
		case port_5:
		ret = gpio_info->port;
		break;
		case port_num_6:
		ret = gpio_info->port_num;
		break;	
		}
	return ret;

}

struct gpio_sw {
	struct gpio_sw_classdev		cdev;
	struct gpio_sw_platdata		*pdata;
};

static inline struct gpio_sw *pdev_to_gpio(struct platform_device *dev)
{
	return platform_get_drvdata(dev);
}

static inline struct gpio_sw *to_gpio(struct gpio_sw_classdev *gpio_sw_cdev)
{
	return container_of(gpio_sw_cdev, struct gpio_sw, cdev);
}

static int	gpio_sw_cfg_set(struct gpio_sw_classdev *gpio_sw_cdev,int  mul_sel)
{
	struct gpio_sw *gpio = to_gpio(gpio_sw_cdev);
	struct gpio_sw_platdata *pd = gpio->pdata;
	int ret ;

	GPIO_SW_DEBUG("attending gpio_sw_cfg_set \n");
	GPIO_SW_DEBUG("pio_hdle is %x \n",gpio->cdev.pio_hdle);

	ret =   gpio_set_one_pin_io_status(gpio->cdev.pio_hdle, mul_sel, pd->name);
	if ( !ret )
	gpio_sw_cdev->mul_sel=mul_sel;
	GPIO_SW_DEBUG("left gpio_sw_cfg_set \n");
	return ret;
}

static int	gpio_sw_pull_set(struct gpio_sw_classdev *gpio_sw_cdev,int  pull)
{
	struct gpio_sw *gpio = to_gpio(gpio_sw_cdev);
	struct gpio_sw_platdata *pd = gpio->pdata;
	int ret ;

	GPIO_SW_DEBUG("attending gpio_sw_pull_set \n");
	GPIO_SW_DEBUG("pio_hdle is %x \n",gpio->cdev.pio_hdle);

	ret =   gpio_set_one_pin_pull (gpio->cdev.pio_hdle, pull, pd->name);
	if ( !ret )
	gpio_sw_cdev->pull=pull;
	GPIO_SW_DEBUG("left gpio_sw_pull_set \n");
	return ret ;
}

static int	gpio_sw_data_set(struct gpio_sw_classdev *gpio_sw_cdev,int  data)
{
	struct gpio_sw *gpio = to_gpio(gpio_sw_cdev);
	struct gpio_sw_platdata *pd = gpio->pdata;
	int ret ;

	GPIO_SW_DEBUG("attending gpio_sw_data_set \n");
	GPIO_SW_DEBUG("pio_hdle is %x \n",gpio->cdev.pio_hdle);

	ret =  gpio_write_one_pin_value (gpio->cdev.pio_hdle, data, pd->name);
	if ( !ret )
	gpio_sw_cdev->data=data;
	GPIO_SW_DEBUG("left gpio_sw_data_set \n");
	return ret ;
}

static int	gpio_sw_drv_level_set(struct gpio_sw_classdev *gpio_sw_cdev,int  drv_level)
{
	struct gpio_sw *gpio = to_gpio(gpio_sw_cdev);
	struct gpio_sw_platdata *pd = gpio->pdata;
	int ret ;

	GPIO_SW_DEBUG("attending gpio_sw_drv_level_set \n");
	GPIO_SW_DEBUG("pio_hdle is %x \n",gpio->cdev.pio_hdle);

	ret =  gpio_set_one_pin_driver_level (gpio->cdev.pio_hdle, drv_level, pd->name);
	if ( !ret )
	gpio_sw_cdev->drv_level=drv_level;
	GPIO_SW_DEBUG("left gpio_sw_drv_level_set \n");
	return ret ;
}

static int	gpio_sw_cfg_get(struct gpio_sw_classdev *gpio_sw_cdev)
{
	struct gpio_sw *gpio = to_gpio(gpio_sw_cdev);
	struct gpio_sw_platdata *pd = gpio->pdata;
	int ret;
	ret=get_gpio_member_value( gpio->cdev.pio_hdle , pd->name , mul_sel_1 );
	return ret;
}

static int	gpio_sw_pull_get(struct gpio_sw_classdev *gpio_sw_cdev)
{
	struct gpio_sw *gpio = to_gpio(gpio_sw_cdev);
	struct gpio_sw_platdata *pd = gpio->pdata;
	int ret;
	ret=get_gpio_member_value( gpio->cdev.pio_hdle , pd->name , pull_2 );
	return ret;
}

static int	gpio_sw_data_get(struct gpio_sw_classdev *gpio_sw_cdev)
{
	struct gpio_sw *gpio = to_gpio(gpio_sw_cdev);
	struct gpio_sw_platdata *pd = gpio->pdata;
	int ret;
	ret=get_gpio_member_value( gpio->cdev.pio_hdle , pd->name , data_4 );
	return ret;
}

static int	gpio_sw_drv_level_get(struct gpio_sw_classdev *gpio_sw_cdev)
{
	struct gpio_sw *gpio = to_gpio(gpio_sw_cdev);
	struct gpio_sw_platdata *pd = gpio->pdata;
	int ret;
	ret=get_gpio_member_value( gpio->cdev.pio_hdle , pd->name , drv_level_3 );
	GPIO_SW_DEBUG("in gpio_sw_drv_level_get ret is %d",ret);
	return ret;
}

static int  gpio_sw_put_resource(struct gpio_sw *gpio)
{
	GPIO_SW_DEBUG("attending gpio_sw_put_resource \n");
	GPIO_SW_DEBUG("pio_hdle is %x \n",gpio->cdev.pio_hdle);
	gpio_release(gpio->cdev.pio_hdle, 1);
	return 0;
}

static int __devexit gpio_sw_remove(struct platform_device *dev)
{
	struct gpio_sw *gpio = pdev_to_gpio(dev);
		GPIO_SW_DEBUG("pio_hdle is %x \n",gpio->cdev.pio_hdle);
	gpio_sw_put_resource(gpio);
		GPIO_SW_DEBUG("gpio_sw_put_resource ok !\n");
	gpio_sw_classdev_unregister(&gpio->cdev);
		GPIO_SW_DEBUG("gpio_sw_classdev_unregister ok !\n");
		GPIO_SW_DEBUG("gpio addr is %x !\n" ,gpio);
	kfree(gpio);
		GPIO_SW_DEBUG("kfree ok !\n");
	return 0;
}

static int __devinit gpio_sw_probe(struct platform_device *dev)
{
	struct gpio_sw *gpio;
	struct gpio_sw_platdata *pdata = dev->dev.platform_data;
	unsigned int irq_ctl;
	int ret;
	char io_area[16];

	gpio = kzalloc(sizeof(struct gpio_sw), GFP_KERNEL);
		GPIO_SW_DEBUG("kzalloc ok !\n");

	if (gpio == NULL) {
		dev_err(&dev->dev, "No memory for device\n");
		return -ENOMEM;
	}

	platform_set_drvdata(dev, gpio);
		GPIO_SW_DEBUG("platform_set_drvdata ok !\n");
	gpio->pdata = pdata;
	gpio->cdev.pio_hdle = gpio_request_ex("gpio_para", pdata->name);

		GPIO_SW_DEBUG("pio_hdle is %x \n",gpio->cdev.pio_hdle);
		GPIO_SW_DEBUG("gpio_num = %s\n",pdata->name);
		GPIO_SW_DEBUG("pd->name = %s\n",gpio->pdata->name);

	gpio->cdev.port = get_gpio_member_value(gpio->cdev.pio_hdle,pdata->name,port_5 );
	gpio->cdev.port_num = get_gpio_member_value(gpio->cdev.pio_hdle,pdata->name,port_num_6 );
	gpio->cdev.mul_sel = get_gpio_member_value(gpio->cdev.pio_hdle,pdata->name,mul_sel_1 );
	gpio->cdev.pull = get_gpio_member_value(gpio->cdev.pio_hdle,pdata->name,pull_2 );
	gpio->cdev.drv_level = get_gpio_member_value(gpio->cdev.pio_hdle,pdata->name,drv_level_3 );
	gpio->cdev.data = get_gpio_member_value(gpio->cdev.pio_hdle,pdata->name,data_4 );
	gpio->cdev.irq_type = 0x0;

	sprintf(io_area,"P%c%d",gpio->cdev.port+'A'-1,gpio->cdev.port_num);
		GPIO_SW_DEBUG("io_area is %s \n",io_area);

	gpio->cdev.gpio_sw_cfg_set = gpio_sw_cfg_set;
	gpio->cdev.gpio_sw_pull_set = gpio_sw_pull_set;
	gpio->cdev.gpio_sw_data_set = gpio_sw_data_set;
	gpio->cdev.gpio_sw_drv_level_set = gpio_sw_drv_level_set;

	gpio->cdev.gpio_sw_cfg_get = gpio_sw_cfg_get;
	gpio->cdev.gpio_sw_pull_get = gpio_sw_pull_get;
	gpio->cdev.gpio_sw_data_get = gpio_sw_data_get;
	gpio->cdev.gpio_sw_drv_level_get = gpio_sw_drv_level_get;

	gpio->cdev.name = io_area;
	gpio->cdev.flags |= pdata->flags;

	if(gpio->cdev.mul_sel == 6){
		if(gpio->cdev.port== 'H' - 'A' + 1){
			if((gpio->cdev.port_num >= 0) && (gpio->cdev.port_num <= 21)){
			irq_ctl	=	REG_RD(GPIO_TEST_BASE + 0x210);
			__raw_writel((1 << gpio->cdev.port_num) | irq_ctl, GPIO_TEST_BASE + 0x210);
			gpio->cdev.irq_num = gpio->cdev.port_num + 1;
			all_irq_enable = 1;
			}else{
			printk("[gpio]: this pin don`t have EINT FUNCTION\n");
			kfree(gpio);
			return 1;
			}
		}else if(gpio->cdev.port== 'I' - 'A' + 1){
			if((gpio->cdev.port_num >= 10) && (gpio->cdev.port_num <= 19)){
			irq_ctl	=	REG_RD(GPIO_TEST_BASE + 0x210);
			__raw_writel((1 << (gpio->cdev.port_num + 12)) | irq_ctl, GPIO_TEST_BASE + 0x210);
			gpio->cdev.irq_num = gpio->cdev.port_num + 12 + 1;
			all_irq_enable = 1;
			}else{
			printk("[gpio]: this pin don`t have EINT FUNCTION\n");
			kfree(gpio);
			return 1;
			}
		}
		else{
		printk("[gpio]: this area don`t have EINT FUNCTION\n");
		kfree(gpio);
		return 1;
		}
	}
	gpio->cdev.irq=all_irq_enable;
	ret = gpio_sw_classdev_register(&dev->dev, &gpio->cdev);
		GPIO_SW_DEBUG("gpio_sw_classdev_register ok !\n");
	if (ret < 0) {
		dev_err(&dev->dev, "gpio_sw_classdev_register failed\n");
		kfree(gpio);
		return ret;
	}
		GPIO_SW_DEBUG("pio_hdle is %x \n",gpio->cdev.pio_hdle);
		GPIO_SW_DEBUG("gpio_sw_classdev_register good !\n");
	return 0;
}

static void gpio_sw_release (struct device *dev)
{
	GPIO_SW_DEBUG("gpio_sw_release good !\n");
}

static int gpio_sw_suspend(struct platform_device *dev, pm_message_t state)
{
	GPIO_SW_DEBUG("gpio driver gpio_sw_suspend \n");
	return 0;
}

static int gpio_sw_resume(struct platform_device *dev)
{
	GPIO_SW_DEBUG("gpio driver gpio_sw_resume \n");
	return 0;
}

static struct platform_driver gpio_sw_driver = {
	.probe		= gpio_sw_probe,
	.remove		= gpio_sw_remove,
	.suspend	= gpio_sw_suspend,
	.resume		= gpio_sw_resume,
	.driver		= {
		.name		= "gpio_sw",
		.owner		= THIS_MODULE,
	},
};

static int __init gpio_sw_init(void)
{
	int i, gpio_key_count,gpio_used,ret;
	ret = script_parser_fetch("gpio_para", "gpio_used", &gpio_used, sizeof(int));

	if (ret){
		GPIO_SW_DEBUG("failed to get gpio's used information\n");
		goto INIT_END;
	}
	if(!gpio_used){
		GPIO_SW_DEBUG("gpio module is used not  \n");
		goto INIT_END;
	}
	gpio_key_count = script_parser_mainkey_get_gpio_count("gpio_para");
	for(i=0;i<gpio_key_count;i++)
	{
		pdatesw[i].flags = 0;
		sprintf(pdatesw[i].name, "gpio_pin_%d", i+1);

		gpio_sw_dev[i].name = "gpio_sw";
		gpio_sw_dev[i].id   = i;
		gpio_sw_dev[i].dev.platform_data= &pdatesw[i];
		gpio_sw_dev[i].dev.release		= gpio_sw_release;

		GPIO_SW_DEBUG("pdatesw[%d].gpio_name = %s\n",i,pdatesw[i].name);
		GPIO_SW_DEBUG("pdatesw[%d] 1addr = %x \n",i,&pdatesw[i]);
		GPIO_SW_DEBUG("gpio_sw_dev[%d] addr = %x \n",i,&gpio_sw_dev[i]);
		platform_device_register(&gpio_sw_dev[i]);
	}

	platform_driver_register(&gpio_sw_driver);
	if(all_irq_enable)
	ret =  request_irq(GPIO_IRQ, sunxi_interrupt, IRQF_DISABLED, "gpio_sw", NULL);
	if (ret) {
        pr_info( "gpio: request irq failed\n");
        return ret;
    }

	GPIO_SW_DEBUG("gpio_sw_driver addr = %x \n",&gpio_sw_driver);

INIT_END:
	GPIO_SW_DEBUG("gpio_init finish ! \n");
	return 0;
}

static void __exit gpio_sw_exit(void)
{
	int i, gpio_key_count,gpio_used,ret;

	ret = script_parser_fetch("gpio_para", "gpio_used", &gpio_used, sizeof(int));

	if (ret){
		GPIO_SW_DEBUG("failed to get gpio's used information\n");
		goto EXIT_END;
	}
	if(!gpio_used){
		GPIO_SW_DEBUG("gpio module is used not  \n");
		goto EXIT_END;
	}

	gpio_key_count = script_parser_mainkey_get_gpio_count("gpio_para");

	if(all_irq_enable){
	__raw_writel(0x00, GPIO_TEST_BASE + 0x210);
	free_irq(GPIO_IRQ,NULL);
	}
	for(i=0;i<gpio_key_count;i++){
		GPIO_SW_DEBUG("gpio_sw_dev[%d] addr = %x \n",i,&gpio_sw_dev[i]);
		gpio_key_count = script_parser_mainkey_get_gpio_count("gpio_para");
		platform_device_unregister(&gpio_sw_dev[i]);
	}

		GPIO_SW_DEBUG("platform_device_unregister finish !  \n");
		GPIO_SW_DEBUG("gpio_sw_driver addr = %x \n",&gpio_sw_driver);
	platform_driver_unregister(&gpio_sw_driver);

EXIT_END:
	GPIO_SW_DEBUG("gpio_exit finish !  \n");
}

module_init(gpio_sw_init);
module_exit(gpio_sw_exit);

MODULE_AUTHOR("panlong <panlong@allwinnertech.com>");
MODULE_DESCRIPTION("SW GPIO driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:gpio_sw");
