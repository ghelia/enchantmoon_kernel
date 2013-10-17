/* Morgan  multi-touch device driver.
 *
 * Copyright(c) 2011 MorganTouch Inc.
 *
 * Original Author: Alexli <alexli05@hotmail.com>
 * Modefied By: Makoto Kondo <kondo@uei.co.jp>
 *
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include "mg-i2c-ts.h"

#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
//#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>

// for linux 2.6.36.3
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <linux/ioport.h>
#include <asm/irq.h>
#include <asm/io.h>

//#include <mach/gpio_v2.h>
#include <mach/irqs.h>
//#include <mach/script_v2.h>
#include <mach/sys_config.h>
#include "ctp_platform_ops.h"

#include <linux/version.h>

#define IOCTL_APP_MODE    		0xD0
#define IOCTL_RFLASH			0xD1
#define IOCTL_INQUIREPID	  	0xD2
#define IOCTL_READ_VERSION  		0xD3
#define IOCTL_WFLASH  			0xD4
#define IOCTL_IAP_MODE			0xD5
#define IOCTL_INQUIREBUSY		0xD6
#define IOCTL_CONFIRMUPDATE		0xD7
#define IOCTL_RECONFIRMUPDATE		0xD8

#define MG_DRIVER_NAME 	"mg-i2c-mtouch"
#define BUF_SIZE 			30
#define FINGER_NUM		5

#define X_MAX 1024
#define Y_MAX 768

#define X_MAX_DIG 14235

#define CAP_MAX_X		0x03ff
#define CAP_MAX_Y		0x02ff
#define CAP_MAX_NUM 5

#define DIG_MAX_X		0x37ff
#define DIG_MAX_Y		0x1fff
#define DIG_MAX_P       	0x03ff

#define STYLUS_KEY1 KEY_LEFT
#define STYLUS_KEY2 KEY_RIGHT


struct point_node_t{
	unsigned char 	active ;
	unsigned char	finger_id;
	unsigned char 	w ;
	unsigned int	posx;
	unsigned int	posy;
};

static struct point_node_t point_slot[FINGER_NUM*2];
static struct point_node_t Temp_point_slot[FINGER_NUM*2];

// #define _DEBUG
// #define	 PIXCIR_CAP_CALIBRAION


#define COORD_INTERPRET(MSB_BYTE, LSB_BYTE) \
		(MSB_BYTE << 8 | LSB_BYTE)

struct mg_i2c_platform_data {
	uint16_t version;
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int intr_gpio;
	int (*power)(int on);
};
static int gpio_int_hdle = 0;
static int gpio_wakeup_hdle = 0;
static int gpio_reset_hdle = 0;
static int gpio_enpwr_hdle = 0;

static void* __iomem gpio_addr = NULL;

static int screen_max_x = 0;
static int screen_max_y = 0;
static int revert_x_flag = 0;
static int revert_y_flag = 0;
static int exchange_x_y_flag = 0;
static __u32 twi_addr = 0;
static __u32 twi_id = 0;
static int	int_cfg_addr[]={PIO_INT_CFG0_OFFSET,PIO_INT_CFG1_OFFSET,
			PIO_INT_CFG2_OFFSET, PIO_INT_CFG3_OFFSET};
/* Addresses to scan */
union{
	unsigned short dirty_addr_buf[2];
	const unsigned short normal_i2c[2];
}u_i2c_addr = {{0x00},};

static int ctp_reset_enable = 0;
static int ctp_wakeup_enable = 0;
static int ctp_enpwr_enable = 0;

static struct i2c_client *this_client;

#define SCREEN_MAX_X    (screen_max_x)
#define SCREEN_MAX_Y    (screen_max_y)
#define PRESS_MAX       255

struct mg_ts_data{
	__u16 	x, y, w, p, id;
	struct i2c_client *client;
	/* capacivite device*/
	struct input_dev *cap_dev;
	/* digitizer */
	struct input_dev *dig_dev;
	/* eraser */
	struct input_dev *eraser_dev;
	
	struct mutex lock;
	int irq;
	
	struct workqueue_struct *mg_wq;
	struct work_struct work;
	int (*power)(int on);
	struct early_suspend early_suspend;
	int intr_gpio;

  struct miscdevice firmware;
	#define IAP_MODE_ENABLE		1	
	int iap_mode;	
};

int IAP_FLAG=0;

volatile int work_pending;
static struct mg_ts_data *private_ts;



/*
 * ctp_get_pendown_state  : get the int_line data state, 
 * 
 * return value:
 *             return PRESS_DOWN: if down
 *             return FREE_UP: if up,
 *             return 0: do not need process, equal free up.
 */
static int ctp_get_pendown_state(void)
{
	unsigned int reg_val;
	static int state = FREE_UP;

	//get the input port state
	reg_val = readl(gpio_addr + PIOH_DATA);
	//printk("reg_val = %x\n",reg_val);
	if(!(reg_val & (1<<CTP_IRQ_NO))){
		state = PRESS_DOWN;
		//printk("pen down. \n");
	}else{ //touch panel is free up
		state = FREE_UP;
		//printk("free up. \n");
	}
	return state;
}

/**
 * ctp_clear_penirq - clear int pending
 *
 */
static void ctp_clear_penirq(void)
{
	int reg_val;
	//clear the IRQ_EINT29 interrupt pending
	//printk("clear pend irq pending\n");
	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);

	if((reg_val = (reg_val&(1<<(CTP_IRQ_NO))))){
		//printk("==CTP_IRQ_NO=\n");              
		writel(reg_val,gpio_addr + PIO_INT_STAT_OFFSET);
	}
	return;
}

/**
 * ctp_set_irq_mode - according sysconfig's subkey "ctp_int_port" to config int port.
 * 
 * return value: 
 *              0:      success;
 *              others: fail; 
 */
static int ctp_set_irq_mode(char *major_key , char *subkey, int ext_int_num, ext_int_mode int_mode)
{
	int ret = 0;
	__u32 reg_num = 0;
	__u32 reg_addr = 0;
	__u32 reg_val = 0;
	//config gpio to int mode
	printk("%s: config gpio to int mode. \n", __func__);
#ifndef SYSCONFIG_GPIO_ENABLE
#else
	if(gpio_int_hdle){
		gpio_release(gpio_int_hdle, 2);
	}
	gpio_int_hdle = gpio_request_ex(major_key, subkey);
	if(!gpio_int_hdle){
		printk("request tp_int_port failed. \n");
		ret = -1;
		goto request_tp_int_port_failed;
	}
#endif

#ifdef AW_GPIO_INT_API_ENABLE
#else
	printk(" INTERRUPT CONFIG\n");
	reg_num = ext_int_num%8;
	reg_addr = ext_int_num/8;
	reg_val = readl(gpio_addr + int_cfg_addr[reg_addr]);
	reg_val &= (~(7 << (reg_num * 4)));
	reg_val |= (int_mode << (reg_num * 4));
	writel(reg_val,gpio_addr+int_cfg_addr[reg_addr]);
                                                               
	ctp_clear_penirq();
                                                               
	reg_val = readl(gpio_addr+PIO_INT_CTRL_OFFSET); 
	reg_val |= (1 << ext_int_num);
	writel(reg_val,gpio_addr+PIO_INT_CTRL_OFFSET);

	udelay(1);
#endif

request_tp_int_port_failed:
	return ret;  
}

/**
 * ctp_set_gpio_mode - according sysconfig's subkey "ctp_io_port" to config io port.
 *
 * return value: 
 *              0:      success;
 *              others: fail; 
 */
static int ctp_set_gpio_mode(void)
{
	//int reg_val;
	int ret = 0;
	//config gpio to io mode
	printk("%s: config gpio to io mode. \n", __func__);
#ifndef SYSCONFIG_GPIO_ENABLE
#else
	if(gpio_int_hdle){
		gpio_release(gpio_int_hdle, 2);
	}
	gpio_int_hdle = gpio_request_ex("ctp_para", "ctp_io_port");
	if(!gpio_int_hdle){
		printk("request ctp_io_port failed. \n");
		ret = -1;
		goto request_tp_io_port_failed;
	}
#endif
	return ret;

request_tp_io_port_failed:
	return ret;
}

/**
 * ctp_judge_int_occur - whether interrupt occur.
 *
 * return value: 
 *              0:      int occur;
 *              others: no int occur; 
 */
static int ctp_judge_int_occur(void)
{
	//int reg_val[3];
	int reg_val;
	int ret = -1;

	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
	if(reg_val&(1<<(CTP_IRQ_NO))){
		ret = 0;
	}
	return ret; 	
}

/**
 * ctp_free_platform_resource - corresponding with ctp_init_platform_resource
 *
 */
static void ctp_free_platform_resource(void)
{
	printk("=======%s=========.\n", __func__);
	if(gpio_addr){
		iounmap(gpio_addr);
	}
	
	if(gpio_int_hdle){
		gpio_release(gpio_int_hdle, 2);
	}
	
	if(gpio_wakeup_hdle){
		gpio_release(gpio_wakeup_hdle, 2);
	}
	
	if(gpio_reset_hdle){
		gpio_release(gpio_reset_hdle, 2);
	}

	if(gpio_enpwr_hdle){
		gpio_release(gpio_enpwr_hdle, 2);
	}

	return;
}


/**
 * ctp_init_platform_resource - initialize platform related resource
 * return value: 0 : success
 *               -EIO :  i/o err.
 *
 */
static int ctp_init_platform_resource(void)
{
	int ret = 0;

	gpio_addr = ioremap(PIO_BASE_ADDRESS, PIO_RANGE_SIZE);
	//printk("%s, gpio_addr = 0x%x. \n", __func__, gpio_addr);
	if(!gpio_addr) {
		ret = -EIO;
		goto exit_ioremap_failed;	
	}
	//ctp_wakeup_enable = 1;
	gpio_wakeup_hdle = gpio_request_ex("ctp_para", "ctp_wakeup");
	if(!gpio_wakeup_hdle) {
		pr_warning("%s: tp_wakeup request gpio fail!\n", __func__);
		ctp_wakeup_enable = 0;
	}

	ctp_reset_enable = 1;
	gpio_reset_hdle = gpio_request_ex("ctp_para", "ctp_reset");
	if(!gpio_reset_hdle) {
		pr_warning("%s: tp_reset request gpio fail!\n", __func__);
		ctp_reset_enable = 0;
	}
	ctp_enpwr_enable = 1;
	gpio_enpwr_hdle = gpio_request_ex("ctp_para", "ctp_power_en");
	if(!gpio_enpwr_hdle) {
		printk("[MG]warning @%s: tp  power enable request gpio fail!\n", __func__);
		ctp_enpwr_enable = 0;
	}

	return ret;

exit_ioremap_failed:
	ctp_free_platform_resource();
	return ret;
}


/**
 * ctp_fetch_sysconfig_para - get config info from sysconfig.fex file.
 * return value:  
 *                    = 0; success;
 *                    < 0; err
 */
static int ctp_fetch_sysconfig_para(void)
{
	int ret = -1;
	int ctp_used = -1;
	char name[I2C_NAME_SIZE];
	script_parser_value_type_t type = SCIRPT_PARSER_VALUE_TYPE_STRING;

	printk("%s. \n", __func__);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_used", &ctp_used, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		//goto script_parser_fetch_err;
	}
	if(1 != ctp_used){
		pr_err("%s: ctp_unused. \n",  __func__);
		//ret = 1;
		return ret;
	}

	if(SCRIPT_PARSER_OK != script_parser_fetch_ex("ctp_para", "ctp_name", (int *)(&name), &type, sizeof(name)/sizeof(int))){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		//goto script_parser_fetch_err;
	}
	if(strcmp(CTP_NAME, name)){
		pr_err("%s: name %s does not match CTP_NAME. \n", __func__, name);
		pr_err(CTP_NAME);
		//ret = 1;
		//return ret;
	}

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_twi_addr", &twi_addr, sizeof(twi_addr)/sizeof(__u32))){
		pr_err("%s: script_parser_fetch err. \n", name);
		//goto script_parser_fetch_err;
	}
	//big-endian or small-endian?
	//printk("%s: before: ctp_twi_addr is 0x%x, dirty_addr_buf: 0x%hx. dirty_addr_buf[1]: 0x%hx \n", __func__, twi_addr, u_i2c_addr.dirty_addr_buf[0], u_i2c_addr.dirty_addr_buf[1]);
	u_i2c_addr.dirty_addr_buf[0] = twi_addr;
	u_i2c_addr.dirty_addr_buf[1] = I2C_CLIENT_END;
	printk("%s: after: ctp_twi_addr is 0x%x, dirty_addr_buf: 0x%hx. dirty_addr_buf[1]: 0x%hx \n", __func__, twi_addr, u_i2c_addr.dirty_addr_buf[0], u_i2c_addr.dirty_addr_buf[1]);
	//printk("%s: after: ctp_twi_addr is 0x%x, u32_dirty_addr_buf: 0x%hx. u32_dirty_addr_buf[1]: 0x%hx \n", __func__, twi_addr, u32_dirty_addr_buf[0],u32_dirty_addr_buf[1]);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_twi_id", &twi_id, sizeof(twi_id)/sizeof(__u32))){
		pr_err("%s: script_parser_fetch err. \n", name);
		//goto script_parser_fetch_err;
	}
	printk("%s: ctp_twi_id is %d. \n", __func__, twi_id);
	
	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_screen_max_x", &screen_max_x, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		//goto script_parser_fetch_err;
	}
	pr_info("%s: screen_max_x = %d. \n", __func__, screen_max_x);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_screen_max_y", &screen_max_y, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		//goto script_parser_fetch_err;
	}
	pr_info("%s: screen_max_y = %d. \n", __func__, screen_max_y);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_revert_x_flag", &revert_x_flag, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		//goto script_parser_fetch_err;
		//revert_x_flag = 0;
	}
	pr_info("%s: revert_x_flag = %d. \n", __func__, revert_x_flag);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_revert_y_flag", &revert_y_flag, 1)){
		pr_err("%s: script_parser_fetch err. \n", __func__);
		//goto script_parser_fetch_err;
		//revert_y_flag = 0;
	}
	pr_info("%s: revert_y_flag = %d. \n", __func__, revert_y_flag);

	if(SCRIPT_PARSER_OK != script_parser_fetch("ctp_para", "ctp_exchange_x_y_flag", &exchange_x_y_flag, 1)){
		pr_err("ft5x_ts: script_parser_fetch err. \n");
		//goto script_parser_fetch_err;
		//exchange_x_y_flag = 0;
	}
	pr_info("%s: exchange_x_y_flag = %d. \n", __func__, exchange_x_y_flag);

	return 0;
}

/**
 * ctp_reset - function
 *
 */
static void ctp_reset(void)
{
	//printk("%s. \n", __func__);
	if(ctp_enpwr_enable) {
		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_enpwr_hdle, 0, "ctp_power_en")){
			printk("[MG]Err @%s: power on tp failure ! \n", __func__);
		} else mdelay(TS_RESET_LOW_PERIOD);
	}
	if(ctp_reset_enable){
	
		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_reset_hdle, 0, "ctp_reset")){
			printk("%s: err when operate gpio. \n", __func__);
		}
		mdelay(TS_RESET_LOW_PERIOD);
		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_reset_hdle, 1, "ctp_reset")){
			printk("%s: err when operate gpio. \n", __func__);
		}
		mdelay(TS_INITIAL_HIGH_PERIOD);
	}
}

/**
 * ctp_wakeup - function
 *
 */
static void ctp_wakeup(void)
{
	//printk("%s. \n", __func__);
	if(1 == ctp_wakeup_enable){  
		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_wakeup_hdle, 1, "ctp_wakeup")){
			printk("%s: err when operate gpio. \n", __func__);
		}
		mdelay(TS_WAKEUP_LOW_PERIOD);
		if(EGPIO_SUCCESS != gpio_write_one_pin_value(gpio_wakeup_hdle, 0, "ctp_wakeup")){
			printk("%s: err when operate gpio. \n", __func__);
		}
		mdelay(TS_WAKEUP_HIGH_PERIOD);

	}
	return;
}
/**
 * ctp_detect - Device detection callback for automatic device creation
 * return value:  
 *                    = 0; success;
 *                    < 0; err
 */
static int ctp_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;

	if(twi_id == adapter->nr)
	{
		pr_info("%s: Detected chip %s at adapter %d, address 0x%02x\n",
			 __func__, CTP_NAME, i2c_adapter_id(adapter), client->addr);

		strlcpy(info->type, CTP_NAME, I2C_NAME_SIZE);
		return 0;
	}else{
		return -ENODEV;
	}
}
////////////////////////////////////////////////////////////////

static struct ctp_platform_ops ctp_ops = {
	.get_pendown_state = ctp_get_pendown_state,
	.clear_penirq	   = ctp_clear_penirq,
	.set_irq_mode      = ctp_set_irq_mode,
	.set_gpio_mode     = ctp_set_gpio_mode,	
	.judge_int_occur   = ctp_judge_int_occur,
	.init_platform_resource = ctp_init_platform_resource,
	.free_platform_resource = ctp_free_platform_resource,
	.fetch_sysconfig_para = ctp_fetch_sysconfig_para,
	.ts_reset =          ctp_reset,
	.ts_wakeup =         ctp_wakeup,
	.ts_detect = ctp_detect,
};

int mg_iap_open(struct inode *inode, struct file *filp)
{ 

	printk("[MG] into  %s.\n", __func__);
	
	IAP_FLAG = 1;
	
	struct mg_ts_data *dev;

	dev = kmalloc(sizeof(struct mg_ts_data), GFP_KERNEL);
	if (dev == NULL) {
		return -ENOMEM;
	}

	filp->private_data = dev;
	
	if (private_ts == NULL)  printk("private_ts is NULL~~~\n");	
		
	return 0;
}

int mg_iap_release(struct inode *inode, struct file *filp)
{    
	printk("[MG]into mg_iap_release\n");
	
	struct mg_ts_data *dev = filp->private_data;
	if (dev)
	{
		kfree(dev);
	}
	filp->private_data=NULL;
	IAP_FLAG = 0;

	return 0;
}
ssize_t mg_iap_write(struct file *filp, const char *buff,    size_t count, loff_t *offp)
{  
#ifdef	_DEBUG	
	printk("[MG]into mg_iap_write\n");
#endif
	int ret;
	char *tmp;
	
 	if (count > 32)
       	count = 32;
	
	tmp = kmalloc(count, GFP_KERNEL);
	
	if (tmp == NULL)
	    return -ENOMEM;
	
	if (copy_from_user(tmp, buff, count)) {
	    return -EFAULT;
	}
#ifdef	_DEBUG	
	int i = 0;
	printk(KERN_INFO "[MG] %s:", __func__);
	for(i = 0; i < count; i++)
	{
		printk(" %02x ", tmp[i]);
	}
	printk("\n");
#endif	
	ret = i2c_smbus_write_i2c_block_data(private_ts->client, 0, count, tmp);
	
	if (ret < 0) printk("[MG] %s fail, ret=%d \n", __func__, ret);
	kfree(tmp);

	return ret;

}

ssize_t mg_iap_read(struct file *filp, const char *buff,    size_t count, loff_t *offp){    
#ifdef	_DEBUG	    
	printk("[MG]into mg_iap_read\n");
#endif	
	char *tmp;
	int ret;  
	
    	if (count > 32)
        	count = 32;
	
	tmp = kmalloc(count, GFP_KERNEL);
	
	if (tmp == NULL)
	    return -ENOMEM;
	
	ret = i2c_smbus_read_i2c_block_data(private_ts->client, 0, count, tmp);
	if (ret >= 0)
    	copy_to_user(buff, tmp, count);
	else
	{
			if (ret != count) printk("[MG] %s fail, ret=%d \n", __func__, ret);
	}
#ifdef	_DEBUG	
	int i = 0;
	printk(KERN_INFO "[MG] %s:", __func__);
	for(i = 0; i < count; i++)
	{
		printk(" %02x ", tmp[i]);
	}
	printk("\n");
#endif
	
	kfree(tmp);

	return ret;
}

int mg_iap_ioctl(struct inode *inode, struct file *filp,    unsigned int cmd, unsigned long arg)
{
	
	int __user *argp = (int __user *) arg;
	
	u_int8_t ret = 0;

// #ifdef	_DEBUG	
	printk("[MG]into mg_iap_ioctl CMD :%d argp=%d\n",cmd,(__user *argp));
// #endif
	IAP_FLAG =1;
	switch (cmd) 
	{        
	case IOCTL_WFLASH: 
#ifdef	_DEBUG	
		printk("IOCTL_WFLASH\n");
#endif
		break;   
		
	case IOCTL_RFLASH:
#ifdef	_DEBUG	
		printk("IOCTL_RFLASH\n");
#endif
		break;
	case IOCTL_INQUIREPID:      
#ifdef	_DEBUG	  
		printk("IOCTL_INQUIREPID\n");
#endif
		ret = i2c_smbus_write_i2c_block_data(private_ts->client,
								0, 5, command_list[4]);
		if(ret < 0)
		{
			printk("[MG]IOCTL_INQUIREPID error!!!!!\n");
			return -1;
		}

		break;    
	case IOCTL_READ_VERSION:      

		printk("IOCTL_READ_VERSION\n");



		ret = i2c_smbus_write_i2c_block_data(private_ts->client,
								0, 5, command_list[3]);

		if(ret < 0)
		{
			printk("[MG]IOCTL_READ_VERSION error!!!!!\n");
			return -1;
		}

		break;    
	case IOCTL_INQUIREBUSY:
#ifdef	_DEBUG	
		printk("IOCTL_INQUIREBUSY\n");
#endif
		ret = i2c_smbus_write_i2c_block_data(private_ts->client,
								0, 5, command_list[5]);

		if(ret < 0)
		{
			printk("[MG]IOCTL_INQUIREBUSY error!!!!!\n");
			return -1;
		}
		break;	
	case IOCTL_CONFIRMUPDATE:
#ifdef	_DEBUG	
		printk("IOCTL_CONFIRMUPDATE\n");
#endif
		ret = i2c_smbus_write_i2c_block_data(private_ts->client,
								0, 5, command_list[2]);

		if(ret < 0)
		{
			printk("[MG]IOCTL_CONFIRMUPDATE error!!!!!\n");
			return -1;
		}
		break;
	case IOCTL_RECONFIRMUPDATE:
#ifdef	_DEBUG			
		printk("IOCTL_RECONFIRMUPDATE\n");
#endif		
		ret = i2c_smbus_write_i2c_block_data(private_ts->client,
								0, 5, command_list[10]);

		if(ret < 0)
		{
			printk("[MG]IOCTL_RECONFIRMUPDATE error!!!!!\n");
			return -1;
		}
		break;
	case IOCTL_APP_MODE:     
#ifdef	_DEBUG	   
		printk("IOCTL_APP_MODE\n");
#endif		
		ret = i2c_smbus_write_i2c_block_data(private_ts->client,
								0, 5, command_list[1]);
		if(ret < 0)
		{
			printk("[MG]IOCTL_APP_MODE error!!!!!\n");
			return -1;
		}
		break;        
			
	case IOCTL_IAP_MODE:
#ifdef	_DEBUG	
		printk("IOCTL_IAP_MODE\n");
#endif		
		ret = i2c_smbus_write_i2c_block_data(private_ts->client,
								0, 5, command_list[0]);
		if(ret < 0)
		{
			printk("[MG]IOCTL_IAP_MODE error!!!!!\n");
			return -1;
		}
		
		break;
	
	default:            
		break;   
	}     

	return 0;
}


struct file_operations mg_touch_fops = {    
        open:       mg_iap_open,    
        write:      mg_iap_write,    
        read:				mg_iap_read,    
        release:    mg_iap_release,   
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
	.unlocked_ioctl = mg_iap_ioctl,
#else
	.ioctl = mg_iap_ioctl,
#endif 

 };

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mg_ts_early_suspend(struct early_suspend *h);
static void mg_ts_late_resume(struct early_suspend *h);
#endif
static struct i2c_driver mg_ts_driver;

static irqreturn_t mg_irq(int irq, void *_mg)
{
	struct mg_ts_data *ts = _mg;
	int reg_val;

	//clear the IRQ_EINT21 interrupt pending
	reg_val = readl(gpio_addr + PIO_INT_STAT_OFFSET);
	 
	if(reg_val&(1<<(CTP_IRQ_NO)))

	{
		//if (!work_pending) 
		{
#ifdef _DEBUG
			printk(KERN_INFO "[MG] %s: enter\n", __func__);
#endif
			
			work_pending = 1;
			writel(reg_val&(1<<(CTP_IRQ_NO)),gpio_addr + PIO_INT_STAT_OFFSET);
			//if (!work_pending(&ts->work)) 
			{
				//disable_irq_nosync(ts->irq);
				//schedule_work(&ts->work);
				queue_work(ts->mg_wq, &ts->work);
			}
		}
	}
	else
	{
#ifdef _DEBUG
		printk("Other Interrupt\n");
#endif
	}
	return IRQ_HANDLED;
}
static inline void mg_report(struct mg_ts_data *ts)
{

    	struct input_dev *input;
	static struct input_dev *lastInput = NULL;
	int state;

	state =  ts->w;

	if(state == 0x10 && ts->p > 0){
            input = ts->eraser_dev;
        }else{
            input = ts->dig_dev;
            input_report_key(input, STYLUS_KEY1, state == 18);
            input_report_key(input, STYLUS_KEY2, state == 19);
        }
        if(lastInput && lastInput != input){
            input_report_key(lastInput, BTN_TOUCH, 0);
            input_sync(lastInput);
        }
        input_report_key(input, BTN_TOUCH, state);
        if(state){
            input_report_abs(input, ABS_X, ts->x);
            input_report_abs(input, ABS_Y, ts->y);
            input_report_abs(input, ABS_PRESSURE, ts->p);
        }
        lastInput = input;

	input_sync(input);
	return;
}
static inline void mg_multi_report(struct mg_ts_data *ts)
{
#ifdef	_DEBUG
	printk("FINGER ID=%d,W=%d,X=%d Y=%d \n",ts->id, ts->w, ts->x,ts->y);
#endif

	input_report_abs(ts->cap_dev, ABS_MT_TRACKING_ID, ts->id);	
	//input_report_abs(ts->cap_dev, ABS_MT_TOUCH_MAJOR, ts->w); org, zhangdp
	if(ts->w > 0)//zhangdp
	{
		input_report_abs(ts->cap_dev, ABS_MT_TOUCH_MAJOR, 100);
	} else {
		input_report_abs(ts->cap_dev, ABS_MT_TOUCH_MAJOR, 0);
	}
	//input_report_abs(ts->cap_dev, ABS_MT_WIDTH_MAJOR, 1);
	input_report_abs(ts->cap_dev, ABS_MT_POSITION_X, ts->x);
	input_report_abs(ts->cap_dev, ABS_MT_POSITION_Y, ts->y);
	input_mt_sync(ts->cap_dev);
}
static void mg_i2c_work(struct work_struct *work)
{
	int i = 0;
	struct mg_ts_data *ts =	container_of(work, struct mg_ts_data, work);
	u_int8_t ret = 0;
	
	unsigned char touch, button;
	unsigned char read_buf[BUF_SIZE] = { 0 };

#ifdef _DEBUG
	//printk(KERN_INFO "[MG] %s: enter\n", __func__);
#endif

	for(i=0;i<BUF_SIZE;i++)
	{
		read_buf[i]=0;
	}
	if(IAP_FLAG == 1)
	{
		
		msleep(10);
		ret = i2c_smbus_read_i2c_block_data(ts->client, 0x0, 8, read_buf);
		if(ret < 0)
		{
			for (i=0;i<10;i++)
			{
				if(i>9)
				{
					printk("Recieve Command ACK Failed!\n");
					return;
				}
				msleep(5);
				printk("Recieve Command ACK Failed %d times!\n",i+1);
				ret = i2c_smbus_read_i2c_block_data(ts->client, 0x0, 8, read_buf);
				if(ret>=0)
				{
					if(read_buf[0] == 0x01 && read_buf[1] == 0x02 && read_buf[2] == 0x04 && read_buf[3] == 0x03 && (read_buf[4] == 0xAA ||read_buf[4] == 0xBB)) 
					{
						msleep(5000);
						printk("APP or Boot return Delay!\n");
					}
					printk("Recieve Command ACK success!\n");
					break;
				}
			}

			
		}
		else 
		{
			if(read_buf[0] == 0x01 && read_buf[1] == 0x02 && read_buf[2] == 0x04 && read_buf[3] == 0x03 && (read_buf[4] == 0xAA ||read_buf[4] == 0xBB)) 
			{
				
				msleep(5000);
				printk("APP or Boot return Delay!\n");
			}
		}
		msleep(10);
		printk("\nACK : "); 
		for(i = 0; i < 8; i ++) 	
		{
			printk("%02x", read_buf[i]);
		}
		printk("\n");


		
		//if(ts->irq)
		//	enable_irq(ts->irq);
		work_pending = 0;
		return;
	}
	
	ret = i2c_smbus_read_i2c_block_data(ts->client,
							0x0, BUF_SIZE, read_buf);
	if(ret < 0)	
	{
		printk("%s: i2c_smbus_read_i2c_block_data failed, error=%d\n",__func__,ret);		
		return;	
	}
#ifdef _DEBUG
	printk("Data:");
	for(i=0;i<BUF_SIZE;i++)
	{
		printk(" %02x ",read_buf[i]);
	}
	printk("\n");
	
#endif


	
	if (read_buf[0] ==0x02) 
	{

		/*		
		input_report_key(ts->cap_dev, BTN_TOUCH, 0);
		input_report_abs(ts->cap_dev, ABS_MT_TOUCH_MAJOR, 0);
		//input_report_abs(ts->cap_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_mt_sync(ts->cap_dev);
		input_sync(ts->cap_dev);
		*/
		ts->x = COORD_INTERPRET(read_buf[3], read_buf[2]);
// 		ts->x = X_MAX_DIG-ts->x;
		ts->y = COORD_INTERPRET(read_buf[5], read_buf[4]);
//                 ts->x = X_MAX_DIG-ts->y;
		ts->w = read_buf[1];
		ts->p = (COORD_INTERPRET(read_buf[7], read_buf[6]));


		mg_report(ts);
		input_sync(ts->dig_dev);
	}

	else if (read_buf[0] ==0x01)
	{
		touch = read_buf[2];
		button = read_buf[1];
#ifdef _DEBUG
		printk("touch=%d, button=%d \n",touch,button);
#endif
#ifdef BUTTON
		if(button) 
		{
			switch(button) 
			{
			case 1:
				input_report_key(ts->cap_dev, KEY_HOME, 1);
			case 2:
				//add other key down report
			case 4:

			case 8:

			case 16:
			case 32:
			case 64:
			case 128:
			default:
				break;
			}
		} 
		else 
		{
			input_report_key(ts->cap_dev, KEY_HOME, 0);
			//add other key up report
		}
#endif


		for (i=0; i<FINGER_NUM*2; i++) 
		{
			point_slot[i].active = 0;
			point_slot[i].finger_id =0;
			point_slot[i].w = 0;
			point_slot[i].posx = 0;
			point_slot[i].posy = 0;
		}
		for (i=0; i<FINGER_NUM*2; i++) 
		{
			Temp_point_slot[i].active = 0;
			Temp_point_slot[i].finger_id =0;
			Temp_point_slot[i].w = 0;
			Temp_point_slot[i].posx = 0;
			Temp_point_slot[i].posy = 0;
		}
#if 0
		if(touch>=1)
		{


			ts->x = (read_buf[6+6*0]<<8)|read_buf[5+6*0];
			ts->y = Y_MAX-((read_buf[8+6*0]<<8)|read_buf[7+6*0]);

			ts->w = read_buf[4+6*0];

			ts->id= read_buf[3+6*0];

	    
			mg_multi_report(ts);

				
		}
		if(touch>=2)
		{

			ts->x = (read_buf[6+6*1]<<8)|read_buf[5+6*1];
			ts->y = Y_MAX-((read_buf[8+6*1]<<8)|read_buf[7+6*1]);

			ts->w = read_buf[4+6*1];

			ts->id= read_buf[3+6*1];

			mg_multi_report(ts);

		}
		if(touch>=3)
		{
			ts->x = (read_buf[17+5*(2-2)]<<8)|read_buf[16+5*(2-2)];
			ts->y = Y_MAX-((read_buf[19+5*(2-2)]<<8)|read_buf[18+5*(2-2)]);

			ts->w = read_buf[15+5*(2-2)]&0x0f;

			ts->id= read_buf[15+5*(2-2)]>>4;

			mg_multi_report(ts);

		}
		if(touch>=4)
		{
			ts->x = (read_buf[17+5*(3-2)]<<8)|read_buf[16+5*(3-2)];
			ts->y = Y_MAX-((read_buf[19+5*(3-2)]<<8)|read_buf[18+5*(3-2)]);

			ts->w = read_buf[15+5*(3-2)]&0x0f;

			ts->id= read_buf[15+5*(3-2)]>>4;

			mg_multi_report(ts);

		}
		if(touch>=5)
		{
			ts->x = (read_buf[17+5*(4-2)]<<8)|read_buf[16+5*(4-2)];
			ts->y = Y_MAX-((read_buf[19+5*(4-2)]<<8)|read_buf[18+5*(4-2)]);

			ts->w = read_buf[15+5*(4-2)]&0x0f;

			ts->id= read_buf[15+5*(4-2)]>>4;

			mg_multi_report(ts);
		}
#else
		for (i=0; i<touch; i++)	
		{
			if(i<2)
			{
				Temp_point_slot[i].finger_id = read_buf[3+6*i];
				Temp_point_slot[i].w=read_buf[4+6*i];
				Temp_point_slot[i].posx = (read_buf[6+6*i]<<8)|read_buf[5+6*i];
				Temp_point_slot[i].posy = (read_buf[8+6*i]<<8)|read_buf[7+6*i];
			}
			else
			{
				Temp_point_slot[i].finger_id=read_buf[15+5*(i-2)]>>4;
				Temp_point_slot[i].w=read_buf[15+5*(i-2)]&0x0f;
				Temp_point_slot[i].posx=(read_buf[17+5*(i-2)]<<8)|read_buf[16+5*(i-2)];
				Temp_point_slot[i].posy=(read_buf[19+5*(i-2)]<<8)|read_buf[18+5*(i-2)];
			}
		}
		int j=0;	
		for (i=0; i<FINGER_NUM; i++) 
		{
			for(j=0;j<FINGER_NUM;j++)
			{	
				if(Temp_point_slot[j].finger_id == i)
				{
					point_slot[i].finger_id = Temp_point_slot[j].finger_id;
					point_slot[i].w=Temp_point_slot[j].w;
					point_slot[i].posx = Temp_point_slot[j].posx;
					point_slot[i].posy = Temp_point_slot[j].posy;
					point_slot[i].active = 1;
					break;
				}
			}
		}

		for (i=0; i<FINGER_NUM; i++) 
		{
			if (point_slot[i].active == 1) 
			{
				if( point_slot[i].w > 0 )
				{
					input_report_key(ts->cap_dev, ABS_MT_TRACKING_ID, i);
					input_report_abs(ts->cap_dev, ABS_MT_TOUCH_MAJOR, point_slot[i].w>0 ? 100 : 0);
					input_report_abs(ts->cap_dev, ABS_MT_POSITION_X,  point_slot[i].posx);
					input_report_abs(ts->cap_dev, ABS_MT_POSITION_Y,  Y_MAX-point_slot[i].posy);
					input_report_abs(ts->cap_dev, ABS_MT_WIDTH_MAJOR, 150);
				} else if ( point_slot[i].w == 0 )	{
					input_report_abs(ts->cap_dev, ABS_MT_TOUCH_MAJOR, 0);
					input_report_abs(ts->cap_dev, ABS_MT_WIDTH_MAJOR, 0);
				}
				input_mt_sync(ts->cap_dev);
#ifdef	_DEBUG
				printk("slot=%d,x%d=%d,y%d=%d  \n",i, i/2,point_slot[i].posx, i/2,abs(Y_MAX-point_slot[i].posy));
#endif
			}
		}	
#endif
		
		input_sync(ts->cap_dev);

		for (i=0; i<FINGER_NUM*2; i++) 
		{
			point_slot[i].active = 0;
			point_slot[i].finger_id =0;
			point_slot[i].w = 0;
			point_slot[i].posx = 0;
			point_slot[i].posy = 0;
		}
		for (i=0; i<FINGER_NUM*2; i++) 
		{
			Temp_point_slot[i].active = 0;
			Temp_point_slot[i].finger_id =0;
			Temp_point_slot[i].w = 0;
			Temp_point_slot[i].posx = 0;
			Temp_point_slot[i].posy = 0;
		}
	}
	//if(ts->irq)
	//	enable_irq(ts->irq);
	work_pending = 0;
}

static int pixcir_cap_calibration()
{
	
	int rc = 0, retry = 5;
	struct mg_ts_data *ts = i2c_get_clientdata(this_client);
	printk(KERN_INFO "[MG] %s: enter\n", __func__);
	msleep(5000);	
	do {
		rc = i2c_smbus_write_i2c_block_data(ts->client, 0, 5, command_list[11]);
		if (rc < 0)
		{
			printk(KERN_ERR "[MG] %s: cap calibrate failed! err = %d\n",
				__func__, rc);
		}
		else
		{
			printk(KERN_INFO "[MG] %s: cap calibrate  success!\n",__func__);
			break;
		}
		msleep(2000);
	} while (--retry);
	return rc;
}


static int mg_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct mg_i2c_platform_data *pdata;
	struct mg_ts_data *ts;
	int i = 0;
	int reg_val;

	//reset
	ctp_ops.ts_reset();

	for(i=0; i<FINGER_NUM*2; i++) 
	{
		point_slot[i].active = 0;
	}
	work_pending = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "%s: i2c check functionality error\n", __func__);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	/* allocate ts data */
	ts = kzalloc(sizeof(struct mg_ts_data), GFP_KERNEL);
	if (ts == NULL) 
	{
		printk(KERN_ERR "%s: allocate mg_ts_data failed\n", __func__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}
	

	ts->mg_wq = create_singlethread_workqueue("mg_wq");
	if (!ts->mg_wq) {
		printk(KERN_ERR "%s: create workqueue failed\n", __func__);
		err = -ENOMEM;
		goto err_create_wq_failed;
	}
	this_client = client;

	
	ts->client = client;
	dev_info(&ts->client->dev, "device probing\n");
	i2c_set_clientdata(client, ts);
	pdata = client->dev.platform_data;
	
	if (likely(pdata != NULL)) 
	{
		ts->power = pdata->power;
		ts->intr_gpio = pdata->intr_gpio;
	}
	
	ts->firmware.minor = MISC_DYNAMIC_MINOR;
	ts->firmware.name = "FWUpdate";
	ts->firmware.fops = &mg_touch_fops;  //keven 2011-12-12
	ts->firmware.mode = S_IRWXUGO; 
		
	if (misc_register(&ts->firmware) < 0)
		printk("[MG]misc_register failed\n");
	else
		printk("[MG]misc_register finished\n"); 
		
	if (ts->power)
  	ts->power(1);		
  	
	mutex_init(&ts->lock);

	err = ctp_ops.set_irq_mode("ctp_para", "ctp_int_port", CTP_IRQ_NO, CTP_IRQ_MODE);
	if(0 != err){
		printk("%s:ctp_ops.set_irq_mode err. \n", __func__);
		goto exit_gpio_int_request_failed;
	}



	/* allocate input device for capacitance */
	ts->cap_dev = input_allocate_device();
	if (ts->cap_dev == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev, "Failed to allocate cap device\n");
		goto err_cap_dev_alloc_failed;
	}


	__set_bit(EV_ABS, ts->cap_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT, ts->cap_dev->propbit);
	__set_bit(ABS_MT_POSITION_X, ts->cap_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, ts->cap_dev->absbit);

	__set_bit(ABS_MT_WIDTH_MAJOR, ts->cap_dev->absbit);
	__set_bit(ABS_MT_TRACKING_ID, ts->cap_dev->absbit);
	input_set_abs_params(ts->cap_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->cap_dev, ABS_MT_TRACKING_ID, 0, FINGER_NUM, 0, 0);
	input_set_abs_params(ts->cap_dev, ABS_MT_POSITION_X, 0, X_MAX, 0, 0);
	input_set_abs_params(ts->cap_dev, ABS_MT_POSITION_Y, 0, Y_MAX, 0, 0);

	ts->cap_dev->name = "garuda-capacitance";
	ts->cap_dev->id.bustype = BUS_I2C;


	err = input_register_device(ts->cap_dev);
	if (err) {
		dev_err(&client->dev,
			"%s: unable to register %s cap device\n",
			__func__, ts->cap_dev->name);
		goto err_cap_input_register_device_failed;
	}

	/* allocate input device for digitizer */
	ts->dig_dev = input_allocate_device();
	if (ts->dig_dev == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev, "Failed to allocate dig device\n");
		goto err_dig_dev_alloc_failed;
	}

	set_bit(EV_KEY, ts->dig_dev->evbit);
	set_bit(EV_ABS, ts->dig_dev->evbit);
	set_bit(ABS_PRESSURE, ts->dig_dev->absbit);
	set_bit(ABS_X, ts->dig_dev->absbit);
	set_bit(ABS_Y, ts->dig_dev->absbit);
    	set_bit(BTN_TOUCH, ts->dig_dev->keybit);
    	set_bit(STYLUS_KEY1, ts->dig_dev->keybit);
    	set_bit(STYLUS_KEY2, ts->dig_dev->keybit);
    	set_bit(INPUT_PROP_DIRECT, ts->dig_dev->propbit);

	ts->dig_dev->name = "garuda-digitizer";
	ts->dig_dev->id.bustype = BUS_I2C;

	input_set_abs_params(ts->dig_dev, ABS_X, 0, DIG_MAX_X, 0, 0);
	input_set_abs_params(ts->dig_dev, ABS_Y, 0, DIG_MAX_Y, 0, 0);
	input_set_abs_params(ts->dig_dev, ABS_PRESSURE, 0, DIG_MAX_P, 0, 0);
	
	err = 0;
	err = input_register_device(ts->dig_dev);

	if (err) 
	{
		dev_err(&client->dev,
			"%s: unable to register %s dig device\n",
			__func__, ts->dig_dev->name);
		goto err_dig_input_register_device_failed;
	}

	/* allocate input device for eraser */
	ts->eraser_dev = input_allocate_device();
	if (ts->eraser_dev == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev, "Failed to allocate eraser device\n");
		goto err_eraser_dev_alloc_failed;
	}

	set_bit(EV_KEY, ts->eraser_dev->evbit);
	set_bit(EV_ABS, ts->eraser_dev->evbit);
	set_bit(ABS_PRESSURE, ts->eraser_dev->absbit);
	set_bit(ABS_X, ts->eraser_dev->absbit);
	set_bit(ABS_Y, ts->eraser_dev->absbit);
    	set_bit(BTN_TOUCH, ts->eraser_dev->keybit);
    	set_bit(INPUT_PROP_DIRECT, ts->eraser_dev->propbit);

	ts->eraser_dev->name = "garuda-eraser";
	ts->eraser_dev->id.bustype = BUS_I2C;


	input_set_abs_params(ts->eraser_dev, ABS_X, 0, DIG_MAX_X, 0, 0);
	input_set_abs_params(ts->eraser_dev, ABS_Y, 0, DIG_MAX_Y, 0, 0);
	input_set_abs_params(ts->eraser_dev, ABS_PRESSURE, 0, DIG_MAX_P, 0, 0);
	
	err = 0;
	err = input_register_device(ts->eraser_dev);

	if (err) 
	{
		dev_err(&client->dev,
			"%s: unable to register %s dig device\n",
			__func__, ts->eraser_dev->name);
		goto err_eraser_input_register_device_failed;
	}

	INIT_WORK(&ts->work, mg_i2c_work);


	/* request IRQ resouce */
	if (client->irq < 0) 
	{
		dev_err(&ts->client->dev,"No irq allocated in client resources!\n");
		goto err_dig_input_register_device_failed;
	}

	ts->irq =client->irq= SW_INT_IRQNO_PIO;//client->irq=SW_INT_IRQNO_PIO;
	err = request_irq(ts->irq, mg_irq,
		IRQF_TRIGGER_FALLING | IRQF_DISABLED, MG_DRIVER_NAME, ts);
	if (err < 0)  printk("[ZHANGDP] Err: unable to request irq %d \n", ts->irq);
#ifdef PIXCIR_CAP_CALIBRAION
	pixcir_cap_calibration();	
#endif
#if 1
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = mg_ts_early_suspend;
	ts->early_suspend.resume = mg_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
#endif


	private_ts = ts;

	return 0;
	
err_cap_input_register_device_failed:
	if (ts->cap_dev)
	{
		input_free_device(ts->cap_dev);	
	}
	
err_dig_input_register_device_failed:
	if (ts->dig_dev)
	{
		input_free_device(ts->dig_dev);
	}

err_eraser_input_register_device_failed:
	if (ts->dig_dev)
	{
		input_free_device(ts->eraser_dev);
	}

err_dig_dev_alloc_failed:	
err_cap_dev_alloc_failed:
err_eraser_dev_alloc_failed:
	if (ts->mg_wq)
	{
		destroy_workqueue(ts->mg_wq);
	}
			
err_create_wq_failed:	
	kfree(ts);
exit_gpio_int_request_failed:
err_alloc_data_failed:	
err_check_functionality_failed:	
exit_ioremap_failed:
    if(gpio_addr)
    {
        iounmap(gpio_addr);
    }
		
	return err;
}

static int __devexit mg_ts_remove(struct i2c_client *client)
{
	struct mg_ts_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	free_irq(ts->irq, ts);
	input_unregister_device(ts->cap_dev);
	input_unregister_device(ts->dig_dev);
	input_unregister_device(ts->eraser_dev);
	kfree(ts);
	if(gpio_addr)
	{
		iounmap(gpio_addr);
	}
	gpio_release(gpio_int_hdle, 2);
	gpio_release(gpio_wakeup_hdle, 2);
	gpio_release(gpio_reset_hdle, 2);
	gpio_release(gpio_enpwr_hdle, 2);	
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static int mg_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct	mg_ts_data *ts = i2c_get_clientdata(client);
	int rc = 0;

	printk(KERN_INFO "[MG] %s: enter\n", __func__);
	if (ts->irq)
	{
	//	disable_irq(client->irq);
	}

	rc = cancel_work_sync(&ts->work);
	//if (rc  && ts->irq)
	//	enable_irq(client->irq);

	rc = i2c_smbus_write_i2c_block_data(ts->client, 0, 5, command_list[8]);
	if (rc < 0)
	{
		printk(KERN_ERR "%s: i2c_smbus_write_i2c_block_data failed! err = %d\n",
				__func__, rc);
	}
	else 
	{
		printk(KERN_INFO "%s: i2c_smbus_write_i2c_block_data  success\n",__func__);
	}
	return 0;
}

static int mg_ts_resume(struct i2c_client *client)
{
	struct mg_ts_data *ts = i2c_get_clientdata(client);

	//reset
	ctp_ops.ts_reset();
	
	int rc = 0, retry = 5;

	printk(KERN_INFO "[MG] %s: enter\n", __func__);
	
	//if (ts->irq)
	//	enable_irq(client->irq);
		
	do {
		rc = i2c_smbus_write_i2c_block_data(ts->client, 0, 5, command_list[9]);
		if (rc < 0)
		{
			printk(KERN_ERR "[MG] %s: wake up tp failed! err = %d\n",
				__func__, rc);
		}
		else
		{
			printk(KERN_INFO "[MG] %s: wake up tp success!\n",__func__);

			break;
		}
	} while (--retry);
	return 0;
}

#endif


#ifdef CONFIG_HAS_EARLYSUSPEND
static void mg_ts_early_suspend(struct early_suspend *h)
{
	struct mg_ts_data *ts;
	ts = container_of(h, struct mg_ts_data, early_suspend);
	if(IAP_FLAG == 0)
	{
		mg_ts_suspend(ts->client, PMSG_SUSPEND);
	}
}
static void mg_ts_late_resume(struct early_suspend *h)
{
	struct mg_ts_data *ts;
	ts = container_of(h, struct mg_ts_data, early_suspend);
	if(IAP_FLAG == 0)
	{
		mg_ts_resume(ts->client);
	}
}
#endif

static const struct i2c_device_id mg_ts_id_table[] = {
    /* the slave address is passed by i2c_boardinfo */
    {MG_DRIVER_NAME, 0},
    {/* end of list */}
};


static struct i2c_driver mg_ts_driver = {
	.class = I2C_CLASS_HWMON,
	.probe		= mg_ts_probe,
	.remove		= mg_ts_remove,
#ifdef CONFIG_HAS_EARLYSUSPEND
#else
#ifdef CONFIG_PM
	.suspend	= mg_ts_suspend,
	.resume		= mg_ts_resume,
#endif
#endif
	.id_table	= mg_ts_id_table,
	.driver = {
		.owner	= THIS_MODULE,
		.name	 = MG_DRIVER_NAME,
		
	},
	.address_list	= u_i2c_addr.normal_i2c,
};


static int __init mg_ts_init(void)
{

	int ret = -1;
	int err = -1;

	printk("===========================%s kandy=====================\n", __func__);

	if (ctp_ops.fetch_sysconfig_para)
	{
		if(ctp_ops.fetch_sysconfig_para()){
			printk("%s: err.\n", __func__);
			return -1;
		}
	}
	printk("%s: after fetch_sysconfig_para:  normal_i2c: 0x%hx. normal_i2c[1]: 0x%hx \n", \
	__func__, u_i2c_addr.normal_i2c[0], u_i2c_addr.normal_i2c[1]);

	err = ctp_ops.init_platform_resource();
	if(0 != err){
		printk("%s:ctp_ops.init_platform_resource err. \n", __func__);    
	}

	//reset
	ctp_ops.ts_reset();
	//wakeup
	ctp_ops.ts_wakeup();
	
	mg_ts_driver.detect = ctp_ops.ts_detect;

	ret = i2c_add_driver(&mg_ts_driver);

	return ret;

}

static void mg_ts_exit(void)
{
	i2c_del_driver(&mg_ts_driver);
	ctp_ops.free_platform_resource();
}
module_init(mg_ts_init);
module_exit(mg_ts_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Makoto Kondo<kondo@uei.co.jp>");

