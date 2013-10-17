#ifndef _LINUX_MG_I2C_MTOUCH_H
#define _LINUX_MG_I2C_MTOUCH_H


// gpio base address
//#define PIO_BASE_ADDRESS             (0x01c20800)
//#define PIO_RANGE_SIZE               (0x400)
//#define CONFIG_FT5X0X_MULTITOUCH     (1)

//#define IRQ_EINT21                   (21) 
//#define IRQ_EINT29                   (29) 

//#define PIO_INT_STAT_OFFSET          (0x214)
//#define PIO_INT_CTRL_OFFSET          (0x210)
//#define PIO_INT_CFG2_OFFSET          (0x208)
//#define PIO_INT_CFG3_OFFSET          (0x20c)



#define CTP_IRQ_NO			(IRQ_EINT17)

#define CTP_IRQ_MODE			(NEGATIVE_EDGE)
#define CTP_NAME			"mg-i2c-mtouch"
#define TS_RESET_LOW_PERIOD		(15)
#define TS_INITIAL_HIGH_PERIOD		(15)
#define TS_WAKEUP_LOW_PERIOD	(100)
#define TS_WAKEUP_HIGH_PERIOD	(100)
#define TS_POLL_DELAY			(10)	/* ms delay between samples */
#define TS_POLL_PERIOD			(10)	/* ms delay between samples */
#define SCREEN_MAX_HEIGHT		(screen_max_x)
#define SCREEN_MAX_WIDTH		(screen_max_y)


                                 
#undef  AW_GPIO_INT_API_ENABLE

#define AW_GPIO_API_ENABLE


static u_int8_t command_list[12][5] = 
{
	{0x01, 0x02, 0x04 , 0x03 , 0x10 },// 0 Turn to Bootloader
	{0x01, 0x02, 0x04 , 0x03 , 0x00 },// 1 Turn to App
	{0x01, 0x02, 0x07 , 0x01 , 0xAA },// 2 Confirm Update Pen
	{0x01, 0x02, 0x06 , 0x03 , 0x02 },// 3  read firewire version
	{0x01, 0x02, 0x05 , 0x03 , 0x01 },// 4  read product ID
	{0x01, 0x02, 0x03 , 0x55 , 0xAA },// 5 InquireBusy
	{0x01, 0x02, 0x02 , 0xA1 , 0xA0 },// 6  read flash
	{0x01, 0x02, 0x01 , 0xA1 , 0xA0 },// 7  write flash
	{0xDE, 0x55, 0x00 , 0x00 , 0x00 },// 8  sleep
	{0xDE, 0x5A, 0x00 , 0x00 , 0x00 },// 9  wake up
	{0x01, 0x02, 0x08 , 0x01 , 0xBB },// 10 ReConfirm Update Pen
	{0xDE, 0x33, 0x00 , 0x00 , 0x00 } // 11 Cap calibration
};



#endif 	/* _LINUX_MG_I2C_MTOUCH_H */

