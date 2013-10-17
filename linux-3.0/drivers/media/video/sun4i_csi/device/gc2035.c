/*
 * A V4L2 driver for GalaxyCore gc2035 cameras.
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-mediabus.h>//linux-3.0
#include <linux/io.h>
//#include <mach/gpio_v2.h>
#include <mach/sys_config.h>
#include <linux/regulator/consumer.h>
#include <mach/system.h>
#include "../../../../power/axp_power/axp-gpio.h"
#if defined CONFIG_ARCH_SUN4I
#include "../include/sun4i_csi_core.h"
#include "../include/sun4i_dev_csi.h"
#elif defined CONFIG_ARCH_SUN5I
#include "../include/sun5i_csi_core.h"
#include "../include/sun5i_dev_csi.h"
#endif



MODULE_AUTHOR("raymonxiu");
MODULE_DESCRIPTION("A low-level driver for GalaxyCore gc2035 sensors");
MODULE_LICENSE("GPL");

//for internel driver debug
#define DEV_DBG_EN   		0 
#if(DEV_DBG_EN == 1)		
#define csi_dev_dbg(x,arg...) printk(KERN_INFO"[CSI_DEBUG][GC2035]"x,##arg)
#else
#define csi_dev_dbg(x,arg...) 
#endif

#define csi_dev_err(x,arg...) printk(KERN_INFO"[CSI_ERR][GC2035]"x,##arg)
#define csi_dev_print(x,arg...) printk(KERN_INFO"[CSI][GC2035]"x,##arg)

#define MCLK (24*1000*1000)
#define VREF_POL	CSI_HIGH
#define HREF_POL	CSI_HIGH
#define CLK_POL		CSI_RISING
#define IO_CFG		0						//0:csi back 1:csi front
#define V4L2_IDENT_SENSOR 0x2035

//define the voltage level of control signal
#define CSI_STBY_ON			1
#define CSI_STBY_OFF 		0
#define CSI_RST_ON			0
#define CSI_RST_OFF			1
#define CSI_PWR_ON			1
#define CSI_PWR_OFF			0

#define REG_TERM 0xff
#define VAL_TERM 0xff


#define REG_ADDR_STEP 1
#define REG_DATA_STEP 1
#define REG_STEP 	     (REG_ADDR_STEP+REG_DATA_STEP)

/*
 * Basic window sizes.  These probably belong somewhere more globally
 * useful.
 */
#define UXGA_WIDTH		1600
#define UXGA_HEIGHT		1200

#define SXGA_WIDTH 	       1280
#define SXGA_HEIGHT   	1024




#define XGA_WIDTH 	       1024
#define XGA_HEIGHT 	       768

#define SVGA_WIDTH		800
#define SVGA_HEIGHT       600


#define QVGA_WIDTH		320
#define QVGA_HEIGHT	240

#define CIF_WIDTH		352
#define CIF_HEIGHT		288

#define QCIF_WIDTH		176
#define	QCIF_HEIGHT	144

/*
 * Our nominal (default) frame rate.
 */
#define SENSOR_FRAME_RATE 8

/*
 * The gc2035 sits on i2c with ID 0x78
 */
#define I2C_ADDR                    0x78   
#define GC2035_SENSOR_ID    0x2035
/* Registers */

/*
 * Information we maintain about a known sensor.
 */
struct sensor_format_struct;  /* coming later */
struct snesor_colorfx_struct; /* coming later */
__csi_subdev_info_t ccm_info_con = 
{
	.mclk 	= MCLK,
	.vref 	= VREF_POL,
	.href 	= HREF_POL,
	.clock	= CLK_POL,
	.iocfg	= IO_CFG,
};

struct sensor_info {
	struct v4l2_subdev sd;
	struct sensor_format_struct *fmt;  /* Current format */
	__csi_subdev_info_t *ccm_info;
	int	width;
	int	height;
	int brightness;
	int	contrast;
	int saturation;
	int hue;
	int hflip;
	int vflip;
	int gain;
	int autogain;
	int exp;
	enum v4l2_exposure_auto_type autoexp;
	int autowb;
	enum v4l2_whiteblance wb;
	enum v4l2_colorfx clrfx;
  enum v4l2_flash_mode flash_mode;
	u8 clkrc;			/* Clock divider value */
};

static inline struct sensor_info *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct sensor_info, sd);
}



/*
 * The default register settings
 *
 */

struct regval_list {
	unsigned char reg_num[REG_ADDR_STEP];
	unsigned char value[REG_DATA_STEP];
};



static struct regval_list sensor_default_regs[] = {
	{{0xfe},{0x80}},
	{{0xfe},{0x80}},
	{{0xfe},{0x80}},  
	{{0xfc},{0x06}},
	{{0xf2},{0x00}},
	{{0xf3},{0x00}},
	{{0xf4},{0x00}},
	{{0xf5},{0x00}},
	{{0xf9},{0xfe}}, //[0] pll enable
	{{0xfa},{0x00}},
	{{0xf6},{0x00}},
	{{0xf7},{0x15}}, //pll enable

	{{0xf8},{0x85}},
	{{0xfe},{0x00}},
	{{0x82},{0x00}},
	{{0xb3},{0x60}},
	{{0xb4},{0x40}},
	{{0xb5},{0x60}},

	{{0x03},{0x02}},
	{{0x04},{0x80}},

	//////////measure window  ///////////
	{{0xfe},{0x00}},
	{{0xec},{0x06}},//04 
	{{0xed},{0x06}},//04 
	{{0xee},{0x62}},//60 
	{{0xef},{0x92}},//90 

	///////////analog/////////////
	{{0x0a},{0x00}}, //row start
	{{0x0c},{0x00}}, //col start
	{{0x0d},{0x04}},
	{{0x0e},{0xc0}},
	{{0x0f},{0x06}}, //Window setting
	{{0x10},{0x58}}, 
	{{0x17},{0x14}}, //[0]mirror [1]flip


	{{0x18 },{0x0e}}, //sdark 4 row in even frame??
	{{0x19 },{0x0c}}, //AD pipe number

	/*
	///  Ã«´Ì ÏÖÏó
	{{0x18 },{0x0a}}, //sdark 4 row in even frame??
	{{0x19 },{0x0a}}, //AD pipe number
	*/
	{{0x1a},{0x01}}, //CISCTL mode4
	{{0x1b},{0x8b}},
	{{0x1e},{0x88}}, //analog mode1 [7] tx-high en [5:3]COL_bias
	{{0x1f},{0x08}}, //[3] tx-low en//
	{{0x20},{0x05}}, //[0]adclk mode},{0x[1]rowclk_MODE [2]rsthigh_en
	{{0x21},{0x0f}}, //[6:4]rsg
	{{0x22},{0xf0}}, //[3:0]vref
	{{0x23},{0xc3}}, //f3//ADC_r
	{{0x24},{0x17}}, //pad drive  16

	//AEC
	{{0xfe},{0x01}},
	{{0x11},{0x20}},//AEC_out_slope},{0x
	{{0x1f},{0xc0}},//max_post_gain
	{{0x20},{0x60}},//max_pre_gain
	{{0x47},{0x30}},//AEC_outdoor_th
	{{0x0b},{0x10}},//
	{{0x13},{0x75}},//y_target
	{{0xfe},{0x00}},


	{{0x05},{0x01}},//hb
	{{0x06},{0x11}},
	{{0x07},{0x00}},//vb
	{{0x08},{0x50}},
	{{0xfe},{0x01}},
	{{0x27},{0x00}},//step
	{{0x28},{0xa0}},
	{{0x29},{0x05}},//level1
	{{0x2a},{0x00}},
	{{0x2b},{0x05}},//level2
	{{0x2c},{0x00}},
	{{0x2d},{0x06}},//6e8//level3
	{{0x2e},{0xe0}},
	{{0x2f},{0x0a}},//level4
	{{0x30},{0x00}},
	{{0x3e },{0x40}},
	{{0xfe},{0x00}},
	{{0xfe},{0x00}},  //0x},{0x},{0x},{0x},{0x 
	{{0xb6},{0x03}}, //AEC enable
	{{0xfe},{0x00}},

	/////////BLK//////
	{{0x3f},{0x00}}, //prc close
	{{0x40},{0x77}},//
	{{0x42},{0x7f}},
	{{0x43},{0x30}},
	{{0x5c},{0x08}},
	{{0x5e},{0x20}},
	{{0x5f},{0x20}},
	{{0x60},{0x20}},
	{{0x61},{0x20}},
	{{0x62},{0x20}},
	{{0x63},{0x20}},
	{{0x64},{0x20}},
	{{0x65},{0x20}},

	///block////////////
	{{0x80},{0xff}},
	{{0x81},{0x26}},//38},{0x//skin_Y 8c_debug
	{{0x87},{0x90}}, //[7]middle gamma 
	{{0x84},{0x02}}, //output put foramat
	{{0x86},{0x07}}, //02 //sync plority 
	{{0x8b},{0xbc}},
	{{0xb0},{0x80}}, //globle gain
	{{0xc0},{0x40}},//Yuv bypass

	//////lsc/////////////
	{{0xfe},{0x01}},
	{{0xc2},{0x38}},
	{{0xc3},{0x25}},
	{{0xc4},{0x21}},
	{{0xc8},{0x19}},
	{{0xc9},{0x12}},
	{{0xca},{0x0e}},
	{{0xbc},{0x43}},
	{{0xbd},{0x18}},
	{{0xbe},{0x1b}},
	{{0xb6},{0x40}},
	{{0xb7},{0x2e}},
	{{0xb8},{0x26}},
	{{0xc5},{0x05}},
	{{0xc6},{0x03}},
	{{0xc7},{0x04}},
	{{0xcb},{0x00}},
	{{0xcc},{0x00}},
	{{0xcd},{0x00}},
	{{0xbf},{0x14}},
	{{0xc0},{0x22}},
	{{0xc1},{0x1b}},
	{{0xb9},{0x00}},
	{{0xba},{0x05}},
	{{0xbb},{0x05}},
	{{0xaa},{0x35}},
	{{0xab},{0x33}},
	{{0xac},{0x33}},
	{{0xad},{0x25}},
	{{0xae},{0x22}},
	{{0xaf},{0x27}},
	{{0xb0},{0x1d}},
	{{0xb1},{0x20}},
	{{0xb2},{0x22}},
	{{0xb3},{0x14}},
	{{0xb4},{0x15}},
	{{0xb5},{0x16}},
	{{0xd0},{0x00}},
	{{0xd2},{0x07}},
	{{0xd3},{0x08}},
	{{0xd8},{0x00}},
	{{0xda},{0x13}},
	{{0xdb},{0x17}},
	{{0xdc},{0x00}},
	{{0xde},{0x0a}},
	{{0xdf},{0x08}},
	{{0xd4},{0x00}},
	{{0xd6},{0x00}},
	{{0xd7},{0x0c}},
	{{0xa4},{0x00}},
	{{0xa5},{0x00}},
	{{0xa6},{0x00}},
	{{0xa7},{0x00}},
	{{0xa8},{0x00}},
	{{0xa9},{0x00}},
	{{0xa1},{0x80}},
	{{0xa2},{0x80}},

	//////////cc//////////////
	{{0xfe},{0x02}},
	{{0xc0},{0x01}},
	{{0xc1},{0x40}}, //Green_cc for d
	{{0xc2},{0xfc}},
	{{0xc3},{0x05}},
	{{0xc4},{0xec}},
	{{0xc5},{0x42}},
	{{0xc6},{0xf8}},
	{{0xc7},{0x40}},//for cwf 
	{{0xc8},{0xf8}},
	{{0xc9},{0x06}},
	{{0xca},{0xfd}},
	{{0xcb},{0x3e}},
	{{0xcc},{0xf3}},
	{{0xcd},{0x36}},//for A
	{{0xce},{0xf6}},
	{{0xcf},{0x04}},
	{{0xe3},{0x0c}},
	{{0xe4},{0x44}},
	{{0xe5},{0xe5}},
	{{0xfe},{0x00}},

	///////awb start ////////////////
	//AWB clear
	{{0xfe},{0x01}},
	{{0x4f},{0x00}},
	{{0x4d},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4d},{0x10}}, // 10
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4d},{0x20}}, // 20
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4d},{0x30}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}}, // 30
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4d},{0x40}}, // 40
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4d},{0x50}}, // 50
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4d},{0x60}}, // 60
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4d},{0x70}}, // 70
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4d},{0x80}}, // 80
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4d},{0x90}}, // 90
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4d},{0xa0}}, // a0
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4d},{0xb0}}, // b0
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4d},{0xc0}}, // c0
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4d},{0xd0}}, // d0
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4e},{0x00}},
	{{0x4f},{0x01}},
	/////// awb value////////
	{{0xfe},{0x01}},
	{{0x4f},{0x00}},
	{{0x4d},{0x30}},
	{{0x4e},{0x00}},
	{{0x4e},{0x80}},
	{{0x4e},{0x80}},
	{{0x4e},{0x02}},
	{{0x4e},{0x02}},
	{{0x4d},{0x40}},
	{{0x4e},{0x00}},
	{{0x4e},{0x80}},
	{{0x4e},{0x80}},
	{{0x4e},{0x02}},
	{{0x4e},{0x02}},
	{{0x4e},{0x02}},
	{{0x4d},{0x53}},
	{{0x4e},{0x08}},
	{{0x4e},{0x04}},
	{{0x4d},{0x62}},
	{{0x4e},{0x10}},
	{{0x4d},{0x72}},
	{{0x4e},{0x20}},
	{{0x4f},{0x01}},

	/////awb////
	{{0xfe},{0x01}},
	{{0x50},{0x88}},//c0//[6]green mode
	{{0x52},{0x40}},
	{{0x54},{0x60}},
	{{0x56},{0x06}},
	{{0x57},{0x20}}, //pre adjust
	{{0x58},{0x01}}, 
	{{0x5b},{0x02}}, //AWB_gain_delta
	{{0x61},{0xaa}},//R/G stand
	{{0x62},{0xaa}},//R/G stand
	{{0x71},{0x00}},
	{{0x74},{0x10}},  //0x//AWB_C_max
	{{0x77},{0x08}}, // 0x//AWB_p2_x
	{{0x78},{0xfd}}, //AWB_p2_y
	{{0x86},{0x30}},
	{{0x87},{0x00}},
	{{0x88},{0x04}},//06},{0x//[1]dark mode
	{{0x8a},{0xc0}},//awb move mode
	{{0x89},{0x75}},
	{{0x84},{0x08}},  //0x//auto_window
	{{0x8b},{0x00}}, // 0x//awb compare luma
	{{0x8d},{0x70}}, //awb gain limit R 
	{{0x8e},{0x70}},//G
	{{0x8f},{0xf4}},//B
	{{0xfe},{0x00}},
	{{0x82},{0x02}},//awb_en
       /////////awb end /////////////
       
	///==========asde
	{{0xfe},{0x01}},
	{{0x21},{0xbf}},
	{{0xfe},{0x02}},
	{{0xa4},{0x00}},//
	{{0xa5},{0x40}}, //lsc_th
	{{0xa2},{0xa0}}, //lsc_dec_slope
	{{0xa6},{0x80}}, //dd_th
	{{0xa7},{0x80}}, //ot_th
	{{0xab},{0x31}}, //
	{{0xa9},{0x6f}}, //
	{{0xb0},{0x99}}, //0x//edge effect slope low
	{{0xb1},{0x34}},//edge effect slope low
	{{0xb3},{0x80}}, //saturation dec slope
	{{0xde},{0xb6}},  //
	{{0x38},{0x0f}}, // 
	{{0x39},{0x60}}, //
	{{0xfe},{0x00}},
	{{0x81},{0x26}},
	{{0xfe},{0x02}},
	{{0x83},{0x00}},//
	{{0x84},{0x45}},//
	////////////YCP//////////
	{{0xd1},{0x38}},//saturation_cb
	{{0xd2},{0x38}},//saturation_Cr
	{{0xd3},{0x40}},//contrast ?	{{0xd4},{0x80}},//contrast center 
	{{0xd4 },{0x80}},//contrast center 
	{{0xd5},{0x00}},//luma_offset 
	{{0xdc},{0x30}},
	{{0xdd},{0xb8}},//edge_sa_g,b
	{{0xfe},{0x00}},
	///////dndd///////////
	{{0xfe},{0x02}},
	{{0x88},{0x15}},//dn_b_base
	{{0x8c},{0xf6}}, //[2]b_in_dark_inc
	{{0x89},{0x03}}, //dn_c_weight
	////////EE ///////////
	{{0xfe},{0x02}},
	{{0x90},{0x6c}},// EEINTP mode1
	{{0x97},{0x45}},// edge effect
	////==============RGB Gamma 
	{{0xfe},{0x02}},
	{{0x15},{0x0a}},
	{{0x16},{0x12}},
	{{0x17},{0x19}},
	{{0x18},{0x1f}},
	{{0x19},{0x2c}},
	{{0x1a},{0x38}},
	{{0x1b},{0x42}},
	{{0x1c},{0x4e}},
	{{0x1d},{0x63}},
	{{0x1e},{0x76}},
	{{0x1f},{0x87}},
	{{0x20},{0x96}},
	{{0x21},{0xa2}},
	{{0x22},{0xb8}},
	{{0x23},{0xca}},
	{{0x24},{0xd8}},
	{{0x25},{0xe3}},
	{{0x26},{0xf0}},
	{{0x27},{0xf8}},
	{{0x28},{0xfd}},
	{{0x29},{0xff}},

	///=================y gamma
	{{0xfe},{0x02}},
	{{0x2b},{0x00}},
	{{0x2c},{0x04}},
	{{0x2d},{0x09}},
	{{0x2e},{0x18}},
	{{0x2f},{0x27}},
	{{0x30},{0x37}},
	{{0x31},{0x49}},
	{{0x32},{0x5c}},
	{{0x33},{0x7e}},
	{{0x34},{0xa0}},
	{{0x35},{0xc0}},
	{{0x36},{0xe0}},
	{{0x37},{0xff}},
	/////1600x1200size// 
	{{0xfe},{0x00}},//
	{{0x90},{0x01}}, //0x//crop enable
	{{0x95},{0x04}},  //0x//1600x1200
	{{0x96},{0xb0}},
	{{0x97},{0x06}},
	{{0x98},{0x40}},

	{{0xfe},{0x03}},
	{{0x42},{0x40}}, 
	{{0x43},{0x06}}, //output buf width
	{{0x41},{0x02}}, // Pclk_polarity
	{{0x40},{0x40}},  //00  
	{{0x17},{0x00}}, //widv 
	{{0xfe},{0x00}},
	////output DVP/////
	{{0xfe },{0x00}},
	{{0xb6 },{0x03}},
	{{0xf7 },{0x15}},

	{{0xc8 },{0x00}},//close scaler
	{{0x99 },{0x22}},// 1/2 subsample
	{{0x9a },{0x06}},
	{{0x9b },{0x00}},
	{{0x9c },{0x00}},
	{{0x9d },{0x00}},
	{{0x9e },{0x00}},
	{{0x9f },{0x00}},
	{{0xa0 },{0x00}},  
	{{0xa1 },{0x00}},
	{{0xa2 },{0x00}},
	
	{{0x90 },{0x01}},  //crop enable
	{{0x94 },{0x02}},
	{{0x95 },{0x02}},
	{{0x96 },{0x58}},
	{{0x97 },{0x03}},
	{{0x98 },{0x20}},
	{{0xfe },{0x00}},
	{{0x82 },{0xfe}},  // fe
	{{0xf2 },{0x70}}, 
	{{0xf3 },{0xff}},
	{{0xf4 },{0x00}},
	{{0xf5 },{0x30}},
	
#if 0   
        /////////  re zao///
	{{0xfe },{0x00}},
	{{0x22 },{0xd0}},
	{{0xfe },{0x01}},
	{{0x21 },{0xff}},
	{{0xfe },{0x02}},  
	{{0x8a },{0x33}},
	{{0x8c },{0x76}},
	{{0x8d },{0x85}},
	{{0xa6 },{0xf0}},	
	{{0xae },{0x9f}},
	{{0xa2 },{0x90}},
	{{0xa5 },{0x40}},  
	{{0xa7 },{0x30}},
	{{0xb0 },{0x88}},
	{{0x38 },{0x0b}},
	{{0x39 },{0x30}},
	{{0xfe },{0x00}},  
	{{0x87 },{0xb0}},

       //// small  RGB gamma////
	{{0xfe},{0x02}},
	{{0x15},{0x0b}},
	{{0x16},{0x0e}},
	{{0x17},{0x10}},
	{{0x18},{0x12}},
	{{0x19},{0x19}},
	{{0x1a},{0x21}},
	{{0x1b},{0x29}},
	{{0x1c},{0x31}},
	{{0x1d},{0x41}},
	{{0x1e},{0x50}},
	{{0x1f},{0x5f}},
	{{0x20},{0x6d}},
	{{0x21},{0x79}},
	{{0x22},{0x91}},
	{{0x23},{0xa5}},
	{{0x24},{0xb9}},
	{{0x25},{0xc9}},
	{{0x26},{0xe1}},
	{{0x27},{0xee}},
	{{0x28},{0xf7}},
	{{0x29},{0xff}},
	
 	////dark sun/////
	{{0xfe},{0x02}},
	{{0x40},{0x06}},
	{{0x41},{0x23}},
	{{0x42},{0x3f}},
	{{0x43},{0x06}},
	{{0x44},{0x00}},
	{{0x45},{0x00}},
	{{0x46},{0x14}},
	{{0x47},{0x09}},
 
  #endif

};

/* 1600X1200 UXGA capture */
static struct regval_list sensor_uxga_regs[] ={
 
	{{0xfe},{0x00}},
	{{0xf7},{0x17}},
	{{0xc8},{0x00}},
	
	{{0xc8},{0x00}},//close scaler
	{{0x99},{0x11}},// 1/2 subsample
	{{0x9a},{0x06}},
	{{0x9b},{0x00}},
	{{0x9c},{0x00}},
	{{0x9d},{0x00}},
	{{0x9e},{0x00}},
	{{0x9f},{0x00}},
	{{0xa0},{0x00}},  
	{{0xa1},{0x00}},
	{{0xa2},{0x00}},
	
	{{0x90},{0x01}},
	{{0x95},{0x04}},
	{{0x96},{0xb0}},  
	{{0x97},{0x06}},
	{{0x98},{0x40}},
	 
};

/* 1280X1024 SXGA */
static struct regval_list sensor_sxga_regs[] =
{};
/*1024*768*/
static struct regval_list sensor_xga_regs[] =
{};
/* 800X600 SVGA,30fps*/
static struct regval_list sensor_svga_regs[] ={
		////////subsample   800X600///////
	{{0xfe},{0x00}},
	{{0xb6},{0x03}},
	{{0xf7},{0x15}},

	{{0xc8},{0x00}},//close scaler
	{{0x99},{0x22}},// 1/2 subsample
	{{0x9a},{0x06}},
	{{0x9b},{0x00}},
	{{0x9c},{0x00}},
	{{0x9d},{0x00}},
	{{0x9e},{0x00}},
	{{0x9f},{0x00}},
	{{0xa0},{0x00}},  
	{{0xa1},{0x00}},
	{{0xa2},{0x00}},
	
	{{0x90},{0x01}},  //crop enable
	{{0x95},{0x02}},
	{{0x96},{0x58}},
	{{0x97},{0x03}},
	{{0x98},{0x20}},
	
};





/*
 * The white balance settings
 * Here only tune the R G B channel gain. 
 * The white balance enalbe bit is modified in sensor_s_autowb and sensor_s_wb
 */
static struct regval_list sensor_wb_auto_regs[] = {

	{{0xb3},{0x61}},
	{{0xb4},{0x40}}, 
	{{0xb5},{0x61}},
			
};

static struct regval_list sensor_wb_cloud_regs[] = {
	{{0xb3},{0x58}},
	{{0xb4},{0x40}}, 
	{{0xb5},{0x50}},
			
};

static struct regval_list sensor_wb_daylight_regs[] = {
	//tai yang guang
	 //Sunny 
	{{0xb3},{0x78}},
	{{0xb4},{0x40}}, 
	{{0xb5},{0x50}},
		
};

static struct regval_list sensor_wb_incandescence_regs[] = {
	//bai re guang	
	{{0xb3},{0x50}},
	{{0xb4},{0x40}}, 
	{{0xb5},{0xa8}},
			
};

static struct regval_list sensor_wb_fluorescent_regs[] = {
	//ri guang deng
	{{0xb3},{0x72}},
	{{0xb4},{0x40}}, 
	{{0xb5},{0x5b}},
			
};

static struct regval_list sensor_wb_tungsten_regs[] = {
	//wu si deng
	{{0xb3},{0xa0}},
	{{0xb4},{0x45}}, 
	{{0xb5},{0x40}},
			
};

/*
 * The color effect settings
 */
 
static struct regval_list sensor_colorfx_none_regs[] = {
	{{0xfe},{0x00}}, 
	{{0x83},{0xe0}},
		

};

static struct regval_list sensor_colorfx_bw_regs[] = {
	
};

static struct regval_list sensor_colorfx_sepia_regs[] = {
	{{0xfe},{0x00}}, 
	{{0x83},{0x82}},
		

};

static struct regval_list sensor_colorfx_negative_regs[] = {
	{{0xfe},{0x00}}, 
	{{0x83},{0x01}},
		

};

static struct regval_list sensor_colorfx_emboss_regs[] = {
	{{0xfe},{0x00}}, 
	{{0x83},{0x12}},///CAM_EFFECT_ENC_GRAYSCALE
		
};

static struct regval_list sensor_colorfx_sketch_regs[] = {
//NULL
};

static struct regval_list sensor_colorfx_sky_blue_regs[] = {
	{{0xfe},{0x00}}, 
	{{0x83},{0x62}},
			

};

static struct regval_list sensor_colorfx_grass_green_regs[] = {
	{{0xfe},{0x00}}, 
	{{0x83},{0x52}},
		
};

static struct regval_list sensor_colorfx_skin_whiten_regs[] = {
//NULL
};

static struct regval_list sensor_colorfx_vivid_regs[] = {
//NULL
};

/*
 * The brightness setttings
 */
static struct regval_list sensor_brightness_neg4_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_neg3_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_neg2_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_neg1_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_zero_regs[] = {
//NULL	
};

static struct regval_list sensor_brightness_pos1_regs[] = {
	//NULL
};

static struct regval_list sensor_brightness_pos2_regs[] = {
//NULL	
};

static struct regval_list sensor_brightness_pos3_regs[] = {
//NULL	
};

static struct regval_list sensor_brightness_pos4_regs[] = {
//NULL
};

/*
 * The contrast setttings
 */
static struct regval_list sensor_contrast_neg4_regs[] = {

};

static struct regval_list sensor_contrast_neg3_regs[] = {
	 
};

static struct regval_list sensor_contrast_neg2_regs[] = {

};

static struct regval_list sensor_contrast_neg1_regs[] = {

};

static struct regval_list sensor_contrast_zero_regs[] = {

};

static struct regval_list sensor_contrast_pos1_regs[] = {

};

static struct regval_list sensor_contrast_pos2_regs[] = {

};

static struct regval_list sensor_contrast_pos3_regs[] = {
 
};

static struct regval_list sensor_contrast_pos4_regs[] = {

};

/*
 * The saturation setttings
 */
static struct regval_list sensor_saturation_neg4_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_neg3_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_neg2_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_neg1_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_zero_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_pos1_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_pos2_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_pos3_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_pos4_regs[] = {
//NULL
};

/*
 * The exposure target setttings
 */
static struct regval_list sensor_ev_neg4_regs[] = {
       {{0xfe},{0x01}},
	{{0x13},{0x40}},
	{{0xfe},{0x02}},
	{{0xd5},{0xc0}}, 
	{{0xfe},{0x00}},
			
};

static struct regval_list sensor_ev_neg3_regs[] = {
       {{0xfe},{0x01}},
	{{0x13},{0x50}}, 
	{{0xfe},{0x02}},
	{{0xd5},{0xd0}}, 
	{{0xfe},{0x00}},
			
};

static struct regval_list sensor_ev_neg2_regs[] = {
        {{0xfe},{0x01}},
	{{0x13},{0x60}}, 
	{{0xfe},{0x02}},
	{{0xd5},{0xe0}}, 
	{{0xfe},{0x00}},
			
};

static struct regval_list sensor_ev_neg1_regs[] = {
       {{0xfe},{0x01}},
	{{0x13},{0x70}}, 
	{{0xfe},{0x02}},
	{{0xd5},{0xf0}}, 
	{{0xfe},{0x00}},
			
};

static struct regval_list sensor_ev_zero_regs[] = {
       {{0xfe},{0x01}},
	{{0x13},{0x80}}, 
       {{0xfe},{0x02}},
	{{0xd5},{0x00}}, 
	{{0xfe},{0x00}},
			
};

static struct regval_list sensor_ev_pos1_regs[] = {
       {{0xfe},{0x01}},
	{{0x13},{0x98}}, 
	{{0xfe},{0x02}},
	{{0xd5},{0x10}}, 
	{{0xfe},{0x00}},
			
};

static struct regval_list sensor_ev_pos2_regs[] = {
	{{0xfe},{0x01}},
	{{0x13},{0xb0}}, 
	{{0xfe},{0x02}},
	{{0xd5},{0x20}}, 
	{{0xfe},{0x00}},
			
};

static struct regval_list sensor_ev_pos3_regs[] = {
       {{0xfe},{0x01}},
	{{0x13},{0xc0}}, 
	{{0xfe},{0x02}},
	{{0xd5},{0x30}}, 
	{{0xfe},{0x00}},	
			
};

static struct regval_list sensor_ev_pos4_regs[] = {
       {{0xfe},{0x01}},
	{{0x13},{0xd0}}, 
	{{0xfe},{0x02}},
	{{0xd5},{0x50}}, 
	{{0xfe},{0x00}},	
		
};


/*
 * Here we'll try to encapsulate the changes for just the output
 * video format.
 * 
 */

static struct regval_list sensor_fmt_yuv422_yuyv[] = {

};


static struct regval_list sensor_fmt_yuv422_yvyu[] = {

};

static struct regval_list sensor_fmt_yuv422_vyuy[] = {

};

static struct regval_list sensor_fmt_yuv422_uyvy[] = {

};

static struct regval_list sensor_fmt_raw[] = {

};



/*
 * Low-level register I/O.
 *
 */


/*
 * On most platforms, we'd rather do straight i2c I/O.
 */
static int sensor_read(struct v4l2_subdev *sd, unsigned char *reg,
		unsigned char *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 data[REG_STEP];
	struct i2c_msg msg;
	int ret,i;
	
	for(i = 0; i < REG_ADDR_STEP; i++)
		data[i] = reg[i];
	
	data[REG_ADDR_STEP] = 0xff;
	/*
	 * Send out the register address...
	 */
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = REG_ADDR_STEP;
	msg.buf = data;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		csi_dev_err("Error %d on register write\n", ret);
		return ret;
	}
	/*
	 * ...then read back the result.
	 */
	
	msg.flags = I2C_M_RD;
	msg.len = REG_DATA_STEP;
	msg.buf = &data[REG_ADDR_STEP];
	
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0) {
		for(i = 0; i < REG_DATA_STEP; i++)
			value[i] = data[i+REG_ADDR_STEP];
		ret = 0;
	}
	else {
		csi_dev_err("Error %d on register read\n", ret);
	}
	return ret;
}


static int sensor_write(struct v4l2_subdev *sd, unsigned char *reg,
		unsigned char *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_msg msg;
	unsigned char data[REG_STEP];
	int ret,i;
	
	for(i = 0; i < REG_ADDR_STEP; i++)
			data[i] = reg[i];
	for(i = REG_ADDR_STEP; i < REG_STEP; i++)
			data[i] = value[i-REG_ADDR_STEP];
	
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = REG_STEP;
	msg.buf = data;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret > 0) {
		ret = 0;
	}
	else if (ret < 0) {
		csi_dev_err("sensor_write error!\n");
	}
	return ret;
}



/*
 * Write a list of register settings;
 */
static int sensor_write_array(struct v4l2_subdev *sd, struct regval_list *vals , uint size)
{
	int i,ret;
	if (size == 0)
		return -EINVAL;
	
	for(i = 0; i < size ; i++)
	{
		if(vals->reg_num[0] == 0xff) {
			mdelay(vals->value[0]);
		} else {
			ret = sensor_write(sd, vals->reg_num, vals->value);
			if (ret < 0)
				{
					csi_dev_err("sensor_write_err!\n");
					return ret;
				}	
		}
		
		vals++;
	}

	return 0;
}

/*
 * CSI GPIO control
 */
static void csi_gpio_write(struct v4l2_subdev *sd, user_gpio_set_t *gpio, int status)
{
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
		
  if(gpio->port == 0xffff) {
    axp_gpio_set_io(gpio->port_num, 1);
    axp_gpio_set_value(gpio->port_num, status); 
  } else {
    gpio_write_one_pin_value(dev->csi_pin_hd,status,(char *)&gpio->gpio_name);
  }
}

static void csi_gpio_set_status(struct v4l2_subdev *sd, user_gpio_set_t *gpio, int status)
{
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
		
  if(gpio->port == 0xffff) {
    axp_gpio_set_io(gpio->port_num, status);
  } else {
    gpio_set_one_pin_io_status(dev->csi_pin_hd,status,(char *)&gpio->gpio_name);
  }
}

/*
 * Stuff that knows about the sensor.
 */
 
static int sensor_power(struct v4l2_subdev *sd, int on)
{
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	
  //make sure that no device can access i2c bus during sensor initial or power down
  //when using i2c_lock_adpater function, the following codes must not access i2c bus before calling i2c_unlock_adapter
  i2c_lock_adapter(client->adapter);

  //insure that clk_disable() and clk_enable() are called in pair 
  //when calling CSI_SUBDEV_STBY_ON/OFF and CSI_SUBDEV_PWR_ON/OFF  
  switch(on)
	{
		case CSI_SUBDEV_STBY_ON:
			csi_dev_dbg("CSI_SUBDEV_STBY_ON\n");
			//reset off io
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(10);
			//standby on io
			csi_gpio_write(sd,&dev->standby_io,CSI_STBY_ON);
			mdelay(10);
			csi_gpio_write(sd,&dev->standby_io,CSI_STBY_OFF);
			mdelay(10);
			csi_gpio_write(sd,&dev->standby_io,CSI_STBY_ON);
			mdelay(10);
			//inactive mclk after stadby in
			clk_disable(dev->csi_module_clk);
			//reset on io
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			mdelay(10);
			break;
		case CSI_SUBDEV_STBY_OFF:
			csi_dev_dbg("CSI_SUBDEV_STBY_OFF\n");
			//active mclk before stadby out
			clk_enable(dev->csi_module_clk);
			mdelay(10);
			//standby off io
			csi_gpio_write(sd,&dev->standby_io,CSI_STBY_OFF);
			mdelay(10);
			//reset off io
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(10);
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			mdelay(10);
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(10);
			break;
		case CSI_SUBDEV_PWR_ON:
			csi_dev_dbg("CSI_SUBDEV_PWR_ON\n");
			//power on reset
			csi_gpio_set_status(sd,&dev->standby_io,1);//set the gpio to output
			csi_gpio_set_status(sd,&dev->reset_io,1);//set the gpio to output
			csi_gpio_write(sd,&dev->standby_io,CSI_STBY_ON);
			//reset on io
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			mdelay(1);
			//active mclk before power on
			clk_enable(dev->csi_module_clk);
			mdelay(10);
			//power supply
			csi_gpio_write(sd,&dev->power_io,CSI_PWR_ON);
			mdelay(10);
			if(dev->dvdd) {
				regulator_enable(dev->dvdd);
				mdelay(10);
			}
			if(dev->avdd) {
				regulator_enable(dev->avdd);
				mdelay(10);
			}
			if(dev->iovdd) {
				regulator_enable(dev->iovdd);
				mdelay(10);
			}
			//standby off io
			csi_gpio_write(sd,&dev->standby_io,CSI_STBY_OFF);
			mdelay(10);
			//reset after power on
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(10);
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			mdelay(10);
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(10);
			break;
		case CSI_SUBDEV_PWR_OFF:
			csi_dev_dbg("CSI_SUBDEV_PWR_OFF\n");
			//standby and reset io
			csi_gpio_write(sd,&dev->standby_io,CSI_STBY_ON);
			mdelay(10);
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			mdelay(10);
			//power supply off
			if(dev->iovdd) {
				regulator_disable(dev->iovdd);
				mdelay(10);
			}
			if(dev->avdd) {
				regulator_disable(dev->avdd);
				mdelay(10);
			}
			if(dev->dvdd) {
				regulator_disable(dev->dvdd);
				mdelay(10);	
			}
			csi_gpio_write(sd,&dev->power_io,CSI_PWR_OFF);
			mdelay(10);
			//inactive mclk after power off
			clk_disable(dev->csi_module_clk);
			//set the io to hi-z
			csi_gpio_set_status(sd,&dev->reset_io,0);//set the gpio to input
			csi_gpio_set_status(sd,&dev->standby_io,0);//set the gpio to input
			break;
		default:
			return -EINVAL;
	}		

	//remember to unlock i2c adapter, so the device can access the i2c bus again
	i2c_unlock_adapter(client->adapter);	
	return 0;
}
 
static int sensor_reset(struct v4l2_subdev *sd, u32 val)
{
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);

	switch(val)
	{
		case CSI_SUBDEV_RST_OFF:
			csi_dev_dbg("CSI_SUBDEV_RST_OFF\n");
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(10);
			break;
		case CSI_SUBDEV_RST_ON:
			csi_dev_dbg("CSI_SUBDEV_RST_ON\n");
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			mdelay(10);
			break;
		case CSI_SUBDEV_RST_PUL:
			csi_dev_dbg("CSI_SUBDEV_RST_PUL\n");
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(10);
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_ON);
			mdelay(10);
			csi_gpio_write(sd,&dev->reset_io,CSI_RST_OFF);
			mdelay(10);
			break;
		default:
			return -EINVAL;
	}
		
	return 0;
}

static int sensor_detect(struct v4l2_subdev *sd)
{
	int ret;
	unsigned   int SENSOR_ID=0;
	struct regval_list regs;
	
	//regs.reg_num[0] = 0xfe;
	//regs.value[0] = 0x00; //PAGE 0x00
	//ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_detect!\n");
		return ret;
	}
	
	regs.reg_num[0] = 0xf0;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	SENSOR_ID|= (regs.value[0]<< 8);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_detect!\n");
		return ret;
	}
	
	regs.reg_num[0] = 0xf1;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	SENSOR_ID|= (regs.value[0]);
	printk("GC2035_SENSOR_ID=%x",SENSOR_ID);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_detect!\n");
		return ret;
	}
	
	if(SENSOR_ID != GC2035_SENSOR_ID)
		return -ENODEV;
	
	return 0;
}

static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	csi_dev_dbg("sensor_init\n");
	/*Make sure it is a target sensor*/
	ret = sensor_detect(sd);
	if (ret) {
		csi_dev_err("chip found is not an target chip.\n");
		return ret;
	}
	
	return sensor_write_array(sd, sensor_default_regs , ARRAY_SIZE(sensor_default_regs));
}

static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret=0;
	
	switch(cmd){
		case CSI_SUBDEV_CMD_GET_INFO: 
		{
			struct sensor_info *info = to_state(sd);
			__csi_subdev_info_t *ccm_info = arg;
			
			csi_dev_dbg("CSI_SUBDEV_CMD_GET_INFO\n");
			
			ccm_info->mclk 	=	info->ccm_info->mclk ;
			ccm_info->vref 	=	info->ccm_info->vref ;
			ccm_info->href 	=	info->ccm_info->href ;
			ccm_info->clock	=	info->ccm_info->clock;
			ccm_info->iocfg	=	info->ccm_info->iocfg;
	
			csi_dev_dbg("ccm_info.mclk=%x\n ",info->ccm_info->mclk);
			csi_dev_dbg("ccm_info.vref=%x\n ",info->ccm_info->vref);
			csi_dev_dbg("ccm_info.href=%x\n ",info->ccm_info->href);
			csi_dev_dbg("ccm_info.clock=%x\n ",info->ccm_info->clock);
			csi_dev_dbg("ccm_info.iocfg=%x\n ",info->ccm_info->iocfg);
			
			break;
		}
		case CSI_SUBDEV_CMD_SET_INFO:
		{
			struct sensor_info *info = to_state(sd);
			__csi_subdev_info_t *ccm_info = arg;
			
			csi_dev_dbg("CSI_SUBDEV_CMD_SET_INFO\n");
			
			info->ccm_info->mclk 	=	ccm_info->mclk 	;
			info->ccm_info->vref 	=	ccm_info->vref 	;
			info->ccm_info->href 	=	ccm_info->href 	;
			info->ccm_info->clock	=	ccm_info->clock	;
			info->ccm_info->iocfg	=	ccm_info->iocfg	;
			
			csi_dev_dbg("ccm_info.mclk=%x\n ",info->ccm_info->mclk);
			csi_dev_dbg("ccm_info.vref=%x\n ",info->ccm_info->vref);
			csi_dev_dbg("ccm_info.href=%x\n ",info->ccm_info->href);
			csi_dev_dbg("ccm_info.clock=%x\n ",info->ccm_info->clock);
			csi_dev_dbg("ccm_info.iocfg=%x\n ",info->ccm_info->iocfg);
			
			break;
		}
		default:
			return -EINVAL;
	}		
		return ret;
}


/*
 * Store information about the video data format. 
 */
static struct sensor_format_struct {
	__u8 *desc;
	//__u32 pixelformat;
	enum v4l2_mbus_pixelcode mbus_code;//linux-3.0
	struct regval_list *regs;
	int	regs_size;
	int bpp;   /* Bytes per pixel */
} sensor_formats[] = {
	{
		.desc		= "YUYV 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_YUYV8_2X8,//linux-3.0
		.regs 		= sensor_fmt_yuv422_yuyv,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_yuyv),
		.bpp		= 2,
	},
	{
		.desc		= "YVYU 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_YVYU8_2X8,//linux-3.0
		.regs 		= sensor_fmt_yuv422_yvyu,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_yvyu),
		.bpp		= 2,
	},
	{
		.desc		= "UYVY 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_UYVY8_2X8,//linux-3.0
		.regs 		= sensor_fmt_yuv422_uyvy,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_uyvy),
		.bpp		= 2,
	},
	{
		.desc		= "VYUY 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_VYUY8_2X8,//linux-3.0
		.regs 		= sensor_fmt_yuv422_vyuy,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_vyuy),
		.bpp		= 2,
	},
	{
		.desc		= "Raw RGB Bayer",
		.mbus_code	= V4L2_MBUS_FMT_SBGGR8_1X8,//linux-3.0
		.regs 		= sensor_fmt_raw,
		.regs_size = ARRAY_SIZE(sensor_fmt_raw),
		.bpp		= 1
	},
};
#define N_FMTS ARRAY_SIZE(sensor_formats)

	

/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */


static struct sensor_win_size {
	int	width;
	int	height;
	int	hstart;		/* Start/stop values for the camera.  Note */
	int	hstop;		/* that they do not always make complete */
	int	vstart;		/* sense to humans, but evidently the sensor */
	int	vstop;		/* will do the right thing... */
	struct regval_list *regs; /* Regs to tweak */
	int regs_size;
	int (*set_size) (struct v4l2_subdev *sd);
/* h/vref stuff */
} sensor_win_sizes[] = {
	/* UXGA */
	{
		.width			= UXGA_WIDTH,
		.height			= UXGA_HEIGHT,
		.regs 			= sensor_uxga_regs,
		.regs_size	= ARRAY_SIZE(sensor_uxga_regs),
		.set_size		= NULL,
	},
	/* XVGA */
	{
		.width			= SXGA_WIDTH,
		.height			= SXGA_HEIGHT,
		.regs				= sensor_sxga_regs,
		.regs_size	= ARRAY_SIZE(sensor_sxga_regs),
		.set_size		= NULL,
	},
		
	/* XGA */
	{
		.width			= XGA_WIDTH,
		.height			= XGA_HEIGHT,
		.regs				= sensor_xga_regs,
		.regs_size	= ARRAY_SIZE(sensor_xga_regs),
		.set_size		= NULL,
	},
	/* SVGA */
	{
		.width			= SVGA_WIDTH,
		.height			= SVGA_HEIGHT,
		.regs				= sensor_svga_regs,
		.regs_size	= ARRAY_SIZE(sensor_svga_regs),
		.set_size		= NULL,
	},
	
};

#define N_WIN_SIZES (ARRAY_SIZE(sensor_win_sizes))




static int sensor_enum_fmt(struct v4l2_subdev *sd, unsigned index,
                 enum v4l2_mbus_pixelcode *code)//linux-3.0
{
//	struct sensor_format_struct *ofmt;

	if (index >= N_FMTS)//linux-3.0
		return -EINVAL;

	*code = sensor_formats[index].mbus_code;//linux-3.0

//	ofmt = sensor_formats + fmt->index;
//	fmt->flags = 0;
//	strcpy(fmt->description, ofmt->desc);
//	fmt->pixelformat = ofmt->pixelformat;
	return 0;
}


static int sensor_try_fmt_internal(struct v4l2_subdev *sd,
		//struct v4l2_format *fmt,
		struct v4l2_mbus_framefmt *fmt,//linux-3.0
		struct sensor_format_struct **ret_fmt,
		struct sensor_win_size **ret_wsize)
{
	int index;
	struct sensor_win_size *wsize;
//	struct v4l2_pix_format *pix = &fmt->fmt.pix;//linux-3.0
	csi_dev_dbg("sensor_try_fmt_internal\n");
	for (index = 0; index < N_FMTS; index++)
		if (sensor_formats[index].mbus_code == fmt->code)//linux-3.0
			break;
	
	if (index >= N_FMTS) {
		/* default to first format */
		index = 0;
		fmt->code = sensor_formats[0].mbus_code;//linux-3.0
	}
	
	if (ret_fmt != NULL)
		*ret_fmt = sensor_formats + index;
		
	/*
	 * Fields: the sensor devices claim to be progressive.
	 */
	fmt->field = V4L2_FIELD_NONE;//linux-3.0
	
	
	/*
	 * Round requested image size down to the nearest
	 * we support, but not below the smallest.
	 */
	for (wsize = sensor_win_sizes; wsize < sensor_win_sizes + N_WIN_SIZES;
	     wsize++)
		if (fmt->width >= wsize->width && fmt->height >= wsize->height)//linux-3.0
			break;
	
	if (wsize >= sensor_win_sizes + N_WIN_SIZES)
		wsize--;   /* Take the smallest one */
	if (ret_wsize != NULL)
		*ret_wsize = wsize;
	/*
	 * Note the size we'll actually handle.
	 */
	fmt->width = wsize->width;//linux-3.0
	fmt->height = wsize->height;//linux-3.0
	//pix->bytesperline = pix->width*sensor_formats[index].bpp;//linux-3.0
	//pix->sizeimage = pix->height*pix->bytesperline;//linux-3.0
	
	return 0;
}

static int sensor_try_fmt(struct v4l2_subdev *sd, 
             struct v4l2_mbus_framefmt *fmt)//linux-3.0
{
	return sensor_try_fmt_internal(sd, fmt, NULL, NULL);
}

/*
 * Set a format.
 */
static int sensor_s_fmt(struct v4l2_subdev *sd, 
             struct v4l2_mbus_framefmt *fmt)//linux-3.0
{
	int ret;
	struct sensor_format_struct *sensor_fmt;
	struct sensor_win_size *wsize;
	struct sensor_info *info = to_state(sd);	
	unsigned  int  temp=0,shutter=0;
	struct regval_list regs;
	printk("chr wsize.width = [%d], wsize.height = [%d]\n", wsize->width, wsize->height);		
	csi_dev_dbg("sensor_s_fmt\n");

	//////////////shutter-gain///////////////
	ret = sensor_try_fmt_internal(sd, fmt, &sensor_fmt, &wsize);
	if (ret)
		return ret;



#if 1
	////////////////////////////////

	if((wsize->width==1600)&&(wsize->height==1200))  //capture mode  >640*480
	{
			printk(" read  2035 exptime 11111111\n" );

		regs.reg_num[0] = 0xfe;
		regs.value[0] = 0x00; //page 0
		sensor_write(sd, regs.reg_num, regs.value);
	   
		regs.reg_num[0] = 0xb6;
		regs.value[0] = 0x02; //turn off aec
		sensor_write(sd, regs.reg_num, regs.value);
		//printk(" regb6 = [%x]\n", regs.value[0] );
	     //  msleep(100);

		/*read shutter */
		regs.reg_num[0] = 0x03;
		sensor_read(sd, regs.reg_num, regs.value);

		 
		temp |= (regs.value[0]<< 8);
              printk(" read   0x03 = [%x]\n", regs.value[0]);	
			  
		regs.reg_num[0] = 0x04;	
		sensor_read(sd, regs.reg_num,regs.value);
		temp |= (regs.value[0] & 0xff);
		  printk(" read   0x04 = [%x]\n", regs.value[0]);
		  
		shutter=temp;
		printk(" shutter = [%x]\n", shutter);		
	
	}

	////////////////////////////////
	
#endif 


	

	sensor_write_array(sd, sensor_fmt->regs , sensor_fmt->regs_size);


	////////////////////////////

	#if 1

	if((wsize->width==640)&&(wsize->height==480))  //capture mode  >640*480
	{
			msleep(300);

	}
	if((wsize->width==800)&&(wsize->height==600))  //capture mode  >640*480
	{
			msleep(300);

	}
	#endif 

	////////////////////////////

	
	ret = 0;
	if (wsize->regs)
	{
		ret = sensor_write_array(sd, wsize->regs , wsize->regs_size);
		if (ret < 0)
			return ret;
	}
	
	if (wsize->set_size)
	{
		ret = wsize->set_size(sd);
		if (ret < 0)
			return ret;
	}

	

 //////////

 #if 1
	if((wsize->width==1600)&&(wsize->height==1200))//capture mode  james added
	{


	printk(" write  2035 exptime 22222222\n" );


	regs.reg_num[0] = 0xfe;
	regs.value[0] = 0x00; //page 0
	sensor_write(sd, regs.reg_num, regs.value);


		
	shutter= shutter /2;	// 2
	 printk(" cap shutter = [%x]\n", shutter);
	if(shutter < 1) shutter = 1;
	regs.reg_num[0] = 0x03;


	regs.value[0] = ((shutter>>8)&0xff); 

       printk(" write0x03 = [%x]\n", regs.value[0]);

	sensor_write(sd, regs.reg_num, regs.value);



	regs.reg_num[0] = 0x04;
	regs.value[0] = (shutter&0xff); 

	printk(" write0x03 = [%x]\n", regs.value[0]);

	sensor_write(sd, regs.reg_num, regs.value);	

	msleep(550);
	}
 
#endif	
/////////////////////////////


	info->fmt = sensor_fmt;
	info->width = wsize->width;
	info->height = wsize->height;

	return 0;
}

/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int sensor_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct sensor_info *info = to_state(sd);

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = 1;
	
	if (info->width > SVGA_WIDTH && info->height > SVGA_HEIGHT) {
		cp->timeperframe.denominator = SENSOR_FRAME_RATE/2;
	} 
	else {
		cp->timeperframe.denominator = SENSOR_FRAME_RATE;
	}
	
	return 0;
}

static int sensor_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
//	struct v4l2_captureparm *cp = &parms->parm.capture;
//	struct v4l2_fract *tpf = &cp->timeperframe;
//	struct sensor_info *info = to_state(sd);
//	int div;

//	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
//		return -EINVAL;
//	if (cp->extendedmode != 0)
//		return -EINVAL;

//	if (tpf->numerator == 0 || tpf->denominator == 0)
//		div = 1;  /* Reset to full rate */
//	else {
//		if (info->width > SVGA_WIDTH && info->height > SVGA_HEIGHT) {
//			div = (tpf->numerator*SENSOR_FRAME_RATE/2)/tpf->denominator;
//		}
//		else {
//			div = (tpf->numerator*SENSOR_FRAME_RATE)/tpf->denominator;
//		}
//	}	
//	
//	if (div == 0)
//		div = 1;
//	else if (div > 8)
//		div = 8;
//	
//	switch()
//	
//	info->clkrc = (info->clkrc & 0x80) | div;
//	tpf->numerator = 1;
//	tpf->denominator = sensor_FRAME_RATE/div;
//	
//	sensor_write(sd, REG_CLKRC, info->clkrc);
	return -EINVAL;
}


/* 
 * Code for dealing with controls.
 * fill with different sensor module
 * different sensor module has different settings here
 * if not support the follow function ,retrun -EINVAL
 */

/* *********************************************begin of ******************************************** */
static int sensor_queryctrl(struct v4l2_subdev *sd,
		struct v4l2_queryctrl *qc)
{
	/* Fill in min, max, step and default value for these controls. */
	/* see include/linux/videodev2.h for details */
	/* see sensor_s_parm and sensor_g_parm for the meaning of value */
	
	switch (qc->id) {
//	case V4L2_CID_BRIGHTNESS:
//		return v4l2_ctrl_query_fill(qc, -4, 4, 1, 1);
//	case V4L2_CID_CONTRAST:
//		return v4l2_ctrl_query_fill(qc, -4, 4, 1, 1);
//	case V4L2_CID_SATURATION:
//		return v4l2_ctrl_query_fill(qc, -4, 4, 1, 1);
//	case V4L2_CID_HUE:
//		return v4l2_ctrl_query_fill(qc, -180, 180, 5, 0);
	case V4L2_CID_VFLIP:
	case V4L2_CID_HFLIP:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
//	case V4L2_CID_GAIN:
//		return v4l2_ctrl_query_fill(qc, 0, 255, 1, 128);
//	case V4L2_CID_AUTOGAIN:
//		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 1);
	case V4L2_CID_EXPOSURE:
		return v4l2_ctrl_query_fill(qc, -4, 4, 1, 0);
	case V4L2_CID_EXPOSURE_AUTO:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
	case V4L2_CID_DO_WHITE_BALANCE:
		return v4l2_ctrl_query_fill(qc, 0, 5, 1, 0);
	case V4L2_CID_AUTO_WHITE_BALANCE:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 1);
	case V4L2_CID_COLORFX:
		return v4l2_ctrl_query_fill(qc, 0, 9, 1, 0);
	case V4L2_CID_CAMERA_FLASH_MODE:
	  return v4l2_ctrl_query_fill(qc, 0, 4, 1, 0);	
	}
	return -EINVAL;
}

static int sensor_g_hflip(struct v4l2_subdev *sd, __s32 *value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;
	
	regs.reg_num[0] = 0xfe;
	regs.value[0] = 0x00;		//page 0
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_g_hflip!\n");
		return ret;
	}
	
	regs.reg_num[0] = 0x17;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_g_hflip!\n");
		return ret;
	}
	
	regs.value[0] &= (1<<0);
	regs.value[0] = regs.value[0]>>0;		//0x29 bit0 is mirror
		
	*value = regs.value[0];

	info->hflip = *value;
	return 0;
}

static int sensor_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;
	
	regs.reg_num[0] = 0xfe;
	regs.value[0] = 0x00;		//page 0
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_s_hflip!\n");
		return ret;
	}
	regs.reg_num[0] = 0x17;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_s_hflip!\n");
		return ret;
	}
	
	switch (value) {
		case 0:
		  regs.value[0] &= 0xfe;
			break;
		case 1:
			regs.value[0] |= 0x01;
			break;
		default:
			return -EINVAL;
	}
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_s_hflip!\n");
		return ret;
	}
	
	mdelay(20);
	
	info->hflip = value;
	return 0;
}

static int sensor_g_vflip(struct v4l2_subdev *sd, __s32 *value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;
	
	regs.reg_num[0] = 0xfe;
	regs.value[0] = 0x00;		//page 0
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_g_vflip!\n");
		return ret;
	}
	
	regs.reg_num[0] = 0x17;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_g_vflip!\n");
		return ret;
	}
	
	regs.value[0] &= (1<<1);
	regs.value[0] = regs.value[0]>>1;		//0x29 bit1 is upsidedown
		
	*value = regs.value[0];

	info->vflip = *value;
	return 0;
}

static int sensor_s_vflip(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;
	
	regs.reg_num[0] = 0xfe;
	regs.value[0] = 0x00;		//page 0
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_s_vflip!\n");
		return ret;
	}
	
	regs.reg_num[0] = 0x17;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_s_vflip!\n");
		return ret;
	}
	
	switch (value) {
		case 0:
		  regs.value[0] &= 0xfd;
			break;
		case 1:
			regs.value[0] |= 0x02;
			break;
		default:
			return -EINVAL;
	}
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_s_vflip!\n");
		return ret;
	}
	
	mdelay(20);
	
	info->vflip = value;
	return 0;
}

static int sensor_g_autogain(struct v4l2_subdev *sd, __s32 *value)
{
	return -EINVAL;
}

static int sensor_s_autogain(struct v4l2_subdev *sd, int value)
{
	return -EINVAL;
}

static int sensor_g_autoexp(struct v4l2_subdev *sd, __s32 *value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;
	
	regs.reg_num[0] = 0xfe;
	regs.value[0] = 0x00; //page 0
	
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_g_autoexp!\n");
		return ret;
	}
	
	regs.reg_num[0] = 0xb6;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_g_autoexp!\n");
		return ret;
	}

	regs.value[0] &= 0x01;
	if (regs.value[0] == 0x01) {
		*value = V4L2_EXPOSURE_AUTO;
	}
	else
	{
		*value = V4L2_EXPOSURE_MANUAL;
	}
	
	info->autoexp = *value;
	return 0;
}

static int sensor_s_autoexp(struct v4l2_subdev *sd,
		enum v4l2_exposure_auto_type value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;
	
	regs.reg_num[0] = 0xfe;
	regs.value[0] = 0x00; //page 0
	
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_s_autoexp!\n");
		return ret;
	}
	
	regs.reg_num[0] = 0xb6;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_s_autoexp!\n");
		return ret;
	}

	switch (value) {
		case V4L2_EXPOSURE_AUTO:
		  regs.value[0] |= 0x01;
			break;
		case V4L2_EXPOSURE_MANUAL:
			regs.value[0] &= 0xfe;
			break;
		case V4L2_EXPOSURE_SHUTTER_PRIORITY:
			return -EINVAL;    
		case V4L2_EXPOSURE_APERTURE_PRIORITY:
			return -EINVAL;
		default:
			return -EINVAL;
	}
		
	//ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_s_autoexp!\n");
		return ret;
	}
	
	mdelay(10);
	
	info->autoexp = value;
	return 0;
}

static int sensor_g_autowb(struct v4l2_subdev *sd, int *value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;
	
	regs.reg_num[0] = 0xfe;
	regs.value[0] = 0x00; //page 0
	
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_g_autowb!\n");
		return ret;
	}
	
	regs.reg_num[0] = 0x82;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_g_autowb!\n");
		return ret;
	}

	regs.value[0] &= (1<<1);
	regs.value[0] = regs.value[0]>>1;		//0x42 bit1 is awb enable
		
	*value = regs.value[0];
	info->autowb = *value;
	
	return 0;
}

static int sensor_s_autowb(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	struct regval_list regs;
	
	ret = sensor_write_array(sd, sensor_wb_auto_regs, ARRAY_SIZE(sensor_wb_auto_regs));
	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_autowb!\n");
		return ret;
	}
	
	regs.reg_num[0] = 0x82;
	ret = sensor_read(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_read err at sensor_s_autowb!\n");
		return ret;
	}

	switch(value) {
	case 0:
		regs.value[0] &= 0xfd;
		break;
	case 1:
		regs.value[0] |= 0x02;
		break;
	default:
		break;
	}	
	ret = sensor_write(sd, regs.reg_num, regs.value);
	if (ret < 0) {
		csi_dev_err("sensor_write err at sensor_s_autowb!\n");
		return ret;
	}
	
	mdelay(10);
	
	info->autowb = value;
	return 0;
}

static int sensor_g_hue(struct v4l2_subdev *sd, __s32 *value)
{
	return -EINVAL;
}

static int sensor_s_hue(struct v4l2_subdev *sd, int value)
{
	return -EINVAL;
}

static int sensor_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	return -EINVAL;
}

static int sensor_s_gain(struct v4l2_subdev *sd, int value)
{
	return -EINVAL;
}
/* *********************************************end of ******************************************** */

static int sensor_g_brightness(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	
	*value = info->brightness;
	return 0;
}

static int sensor_s_brightness(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	
	switch (value) {
		case -4:
		  ret = sensor_write_array(sd, sensor_brightness_neg4_regs, ARRAY_SIZE(sensor_brightness_neg4_regs));
			break;
		case -3:
			ret = sensor_write_array(sd, sensor_brightness_neg3_regs, ARRAY_SIZE(sensor_brightness_neg3_regs));
			break;
		case -2:
			ret = sensor_write_array(sd, sensor_brightness_neg2_regs, ARRAY_SIZE(sensor_brightness_neg2_regs));
			break;   
		case -1:
			ret = sensor_write_array(sd, sensor_brightness_neg1_regs, ARRAY_SIZE(sensor_brightness_neg1_regs));
			break;
		case 0:   
			ret = sensor_write_array(sd, sensor_brightness_zero_regs, ARRAY_SIZE(sensor_brightness_zero_regs));
			break;
		case 1:
			ret = sensor_write_array(sd, sensor_brightness_pos1_regs, ARRAY_SIZE(sensor_brightness_pos1_regs));
			break;
		case 2:
			ret = sensor_write_array(sd, sensor_brightness_pos2_regs, ARRAY_SIZE(sensor_brightness_pos2_regs));
			break;	
		case 3:
			ret = sensor_write_array(sd, sensor_brightness_pos3_regs, ARRAY_SIZE(sensor_brightness_pos3_regs));
			break;
		case 4:
			ret = sensor_write_array(sd, sensor_brightness_pos4_regs, ARRAY_SIZE(sensor_brightness_pos4_regs));
			break;
		default:
			return -EINVAL;
	}
	
	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_brightness!\n");
		return ret;
	}
	
	mdelay(10);
	
	info->brightness = value;
	return 0;
}

static int sensor_g_contrast(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	
	*value = info->contrast;
	return 0;
}

static int sensor_s_contrast(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	
	switch (value) {
		case -4:
		  ret = sensor_write_array(sd, sensor_contrast_neg4_regs, ARRAY_SIZE(sensor_contrast_neg4_regs));
			break;
		case -3:
			ret = sensor_write_array(sd, sensor_contrast_neg3_regs, ARRAY_SIZE(sensor_contrast_neg3_regs));
			break;
		case -2:
			ret = sensor_write_array(sd, sensor_contrast_neg2_regs, ARRAY_SIZE(sensor_contrast_neg2_regs));
			break;   
		case -1:
			ret = sensor_write_array(sd, sensor_contrast_neg1_regs, ARRAY_SIZE(sensor_contrast_neg1_regs));
			break;
		case 0:   
			ret = sensor_write_array(sd, sensor_contrast_zero_regs, ARRAY_SIZE(sensor_contrast_zero_regs));
			break;
		case 1:
			ret = sensor_write_array(sd, sensor_contrast_pos1_regs, ARRAY_SIZE(sensor_contrast_pos1_regs));
			break;
		case 2:
			ret = sensor_write_array(sd, sensor_contrast_pos2_regs, ARRAY_SIZE(sensor_contrast_pos2_regs));
			break;	
		case 3:
			ret = sensor_write_array(sd, sensor_contrast_pos3_regs, ARRAY_SIZE(sensor_contrast_pos3_regs));
			break;
		case 4:
			ret = sensor_write_array(sd, sensor_contrast_pos4_regs, ARRAY_SIZE(sensor_contrast_pos4_regs));
			break;
		default:
			return -EINVAL;
	}
	
	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_contrast!\n");
		return ret;
	}
	
	mdelay(10);
	
	info->contrast = value;
	return 0;
}

static int sensor_g_saturation(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	
	*value = info->saturation;
	return 0;
}

static int sensor_s_saturation(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	
	switch (value) {
		case -4:
		  ret = sensor_write_array(sd, sensor_saturation_neg4_regs, ARRAY_SIZE(sensor_saturation_neg4_regs));
			break;
		case -3:
			ret = sensor_write_array(sd, sensor_saturation_neg3_regs, ARRAY_SIZE(sensor_saturation_neg3_regs));
			break;
		case -2:
			ret = sensor_write_array(sd, sensor_saturation_neg2_regs, ARRAY_SIZE(sensor_saturation_neg2_regs));
			break;   
		case -1:
			ret = sensor_write_array(sd, sensor_saturation_neg1_regs, ARRAY_SIZE(sensor_saturation_neg1_regs));
			break;
		case 0:   
			ret = sensor_write_array(sd, sensor_saturation_zero_regs, ARRAY_SIZE(sensor_saturation_zero_regs));
			break;
		case 1:
			ret = sensor_write_array(sd, sensor_saturation_pos1_regs, ARRAY_SIZE(sensor_saturation_pos1_regs));
			break;
		case 2:
			ret = sensor_write_array(sd, sensor_saturation_pos2_regs, ARRAY_SIZE(sensor_saturation_pos2_regs));
			break;	
		case 3:
			ret = sensor_write_array(sd, sensor_saturation_pos3_regs, ARRAY_SIZE(sensor_saturation_pos3_regs));
			break;
		case 4:
			ret = sensor_write_array(sd, sensor_saturation_pos4_regs, ARRAY_SIZE(sensor_saturation_pos4_regs));
			break;
		default:
			return -EINVAL;
	}
	
	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_saturation!\n");
		return ret;
	}
	
	mdelay(10);
	
	info->saturation = value;
	return 0;
}

static int sensor_g_exp(struct v4l2_subdev *sd, __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	
	*value = info->exp;
	return 0;
}

static int sensor_s_exp(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	
	switch (value) {
		case -4:
		  ret = sensor_write_array(sd, sensor_ev_neg4_regs, ARRAY_SIZE(sensor_ev_neg4_regs));
			break;
		case -3:
			ret = sensor_write_array(sd, sensor_ev_neg3_regs, ARRAY_SIZE(sensor_ev_neg3_regs));
			break;
		case -2:
			ret = sensor_write_array(sd, sensor_ev_neg2_regs, ARRAY_SIZE(sensor_ev_neg2_regs));
			break;   
		case -1:
			ret = sensor_write_array(sd, sensor_ev_neg1_regs, ARRAY_SIZE(sensor_ev_neg1_regs));
			break;
		case 0:   
			ret = sensor_write_array(sd, sensor_ev_zero_regs, ARRAY_SIZE(sensor_ev_zero_regs));
			break;
		case 1:
			ret = sensor_write_array(sd, sensor_ev_pos1_regs, ARRAY_SIZE(sensor_ev_pos1_regs));
			break;
		case 2:
			ret = sensor_write_array(sd, sensor_ev_pos2_regs, ARRAY_SIZE(sensor_ev_pos2_regs));
			break;	
		case 3:
			ret = sensor_write_array(sd, sensor_ev_pos3_regs, ARRAY_SIZE(sensor_ev_pos3_regs));
			break;
		case 4:
			ret = sensor_write_array(sd, sensor_ev_pos4_regs, ARRAY_SIZE(sensor_ev_pos4_regs));
			break;
		default:
			return -EINVAL;
	}
	
	if (ret < 0) {
		csi_dev_err("sensor_write_array err at sensor_s_exp!\n");
		return ret;
	}
	
	mdelay(10);
	
	info->exp = value;
	return 0;
}

static int sensor_g_wb(struct v4l2_subdev *sd, int *value)
{
	struct sensor_info *info = to_state(sd);
	enum v4l2_whiteblance *wb_type = (enum v4l2_whiteblance*)value;
	
	*wb_type = info->wb;
	
	return 0;
}

static int sensor_s_wb(struct v4l2_subdev *sd,
		enum v4l2_whiteblance value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	
	if (value == V4L2_WB_AUTO) {
		ret = sensor_s_autowb(sd, 1);
		return ret;
	} 
	else {
		ret = sensor_s_autowb(sd, 0);
		if(ret < 0) {
			csi_dev_err("sensor_s_autowb error, return %x!\n",ret);
			return ret;
		}
		
		switch (value) {
			case V4L2_WB_CLOUD:
			  ret = sensor_write_array(sd, sensor_wb_cloud_regs, ARRAY_SIZE(sensor_wb_cloud_regs));
				break;
			case V4L2_WB_DAYLIGHT:
				ret = sensor_write_array(sd, sensor_wb_daylight_regs, ARRAY_SIZE(sensor_wb_daylight_regs));
				break;
			case V4L2_WB_INCANDESCENCE:
				ret = sensor_write_array(sd, sensor_wb_incandescence_regs, ARRAY_SIZE(sensor_wb_incandescence_regs));
				break;    
			case V4L2_WB_FLUORESCENT:
				ret = sensor_write_array(sd, sensor_wb_fluorescent_regs, ARRAY_SIZE(sensor_wb_fluorescent_regs));
				break;
			case V4L2_WB_TUNGSTEN:   
				ret = sensor_write_array(sd, sensor_wb_tungsten_regs, ARRAY_SIZE(sensor_wb_tungsten_regs));
				break;
			default:
				return -EINVAL;
		} 
	}
	
	if (ret < 0) {
		csi_dev_err("sensor_s_wb error, return %x!\n",ret);
		return ret;
	}
	
	mdelay(10);
	
	info->wb = value;
	return 0;
}

static int sensor_g_colorfx(struct v4l2_subdev *sd,
		__s32 *value)
{
	struct sensor_info *info = to_state(sd);
	enum v4l2_colorfx *clrfx_type = (enum v4l2_colorfx*)value;
	
	*clrfx_type = info->clrfx;
	return 0;
}

static int sensor_s_colorfx(struct v4l2_subdev *sd,
		enum v4l2_colorfx value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	
	switch (value) {
	case V4L2_COLORFX_NONE:
	  ret = sensor_write_array(sd, sensor_colorfx_none_regs, ARRAY_SIZE(sensor_colorfx_none_regs));
		break;
	case V4L2_COLORFX_BW:
		ret = sensor_write_array(sd, sensor_colorfx_bw_regs, ARRAY_SIZE(sensor_colorfx_bw_regs));
		break;  
	case V4L2_COLORFX_SEPIA:
		ret = sensor_write_array(sd, sensor_colorfx_sepia_regs, ARRAY_SIZE(sensor_colorfx_sepia_regs));
		break;   
	case V4L2_COLORFX_NEGATIVE:
		ret = sensor_write_array(sd, sensor_colorfx_negative_regs, ARRAY_SIZE(sensor_colorfx_negative_regs));
		break;
	case V4L2_COLORFX_EMBOSS:   
		ret = sensor_write_array(sd, sensor_colorfx_emboss_regs, ARRAY_SIZE(sensor_colorfx_emboss_regs));
		break;
	case V4L2_COLORFX_SKETCH:     
		ret = sensor_write_array(sd, sensor_colorfx_sketch_regs, ARRAY_SIZE(sensor_colorfx_sketch_regs));
		break;
	case V4L2_COLORFX_SKY_BLUE:
		ret = sensor_write_array(sd, sensor_colorfx_sky_blue_regs, ARRAY_SIZE(sensor_colorfx_sky_blue_regs));
		break;
	case V4L2_COLORFX_GRASS_GREEN:
		ret = sensor_write_array(sd, sensor_colorfx_grass_green_regs, ARRAY_SIZE(sensor_colorfx_grass_green_regs));
		break;
	case V4L2_COLORFX_SKIN_WHITEN:
		ret = sensor_write_array(sd, sensor_colorfx_skin_whiten_regs, ARRAY_SIZE(sensor_colorfx_skin_whiten_regs));
		break;
	case V4L2_COLORFX_VIVID:
		ret = sensor_write_array(sd, sensor_colorfx_vivid_regs, ARRAY_SIZE(sensor_colorfx_vivid_regs));
		break;
	default:
		return -EINVAL;
	}
	
	if (ret < 0) {
		csi_dev_err("sensor_s_colorfx error, return %x!\n",ret);
		return ret;
	}
	
	mdelay(10);
	
	info->clrfx = value;
	return 0;
}

static int sensor_g_flash_mode(struct v4l2_subdev *sd,
    __s32 *value)
{
	struct sensor_info *info = to_state(sd);
	enum v4l2_flash_mode *flash_mode = (enum v4l2_flash_mode*)value;
	
	*flash_mode = info->flash_mode;
	return 0;
}

static int sensor_s_flash_mode(struct v4l2_subdev *sd,
    enum v4l2_flash_mode value)
{
	struct sensor_info *info = to_state(sd);
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
	int flash_on,flash_off;
	
	flash_on = (dev->flash_pol!=0)?1:0;
	flash_off = (flash_on==1)?0:1;
	
	switch (value) {
	case V4L2_FLASH_MODE_OFF:
		csi_gpio_write(sd,&dev->flash_io,flash_off);
		break;
	case V4L2_FLASH_MODE_AUTO:
		return -EINVAL;
		break;  
	case V4L2_FLASH_MODE_ON:
		csi_gpio_write(sd,&dev->flash_io,flash_on);
		break;   
	case V4L2_FLASH_MODE_TORCH:
		return -EINVAL;
		break;
	case V4L2_FLASH_MODE_RED_EYE:   
		return -EINVAL;
		break;
	default:
		return -EINVAL;
	}
	
	info->flash_mode = value;
	return 0;
}

static int sensor_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return sensor_g_brightness(sd, &ctrl->value);
	case V4L2_CID_CONTRAST:
		return sensor_g_contrast(sd, &ctrl->value);
	case V4L2_CID_SATURATION:
		return sensor_g_saturation(sd, &ctrl->value);
	case V4L2_CID_HUE:
		return sensor_g_hue(sd, &ctrl->value);	
	case V4L2_CID_VFLIP:
		return sensor_g_vflip(sd, &ctrl->value);
	case V4L2_CID_HFLIP:
		return sensor_g_hflip(sd, &ctrl->value);
	case V4L2_CID_GAIN:
		return sensor_g_gain(sd, &ctrl->value);
	case V4L2_CID_AUTOGAIN:
		return sensor_g_autogain(sd, &ctrl->value);
	case V4L2_CID_EXPOSURE:
		return sensor_g_exp(sd, &ctrl->value);
	case V4L2_CID_EXPOSURE_AUTO:
		return sensor_g_autoexp(sd, &ctrl->value);
	case V4L2_CID_DO_WHITE_BALANCE:
		return sensor_g_wb(sd, &ctrl->value);
	case V4L2_CID_AUTO_WHITE_BALANCE:
		return sensor_g_autowb(sd, &ctrl->value);
	case V4L2_CID_COLORFX:
		return sensor_g_colorfx(sd,	&ctrl->value);
	case V4L2_CID_CAMERA_FLASH_MODE:
		return sensor_g_flash_mode(sd, &ctrl->value);
	}
	return -EINVAL;
}

static int sensor_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return sensor_s_brightness(sd, ctrl->value);
	case V4L2_CID_CONTRAST:
		return sensor_s_contrast(sd, ctrl->value);
	case V4L2_CID_SATURATION:
		return sensor_s_saturation(sd, ctrl->value);
	case V4L2_CID_HUE:
		return sensor_s_hue(sd, ctrl->value);		
	case V4L2_CID_VFLIP:
		return sensor_s_vflip(sd, ctrl->value);
	case V4L2_CID_HFLIP:
		return sensor_s_hflip(sd, ctrl->value);
	case V4L2_CID_GAIN:
		return sensor_s_gain(sd, ctrl->value);
	case V4L2_CID_AUTOGAIN:
		return sensor_s_autogain(sd, ctrl->value);
	case V4L2_CID_EXPOSURE:
		return sensor_s_exp(sd, ctrl->value);
	case V4L2_CID_EXPOSURE_AUTO:
		return sensor_s_autoexp(sd,
				(enum v4l2_exposure_auto_type) ctrl->value);
	case V4L2_CID_DO_WHITE_BALANCE:
		return sensor_s_wb(sd,
				(enum v4l2_whiteblance) ctrl->value);	
	case V4L2_CID_AUTO_WHITE_BALANCE:
		return sensor_s_autowb(sd, ctrl->value);
	case V4L2_CID_COLORFX:
		return sensor_s_colorfx(sd,
				(enum v4l2_colorfx) ctrl->value);
	case V4L2_CID_CAMERA_FLASH_MODE:
	  return sensor_s_flash_mode(sd,
	      (enum v4l2_flash_mode) ctrl->value);
	}
	return -EINVAL;
}


static int sensor_g_chip_ident(struct v4l2_subdev *sd,
		struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_SENSOR, 0);
}


/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops sensor_core_ops = {
	.g_chip_ident = sensor_g_chip_ident,
	.g_ctrl = sensor_g_ctrl,
	.s_ctrl = sensor_s_ctrl,
	.queryctrl = sensor_queryctrl,
	.reset = sensor_reset,
	.init = sensor_init,
	.s_power = sensor_power,
	.ioctl = sensor_ioctl,
};

static const struct v4l2_subdev_video_ops sensor_video_ops = {
	.enum_mbus_fmt = sensor_enum_fmt,//linux-3.0
	.try_mbus_fmt = sensor_try_fmt,//linux-3.0
	.s_mbus_fmt = sensor_s_fmt,//linux-3.0
	.s_parm = sensor_s_parm,//linux-3.0
	.g_parm = sensor_g_parm,//linux-3.0
};

static const struct v4l2_subdev_ops sensor_ops = {
	.core = &sensor_core_ops,
	.video = &sensor_video_ops,
};

/* ----------------------------------------------------------------------- */

static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct sensor_info *info;
//	int ret;

	info = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;
	v4l2_i2c_subdev_init(sd, client, &sensor_ops);

	info->fmt = &sensor_formats[0];
	info->ccm_info = &ccm_info_con;
	
	info->brightness = 0;
	info->contrast = 0;
	info->saturation = 0;
	info->hue = 0;
	info->hflip = 0;
	info->vflip = 0;
	info->gain = 0;
	info->autogain = 1;
	info->exp = 0;
	info->autoexp = 0;
	info->autowb = 1;
	info->wb = 0;
	info->clrfx = 0;
//	info->clkrc = 1;	/* 30fps */

	return 0;
}


static int sensor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{ "gc2035", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

//linux-3.0
static struct i2c_driver sensor_driver = {
	.driver = {
		.owner = THIS_MODULE,
	.name = "gc2035",
	},
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};
static __init int init_sensor(void)
{
	return i2c_add_driver(&sensor_driver);
}

static __exit void exit_sensor(void)
{
  i2c_del_driver(&sensor_driver);
}

module_init(init_sensor);
module_exit(exit_sensor);
