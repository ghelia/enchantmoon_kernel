/*
 * sound\soc\sun4i\spdif\sun4i_sndspdif.c
 * (C) Copyright 2007-2011
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * chenpailin <chenpailin@allwinnertech.com>
 *
 * some simple description for this code
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/mutex.h>

#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include <mach/sys_config.h>
#include <linux/io.h>

#include "sun4i_spdif.h"
#include "sun4i_spdma.h"

#include "sndspdif.h"

static int spdif_used = 0;
static struct clk *xtal;
static int clk_users;
static DEFINE_MUTEX(clk_lock);

#ifdef ENFORCE_RATES
static struct snd_pcm_hw_constraint_list hw_constraints_rates = {
	.count	= ARRAY_SIZE(rates),
	.list	= rates,
	.mask	= 0,
};
#endif

static int sun4i_sndspdif_startup(struct snd_pcm_substream *substream)
{
	int ret = 0;
	#ifdef ENFORCE_RATES
		struct snd_pcm_runtime *runtime = substream->runtime;;
	#endif
	if (!ret) {
	#ifdef ENFORCE_RATES
		ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE, &hw_constraints_rates);
		if (ret < 0)
			return ret;
	#endif
	}
	return ret;
}

static void sun4i_sndspdif_shutdown(struct snd_pcm_substream *substream)
{
	mutex_lock(&clk_lock);
	clk_users -= 1;
	if (clk_users == 0) {
		clk_put(xtal);
		xtal = NULL;
	}
	mutex_unlock(&clk_lock);
}

typedef struct __MCLK_SET_INF
{
    __u32   samp_rate;      // sample rate
	__u16 	mult_fs;        // multiply of smaple rate

    __u8    clk_div;        // mpll division
    __u8    mpll;           // select mpll, 0 - 24.576 Mhz, 1 - 22.5792 Mhz

} __mclk_set_inf;


typedef struct __BCLK_SET_INF
{
    __u8    bitpersamp;     // bits per sample
    __u8    clk_div;        // clock division
    __u16   mult_fs;        // multiplay of sample rate

} __bclk_set_inf;


static __bclk_set_inf BCLK_INF[] =
{
    // 16bits per sample
    {16,  4, 128}, {16,  6, 192}, {16,  8, 256},
    {16, 12, 384}, {16, 16, 512},

    //24 bits per sample
    {24,  4, 192}, {24,  8, 384}, {24, 16, 768},

    //32 bits per sample
    {32,  2, 128}, {32,  4, 256}, {32,  6, 384},
    {32,  8, 512}, {32, 12, 768},

    //end flag
    {0xff, 0, 0},
};

//TX RATIO value
static __mclk_set_inf  MCLK_INF[] =
{
	//88.2k bitrate    //2
    { 88200, 128,  2, 1}, { 88200, 256,  2, 1},
	
	 //22.05k bitrate   //8
    { 22050, 128,  8, 1}, { 22050, 256,  8, 1},
    { 22050, 512,  8, 1}, 
	
	// 24k bitrate   //8
    { 24000, 128,  8, 0}, { 24000, 256, 8, 0}, { 24000, 512, 8, 0},
 
    // 32k bitrate   //2.048MHz   24/4 = 6
    { 32000, 128,  6, 0}, { 32000, 192,  6, 0}, { 32000, 384,  6, 0},
    { 32000, 768,  6, 0},

     // 48K bitrate   3.072  Mbit/s   16/4 = 4
    { 48000, 128,  4, 0}, { 48000, 256,  4, 0}, { 48000, 512, 4, 0},   

    // 96k bitrate  6.144MHZ   8/4 = 2
    { 96000, 128 , 2, 0}, { 96000, 256,  2, 0},

    //192k bitrate   12.288MHZ  4/4 = 1
    {192000, 128,  1, 0},

    //44.1k bitrate  2.8224MHz   16/4 = 4
    { 44100, 128,  4, 1}, { 44100, 256,  4, 1}, { 44100, 512,  4, 1},

     //176.4k bitrate  11.2896MHZ 4/4 = 1
    {176400, 128, 1, 1},

    //end flag 0xffffffff
    {0xffffffff, 0, 0, 0},
};

static s32 get_clock_divder(u32 sample_rate, u32 sample_width, u32 * mclk_div, u32* mpll, u32* bclk_div, u32* mult_fs)
{
	u32 i, j, ret = -EINVAL;

	for(i=0; i< 100; i++) {
		 if((MCLK_INF[i].samp_rate == sample_rate) && 
		 	((MCLK_INF[i].mult_fs == 256) || (MCLK_INF[i].mult_fs == 128))) {
			  for(j=0; j<ARRAY_SIZE(BCLK_INF); j++) {
					if((BCLK_INF[j].bitpersamp == sample_width) && 
						(BCLK_INF[j].mult_fs == MCLK_INF[i].mult_fs)) {
						 *mclk_div = MCLK_INF[i].clk_div;
						 *mpll = MCLK_INF[i].mpll;
						 *bclk_div = BCLK_INF[j].clk_div;
						 *mult_fs = MCLK_INF[i].mult_fs;
						 ret = 0;
						 break;
					}
			  }
		 }
		 else if(MCLK_INF[i].samp_rate == 0xffffffff)
		 	break;
	}

	return ret;
}

static int sun4i_sndspdif_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret = 0;
	unsigned long rate = params_rate(params);
	unsigned int fmt = 0;
	u32 mclk_div=0, mpll=0, bclk_div=0, mult_fs=0;

	get_clock_divder(rate, 32, &mclk_div, &mpll, &bclk_div, &mult_fs);
	
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/*add the pcm and raw data select interface*/
	switch(params_channels(params)) {
		case 1:/*pcm mode*/
		case 2:
			fmt = 0;
			break;
		case 4:/*raw data mode*/
			fmt = 1;
			break;
		default:
			return -EINVAL;
	}
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);//0:pcm,1:raw data
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, 0 , mpll, 0);
	if (ret < 0)
		return ret;
		
	ret = snd_soc_dai_set_sysclk(codec_dai, 0 , mpll, 0);
	if (ret < 0)
		return ret;
		
	ret = snd_soc_dai_set_clkdiv(cpu_dai, SUN4I_DIV_MCLK, mclk_div);
	if (ret < 0)
		return ret;
		
	ret = snd_soc_dai_set_clkdiv(cpu_dai, SUN4I_DIV_BCLK, bclk_div);
	if (ret < 0)
		return ret;
		
	ret = snd_soc_dai_set_clkdiv(codec_dai, 0, mult_fs);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops sun4i_sndspdif_ops = {
	.startup 	= sun4i_sndspdif_startup,
	.shutdown 	= sun4i_sndspdif_shutdown,
	.hw_params 	= sun4i_sndspdif_hw_params,
};

static struct snd_soc_dai_link sun4i_sndspdif_dai_link = {
	.name 			= "SPDIF",
	.stream_name 	= "SUN4I-SPDIF",
	.cpu_dai_name 	= "sun4i-spdif.0",
	.codec_dai_name = "sndspdif",
	.platform_name 	= "sun4i-spdif-pcm-audio.0",
	.codec_name 	= "sun4i-spdif-codec.0",
	.ops 			= &sun4i_sndspdif_ops,
};

static struct snd_soc_card snd_soc_sun4i_sndspdif = {
	.name 		= "sun4i-sndspdif",
	.dai_link 	= &sun4i_sndspdif_dai_link,
	.num_links 	= 1,
};

static struct platform_device *sun4i_sndspdif_device;

static int __init sun4i_sndspdif_init(void)
{
	int ret;
	int ret2;
	
	ret2 = script_parser_fetch("spdif_para","spdif_used", &spdif_used, sizeof(int));
	if (ret2) {
        printk("[SPDIF]sun4i_sndspdif_init fetch spdif using configuration failed\n");
    } 
    
    if (spdif_used) {
		sun4i_sndspdif_device = platform_device_alloc("soc-audio", 1);
		
		if(!sun4i_sndspdif_device)
			return -ENOMEM;
			
		platform_set_drvdata(sun4i_sndspdif_device, &snd_soc_sun4i_sndspdif);
		
		ret = platform_device_add(sun4i_sndspdif_device);				
		if (ret) {			
			platform_device_put(sun4i_sndspdif_device);
		}
	} else {
		printk("[SPDIF]sun4i_sndspdif cannot find any using configuration for controllers, return directly!\n");
        return 0;
	}
		
	return ret;
}

static void __exit sun4i_sndspdif_exit(void)
{
	if (spdif_used) {
		spdif_used = 0;
		platform_device_unregister(sun4i_sndspdif_device);
	}
	
}

module_init(sun4i_sndspdif_init);
module_exit(sun4i_sndspdif_exit);

MODULE_AUTHOR("ALL WINNER");
MODULE_DESCRIPTION("SUN4I_SNDSPDIF ALSA SoC audio driver");
MODULE_LICENSE("GPL");

