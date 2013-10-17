/*
 * sound\soc\sun4i\i2s\sun4i-i2sdma.c
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
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <asm/dma.h>
#include <mach/hardware.h>
#include <mach/dma.h>

#include "sun4i-i2s.h"
#include "sun4i-i2sdma.h"

static volatile unsigned int capture_dmasrc = 0;
static volatile unsigned int capture_dmadst = 0;
static volatile unsigned int play_dmasrc = 0;
static volatile unsigned int play_dmadst = 0;

static const struct snd_pcm_hardware sun4i_pcm_play_hardware = {
	.info			= SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
				      SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
				      SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S24_LE,
	.rates			= SNDRV_PCM_RATE_8000_192000 | SNDRV_PCM_RATE_KNOT,
	.rate_min		= 8000,
	.rate_max		= 192000,
	.channels_min		= 1,
	.channels_max		= 2,
	.buffer_bytes_max	= 128*1024,    /* value must be (2^n)Kbyte size */
	.period_bytes_min	= 1024*4,//1024*4,
	.period_bytes_max	= 1024*32,//1024*32,
	.periods_min		= 4,//4,
	.periods_max		= 8,//8,
	.fifo_size		= 128,//32,
};

static const struct snd_pcm_hardware sun4i_pcm_capture_hardware = {
	.info			= SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
				      SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
				      SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S24_LE,
	.rates			= SNDRV_PCM_RATE_8000_192000 | SNDRV_PCM_RATE_KNOT,
	.rate_min		= 8000,
	.rate_max		= 192000,
	.channels_min		= 1,
	.channels_max		= 2,
	.buffer_bytes_max	= 128*1024,    /* value must be (2^n)Kbyte size */
	.period_bytes_min	= 1024*4,//1024*4,
	.period_bytes_max	= 1024*32,//1024*32,
	.periods_min		= 4,//4,
	.periods_max		= 8,//8,
	.fifo_size		= 128,//32,
};

struct sun4i_playback_runtime_data {
	spinlock_t lock;
	int state;
	unsigned int dma_loaded;
	unsigned int dma_limit;
	unsigned int dma_period;
	dma_addr_t dma_start;
	dma_addr_t dma_pos;
	dma_addr_t dma_end;
	struct sun4i_dma_params *params;
};

struct sun4i_capture_runtime_data {
	spinlock_t lock;
	int state;
	unsigned int dma_loaded;
	unsigned int dma_limit;
	unsigned int dma_period;
	dma_addr_t dma_start;
	dma_addr_t dma_pos;
	dma_addr_t dma_end;
	struct sun4i_dma_params *params;
};

static void sun4i_pcm_enqueue(struct snd_pcm_substream *substream)
{
	int play_ret = 0, capture_ret = 0;
	struct sun4i_playback_runtime_data *play_prtd = NULL;
	struct sun4i_capture_runtime_data *capture_prtd = NULL;
	dma_addr_t play_pos = 0, capture_pos = 0;
	unsigned long play_len = 0, capture_len = 0;
	unsigned int play_limit = 0, capture_limit = 0;
	
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		play_prtd = substream->runtime->private_data;
		play_pos = play_prtd->dma_pos;
		play_len = play_prtd->dma_period;
		play_limit = play_prtd->dma_limit;
		while (play_prtd->dma_loaded < play_limit) {
			if ((play_pos + play_len) > play_prtd->dma_end) {
				play_len  = play_prtd->dma_end - play_pos;
			}
			play_ret = sw_dma_enqueue(play_prtd->params->channel, substream, __bus_to_virt(play_pos),  play_len);
			if (play_ret == 0) {
				play_prtd->dma_loaded++;
				play_pos += play_prtd->dma_period;
				if(play_pos >= play_prtd->dma_end)
					play_pos = play_prtd->dma_start;
			} else {
				break;
			}
		}
		play_prtd->dma_pos = play_pos;
	} else {
		capture_prtd = substream->runtime->private_data;
		capture_pos = capture_prtd->dma_pos;
		capture_len = capture_prtd->dma_period;
		capture_limit = capture_prtd->dma_limit;
		while (capture_prtd->dma_loaded < capture_limit) {
			if ((capture_pos + capture_len) > capture_prtd->dma_end) {
				capture_len  = capture_prtd->dma_end - capture_pos;
			}
			capture_ret = sw_dma_enqueue(capture_prtd->params->channel, substream, __bus_to_virt(capture_pos), capture_len);
			if (capture_ret == 0) {
			capture_prtd->dma_loaded++;
			capture_pos += capture_prtd->dma_period;
			if (capture_pos >= capture_prtd->dma_end)
				capture_pos = capture_prtd->dma_start;
			} else {
				break;
			}
		}
		capture_prtd->dma_pos = capture_pos;
	}
}

static void sun4i_audio_capture_buffdone(struct sw_dma_chan *channel, 
		                                  void *dev_id, int size,
		                                  enum sw_dma_buffresult result)
{
	struct sun4i_capture_runtime_data *capture_prtd;
	struct snd_pcm_substream *substream = dev_id;
	//printk("%s,line:%d\n", __func__, __LINE__);
	if (result == SW_RES_ABORT || result == SW_RES_ERR)
		return;
		
	capture_prtd = substream->runtime->private_data;
		if (substream) {				
			snd_pcm_period_elapsed(substream);
		}	

	spin_lock(&capture_prtd->lock);
	{
		capture_prtd->dma_loaded--;
		sun4i_pcm_enqueue(substream);
	}
	spin_unlock(&capture_prtd->lock);
	//printk("%s,line:%d\n", __func__, __LINE__);
}

static void sun4i_audio_play_buffdone(struct sw_dma_chan *channel, 
		                                  void *dev_id, int size,
		                                  enum sw_dma_buffresult result)
{
	struct sun4i_playback_runtime_data *play_prtd;
	struct snd_pcm_substream *substream = dev_id;
	//printk("%s,line:%d\n", __func__, __LINE__);
	if (result == SW_RES_ABORT || result == SW_RES_ERR)
		return;
		
	play_prtd = substream->runtime->private_data;
	if (substream) {
		snd_pcm_period_elapsed(substream);
	}

	spin_lock(&play_prtd->lock);
	{
		play_prtd->dma_loaded--;
		sun4i_pcm_enqueue(substream);
	}
	spin_unlock(&play_prtd->lock);
	//printk("%s,line:%d\n", __func__, __LINE__);
}

static snd_pcm_uframes_t sun4i_pcm_pointer(struct snd_pcm_substream *substream)
{
	unsigned long play_res = 0, capture_res = 0;
	struct sun4i_playback_runtime_data *play_prtd = NULL;
	struct sun4i_capture_runtime_data *capture_prtd = NULL;
    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
    	play_prtd = substream->runtime->private_data;
   		spin_lock(&play_prtd->lock);
		sw_dma_getcurposition(DMACH_NIIS_PLAY, (dma_addr_t*)&play_dmasrc, (dma_addr_t*)&play_dmadst);
		play_res = play_dmasrc + play_prtd->dma_period - play_prtd->dma_start;
		spin_unlock(&play_prtd->lock);
		if (play_res >= snd_pcm_lib_buffer_bytes(substream)) {
			if (play_res == snd_pcm_lib_buffer_bytes(substream))
				play_res = 0;
		}
		return bytes_to_frames(substream->runtime, play_res);
    } else {
    	//printk("%s,line:%d\n", __func__, __LINE__);
    	capture_prtd = substream->runtime->private_data;
    	spin_lock(&capture_prtd->lock);
    	sw_dma_getcurposition(DMACH_NIIS_CAPTURE, (dma_addr_t*)&capture_dmasrc, (dma_addr_t*)&capture_dmadst);
    	capture_res = capture_dmadst + capture_prtd->dma_period - capture_prtd->dma_start;
    	spin_unlock(&capture_prtd->lock);
    	//printk("%s,line:%d,capture_res:%lu\n", __func__, __LINE__, capture_res);
    	if (capture_res >= snd_pcm_lib_buffer_bytes(substream)) {
			if (capture_res == snd_pcm_lib_buffer_bytes(substream))
				capture_res = 0;
		}
		//printk("%s,line:%d,capture_res:%lu\n", __func__, __LINE__, capture_res);
		return bytes_to_frames(substream->runtime, capture_res);
    }
}

static int sun4i_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	int play_ret = 0, capture_ret = 0;
    struct snd_pcm_runtime *play_runtime = NULL, *capture_runtime = NULL;
    struct sun4i_playback_runtime_data *play_prtd = NULL;
    struct sun4i_capture_runtime_data *capture_prtd = NULL;
    struct snd_soc_pcm_runtime *play_rtd = NULL;
    struct snd_soc_pcm_runtime *capture_rtd = NULL;
    struct sun4i_dma_params *play_dma = NULL;
    struct sun4i_dma_params *capture_dma = NULL;
    unsigned long play_totbytes = 0, capture_totbytes = 0;
    
    if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
    	play_runtime = substream->runtime;
    	play_prtd = play_runtime ->private_data;
    	play_rtd = substream->private_data;
    	play_totbytes = params_buffer_bytes(params);
    	play_dma = snd_soc_dai_get_dma_data(play_rtd->cpu_dai, substream);
    	
    	if (!play_dma) {
    		return 0;
    	}
		
		if (play_prtd->params == NULL) {
			play_prtd->params = play_dma;
			play_ret = sw_dma_request(play_prtd->params->channel,
						  play_prtd->params->client, NULL);
			if (play_ret < 0) {
					return play_ret;
			}
		}

		sw_dma_set_buffdone_fn(play_prtd->params->channel,
					    sun4i_audio_play_buffdone);
		snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
		play_runtime->dma_bytes = play_totbytes;
		
		spin_lock_irq(&play_prtd->lock);
		play_prtd->dma_loaded = 0;
		play_prtd->dma_limit = play_runtime->hw.periods_min;
		play_prtd->dma_period = params_period_bytes(params);
		play_prtd->dma_start = play_runtime->dma_addr;
		play_prtd->dma_pos = play_prtd->dma_start;
		play_prtd->dma_end = play_prtd->dma_start + play_totbytes;
		spin_unlock_irq(&play_prtd->lock);
    
    } else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
    	capture_runtime = substream->runtime;
    	capture_prtd = capture_runtime ->private_data;
    	capture_rtd = substream->private_data;
    	capture_totbytes = params_buffer_bytes(params);
    	capture_dma = snd_soc_dai_get_dma_data(capture_rtd->cpu_dai, substream);
    	
    	if (!capture_dma) {
    		return 0;
    	}
		
		if (capture_prtd->params == NULL) {
			capture_prtd->params = capture_dma;
			capture_ret = sw_dma_request(capture_prtd->params->channel,
						  capture_prtd->params->client, NULL);
			if (capture_ret < 0) {
					return capture_ret;
			}
		}

		sw_dma_set_buffdone_fn(capture_prtd->params->channel,
					    sun4i_audio_capture_buffdone);
		snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
		capture_runtime->dma_bytes = capture_totbytes;
		
		spin_lock_irq(&capture_prtd->lock);
		capture_prtd->dma_loaded = 0;
		capture_prtd->dma_limit = capture_runtime->hw.periods_min;
		capture_prtd->dma_period = params_period_bytes(params);
		capture_prtd->dma_start = capture_runtime->dma_addr;
		capture_prtd->dma_pos = capture_prtd->dma_start;
		capture_prtd->dma_end = capture_prtd->dma_start + capture_totbytes;
		spin_unlock_irq(&capture_prtd->lock);
    
    } else {
    	return -EINVAL;
    }
    
	return 0;
}

static int sun4i_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct sun4i_playback_runtime_data *play_prtd = NULL;
	struct sun4i_capture_runtime_data *capture_prtd = NULL;	

   	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
 		play_prtd = substream->runtime->private_data;
 		/* TODO - do we need to ensure DMA flushed */
		if (play_prtd->params) {
	  		sw_dma_ctrl(play_prtd->params->channel, SW_DMAOP_FLUSH);
		}
		
		snd_pcm_set_runtime_buffer(substream, NULL);
		
		if (play_prtd->params) {
			sw_dma_free(play_prtd->params->channel, play_prtd->params->client);
			play_prtd->params = NULL;
		}
   	} else if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		capture_prtd = substream->runtime->private_data;
   		/* TODO - do we need to ensure DMA flushed */
		if (capture_prtd->params) {
	  		sw_dma_ctrl(capture_prtd->params->channel, SW_DMAOP_FLUSH);
		}
		
		snd_pcm_set_runtime_buffer(substream, NULL);
		
		if (capture_prtd->params) {
			sw_dma_free(capture_prtd->params->channel, capture_prtd->params->client);
			capture_prtd->params = NULL;
		}
   	} else {
		return -EINVAL;
	}
	
	return 0;
}

static int sun4i_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct dma_hw_conf codec_play_dma_conf;
	struct dma_hw_conf codec_capture_dma_conf;
	int play_ret = 0, capture_ret = 0;
	struct sun4i_playback_runtime_data *play_prtd = NULL;
	struct sun4i_capture_runtime_data *capture_prtd = NULL;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		printk("%s,%d\n", __func__, __LINE__);
		play_prtd = substream->runtime->private_data;
		if (!play_prtd->params) {
			return 0;
		}

		codec_play_dma_conf.drqsrc_type  = DRQ_TYPE_SDRAM;
		codec_play_dma_conf.drqdst_type  = DRQ_TYPE_IIS;
		codec_play_dma_conf.xfer_type    = DMAXFER_D_BHALF_S_BHALF;
		codec_play_dma_conf.address_type = DMAADDRT_D_FIX_S_INC;
		codec_play_dma_conf.dir          = SW_DMA_WDEV;
		codec_play_dma_conf.reload       = 0;
		codec_play_dma_conf.hf_irq       = SW_DMA_IRQ_FULL;
		codec_play_dma_conf.from         = play_prtd->dma_start;
		codec_play_dma_conf.to           = play_prtd->params->dma_addr;
		play_ret = sw_dma_config(play_prtd->params->channel, &codec_play_dma_conf);

		/* flush the DMA channel */
		sw_dma_ctrl(play_prtd->params->channel, SW_DMAOP_FLUSH);
		play_prtd->dma_loaded = 0;
		play_prtd->dma_pos = play_prtd->dma_start;

		/* enqueue dma buffers */
		sun4i_pcm_enqueue(substream);
		
		return play_ret;
	} else {
		printk("%s,%d\n", __func__, __LINE__);		
		capture_prtd = substream->runtime->private_data;
		
		if (!capture_prtd->params) {
			return 0;
		}

		codec_capture_dma_conf.drqsrc_type  = DRQ_TYPE_IIS;
		codec_capture_dma_conf.drqdst_type  = DRQ_TYPE_SDRAM;
		codec_capture_dma_conf.xfer_type    = DMAXFER_D_BHALF_S_BHALF;
		codec_capture_dma_conf.address_type = DMAADDRT_D_INC_S_FIX;
		codec_capture_dma_conf.dir          = SW_DMA_RDEV;
		codec_capture_dma_conf.reload       = 0;
		codec_capture_dma_conf.hf_irq       = SW_DMA_IRQ_FULL;
		codec_capture_dma_conf.from         = capture_prtd->params->dma_addr;
		codec_capture_dma_conf.to           = capture_prtd->dma_start;
		capture_ret = sw_dma_config(capture_prtd->params->channel, &codec_capture_dma_conf);

		/* flush the DMA channel */
		sw_dma_ctrl(capture_prtd->params->channel, SW_DMAOP_FLUSH);
		capture_prtd->dma_loaded = 0;
		capture_prtd->dma_pos = capture_prtd->dma_start;

		/* enqueue dma buffers */
		sun4i_pcm_enqueue(substream);
		
		return capture_ret;
	}	
}

static int sun4i_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int ret = 0;
	struct sun4i_playback_runtime_data *play_prtd = NULL;
	struct sun4i_capture_runtime_data *capture_prtd = NULL;
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		play_prtd = substream->runtime->private_data;
		spin_lock(&play_prtd->lock);
			
		switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			printk("[IIS] playback dma trigger start\n");
			//printk("[IIS] 0x01c22400+0x24 = %#x, line= %d\n", readl(0xf1c22400+0x24), __LINE__);
			sw_dma_ctrl(play_prtd->params->channel, SW_DMAOP_START);
			break;			
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	        printk("[IIS] playback dma trigger stop\n");
	        //printk("[IIS] 0x01c22400+0x24 = %#x, line= %d\n", readl(0xf1c22400+0x24), __LINE__);
			sw_dma_ctrl(play_prtd->params->channel, SW_DMAOP_STOP);
			break;	
		default:
			ret = -EINVAL;
			break;
		}
		spin_unlock(&play_prtd->lock);			
	} else {
		capture_prtd = substream->runtime->private_data;
		spin_lock(&capture_prtd->lock);

		switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			printk("[IIS] capture dma trigger start\n");
			//printk("[IIS] 0x01c22400+0x00 = %#x, line= %d\n", readl(0xf1c22400+0x00), __LINE__);
			//printk("[IIS] 0x01c22400+0x04 = %#x, line= %d\n", readl(0xf1c22400+0x04), __LINE__);
			//printk("[IIS] 0x01c22400+0x14 = %#x, line= %d\n", readl(0xf1c22400+0x14), __LINE__);
			//printk("[IIS] 0x01c22400+0x18 = %#x, line= %d\n", readl(0xf1c22400+0x18), __LINE__);
			//printk("[IIS] 0x01c22400+0x1c = %#x, line= %d\n", readl(0xf1c22400+0x1c), __LINE__);
			//printk("[IIS] 0x01c22400+0x20 = %#x, line= %d\n", readl(0xf1c22400+0x20), __LINE__);
			//printk("[IIS] 0x01c22400+0x2c = %#x, line= %d\n", readl(0xf1c22400+0x2c), __LINE__);
			//printk("[IIS] 0x01c22400+0x38 = %#x, line= %d\n", readl(0xf1c22400+0x38), __LINE__);
			//printk("[IIS] 0x01c22400+0x3c = %#x, line= %d\n", readl(0xf1c22400+0x3c), __LINE__);
			sw_dma_ctrl(capture_prtd->params->channel, SW_DMAOP_START);
			break;			
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	        printk("[IIS] capture dma trigger stop\n");
	        //printk("[IIS] 0x01c22400+0x24 = %#x, line= %d\n", readl(0xf1c22400+0x24), __LINE__);
			sw_dma_ctrl(capture_prtd->params->channel, SW_DMAOP_STOP);
			break;
		default:
			ret = -EINVAL;
			break;
		}
		spin_unlock(&capture_prtd->lock);
	}
	return ret;
}

static int sun4i_pcm_open(struct snd_pcm_substream *substream)
{	
	struct sun4i_playback_runtime_data *play_prtd = NULL;
	struct sun4i_capture_runtime_data *capture_prtd = NULL;
	struct snd_pcm_runtime *play_runtime = NULL;
	struct snd_pcm_runtime *capture_runtime = NULL;
	
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		play_runtime = substream->runtime;

		snd_pcm_hw_constraint_integer(play_runtime, SNDRV_PCM_HW_PARAM_PERIODS);
		snd_soc_set_runtime_hwparams(substream, &sun4i_pcm_play_hardware);
		play_prtd = kzalloc(sizeof(struct sun4i_playback_runtime_data), GFP_KERNEL);
	
		if (play_prtd == NULL) {
			return -ENOMEM;
		}
	
		spin_lock_init(&play_prtd->lock);
	
		play_runtime->private_data = play_prtd;
	} else {
		capture_runtime = substream->runtime;
		
		snd_pcm_hw_constraint_integer(capture_runtime, SNDRV_PCM_HW_PARAM_PERIODS);
		snd_soc_set_runtime_hwparams(substream, &sun4i_pcm_capture_hardware);
		
		capture_prtd = kzalloc(sizeof(struct sun4i_capture_runtime_data), GFP_KERNEL);
		if (capture_prtd == NULL)
			return -ENOMEM;
			
		spin_lock_init(&capture_prtd->lock);
		
		capture_runtime->private_data = capture_prtd;
	}

	return 0;
}

static int sun4i_pcm_close(struct snd_pcm_substream *substream)
{
	struct sun4i_playback_runtime_data *play_prtd = NULL;
	struct sun4i_capture_runtime_data *capture_prtd = NULL;
	struct snd_pcm_runtime *play_runtime = NULL;
	struct snd_pcm_runtime *capture_runtime = NULL;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		play_runtime = substream->runtime;
		play_prtd = play_runtime->private_data;
		
		kfree(play_prtd);		
	} else {
		capture_runtime = substream->runtime;
		capture_prtd = capture_runtime->private_data;
		
		kfree(capture_prtd);
	}
	
	return 0;
}

static int sun4i_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *play_runtime = NULL;
	struct snd_pcm_runtime *capture_runtime = NULL;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		play_runtime = substream->runtime;
		
		return dma_mmap_writecombine(substream->pcm->card->dev, vma,
					     play_runtime->dma_area,
					     play_runtime->dma_addr,
					     play_runtime->dma_bytes);
	} else {
		capture_runtime = substream->runtime;
		
		return dma_mmap_writecombine(substream->pcm->card->dev, vma,
					     play_runtime->dma_area,
					     play_runtime->dma_addr,
					     play_runtime->dma_bytes);
	}

}

static struct snd_pcm_ops sun4i_pcm_ops = {
	.open			= sun4i_pcm_open,
	.close			= sun4i_pcm_close,
	.ioctl			= snd_pcm_lib_ioctl,
	.hw_params		= sun4i_pcm_hw_params,
	.hw_free		= sun4i_pcm_hw_free,
	.prepare		= sun4i_pcm_prepare,
	.trigger		= sun4i_pcm_trigger,
	.pointer		= sun4i_pcm_pointer,
	.mmap			= sun4i_pcm_mmap,
};

static int sun4i_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = 0;
	
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		size = sun4i_pcm_play_hardware.buffer_bytes_max;
	} else {
		size = sun4i_pcm_capture_hardware.buffer_bytes_max;
	}
	
	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_writecombine(pcm->card->dev, size,
					   &buf->addr, GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;
	return 0;
}

static void sun4i_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;
	
	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_writecombine(pcm->card->dev, buf->bytes,
				      buf->area, buf->addr);
		buf->area = NULL;
	}
}

static u64 sun4i_pcm_mask = DMA_BIT_MASK(32);

static int sun4i_pcm_new(struct snd_card *card,
			   struct snd_soc_dai *dai, struct snd_pcm *pcm)
{
	int ret = 0;	
	if (!card->dev->dma_mask)
		card->dev->dma_mask = &sun4i_pcm_mask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = 0xffffffff;

	if (dai->driver->playback.channels_min) {
		ret = sun4i_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (dai->driver->capture.channels_min) {
		ret = sun4i_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}
 out:
	return ret;
}

static struct snd_soc_platform_driver sun4i_soc_platform = {
	.ops		= &sun4i_pcm_ops,
	.pcm_new	= sun4i_pcm_new,
	.pcm_free	= sun4i_pcm_free_dma_buffers,
};

static int __devinit sun4i_i2s_pcm_probe(struct platform_device *pdev)
{
	return snd_soc_register_platform(&pdev->dev, &sun4i_soc_platform);
}

static int __devexit sun4i_i2s_pcm_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

/*data relating*/
static struct platform_device sun4i_i2s_pcm_device = {
	.name = "sun4i-i2s-pcm-audio",
};

/*method relating*/
static struct platform_driver sun4i_i2s_pcm_driver = {
	.probe = sun4i_i2s_pcm_probe,
	.remove = __devexit_p(sun4i_i2s_pcm_remove),
	.driver = {
		.name = "sun4i-i2s-pcm-audio",
		.owner = THIS_MODULE,
	},
};

static int __init sun4i_soc_platform_i2s_init(void)
{
	int err = 0;	
	if((err = platform_device_register(&sun4i_i2s_pcm_device)) < 0)
		return err;

	if ((err = platform_driver_register(&sun4i_i2s_pcm_driver)) < 0)
		return err;
	return 0;	
}
module_init(sun4i_soc_platform_i2s_init);

static void __exit sun4i_soc_platform_i2s_exit(void)
{
	return platform_driver_unregister(&sun4i_i2s_pcm_driver);
}
module_exit(sun4i_soc_platform_i2s_exit);

MODULE_AUTHOR("All winner");
MODULE_DESCRIPTION("SUN4I PCM DMA module");
MODULE_LICENSE("GPL");

