/*
 * (C) Copyright 2007-2012
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Tom Cubie <tangliang@allwinnertech.com>
 *
 * SUN4I gps module definition
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */

unsigned int gps_pio_hdle = 0;

static int gps_init(void)
{
	char* pio_para = "gps_para";

	gps_pio_hdle = gpio_request_ex(pio_para, NULL);
	if (!gps_pio_hdle) {
    	pr_err("gps request pio parameter failed\n");
	}   
    
    /* Enable RF module */
    gpio_write_one_pin_value(gps_pio_hdle, 1, "gps_vcc_en");
    	
	ahb_gps = clk_get(NULL, "ahb_com");
	if(!ahb_gps) {
		pr_err("try to get gps hclk failed!\n");
	}

	a_gps = clk_get(NULL, "com");
	if(!a_gps) {
		pr_err("try to get gps reset failed!\n");
	}

	if(clk_enable(ahb_gps)) {
		pr_err("try to enable gps hclk  failed!\n");
	}

	clk_reset(a_gps,1);
	
	mdelay(10);
	
	clk_reset(a_gps, 0);
		
	pr_info("Exit gps_soc_init\n");	

	return 0;
}


static int gps_deinit(void)
{
	if( gps_pio_hdle ) {
		gpio_write_one_pin_value(gps_pio_hdle, 0, "gps_osc_en");
		gpio_write_one_pin_value(gps_pio_hdle, 0, "gps_rx_en");
		gpio_write_one_pin_value(gps_pio_hdle, 0, "gps_vcc_en");
		gpio_release(gps_pio_hdle, NULL );
	}
	
	if( a_gps ) {
		clk_disable(a_gps);
	}

	if( a_gps) {
		clk_put(a_gps);
	}

	if( ahb ) {
		clk_put(ahb);
	}

	return 0;
}
