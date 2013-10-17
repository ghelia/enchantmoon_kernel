/*
*************************************************************************************
*                         			      Linux
*					                 USB Host Driver
*
*				        (c) Copyright 2006-2012, All winners Co,Ld.
*							       All Rights Reserved
*
* File Name 	: sw_usb_mu509.h
*
* Author 		: javen
*
* Description 	: USB 3G operations
*
* History 		:
*      <author>    		<time>       	<version >    		<desc>
*       javen     	  2012-4-10            1.0          create this file
*
*************************************************************************************
*/

#ifndef  __SW_USB_MU509_H__
#define  __SW_USB_MU509_H__

enum sw_usbc_type{
	SW_USB_UNKOWN = 0,
	SW_USB_EHCI,
	SW_USB_OHCI,
};

void mu509_vbat(u32 usbc_no, u32 on);
void mu509_wakeup_sleep(u32 usbc_no, u32 sleep);
void mu509_power(u32 usbc_no, u32 on);
void mu509_reset(u32 usbc_no);
u32 is_suspport_mu509(u32 usbc_no, u32 usbc_type);

int mu509_init(void);
int mu509_exit(void);


#endif   //__SW_USB_MU509_H__

