/******************************************************************************
 *
 * Copyright(c) 2007 - 2011 Realtek Corporation. All rights reserved.
 *                                        
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 *
 ******************************************************************************/
#define CONFIG_PHY_SETTING_WITH_ODM
#define CONFIG_CHIP_VER_INTEGRATION

/*
 * Public  General Config
 */

#define AUTOCONF_INCLUDED

#define RTL871X_MODULE_NAME "8723AS-VAU"
#define DRV_NAME "rtl8723as-vau"

#define CONFIG_RTL8723A 1
#define CONFIG_USB_HCI	1
#define PLATFORM_LINUX	1

#define CONFIG_EMBEDDED_FWIMG	1

#define CONFIG_DEBUG 1


/*
 * Functions Config
 */
#define CONFIG_80211N_HT	1
#define CONFIG_RECV_REORDERING_CTRL	1


#define SUPPORT_HW_RFOFF_DETECTED	1

#define CONFIG_AP_MODE	1
#define CONFIG_NATIVEAP_MLME	1


//#define CONFIG_P2P	1

//#define CONFIG_IOCTL_CFG80211 1	// enable this will disable wext ioctl support


//#define CONFIG_TDLS	1

//#define CONFIG_CONCURRENT_MODE 1

#define CONFIG_SKB_COPY	1//for amsdu

//#define CONFIG_LED

#define USB_INTERFERENCE_ISSUE // this should be checked in all usb interface
//#define CONFIG_ADAPTOR_INFO_CACHING_FILE // now just applied on 8192cu only, should make it general...
//#define CONFIG_RESUME_IN_WORKQUEUE
//#define CONFIG_SET_SCAN_DENY_TIMER
#define CONFIG_LONG_DELAY_ISSUE
#define CONFIG_NEW_SIGNAL_STAT_PROCESS


/*
 * Auto Config Section
 */

#ifdef CONFIG_MP_INCLUDED
	#define MP_DRIVER		1
	#define CONFIG_MP_IWPRIV_SUPPORT 1
	// disable unnecessary functions for MP
	#undef CONFIG_POWER_SAVING
	#undef CONFIG_BT_COEXIST
	#undef CONFIG_ANTENNA_DIVERSITY
	#undef SUPPORT_HW_RFOFF_DETECTED
#else // #ifdef CONFIG_MP_INCLUDED
	#define MP_DRIVER		0
	#undef CONFIG_MP_IWPRIV_SUPPORT 
	#define CONFIG_LPS		1
#endif // #ifdef CONFIG_MP_INCLUDED

#ifdef CONFIG_ANTENNA_DIVERSITY
#define CONFIG_SW_ANTENNA_DIVERSITY	 
//#define CONFIG_HW_ANTENNA_DIVERSITY	
#endif

	
#ifdef CONFIG_AP_MODE
	#ifndef CONFIG_NATIVEAP_MLME
		#define CONFIG_HOSTAPD_MLME	1
	#endif			
	#define CONFIG_FIND_BEST_CHANNEL	1
//#define CONFIG_NO_WIRELESS_HANDLERS	1
#endif


#ifdef CONFIG_LED
	#define CONFIG_SW_LED
	#ifdef CONFIG_SW_LED
		//#define CONFIG_LED_HANDLED_BY_CMD_THREAD
	#endif
#endif // CONFIG_LED

#ifdef CONFIG_POWER_SAVING

#define CONFIG_IPS		1
#define CONFIG_LPS		1
#define CONFIG_LPS_LCLK		1
#endif // #ifdef CONFIG_POWER_SAVING



#ifdef CONFIG_IOCTL_CFG80211

#define CONFIG_AP_MODE 1
#define CONFIG_NATIVEAP_MLME 1
#ifdef CONFIG_HOSTAPD_MLME
	#undef CONFIG_HOSTAPD_MLME
#endif

#define CONFIG_P2P 1

#endif // CONFIG_IOCTL_CFG80211



//#define CONFIG_BR_EXT	1	// Enable NAT2.5 support for STA mode interface with a L2 Bridge
#ifdef CONFIG_BR_EXT
#define CONFIG_BR_EXT_BRNAME	"br0"
#endif	// CONFIG_BR_EXT

//#define CONFIG_TX_MCAST2UNI	1	// Support IP multicast->unicast


#if defined(CONFIG_BT_COEXIST) || defined(CONFIG_POWER_SAVING)
#ifndef CONFIG_USB_INTERRUPT_IN_PIPE
#define CONFIG_USB_INTERRUPT_IN_PIPE 1
#endif
#endif

/*
 * Interface  Related Config
 */
//#define CONFIG_USB_INTERRUPT_IN_PIPE	1


#define CONFIG_PREALLOC_RECV_SKB	1
//#define CONFIG_REDUCE_USB_TX_INT	1	// Trade-off: Improve performance, but may cause TX URBs blocked by USB Host/Bus driver on few platforms.
//#define CONFIG_EASY_REPLACEMENT	1

/* 
 * CONFIG_USE_USB_BUFFER_ALLOC_XX uses Linux USB Buffer alloc API and is for Linux platform only now!
 */
#define CONFIG_USE_USB_BUFFER_ALLOC_TX 1	// Trade-off: For TX path, improve stability on some platforms, but may cause performance degrade on other platforms.
//#define CONFIG_USE_USB_BUFFER_ALLOC_RX 1	// For RX path

/* 
 * USB VENDOR REQ BUFFER ALLOCATION METHOD
 * if not set we'll use function local variable (stack memory)
 */
//#define CONFIG_USB_VENDOR_REQ_BUFFER_DYNAMIC_ALLOCATE
#define CONFIG_USB_VENDOR_REQ_BUFFER_PREALLOC

#define CONFIG_USB_VENDOR_REQ_MUTEX
#define CONFIG_VENDOR_REQ_RETRY

//#define CONFIG_USB_SUPPORT_ASYNC_VDN_REQ 1


/*
 * HAL  Related Config
 */

#define RTL8192C_RX_PACKET_INCLUDE_CRC	0

#define SUPPORTED_BLOCK_IO



#define RTL8192CU_FW_DOWNLOAD_ENABLE	1

//#define CONFIG_ONLY_ONE_OUT_EP_TO_LOW	0

#define CONFIG_OUT_EP_WIFI_MODE	0

#define ENABLE_USB_DROP_INCORRECT_OUT	0

#define RTL8192CU_ASIC_VERIFICATION	0	// For ASIC verification.

#define RTL8192CU_ADHOC_WORKAROUND_SETTING	1

#define DISABLE_BB_RF	0

#define RTL8191C_FPGA_NETWORKTYPE_ADHOC 0

#ifdef CONFIG_MP_INCLUDED
	#define MP_DRIVER 1
	#undef CONFIG_USB_TX_AGGREGATION
	#undef CONFIG_USB_RX_AGGREGATION
#else
	#define MP_DRIVER 0
#endif


/*
 * Platform  Related Config
 */
#ifdef CONFIG_PLATFORM_MN10300
#define CONFIG_SPECIAL_SETTING_FOR_FUNAI_TV

#if	defined (CONFIG_SW_ANTENNA_DIVERSITY)
	#undef CONFIG_SW_ANTENNA_DIVERSITY
	#define CONFIG_HW_ANTENNA_DIVERSITY
#endif

#endif

#ifdef CONFIG_WISTRON_PLATFORM

#endif

#ifdef CONFIG_PLATFORM_TI_DM365
#define CONFIG_USE_USB_BUFFER_ALLOC_RX 1
#endif
/*
 * Outsource  Related Config
 */

#define 	RTL8192CE_SUPPORT 				0
#define 	RTL8192CU_SUPPORT 			0
#define 	RTL8192C_SUPPORT 				(RTL8192CE_SUPPORT|RTL8192CU_SUPPORT)	

#define 	RTL8192DE_SUPPORT 				0
#define 	RTL8192DU_SUPPORT 			0
#define 	RTL8192D_SUPPORT 				(RTL8192DE_SUPPORT|RTL8192DU_SUPPORT)	

#define 	RTL8723AU_SUPPORT				1
#define 	RTL8723AS_SUPPORT				0
#define 	RTL8723AE_SUPPORT				0
#define 	RTL8723A_SUPPORT				(RTL8723AU_SUPPORT|RTL8723AS_SUPPORT|RTL8723AE_SUPPORT)

#define 	RTL8723_FPGA_VERIFICATION		0

#define RTL8188EE_SUPPORT				0
#define RTL8188EU_SUPPORT				0
#define RTL8188ES_SUPPORT				0
#define RTL8188E_SUPPORT				(RTL8188EE_SUPPORT|RTL8188EU_SUPPORT|RTL8188ES_SUPPORT)

//#if (RTL8188E_SUPPORT==1)
#define RATE_ADAPTIVE_SUPPORT 			0
#define POWER_TRAINING_ACTIVE			0
//#endif

/*
 * Debug  Related Config
 */
//#define CONFIG_DEBUG_RTL871X

#define DBG	1
#define CONFIG_DEBUG_RTL819X

#define CONFIG_PROC_DEBUG	1
//#define DBG_IO
//#define DBG_DELAY_OS
//#define DBG_MEM_ALLOC
//#define DBG_IOCTL

//#define DBG_TX
//#define DBG_XMIT_BUF
//#define DBG_TX_DROP_FRAME

//#define DBG_RX_DROP_FRAME
//#define DBG_RX_SEQ

//#define DBG_SHOW_MCUFWDL_BEFORE_51_ENABLE
//#define DBG_ROAMING_TEST

//#define DBG_HAL_INIT_PROFILING

//#define DBG_MEMORY_LEAK	1

//#define DBG_CONFIG_ERROR_DETECT
//#define DBG_CONFIG_ERROR_RESET

