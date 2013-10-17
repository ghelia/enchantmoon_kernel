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
#define _HAL_INIT_C_

#include <drv_types.h>
#include <rtw_byteorder.h>
#include <rtw_efuse.h>

#include <rtl8723a_hal.h>


static VOID
_FWDownloadEnable(
	IN	PADAPTER		padapter,
	IN	BOOLEAN			enable
	)
{
	u8	tmp;

	if(enable)
	{
		// 8051 enable
		tmp = rtw_read8(padapter, REG_SYS_FUNC_EN+1);
		rtw_write8(padapter, REG_SYS_FUNC_EN+1, tmp|0x04);

		// MCU firmware download enable.
		tmp = rtw_read8(padapter, REG_MCUFWDL);
		rtw_write8(padapter, REG_MCUFWDL, tmp|0x01);

		// 8051 reset
		tmp = rtw_read8(padapter, REG_MCUFWDL+2);
		rtw_write8(padapter, REG_MCUFWDL+2, tmp&0xf7);
	}
	else
	{
		// MCU firmware download disable.
		tmp = rtw_read8(padapter, REG_MCUFWDL);
		rtw_write8(padapter, REG_MCUFWDL, tmp&0xfe);

		// Reserved for fw extension.
		rtw_write8(padapter, REG_MCUFWDL+1, 0x00);
	}
}

static VOID
_BlockWrite(
	IN		PADAPTER		padapter,
	IN		PVOID		buffer,
	IN		u32			buffSize
	)
{
	u32			blockSize_p1 = 4;	// (Default) Phase #1 : PCI muse use 4-byte write to download FW
	u32			blockSize_p2 = 8;	// Phase #2 : Use 8-byte, if Phase#1 use big size to write FW.
	u32			blockSize_p3 = 1;	// Phase #3 : Use 1-byte, the remnant of FW image.
	u32			blockCount_p1 = 0, blockCount_p2 = 0, blockCount_p3 = 0;
	u32			remainSize_p1 = 0, remainSize_p2 = 0;
	u8			*bufferPtr	= (u8*)buffer;
	u32			i=0, offset=0;

#ifdef CONFIG_USB_HCI
	if(IS_HARDWARE_TYPE_8192DU(padapter))
	{
		blockSize_p1 = 64;		// Use 64-byte write to download FW
	}
	else
	{
	#ifdef PLATFORM_MACOSX
		blockSize_p1 = 196; // Use 196-byte write to download FW
	#else
		// Small block size will increase USB init speed. But prevent FW download fail
		// use 4-Byte instead of 196-Byte to write FW.
		//blockSize_p1	= 196; // Use 196-byte write to download FW
	#endif
	}
#endif

	//3 Phase #1
	blockCount_p1 = buffSize / blockSize_p1;
	remainSize_p1 = buffSize % blockSize_p1;

	if (blockCount_p1) {
		RT_TRACE(_module_hal_init_c_, _drv_notice_,
				("_BlockWrite: [P1] buffSize(%d) blockSize_p1(%d) blockCount_p1(%d) remainSize_p1(%d)\n",
				buffSize, blockSize_p1, blockCount_p1, remainSize_p1));
	}

	for (i = 0; i < blockCount_p1; i++)
	{
#ifdef CONFIG_USB_HCI
		rtw_writeN(padapter, (FW_8723A_START_ADDRESS + i * blockSize_p1), blockSize_p1, (bufferPtr + i * blockSize_p1));
#else
		rtw_write32(padapter, (FW_8723A_START_ADDRESS + i * blockSize_p1), le32_to_cpu(*((u32*)(bufferPtr + i * blockSize_p1))));
#endif
	}

	//3 Phase #2
	if (remainSize_p1)
	{
		offset = blockCount_p1 * blockSize_p1;

		blockCount_p2 = remainSize_p1/blockSize_p2;
		remainSize_p2 = remainSize_p1%blockSize_p2;

		if (blockCount_p2) {
				RT_TRACE(_module_hal_init_c_, _drv_notice_,
						("_BlockWrite: [P2] buffSize_p2(%d) blockSize_p2(%d) blockCount_p2(%d) remainSize_p2(%d)\n",
						(buffSize-offset), blockSize_p2 ,blockCount_p2, remainSize_p2));
		}

#ifdef CONFIG_USB_HCI
		for (i = 0; i < blockCount_p2; i++) {
			rtw_writeN(padapter, (FW_8723A_START_ADDRESS + offset + i*blockSize_p2), blockSize_p2, (bufferPtr + offset + i*blockSize_p2));
		}
#endif
	}

	//3 Phase #3
	if (remainSize_p2)
	{
		offset = (blockCount_p1 * blockSize_p1) + (blockCount_p2 * blockSize_p2);

		blockCount_p3 = remainSize_p2 / blockSize_p3;

		RT_TRACE(_module_hal_init_c_, _drv_notice_,
				("_BlockWrite: [P3] buffSize_p3(%d) blockSize_p3(%d) blockCount_p3(%d)\n",
				(buffSize-offset), blockSize_p3, blockCount_p3));

		for(i = 0 ; i < blockCount_p3 ; i++){
			rtw_write8(padapter, (FW_8723A_START_ADDRESS + offset + i), *(bufferPtr + offset + i));
		}
	}
}

static VOID
_PageWrite(
	IN		PADAPTER	padapter,
	IN		u32			page,
	IN		PVOID		buffer,
	IN		u32			size
	)
{
	u8 value8;
	u8 u8Page = (u8) (page & 0x07) ;

	value8 = (rtw_read8(padapter, REG_MCUFWDL+2) & 0xF8) | u8Page ;
	rtw_write8(padapter, REG_MCUFWDL+2,value8);

	_BlockWrite(padapter,buffer,size);
}

static VOID
_FillDummy(
	u8*		pFwBuf,
	u32*	pFwLen
	)
{
	u32	FwLen = *pFwLen;
	u8	remain = (u8)(FwLen%4);
	remain = (remain==0)?0:(4-remain);

	while(remain>0)
	{
		pFwBuf[FwLen] = 0;
		FwLen++;
		remain--;
	}

	*pFwLen = FwLen;
}

static VOID
_WriteFW(
	IN		PADAPTER		padapter,
	IN		PVOID			buffer,
	IN		u32			size
	)
{
	// Since we need dynamic decide method of dwonload fw, so we call this function to get chip version.
	// We can remove _ReadChipVersion from ReadpadapterInfo8192C later.
	BOOLEAN			isNormalChip;
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);

	isNormalChip = IS_NORMAL_CHIP(pHalData->VersionID);

	if (isNormalChip)
	{
		u32 	pageNums,remainSize ;
		u32 	page, offset;
		u8		*bufferPtr = (u8*)buffer;

#ifdef CONFIG_PCI_HCI
		// 20100120 Joseph: Add for 88CE normal chip.
		// Fill in zero to make firmware image to dword alignment.
		_FillDummy(bufferPtr, &size);
#endif

		pageNums = size / MAX_PAGE_SIZE ;
		//RT_ASSERT((pageNums <= 4), ("Page numbers should not greater then 4 \n"));
		remainSize = size % MAX_PAGE_SIZE;

		for (page = 0; page < pageNums; page++) {
			offset = page * MAX_PAGE_SIZE;
			_PageWrite(padapter, page, bufferPtr+offset, MAX_PAGE_SIZE);
		}
		if (remainSize) {
			offset = pageNums * MAX_PAGE_SIZE;
			page = pageNums;
			_PageWrite(padapter, page, bufferPtr+offset, remainSize);
		}
		RT_TRACE(_module_hal_init_c_, _drv_info_, ("_WriteFW Done- for Normal chip.\n"));
	}
	else {
		_BlockWrite(padapter, buffer, size);
		RT_TRACE(_module_hal_init_c_, _drv_info_, ("_WriteFW Done- for Test chip.\n"));
	}
}

static s32 _FWFreeToGo(PADAPTER padapter)
{
	u32	counter = 0;
	u32	value32;

	// polling CheckSum report
	do {
		value32 = rtw_read32(padapter, REG_MCUFWDL);
		if (value32 & FWDL_ChkSum_rpt) break;
	} while (counter++ < POLLING_READY_TIMEOUT_COUNT);

	if (counter >= POLLING_READY_TIMEOUT_COUNT) {
		RT_TRACE(_module_hal_init_c_, _drv_err_, ("%s: chksum report fail! REG_MCUFWDL:0x%08x\n", __FUNCTION__, value32));
		return _FAIL;
	}
	RT_TRACE(_module_hal_init_c_, _drv_info_, ("%s: Checksum report OK! REG_MCUFWDL:0x%08x\n", __FUNCTION__, value32));

	value32 = rtw_read32(padapter, REG_MCUFWDL);
	value32 |= MCUFWDL_RDY;
	value32 &= ~WINTINI_RDY;
	rtw_write32(padapter, REG_MCUFWDL, value32);

	// polling for FW ready
	counter = 0;
	do {
		value32 = rtw_read32(padapter, REG_MCUFWDL);
		if (value32 & WINTINI_RDY) {
			RT_TRACE(_module_hal_init_c_, _drv_info_, ("%s: Polling FW ready success!! REG_MCUFWDL:0x%08x\n", __FUNCTION__, value32));
			return _SUCCESS;
		}
		rtw_udelay_os(5);
	} while (counter++ < POLLING_READY_TIMEOUT_COUNT);

	RT_TRACE(_module_hal_init_c_, _drv_err_, ("%s: Polling FW ready fail!! REG_MCUFWDL:0x%08x\n", __FUNCTION__, value32));
	return _FAIL;
}

#define IS_FW_81xxC(padapter)	(((GET_HAL_DATA(padapter))->FirmwareSignature & 0xFFF0) == 0x88C0)

void rtl8723a_FirmwareSelfReset(PADAPTER padapter)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	u8	u1bTmp;
	u8	Delay = 100;


	if (!(IS_FW_81xxC(padapter) &&
		  ((pHalData->FirmwareVersion < 0x21) ||
		   (pHalData->FirmwareVersion == 0x21 &&
		    pHalData->FirmwareSubVersion < 0x01)))) // after 88C Fw v33.1
	{
		//0x1cf=0x20. Inform 8051 to reset. 2009.12.25. tynli_test
		rtw_write8(padapter, REG_HMETFR+3, 0x20);

		u1bTmp = rtw_read8(padapter, REG_SYS_FUNC_EN+1);
		while (u1bTmp & BIT2)
		{
			Delay--;
			if(Delay == 0)
				break;
			rtw_udelay_os(50);
			u1bTmp = rtw_read8(padapter, REG_SYS_FUNC_EN+1);
		}
		RT_TRACE(_module_hal_init_c_, _drv_info_, ("-%s: 8051 reset success (%d)\n", __FUNCTION__, Delay));

		if ((Delay == 0) && IS_HARDWARE_TYPE_8723AU(padapter))
		{
			//force firmware reset
			u1bTmp = rtw_read8(padapter, REG_SYS_FUNC_EN+1);
			rtw_write8(padapter, REG_SYS_FUNC_EN+1, u1bTmp&(~BIT2));
		}
	}
}

//
//	Description:
//		Download 8192C firmware code.
//
//
s32 rtl8723a_FirmwareDownload(PADAPTER padapter)
{
	s32	rtStatus = _SUCCESS;
	PHAL_DATA_TYPE	pHalData = GET_HAL_DATA(padapter);
	s8 			R8723FwImageFileName_UMC[] ={RTL8723_FW_UMC_IMG};
	s8 			R8723FwImageFileName_UMC_B[] ={RTL8723_FW_UMC_B_IMG};
	u8			*FwImage;
	u32			FwImageLen;
	u8			*pFwImageFileName;
	u8			*pucMappedFile = NULL;
	PRT_FIRMWARE_8723A	pFirmware = NULL;
	PRT_8723A_FIRMWARE_HDR		pFwHdr = NULL;
	u8			*pFirmwareBuf;
	u32			FirmwareLen;


	RT_TRACE(_module_hal_init_c_, _drv_info_, ("+%s\n", __FUNCTION__));
	pFirmware = (PRT_FIRMWARE_8723A)rtw_zmalloc(sizeof(RT_FIRMWARE_8723A));
	if(!pFirmware)
	{
		rtStatus = _FAIL;
		goto Exit;
	}

	if (IS_HARDWARE_TYPE_8723A(padapter))
	{
		if (IS_8723A_A_CUT(pHalData->VersionID))
		{
			pFwImageFileName = R8723FwImageFileName_UMC;
			FwImage = (u8*)Rtl8723_FwImageArray;
			FwImageLen = Rtl8723_ImgArrayLength;
			RT_TRACE(_module_hal_init_c_, _drv_info_, ("rtl8723a_FirmwareDownload: R8723FwImageArray_UMC for RTL8723A A CUT\n"));
		}
		else if (IS_8723A_B_CUT(pHalData->VersionID))
		{
			// WLAN Fw.
			pFwImageFileName = R8723FwImageFileName_UMC_B;
			FwImage = (u8*)Rtl8723_FwUMCBCutImageArray;
			FwImageLen = Rtl8723_UMCBCutImgArrayLength;
			RT_TRACE(_module_hal_init_c_, _drv_info_, ("rtl8723a_FirmwareDownload: R8723FwImageArray_UMC_B for RTL8723A B CUT\n"));
		}
		else
		{
			// <Roger_TODO> We should download proper RAM Code here  to match the ROM code.
			RT_TRACE(_module_hal_init_c_, _drv_err_, ("%s: unknow version!\n", __FUNCTION__));
//			return RT_STATUS_FAILURE;
			rtStatus = _FAIL;
			goto Exit;
		}
	}
	else
	{
		RT_TRACE(_module_hal_init_c_, _drv_err_, ("%s: unknow chip!\n", __FUNCTION__));
		rtStatus = _FAIL;
		goto Exit;
	}

//	RT_TRACE(_module_hal_init_c_, _drv_err_, ("rtl8723a_FirmwareDownload: %s\n", pFwImageFileName));

#ifdef CONFIG_EMBEDDED_FWIMG
	pFirmware->eFWSource = FW_SOURCE_HEADER_FILE;
#else
	pFirmware->eFWSource = FW_SOURCE_IMG_FILE; // We should decided by Reg.
#endif

	switch(pFirmware->eFWSource)
	{
		case FW_SOURCE_IMG_FILE:
			//TODO:
			break;
		case FW_SOURCE_HEADER_FILE:
			if (FwImageLen > FW_8723A_SIZE) {
				rtStatus = _FAIL;
				RT_TRACE(_module_hal_init_c_, _drv_err_, ("Firmware size exceed 0x%X. Check it.\n", FW_8723A_SIZE) );
				goto Exit;
			}

			pFirmware->szFwBuffer = FwImage;
			pFirmware->ulFwLength = FwImageLen;
			break;
	}

	pFirmwareBuf = pFirmware->szFwBuffer;
	FirmwareLen = pFirmware->ulFwLength;

	// To Check Fw header. Added by tynli. 2009.12.04.
	pFwHdr = (PRT_8723A_FIRMWARE_HDR)pFirmware->szFwBuffer;

	pHalData->FirmwareVersion =  le16_to_cpu(pFwHdr->Version);
	pHalData->FirmwareSubVersion = pFwHdr->Subversion;
	pHalData->FirmwareSignature = le16_to_cpu(pFwHdr->Signature);

	RT_TRACE(_module_hal_init_c_, _drv_info_,
		 ("%s: fw_ver=%d fw_subver=%d sig=0x%x\n",
		  __FUNCTION__, pHalData->FirmwareVersion, pHalData->FirmwareSubVersion, pHalData->FirmwareSignature));

	if (IS_FW_HEADER_EXIST(pFwHdr))
	{
		// Shift 32 bytes for FW header
		pFirmwareBuf = pFirmwareBuf + 32;
		FirmwareLen = FirmwareLen - 32;
	}

	// Suggested by Filen. If 8051 is running in RAM code, driver should inform Fw to reset by itself,
	// or it will cause download Fw fail. 2010.02.01. by tynli.
	if (rtw_read8(padapter, REG_MCUFWDL) & RAM_DL_SEL) //8051 RAM code
	{
		rtl8723a_FirmwareSelfReset(padapter);
		rtw_write8(padapter, REG_MCUFWDL, 0x00);
	}

	_FWDownloadEnable(padapter, _TRUE);
	_WriteFW(padapter, pFirmwareBuf, FirmwareLen);
	_FWDownloadEnable(padapter, _FALSE);

	rtStatus = _FWFreeToGo(padapter);
	if (_SUCCESS != rtStatus) {
		RT_TRACE(_module_hal_init_c_, _drv_err_, ("DL Firmware failed!\n"));
		goto Exit;
	}
	RT_TRACE(_module_hal_init_c_, _drv_info_, ("Firmware is ready to run!\n"));

Exit:

	if (pFirmware)
		rtw_mfree((u8*)pFirmware, sizeof(RT_FIRMWARE_8723A));

	//RT_TRACE(COMP_INIT, DBG_LOUD, (" <=== FirmwareDownload91C()\n"));
	return rtStatus;
}

void rtl8723a_InitializeFirmwareVars(PADAPTER padapter)
{
	PHAL_DATA_TYPE pHalData = GET_HAL_DATA(padapter);

	// Init Fw LPS related.
	padapter->pwrctrlpriv.bFwCurrentInPSMode = _FALSE;

	// Init H2C counter. by tynli. 2009.12.09.
	pHalData->LastHMEBoxNum = 0;
//	pHalData->H2CQueueHead = 0;
//	pHalData->H2CQueueTail = 0;
//	pHalData->H2CStopInsertQueue = _FALSE;
}

void rtl8192c_HalSetBrateCfg(
	IN PADAPTER		Adapter,
	IN u8			*mBratesOS,
	OUT u16			*pBrateCfg)
{
	u8	is_brate;
	u8	i;
	u8	brate;

	for(i=0;i<NDIS_802_11_LENGTH_RATES_EX;i++)
	{
		is_brate = mBratesOS[i] & IEEE80211_BASIC_RATE_MASK;
		brate = mBratesOS[i] & 0x7f;
		if( is_brate )
		{
			switch(brate)
			{
				case IEEE80211_CCK_RATE_1MB:	*pBrateCfg |= RATE_1M;	break;
				case IEEE80211_CCK_RATE_2MB:	*pBrateCfg |= RATE_2M;	break;
				case IEEE80211_CCK_RATE_5MB:	*pBrateCfg |= RATE_5_5M;break;
				case IEEE80211_CCK_RATE_11MB:	*pBrateCfg |= RATE_11M;	break;
				case IEEE80211_OFDM_RATE_6MB:	*pBrateCfg |= RATE_6M;	break;
				case IEEE80211_OFDM_RATE_9MB:	*pBrateCfg |= RATE_9M;	break;
				case IEEE80211_OFDM_RATE_12MB:	*pBrateCfg |= RATE_12M;	break;
				case IEEE80211_OFDM_RATE_18MB:	*pBrateCfg |= RATE_18M;	break;
				case IEEE80211_OFDM_RATE_24MB:	*pBrateCfg |= RATE_24M;	break;
				case IEEE80211_OFDM_RATE_36MB:	*pBrateCfg |= RATE_36M;	break;
				case IEEE80211_OFDM_RATE_48MB:	*pBrateCfg |= RATE_48M;	break;
				case IEEE80211_OFDM_RATE_54MB:	*pBrateCfg |= RATE_54M;	break;
			}
		}

	}
}

static void rtl8723a_free_hal_data(PADAPTER padapter)
{
_func_enter_;
	if (padapter->HalData) {
		rtw_mfree(padapter->HalData, sizeof(HAL_DATA_TYPE));
		padapter->HalData = NULL;
	}
_func_exit_;
}

//===========================================================
//				Efuse related code
//===========================================================
enum{
		VOLTAGE_V25						= 0x03,
		LDOE25_SHIFT						= 28 ,
	};

static BOOLEAN
hal_EfusePgPacketWrite2ByteHeader(
	IN	PADAPTER		pAdapter,
	IN	u8				efuseType,
	IN	u16				*pAddr,
	IN	PPGPKT_STRUCT	pTargetPkt,
	IN	BOOLEAN			bPseudoTest);
static BOOLEAN
hal_EfusePgPacketWrite1ByteHeader(
	IN	PADAPTER		pAdapter,
	IN	u8				efuseType,
	IN	u16				*pAddr,
	IN	PPGPKT_STRUCT	pTargetPkt,
	IN	BOOLEAN			bPseudoTest);
static BOOLEAN
hal_EfusePgPacketWriteData(
	IN	PADAPTER		pAdapter,
	IN	u8				efuseType,
	IN	u16				*pAddr,
	IN	PPGPKT_STRUCT	pTargetPkt,
	IN	BOOLEAN			bPseudoTest);
static BOOLEAN
hal_EfusePgPacketWrite_BT(
	IN	PADAPTER		pAdapter,
	IN	u8				offset,
	IN	u8				word_en,
	IN	u8				*pData,
	IN	BOOLEAN			bPseudoTest);

static VOID
hal_EfusePowerSwitch_RTL8192C(
	IN	PADAPTER	pAdapter,
	IN	u8		bWrite,
	IN	u8		PwrState)
{
	u8	tempval;
	u16	tmpV16;

	if (PwrState == _TRUE)
	{
		// 1.2V Power: From VDDON with Power Cut(0x0000h[15]), defualt valid
		tmpV16 = rtw_read16(pAdapter,REG_SYS_ISO_CTRL);
		if( ! (tmpV16 & PWC_EV12V ) ){
			tmpV16 |= PWC_EV12V ;
			rtw_write16(pAdapter,REG_SYS_ISO_CTRL,tmpV16);
		}
		// Reset: 0x0000h[28], default valid
		tmpV16 =  rtw_read16(pAdapter,REG_SYS_FUNC_EN);
		if( !(tmpV16 & FEN_ELDR) ){
			tmpV16 |= FEN_ELDR ;
			rtw_write16(pAdapter,REG_SYS_FUNC_EN,tmpV16);
		}

		// Clock: Gated(0x0008h[5]) 8M(0x0008h[1]) clock from ANA, default valid
		tmpV16 = rtw_read16(pAdapter,REG_SYS_CLKR);
		if( (!(tmpV16 & LOADER_CLK_EN) )  ||(!(tmpV16 & ANA8M) ) ){
			tmpV16 |= (LOADER_CLK_EN |ANA8M ) ;
			rtw_write16(pAdapter,REG_SYS_CLKR,tmpV16);
		}

		if(bWrite == _TRUE)
		{
			// Enable LDO 2.5V before read/write action
			tempval = rtw_read8(pAdapter, EFUSE_TEST+3);
			tempval &= 0x0F;
			tempval |= (VOLTAGE_V25 << 4);
			rtw_write8(pAdapter, EFUSE_TEST+3, (tempval | 0x80));
		}
	}
	else
	{
		if(bWrite == _TRUE){
			// Disable LDO 2.5V after read/write action
			tempval = rtw_read8(pAdapter, EFUSE_TEST+3);
			rtw_write8(pAdapter, EFUSE_TEST+3, (tempval & 0x7F));
		}
	}
}

static VOID
hal_EfusePowerSwitch_RTL8723A(
	IN	PADAPTER	pAdapter,
	IN	u8		bWrite,
	IN	u8		PwrState)
{
	u8	tempval;
	u16	tmpV16;

	if (PwrState == _TRUE)
	{
		rtw_write8(pAdapter, REG_EFUSE_ACCESS, EFUSE_ACCESS_ON);

		// 1.2V Power: From VDDON with Power Cut(0x0000h[15]), defualt valid
		tmpV16 = rtw_read16(pAdapter,REG_SYS_ISO_CTRL);
		if( ! (tmpV16 & PWC_EV12V ) ){
			tmpV16 |= PWC_EV12V ;
			 rtw_write16(pAdapter,REG_SYS_ISO_CTRL,tmpV16);
		}
		// Reset: 0x0000h[28], default valid
		tmpV16 =  rtw_read16(pAdapter,REG_SYS_FUNC_EN);
		if( !(tmpV16 & FEN_ELDR) ){
			tmpV16 |= FEN_ELDR ;
			rtw_write16(pAdapter,REG_SYS_FUNC_EN,tmpV16);
		}

		// Clock: Gated(0x0008h[5]) 8M(0x0008h[1]) clock from ANA, default valid
		tmpV16 = rtw_read16(pAdapter,REG_SYS_CLKR);
		if( (!(tmpV16 & LOADER_CLK_EN) )  ||(!(tmpV16 & ANA8M) ) ){
			tmpV16 |= (LOADER_CLK_EN |ANA8M ) ;
			rtw_write16(pAdapter,REG_SYS_CLKR,tmpV16);
		}

		if(bWrite == _TRUE)
		{
			// Enable LDO 2.5V before read/write action
			tempval = rtw_read8(pAdapter, EFUSE_TEST+3);
			tempval &= 0x0F;
			tempval |= (VOLTAGE_V25 << 4);
			rtw_write8(pAdapter, EFUSE_TEST+3, (tempval | 0x80));
		}
	}
	else
	{
		rtw_write8(pAdapter, REG_EFUSE_ACCESS, EFUSE_ACCESS_OFF);

		if(bWrite == _TRUE){
			// Disable LDO 2.5V after read/write action
			tempval = rtw_read8(pAdapter, EFUSE_TEST+3);
			rtw_write8(pAdapter, EFUSE_TEST+3, (tempval & 0x7F));
		}
	}
}

static VOID
rtl8192c_EfusePowerSwitch(
	IN	PADAPTER	pAdapter,
	IN	u8		bWrite,
	IN	u8		PwrState)
{
	if(IS_HARDWARE_TYPE_8192C(pAdapter))
	{
		hal_EfusePowerSwitch_RTL8192C(pAdapter, bWrite, PwrState);
	}
	else if(IS_HARDWARE_TYPE_8723A(pAdapter))
	{
		hal_EfusePowerSwitch_RTL8723A(pAdapter, bWrite, PwrState);
	}
}

static VOID
ReadEFuse_RTL8192C(
	PADAPTER	Adapter,
	u16		 _offset,
	u16 		_size_byte,
	u8      	*pbuf,
	IN	BOOLEAN		bPseudoTest
	)
{
	u8	efuseTbl[EFUSE_MAP_LEN];
	u8	rtemp8[1];
	u16 	eFuse_Addr = 0;
	u8  	offset, wren;
	u16	i, j;
	u16	eFuseWord[EFUSE_MAX_SECTION][EFUSE_MAX_WORD_UNIT];
	u16	efuse_utilized = 0;
	u8	efuse_usage = 0;

	//
	// Do NOT excess total size of EFuse table. Added by Roger, 2008.11.10.
	//
	if((_offset + _size_byte)>EFUSE_MAP_LEN)
	{// total E-Fuse table is 128bytes
		//DBG_8192C("ReadEFuse_RTL8192C(): Invalid offset(%#x) with read bytes(%#x)!!\n",_offset, _size_byte);
		return;
	}

	// 0. Refresh efuse init map as all oxFF.
	for (i = 0; i < EFUSE_MAX_SECTION; i++)
		for (j = 0; j < EFUSE_MAX_WORD_UNIT; j++)
			eFuseWord[i][j] = 0xFFFF;


	//
	// 1. Read the first byte to check if efuse is empty!!!
	//
	//
	ReadEFuseByte(Adapter, eFuse_Addr, rtemp8, bPseudoTest);
	if(*rtemp8 != 0xFF)
	{
		efuse_utilized++;
		//RTPRINT(FEEPROM, EFUSE_READ_ALL, ("Addr=%d\n", eFuse_Addr));
		eFuse_Addr++;
	}

	//
	// 2. Read real efuse content. Filter PG header and every section data.
	//
	while((*rtemp8 != 0xFF) && (eFuse_Addr < EFUSE_REAL_CONTENT_LEN))
	{
		// Check PG header for section num.
		offset = ((*rtemp8 >> 4) & 0x0f);

		if(offset < EFUSE_MAX_SECTION)
		{
			// Get word enable value from PG header
			wren = (*rtemp8 & 0x0f);
			//RTPRINT(FEEPROM, EFUSE_READ_ALL, ("Offset-%d Worden=%x\n", offset, wren));

			for(i=0; i<EFUSE_MAX_WORD_UNIT; i++)
			{
				// Check word enable condition in the section
				if(!(wren & 0x01))
				{
					//RTPRINT(FEEPROM, EFUSE_READ_ALL, ("Addr=%d\n", eFuse_Addr));
					ReadEFuseByte(Adapter, eFuse_Addr, rtemp8, bPseudoTest);	eFuse_Addr++;
					efuse_utilized++;
					eFuseWord[offset][i] = (*rtemp8 & 0xff);


					if(eFuse_Addr >= EFUSE_REAL_CONTENT_LEN)
						break;

					//RTPRINT(FEEPROM, EFUSE_READ_ALL, ("Addr=%d\n", eFuse_Addr));
					ReadEFuseByte(Adapter, eFuse_Addr, rtemp8, bPseudoTest);	eFuse_Addr++;
					efuse_utilized++;
					eFuseWord[offset][i] |= (((u16)*rtemp8 << 8) & 0xff00);

					if(eFuse_Addr >= EFUSE_REAL_CONTENT_LEN)
						break;
				}

				wren >>= 1;

			}
		}

		//RTPRINT(FEEPROM, EFUSE_READ_ALL, ("Addr=%d\n", eFuse_Addr));
		// Read next PG header
		ReadEFuseByte(Adapter, eFuse_Addr, rtemp8, bPseudoTest);
		if(*rtemp8 != 0xFF && (eFuse_Addr < 512))
		{
			efuse_utilized++;
			eFuse_Addr++;
		}
	}

	//
	// 3. Collect 16 sections and 4 word unit into Efuse map.
	//
	for(i=0; i<EFUSE_MAX_SECTION; i++)
	{
		for(j=0; j<EFUSE_MAX_WORD_UNIT; j++)
		{
			efuseTbl[(i*8)+(j*2)]=(eFuseWord[i][j] & 0xff);
			efuseTbl[(i*8)+((j*2)+1)]=((eFuseWord[i][j] >> 8) & 0xff);
		}
	}

	//
	// 4. Copy from Efuse map to output pointer memory!!!
	//
	for(i=0; i<_size_byte; i++)
	{
		pbuf[i] = efuseTbl[_offset+i];
	}

	//
	// 5. Calculate Efuse utilization.
	//
	efuse_usage = (u8)((efuse_utilized*100)/EFUSE_REAL_CONTENT_LEN);
	Adapter->HalFunc.SetHwRegHandler(Adapter, HW_VAR_EFUSE_BYTES, (u8 *)&efuse_utilized);
	//Adapter->HalFunc.SetHwRegHandler(Adapter, HW_VAR_EFUSE_USAGE, (u8*)&efuse_usage);
}

static VOID
ReadEFuse_RTL8723A(
	PADAPTER	Adapter,
	u16		 _offset,
	u16 		_size_byte,
	u8      	*pbuf,
	IN	BOOLEAN		bPseudoTest
	)
{
	u8	efuseTbl[EFUSE_MAP_LEN_8723A];
	u16	eFuse_Addr = 0;
	u8	offset, wden;
	u16	i, j;
	u16	eFuseWord[EFUSE_MAX_SECTION_8723A][EFUSE_MAX_WORD_UNIT];
	u16	efuse_utilized = 0;
	u8	efuse_usage = 0;
	u8	offset_2_0 = 0;
	u8	efuseHeader = 0, efuseExtHdr = 0, efuseData = 0;
	//
	// Do NOT excess total size of EFuse table. Added by Roger, 2008.11.10.
	//
	if((_offset + _size_byte)>EFUSE_MAP_LEN_8723A)
	{
		//RT_TRACE(COMP_EFUSE, DBG_LOUD, ("ReadEFuse_RTL8723A(): Invalid offset(%#x) with read bytes(%#x)!!\n",_offset, _size_byte));
		return;
	}

	// 0. Refresh efuse init map as all oxFF.
	for (i = 0; i < EFUSE_MAX_SECTION_8723A; i++)
		for (j = 0; j < EFUSE_MAX_WORD_UNIT; j++)
			eFuseWord[i][j] = 0xFFFF;

	//
	// 1. Read the first byte to check if efuse is empty!!!
	//
	//
	ReadEFuseByte(Adapter, eFuse_Addr++, &efuseHeader, bPseudoTest);

	if(efuseHeader != 0xFF)
	{
		efuse_utilized++;
	}
	else
	{
		//RTPRINT(FEEPROM, EFUSE_READ_ALL, ("EFUSE is empty\n"));
		return;
	}


	//
	// 2. Read real efuse content. Filter PG header and every section data.
	//
	while((efuseHeader != 0xFF) && AVAILABLE_EFUSE_ADDR(eFuse_Addr))
	{
		//RTPRINT(FEEPROM, EFUSE_READ_ALL, ("efuse[%d]=%x\n", eFuse_Addr-1, efuseHeader));

		// Check PG header for section num.
		if(EXT_HEADER(efuseHeader))		//extended header
		{
			offset_2_0 = GET_HDR_OFFSET_2_0(efuseHeader);
			//RTPRINT(FEEPROM, EFUSE_READ_ALL, ("extended header offset_2_0=%x\n", offset_2_0));

			ReadEFuseByte(Adapter, eFuse_Addr++, &efuseExtHdr, bPseudoTest);

			//RTPRINT(FEEPROM, EFUSE_READ_ALL, ("efuse[%d]=%x\n", eFuse_Addr-1, efuseExtHdr));

			if(efuseExtHdr != 0xff)
			{
				efuse_utilized++;
				if(ALL_WORDS_DISABLED(efuseExtHdr))
				{
					ReadEFuseByte(Adapter, eFuse_Addr++, &efuseHeader, bPseudoTest);
					if(efuseHeader != 0xff)
					{
						efuse_utilized++;
					}
					continue;
				}
				else
				{
					offset = ((efuseExtHdr & 0xF0) >> 1) | offset_2_0;
					wden = (efuseExtHdr & 0x0F);
				}
			}
			else
			{
				//RTPRINT(FEEPROM, EFUSE_READ_ALL, ("Error condition, extended = 0xff\n"));
				// We should handle this condition.
			}
		}
		else
		{
			offset = ((efuseHeader >> 4) & 0x0f);
			wden = (efuseHeader & 0x0f);
		}

		if(offset < EFUSE_MAX_SECTION_8723A)
		{
			// Get word enable value from PG header
			//RTPRINT(FEEPROM, EFUSE_READ_ALL, ("Offset-%d Worden=%x\n", offset, wden));

			for(i=0; i<EFUSE_MAX_WORD_UNIT; i++)
			{
				// Check word enable condition in the section
				if(!(wden & (0x01<<i)))
				{
					ReadEFuseByte(Adapter, eFuse_Addr++, &efuseData, bPseudoTest);
					//RTPRINT(FEEPROM, EFUSE_READ_ALL, ("efuse[%d]=%x\n", eFuse_Addr-1, efuseData));
					efuse_utilized++;
					eFuseWord[offset][i] = (efuseData & 0xff);

					if(!AVAILABLE_EFUSE_ADDR(eFuse_Addr))
						break;

					ReadEFuseByte(Adapter, eFuse_Addr++, &efuseData, bPseudoTest);
					//RTPRINT(FEEPROM, EFUSE_READ_ALL, ("efuse[%d]=%x\n", eFuse_Addr-1, efuseData));
					efuse_utilized++;
					eFuseWord[offset][i] |= (((u16)efuseData << 8) & 0xff00);

					if(!AVAILABLE_EFUSE_ADDR(eFuse_Addr))
						break;
				}
			}
		}

		// Read next PG header
		ReadEFuseByte(Adapter, eFuse_Addr++, &efuseHeader, bPseudoTest);

		if(efuseHeader != 0xFF)
		{
			efuse_utilized++;
		}
	}

	//
	// 3. Collect 16 sections and 4 word unit into Efuse map.
	//
	for(i=0; i<EFUSE_MAX_SECTION_8723A; i++)
	{
		for(j=0; j<EFUSE_MAX_WORD_UNIT; j++)
		{
			efuseTbl[(i*8)+(j*2)]=(eFuseWord[i][j] & 0xff);
			efuseTbl[(i*8)+((j*2)+1)]=((eFuseWord[i][j] >> 8) & 0xff);
		}
	}

	//
	// 4. Copy from Efuse map to output pointer memory!!!
	//
	for(i=0; i<_size_byte; i++)
	{
		pbuf[i] = efuseTbl[_offset+i];
	}

	//
	// 5. Calculate Efuse utilization.
	//
	efuse_usage = (u8)((efuse_utilized*100)/EFUSE_REAL_CONTENT_LEN);
	Adapter->HalFunc.SetHwRegHandler(Adapter, HW_VAR_EFUSE_BYTES, (u8 *)&efuse_utilized);
	//Adapter->HalFunc.SetHwRegHandler(Adapter, HW_VAR_EFUSE_USAGE, (u8*)&efuse_usage);
}

static BOOLEAN
Hal_EfuseSwitchToBank(
	IN		PADAPTER	pAdapter,
	IN		u8			bank,
	IN		BOOLEAN		bPseudoTest
	)
{
	BOOLEAN		bRet = _FALSE;
	u32		value32=0;

	//RTPRINT(FEEPROM, EFUSE_PG, ("Efuse switch bank to %d\n", bank));
	if(bPseudoTest)
	{
		fakeEfuseBank = bank;
		bRet = _TRUE;
	}
	else
	{
		if(IS_HARDWARE_TYPE_8723A(pAdapter) &&
			INCLUDE_MULTI_FUNC_BT(pAdapter))
		{
			value32 = rtw_read32(pAdapter, EFUSE_TEST);
			bRet = _TRUE;
			switch(bank)
			{
			case 0:
				value32 = (value32 & ~EFUSE_SEL_MASK) | EFUSE_SEL(EFUSE_WIFI_SEL_0);
				break;
			case 1:
				value32 = (value32 & ~EFUSE_SEL_MASK) | EFUSE_SEL(EFUSE_BT_SEL_0);
				break;
			case 2:
				value32 = (value32 & ~EFUSE_SEL_MASK) | EFUSE_SEL(EFUSE_BT_SEL_1);
				break;
			case 3:
				value32 = (value32 & ~EFUSE_SEL_MASK) | EFUSE_SEL(EFUSE_BT_SEL_2);
				break;
			default:
				value32 = (value32 & ~EFUSE_SEL_MASK) | EFUSE_SEL(EFUSE_WIFI_SEL_0);
				bRet = _FALSE;
				break;
			}
			rtw_write32(pAdapter, EFUSE_TEST, value32);
		}
		else
			bRet = _TRUE;
	}
	return bRet;
}

static VOID
ReadEFuse_BT(
	PADAPTER	Adapter,
	u16		 _offset,
	u16 		_size_byte,
	u8      	*pbuf,
	IN	BOOLEAN	bPseudoTest
	)
{
	u8  	*efuseTbl;
	u16 	eFuse_Addr = 0;
	u8  	offset, wden;
	u16	i, j;
	u16 	**eFuseWord;
	u16	efuse_utilized = 0;
	u8	efuse_usage = 0;
	u8	offset_2_0=0;
	u8	efuseHeader=0, efuseExtHdr=0, efuseData=0;
	u8	bank=0;
	BOOLEAN	bCheckNextBank=_FALSE;

	efuseTbl = rtw_malloc(EFUSE_BT_MAP_LEN);
	if(efuseTbl == NULL){
		DBG_8192C("efuseTbl malloc fail !\n");
		return;
	}

	eFuseWord = (u16 **)rtw_zmalloc(sizeof(u16 *)*EFUSE_BT_MAX_SECTION);
	if(eFuseWord == NULL){
		DBG_8192C("eFuseWord malloc fail !\n");
		return;
	}
	else{
		for(i=0;i<EFUSE_BT_MAX_SECTION;i++){
			eFuseWord[i]= (u16 *)rtw_zmalloc(sizeof(u16)*EFUSE_MAX_WORD_UNIT);
			if(eFuseWord[i]==NULL){
				DBG_8192C("eFuseWord[] malloc fail !\n");
				return;
			}
		}
	}

	//
	// Do NOT excess total size of EFuse table. Added by Roger, 2008.11.10.
	//
	if((_offset + _size_byte)>EFUSE_BT_MAP_LEN)
	{
		//RT_TRACE(COMP_EFUSE, DBG_LOUD, ("ReadEFuse_BT(): Invalid offset(%#x) with read bytes(%#x)!!\n",_offset, _size_byte));
		return;
	}

	// 0. Refresh efuse init map as all oxFF.
	for (i = 0; i < EFUSE_BT_MAX_SECTION; i++)
		for (j = 0; j < EFUSE_MAX_WORD_UNIT; j++)
			eFuseWord[i][j] = 0xFFFF;

	for(bank=1; bank<EFUSE_MAX_BANK; bank++)
	{
		if(!Hal_EfuseSwitchToBank(Adapter, bank, bPseudoTest))
		{
			//RTPRINT(FEEPROM, EFUSE_READ_ALL, ("Hal_EfuseSwitchToBank() Fail!!\n"));
			return;
		}
		eFuse_Addr = 0;
		//
		// 1. Read the first byte to check if efuse is empty!!!
		//
		ReadEFuseByte(Adapter, eFuse_Addr++, &efuseHeader, bPseudoTest);

		if(efuseHeader != 0xFF)
		{
			efuse_utilized++;
		}
		else
		{
			//RTPRINT(FEEPROM, EFUSE_READ_ALL, ("EFUSE is empty\n"));
			return;
		}
		//
		// 2. Read real efuse content. Filter PG header and every section data.
		//
		while((efuseHeader != 0xFF) && AVAILABLE_EFUSE_ADDR(eFuse_Addr))
		{
			//RTPRINT(FEEPROM, EFUSE_READ_ALL, ("efuse[%d]=0x%02x (header)\n", (((bank-1)*EFUSE_REAL_CONTENT_LEN)+eFuse_Addr-1), efuseHeader));

			// Check PG header for section num.
			if(EXT_HEADER(efuseHeader))		//extended header
			{
				offset_2_0 = GET_HDR_OFFSET_2_0(efuseHeader);
				//RTPRINT(FEEPROM, EFUSE_READ_ALL, ("extended header offset_2_0=%x\n", offset_2_0));

				ReadEFuseByte(Adapter, eFuse_Addr++, &efuseExtHdr, bPseudoTest);

				//RTPRINT(FEEPROM, EFUSE_READ_ALL, ("efuse[%d]=0x%02x (ext header)\n", (((bank-1)*EFUSE_REAL_CONTENT_LEN)+eFuse_Addr-1), efuseExtHdr));

				if(efuseExtHdr != 0xff)
				{
					efuse_utilized++;
					if(ALL_WORDS_DISABLED(efuseExtHdr))
					{
						ReadEFuseByte(Adapter, eFuse_Addr++, &efuseHeader, bPseudoTest);
						if(efuseHeader != 0xff)
						{
							efuse_utilized++;
						}
						continue;
					}
					else
					{
						offset = ((efuseExtHdr & 0xF0) >> 1) | offset_2_0;
						wden = (efuseExtHdr & 0x0F);
					}
				}
				else
				{
					//RTPRINT(FEEPROM, EFUSE_READ_ALL, ("Error condition, extended = 0xff\n"));
					// We should handle this condition.
				}
			}
			else
			{
				offset = ((efuseHeader >> 4) & 0x0f);
				wden = (efuseHeader & 0x0f);
			}

			if(offset < EFUSE_BT_MAX_SECTION)
			{
				// Get word enable value from PG header
				//RTPRINT(FEEPROM, EFUSE_READ_ALL, ("Offset-%d Worden=%x\n", offset, wden));

				for(i=0; i<EFUSE_MAX_WORD_UNIT; i++)
				{
					// Check word enable condition in the section
					if(!(wden & (0x01<<i)))
					{
						ReadEFuseByte(Adapter, eFuse_Addr++, &efuseData, bPseudoTest);
						//RTPRINT(FEEPROM, EFUSE_READ_ALL, ("efuse[%d]=0x%02x\n", (((bank-1)*EFUSE_REAL_CONTENT_LEN)+eFuse_Addr-1), efuseData));
						efuse_utilized++;
						eFuseWord[offset][i] = (efuseData & 0xff);

						if(!AVAILABLE_EFUSE_ADDR(eFuse_Addr))
							break;

						ReadEFuseByte(Adapter, eFuse_Addr++, &efuseData, bPseudoTest);
						//RTPRINT(FEEPROM, EFUSE_READ_ALL, ("efuse[%d]=0x%02x\n", (((bank-1)*EFUSE_REAL_CONTENT_LEN)+eFuse_Addr-1), efuseData));
						efuse_utilized++;
						eFuseWord[offset][i] |= (((u16)efuseData << 8) & 0xff00);

						if(!AVAILABLE_EFUSE_ADDR(eFuse_Addr))
							break;
					}
				}
			}

			// Read next PG header
			ReadEFuseByte(Adapter, eFuse_Addr++, &efuseHeader, bPseudoTest);

			if(efuseHeader != 0xFF)
			{
				efuse_utilized++;
			}
			else
			{
				if((eFuse_Addr + EFUSE_PROTECT_BYTES_BANK) >= EFUSE_REAL_CONTENT_LEN)
					bCheckNextBank = _TRUE;
				else
					bCheckNextBank = _FALSE;
			}
		}
		if(!bCheckNextBank)
		{
			//RTPRINT(FEEPROM, EFUSE_READ_ALL, ("Stop to check next bank\n"));
			break;
		}
	}

	// switch bank back to bank 0 for later BT and wifi use.
	Hal_EfuseSwitchToBank(Adapter, 0, bPseudoTest);

	//
	// 3. Collect 16 sections and 4 word unit into Efuse map.
	//
	for(i=0; i<EFUSE_BT_MAX_SECTION; i++)
	{
		for(j=0; j<EFUSE_MAX_WORD_UNIT; j++)
		{
			efuseTbl[(i*8)+(j*2)]=(eFuseWord[i][j] & 0xff);
			efuseTbl[(i*8)+((j*2)+1)]=((eFuseWord[i][j] >> 8) & 0xff);
		}
	}

	//
	// 4. Copy from Efuse map to output pointer memory!!!
	//
	for(i=0; i<_size_byte; i++)
	{
		pbuf[i] = efuseTbl[_offset+i];
	}

	//
	// 5. Calculate Efuse utilization.
	//
	efuse_usage = (u8)((efuse_utilized*100)/EFUSE_BT_REAL_CONTENT_LEN);
	if(bPseudoTest)
	{
		fakeBTEfuseUsedBytes =  (EFUSE_REAL_CONTENT_LEN*(bank-1))+eFuse_Addr-1;
	}
	else
	{
		BTEfuseUsedBytes =  (EFUSE_REAL_CONTENT_LEN*(bank-1))+eFuse_Addr-1;
	}

	for(i=0;i<EFUSE_BT_MAX_SECTION;i++)
		rtw_mfree((u8 *)eFuseWord[i], sizeof(u16)*EFUSE_MAX_WORD_UNIT);
	rtw_mfree((u8 *)eFuseWord, sizeof(u16 *)*EFUSE_BT_MAX_SECTION);
	rtw_mfree(efuseTbl, EFUSE_BT_MAP_LEN);
}


static VOID
ReadEFuseByIC(
	PADAPTER	Adapter,
	u8		efuseType,
	u16		 _offset,
	u16 		_size_byte,
	u8      	*pbuf,
	IN BOOLEAN	bPseudoTest
	)
{
	if(efuseType == EFUSE_WIFI)
	{
		if(IS_HARDWARE_TYPE_8192C(Adapter))
		{
			ReadEFuse_RTL8192C(Adapter, _offset, _size_byte, pbuf, bPseudoTest);
		}
		else if(IS_HARDWARE_TYPE_8723A(Adapter))
		{
			ReadEFuse_RTL8723A(Adapter, _offset, _size_byte, pbuf, bPseudoTest);
		}
	}
	else
		ReadEFuse_BT(Adapter, _offset, _size_byte, pbuf, bPseudoTest);
}

static VOID
ReadEFuse_Pseudo(
	PADAPTER	Adapter,
	u8		efuseType,
	u16		 _offset,
	u16 		_size_byte,
	u8      	*pbuf,
	IN BOOLEAN	bPseudoTest
	)
{
	if(efuseType == EFUSE_WIFI)
		ReadEFuse_RTL8723A(Adapter, _offset, _size_byte, pbuf, bPseudoTest);
	else
		ReadEFuse_BT(Adapter, _offset, _size_byte, pbuf, bPseudoTest);
}

static VOID
rtl8192c_ReadEFuse(
	PADAPTER	Adapter,
	u8		efuseType,
	u16		_offset,
	u16 		_size_byte,
	u8      	*pbuf,
	IN	BOOLEAN	bPseudoTest
	)
{
	if(bPseudoTest)
	{
		ReadEFuse_Pseudo(Adapter, efuseType, _offset, _size_byte, pbuf, bPseudoTest);
	}
	else
	{
		ReadEFuseByIC(Adapter, efuseType, _offset, _size_byte, pbuf, bPseudoTest);
	}
}

static VOID
Hal_EFUSEGetEfuseDefinition(
	IN		PADAPTER	pAdapter,
	IN		u8		efuseType,
	IN		u8		type,
	OUT		PVOID		*pOut
	)
{
	switch(type)
	{
		case TYPE_EFUSE_MAX_SECTION:
			{
				u8	*pMax_section;
				pMax_section = (u8 *)pOut;

				if(efuseType == EFUSE_WIFI)
				{
					if(IS_HARDWARE_TYPE_8192C(pAdapter))
					{
						*pMax_section = EFUSE_MAX_SECTION;
					}
					else if(IS_HARDWARE_TYPE_8723A(pAdapter))
					{
						*pMax_section = EFUSE_MAX_SECTION_8723A;
					}
				}
				else
					*pMax_section = EFUSE_BT_MAX_SECTION;
			}
			break;
		case TYPE_EFUSE_REAL_CONTENT_LEN:
			{
				u16 *pu2Tmp;
				pu2Tmp = (u16 *)pOut;
				if(efuseType == EFUSE_WIFI)
				{
					if(IS_HARDWARE_TYPE_8192C(pAdapter))
					{
						*pu2Tmp = EFUSE_REAL_CONTENT_LEN;
					}
					else if(IS_HARDWARE_TYPE_8723A(pAdapter))
					{
						*pu2Tmp = EFUSE_REAL_CONTENT_LEN_8723A;
					}
				}
				else
					*pu2Tmp = EFUSE_BT_REAL_CONTENT_LEN;
			}
			break;
		case TYPE_AVAILABLE_EFUSE_BYTES_BANK:
			{
				u16	*pu2Tmp;
				pu2Tmp = (u16 *)pOut;
				if(efuseType == EFUSE_WIFI)
				{
					if(IS_HARDWARE_TYPE_8192C(pAdapter))
					{
						*pu2Tmp = (u16)(EFUSE_REAL_CONTENT_LEN-EFUSE_OOB_PROTECT_BYTES);
					}
					else if(IS_HARDWARE_TYPE_8723A(pAdapter))
					{
						*pu2Tmp = (u16)(EFUSE_REAL_CONTENT_LEN_8723A-EFUSE_OOB_PROTECT_BYTES);
					}
				}
				else
					*pu2Tmp = (u16)(EFUSE_REAL_CONTENT_LEN-EFUSE_PROTECT_BYTES_BANK);
			}
			break;
		case TYPE_AVAILABLE_EFUSE_BYTES_TOTAL:
			{
				u16 *pu2Tmp;
				pu2Tmp = (u16 *)pOut;
				if(efuseType == EFUSE_WIFI)
				{
					if(IS_HARDWARE_TYPE_8192C(pAdapter))
					{
						*pu2Tmp = (u16)(EFUSE_REAL_CONTENT_LEN-EFUSE_OOB_PROTECT_BYTES);
					}
					else if(IS_HARDWARE_TYPE_8723A(pAdapter))
					{
						*pu2Tmp = (u16)(EFUSE_REAL_CONTENT_LEN_8723A-EFUSE_OOB_PROTECT_BYTES);
					}
				}
				else
					*pu2Tmp = (u16)(EFUSE_BT_REAL_CONTENT_LEN-(EFUSE_PROTECT_BYTES_BANK*3));
			}
			break;
		case TYPE_EFUSE_MAP_LEN:
			{
				u16 *pu2Tmp;
				pu2Tmp = (u16 *)pOut;

				if(efuseType == EFUSE_WIFI)
				{
					if(IS_HARDWARE_TYPE_8192C(pAdapter))
					{
						*pu2Tmp = (u16)EFUSE_MAP_LEN;
					}
					else if(IS_HARDWARE_TYPE_8723A(pAdapter))
					{
						*pu2Tmp = (u16)EFUSE_MAP_LEN_8723A;
					}
				}
				else
					*pu2Tmp = (u16)EFUSE_BT_MAP_LEN;
			}
			break;
		case TYPE_EFUSE_PROTECT_BYTES_BANK:
			{
				u8 *pu1Tmp;
				pu1Tmp = (u8 *)pOut;
				if(efuseType == EFUSE_WIFI)
					*pu1Tmp = (u8)(EFUSE_OOB_PROTECT_BYTES);
				else
					*pu1Tmp = (u8)(EFUSE_PROTECT_BYTES_BANK);
			}
			break;
		default:
			{
				u8 *pu1Tmp;
				pu1Tmp = (u8 *)pOut;
				*pu1Tmp = 0;
			}
			break;
	}
}

static VOID
Hal_EFUSEGetEfuseDefinition_Pseudo(
	IN		PADAPTER	pAdapter,
	IN		u8		efuseType,
	IN		u8		type,
	OUT		PVOID		*pOut
	)
{
	switch(type)
	{
		case TYPE_EFUSE_MAX_SECTION:
			{
				u8	*pMax_section;
				pMax_section = (u8 *)pOut;
				if(efuseType == EFUSE_WIFI)
					*pMax_section = EFUSE_MAX_SECTION_8723A;
				else
					*pMax_section = EFUSE_BT_MAX_SECTION;
			}
			break;
		case TYPE_EFUSE_REAL_CONTENT_LEN:
			{
				u16 *pu2Tmp;
				pu2Tmp = (u16 *)pOut;
				if(efuseType == EFUSE_WIFI)
				{
					if(IS_HARDWARE_TYPE_8192C(pAdapter))
					{
						*pu2Tmp = EFUSE_REAL_CONTENT_LEN;
					}
					else if(IS_HARDWARE_TYPE_8723A(pAdapter))
					{
						*pu2Tmp = EFUSE_REAL_CONTENT_LEN_8723A;
					}
				}
				else
					*pu2Tmp = EFUSE_BT_REAL_CONTENT_LEN;
			}
			break;
		case TYPE_AVAILABLE_EFUSE_BYTES_BANK:
			{
				u16 *pu2Tmp;
				pu2Tmp = (u16 *)pOut;
				if(efuseType == EFUSE_WIFI)
				{
					if(IS_HARDWARE_TYPE_8192C(pAdapter))
					{
						*pu2Tmp = (u16)(EFUSE_REAL_CONTENT_LEN-EFUSE_OOB_PROTECT_BYTES);
					}
					else if(IS_HARDWARE_TYPE_8723A(pAdapter))
					{
						*pu2Tmp = (u16)(EFUSE_REAL_CONTENT_LEN_8723A-EFUSE_OOB_PROTECT_BYTES);
					}
				}
				else
					*pu2Tmp = (u16)(EFUSE_REAL_CONTENT_LEN-EFUSE_PROTECT_BYTES_BANK);
			}
			break;
		case TYPE_AVAILABLE_EFUSE_BYTES_TOTAL:
			{
				u16 *pu2Tmp;
				pu2Tmp = (u16 *)pOut;
				if(efuseType == EFUSE_WIFI)
				{
					if(IS_HARDWARE_TYPE_8192C(pAdapter))
					{
						*pu2Tmp = (u16)(EFUSE_REAL_CONTENT_LEN-EFUSE_OOB_PROTECT_BYTES);
					}
					else if(IS_HARDWARE_TYPE_8723A(pAdapter))
					{
						*pu2Tmp = (u16)(EFUSE_REAL_CONTENT_LEN_8723A-EFUSE_OOB_PROTECT_BYTES);
					}
				}
				else
					*pu2Tmp = (u16)(EFUSE_BT_REAL_CONTENT_LEN-(EFUSE_PROTECT_BYTES_BANK*3));
			}
			break;
		case TYPE_EFUSE_MAP_LEN:
			{
				u16 *pu2Tmp;
				pu2Tmp = (u16 *)pOut;
				if(efuseType == EFUSE_WIFI)
					*pu2Tmp = (u16)EFUSE_MAP_LEN_8723A;
				else
					*pu2Tmp = (u16)EFUSE_BT_MAP_LEN;
			}
			break;
		case TYPE_EFUSE_PROTECT_BYTES_BANK:
			{
				u8 *pu1Tmp;
				pu1Tmp = (u8 *)pOut;
				if(efuseType == EFUSE_WIFI)
					*pu1Tmp = (u8)(EFUSE_OOB_PROTECT_BYTES);
				else
					*pu1Tmp = (u8)(EFUSE_PROTECT_BYTES_BANK);
			}
			break;
		default:
			{
				u8 *pu1Tmp;
				pu1Tmp = (u8 *)pOut;
				*pu1Tmp = 0;
			}
			break;
	}
}

static VOID
rtl8192c_EFUSE_GetEfuseDefinition(
	IN		PADAPTER	pAdapter,
	IN		u8		efuseType,
	IN		u8		type,
	OUT		PVOID		*pOut,
	IN		BOOLEAN		bPseudoTest
	)
{
	if(bPseudoTest)
	{
		Hal_EFUSEGetEfuseDefinition_Pseudo(pAdapter, efuseType, type, pOut);
	}
	else
	{
		Hal_EFUSEGetEfuseDefinition(pAdapter, efuseType, type, pOut);
	}
}

static u8
Hal_EfuseWordEnableDataWrite(	IN	PADAPTER	pAdapter,
							IN	u16		efuse_addr,
							IN	u8		word_en,
							IN	u8		*data,
							IN	BOOLEAN		bPseudoTest)
{
	u16	tmpaddr = 0;
	u16	start_addr = efuse_addr;
	u8	badworden = 0x0F;
	u8	tmpdata[8];

	_rtw_memset((PVOID)tmpdata, 0xff, PGPKT_DATA_SIZE);
	//RT_TRACE(COMP_EFUSE, DBG_LOUD, ("word_en = %x efuse_addr=%x\n", word_en, efuse_addr));

	if(!(word_en&BIT0))
	{
		tmpaddr = start_addr;
		efuse_OneByteWrite(pAdapter,start_addr++, data[0], bPseudoTest);
		efuse_OneByteWrite(pAdapter,start_addr++, data[1], bPseudoTest);

		efuse_OneByteRead(pAdapter,tmpaddr, &tmpdata[0], bPseudoTest);
		efuse_OneByteRead(pAdapter,tmpaddr+1, &tmpdata[1], bPseudoTest);
		if((data[0]!=tmpdata[0])||(data[1]!=tmpdata[1])){
			badworden &= (~BIT0);
		}
	}
	if(!(word_en&BIT1))
	{
		tmpaddr = start_addr;
		efuse_OneByteWrite(pAdapter,start_addr++, data[2], bPseudoTest);
		efuse_OneByteWrite(pAdapter,start_addr++, data[3], bPseudoTest);

		efuse_OneByteRead(pAdapter,tmpaddr    , &tmpdata[2], bPseudoTest);
		efuse_OneByteRead(pAdapter,tmpaddr+1, &tmpdata[3], bPseudoTest);
		if((data[2]!=tmpdata[2])||(data[3]!=tmpdata[3])){
			badworden &=( ~BIT1);
		}
	}
	if(!(word_en&BIT2))
	{
		tmpaddr = start_addr;
		efuse_OneByteWrite(pAdapter,start_addr++, data[4], bPseudoTest);
		efuse_OneByteWrite(pAdapter,start_addr++, data[5], bPseudoTest);

		efuse_OneByteRead(pAdapter,tmpaddr, &tmpdata[4], bPseudoTest);
		efuse_OneByteRead(pAdapter,tmpaddr+1, &tmpdata[5], bPseudoTest);
		if((data[4]!=tmpdata[4])||(data[5]!=tmpdata[5])){
			badworden &=( ~BIT2);
		}
	}
	if(!(word_en&BIT3))
	{
		tmpaddr = start_addr;
		efuse_OneByteWrite(pAdapter,start_addr++, data[6], bPseudoTest);
		efuse_OneByteWrite(pAdapter,start_addr++, data[7], bPseudoTest);

		efuse_OneByteRead(pAdapter,tmpaddr, &tmpdata[6], bPseudoTest);
		efuse_OneByteRead(pAdapter,tmpaddr+1, &tmpdata[7], bPseudoTest);
		if((data[6]!=tmpdata[6])||(data[7]!=tmpdata[7])){
			badworden &=( ~BIT3);
		}
	}
	return badworden;
}

static u8
Hal_EfuseWordEnableDataWrite_Pseudo(	IN	PADAPTER	pAdapter,
							IN	u16		efuse_addr,
							IN	u8		word_en,
							IN	u8		*data,
							IN	BOOLEAN		bPseudoTest)
{
	u8	ret=0;

	ret = Hal_EfuseWordEnableDataWrite(pAdapter, efuse_addr, word_en, data, bPseudoTest);

	return ret;
}

static u8
rtl8192c_Efuse_WordEnableDataWrite(	IN	PADAPTER	pAdapter,
							IN	u16		efuse_addr,
							IN	u8		word_en,
							IN	u8		*data,
							IN	BOOLEAN		bPseudoTest)
{
	u8	ret=0;

	if(bPseudoTest)
	{
		ret = Hal_EfuseWordEnableDataWrite_Pseudo(pAdapter, efuse_addr, word_en, data, bPseudoTest);
	}
	else
	{
		ret = Hal_EfuseWordEnableDataWrite(pAdapter, efuse_addr, word_en, data, bPseudoTest);
	}

	return ret;
}


static u16
hal_EfuseGetCurrentSize_8192C(IN	PADAPTER	pAdapter,
	IN		BOOLEAN		bPseudoTest)
{
	int bContinual = _TRUE;

	u16	efuse_addr = 0;
	u8	hoffset=0,hworden=0;
	u8	efuse_data,word_cnts=0;

	while (	bContinual &&
			efuse_OneByteRead(pAdapter, efuse_addr ,&efuse_data, bPseudoTest) &&
			(efuse_addr  < EFUSE_REAL_CONTENT_LEN) )
	{
		if(efuse_data!=0xFF)
		{
			hoffset = (efuse_data>>4) & 0x0F;
			hworden =  efuse_data & 0x0F;
			word_cnts = Efuse_CalculateWordCnts(hworden);
			//read next header
			efuse_addr = efuse_addr + (word_cnts*2)+1;
		}
		else
		{
			bContinual = _FALSE ;
		}
	}

	return efuse_addr;
}

static u16
Hal_EfuseGetCurrentSize_BT(IN	PADAPTER	pAdapter,
		IN		BOOLEAN			bPseudoTest)
{
	int	bContinual = _TRUE;
	u16	efuse_addr = 0;
	u8	hoffset=0,hworden=0;
	u8	efuse_data,word_cnts=0;
	u8	bank=0, startBank=0;
	u16	retU2=0;
	u32	total_efuse_used=0;

	if(bPseudoTest)
	{
		efuse_addr = (u16)((fakeBTEfuseUsedBytes%EFUSE_REAL_CONTENT_LEN));
		startBank = (u8)(1+(fakeBTEfuseUsedBytes/EFUSE_REAL_CONTENT_LEN));
	}
	else
	{
		efuse_addr = (u16)((BTEfuseUsedBytes%EFUSE_REAL_CONTENT_LEN));
		startBank = (u8)(1+(BTEfuseUsedBytes/EFUSE_REAL_CONTENT_LEN));
	}

	if((startBank < 1) || (startBank >= EFUSE_MAX_BANK))
		DBG_8192C("Error, bank error, bank=%d\n", bank);

	//RTPRINT(FEEPROM, EFUSE_PG, ("Hal_EfuseGetCurrentSize_BT(), start bank=%d, start_efuse_addr = %d\n", startBank, efuse_addr));

	for(bank=startBank; bank<EFUSE_MAX_BANK; bank++)
	{
		if(!Hal_EfuseSwitchToBank(pAdapter, bank, bPseudoTest))
			break;
		else
		{
			bContinual = _TRUE;
			if(bank != startBank)	// only when bank is switched we have to reset the efuse_addr.
				efuse_addr = 0;
		}

		while (	bContinual &&
				efuse_OneByteRead(pAdapter, efuse_addr ,&efuse_data, bPseudoTest) &&
				AVAILABLE_EFUSE_ADDR(efuse_addr))
		{
			if(efuse_data!=0xFF)
			{
				if((efuse_data&0x1F) == 0x0F)		//extended header
				{
					hoffset = efuse_data;
					efuse_addr++;
					efuse_OneByteRead(pAdapter, efuse_addr ,&efuse_data, bPseudoTest);
					if((efuse_data & 0x0F) == 0x0F)
					{
						efuse_addr++;
						continue;
					}
					else
					{
						hoffset = ((hoffset & 0xE0) >> 5) | ((efuse_data & 0xF0) >> 1);
						hworden = efuse_data & 0x0F;
					}
				}
				else
				{
					hoffset = (efuse_data>>4) & 0x0F;
					hworden =  efuse_data & 0x0F;
				}
				word_cnts = Efuse_CalculateWordCnts(hworden);
				//read next header
				efuse_addr = efuse_addr + (word_cnts*2)+1;
			}
			else
			{
				bContinual = _FALSE ;
			}
		}

		// Check if we need to check next bank efuse
		if(efuse_addr < (EFUSE_REAL_CONTENT_LEN-EFUSE_PROTECT_BYTES_BANK))
		{
			break;// don't need to check next bank.
		}
	}

	retU2 = ((bank-1)*EFUSE_REAL_CONTENT_LEN)+efuse_addr;
	if(bPseudoTest)
	{
		fakeBTEfuseUsedBytes = retU2;
		//RTPRINT(FEEPROM, EFUSE_PG, ("Hal_EfuseGetCurrentSize_BT(), return %d\n", fakeBTEfuseUsedBytes));
	}
	else
	{
		BTEfuseUsedBytes = retU2;
		//RTPRINT(FEEPROM, EFUSE_PG, ("Hal_EfuseGetCurrentSize_BT(), return %d\n", BTEfuseUsedBytes));
	}

	return retU2;
}


static u16
hal_EfuseGetCurrentSize_8723A(IN	PADAPTER	pAdapter,
		IN		BOOLEAN			bPseudoTest)
{
	int	bContinual = _TRUE;

	u16	efuse_addr = 0;
	u8	hoffset=0,hworden=0;
	u8	efuse_data,word_cnts=0;

	if(bPseudoTest)
	{
		efuse_addr = (u16)(fakeEfuseUsedBytes);
	}
	else
	{
		pAdapter->HalFunc.GetHwRegHandler(pAdapter, HW_VAR_EFUSE_BYTES, (u8 *)&efuse_addr);
	}
	//RTPRINT(FEEPROM, EFUSE_PG, ("hal_EfuseGetCurrentSize_8723A(), start_efuse_addr = %d\n", efuse_addr));

	while (	bContinual &&
			efuse_OneByteRead(pAdapter, efuse_addr ,&efuse_data, bPseudoTest) &&
			AVAILABLE_EFUSE_ADDR(efuse_addr))
	{
		if(efuse_data!=0xFF)
		{
			if((efuse_data&0x1F) == 0x0F)		//extended header
			{
				hoffset = efuse_data;
				efuse_addr++;
				efuse_OneByteRead(pAdapter, efuse_addr ,&efuse_data, bPseudoTest);
				if((efuse_data & 0x0F) == 0x0F)
				{
					efuse_addr++;
					continue;
				}
				else
				{
					hoffset = ((hoffset & 0xE0) >> 5) | ((efuse_data & 0xF0) >> 1);
					hworden = efuse_data & 0x0F;
				}
			}
			else
			{
				hoffset = (efuse_data>>4) & 0x0F;
				hworden =  efuse_data & 0x0F;
			}
			word_cnts = Efuse_CalculateWordCnts(hworden);
			//read next header
			efuse_addr = efuse_addr + (word_cnts*2)+1;
		}
		else
		{
			bContinual = _FALSE ;
		}
	}

	if(bPseudoTest)
	{
		fakeEfuseUsedBytes = efuse_addr;
		//RTPRINT(FEEPROM, EFUSE_PG, ("hal_EfuseGetCurrentSize_8723A(), return %d\n", fakeEfuseUsedBytes));
	}
	else
	{
		pAdapter->HalFunc.SetHwRegHandler(pAdapter, HW_VAR_EFUSE_BYTES, (u8 *)&efuse_addr);
		//RTPRINT(FEEPROM, EFUSE_PG, ("hal_EfuseGetCurrentSize_8723A(), return %d\n", efuse_addr));
	}

	return efuse_addr;
}

static u16
Hal_EfuseGetCurrentSize_Pseudo(IN	PADAPTER	pAdapter,
		IN		BOOLEAN			bPseudoTest)
{
	u16	ret=0;

	ret = hal_EfuseGetCurrentSize_8723A(pAdapter, bPseudoTest);

	return ret;
}

static u16
rtl8192c_EfuseGetCurrentSize(
	IN	PADAPTER	pAdapter,
	IN	u8			efuseType,
	IN	BOOLEAN		bPseudoTest)
{
	u16	ret=0;

	if(efuseType == EFUSE_WIFI)
	{
		if(bPseudoTest)
		{
			ret = Hal_EfuseGetCurrentSize_Pseudo(pAdapter, bPseudoTest);
		}
		else
		{
			if(IS_HARDWARE_TYPE_8192C(pAdapter))
			{
				ret = hal_EfuseGetCurrentSize_8192C(pAdapter, bPseudoTest);
			}
			else if(IS_HARDWARE_TYPE_8723A(pAdapter))
			{
				ret = hal_EfuseGetCurrentSize_8723A(pAdapter, bPseudoTest);
			}
		}
	}
	else
	{
		ret = Hal_EfuseGetCurrentSize_BT(pAdapter, bPseudoTest);
	}

	return ret;
}

static int
hal_EfusePgPacketRead_8192C(	IN	PADAPTER	pAdapter,
					IN	u8			offset,
					IN	u8			*data,
					IN	BOOLEAN			bPseudoTest)
{
	u8	ReadState = PG_STATE_HEADER;

	int	bContinual = _TRUE;
	int	bDataEmpty = _TRUE ;

	u8	efuse_data,word_cnts=0;
	u16	efuse_addr = 0;
	u8	hoffset=0,hworden=0;
	u8	tmpidx=0;
	u8	tmpdata[8];

	if(data==NULL)	return _FALSE;
	if(offset>15)		return _FALSE;


	_rtw_memset((PVOID)data, 0xff, sizeof(u8)*PGPKT_DATA_SIZE);
	_rtw_memset((PVOID)tmpdata, 0xff, sizeof(u8)*PGPKT_DATA_SIZE);

	//
	// <Roger_TODO> Efuse has been pre-programmed dummy 5Bytes at the end of Efuse by CP.
	// Skip dummy parts to prevent unexpected data read from Efuse.
	// By pass right now. 2009.02.19.
	//
	while(bContinual && (efuse_addr  < EFUSE_REAL_CONTENT_LEN) )
	{
		//-------  Header Read -------------
		if(ReadState & PG_STATE_HEADER)
		{
			if(efuse_OneByteRead(pAdapter, efuse_addr ,&efuse_data, bPseudoTest)&&(efuse_data!=0xFF)){
				hoffset = (efuse_data>>4) & 0x0F;
				hworden =  efuse_data & 0x0F;
				word_cnts = Efuse_CalculateWordCnts(hworden);
				bDataEmpty = _TRUE ;

				if(hoffset==offset){
					for(tmpidx = 0;tmpidx< word_cnts*2 ;tmpidx++){
						if(efuse_OneByteRead(pAdapter, efuse_addr+1+tmpidx ,&efuse_data, bPseudoTest) ){
							tmpdata[tmpidx] = efuse_data;
							if(efuse_data!=0xff){
								bDataEmpty = _FALSE;
							}
						}
					}
					if(bDataEmpty==_FALSE){
						ReadState = PG_STATE_DATA;
					}else{//read next header
						efuse_addr = efuse_addr + (word_cnts*2)+1;
						ReadState = PG_STATE_HEADER;
					}
				}
				else{//read next header
					efuse_addr = efuse_addr + (word_cnts*2)+1;
					ReadState = PG_STATE_HEADER;
				}

			}
			else{
				bContinual = _FALSE ;
			}
		}
		//-------  Data section Read -------------
		else if(ReadState & PG_STATE_DATA)
		{
			efuse_WordEnableDataRead(hworden,tmpdata,data);
			efuse_addr = efuse_addr + (word_cnts*2)+1;
			ReadState = PG_STATE_HEADER;
		}

	}

	if(	(data[0]==0xff) &&(data[1]==0xff) && (data[2]==0xff)  && (data[3]==0xff) &&
		(data[4]==0xff) &&(data[5]==0xff) && (data[6]==0xff)  && (data[7]==0xff))
		return _FALSE;
	else
		return _TRUE;

}

static int
hal_EfusePgPacketRead_8723A(
	IN	PADAPTER	pAdapter,
	IN	u8			offset,
	IN	u8			*data,
	IN	BOOLEAN		bPseudoTest)
{
	u8	ReadState = PG_STATE_HEADER;

	int	bContinual = _TRUE;
	int	bDataEmpty = _TRUE ;

	u8	efuse_data,word_cnts = 0;
	u16	efuse_addr = 0;
	u8	hoffset = 0,hworden = 0;
	u8	tmpidx = 0;
	u8	tmpdata[8];
	u8	max_section = 0;
	u8	tmp_header = 0;

	EFUSE_GetEfuseDefinition(pAdapter, EFUSE_WIFI, TYPE_EFUSE_MAX_SECTION, (PVOID)&max_section, bPseudoTest);

	if(data==NULL)
		return _FALSE;
	if(offset>max_section)
		return _FALSE;

	_rtw_memset((PVOID)data, 0xff, sizeof(u8)*PGPKT_DATA_SIZE);
	_rtw_memset((PVOID)tmpdata, 0xff, sizeof(u8)*PGPKT_DATA_SIZE);


	//
	// <Roger_TODO> Efuse has been pre-programmed dummy 5Bytes at the end of Efuse by CP.
	// Skip dummy parts to prevent unexpected data read from Efuse.
	// By pass right now. 2009.02.19.
	//
	while(bContinual && AVAILABLE_EFUSE_ADDR(efuse_addr) )
	{
		//-------  Header Read -------------
		if(ReadState & PG_STATE_HEADER)
		{
			if(efuse_OneByteRead(pAdapter, efuse_addr ,&efuse_data, bPseudoTest)&&(efuse_data!=0xFF))
			{
				if(EXT_HEADER(efuse_data))
				{
					tmp_header = efuse_data;
					efuse_addr++;
					efuse_OneByteRead(pAdapter, efuse_addr ,&efuse_data, bPseudoTest);
					if(!ALL_WORDS_DISABLED(efuse_data))
					{
						hoffset = ((tmp_header & 0xE0) >> 5) | ((efuse_data & 0xF0) >> 1);
						hworden = efuse_data & 0x0F;
					}
					else
					{
						DBG_8192C("Error, All words disabled\n");
						efuse_addr++;
						continue;
					}
				}
				else
				{
					hoffset = (efuse_data>>4) & 0x0F;
					hworden =  efuse_data & 0x0F;
				}
				word_cnts = Efuse_CalculateWordCnts(hworden);
				bDataEmpty = _TRUE ;

				if(hoffset==offset)
				{
					for(tmpidx = 0;tmpidx< word_cnts*2 ;tmpidx++)
					{
						if(efuse_OneByteRead(pAdapter, efuse_addr+1+tmpidx ,&efuse_data, bPseudoTest) )
						{
							tmpdata[tmpidx] = efuse_data;
							if(efuse_data!=0xff)
							{
								bDataEmpty = _FALSE;
							}
						}
					}
					if(bDataEmpty==_FALSE){
						ReadState = PG_STATE_DATA;
					}else{//read next header
						efuse_addr = efuse_addr + (word_cnts*2)+1;
						ReadState = PG_STATE_HEADER;
					}
				}
				else{//read next header
					efuse_addr = efuse_addr + (word_cnts*2)+1;
					ReadState = PG_STATE_HEADER;
				}

			}
			else{
				bContinual = _FALSE ;
			}
		}
		//-------  Data section Read -------------
		else if(ReadState & PG_STATE_DATA)
		{
			efuse_WordEnableDataRead(hworden,tmpdata,data);
			efuse_addr = efuse_addr + (word_cnts*2)+1;
			ReadState = PG_STATE_HEADER;
		}

	}

	if(	(data[0]==0xff) &&(data[1]==0xff) && (data[2]==0xff)  && (data[3]==0xff) &&
		(data[4]==0xff) &&(data[5]==0xff) && (data[6]==0xff)  && (data[7]==0xff))
		return _FALSE;
	else
		return _TRUE;

}

static int
Hal_EfusePgPacketRead(	IN	PADAPTER	pAdapter,
					IN	u8			offset,
					IN	u8			*data,
					IN	BOOLEAN			bPseudoTest)
{
	int	ret=0;

	if(IS_HARDWARE_TYPE_8192C(pAdapter))
	{
		ret = hal_EfusePgPacketRead_8192C(pAdapter, offset, data, bPseudoTest);
	}
	else if(IS_HARDWARE_TYPE_8723A(pAdapter))
	{
		ret = hal_EfusePgPacketRead_8723A(pAdapter, offset, data, bPseudoTest);
	}

	return ret;
}

static int
Hal_EfusePgPacketRead_Pseudo(	IN	PADAPTER	pAdapter,
					IN	u8			offset,
					IN	u8			*data,
					IN	BOOLEAN		bPseudoTest)
{
	int	ret=0;

	ret = hal_EfusePgPacketRead_8723A(pAdapter, offset, data, bPseudoTest);

	return ret;
}

static int
rtl8192c_Efuse_PgPacketRead(	IN	PADAPTER	pAdapter,
					IN	u8			offset,
					IN	u8			*data,
					IN	BOOLEAN		bPseudoTest)
{
	int	ret=0;

	if(bPseudoTest)
	{
		ret = Hal_EfusePgPacketRead_Pseudo(pAdapter, offset, data, bPseudoTest);
	}
	else
	{
		ret = Hal_EfusePgPacketRead(pAdapter, offset, data, bPseudoTest);
	}

	return ret;
}

static BOOLEAN
hal_EfuseFixHeaderProcess(
	IN		PADAPTER			pAdapter,
	IN		u8					efuseType,
	IN		PPGPKT_STRUCT		pFixPkt,
	IN		u16					*pAddr,
	IN		BOOLEAN				bPseudoTest
)
{
	u8	originaldata[8], badworden=0;
	u16	efuse_addr=*pAddr;
	u32	PgWriteSuccess=0;

	_rtw_memset((PVOID)originaldata, 0xff, 8);

	if(Efuse_PgPacketRead(pAdapter, pFixPkt->offset, originaldata, bPseudoTest))
	{	//check if data exist
		badworden = Efuse_WordEnableDataWrite(pAdapter, efuse_addr+1, pFixPkt->word_en, originaldata, bPseudoTest);

		if(badworden != 0xf)	// write fail
		{
			if(efuseType == EFUSE_WIFI)
				PgWriteSuccess = Efuse_PgPacketWrite(pAdapter, pFixPkt->offset, badworden, originaldata, bPseudoTest);
			else
				PgWriteSuccess = hal_EfusePgPacketWrite_BT(pAdapter, pFixPkt->offset, badworden, originaldata, bPseudoTest);
			if(!PgWriteSuccess)
				return _FALSE;
			else
				efuse_addr = Efuse_GetCurrentSize(pAdapter, efuseType, bPseudoTest);
		}
		else
		{
			efuse_addr = efuse_addr + (pFixPkt->word_cnts*2) +1;
		}
	}
	else
	{
		efuse_addr = efuse_addr + (pFixPkt->word_cnts*2) +1;
	}
	*pAddr = efuse_addr;
	return _TRUE;
}

static BOOLEAN
hal_EfusePgPacketWrite2ByteHeader(
	IN			PADAPTER		pAdapter,
	IN			u8				efuseType,
	IN			u16				*pAddr,
	IN			PPGPKT_STRUCT	pTargetPkt,
	IN			BOOLEAN			bPseudoTest)
{
	BOOLEAN		bRet=_FALSE, bContinual=_TRUE;
	u16	efuse_addr=*pAddr, efuse_max_available_len=0;
	u8	pg_header=0, tmp_header=0, pg_header_temp=0;
	u8	repeatcnt=0;

	//RTPRINT(FEEPROM, EFUSE_PG, ("Wirte 2byte header\n"));
	EFUSE_GetEfuseDefinition(pAdapter, efuseType, TYPE_AVAILABLE_EFUSE_BYTES_BANK, (PVOID)&efuse_max_available_len, bPseudoTest);

	while(efuse_addr < efuse_max_available_len)
	{
		pg_header = ((pTargetPkt->offset & 0x07) << 5) | 0x0F;
		//RTPRINT(FEEPROM, EFUSE_PG, ("pg_header = 0x%x\n", pg_header));
		efuse_OneByteWrite(pAdapter, efuse_addr, pg_header, bPseudoTest);
		efuse_OneByteRead(pAdapter, efuse_addr, &tmp_header, bPseudoTest);

		while(tmp_header == 0xFF)
		{
			if(repeatcnt++ > EFUSE_REPEAT_THRESHOLD_)
			{
				//RTPRINT(FEEPROM, EFUSE_PG, ("Repeat over limit for pg_header!!\n"));
				return _FALSE;
			}

			efuse_OneByteWrite(pAdapter, efuse_addr, pg_header, bPseudoTest);
			efuse_OneByteRead(pAdapter, efuse_addr, &tmp_header, bPseudoTest);
		}

		//to write ext_header
		if(tmp_header == pg_header)
		{
			efuse_addr++;
			pg_header_temp = pg_header;
			pg_header = ((pTargetPkt->offset & 0x78) << 1) | pTargetPkt->word_en;

			efuse_OneByteWrite(pAdapter, efuse_addr, pg_header, bPseudoTest);
			efuse_OneByteRead(pAdapter, efuse_addr, &tmp_header, bPseudoTest);

			while(tmp_header == 0xFF)
			{
				if(repeatcnt++ > EFUSE_REPEAT_THRESHOLD_)
				{
					//RTPRINT(FEEPROM, EFUSE_PG, ("Repeat over limit for ext_header!!\n"));
					return _FALSE;
				}

				efuse_OneByteWrite(pAdapter, efuse_addr, pg_header, bPseudoTest);
				efuse_OneByteRead(pAdapter, efuse_addr, &tmp_header, bPseudoTest);
			}

			if((tmp_header & 0x0F) == 0x0F)	//word_en PG fail
			{
				if(repeatcnt++ > EFUSE_REPEAT_THRESHOLD_)
				{
					//RTPRINT(FEEPROM, EFUSE_PG, ("Repeat over limit for word_en!!\n"));
					return _FALSE;
				}
				else
				{
					efuse_addr++;
					continue;
				}
			}
			else if(pg_header != tmp_header)	//offset PG fail
			{
				PGPKT_STRUCT	fixPkt;
				//RTPRINT(FEEPROM, EFUSE_PG, ("Error condition for offset PG fail, need to cover the existed data\n"));
				fixPkt.offset = ((pg_header_temp & 0xE0) >> 5) | ((tmp_header & 0xF0) >> 1);
				fixPkt.word_en = tmp_header & 0x0F;
				fixPkt.word_cnts = Efuse_CalculateWordCnts(fixPkt.word_en);
				if(!hal_EfuseFixHeaderProcess(pAdapter, efuseType, &fixPkt, &efuse_addr, bPseudoTest))
					return _FALSE;
			}
			else
			{
				bRet = _TRUE;
				break;
			}
		}
		else if ((tmp_header & 0x1F) == 0x0F)		//wrong extended header
		{
			efuse_addr+=2;
			continue;
		}
	}

	*pAddr = efuse_addr;
	return bRet;
}

static BOOLEAN
hal_EfusePgPacketWrite1ByteHeader(
	IN			PADAPTER		pAdapter,
	IN			u8				efuseType,
	IN			u16				*pAddr,
	IN			PPGPKT_STRUCT	pTargetPkt,
	IN			BOOLEAN			bPseudoTest)
{
	BOOLEAN		bRet=_FALSE;
	u8	pg_header=0, tmp_header=0;
	u16	efuse_addr=*pAddr;
	u8	repeatcnt=0;

	//RTPRINT(FEEPROM, EFUSE_PG, ("Wirte 1byte header\n"));
	pg_header = ((pTargetPkt->offset << 4) & 0xf0) |pTargetPkt->word_en;

	efuse_OneByteWrite(pAdapter, efuse_addr, pg_header, bPseudoTest);
	efuse_OneByteRead(pAdapter, efuse_addr, &tmp_header, bPseudoTest);

	while(tmp_header == 0xFF)
	{
		if(repeatcnt++ > EFUSE_REPEAT_THRESHOLD_)
		{
			return _FALSE;
		}
		efuse_OneByteWrite(pAdapter,efuse_addr, pg_header, bPseudoTest);
		efuse_OneByteRead(pAdapter,efuse_addr, &tmp_header, bPseudoTest);
	}

	if(pg_header == tmp_header)
	{
		bRet = _TRUE;
	}
	else
	{
		PGPKT_STRUCT	fixPkt;
		//RTPRINT(FEEPROM, EFUSE_PG, ("Error condition for fixed PG packet, need to cover the existed data\n"));
		fixPkt.offset = (tmp_header>>4) & 0x0F;
		fixPkt.word_en = tmp_header & 0x0F;
		fixPkt.word_cnts = Efuse_CalculateWordCnts(fixPkt.word_en);
		if(!hal_EfuseFixHeaderProcess(pAdapter, efuseType, &fixPkt, &efuse_addr, bPseudoTest))
			return _FALSE;
	}

	*pAddr = efuse_addr;
	return bRet;
}

static BOOLEAN
hal_EfusePgPacketWriteData(
	IN			PADAPTER		pAdapter,
	IN			u8				efuseType,
	IN			u16				*pAddr,
	IN			PPGPKT_STRUCT	pTargetPkt,
	IN			BOOLEAN			bPseudoTest)
{
	BOOLEAN	bRet=_FALSE;
	u16	efuse_addr=*pAddr;
	u8	badworden=0;
	u32	PgWriteSuccess=0;

	badworden = 0x0f;
	badworden = Efuse_WordEnableDataWrite(pAdapter, efuse_addr+1, pTargetPkt->word_en, pTargetPkt->data, bPseudoTest);
	if(badworden == 0x0F)
	{
		// write ok
		//RTPRINT(FEEPROM, EFUSE_PG, ("hal_EfusePgPacketWriteData ok!!\n"));
		return _TRUE;
	}
	else
	{
		//RTPRINT(FEEPROM, EFUSE_PG, ("hal_EfusePgPacketWriteData Fail!!\n"));
		//reorganize other pg packet
		if(efuseType == EFUSE_WIFI)
			PgWriteSuccess = Efuse_PgPacketWrite(pAdapter, pTargetPkt->offset, badworden, pTargetPkt->data, bPseudoTest);
		else
			PgWriteSuccess = hal_EfusePgPacketWrite_BT(pAdapter, pTargetPkt->offset, badworden, pTargetPkt->data, bPseudoTest);
		if(!PgWriteSuccess)
			return _FALSE;
		else
			return _TRUE;
	}

	return bRet;
}

static BOOLEAN
hal_EfusePgPacketWriteHeader(
	IN			PADAPTER		pAdapter,
	IN			u8				efuseType,
	IN			u16				*pAddr,
	IN			PPGPKT_STRUCT	pTargetPkt,
	IN			BOOLEAN			bPseudoTest)
{
	BOOLEAN		bRet=_FALSE;

	if(pTargetPkt->offset >= EFUSE_MAX_SECTION_BASE)
	{
		bRet = hal_EfusePgPacketWrite2ByteHeader(pAdapter, efuseType, pAddr, pTargetPkt, bPseudoTest);
	}
	else
	{
		bRet = hal_EfusePgPacketWrite1ByteHeader(pAdapter, efuseType, pAddr, pTargetPkt, bPseudoTest);
	}

	return bRet;
}

static BOOLEAN
wordEnMatched(
	IN	PPGPKT_STRUCT	pTargetPkt,
	IN	PPGPKT_STRUCT	pCurPkt,
	IN	u8				*pWden
)
{
	u8	match_word_en = 0x0F;	// default all words are disabled
	u8	i;

	// check if the same words are enabled both target and current PG packet
	if( ((pTargetPkt->word_en & BIT0) == 0) &&
		((pCurPkt->word_en & BIT0) == 0) )
	{
		match_word_en &= ~BIT0;				// enable word 0
	}
	if( ((pTargetPkt->word_en & BIT1) == 0) &&
		((pCurPkt->word_en & BIT1) == 0) )
	{
		match_word_en &= ~BIT1;				// enable word 1
	}
	if( ((pTargetPkt->word_en & BIT2) == 0) &&
		((pCurPkt->word_en & BIT2) == 0) )
	{
		match_word_en &= ~BIT2;				// enable word 2
	}
	if( ((pTargetPkt->word_en & BIT3) == 0) &&
		((pCurPkt->word_en & BIT3) == 0) )
	{
		match_word_en &= ~BIT3;				// enable word 3
	}

	*pWden = match_word_en;

	if(match_word_en != 0xf)
		return _TRUE;
	else
		return _FALSE;
}

static BOOLEAN
hal_EfuseCheckIfDatafollowed(
	IN		PADAPTER		pAdapter,
	IN		u8				word_cnts,
	IN		u16				startAddr,
	IN		BOOLEAN			bPseudoTest
	)
{
	BOOLEAN		bRet=_FALSE;
	u8	i, efuse_data;

	for(i=0; i<(word_cnts*2) ; i++)
	{
		if(efuse_OneByteRead(pAdapter, (startAddr+i) ,&efuse_data, bPseudoTest)&&(efuse_data != 0xFF))
			bRet = _TRUE;
	}

	return bRet;
}

static BOOLEAN
hal_EfusePartialWriteCheck(
					IN	PADAPTER		pAdapter,
					IN	u8				efuseType,
					IN	u16				*pAddr,
					IN	PPGPKT_STRUCT	pTargetPkt,
					IN	BOOLEAN			bPseudoTest
					)
{
	BOOLEAN		bRet=_FALSE;
	u8	i, efuse_data=0, cur_header=0;
	u8	new_wden=0, matched_wden=0, badworden=0;
	u16	startAddr=0, efuse_max_available_len=0, efuse_max=0;
	PGPKT_STRUCT	curPkt;

	EFUSE_GetEfuseDefinition(pAdapter, efuseType, TYPE_AVAILABLE_EFUSE_BYTES_BANK, (PVOID)&efuse_max_available_len, bPseudoTest);
	EFUSE_GetEfuseDefinition(pAdapter, efuseType, TYPE_EFUSE_REAL_CONTENT_LEN, (PVOID)&efuse_max, bPseudoTest);

	if(efuseType == EFUSE_WIFI)
	{
		if(bPseudoTest)
		{
			startAddr = (u16)(fakeEfuseUsedBytes%EFUSE_REAL_CONTENT_LEN);
		}
		else
		{
			pAdapter->HalFunc.GetHwRegHandler(pAdapter, HW_VAR_EFUSE_BYTES, (u8 *)&startAddr);
			startAddr%=EFUSE_REAL_CONTENT_LEN;
		}
	}
	else
	{
		if(bPseudoTest)
		{
			startAddr = (u16)(fakeBTEfuseUsedBytes%EFUSE_REAL_CONTENT_LEN);
		}
		else
		{
			startAddr = (u16)(BTEfuseUsedBytes%EFUSE_REAL_CONTENT_LEN);
		}
	}
	//RTPRINT(FEEPROM, EFUSE_PG, ("hal_EfusePartialWriteCheck(), startAddr=%d\n", startAddr));

	while(1)
	{
		if(startAddr >= efuse_max_available_len)
		{
			bRet = _FALSE;
			break;
		}

		if(efuse_OneByteRead(pAdapter, startAddr, &efuse_data, bPseudoTest) && (efuse_data!=0xFF))
		{
			if(EXT_HEADER(efuse_data))
			{
				cur_header = efuse_data;
				startAddr++;
				efuse_OneByteRead(pAdapter, startAddr, &efuse_data, bPseudoTest);
				if(ALL_WORDS_DISABLED(efuse_data))
				{
					//RTPRINT(FEEPROM, EFUSE_PG, ("Error condition, all words disabled"));
					bRet = _FALSE;
					break;
				}
				else
				{
					curPkt.offset = ((cur_header & 0xE0) >> 5) | ((efuse_data & 0xF0) >> 1);
					curPkt.word_en = efuse_data & 0x0F;
				}
			}
			else
			{
				cur_header  =  efuse_data;
				curPkt.offset = (cur_header>>4) & 0x0F;
				curPkt.word_en = cur_header & 0x0F;
			}

			curPkt.word_cnts = Efuse_CalculateWordCnts(curPkt.word_en);
			// if same header is found but no data followed
			// write some part of data followed by the header.
			if( (curPkt.offset == pTargetPkt->offset) &&
				(!hal_EfuseCheckIfDatafollowed(pAdapter, curPkt.word_cnts, startAddr+1, bPseudoTest)) &&
				wordEnMatched(pTargetPkt, &curPkt, &matched_wden) )
			{
				//RTPRINT(FEEPROM, EFUSE_PG, ("Need to partial write data by the previous wrote header\n"));
				// Here to write partial data
				badworden = Efuse_WordEnableDataWrite(pAdapter, startAddr+1, matched_wden, pTargetPkt->data, bPseudoTest);
				if(badworden != 0x0F)
				{
					u32	PgWriteSuccess=0;
					// if write fail on some words, write these bad words again
					if(efuseType == EFUSE_WIFI)
						PgWriteSuccess = Efuse_PgPacketWrite(pAdapter, pTargetPkt->offset, badworden, pTargetPkt->data, bPseudoTest);
					else
						PgWriteSuccess = hal_EfusePgPacketWrite_BT(pAdapter, pTargetPkt->offset, badworden, pTargetPkt->data, bPseudoTest);

					if(!PgWriteSuccess)
					{
						bRet = _FALSE;	// write fail, return
						break;
					}
				}
				// partial write ok, update the target packet for later use
				for(i=0; i<4; i++)
				{
					if((matched_wden & (0x1<<i)) == 0)	// this word has been written
					{
						pTargetPkt->word_en |= (0x1<<i);	// disable the word
					}
				}
				pTargetPkt->word_cnts = Efuse_CalculateWordCnts(pTargetPkt->word_en);
			}
			// read from next header
			startAddr = startAddr + (curPkt.word_cnts*2) +1;
		}
		else
		{
			// not used header, 0xff
			*pAddr = startAddr;
			//RTPRINT(FEEPROM, EFUSE_PG, ("Started from unused header offset=%d\n", startAddr));
			bRet = _TRUE;
			break;
		}
	}
	return bRet;
}

static BOOLEAN
hal_EfusePgCheckAvailableAddr(
	IN	PADAPTER	pAdapter,
	IN	u8			efuseType,
	IN	BOOLEAN		bPseudoTest
	)
{
	u16	efuse_max_available_len=0;

	EFUSE_GetEfuseDefinition(pAdapter, efuseType, TYPE_AVAILABLE_EFUSE_BYTES_TOTAL, (PVOID)&efuse_max_available_len, bPseudoTest);
	//RTPRINT(FEEPROM, EFUSE_PG, ("efuse_max_available_len = %d\n", efuse_max_available_len));

	if(Efuse_GetCurrentSize(pAdapter, efuseType, bPseudoTest) >= efuse_max_available_len)
	{
		//RTPRINT(FEEPROM, EFUSE_PG, ("hal_EfusePgCheckAvailableAddr error!!\n"));
		return _FALSE;
	}
	return _TRUE;
}

static VOID
hal_EfuseConstructPGPkt(
					IN	u8 				offset,
					IN	u8				word_en,
					IN	u8				*pData,
					IN	PPGPKT_STRUCT	pTargetPkt

)
{
	_rtw_memset((PVOID)pTargetPkt->data, 0xFF, sizeof(u8)*8);
	pTargetPkt->offset = offset;
	pTargetPkt->word_en= word_en;
	efuse_WordEnableDataRead(word_en, pData, pTargetPkt->data);
	pTargetPkt->word_cnts = Efuse_CalculateWordCnts(pTargetPkt->word_en);

	//RTPRINT(FEEPROM, EFUSE_PG, ("hal_EfuseConstructPGPkt(), targetPkt, offset=%d, word_en=0x%x, word_cnts=%d\n", pTargetPkt->offset, pTargetPkt->word_en, pTargetPkt->word_cnts));
}

static BOOLEAN
hal_EfusePgPacketWrite_BT(
					IN	PADAPTER	pAdapter,
					IN	u8 			offset,
					IN	u8			word_en,
					IN	u8			*pData,
					IN	BOOLEAN		bPseudoTest
					)
{
	PGPKT_STRUCT 	targetPkt;
	u16	startAddr=0;
	u8	efuseType=EFUSE_BT;

	if(!hal_EfusePgCheckAvailableAddr(pAdapter, efuseType, bPseudoTest))
		return _FALSE;

	hal_EfuseConstructPGPkt(offset, word_en, pData, &targetPkt);

	if(!hal_EfusePartialWriteCheck(pAdapter, efuseType, &startAddr, &targetPkt, bPseudoTest))
		return _FALSE;

	if(!hal_EfusePgPacketWriteHeader(pAdapter, efuseType, &startAddr, &targetPkt, bPseudoTest))
		return _FALSE;

	if(!hal_EfusePgPacketWriteData(pAdapter, efuseType, &startAddr, &targetPkt, bPseudoTest))
		return _FALSE;

	return _TRUE;
}

static BOOLEAN
hal_EfusePgPacketWrite_8723A(
					IN	PADAPTER		pAdapter,
					IN	u8 			offset,
					IN	u8			word_en,
					IN	u8			*pData,
					IN	BOOLEAN		bPseudoTest
					)
{
	PGPKT_STRUCT 	targetPkt;
	u16			startAddr=0;
	u8			efuseType=EFUSE_WIFI;

	if(!hal_EfusePgCheckAvailableAddr(pAdapter, efuseType, bPseudoTest))
		return _FALSE;

	hal_EfuseConstructPGPkt(offset, word_en, pData, &targetPkt);

	if(!hal_EfusePartialWriteCheck(pAdapter, efuseType, &startAddr, &targetPkt, bPseudoTest))
		return _FALSE;

	if(!hal_EfusePgPacketWriteHeader(pAdapter, efuseType, &startAddr, &targetPkt, bPseudoTest))
		return _FALSE;

	if(!hal_EfusePgPacketWriteData(pAdapter, efuseType, &startAddr, &targetPkt, bPseudoTest))
		return _FALSE;

	return _TRUE;
}

static int
hal_EfusePgPacketWrite_8192C(IN	PADAPTER	pAdapter,
					IN	u8 			offset,
					IN	u8			word_en,
					IN	u8			*data,
					IN	BOOLEAN		bPseudoTest)
{
	u8	WriteState = PG_STATE_HEADER;

	int	bContinual = _TRUE,bDataEmpty=_TRUE, bResult = _TRUE;
	u16	efuse_addr = 0;
	u8	efuse_data;

	u8	pg_header = 0;

	u8	tmp_word_cnts=0,target_word_cnts=0;
	u8	tmp_header,match_word_en,tmp_word_en;

	PGPKT_STRUCT target_pkt;
	PGPKT_STRUCT tmp_pkt;

	u8	originaldata[sizeof(u8)*8];
	u8	tmpindex = 0,badworden = 0x0F;

	static int	repeat_times = 0;
	u8	efuseType=EFUSE_WIFI;

	//
	// <Roger_Notes> Efuse has been pre-programmed dummy 5Bytes at the end of Efuse by CP.
	// So we have to prevent unexpected data string connection, which will cause
	// incorrect data auto-load from HW. The total size is equal or smaller than 498bytes
	// (i.e., offset 0~497, and dummy 1bytes) expected after CP test.
	// 2009.02.19.
	//
	if( Efuse_GetCurrentSize(pAdapter, efuseType, bPseudoTest) >= (EFUSE_REAL_CONTENT_LEN-EFUSE_OOB_PROTECT_BYTES))
	{
		//RTPRINT(FEEPROM, EFUSE_PG, ("hal_EfusePgPacketWrite_8192C(), over size\n"));
		return _FALSE;
	}

	// Init the 8 bytes content as 0xff
	target_pkt.offset = offset;
	target_pkt.word_en= word_en;

	_rtw_memset((PVOID)target_pkt.data, 0xFF, sizeof(u8)*8);

	efuse_WordEnableDataRead(word_en,data,target_pkt.data);
	target_word_cnts = Efuse_CalculateWordCnts(target_pkt.word_en);

	//efuse_reg_ctrl(pAdapter,_TRUE);//power on
	//RTPRINT(FEEPROM, EFUSE_PG, ("EFUSE Power ON\n"));

	//
	// <Roger_Notes> Efuse has been pre-programmed dummy 5Bytes at the end of Efuse by CP.
	// So we have to prevent unexpected data string connection, which will cause
	// incorrect data auto-load from HW. Dummy 1bytes is additional.
	// 2009.02.19.
	//
	while( bContinual && (efuse_addr  < (EFUSE_REAL_CONTENT_LEN-EFUSE_OOB_PROTECT_BYTES)) )
	{

		if(WriteState==PG_STATE_HEADER)
		{
			bDataEmpty=_TRUE;
			badworden = 0x0F;
			//************  so *******************
			//RTPRINT(FEEPROM, EFUSE_PG, ("EFUSE PG_STATE_HEADER\n"));
			if (	efuse_OneByteRead(pAdapter, efuse_addr ,&efuse_data, bPseudoTest) &&
				(efuse_data!=0xFF))
			{
				tmp_header  =  efuse_data;

				tmp_pkt.offset 	= (tmp_header>>4) & 0x0F;
				tmp_pkt.word_en 	= tmp_header & 0x0F;
				tmp_word_cnts =  Efuse_CalculateWordCnts(tmp_pkt.word_en);

				//************  so-1 *******************
				if(tmp_pkt.offset  != target_pkt.offset)
				{
					efuse_addr = efuse_addr + (tmp_word_cnts*2) +1; //Next pg_packet
					#if (EFUSE_ERROE_HANDLE == 1)
					WriteState = PG_STATE_HEADER;
					#endif
				}
				else
				{
					//************  so-2 *******************
					for(tmpindex=0 ; tmpindex<(tmp_word_cnts*2) ; tmpindex++)
					{
						if(efuse_OneByteRead(pAdapter, (efuse_addr+1+tmpindex) ,&efuse_data, bPseudoTest)&&(efuse_data != 0xFF)){
							bDataEmpty = _FALSE;
						}
					}
					//************  so-2-1 *******************
					if(bDataEmpty == _FALSE)
					{
						efuse_addr = efuse_addr + (tmp_word_cnts*2) +1; //Next pg_packet
						#if (EFUSE_ERROE_HANDLE == 1)
						WriteState=PG_STATE_HEADER;
						#endif
					}
					else
					{//************  so-2-2 *******************
						match_word_en = 0x0F;
						if(   !( (target_pkt.word_en&BIT0)|(tmp_pkt.word_en&BIT0)  ))
						{
							 match_word_en &= (~BIT0);
						}
						if(   !( (target_pkt.word_en&BIT1)|(tmp_pkt.word_en&BIT1)  ))
						{
							 match_word_en &= (~BIT1);
						}
						if(   !( (target_pkt.word_en&BIT2)|(tmp_pkt.word_en&BIT2)  ))
						{
							 match_word_en &= (~BIT2);
						}
						if(   !( (target_pkt.word_en&BIT3)|(tmp_pkt.word_en&BIT3)  ))
						{
							 match_word_en &= (~BIT3);
						}

						//************  so-2-2-A *******************
						if((match_word_en&0x0F)!=0x0F)
						{
							badworden = Efuse_WordEnableDataWrite(pAdapter,efuse_addr+1, tmp_pkt.word_en ,target_pkt.data, bPseudoTest);

							//************  so-2-2-A-1 *******************
							//############################
							if(0x0F != (badworden&0x0F))
							{
								u8	reorg_offset = offset;
								u8	reorg_worden=badworden;
								Efuse_PgPacketWrite(pAdapter,reorg_offset,reorg_worden,originaldata, bPseudoTest);
							}
							//############################

							tmp_word_en = 0x0F;
							if(  (target_pkt.word_en&BIT0)^(match_word_en&BIT0)  )
							{
								tmp_word_en &= (~BIT0);
							}
							if(   (target_pkt.word_en&BIT1)^(match_word_en&BIT1) )
							{
								tmp_word_en &=  (~BIT1);
							}
							if(   (target_pkt.word_en&BIT2)^(match_word_en&BIT2) )
							{
								tmp_word_en &= (~BIT2);
							}
							if(   (target_pkt.word_en&BIT3)^(match_word_en&BIT3) )
							{
								tmp_word_en &=(~BIT3);
							}

							//************  so-2-2-A-2 *******************
							if((tmp_word_en&0x0F)!=0x0F){
								//reorganize other pg packet
								//efuse_addr = efuse_addr + (2*tmp_word_cnts) +1;//next pg packet addr
								efuse_addr = Efuse_GetCurrentSize(pAdapter, efuseType, bPseudoTest);
								//===========================
								target_pkt.offset = offset;
								target_pkt.word_en= tmp_word_en;
								//===========================
							}else{
								bContinual = _FALSE;
							}
							#if (EFUSE_ERROE_HANDLE == 1)
							WriteState=PG_STATE_HEADER;
							repeat_times++;
							if(repeat_times>EFUSE_REPEAT_THRESHOLD_){
								bContinual = _FALSE;
								bResult = _FALSE;
							}
							#endif
						}
						else{//************  so-2-2-B *******************
							//reorganize other pg packet
							efuse_addr = efuse_addr + (2*tmp_word_cnts) +1;//next pg packet addr
							//===========================
							target_pkt.offset = offset;
							target_pkt.word_en= target_pkt.word_en;
							//===========================
							#if (EFUSE_ERROE_HANDLE == 1)
							WriteState=PG_STATE_HEADER;
							#endif
						}
					}
				}
				//RTPRINT(FEEPROM, EFUSE_PG, ("EFUSE PG_STATE_HEADER-1\n"));
			}
			else		//************  s1: header == oxff  *******************
			{
				pg_header = ((target_pkt.offset << 4)&0xf0) |target_pkt.word_en;

				efuse_OneByteWrite(pAdapter,efuse_addr, pg_header, bPseudoTest);
				efuse_OneByteRead(pAdapter,efuse_addr, &tmp_header, bPseudoTest);

				if(tmp_header == pg_header)
				{ //************  s1-1*******************
					WriteState = PG_STATE_DATA;
				}
				#if (EFUSE_ERROE_HANDLE == 1)
				else if(tmp_header == 0xFF){//************  s1-3: if Write or read func doesn't work *******************
					//efuse_addr doesn't change
					WriteState = PG_STATE_HEADER;
					repeat_times++;
					if(repeat_times>EFUSE_REPEAT_THRESHOLD_){
						bContinual = _FALSE;
						bResult = _FALSE;
					}
				}
				#endif
				else
				{//************  s1-2 : fixed the header procedure *******************
					tmp_pkt.offset = (tmp_header>>4) & 0x0F;
					tmp_pkt.word_en=  tmp_header & 0x0F;
					tmp_word_cnts =  Efuse_CalculateWordCnts(tmp_pkt.word_en);

					//************  s1-2-A :cover the exist data *******************
					//memset(originaldata,0xff,sizeof(UINT8)*8);
					_rtw_memset((PVOID)originaldata, 0xff, sizeof(u8)*8);

					if(Efuse_PgPacketRead( pAdapter, tmp_pkt.offset,originaldata, bPseudoTest))
					{	//check if data exist
						//efuse_reg_ctrl(pAdapter,_TRUE);//power on
						badworden = Efuse_WordEnableDataWrite(pAdapter,efuse_addr+1,tmp_pkt.word_en,originaldata, bPseudoTest);
						//############################
						if(0x0F != (badworden&0x0F))
						{
							u8	reorg_offset = tmp_pkt.offset;
							u8	reorg_worden=badworden;
							Efuse_PgPacketWrite(pAdapter,reorg_offset,reorg_worden,originaldata, bPseudoTest);
							efuse_addr = Efuse_GetCurrentSize(pAdapter, efuseType, bPseudoTest);
						}
						//############################
						else{
							efuse_addr = efuse_addr + (tmp_word_cnts*2) +1; //Next pg_packet
						}
					}
					 //************  s1-2-B: wrong address*******************
					else
					{
						efuse_addr = efuse_addr + (tmp_word_cnts*2) +1; //Next pg_packet
					}

					#if (EFUSE_ERROE_HANDLE == 1)
					WriteState=PG_STATE_HEADER;
					repeat_times++;
					if(repeat_times>EFUSE_REPEAT_THRESHOLD_){
						bContinual = _FALSE;
						bResult = _FALSE;
					}
					#endif

					//RTPRINT(FEEPROM, EFUSE_PG, ("EFUSE PG_STATE_HEADER-2\n"));
				}

			}

		}
		//write data state
		else if(WriteState==PG_STATE_DATA)
		{	//************  s1-1  *******************
			//RTPRINT(FEEPROM, EFUSE_PG, ("EFUSE PG_STATE_DATA\n"));
			badworden = 0x0f;
			badworden = Efuse_WordEnableDataWrite(pAdapter,efuse_addr+1,target_pkt.word_en,target_pkt.data, bPseudoTest);
			if((badworden&0x0F)==0x0F)
			{ //************  s1-1-A *******************
				bContinual = _FALSE;
			}
			else
			{//reorganize other pg packet //************  s1-1-B *******************
				efuse_addr = efuse_addr + (2*target_word_cnts) +1;//next pg packet addr

				//===========================
				target_pkt.offset = offset;
				target_pkt.word_en= badworden;
				target_word_cnts =  Efuse_CalculateWordCnts(target_pkt.word_en);
				//===========================
				#if (EFUSE_ERROE_HANDLE == 1)
				WriteState=PG_STATE_HEADER;
				repeat_times++;
				if(repeat_times>EFUSE_REPEAT_THRESHOLD_){
					bContinual = _FALSE;
					bResult = _FALSE;
				}
				#endif
				//RTPRINT(FEEPROM, EFUSE_PG, ("EFUSE PG_STATE_HEADER-3\n"));
			}
		}
	}

	if(efuse_addr  >= (EFUSE_REAL_CONTENT_LEN-EFUSE_OOB_PROTECT_BYTES))
	{
		//RT_TRACE(COMP_EFUSE, DBG_LOUD, ("hal_EfusePgPacketWrite_8192C(): efuse_addr(%#x) Out of size!!\n", efuse_addr));
	}
	//efuse_reg_ctrl(pAdapter,_FALSE);//power off

	return _TRUE;
}

static int
Hal_EfusePgPacketWrite_Pseudo(IN	PADAPTER	pAdapter,
					IN	u8 			offset,
					IN	u8			word_en,
					IN	u8			*data,
					IN	BOOLEAN		bPseudoTest)
{
	int ret;

	ret = hal_EfusePgPacketWrite_8723A(pAdapter, offset, word_en, data, bPseudoTest);

	return ret;
}

static int
Hal_EfusePgPacketWrite(IN	PADAPTER	pAdapter,
					IN	u8 			offset,
					IN	u8			word_en,
					IN	u8			*data,
					IN	BOOLEAN		bPseudoTest)
{
	int	ret=0;

	if(IS_HARDWARE_TYPE_8192C(pAdapter))
	{
		ret = hal_EfusePgPacketWrite_8192C(pAdapter, offset, word_en, data, bPseudoTest);
	}
	else if(IS_HARDWARE_TYPE_8723A(pAdapter))
	{
		ret = hal_EfusePgPacketWrite_8723A(pAdapter, offset, word_en, data, bPseudoTest);
	}

	return ret;
}

static int
rtl8192c_Efuse_PgPacketWrite(IN	PADAPTER	pAdapter,
					IN	u8 			offset,
					IN	u8			word_en,
					IN	u8			*data,
					IN	BOOLEAN		bPseudoTest)
{
	int	ret;

	if(bPseudoTest)
	{
		ret = Hal_EfusePgPacketWrite_Pseudo(pAdapter, offset, word_en, data, bPseudoTest);
	}
	else
	{
		ret = Hal_EfusePgPacketWrite(pAdapter, offset, word_en, data, bPseudoTest);
	}
	return ret;
}

static 
#ifdef CONFIG_CHIP_VER_INTEGRATION
HAL_VERSION	
ReadChipVersion8723A(
	IN	PADAPTER	padapter
	)
{
	u32				value32;
	HAL_VERSION		ChipVersion;
	HAL_DATA_TYPE	*pHalData;


	pHalData = GET_HAL_DATA(padapter);

	value32 = rtw_read32(padapter, REG_SYS_CFG);
	ChipVersion.ICType = CHIP_8723A;	
	ChipVersion.ChipType = ((value32 & RTL_ID) ? TEST_CHIP : NORMAL_CHIP);
	ChipVersion.RFType = RF_TYPE_1T1R ;
	ChipVersion.VendorType = ((value32 & VENDOR_ID) ? CHIP_VENDOR_UMC : CHIP_VENDOR_TSMC);
	ChipVersion.CUTVersion = (value32 & CHIP_VER_RTL_MASK)>>CHIP_VER_RTL_SHIFT; // IC version (CUT)

	// For regulator mode. by tynli. 2011.01.14
	pHalData->RegulatorMode = ((value32 & SPS_SEL) ? RT_LDO_REGULATOR : RT_SWITCHING_REGULATOR);

	value32 = rtw_read32(padapter, REG_GPIO_OUTSTS);
	ChipVersion.ROMVer = ((value32 & RF_RL_ID) >> 20);	// ROM code version.

	// For multi-function consideration. Added by Roger, 2010.10.06.
	pHalData->MultiFunc = RT_MULTI_FUNC_NONE;
	value32 = rtw_read32(padapter, REG_MULTI_FUNC_CTRL);
	pHalData->MultiFunc |= ((value32 & WL_FUNC_EN) ? RT_MULTI_FUNC_WIFI : 0);
	pHalData->MultiFunc |= ((value32 & BT_FUNC_EN) ? RT_MULTI_FUNC_BT : 0);
	pHalData->MultiFunc |= ((value32 & GPS_FUNC_EN) ? RT_MULTI_FUNC_GPS : 0);
	pHalData->PolarityCtl = ((value32 & WL_HWPDN_SL) ? RT_POLARITY_HIGH_ACT : RT_POLARITY_LOW_ACT);
//#if DBG
#if 1
	dump_chip_info(ChipVersion);
#endif
	pHalData->VersionID = ChipVersion;

	if (IS_1T2R(ChipVersion))
		pHalData->rf_type = RF_1T2R;
	else if (IS_2T2R(ChipVersion))
		pHalData->rf_type = RF_2T2R;
	else
		pHalData->rf_type = RF_1T1R;

	MSG_8192C("RF_Type is %x!!\n", pHalData->rf_type);

	return ChipVersion;
}

#else
VERSION_8192C
ReadChipVersion8723A(
	IN	PADAPTER	padapter
	)
{
	u32				value32;
	u32				ChipVersion;
	HAL_DATA_TYPE	*pHalData;


	pHalData = GET_HAL_DATA(padapter);

	value32 = rtw_read32(padapter, REG_SYS_CFG);
	ChipVersion = CHIP_8723;
	ChipVersion |= ((value32 & RTL_ID) ? 0 : NORMAL_CHIP);
//	ChipVersion |= ((value32 & TYPE_ID) ? RF_TYPE_2T2R : 0); // 1:92c; 0:88c
	ChipVersion |= ((value32 & VENDOR_ID) ? CHIP_VENDOR_UMC : 0);
	ChipVersion |= (value32 & CHIP_VER_RTL_MASK); // IC version (CUT)

	// For regulator mode. by tynli. 2011.01.14
	pHalData->RegulatorMode = ((value32 & SPS_SEL) ? RT_LDO_REGULATOR : RT_SWITCHING_REGULATOR);

	value32 = rtw_read32(padapter, REG_GPIO_OUTSTS);
	ChipVersion |= ((value32 & RF_RL_ID) >> 20);	// ROM code version.

	// For multi-function consideration. Added by Roger, 2010.10.06.
	pHalData->MultiFunc = RT_MULTI_FUNC_NONE;
	value32 = rtw_read32(padapter, REG_MULTI_FUNC_CTRL);
	pHalData->MultiFunc |= ((value32 & WL_FUNC_EN) ? RT_MULTI_FUNC_WIFI : 0);
	pHalData->MultiFunc |= ((value32 & BT_FUNC_EN) ? RT_MULTI_FUNC_BT : 0);
	pHalData->MultiFunc |= ((value32 & GPS_FUNC_EN) ? RT_MULTI_FUNC_GPS : 0);
	pHalData->PolarityCtl = ((value32 & WL_HWPDN_SL) ? RT_POLARITY_HIGH_ACT : RT_POLARITY_LOW_ACT);

//#if DBG
#if 1
	switch(ChipVersion)
	{
		case VERSION_NORMAL_TSMC_CHIP_92C_1T2R:
			MSG_8192C("Chip Version ID: VERSION_NORMAL_TSMC_CHIP_92C_1T2R.\n");
			break;
		case VERSION_NORMAL_TSMC_CHIP_92C:
			MSG_8192C("Chip Version ID: VERSION_NORMAL_TSMC_CHIP_92C.\n");
			break;
		case VERSION_NORMAL_TSMC_CHIP_88C:
			MSG_8192C("Chip Version ID: VERSION_NORMAL_TSMC_CHIP_88C.\n");
			break;
		case VERSION_NORMAL_UMC_CHIP_92C_1T2R_A_CUT:
			MSG_8192C("Chip Version ID: VERSION_NORMAL_UMC_CHIP_92C_1T2R_A_CUT.\n");
			break;
		case VERSION_NORMAL_UMC_CHIP_92C_A_CUT:
			MSG_8192C("Chip Version ID: VERSION_NORMAL_UMC_CHIP_92C_A_CUT.\n");
			break;
		case VERSION_NORMAL_UMC_CHIP_88C_A_CUT:
			MSG_8192C("Chip Version ID: VERSION_NORMAL_UMC_CHIP_88C_A_CUT.\n");
			break;
		case VERSION_NORMAL_UMC_CHIP_92C_1T2R_B_CUT:
			MSG_8192C("Chip Version ID: VERSION_NORMAL_UMC_CHIP_92C_1T2R_B_CUT.\n");
			break;
		case VERSION_NORMAL_UMC_CHIP_92C_B_CUT:
			MSG_8192C("Chip Version ID: VERSION_NORMAL_UMC_CHIP_92C_B_CUT.\n");
			break;
		case VERSION_NORMAL_UMC_CHIP_88C_B_CUT:
			MSG_8192C("Chip Version ID: VERSION_NORMAL_UMC_CHIP_88C_B_CUT.\n");
			break;
		case VERSION_TEST_CHIP_92C:
			MSG_8192C("Chip Version ID: VERSION_TEST_CHIP_92C.\n");
			break;
		case VERSION_TEST_CHIP_88C:
			MSG_8192C("Chip Version ID: VERSION_TEST_CHIP_88C.\n");
			break;
		case VERSION_TEST_UMC_CHIP_8723:
			MSG_8192C("Chip Version ID: VERSION_TEST_UMC_CHIP_8723.\n");
			break;
		case VERSION_NORMAL_UMC_CHIP_8723_1T1R_A_CUT:
			MSG_8192C("Chip Version ID: VERSION_NORMA_UMC_CHIP_8723_1T1R_A_CUT.\n");
			break;
		case VERSION_NORMAL_UMC_CHIP_8723_1T1R_B_CUT:
			MSG_8192C("Chip Version ID: VERSION_NORMA_UMC_CHIP_8723_1T1R_B_CUT.\n");
			break;
		default:
			MSG_8192C("Chip Version ID: unknown(0x%x)\n", ChipVersion);
			break;
	}
#endif

	pHalData->VersionID = ChipVersion;

	if (IS_1T2R(ChipVersion))
		pHalData->rf_type = RF_1T2R;
	else if (IS_2T2R(ChipVersion))
		pHalData->rf_type = RF_2T2R;
	else
		pHalData->rf_type = RF_1T1R;

	MSG_8192C("RF_Type is %x!!\n", pHalData->rf_type);

	return ChipVersion;
}
#endif

static void rtl8723a_read_chip_version(PADAPTER padapter)
{
	ReadChipVersion8723A(padapter);
}

//====================================================================================
//
// 20100209 Joseph:
// This function is used only for 92C to set REG_BCN_CTRL(0x550) register.
// We just reserve the value of the register in variable pHalData->RegBcnCtrlVal and then operate
// the value of the register via atomic operation.
// This prevents from race condition when setting this register.
// The value of pHalData->RegBcnCtrlVal is initialized in HwConfigureRTL8192CE() function.
//
void SetBcnCtrlReg(PADAPTER padapter, u8 SetBits, u8 ClearBits)
{
	PHAL_DATA_TYPE pHalData;
	u32 addr;
	u8 *pRegBcnCtrlVal;


	pHalData = GET_HAL_DATA(padapter);
	pRegBcnCtrlVal = (u8*)&pHalData->RegBcnCtrlVal;

#ifdef CONFIG_CONCURRENT_MODE
	if (padapter->iface_type == IFACE_PORT1)
	{
		addr = REG_BCN_CTRL_1;
		pRegBcnCtrlVal++;
	}
	else
#endif
	{
		addr = REG_BCN_CTRL;
	}

	*pRegBcnCtrlVal = rtw_read8(padapter, addr);
	*pRegBcnCtrlVal |= SetBits;
	*pRegBcnCtrlVal &= ~ClearBits;

#if 0
//#ifdef CONFIG_SDIO_HCI
	if (pHalData->sdio_himr & (SDIO_HIMR_TXBCNOK_MSK | SDIO_HIMR_TXBCNERR_MSK))
		*pRegBcnCtrlVal |= EN_TXBCN_RPT;
#endif

	rtw_write8(padapter, addr, *pRegBcnCtrlVal);
}

void rtl8723a_InitBeaconParameters(PADAPTER padapter)
{
	PHAL_DATA_TYPE pHalData = GET_HAL_DATA(padapter);


	rtw_write16(padapter, REG_BCN_CTRL, 0x1010);
	pHalData->RegBcnCtrlVal = 0x1010;

	// TODO: Remove these magic number
	rtw_write16(padapter, REG_TBTT_PROHIBIT, 0x6404);// ms
	// Firmware will control REG_DRVERLYINT when power saving is enable,
	// so don't set this register on STA mode.
	if (check_fwstate(&padapter->mlmepriv, WIFI_STATION_STATE) == _FALSE)
		rtw_write8(padapter, REG_DRVERLYINT, DRIVER_EARLY_INT_TIME); // 5ms
	rtw_write8(padapter, REG_BCNDMATIM, BCN_DMA_ATIME_INT_TIME); // 2ms

	// Suggested by designer timchen. Change beacon AIFS to the largest number
	// beacause test chip does not contension before sending beacon. by tynli. 2009.11.03
	rtw_write16(padapter, REG_BCNTCFG, 0x660F);
}

void rtl8723a_InitBeaconMaxError(PADAPTER padapter, u8 InfraMode)
{
#ifdef RTL8192CU_ADHOC_WORKAROUND_SETTING
	rtw_write8(padapter, REG_BCN_MAX_ERR, 0xFF);
#else
	//rtw_write8(Adapter, REG_BCN_MAX_ERR, (InfraMode ? 0xFF : 0x10));
#endif
}

static void ResumeTxBeacon(PADAPTER padapter)
{
	PHAL_DATA_TYPE pHalData = GET_HAL_DATA(padapter);


	// 2010.03.01. Marked by tynli. No need to call workitem beacause we record the value
	// which should be read from register to a global variable.

	RT_TRACE(_module_hci_hal_init_c_, _drv_info_, ("+ResumeTxBeacon\n"));

	pHalData->RegFwHwTxQCtrl |= BIT(6);
	rtw_write8(padapter, REG_FWHW_TXQ_CTRL+2, pHalData->RegFwHwTxQCtrl);
	rtw_write8(padapter, REG_TBTT_PROHIBIT+1, 0xff);
	pHalData->RegReg542 |= BIT(0);
	rtw_write8(padapter, REG_TBTT_PROHIBIT+2, pHalData->RegReg542);
}

static void StopTxBeacon(PADAPTER padapter)
{
	PHAL_DATA_TYPE pHalData = GET_HAL_DATA(padapter);


	// 2010.03.01. Marked by tynli. No need to call workitem beacause we record the value
	// which should be read from register to a global variable.

	RT_TRACE(_module_hci_hal_init_c_, _drv_info_, ("+StopTxBeacon\n"));

	pHalData->RegFwHwTxQCtrl &= ~BIT(6);
	rtw_write8(padapter, REG_FWHW_TXQ_CTRL+2, pHalData->RegFwHwTxQCtrl);
	rtw_write8(padapter, REG_TBTT_PROHIBIT+1, 0x64);
	pHalData->RegReg542 &= ~BIT(0);
	rtw_write8(padapter, REG_TBTT_PROHIBIT+2, pHalData->RegReg542);

	CheckFwRsvdPageContent(padapter);  // 2010.06.23. Added by tynli.
}

static void _BeaconFunctionEnable(PADAPTER padapter, u8 Enable, u8 Linked)
{
	SetBcnCtrlReg(padapter, DIS_TSF_UDT | EN_BCN_FUNCTION | DIS_BCNQ_SUB, 0);
	rtw_write8(padapter, REG_RD_CTRL+1, 0x6F);
}

static void rtl8723a_SetBeaconRelatedRegisters(PADAPTER padapter)
{
	u32 value32;
	PHAL_DATA_TYPE pHalData = GET_HAL_DATA(padapter);
	struct mlme_ext_priv *pmlmeext = &padapter->mlmeextpriv;
	struct mlme_ext_info *pmlmeinfo = &pmlmeext->mlmext_info;


	//reset TSF, enable update TSF, correcting TSF On Beacon 
	
	//REG_BCN_INTERVAL
	//REG_BCNDMATIM
	//REG_ATIMWND
	//REG_TBTT_PROHIBIT
	//REG_DRVERLYINT
	//REG_BCN_MAX_ERR	
	//REG_BCNTCFG //(0x510)
	//REG_DUAL_TSF_RST
	//REG_BCN_CTRL //(0x550) 

	//
	// ATIM window
	//
	rtw_write16(padapter, REG_ATIMWND, 2);

	//
	// Beacon interval (in unit of TU).
	//
	rtw_write16(padapter, REG_BCN_INTERVAL, pmlmeinfo->bcn_interval);

	rtl8723a_InitBeaconParameters(padapter);

	rtw_write8(padapter, REG_SLOT, 0x09);

	//
	// Reset TSF Timer to zero, added by Roger. 2008.06.24
	//
	value32 = rtw_read32(padapter, REG_TCR); 
	value32 &= ~TSFRST;
	rtw_write32(padapter, REG_TCR, value32); 

	value32 |= TSFRST;
	rtw_write32(padapter, REG_TCR, value32); 

	// NOTE: Fix test chip's bug (about contention windows's randomness)
	if (check_fwstate(&padapter->mlmepriv, WIFI_ADHOC_STATE|WIFI_ADHOC_MASTER_STATE|WIFI_AP_STATE) == _TRUE)
	{
		rtw_write8(padapter, REG_RXTSF_OFFSET_CCK, 0x50);
		rtw_write8(padapter, REG_RXTSF_OFFSET_OFDM, 0x50);
	}

	_BeaconFunctionEnable(padapter, _TRUE, _TRUE);

	ResumeTxBeacon(padapter);
	SetBcnCtrlReg(padapter, DIS_BCNQ_SUB, 0);
}

void rtl8723a_GetHalODMVar(	
	PADAPTER				Adapter,
	HAL_ODM_VARIABLE		eVariable,
	PVOID					pValue1,
	BOOLEAN					bSet)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	PDM_ODM_T podmpriv = &pHalData->odmpriv;
	switch(eVariable){
		case HAL_ODM_STA_INFO:
			break;
		default:
			break;
	}
}

void rtl8723a_SetHalODMVar(
	PADAPTER				Adapter,
	HAL_ODM_VARIABLE		eVariable,	
	PVOID					pValue1,
	BOOLEAN					bSet)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(Adapter);
	PDM_ODM_T podmpriv = &pHalData->odmpriv;
	switch(eVariable){
		case HAL_ODM_STA_INFO:
			{					
				struct sta_info *psta = (struct sta_info *)pValue1;
				#ifdef CONFIG_CONCURRENT_MODE	
				//get Primary adapter's odmpriv
				if(Adapter->adapter_type > PRIMARY_ADAPTER){
					pHalData = GET_HAL_DATA(Adapter->pbuddy_adapter);
					podmpriv = &pHalData->odmpriv;	
				}
				#endif
				
				if(bSet){
					DBG_8192C("Set STA_(%d) info\n",psta->mac_id);
					ODM_CmnInfoPtrArrayHook(podmpriv, ODM_CMNINFO_STA_STATUS,psta->mac_id,psta);
				}
				else{
					DBG_8192C("Clean STA_(%d) info\n",psta->mac_id);
					ODM_CmnInfoPtrArrayHook(podmpriv, ODM_CMNINFO_STA_STATUS,psta->mac_id,NULL);
				}
			}
			break;
		case HAL_ODM_P2P_STATE:		
				ODM_CmnInfoUpdate(podmpriv,ODM_CMNINFO_WIFI_DIRECT,bSet);
			break;
		case HAL_ODM_WIFI_DISPLAY_STATE:
				ODM_CmnInfoUpdate(podmpriv,ODM_CMNINFO_WIFI_DISPLAY,bSet);
			break;
		default:
			break;
	}
}	

void rtl8723a_set_hal_ops(struct hal_ops *pHalFunc)
{
	pHalFunc->free_hal_data = &rtl8723a_free_hal_data;

	pHalFunc->dm_init = &rtl8723a_init_dm_priv;
	pHalFunc->dm_deinit = &rtl8723a_deinit_dm_priv;

	pHalFunc->read_chip_version = &rtl8723a_read_chip_version;

	pHalFunc->set_bwmode_handler = &PHY_SetBWMode8192C;
	pHalFunc->set_channel_handler = &PHY_SwChnl8192C;

	pHalFunc->hal_dm_watchdog = &rtl8723a_HalDmWatchDog;

	pHalFunc->SetBeaconRelatedRegistersHandler = &rtl8723a_SetBeaconRelatedRegisters;

	pHalFunc->Add_RateATid = &rtl8192c_Add_RateATid;

#ifdef CONFIG_ANTENNA_DIVERSITY
	pHalFunc->SwAntDivBeforeLinkHandler = &SwAntDivBeforeLink8192C;
	pHalFunc->SwAntDivCompareHandler = &SwAntDivCompare8192C;
#endif

	pHalFunc->read_bbreg = &rtl8192c_PHY_QueryBBReg;
	pHalFunc->write_bbreg = &rtl8192c_PHY_SetBBReg;
	pHalFunc->read_rfreg = &rtl8192c_PHY_QueryRFReg;
	pHalFunc->write_rfreg = &rtl8192c_PHY_SetRFReg;


	// Efuse related function
	pHalFunc->EfusePowerSwitch = &rtl8192c_EfusePowerSwitch;
	pHalFunc->ReadEFuse = &rtl8192c_ReadEFuse;
	pHalFunc->EFUSEGetEfuseDefinition = &rtl8192c_EFUSE_GetEfuseDefinition;
	pHalFunc->EfuseGetCurrentSize = &rtl8192c_EfuseGetCurrentSize;
	pHalFunc->Efuse_PgPacketRead = &rtl8192c_Efuse_PgPacketRead;
	pHalFunc->Efuse_PgPacketWrite = &rtl8192c_Efuse_PgPacketWrite;
	pHalFunc->Efuse_WordEnableDataWrite = &rtl8192c_Efuse_WordEnableDataWrite;

#ifdef SILENT_RESET_FOR_SPECIFIC_PLATFOM
	pHalFunc->sreset_init_value = &rtl8192c_sreset_init_value;
	pHalFunc->sreset_reset_value = &rtl8192c_sreset_reset_value;
	pHalFunc->silentreset = &rtl8192c_silentreset_for_specific_platform;
	pHalFunc->sreset_xmit_status_check = &rtl8192c_sreset_xmit_status_check;
	pHalFunc->sreset_linked_status_check  = &rtl8192c_sreset_linked_status_check;
	pHalFunc->sreset_get_wifi_status  = &rtl8192c_sreset_get_wifi_status;
#endif
	pHalFunc->GetHalODMVarHandler = &rtl8723a_GetHalODMVar;
	pHalFunc->SetHalODMVarHandler = &rtl8723a_SetHalODMVar;
}

void rtl8723a_InitAntenna_Selection(PADAPTER padapter)
{
	PHAL_DATA_TYPE pHalData;
	u8 val;


	pHalData = GET_HAL_DATA(padapter);

	val = rtw_read8(padapter, REG_LEDCFG2);
	// Let 8051 take control antenna settting
	val |= BIT(7); // DPDT_SEL_EN, 0x4C[23]
	rtw_write8(padapter, REG_LEDCFG2, val);
}

u8 GetEEPROMSize8723A(PADAPTER padapter)
{
	u8 size = 0;
	u32	cr;

	cr = rtw_read16(padapter, REG_9346CR);
	// 6: EEPROM used is 93C46, 4: boot from E-Fuse.
	size = (cr & BOOT_FROM_EEPROM) ? 6 : 4;

	MSG_8192C("EEPROM type is %s\n", size==4 ? "E-FUSE" : "93C46");

	return size;
}

#if defined(CONFIG_USB_HCI) || defined(CONFIG_SDIO_HCI)
//-------------------------------------------------------------------------
//
// LLT R/W/Init function
//
//-------------------------------------------------------------------------
s32 _LLTWrite(PADAPTER padapter, u32 address, u32 data)
{
	s32	status = _SUCCESS;
	s32	count = 0;
	u32	value = _LLT_INIT_ADDR(address) | _LLT_INIT_DATA(data) | _LLT_OP(_LLT_WRITE_ACCESS);
	u16	LLTReg = REG_LLT_INIT;


	rtw_write32(padapter, LLTReg, value);

	//polling
	do {
		value = rtw_read32(padapter, LLTReg);
		if (_LLT_NO_ACTIVE == _LLT_OP_VALUE(value)) {
			break;
		}

		if (count > POLLING_LLT_THRESHOLD) {
			RT_TRACE(_module_hal_init_c_, _drv_err_, ("Failed to polling write LLT done at address %d!\n", address));
			status = _FAIL;
			break;
		}
	} while (count++);

	return status;
}

u8 _LLTRead(PADAPTER padapter, u32 address)
{
	s32	count = 0;
	u32	value = _LLT_INIT_ADDR(address) | _LLT_OP(_LLT_READ_ACCESS);
	u16	LLTReg = REG_LLT_INIT;


	rtw_write32(padapter, LLTReg, value);

	//polling and get value
	do {
		value = rtw_read32(padapter, LLTReg);
		if (_LLT_NO_ACTIVE == _LLT_OP_VALUE(value)) {
			return (u8)value;
		}

		if (count > POLLING_LLT_THRESHOLD) {
			RT_TRACE(_module_hal_init_c_, _drv_err_, ("Failed to polling read LLT done at address %d!\n", address));
			break;
		}
	} while (count++);

	return 0xFF;
}

s32 InitLLTTable(PADAPTER padapter, u32 boundary)
{
	s32	status = _SUCCESS;
	u32	i;
	u32	txpktbuf_bndy = boundary;
	u32	Last_Entry_Of_TxPktBuf = LAST_ENTRY_OF_TX_PKT_BUFFER;
	HAL_DATA_TYPE *pHalData	= GET_HAL_DATA(padapter);

#if 0
	if (IS_HARDWARE_TYPE_8192DU(padapter))
	{
		if (pHalData->MacPhyMode92D != SINGLEMAC_SINGLEPHY) {
			// for 92du two mac: The page size is different from 92c and 92s
			txpktbuf_bndy = TX_PAGE_BOUNDARY_DUAL_MAC;
			Last_Entry_Of_TxPktBuf = LAST_ENTRY_OF_TX_PKT_BUFFER_DUAL_MAC;
		} else {
			txpktbuf_bndy = boundary;
			Last_Entry_Of_TxPktBuf = LAST_ENTRY_OF_TX_PKT_BUFFER;
			//txpktbuf_bndy =253;
			//Last_Entry_Of_TxPktBuf=255;
		}
	}
#endif
	for (i = 0; i < (txpktbuf_bndy - 1); i++) {
		status = _LLTWrite(padapter, i, i + 1);
		if (_SUCCESS != status) {
			return status;
		}
	}

	// end of list
	status = _LLTWrite(padapter, (txpktbuf_bndy - 1), 0xFF);
	if (_SUCCESS != status) {
		return status;
	}

	// Make the other pages as ring buffer
	// This ring buffer is used as beacon buffer if we config this MAC as two MAC transfer.
	// Otherwise used as local loopback buffer.
	for (i = txpktbuf_bndy; i < Last_Entry_Of_TxPktBuf; i++) {
		status = _LLTWrite(padapter, i, (i + 1));
		if (_SUCCESS != status) {
			return status;
		}
	}

	// Let last entry point to the start entry of ring buffer
	status = _LLTWrite(padapter, Last_Entry_Of_TxPktBuf, txpktbuf_bndy);
	if (_SUCCESS != status) {
		return status;
	}

	return status;
}
#endif

#if defined(CONFIG_USB_HCI) || defined(CONFIG_SDIO_HCI)
void _DisableGPIO(PADAPTER	padapter)
{
/***************************************
j. GPIO_PIN_CTRL 0x44[31:0]=0x000		//
k.Value = GPIO_PIN_CTRL[7:0]
l. GPIO_PIN_CTRL 0x44[31:0] = 0x00FF0000 | (value <<8); //write external PIN level
m. GPIO_MUXCFG 0x42 [15:0] = 0x0780
n. LEDCFG 0x4C[15:0] = 0x8080
***************************************/
	u8	value8;
	u16	value16;
	u32	value32;
	u32	u4bTmp;


	//1. Disable GPIO[7:0]
	rtw_write16(padapter, REG_GPIO_PIN_CTRL+2, 0x0000);
	value32 = rtw_read32(padapter, REG_GPIO_PIN_CTRL) & 0xFFFF00FF;
	u4bTmp = value32 & 0x000000FF;
	value32 |= ((u4bTmp<<8) | 0x00FF0000);
	rtw_write32(padapter, REG_GPIO_PIN_CTRL, value32);

	if (IS_HARDWARE_TYPE_8723AU(padapter) ||
		IS_HARDWARE_TYPE_8723AS(padapter))
	{
		//
		// <Roger_Notes> For RTL8723u multi-function configuration which was autoload from Efuse offset 0x0a and 0x0b,
		// WLAN HW GPIO[9], GPS HW GPIO[10] and BT HW GPIO[11].
		// Added by Roger, 2010.10.07.
		//
		//2. Disable GPIO[8] and GPIO[12]
		rtw_write16(padapter, REG_GPIO_IO_SEL_2, 0x0000); // Configure all pins as input mode.
	    	value32 = rtw_read32(padapter, REG_GPIO_PIN_CTRL_2) & 0xFFFF001F;
		u4bTmp = value32 & 0x0000001F;
//		if( IS_MULTI_FUNC_CHIP(padapter) )
//			value32 |= ((u4bTmp<<8) | 0x00110000); // Set pin 8 and pin 12 to output mode.
//		else
			value32 |= ((u4bTmp<<8) | 0x001D0000); // Set pin 8, 10, 11 and pin 12 to output mode.
		rtw_write32(padapter, REG_GPIO_PIN_CTRL_2, value32);
	}
	else
	{
		//2. Disable GPIO[10:8]
		rtw_write8(padapter, REG_MAC_PINMUX_CFG, 0x00);
		value16 = rtw_read16(padapter, REG_GPIO_IO_SEL) & 0xFF0F;
		value8 = (u8) (value16&0x000F);
		value16 |= ((value8<<4) | 0x0780);
		rtw_write16(padapter, REG_GPIO_IO_SEL, value16);
	}

	//3. Disable LED0 & 1
	if(IS_HARDWARE_TYPE_8192DU(padapter))
	{
		rtw_write16(padapter, REG_LEDCFG0, 0x8888);
	}
	else
	{
		rtw_write16(padapter, REG_LEDCFG0, 0x08080);
	}
//	RT_TRACE(COMP_INIT, DBG_LOUD, ("======> Disable GPIO and LED.\n"));
} //end of _DisableGPIO()

void _DisableRFAFEAndResetBB8192C(PADAPTER padapter)
{
/**************************************
a.	TXPAUSE 0x522[7:0] = 0xFF             //Pause MAC TX queue
b.	RF path 0 offset 0x00 = 0x00            // disable RF
c. 	APSD_CTRL 0x600[7:0] = 0x40
d.	SYS_FUNC_EN 0x02[7:0] = 0x16		//reset BB state machine
e.	SYS_FUNC_EN 0x02[7:0] = 0x14		//reset BB state machine
***************************************/
    	u8 eRFPath = 0, value8 = 0;

	rtw_write8(padapter, REG_TXPAUSE, 0xFF);

	PHY_SetRFReg(padapter, (RF_RADIO_PATH_E)eRFPath, 0x0, bMaskByte0, 0x0);

	value8 |= APSDOFF;
	rtw_write8(padapter, REG_APSD_CTRL, value8);//0x40

	// Set BB reset at first
	value8 = 0 ;
	value8 |= (FEN_USBD | FEN_USBA | FEN_BB_GLB_RSTn);
	rtw_write8(padapter, REG_SYS_FUNC_EN, value8 );//0x16

	// Set global reset.
	value8 &= ~FEN_BB_GLB_RSTn;
	rtw_write8(padapter, REG_SYS_FUNC_EN, value8); //0x14

	// 2010/08/12 MH We need to set BB/GLBAL reset to save power for SS mode.

//	RT_TRACE(COMP_INIT, DBG_LOUD, ("======> RF off and reset BB.\n"));
}

void _DisableRFAFEAndResetBB(PADAPTER padapter)
{
#if 0
	if (IS_HARDWARE_TYPE_8192D(padapter))
		_DisableRFAFEAndResetBB8192D(padapter);
	else
#endif
		_DisableRFAFEAndResetBB8192C(padapter);
}

void _ResetDigitalProcedure1_92C(PADAPTER padapter, BOOLEAN bWithoutHWSM)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);

	if (IS_FW_81xxC(padapter) && (pHalData->FirmwareVersion <= 0x20))
	{
		#if 0
/*****************************
		f.	SYS_FUNC_EN 0x03[7:0]=0x54		// reset MAC register, DCORE
		g.	MCUFWDL 0x80[7:0]=0				// reset MCU ready status
******************************/
	u32	value32 = 0;
		rtw_write8(padapter, REG_SYS_FUNC_EN+1, 0x54);
		rtw_write8(padapter, REG_MCUFWDL, 0);
		#else
		/*****************************
		f.	MCUFWDL 0x80[7:0]=0				// reset MCU ready status
		g.	SYS_FUNC_EN 0x02[10]= 0			// reset MCU register, (8051 reset)
		h.	SYS_FUNC_EN 0x02[15-12]= 5		// reset MAC register, DCORE
		i.     SYS_FUNC_EN 0x02[10]= 1			// enable MCU register, (8051 enable)
		******************************/
			u16 valu16 = 0;
			rtw_write8(padapter, REG_MCUFWDL, 0);

			valu16 = rtw_read16(padapter, REG_SYS_FUNC_EN);
			rtw_write16(padapter, REG_SYS_FUNC_EN, (valu16 & (~FEN_CPUEN)));//reset MCU ,8051

			valu16 = rtw_read16(padapter, REG_SYS_FUNC_EN)&0x0FFF;
			rtw_write16(padapter, REG_SYS_FUNC_EN, (valu16 |(FEN_HWPDN|FEN_ELDR)));//reset MAC

			valu16 = rtw_read16(padapter, REG_SYS_FUNC_EN);
			rtw_write16(padapter, REG_SYS_FUNC_EN, (valu16 | FEN_CPUEN));//enable MCU ,8051
		#endif
	}
	else
	{
		u8 retry_cnts = 0;

		// 2010/08/12 MH For USB SS, we can not stop 8051 when we are trying to
		// enter IPS/HW&SW radio off. For S3/S4/S5/Disable, we can stop 8051 because
		// we will init FW when power on again.
		//if(!pDevice->RegUsbSS)
		{	// If we want to SS mode, we can not reset 8051.
			if(rtw_read8(padapter, REG_MCUFWDL) & BIT1)
			{ //IF fw in RAM code, do reset


				if(padapter->bFWReady)
				{
					// 2010/08/25 MH Accordign to RD alfred's suggestion, we need to disable other
					// HRCV INT to influence 8051 reset.
					rtw_write8(padapter, REG_FWIMR, 0x20);
					// 2011/02/15 MH According to Alex's suggestion, close mask to prevent incorrect FW write operation.
					rtw_write8(padapter, REG_FTIMR, 0x00);
					rtw_write8(padapter, REG_FSIMR, 0x00);

					rtw_write8(padapter, REG_HMETFR+3, 0x20);//8051 reset by self

					while( (retry_cnts++ <100) && (FEN_CPUEN &rtw_read16(padapter, REG_SYS_FUNC_EN)))
					{
						rtw_udelay_os(50);//us
						// 2010/08/25 For test only We keep on reset 5051 to prevent fail.
						//rtw_write8(padapter, REG_HMETFR+3, 0x20);//8051 reset by self
					}
//					RT_ASSERT((retry_cnts < 100), ("8051 reset failed!\n"));

					if (retry_cnts >= 100)
					{
						// if 8051 reset fail we trigger GPIO 0 for LA
						//rtw_write32(	padapter,
						//						REG_GPIO_PIN_CTRL,
						//						0x00010100);
						// 2010/08/31 MH According to Filen's info, if 8051 reset fail, reset MAC directly.
						rtw_write8(padapter, REG_SYS_FUNC_EN+1, 0x50);	//Reset MAC and Enable 8051
						rtw_mdelay_os(10);
					}
//					else
//					RT_TRACE(COMP_INIT, DBG_LOUD, ("=====> 8051 reset success (%d) .\n",retry_cnts));
				}
			}
//			else
//			{
//				RT_TRACE(COMP_INIT, DBG_LOUD, ("=====> 8051 in ROM.\n"));
//			}
			rtw_write8(padapter, REG_SYS_FUNC_EN+1, 0x54);	//Reset MAC and Enable 8051
			rtw_write8(padapter, REG_MCUFWDL, 0);
		}
	}

	//if(pDevice->RegUsbSS)
		//bWithoutHWSM = TRUE;	// Sugest by Filen and Issau.

	if(bWithoutHWSM)
	{
		//HAL_DATA_TYPE		*pHalData	= GET_HAL_DATA(padapter);
	/*****************************
		Without HW auto state machine
	g.	SYS_CLKR 0x08[15:0] = 0x30A3			//disable MAC clock
	h.	AFE_PLL_CTRL 0x28[7:0] = 0x80			//disable AFE PLL
	i.	AFE_XTAL_CTRL 0x24[15:0] = 0x880F		//gated AFE DIG_CLOCK
	j.	SYS_ISO_CTRL 0x00[7:0] = 0xF9			// isolated digital to PON
	******************************/
		//rtw_write16(padapter, REG_SYS_CLKR, 0x30A3);
		//if(!pDevice->RegUsbSS)
		// 2011/01/26 MH SD4 Scott suggest to fix UNC-B cut bug.
		//if (IS_81xxC_VENDOR_UMC_B_CUT(pHalData->VersionID))
			//rtw_write16(padapter, REG_SYS_CLKR, (0x70A3|BIT6));  //modify to 0x70A3 by Scott.
		//else
			rtw_write16(padapter, REG_SYS_CLKR, 0x70A3);  //modify to 0x70A3 by Scott.
		rtw_write8(padapter, REG_AFE_PLL_CTRL, 0x80);
		rtw_write16(padapter, REG_AFE_XTAL_CTRL, 0x880F);
		//if(!pDevice->RegUsbSS)
			rtw_write8(padapter, REG_SYS_ISO_CTRL, 0xF9);
	}
	else
	{
		// Disable all RF/BB power
		rtw_write8(padapter, REG_RF_CTRL, 0x00);
	}
//	RT_TRACE(COMP_INIT, DBG_LOUD, ("======> Reset Digital.\n"));

}

void _ResetDigitalProcedure1(PADAPTER padapter, BOOLEAN bWithoutHWSM)
{
#if 0
	if(IS_HARDWARE_TYPE_8192D(padapter))
		_ResetDigitalProcedure1_92D(padapter, bWithoutHWSM);
	else
#endif
		_ResetDigitalProcedure1_92C(padapter, bWithoutHWSM);
}

void _ResetDigitalProcedure2(PADAPTER padapter)
{
	//HAL_DATA_TYPE		*pHalData	= GET_HAL_DATA(padapter);
/*****************************
k.	SYS_FUNC_EN 0x03[7:0] = 0x44			// disable ELDR runction
l.	SYS_CLKR 0x08[15:0] = 0x3083			// disable ELDR clock
m.	SYS_ISO_CTRL 0x01[7:0] = 0x83			// isolated ELDR to PON
******************************/
	//rtw_write8(padapter, REG_SYS_FUNC_EN+1, 0x44); //marked by Scott.
	// 2011/01/26 MH SD4 Scott suggest to fix UNC-B cut bug.
	//if (IS_81xxC_VENDOR_UMC_B_CUT(pHalData->VersionID))
		//rtw_write16(padapter, REG_SYS_CLKR, 0x70a3|BIT6);
	//else
		rtw_write16(padapter, REG_SYS_CLKR, 0x70a3); //modify to 0x70a3 by Scott.
	rtw_write8(padapter, REG_SYS_ISO_CTRL+1, 0x82); //modify to 0x82 by Scott.
}

void _DisableAnalog(PADAPTER padapter, BOOLEAN bWithoutHWSM)
{
	HAL_DATA_TYPE	*pHalData	= GET_HAL_DATA(padapter);
	u16 value16 = 0;
	u8 value8 = 0;


	if (bWithoutHWSM)
	{
		/*****************************
		n.	LDOA15_CTRL 0x20[7:0] = 0x04		// disable A15 power
		o.	LDOV12D_CTRL 0x21[7:0] = 0x54		// disable digital core power
		r.	When driver call disable, the ASIC will turn off remaining clock automatically
		******************************/

		rtw_write8(padapter, REG_LDOA15_CTRL, 0x04);
		//rtw_write8(padapter, REG_LDOV12D_CTRL, 0x54);

		value8 = rtw_read8(padapter, REG_LDOV12D_CTRL);
		value8 &= (~LDV12_EN);
		rtw_write8(padapter, REG_LDOV12D_CTRL, value8);
//		RT_TRACE(COMP_INIT, DBG_LOUD, (" REG_LDOV12D_CTRL Reg0x21:0x%02x.\n",value8));
	}

	/*****************************
	h.	SPS0_CTRL 0x11[7:0] = 0x23			//enter PFM mode
	i.	APS_FSMCO 0x04[15:0] = 0x4802		// set USB suspend
	******************************/
	value8 = 0x23;
	if (IS_81xxC_VENDOR_UMC_B_CUT(pHalData->VersionID))
		value8 |= BIT3;

	rtw_write8(padapter, REG_SPS0_CTRL, value8);

	if(bWithoutHWSM)
	{
		//value16 |= (APDM_HOST | /*AFSM_HSUS |*/PFM_ALDN);
		// 2010/08/31 According to Filen description, we need to use HW to shut down 8051 automatically.
		// Becasue suspend operatione need the asistance of 8051 to wait for 3ms.
		value16 |= (APDM_HOST | AFSM_HSUS | PFM_ALDN);
	}
	else
	{
		value16 |= (APDM_HOST | AFSM_HSUS | PFM_ALDN);
	}

	rtw_write16(padapter, REG_APS_FSMCO, value16);//0x4802

	rtw_write8(padapter, REG_RSV_CTRL, 0x0e);

#if 0
	//tynli_test for suspend mode.
	if(!bWithoutHWSM){
		rtw_write8(padapter, 0xfe10, 0x19);
	}
#endif

//	RT_TRACE(COMP_INIT, DBG_LOUD, ("======> Disable Analog Reg0x04:0x%04x.\n",value16));
}

// HW Auto state machine
s32 CardDisableHWSM(PADAPTER padapter, u8 resetMCU)
{
	int rtStatus = _SUCCESS;


	if (padapter->bSurpriseRemoved){
		return rtStatus;
	}
	//==== RF Off Sequence ====
	_DisableRFAFEAndResetBB(padapter);

	//  ==== Reset digital sequence   ======
	_ResetDigitalProcedure1(padapter, _FALSE);

	//  ==== Pull GPIO PIN to balance level and LED control ======
	_DisableGPIO(padapter);

	//  ==== Disable analog sequence ===
	_DisableAnalog(padapter, _FALSE);

	RT_TRACE(_module_hci_hal_init_c_, _drv_info_, ("======> Card disable finished.\n"));

	return rtStatus;
}

// without HW Auto state machine
s32 CardDisableWithoutHWSM(PADAPTER padapter)
{
	s32 rtStatus = _SUCCESS;


	//RT_TRACE(COMP_INIT, DBG_LOUD, ("======> Card Disable Without HWSM .\n"));
	if (padapter->bSurpriseRemoved) {
		return rtStatus;
	}

	//==== RF Off Sequence ====
	_DisableRFAFEAndResetBB(padapter);

	//  ==== Reset digital sequence   ======
	_ResetDigitalProcedure1(padapter, _TRUE);

	//  ==== Pull GPIO PIN to balance level and LED control ======
	_DisableGPIO(padapter);

	//  ==== Reset digital sequence   ======
	_ResetDigitalProcedure2(padapter);

	//  ==== Disable analog sequence ===
	_DisableAnalog(padapter, _TRUE);

	//RT_TRACE(COMP_INIT, DBG_LOUD, ("<====== Card Disable Without HWSM .\n"));
	return rtStatus;
}

#endif

void
Hal_InitPGData(
	PADAPTER	padapter,
	u8			*PROMContent)
{
	EEPROM_EFUSE_PRIV *pEEPROM = GET_EEPROM_EFUSE_PRIV(padapter);
//	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	u32			i;
	u16			value16;

	if(_FALSE == pEEPROM->bautoload_fail_flag)
	{ // autoload OK.
//		if (IS_BOOT_FROM_EEPROM(padapter))
		if (_TRUE == pEEPROM->EepromOrEfuse)
		{
			// Read all Content from EEPROM or EFUSE.
			for(i = 0; i < HWSET_MAX_SIZE; i += 2)
			{
//				value16 = EF2Byte(ReadEEprom(pAdapter, (u2Byte) (i>>1)));
//				*((u16*)(&PROMContent[i])) = value16;
			}
		}
		else
		{
			// Read EFUSE real map to shadow.
			EFUSE_ShadowMapUpdate(padapter, EFUSE_WIFI, _FALSE);
			_rtw_memcpy((void*)PROMContent, (void*)pEEPROM->efuse_eeprom_data, HWSET_MAX_SIZE);
		}
	}
	else
	{//autoload fail
		RT_TRACE(_module_hci_hal_init_c_, _drv_notice_, ("AutoLoad Fail reported from CR9346!!\n"));
//		pHalData->AutoloadFailFlag = _TRUE;
		//update to default value 0xFF
		if (_FALSE == pEEPROM->EepromOrEfuse)
			EFUSE_ShadowMapUpdate(padapter, EFUSE_WIFI, _FALSE);
		_rtw_memcpy((void*)PROMContent, (void*)pEEPROM->efuse_eeprom_data, HWSET_MAX_SIZE);
	}
}

void
Hal_EfuseParseIDCode(
	IN	PADAPTER	padapter,
	IN	u8			*hwinfo
	)
{
	EEPROM_EFUSE_PRIV *pEEPROM = GET_EEPROM_EFUSE_PRIV(padapter);
//	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	u16			EEPROMId;


	// Checl 0x8129 again for making sure autoload status!!
	EEPROMId = le16_to_cpu(*((u16*)hwinfo));
	if (EEPROMId != RTL_EEPROM_ID)
	{
		DBG_8192C("EEPROM ID(%#x) is invalid!!\n", EEPROMId);
		pEEPROM->bautoload_fail_flag = _TRUE;
	}
	else
	{
		pEEPROM->bautoload_fail_flag = _FALSE;
	}

	RT_TRACE(_module_hal_init_c_, _drv_info_, ("EEPROM ID=0x%04x\n", EEPROMId));
}

static void
Hal_EEValueCheck(
	IN		u8		EEType,
	IN		PVOID		pInValue,
	OUT		PVOID		pOutValue
	)
{
	switch(EEType)
	{
		case EETYPE_TX_PWR:
			{
				u8	*pIn, *pOut;
				pIn = (u8*)pInValue;
				pOut = (u8*)pOutValue;
				if(*pIn >= 0 && *pIn <= 63)
				{
					*pOut = *pIn;
				}
				else
				{
					RT_TRACE(_module_hci_hal_init_c_, _drv_err_, ("EETYPE_TX_PWR, value=%d is invalid, set to default=0x%x\n",
						*pIn, EEPROM_Default_TxPowerLevel));
					*pOut = EEPROM_Default_TxPowerLevel;
				}
			}
			break;
		default:
			break;
	}
}

static void
Hal_ReadPowerValueFromPROM_8723A(
	IN	PTxPowerInfo	pwrInfo,
	IN	u8*			PROMContent,
	IN	BOOLEAN			AutoLoadFail
	)
{
	u32 rfPath, eeAddr, group, rfPathMax=1;

	_rtw_memset(pwrInfo, 0, sizeof(TxPowerInfo));

	if(AutoLoadFail)
	{
		for (group = 0; group < MAX_CHNL_GROUP; group++)
		{
			for(rfPath = 0 ; rfPath < rfPathMax ; rfPath++)
			{
				pwrInfo->CCKIndex[rfPath][group]		= EEPROM_Default_TxPowerLevel;
				pwrInfo->HT40_1SIndex[rfPath][group]	= EEPROM_Default_TxPowerLevel;
				pwrInfo->HT40_2SIndexDiff[rfPath][group]= EEPROM_Default_HT40_2SDiff;
				pwrInfo->HT20IndexDiff[rfPath][group]	= EEPROM_Default_HT20_Diff;
				pwrInfo->OFDMIndexDiff[rfPath][group]	= EEPROM_Default_LegacyHTTxPowerDiff;
				pwrInfo->HT40MaxOffset[rfPath][group]	= EEPROM_Default_HT40_PwrMaxOffset;
				pwrInfo->HT20MaxOffset[rfPath][group]	= EEPROM_Default_HT20_PwrMaxOffset;
			}
		}
		pwrInfo->TSSI_A[0] = EEPROM_Default_TSSI;
		return;
	}

	for(rfPath = 0 ; rfPath < rfPathMax ; rfPath++)
	{
		for (group = 0; group < MAX_CHNL_GROUP; group++)
		{
			eeAddr = EEPROM_CCK_TX_PWR_INX_8723A + (rfPath * 3) + group;
			//pwrInfo->CCKIndex[rfPath][group] = PROMContent[eeAddr];
			Hal_EEValueCheck(EETYPE_TX_PWR, &PROMContent[eeAddr], &pwrInfo->CCKIndex[rfPath][group]);
			eeAddr = EEPROM_HT40_1S_TX_PWR_INX_8723A + (rfPath * 3) + group;
			//pwrInfo->HT40_1SIndex[rfPath][group] = PROMContent[eeAddr];
			Hal_EEValueCheck(EETYPE_TX_PWR, &PROMContent[eeAddr], &pwrInfo->HT40_1SIndex[rfPath][group]);
		}
	}

	for (group = 0; group < MAX_CHNL_GROUP; group++)
	{
		for(rfPath = 0 ; rfPath < rfPathMax ; rfPath++)
		{
			pwrInfo->HT40_2SIndexDiff[rfPath][group] = 0;
			pwrInfo->HT20IndexDiff[rfPath][group] =
			(PROMContent[EEPROM_HT20_TX_PWR_INX_DIFF_8723A + group] >> (rfPath * 4)) & 0xF;
			if(pwrInfo->HT20IndexDiff[rfPath][group] & BIT3)	//4bit sign number to 8 bit sign number
				pwrInfo->HT20IndexDiff[rfPath][group] |= 0xF0;

			pwrInfo->OFDMIndexDiff[rfPath][group] =
			(PROMContent[EEPROM_OFDM_TX_PWR_INX_DIFF_8723A + group] >> (rfPath * 4)) & 0xF;

			pwrInfo->HT40MaxOffset[rfPath][group] =
			(PROMContent[EEPROM_HT40_MAX_PWR_OFFSET_8723A + group] >> (rfPath * 4)) & 0xF;

			pwrInfo->HT20MaxOffset[rfPath][group] =
			(PROMContent[EEPROM_HT20_MAX_PWR_OFFSET_8723A + group] >> (rfPath * 4)) & 0xF;
		}
	}

	pwrInfo->TSSI_A[0] = PROMContent[EEPROM_TSSI_A_8723A];
}

static u8
Hal_GetChnlGroup(
	IN	u8 chnl
	)
{
	u8	group=0;

	if (chnl < 3)			// Cjanel 1-3
		group = 0;
	else if (chnl < 9)		// Channel 4-9
		group = 1;
	else					// Channel 10-14
		group = 2;

	return group;
}

void
Hal_EfuseParseTxPowerInfo_8723A(
	IN	PADAPTER 		padapter,
	IN	u8*			PROMContent,
	IN	BOOLEAN			AutoLoadFail
	)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);
	TxPowerInfo		pwrInfo;
	u8			rfPath, ch, group, rfPathMax=1;
	u8			pwr, diff;

	Hal_ReadPowerValueFromPROM_8723A(&pwrInfo, PROMContent, AutoLoadFail);
	for(rfPath = 0 ; rfPath < rfPathMax ; rfPath++)
	{
		for(ch = 0 ; ch < CHANNEL_MAX_NUMBER ; ch++)
		{
			group = Hal_GetChnlGroup(ch);

			pHalData->TxPwrLevelCck[rfPath][ch] = pwrInfo.CCKIndex[rfPath][group];
			pHalData->TxPwrLevelHT40_1S[rfPath][ch] = pwrInfo.HT40_1SIndex[rfPath][group];

			pHalData->TxPwrHt20Diff[rfPath][ch] = pwrInfo.HT20IndexDiff[rfPath][group];
			pHalData->TxPwrLegacyHtDiff[rfPath][ch] = pwrInfo.OFDMIndexDiff[rfPath][group];
			pHalData->PwrGroupHT20[rfPath][ch] = pwrInfo.HT20MaxOffset[rfPath][group];
			pHalData->PwrGroupHT40[rfPath][ch] = pwrInfo.HT40MaxOffset[rfPath][group];

			pwr	= pwrInfo.HT40_1SIndex[rfPath][group];
			diff	= pwrInfo.HT40_2SIndexDiff[rfPath][group];

			pHalData->TxPwrLevelHT40_2S[rfPath][ch] = (pwr > diff) ? (pwr - diff) : 0;
		}
	}
#if 1
	for(rfPath = 0 ; rfPath < RF_PATH_MAX ; rfPath++)
	{
		for(ch = 0 ; ch < CHANNEL_MAX_NUMBER ; ch++)
		{
			RT_TRACE(_module_hci_hal_init_c_, _drv_info_,
				("RF(%u)-Ch(%u) [CCK / HT40_1S / HT40_2S] = [0x%x / 0x%x / 0x%x]\n",
				rfPath, ch, pHalData->TxPwrLevelCck[rfPath][ch],
				pHalData->TxPwrLevelHT40_1S[rfPath][ch],
				pHalData->TxPwrLevelHT40_2S[rfPath][ch]));

		}
	}
	for(ch = 0 ; ch < CHANNEL_MAX_NUMBER ; ch++)
	{
		RT_TRACE(_module_hci_hal_init_c_, _drv_info_, ("RF-A Ht20 to HT40 Diff[%u] = 0x%x(%d)\n", ch,
			pHalData->TxPwrHt20Diff[RF_PATH_A][ch], pHalData->TxPwrHt20Diff[RF_PATH_A][ch]));
	}
	for(ch = 0 ; ch < CHANNEL_MAX_NUMBER ; ch++)
	{
		RT_TRACE(_module_hci_hal_init_c_, _drv_info_, ("RF-A Legacy to Ht40 Diff[%u] = 0x%x\n", ch, pHalData->TxPwrLegacyHtDiff[RF_PATH_A][ch]));
	}
	for(ch = 0 ; ch < CHANNEL_MAX_NUMBER ; ch++)
	{
		RT_TRACE(_module_hci_hal_init_c_, _drv_info_, ("RF-B Ht20 to HT40 Diff[%u] = 0x%x(%d)\n", ch,
			pHalData->TxPwrHt20Diff[RF_PATH_B][ch], pHalData->TxPwrHt20Diff[RF_PATH_B][ch]));
	}
	for(ch = 0 ; ch < CHANNEL_MAX_NUMBER ; ch++)
	{
		RT_TRACE(_module_hci_hal_init_c_, _drv_info_, ("RF-B Legacy to HT40 Diff[%u] = 0x%x\n", ch, pHalData->TxPwrLegacyHtDiff[RF_PATH_B][ch]));
	}
#endif
	if(!AutoLoadFail)
	{
		pHalData->EEPROMRegulatory = PROMContent[RF_OPTION1_8723A]&0x7;	//bit0~2
	}
	else
	{
		pHalData->EEPROMRegulatory = 0;
	}
	RT_TRACE(_module_hci_hal_init_c_, _drv_info_, ("EEPROMRegulatory = 0x%x\n", pHalData->EEPROMRegulatory));

	if(!AutoLoadFail)
		pHalData->bTXPowerDataReadFromEEPORM = _TRUE;
}

VOID
Hal_EfuseParseBTCoexistInfo_8723A(
	IN PADAPTER			padapter,
	IN u8*			hwinfo,
	IN BOOLEAN			AutoLoadFail
	)
{
	PHAL_DATA_TYPE	pHalData = GET_HAL_DATA(padapter);
	u8			tempval;
	u32			tmpu4;

	if (!AutoLoadFail)
	{
		tmpu4 = rtw_read32(padapter, REG_MULTI_FUNC_CTRL);
		if (tmpu4 & BT_FUNC_EN)
			pHalData->EEPROMBluetoothCoexist = 1;
		else
			pHalData->EEPROMBluetoothCoexist = 0;
		pHalData->EEPROMBluetoothType = BT_RTL8723A;

		// The following need to be checked with newer version of 
		// eeprom spec
		tempval = hwinfo[RF_OPTION4_8723A];
		pHalData->EEPROMBluetoothAntNum = (tempval&0x1);					// bit [0]
		pHalData->EEPROMBluetoothAntIsolation = ((tempval&0x10)>>4);			// bit [4]
		pHalData->EEPROMBluetoothRadioShared = ((tempval&0x20)>>5); 		// bit [5]		
	}
	else
	{
		pHalData->EEPROMBluetoothCoexist = 0;
		pHalData->EEPROMBluetoothType = BT_RTL8723A;
		pHalData->EEPROMBluetoothAntNum = Ant_x2;
		pHalData->EEPROMBluetoothAntIsolation = 0;
		pHalData->EEPROMBluetoothRadioShared = BT_Radio_Shared;
	}
#ifdef CONFIG_BT_COEXIST
	BT_InitHalVars(padapter);
#endif
}

VOID
Hal_EfuseParseEEPROMVer(
	IN	PADAPTER		padapter,
	IN	u8*			hwinfo,
	IN	BOOLEAN			AutoLoadFail
	)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);

	if(!AutoLoadFail)
		pHalData->EEPROMVersion = hwinfo[EEPROM_VERSION_8723A];
	else
		pHalData->EEPROMVersion = 1;
	RT_TRACE(_module_hci_hal_init_c_, _drv_info_, ("Hal_EfuseParseEEPROMVer(), EEVer = %d\n",
		pHalData->EEPROMVersion));
}

VOID
Hal_EfuseParseChnlPlan(
	IN	PADAPTER		padapter,
	IN	u8*			hwinfo,
	IN	BOOLEAN			AutoLoadFail
	)
{
#if 0
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);

	if(!AutoLoadFail)
		pHalData->EEPROMChannelPlan = *(u8*)&hwinfo[EEPROM_ChannelPlan_8723];
	else
		pHalData->EEPROMChannelPlan = 0;

	RT_TRACE(_module_hci_hal_init_c_, _drv_info_, ("EEPROM ChannelPlan = 0x%4x\n", pHalData->EEPROMChannelPlan));
#else
	struct mlme_priv	*pmlmepriv = &padapter->mlmepriv;

	if (!AutoLoadFail)
		pmlmepriv->ChannelPlan = *(u8*)&hwinfo[EEPROM_ChannelPlan_8723A];
	else
		pmlmepriv->ChannelPlan = 0;

	RT_TRACE(_module_hci_hal_init_c_, _drv_info_, ("EEPROM ChannelPlan = 0x%4x\n", pmlmepriv->ChannelPlan));
#endif
}

VOID
Hal_EfuseParseCustomerID(
	IN	PADAPTER		padapter,
	IN	u8*			hwinfo,
	IN	BOOLEAN			AutoLoadFail
	)
{
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);

	if (!AutoLoadFail)
	{
		pHalData->EEPROMCustomerID = hwinfo[EEPROM_CustomID_8723A];
		pHalData->EEPROMSubCustomerID = hwinfo[EEPROM_SubCustomID_8723A];
	}
	else
	{
		pHalData->EEPROMCustomerID = 0;
		pHalData->EEPROMSubCustomerID = 0;
	}
	RT_TRACE(_module_hci_hal_init_c_, _drv_info_, ("EEPROM Customer ID: 0x%2x\n", pHalData->EEPROMCustomerID));
	RT_TRACE(_module_hci_hal_init_c_, _drv_info_, ("EEPROM SubCustomer ID: 0x%02x\n", pHalData->EEPROMSubCustomerID));
}

VOID
Hal_EfuseParseAntennaDiversity(
	IN	PADAPTER		padapter,
	IN	u8*			hwinfo,
	IN	BOOLEAN			AutoLoadFail
	)
{
#if 0
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);

	if(!AutoLoadFail)
	{
		// Antenna Diversity setting.
		if(GetRegAntDiv(padapter) == 2)
			pHalData->AntDivCfg = (hwinfo[RF_OPTION1_8723A]&0x18)>>3;
		else
			pHalData->AntDivCfg = GetRegAntDiv(padapter);

		if(pHalData->EEPROMBluetoothCoexist!=0 && pHalData->EEPROMBluetoothAntNum==Ant_x1)
			pHalData->AntDivCfg = 0;
	}
	else
	{
		pHalData->AntDivCfg = 0;
	}

	RT_TRACE(_module_hci_hal_init_c_, _drv_info_, ("SWAS: bHwAntDiv = %x\n", pHalData->AntDivCfg));
#endif
}

VOID
Hal_EfuseParseRateIndicationOption(
	IN	PADAPTER		padapter,
	IN	u8*			hwinfo,
	IN	BOOLEAN			AutoLoadFail
	)
{
#if 0
	PMGNT_INFO		pMgntInfo = &(padapter->MgntInfo);

	// Rate indication option
	if(pMgntInfo->ShowRateMode == 0)
	{
		if(!AutoLoadFail)
		{
			switch((hwinfo[RF_OPTION3_8723A] & 0x0c) >> 2)
			{
				case 1: // Rx rate
					pMgntInfo->bForcedShowRxRate = TRUE;
					break;

				case 2: // Max Rx rate
					pMgntInfo->bForcedShowRateStill = TRUE;
					pMgntInfo->bForcedShowRxRate = TRUE;
					break;

				default:
					break;
			}
		}
		else
		{
			pMgntInfo->bForcedShowRxRate = TRUE;
		}
	}
	else if(pMgntInfo->ShowRateMode == 2)
	{
		pMgntInfo->bForcedShowRxRate = TRUE;
	}
	else if(pMgntInfo->ShowRateMode == 3)
	{
		pMgntInfo->bForcedShowRxRate = TRUE;
		pMgntInfo->bForcedShowRxRate = TRUE;
	}
#endif
}

void
Hal_EfuseParseXtal_8723A(
	PADAPTER		pAdapter,
	u8			*hwinfo,
	u8			AutoLoadFail
	)
{
#if 0
	PHAL_DATA_TYPE	pHalData = GET_HAL_DATA(pAdapter);

	if (!AutoLoadFail)
		pHalData->CrystalCap = hwinfo[EEPROM_XTAL_K_8723A];
	else
		pHalData->CrystalCap = EEPROM_Default_CrystalCap_8723A;
	RT_TRACE(_module_hci_hal_init_c_, _drv_info_, ("%s: CrystalCap=0x%2x\n", __FUNCTION__, pHalData->CrystalCap));
#endif
}

VOID
Hal_InitChannelPlan(
	IN		PADAPTER	padapter
	)
{
#if 0
	PMGNT_INFO		pMgntInfo = &(padapter->MgntInfo);
	HAL_DATA_TYPE	*pHalData = GET_HAL_DATA(padapter);

	if((pMgntInfo->RegChannelPlan >= RT_CHANNEL_DOMAIN_MAX) || (pHalData->EEPROMChannelPlan & EEPROM_CHANNEL_PLAN_BY_HW_MASK))
	{
		pMgntInfo->ChannelPlan = hal_MapChannelPlan8192C(padapter, (pHalData->EEPROMChannelPlan & (~(EEPROM_CHANNEL_PLAN_BY_HW_MASK))));
		pMgntInfo->bChnlPlanFromHW = (pHalData->EEPROMChannelPlan & EEPROM_CHANNEL_PLAN_BY_HW_MASK) ? _TRUE : _FALSE; // User cannot change  channel plan.
	}
	else
	{
		pMgntInfo->ChannelPlan = (RT_CHANNEL_DOMAIN)pMgntInfo->RegChannelPlan;
	}

	switch(pMgntInfo->ChannelPlan)
	{
		case RT_CHANNEL_DOMAIN_GLOBAL_DOAMIN:
		{
			PRT_DOT11D_INFO	pDot11dInfo = GET_DOT11D_INFO(pMgntInfo);

			pDot11dInfo->bEnabled = TRUE;
		}
			RT_TRACE(_module_hci_hal_init_c_, _drv_info_, ("ReadAdapterInfo8187(): Enable dot11d when RT_CHANNEL_DOMAIN_GLOBAL_DOAMIN!\n"));
			break;

		default: //for MacOSX compiler warning.
			break;
	}

	RT_TRACE(_module_hci_hal_init_c_, _drv_info_, ("RegChannelPlan(%d) EEPROMChannelPlan(%d)", pMgntInfo->RegChannelPlan, pHalData->EEPROMChannelPlan));
	RT_TRACE(_module_hci_hal_init_c_, _drv_info_, ("Mgnt ChannelPlan = %d\n" , pMgntInfo->ChannelPlan));
#endif
}

void rtl8723a_cal_txdesc_chksum(struct tx_desc *ptxdesc)
{
	u16	*usPtr = (u16*)ptxdesc;
	u32 count = 16;		// (32 bytes / 2 bytes per XOR) => 16 times
	u32 index;
	u16 checksum = 0;


	// Clear first
	ptxdesc->txdw7 &= cpu_to_le32(0xffff0000);

	for (index = 0; index < count; index++) {
		checksum ^= le16_to_cpu(*(usPtr + index));
	}

	ptxdesc->txdw7 |= cpu_to_le32(checksum & 0x0000ffff);
}

static void fill_txdesc_sectype(struct pkt_attrib *pattrib, PTXDESC ptxdesc)
{
	if ((pattrib->encrypt > 0) && !pattrib->bswenc)
	{
		switch (pattrib->encrypt)
		{
			// SEC_TYPE
			case _WEP40_:
			case _WEP104_:
			case _TKIP_:
			case _TKIP_WTMIC_:
				ptxdesc->sectype = 1;
				break;

			case _AES_:
				ptxdesc->sectype = 3;
				break;

			case _NO_PRIVACY_:
			default:
					break;
		}
	}
}

static void fill_txdesc_vcs(struct pkt_attrib *pattrib, PTXDESC ptxdesc)
{
	//DBG_8192C("cvs_mode=%d\n", pattrib->vcs_mode);

	switch (pattrib->vcs_mode)
	{
		case RTS_CTS:
			ptxdesc->rtsen = 1;
			break;

		case CTS_TO_SELF:
			ptxdesc->cts2self = 1;
			break;

		case NONE_VCS:
		default:
			break;
	}

	if (pattrib->vcs_mode)
		ptxdesc->hw_rts_en = 1; // ENABLE HW RTS
}

static void fill_txdesc_phy(struct pkt_attrib *pattrib, PTXDESC ptxdesc)
{
	//DBG_8192C("bwmode=%d, ch_off=%d\n", pattrib->bwmode, pattrib->ch_offset);

	if (pattrib->ht_en)
	{
		if (pattrib->bwmode & HT_CHANNEL_WIDTH_40)
			ptxdesc->data_bw = 1;

		switch (pattrib->ch_offset)
		{
			case HAL_PRIME_CHNL_OFFSET_DONT_CARE:
				ptxdesc->data_sc = 0;
				break;

			case HAL_PRIME_CHNL_OFFSET_LOWER:
				ptxdesc->data_sc = 1;
				break;

			case HAL_PRIME_CHNL_OFFSET_UPPER:
				ptxdesc->data_sc = 2;
				break;

			default:
				ptxdesc->data_sc = 3; // Duplicate
				break;
		}
	}
}

void rtl8723a_fill_default_txdesc(
	struct xmit_frame *pxmitframe,
	u8 *pbuf)
{
	PADAPTER padapter;
	HAL_DATA_TYPE *pHalData;
	struct dm_priv *pdmpriv;
	struct pkt_attrib *pattrib;
	PTXDESC ptxdesc;
	s32 bmcst;
#ifdef CONFIG_P2P
	struct wifidirect_info* pwdinfo;
#endif //CONFIG_P2P


	padapter = pxmitframe->padapter;
	pHalData = GET_HAL_DATA(padapter);
	pdmpriv = &pHalData->dmpriv;
#ifdef CONFIG_P2P
	pwdinfo = &padapter->wdinfo;
#endif //CONFIG_P2P

	pattrib = &pxmitframe->attrib;
	bmcst = IS_MCAST(pattrib->ra);

	ptxdesc = (PTXDESC)pbuf;

	if (pxmitframe->frame_tag == DATA_FRAMETAG)
	{
		ptxdesc->macid = pattrib->mac_id; // CAM_ID(MAC_ID)

		if (pattrib->ampdu_en == _TRUE)
			ptxdesc->agg_en = 1; // AGG EN
		else
			ptxdesc->bk = 1; // AGG BK

		ptxdesc->qsel = pattrib->qsel;
		ptxdesc->rate_id = pattrib->raid;

		fill_txdesc_sectype(pattrib, ptxdesc);

		ptxdesc->seq = pattrib->seqnum;

		if ((pattrib->ether_type != 0x888e) &&
			(pattrib->ether_type != 0x0806) &&
			(pattrib->dhcp_pkt != 1))
		{
			// Non EAP & ARP & DHCP type data packet

			fill_txdesc_vcs(pattrib, ptxdesc);
			fill_txdesc_phy(pattrib, ptxdesc);

			ptxdesc->rtsrate = 8; // RTS Rate=24M
			ptxdesc->data_ratefb_lmt = 0x1F;
			ptxdesc->rts_ratefb_lmt = 0xF;

			// use REG_INIDATA_RATE_SEL value
			ptxdesc->datarate = pdmpriv->INIDATA_RATE[pattrib->mac_id];

#if 0
			ptxdesc->userate = 1; // driver uses rate

			if (pattrib->ht_en)
				ptxdesc->sgi = 1; // SGI

			ptxdesc->datarate = 0x13; // init rate - mcs7
#endif
		}
		else
		{
			// EAP data packet and ARP packet.
			// Use the 1M data rate to send the EAP/ARP packet.
			// This will maybe make the handshake smooth.

			ptxdesc->bk = 1; // AGG BK
			ptxdesc->userate = 1; // driver uses rate

#ifdef CONFIG_P2P
			//	Added by Albert 2011/03/22
			//	In the P2P mode, the driver should not support the b mode.
			//	So, the Tx packet shouldn't use the CCK rate
			if(!rtw_p2p_chk_state(pwdinfo, P2P_STATE_NONE))
			{
				ptxdesc->datarate = 0x4; // Use the 6M data rate.
			}
#endif //CONFIG_P2P
		}
#if defined(CONFIG_USB_TX_AGGREGATION) || defined(CONFIG_SDIO_HCI)
		ptxdesc->usb_txagg_num = pxmitframe->agg_num;
#endif
	}
	else if (pxmitframe->frame_tag == MGNT_FRAMETAG)
	{
//		RT_TRACE(_module_hal_xmit_c_, _drv_notice_, ("%s: MGNT_FRAMETAG\n", __FUNCTION__));

		ptxdesc->macid = pattrib->mac_id; // CAM_ID(MAC_ID)
		ptxdesc->qsel = pattrib->qsel;
		ptxdesc->rate_id = pattrib->raid; // Rate ID
		ptxdesc->seq = pattrib->seqnum;
		ptxdesc->userate = 1; // driver uses rate, 1M
		ptxdesc->rty_lmt_en = 1; // retry limit enable
		ptxdesc->data_rt_lmt = 6; // retry limit = 6

#ifdef CONFIG_P2P
		//	Added by Albert 2011/03/17
		//	In the P2P mode, the driver should not support the b mode.
		//	So, the Tx packet shouldn't use the CCK rate
		if(!rtw_p2p_chk_state(pwdinfo, P2P_STATE_NONE))
		{
			ptxdesc->datarate = 0x4; // Use the 6M data rate.
		}
#endif //CONFIG_P2P
	}
	else if (pxmitframe->frame_tag == TXAGG_FRAMETAG)
	{
		RT_TRACE(_module_hal_xmit_c_, _drv_warning_, ("%s: TXAGG_FRAMETAG\n", __FUNCTION__));
	}
#ifdef CONFIG_MP_INCLUDED
	else if (pxmitframe->frame_tag == MP_FRAMETAG)
	{
		struct tx_desc *pdesc;

		pdesc = (struct tx_desc*)ptxdesc;
		RT_TRACE(_module_hal_xmit_c_, _drv_notice_, ("%s: MP_FRAMETAG\n", __FUNCTION__));
		fill_txdesc_for_mp(padapter, pdesc);

		pdesc->txdw0 = le32_to_cpu(pdesc->txdw0);
		pdesc->txdw1 = le32_to_cpu(pdesc->txdw1);
		pdesc->txdw2 = le32_to_cpu(pdesc->txdw2);
		pdesc->txdw3 = le32_to_cpu(pdesc->txdw3);
		pdesc->txdw4 = le32_to_cpu(pdesc->txdw4);
		pdesc->txdw5 = le32_to_cpu(pdesc->txdw5);
		pdesc->txdw6 = le32_to_cpu(pdesc->txdw6);
		pdesc->txdw7 = le32_to_cpu(pdesc->txdw7);
#ifdef CONFIG_PCI_HCI
		pdesc->txdw8 = le32_to_cpu(pdesc->txdw8);
		pdesc->txdw9 = le32_to_cpu(pdesc->txdw9);
		pdesc->txdw10 = le32_to_cpu(pdesc->txdw10);
		pdesc->txdw11 = le32_to_cpu(pdesc->txdw11);
		pdesc->txdw12 = le32_to_cpu(pdesc->txdw12);
		pdesc->txdw13 = le32_to_cpu(pdesc->txdw13);
		pdesc->txdw14 = le32_to_cpu(pdesc->txdw14);
		pdesc->txdw15 = le32_to_cpu(pdesc->txdw15);
#endif
	}
#endif
	else
	{
		RT_TRACE(_module_hal_xmit_c_, _drv_warning_, ("%s: frame_tag=0x%x\n", __FUNCTION__, pxmitframe->frame_tag));

		ptxdesc->macid = 4; // CAM_ID(MAC_ID)
		ptxdesc->rate_id = 6; // Rate ID
		ptxdesc->seq = pattrib->seqnum;
		ptxdesc->userate = 1; // driver uses rate, 1M
	}

	ptxdesc->pktlen = pattrib->last_txcmdsz;
	ptxdesc->offset = TXDESC_SIZE + OFFSET_SZ;
	if (bmcst) ptxdesc->bmc = 1;
	ptxdesc->ls = 1;
	ptxdesc->fs = 1;
	ptxdesc->own = 1;

	// 2009.11.05. tynli_test. Suggested by SD4 Filen for FW LPS.
	// (1) The sequence number of each non-Qos frame / broadcast / multicast /
	// mgnt frame should be controled by Hw because Fw will also send null data
	// which we cannot control when Fw LPS enable.
	// --> default enable non-Qos data sequense number. 2010.06.23. by tynli.
	// (2) Enable HW SEQ control for beacon packet, because we use Hw beacon.
	// (3) Use HW Qos SEQ to control the seq num of Ext port non-Qos packets.
	// 2010.06.23. Added by tynli.
	if (!pattrib->qos_en)
	{
		// Hw set sequence number
		ptxdesc->hwseq_en = 1; // HWSEQ_EN
		ptxdesc->hwseq_sel = 0; // HWSEQ_SEL
	}
}

/*
 *	Description:
 *
 *	Parameters:
 *		pxmitframe	xmitframe
 *		pbuf		where to fill tx desc
 */
void rtl8723a_update_txdesc(struct xmit_frame *pxmitframe, u8 *pbuf)
{
	struct tx_desc *pdesc;


	pdesc = (struct tx_desc*)pbuf;
	_rtw_memset(pdesc, 0, sizeof(struct tx_desc));

	rtl8723a_fill_default_txdesc(pxmitframe, pbuf);

	pdesc->txdw0 = cpu_to_le32(pdesc->txdw0);
	pdesc->txdw1 = cpu_to_le32(pdesc->txdw1);
	pdesc->txdw2 = cpu_to_le32(pdesc->txdw2);
	pdesc->txdw3 = cpu_to_le32(pdesc->txdw3);
	pdesc->txdw4 = cpu_to_le32(pdesc->txdw4);
	pdesc->txdw5 = cpu_to_le32(pdesc->txdw5);
	pdesc->txdw6 = cpu_to_le32(pdesc->txdw6);
	pdesc->txdw7 = cpu_to_le32(pdesc->txdw7);
#ifdef CONFIG_PCI_HCI
	pdesc->txdw8 = cpu_to_le32(pdesc->txdw8);
	pdesc->txdw9 = cpu_to_le32(pdesc->txdw9);
	pdesc->txdw10 = cpu_to_le32(pdesc->txdw10);
	pdesc->txdw11 = cpu_to_le32(pdesc->txdw11);
	pdesc->txdw12 = cpu_to_le32(pdesc->txdw12);
	pdesc->txdw13 = cpu_to_le32(pdesc->txdw13);
	pdesc->txdw14 = cpu_to_le32(pdesc->txdw14);
	pdesc->txdw15 = cpu_to_le32(pdesc->txdw15);
#endif

	rtl8723a_cal_txdesc_chksum(pdesc);
}

//
// Description: In normal chip, we should send some packet to Hw which will be used by Fw
//			in FW LPS mode. The function is to fill the Tx descriptor of this packets, then
//			Fw can tell Hw to send these packet derectly.
// Added by tynli. 2009.10.15.
//
void rtl8723a_fill_fake_txdesc(
	PADAPTER	padapter,
	u8*			pDesc,
	u32			BufferLen,
	u8			IsPsPoll,
	u8			IsBTQosNull)
{
	struct tx_desc *ptxdesc;


	// Clear all status
	ptxdesc = (struct tx_desc*)pDesc;
	_rtw_memset(pDesc, 0, TXDESC_SIZE);

	//offset 0
	ptxdesc->txdw0 |= cpu_to_le32( OWN | FSG | LSG); //own, bFirstSeg, bLastSeg;

	ptxdesc->txdw0 |= cpu_to_le32(((TXDESC_SIZE+OFFSET_SZ)<<OFFSET_SHT)&0x00ff0000); //32 bytes for TX Desc

	ptxdesc->txdw0 |= cpu_to_le32(BufferLen&0x0000ffff); // Buffer size + command header

	//offset 4
	ptxdesc->txdw1 |= cpu_to_le32((QSLT_MGNT<<QSEL_SHT)&0x00001f00); // Fixed queue of Mgnt queue

	//Set NAVUSEHDR to prevent Ps-poll AId filed to be changed to error vlaue by Hw.
	if (IsPsPoll)
	{
		ptxdesc->txdw1 |= cpu_to_le32(NAVUSEHDR);
	}
	else
	{
		ptxdesc->txdw4 |= cpu_to_le32(BIT(7)); // Hw set sequence number
		ptxdesc->txdw3 |= cpu_to_le32((8 <<28)); //set bit3 to 1. Suugested by TimChen. 2009.12.29.
	}

	if (_TRUE == IsBTQosNull)
	{
		ptxdesc->txdw2 |= cpu_to_le32(BIT(23)); // BT NULL
	}

	//offset 16
	ptxdesc->txdw4 |= cpu_to_le32(BIT(8));//driver uses rate

#if defined(CONFIG_USB_HCI) || defined(CONFIG_SDIO_HCI)
	// USB interface drop packet if the checksum of descriptor isn't correct.
	// Using this checksum can let hardware recovery from packet bulk out error (e.g. Cancel URC, Bulk out error.).
	rtl8723a_cal_txdesc_chksum(ptxdesc);
#endif
}

static void hw_var_set_opmode(PADAPTER padapter, u8 variable, u8 *val)
{
	u8 val8;
	u8 mode = *val;
	PHAL_DATA_TYPE pHalData = GET_HAL_DATA(padapter);


	if ((mode == _HW_STATE_STATION_) || (mode == _HW_STATE_NOLINK_))
	{
#ifdef CONFIG_CONCURRENT_MODE
		PADAPTER pbuddy_adapter = padapter->pbuddy_adapter;
		struct mlme_ext_priv *pbuddy_mlmeext = &pbuddy_adapter->mlmeextpriv;
		struct mlme_ext_info *pbuddy_mlmeinfo = &(pbuddy_mlmeext->mlmext_info);

		if ((pbuddy_mlmeinfo->state&0x03) != WIFI_FW_AP_STATE)
#endif
		{
			StopTxBeacon(padapter);
		}

		// disable atim wnd
		val8 = DIS_TSF_UDT|EN_BCN_FUNCTION|DIS_ATIM;
		SetBcnCtrlReg(padapter, val8, ~val8);
	}
	else if ((mode == _HW_STATE_ADHOC_) /*|| (mode == _HW_STATE_AP_)*/)
	{
		ResumeTxBeacon(padapter);

		val8 = DIS_TSF_UDT|EN_BCN_FUNCTION|DIS_BCNQ_SUB;
		SetBcnCtrlReg(padapter, val8, ~val8);
	}
	else if (mode == _HW_STATE_AP_)
	{
		ResumeTxBeacon(padapter);

		val8 = DIS_TSF_UDT|DIS_BCNQ_SUB;
		SetBcnCtrlReg(padapter, val8, ~val8);

		// Set RCR
		//rtw_write32(padapter, REG_RCR, 0x70002a8e);//CBSSID_DATA must set to 0
		rtw_write32(padapter, REG_RCR, 0x7000228e);//CBSSID_DATA must set to 0
		// enable to rx data frame
		rtw_write16(padapter, REG_RXFLTMAP2, 0xFFFF);
		// enable to rx ps-poll
		rtw_write16(padapter, REG_RXFLTMAP1, 0x0400);

		// Beacon Control related register for first time
		rtw_write8(padapter, REG_BCNDMATIM, 0x02); // 2ms
		rtw_write8(padapter, REG_DRVERLYINT, 0x05); // 5ms
		//rtw_write8(padapter, REG_BCN_MAX_ERR, 0xFF);
#ifdef CONFIG_CONCURRENT_MODE
		if (padapter->iface_type == IFACE_PORT1)
			rtw_write8(padapter, REG_ATIMWND_1, 0x0a); // 10ms for port1
		else
#endif
		{
			rtw_write8(padapter, REG_ATIMWND, 0x0a); // 10ms for port0
		}
		rtw_write16(padapter, REG_BCNTCFG, 0x00);
		rtw_write16(padapter, REG_TBTT_PROHIBIT, 0xff04);
		rtw_write16(padapter, REG_TSFTR_SYN_OFFSET, 0x7fff);// +32767 (~32ms)

		// reset TSF
#ifdef CONFIG_CONCURRENT_MODE
		if (padapter->iface_type == IFACE_PORT1)
			rtw_write8(padapter, REG_DUAL_TSF_RST, BIT(1));
		else
#endif
		{
			rtw_write8(padapter, REG_DUAL_TSF_RST, BIT(0));
		}

		// enable BCN Function
		// don't enable update TSF (due to TSF update when beacon/probe rsp are received)
		val8 = DIS_TSF_UDT | EN_BCN_FUNCTION | EN_TXBCN_RPT | DIS_BCNQ_SUB;
		SetBcnCtrlReg(padapter, val8, ~val8);

		// dis BCN ATIM WND of another port if it is station
#ifdef CONFIG_CONCURRENT_MODE
		SetBcnCtrlReg(padapter->pbuddy_adapter, DIS_ATIM, 0);
#else
//		val8 = rtw_read8(padapter, REG_BCN_CTRL_1);
//		val8 |= DIS_ATIM;
//		rtw_write8(padapter, REG_BCN_CTRL_1, val8);
#endif
	}

	val8 = rtw_read8(padapter, MSR);
#ifdef CONFIG_CONCURRENT_MODE
	if (padapter->iface_type == IFACE_PORT1)
		val8 = (val8 & 0x3) | (mode << 2);
	else
#endif
	{
		val8 = (val8 & 0xC) | mode;
	}
	rtw_write8(padapter, MSR, val8);
}

static void hw_var_set_macaddr(PADAPTER padapter, u8 variable, u8 *val)
{
	u8 idx = 0;
	u32 reg_macid;

#ifdef CONFIG_CONCURRENT_MODE
	if (padapter->iface_type == IFACE_PORT1)
	{
		reg_macid = REG_MACID1;
	}
	else
#endif
	{
		reg_macid = REG_MACID;
	}

	for (idx = 0 ; idx < 6; idx++)
	{
		rtw_write8(padapter, (reg_macid+idx), val[idx]);
	}
}

static void hw_var_set_bssid(PADAPTER padapter, u8 variable, u8 *val)
{
	u8	idx = 0;
	u32 reg_bssid;

#ifdef CONFIG_CONCURRENT_MODE
	if (padapter->iface_type == IFACE_PORT1)
	{
		reg_bssid = REG_BSSID1;
	}
	else
#endif
	{
		reg_bssid = REG_BSSID;
	}

	for (idx = 0 ; idx < 6; idx++)
	{
		rtw_write8(padapter, (reg_bssid+idx), val[idx]);
	}
}

static void hw_var_set_correct_tsf(PADAPTER padapter, u8 variable, u8 *val)
{
	u64 tsf;
	u32 reg_tsftr;
	struct mlme_ext_priv *pmlmeext = &padapter->mlmeextpriv;
	struct mlme_ext_info *pmlmeinfo = &pmlmeext->mlmext_info;


	//tsf = pmlmeext->TSFValue - ((u32)pmlmeext->TSFValue % (pmlmeinfo->bcn_interval*1024)) - 1024; //us
	tsf = pmlmeext->TSFValue - rtw_modular64(pmlmeext->TSFValue, (pmlmeinfo->bcn_interval*1024)) - 1024; //us

	if (((pmlmeinfo->state&0x03) == WIFI_FW_ADHOC_STATE) ||
		((pmlmeinfo->state&0x03) == WIFI_FW_AP_STATE))
	{
		//pHalData->RegTxPause |= STOP_BCNQ;BIT(6)
		//rtw_write8(padapter, REG_TXPAUSE, (rtw_read8(Adapter, REG_TXPAUSE)|BIT(6)));
		StopTxBeacon(padapter);
	}

#ifdef CONFIG_CONCURRENT_MODE
	if (padapter->iface_type == IFACE_PORT1)
	{
		reg_tsftr = REG_TSFTR1;
	}
	else
#endif
	{
		reg_tsftr = REG_TSFTR;
	}

	// disable related TSF function
	SetBcnCtrlReg(padapter, 0, EN_BCN_FUNCTION);

	rtw_write32(padapter, reg_tsftr, tsf);
	rtw_write32(padapter, reg_tsftr+4, tsf>>32);

#ifdef CONFIG_CONCURRENT_MODE

	// Update buddy port's TSF if it is SoftAP for beacon TX issue!
	if ( (pmlmeinfo->state&0x03) == WIFI_FW_STATION_STATE
		&& check_fwstate(&padapter->pbuddy_adapter->mlmepriv, WIFI_AP_STATE)
	) { 
		//disable related TSF function
		SetBcnCtrlReg(padapter->pbuddy_adapter, 0, EN_BCN_FUNCTION);
			if (padapter->iface_type == IFACE_PORT1)
		{
			reg_tsftr = REG_TSFTR;
		}
		else
		{
			reg_tsftr = REG_TSFTR1;
		}
		
		rtw_write32(padapter, reg_tsftr, tsf);
		rtw_write32(padapter, reg_tsftr+4, tsf>>32);

		//enable related TSF function
		SetBcnCtrlReg(padapter->pbuddy_adapter,  EN_BCN_FUNCTION,0);
	}		
#endif
	//enable related TSF function
	SetBcnCtrlReg(padapter, EN_BCN_FUNCTION, 0);

	if (((pmlmeinfo->state&0x03) == WIFI_FW_ADHOC_STATE) ||
		((pmlmeinfo->state&0x03) == WIFI_FW_AP_STATE))
	{
		//pHalData->RegTxPause &= (~STOP_BCNQ);
		//rtw_write8(padapter, REG_TXPAUSE, (rtw_read8(padapter, REG_TXPAUSE)&(~BIT(6))));
		ResumeTxBeacon(padapter);
	}
}

static void hw_var_set_mlme_disconnect(PADAPTER padapter, u8 variable, u8 *val)
{
	PHAL_DATA_TYPE pHalData = GET_HAL_DATA(padapter);
#ifdef CONFIG_CONCURRENT_MODE
	PADAPTER pbuddy_adapter = padapter->pbuddy_adapter;
	struct mlme_ext_priv *pbuddy_mlmeext = &pbuddy_adapter->mlmeextpriv;
	struct mlme_ext_info *pbuddy_mlmeinfo = &pbuddy_mlmeext->mlmext_info;


	if ((pbuddy_mlmeinfo->state&0x03) == _HW_STATE_NOLINK_)
#endif
	{
		// Set RCR to not to receive data frame when NO LINK state
		//rtw_write32(padapter, REG_RCR, rtw_read32(padapter, REG_RCR) & ~RCR_ADF);
		// reject all data frames
		rtw_write16(padapter, REG_RXFLTMAP2, 0);
	}

#ifdef CONFIG_CONCURRENT_MODE
	if (padapter->iface_type == IFACE_PORT1)
	{
		// reset TSF1
		rtw_write8(padapter, REG_DUAL_TSF_RST, BIT(1));

		// disable update TSF1
		SetBcnCtrlReg(padapter, DIS_TSF_UDT, 0);
	}
	else
#endif
	{
		// reset TSF
		rtw_write8(padapter, REG_DUAL_TSF_RST, BIT(0));

		// disable update TSF
		SetBcnCtrlReg(padapter, DIS_TSF_UDT, 0);
	}
}

#ifdef CONFIG_CONCURRENT_MODE
static void hw_var_set_mlme_sitesurvey(PADAPTER padapter, u8 variable, u8 *val)
{
	PHAL_DATA_TYPE pHalData = GET_HAL_DATA(padapter);

	struct mlme_ext_priv *pmlmeext = &padapter->mlmeextpriv;
	struct mlme_ext_info *pmlmeinfo = &(pmlmeext->mlmext_info);

	PADAPTER pbuddy_adapter = padapter->pbuddy_adapter;
	struct mlme_priv *pbuddy_mlmepriv = &(pbuddy_adapter->mlmepriv);
	struct mlme_ext_priv *pbuddy_mlmeext = &pbuddy_adapter->mlmeextpriv;
	struct mlme_ext_info *pbuddy_mlmeinfo = &(pbuddy_mlmeext->mlmext_info);

	u32 v32;


	if (*val)//under sitesurvey
	{
		// config RCR to receive different BSSID & not to receive data frame
		//pHalData->ReceiveConfig &= (~(RCR_CBSSID_DATA | RCR_CBSSID_BCN));			
		v32 = rtw_read32(padapter, REG_RCR);
		v32 &= ~(RCR_CBSSID_DATA | RCR_CBSSID_BCN);//| RCR_ADF
		rtw_write32(padapter, REG_RCR, v32);

		// disable update TSF
		if ((pmlmeinfo->state&0x03) == WIFI_FW_STATION_STATE)
			SetBcnCtrlReg(padapter, DIS_TSF_UDT, 0);

		if (((pbuddy_mlmeinfo->state&0x03) == WIFI_FW_AP_STATE) &&
			(check_fwstate(pbuddy_mlmepriv, _FW_LINKED) == _TRUE))
		{
			StopTxBeacon(padapter);
		}
	}
	else//sitesurvey done
	{
		// enable to rx data frame
		//write32(padapter, REG_RCR, read32(padapter, REG_RCR)|RCR_ADF);
		rtw_write16(padapter, REG_RXFLTMAP2, 0xFFFF);

		// enable update TSF
		SetBcnCtrlReg(padapter, 0, DIS_TSF_UDT);

		v32 = rtw_read32(padapter, REG_RCR);
		if ((pmlmeinfo->state&0x03) == WIFI_FW_AP_STATE ||
			(pbuddy_mlmeinfo->state&0x03) == WIFI_FW_AP_STATE)
			v32 |= RCR_CBSSID_BCN;
		else
			v32 |= RCR_CBSSID_DATA|RCR_CBSSID_BCN;
		rtw_write32(padapter, REG_RCR, v32);

		if (((pbuddy_mlmeinfo->state&0x03) == WIFI_FW_AP_STATE) &&
			(check_fwstate(pbuddy_mlmepriv, _FW_LINKED) == _TRUE))
		{
			ResumeTxBeacon(padapter);
#if 0
			// reset TSF 1/2 after ResumeTxBeacon
			if (pbuddy_adapter->iface_type == IFACE_PORT1)
				rtw_write8(padapter, REG_DUAL_TSF_RST, BIT(1));
			else
				rtw_write8(padapter, REG_DUAL_TSF_RST, BIT(0));
#endif			
			
		}
	}
}
#endif

static void hw_var_set_mlme_join(PADAPTER padapter, u8 variable, u8 *val)
{
	u8 RetryLimit = 0x30;
	u8 type = *val;

	PHAL_DATA_TYPE pHalData = GET_HAL_DATA(padapter);
	struct mlme_priv *pmlmepriv = &padapter->mlmepriv;

#ifdef CONFIG_CONCURRENT_MODE
	PADAPTER pbuddy_adapter = padapter->pbuddy_adapter;
	struct mlme_priv *pbuddy_mlmepriv = &pbuddy_adapter->mlmepriv;
	struct mlme_ext_priv *pbuddy_mlmeext = &pbuddy_adapter->mlmeextpriv;
	struct mlme_ext_info *pbuddy_mlmeinfo = &pbuddy_mlmeext->mlmext_info;
#endif


	if (type == 0) // prepare to join
	{
		u32 v32;

#ifdef CONFIG_CONCURRENT_MODE
		if (((pbuddy_mlmeinfo->state&0x03) == WIFI_FW_AP_STATE) &&
			(check_fwstate(pbuddy_mlmepriv, _FW_LINKED) == _TRUE))
		{
			StopTxBeacon(padapter);
		}
#endif

		// enable to rx data frame.Accept all data frame
		//rtw_write32(padapter, REG_RCR, rtw_read32(padapter, REG_RCR)|RCR_ADF);
		rtw_write16(padapter, REG_RXFLTMAP2, 0xFFFF);

		v32 = rtw_read32(padapter, REG_RCR);
#ifdef CONFIG_CONCURRENT_MODE
		if ((pbuddy_mlmeinfo->state&0x03) == WIFI_FW_AP_STATE)
			v32 |= RCR_CBSSID_BCN;
		else
#endif
		{
			v32 |= RCR_CBSSID_DATA | RCR_CBSSID_BCN;
		}
		rtw_write32(padapter, REG_RCR, v32);

		if (check_fwstate(pmlmepriv, WIFI_STATION_STATE) == _TRUE)
			RetryLimit = (pHalData->CustomerID == RT_CID_CCX) ? 7 : 48;
		else // Ad-hoc Mode
			RetryLimit = 0x7;
	}
	else if (type == 1) // joinbss_event callback when join res < 0
	{
#ifdef CONFIG_CONCURRENT_MODE
		if ((pbuddy_mlmeinfo->state&0x03) == _HW_STATE_NOLINK_)
			rtw_write16(padapter, REG_RXFLTMAP2, 0);

		if (((pbuddy_mlmeinfo->state&0x03) == WIFI_FW_AP_STATE) &&
			(check_fwstate(pbuddy_mlmepriv, _FW_LINKED) == _TRUE))
		{
			ResumeTxBeacon(padapter);

			// reset TSF 1/2 after ResumeTxBeacon
			rtw_write8(padapter, REG_DUAL_TSF_RST, BIT(1)|BIT(0));
		}
#else
		// config RCR to receive different BSSID & not to receive data frame during linking 			
		//v32 = rtw_read32(padapter, REG_RCR);
		//v32 &= ~(RCR_CBSSID_DATA | RCR_CBSSID_BCN);//| RCR_ADF
		//rtw_write32(padapter, REG_RCR, v32);
		rtw_write16(padapter, REG_RXFLTMAP2, 0);
#endif
	}
	else if (type == 2) // sta add event callback
	{
		// enable update TSF
		SetBcnCtrlReg(padapter, 0, DIS_TSF_UDT);

		if (check_fwstate(pmlmepriv, WIFI_ADHOC_STATE|WIFI_ADHOC_MASTER_STATE) == _TRUE)
		{
			// fixed beacon issue for 8191su...........
			rtw_write8(padapter, 0x542, 0x02);
			RetryLimit = 0x7;
		}

#ifdef CONFIG_CONCURRENT_MODE
		if (((pbuddy_mlmeinfo->state&0x03) == WIFI_FW_AP_STATE) &&
			(check_fwstate(pbuddy_mlmepriv, _FW_LINKED) == _TRUE))
		{
			ResumeTxBeacon(padapter);

			// reset TSF 1/2 after ResumeTxBeacon
			rtw_write8(padapter, REG_DUAL_TSF_RST, BIT(1)|BIT(0));
		}
#endif
	}

	rtw_write16(padapter, REG_RL, RetryLimit << RETRY_LIMIT_SHORT_SHIFT | RetryLimit << RETRY_LIMIT_LONG_SHIFT);
}

static void process_c2h_event(PADAPTER padapter,u8	*c2hBuf){
	C2H_EVT_HDR		C2hEvent;
	u8				index = 0;
	HAL_DATA_TYPE	*pHalData=GET_HAL_DATA(padapter);
	if(c2hBuf == NULL){
		DBG_8192C("%s c2hbuff is NULL",__FUNCTION__);
		return;
	}
	_rtw_memset(&C2hEvent, 0, sizeof(C2H_EVT_HDR));
	C2hEvent.CmdID = c2hBuf[0] & 0xF;
	C2hEvent.CmdLen = (c2hBuf[0] & 0xF0) >> 4;
	C2hEvent.CmdSeq =c2hBuf[1];
//	printk("%s CmdSeq=%d\n",__FUNCTION__,C2hEvent.CmdSeq);
#if 0
	//
	// Because the EDCA queue field here is different from the definition in the tx desc,
	// we need to translate it. By Bruce, 2010-09-08.
	//
	switch (GET_92C_C2H_TX_RPT_EDCA_QUEUE(tmpBuf))
	{
		case HAL_92C_C2H_TX_RPT_EDCA_VO:
			QueueID = VO_QUEUE;
			break;

		case HAL_92C_C2H_TX_RPT_EDCA_VI:
			QueueID = VI_QUEUE;
			break;

		case HAL_92C_C2H_TX_RPT_EDCA_BE:
			QueueID = BE_QUEUE;
			break;

		case HAL_92C_C2H_TX_RPT_EDCA_BK:
			QueueID = BK_QUEUE;
			break;

		default:
			QueueID = MGNT_QUEUE;
			break;
	}
#endif
	switch (C2hEvent.CmdID)
	{
		case C2H_DBG:
			{
				RT_TRACE(_module_hal_init_c_, _drv_info_, ("C2HCommandHandler: %s\n", c2hBuf));
			}
			break;

		case C2H_CCX_TX_RPT:
//			CCX_FwC2HTxRpt(padapter, QueueID, tmpBuf);
			break;

#ifdef CONFIG_BT_COEXIST
#ifdef CONFIG_PCI_HCI
		case C2H_BT_RSSI: 
//			fwc2h_ODM(padapter, tmpBuf, &C2hEvent);
			BT_FwC2hBtRssi(padapter, c2hBuf);
			break;
#endif
#endif

		case C2H_EXT_RA_RPT:
//			C2HExtRaRptHandler(padapter, tmpBuf, C2hEvent.CmdLen);
			break;

		case C2H_HW_INFO_EXCH:
			RT_TRACE(_module_hal_init_c_, _drv_info_, ("[BT], C2H_HW_INFO_EXCH\n"));
			for (index = 0; index < C2hEvent.CmdLen; c2hBuf ++)
			{
				RT_TRACE(_module_hal_init_c_, _drv_info_, ("[BT], tmpBuf[%d]=0x%x\n", index, c2hBuf[index]));
			}
			break;

		case C2H_C2H_H2C_TEST:
			RT_TRACE(_module_hal_init_c_, _drv_info_, ("[BT], C2H_H2C_TEST\n"));
			RT_TRACE(_module_hal_init_c_, _drv_info_, ("[BT], tmpBuf[0]/[1]/[2]/[3]/[4]=0x%x/ 0x%x/ 0x%x/ 0x%x/ 0x%x\n", 
				c2hBuf[0], c2hBuf[1], c2hBuf[2], c2hBuf[3], c2hBuf[4]));
			break;

#ifdef CONFIG_BT_COEXIST
		case C2H_BT_INFO:
			BT_FwC2hBtInfo(padapter, c2hBuf, C2hEvent.CmdLen);
			break;
#endif

		default:
			break;
	}

	// Clear event to notify FW we have read the command.
	// Note:
	//	If this field isn't clear, the FW won't update the next command message.
	rtw_write8(padapter, REG_C2HEVT_CLEAR, C2H_EVT_HOST_CLOSE);
}
//
//C2H event format:
// Field	 TRIGGER		CONTENT	   CMD_SEQ 	CMD_LEN		 CMD_ID
// BITS	 [127:120]	[119:16]      [15:8]		  [7:4]	 	   [3:0]
//2009.10.08. by tynli.
static void C2HCommandHandler(PADAPTER padapter)
{
#ifdef CONFIG_SDIO_HCI
	C2H_EVT_HDR		C2hEvent;
	u8				*tmpBuf = NULL;
	u8				index = 0;
	u8				bCmdMsgReady = _FALSE;
	u8				U1bTmp = 0;
//	u8				QueueID = 0;
	_rtw_memset(&C2hEvent, 0, sizeof(C2H_EVT_HDR));

	U1bTmp = rtw_read8(padapter, REG_C2HEVT_MSG_NORMAL);

	C2hEvent.CmdID = U1bTmp & 0xF;
	C2hEvent.CmdLen = (U1bTmp & 0xF0) >> 4;

	C2hEvent.CmdSeq = rtw_read8(padapter, REG_C2HEVT_MSG_NORMAL + 1);

	RT_PRINT_DATA(_module_hal_init_c_, _drv_info_, "C2HCommandHandler(): ",
		&C2hEvent , sizeof(C2hEvent));

	U1bTmp = rtw_read8(padapter, REG_C2HEVT_CLEAR);

	if (U1bTmp == C2H_EVT_HOST_CLOSE)
	{
		// Not ready.
		return;
	}
	else if (U1bTmp == C2H_EVT_FW_CLOSE)
	{
		bCmdMsgReady = _TRUE;
	}
	else
	{
		// Not a valid value, reset the clear event.
		rtw_write8(padapter, REG_C2HEVT_CLEAR, C2H_EVT_HOST_CLOSE);
		return;
	}

	tmpBuf = rtw_zmalloc(C2hEvent.CmdLen);
	if (tmpBuf == NULL)
		return;

	// Read the content
	for (index = 0; index < C2hEvent.CmdLen; index++)
	{
		tmpBuf[index] = rtw_read8(padapter, REG_C2HEVT_MSG_NORMAL + sizeof(C2hEvent)+ index);
	}

	RT_PRINT_DATA(_module_hal_init_c_, _drv_info_, "C2HCommandHandler(): Command Content:\n", tmpBuf, C2hEvent.CmdLen);

	process_c2h_event(padapter, tmpBuf);

	if (tmpBuf)
		rtw_mfree(tmpBuf, C2hEvent.CmdLen);
#endif

#ifdef CONFIG_USB_HCI
	HAL_DATA_TYPE	*pHalData=GET_HAL_DATA(padapter);

	process_c2h_event(padapter,&pHalData->C2hArray[0]);
#endif
	

}

void SetHwReg8723A(PADAPTER padapter, u8 variable, u8 *val)
{
	PHAL_DATA_TYPE	pHalData = GET_HAL_DATA(padapter);

_func_enter_;

	switch (variable)
	{
		case HW_VAR_MEDIA_STATUS:
			{
				u8 val8;

				val8 = rtw_read8(padapter, MSR) & 0x0c;
				val8 |= *val;
				rtw_write8(padapter, MSR, val8);
			}
			break;

		case HW_VAR_MEDIA_STATUS1:
			{
				u8 val8;

				val8 = rtw_read8(padapter, MSR) & 0x03;
				val8 |= *val << 2;
				rtw_write8(padapter, MSR, val8);
			}
			break;

		case HW_VAR_SET_OPMODE:
			hw_var_set_opmode(padapter, variable, val);
			break;

		case HW_VAR_MAC_ADDR:
			hw_var_set_macaddr(padapter, variable, val);
			break;

		case HW_VAR_BSSID:
			hw_var_set_bssid(padapter, variable, val);
			break;

		case HW_VAR_BASIC_RATE:
			{
				u16			BrateCfg = 0;
				u8			RateIndex = 0;

				// 2007.01.16, by Emily
				// Select RRSR (in Legacy-OFDM and CCK)
				// For 8190, we select only 24M, 12M, 6M, 11M, 5.5M, 2M, and 1M from the Basic rate.
				// We do not use other rates.
				rtl8192c_HalSetBrateCfg(padapter, val, &BrateCfg);

				//2011.03.30 add by Luke Lee
				//CCK 2M ACK should be disabled for some BCM and Atheros AP IOT
				//because CCK 2M has poor TXEVM
				//CCK 5.5M & 11M ACK should be enabled for better performance

				pHalData->BasicRateSet = BrateCfg = (BrateCfg |0xd) & 0x15d;
				BrateCfg |= 0x01; // default enable 1M ACK rate
				DBG_8192C("HW_VAR_BASIC_RATE: BrateCfg(%#x)\n", BrateCfg);

				// Set RRSR rate table.
				rtw_write8(padapter, REG_RRSR, BrateCfg&0xff);
				rtw_write8(padapter, REG_RRSR+1, (BrateCfg>>8)&0xff);

				// Set RTS initial rate
				while (BrateCfg > 0x1)
				{
					BrateCfg = (BrateCfg >> 1);
					RateIndex++;
				}
				// Ziv - Check
				rtw_write8(padapter, REG_INIRTS_RATE_SEL, RateIndex);
			}
			break;

		case HW_VAR_TXPAUSE:
			rtw_write8(padapter, REG_TXPAUSE, *val);
			break;

		case HW_VAR_BCN_FUNC:
			if (*val)
				SetBcnCtrlReg(padapter, EN_BCN_FUNCTION | EN_TXBCN_RPT, 0);
			else
				SetBcnCtrlReg(padapter, 0, EN_BCN_FUNCTION | EN_TXBCN_RPT);
			break;

		case HW_VAR_CORRECT_TSF:
			hw_var_set_correct_tsf(padapter, variable, val);
			break;

		case HW_VAR_CHECK_BSSID:
			{
				u32 val32;
				val32 = rtw_read32(padapter, REG_RCR);
				if (*val)
					val32 |= RCR_CBSSID_DATA|RCR_CBSSID_BCN;
				else
					val32 &= ~(RCR_CBSSID_DATA|RCR_CBSSID_BCN);
				rtw_write32(padapter, REG_RCR, val32);
			}
			break;

		case HW_VAR_MLME_DISCONNECT:
			hw_var_set_mlme_disconnect(padapter, variable, val);
			break;

		case HW_VAR_MLME_SITESURVEY:
#ifdef CONFIG_CONCURRENT_MODE
			hw_var_set_mlme_sitesurvey(padapter, variable,  val);
#else
			if (*val)//under sitesurvey
			{
				u32 v32;

				// config RCR to receive different BSSID & not to receive data frame
				//pHalData->ReceiveConfig &= (~(RCR_CBSSID_DATA | RCR_CBSSID_BCN));
				v32 = rtw_read32(padapter, REG_RCR);
				v32 &= ~(RCR_CBSSID_DATA | RCR_CBSSID_BCN);//| RCR_ADF
				rtw_write32(padapter, REG_RCR, v32);
				// reject all data frame
				rtw_write16(padapter, REG_RXFLTMAP2, 0);

				// disable update TSF
				SetBcnCtrlReg(padapter, DIS_TSF_UDT, 0);
			}
			else//sitesurvey done
			{
				struct mlme_ext_priv *pmlmeext = &padapter->mlmeextpriv;
				struct mlme_ext_info *pmlmeinfo = &pmlmeext->mlmext_info;
				u32 v32;

				if ((is_client_associated_to_ap(padapter) == _TRUE) ||
					((pmlmeinfo->state&0x03) == WIFI_FW_ADHOC_STATE) ||
					((pmlmeinfo->state&0x03) == WIFI_FW_AP_STATE))
				{
					// enable to rx data frame
#if 0
					v32 = rtw_read32(padapter, REG_RCR);
					v32 |= RCR_ADF;
					rtw_write32(padapter, REG_RCR, v32);
#else
					rtw_write16(padapter, REG_RXFLTMAP2, 0xFFFF);
#endif

					// enable update TSF
					SetBcnCtrlReg(padapter, 0, DIS_TSF_UDT);
				}

				v32 = rtw_read32(padapter, REG_RCR);
				if ((pmlmeinfo->state&0x03) == WIFI_FW_AP_STATE)
					v32 |= RCR_CBSSID_BCN;
				else
					v32 |= RCR_CBSSID_DATA | RCR_CBSSID_BCN;
				rtw_write32(padapter, REG_RCR, v32);
			}
#endif
			break;

		case HW_VAR_MLME_JOIN:
			hw_var_set_mlme_join(padapter, variable,  val);
			break;

		case HW_VAR_BEACON_INTERVAL:
			rtw_write16(padapter, REG_BCN_INTERVAL, *((u16*)val));
			break;

		case HW_VAR_SLOT_TIME:
			{
				u8 u1bAIFS, aSifsTime;
				struct mlme_ext_priv *pmlmeext = &padapter->mlmeextpriv;
				struct mlme_ext_info *pmlmeinfo = &pmlmeext->mlmext_info;

				rtw_write8(padapter, REG_SLOT, *val);

				if (pmlmeinfo->WMM_enable == 0)
				{
					if (pmlmeext->cur_wireless_mode == WIRELESS_11B)
						aSifsTime = 10;
					else
						aSifsTime = 16;

					u1bAIFS = aSifsTime + (2 * pmlmeinfo->slotTime);

					// <Roger_EXP> Temporary removed, 2008.06.20.
					rtw_write8(padapter, REG_EDCA_VO_PARAM, u1bAIFS);
					rtw_write8(padapter, REG_EDCA_VI_PARAM, u1bAIFS);
					rtw_write8(padapter, REG_EDCA_BE_PARAM, u1bAIFS);
					rtw_write8(padapter, REG_EDCA_BK_PARAM, u1bAIFS);
				}
			}
			break;

		case HW_VAR_SIFS:
			// SIFS for OFDM Data ACK
			rtw_write8(padapter, REG_SIFS_CTX+1, val[0]);
			// SIFS for OFDM consecutive tx like CTS data!
			rtw_write8(padapter, REG_SIFS_TRX+1, val[1]);

			rtw_write8(padapter, REG_SPEC_SIFS+1, val[0]);
			rtw_write8(padapter, REG_MAC_SPEC_SIFS+1, val[0]);

			// 20100719 Joseph: Revise SIFS setting due to Hardware register definition change.
			rtw_write8(padapter, REG_R2T_SIFS+1, val[0]);
			rtw_write8(padapter, REG_T2T_SIFS+1, val[0]);
			break;

		case HW_VAR_ACK_PREAMBLE:
			{
				u8 regTmp;
				u8 bShortPreamble = *val;

				// Joseph marked out for Netgear 3500 TKIP channel 7 issue.(Temporarily)
				//regTmp = (pHalData->nCur40MhzPrimeSC)<<5;
				regTmp = 0;
				if (bShortPreamble) regTmp |= 0x80;
				rtw_write8(padapter, REG_RRSR+2, regTmp);
			}
			break;

		case HW_VAR_SEC_CFG:
#ifdef CONFIG_CONCURRENT_MODE
			rtw_write8(padapter, REG_SECCFG, 0x0c|BIT(5));// enable tx enc and rx dec engine, and no key search for MC/BC				
#else
			rtw_write8(padapter, REG_SECCFG, *val);
#endif
			break;

		case HW_VAR_DM_FLAG:
			pHalData->odmpriv.SupportAbility = *((u32*)val);
			break;

		case HW_VAR_DM_FUNC_OP:
			if (*val) // save dm flag
				pHalData->odmpriv.BK_SupportAbility = pHalData->odmpriv.SupportAbility;
			else // restore dm flag
				pHalData->odmpriv.SupportAbility = pHalData->odmpriv.BK_SupportAbility;
			break;

		case HW_VAR_DM_FUNC_SET:
			if (*((u32*)val) == DYNAMIC_ALL_FUNC_ENABLE) {
				pHalData->dmpriv.DMFlag = pHalData->dmpriv.InitDMFlag;
				pHalData->odmpriv.SupportAbility = pHalData->dmpriv.InitODMFlag;
			} else {
				pHalData->odmpriv.SupportAbility |= *((u32*)val);
			}
			break;

		case HW_VAR_DM_FUNC_CLR:
			pHalData->odmpriv.SupportAbility &= *((u32*)val);
			break;

		case HW_VAR_CAM_EMPTY_ENTRY:
			{
				u8	ucIndex = *val;
				u8	i;
				u32	ulCommand = 0;
				u32	ulContent = 0;
				u32	ulEncAlgo = CAM_AES;

				for (i=0; i<CAM_CONTENT_COUNT; i++)
				{
					// filled id in CAM config 2 byte
					if (i == 0)
					{
						ulContent |= (ucIndex & 0x03) | ((u16)(ulEncAlgo)<<2);
						//ulContent |= CAM_VALID;
					}
					else
					{
						ulContent = 0;
					}
					// polling bit, and No Write enable, and address
					ulCommand = CAM_CONTENT_COUNT*ucIndex+i;
					ulCommand = ulCommand | CAM_POLLINIG | CAM_WRITE;
					// write content 0 is equall to mark invalid
					rtw_write32(padapter, WCAMI, ulContent);  //delay_ms(40);
					//RT_TRACE(COMP_SEC, DBG_LOUD, ("CAM_empty_entry(): WRITE A4: %lx \n",ulContent));
					rtw_write32(padapter, RWCAM, ulCommand);  //delay_ms(40);
					//RT_TRACE(COMP_SEC, DBG_LOUD, ("CAM_empty_entry(): WRITE A0: %lx \n",ulCommand));
				}
			}
			break;

		case HW_VAR_CAM_INVALID_ALL:
			rtw_write32(padapter, RWCAM, BIT(31)|BIT(30));
			break;

		case HW_VAR_CAM_WRITE:
			{
				u32 cmd;
				u32 *cam_val = (u32*)val;

				rtw_write32(padapter, WCAMI, cam_val[0]);

				cmd = CAM_POLLINIG | CAM_WRITE | cam_val[1];
				rtw_write32(padapter, RWCAM, cmd);
			}
			break;

		case HW_VAR_AC_PARAM_VO:
			rtw_write32(padapter, REG_EDCA_VO_PARAM, *((u32*)val));
			break;

		case HW_VAR_AC_PARAM_VI:
			rtw_write32(padapter, REG_EDCA_VI_PARAM, *((u32*)val));
			break;

		case HW_VAR_AC_PARAM_BE:
			pHalData->AcParam_BE = ((u32*)(val))[0];
			rtw_write32(padapter, REG_EDCA_BE_PARAM, *((u32*)val));
			break;

		case HW_VAR_AC_PARAM_BK:
			rtw_write32(padapter, REG_EDCA_BK_PARAM, *((u32*)val));
			break;

		case HW_VAR_ACM_CTRL:
			{
				u8 ctrl = *((u8*)val);
				u8 hwctrl = 0;

				if (ctrl != 0)
				{
					hwctrl |= AcmHw_HwEn;

					if (ctrl & BIT(1)) // BE
						hwctrl |= AcmHw_BeqEn;

					if (ctrl & BIT(2)) // VI
						hwctrl |= AcmHw_ViqEn;

					if (ctrl & BIT(3)) // VO
						hwctrl |= AcmHw_VoqEn;
				}

				DBG_8192C("[HW_VAR_ACM_CTRL] Write 0x%02X\n", hwctrl);
				rtw_write8(padapter, REG_ACMHWCTRL, hwctrl);
			}
			break;

		case HW_VAR_AMPDU_MIN_SPACE:
			{
				u8	MinSpacingToSet;
				u8	SecMinSpace;

				MinSpacingToSet = *val;
				if (MinSpacingToSet <= 7)
				{
					switch (padapter->securitypriv.dot11PrivacyAlgrthm)
					{
						case _NO_PRIVACY_:
						case _AES_:
							SecMinSpace = 0;
							break;

						case _WEP40_:
						case _WEP104_:
						case _TKIP_:
						case _TKIP_WTMIC_:
							SecMinSpace = 6;
							break;
						default:
							SecMinSpace = 7;
							break;
					}

					if (MinSpacingToSet < SecMinSpace)
						MinSpacingToSet = SecMinSpace;

					//RT_TRACE(COMP_MLME, DBG_LOUD, ("Set HW_VAR_AMPDU_MIN_SPACE: %#x\n", padapter->MgntInfo.MinSpaceCfg));
					MinSpacingToSet |= rtw_read8(padapter, REG_AMPDU_MIN_SPACE) & 0xf8;
					rtw_write8(padapter, REG_AMPDU_MIN_SPACE, MinSpacingToSet);
				}
			}
			break;

		case HW_VAR_AMPDU_FACTOR:
			{
				u8 RegToSet_Normal[4] = {0x41,0xa8,0x72, 0xb9};
				u8 FactorToSet;
				u8 *pRegToSet;
				u8 index = 0;

				pRegToSet = RegToSet_Normal; // 0xb972a841;

				FactorToSet = *val;
				if (FactorToSet <= 3)
				{
					FactorToSet = (1 << (FactorToSet + 2));
					if (FactorToSet > 0xf)
						FactorToSet = 0xf;

					for (index=0; index<4; index++)
					{
						if ((pRegToSet[index] & 0xf0) > (FactorToSet << 4))
							pRegToSet[index] = (pRegToSet[index] & 0x0f) | (FactorToSet << 4);

						if ((pRegToSet[index] & 0x0f) > FactorToSet)
							pRegToSet[index] = (pRegToSet[index] & 0xf0) | FactorToSet;

						rtw_write8(padapter, REG_AGGLEN_LMT+index, pRegToSet[index]);
					}

					//RT_TRACE(COMP_MLME, DBG_LOUD, ("Set HW_VAR_AMPDU_FACTOR: %#x\n", FactorToSet));
				}
			}
			break;

		case HW_VAR_RXDMA_AGG_PG_TH:
			rtw_write8(padapter, REG_RXDMA_AGG_PG_TH, *val);
			break;

		case HW_VAR_H2C_FW_PWRMODE:
			{
				u8 psmode = *val;

				// Forece leave RF low power mode for 1T1R to prevent conficting setting in Fw power
				// saving sequence. 2010.06.07. Added by tynli. Suggested by SD3 yschang.
				if ((psmode != PS_MODE_ACTIVE) && (!IS_92C_SERIAL(pHalData->VersionID)))
				{
					ODM_RF_Saving(&pHalData->odmpriv, _TRUE);
				}
				rtl8723a_set_FwPwrMode_cmd(padapter, psmode);
			}
			break;

		case HW_VAR_H2C_FW_JOINBSSRPT:
			rtl8723a_set_FwJoinBssReport_cmd(padapter, *val);
			break;

#ifdef CONFIG_P2P
		case HW_VAR_H2C_FW_P2P_PS_OFFLOAD:
			rtl8192c_set_p2p_ps_offload_cmd(padapter, *val);
			break;
#endif //CONFIG_P2P

		case HW_VAR_INITIAL_GAIN:
			{
				DIG_T *pDigTable = &pHalData->odmpriv.DM_DigTable;
				u32 rx_gain = *(u32*)val;

				if (rx_gain == 0xff) {//restore rx gain
					ODM_Write_DIG(&pHalData->odmpriv, pDigTable->BackupIGValue);
				} else {
					pDigTable->BackupIGValue = pDigTable->CurIGValue;
					ODM_Write_DIG(&pHalData->odmpriv, rx_gain);
				}
			}
			break;

#ifdef CONFIG_SW_ANTENNA_DIVERSITY
		case HW_VAR_ANTENNA_DIVERSITY_LINK:
			//SwAntDivRestAfterLink8192C(padapter);
			ODM_SwAntDivRestAfterLink(&pHalData->odmpriv);
			break;

		case HW_VAR_ANTENNA_DIVERSITY_SELECT:
			{
				u8 Optimum_antenna = *val;

				//DBG_8192C("==> HW_VAR_ANTENNA_DIVERSITY_SELECT , Ant_(%s)\n",(Optimum_antenna==2)?"A":"B");

				//PHY_SetBBReg(padapter, rFPGA0_XA_RFInterfaceOE, 0x300, Optimum_antenna);
				ODM_SetAntenna(&pHalData->odmpriv, Optimum_antenna);
			}
			break;
#endif

		case HW_VAR_EFUSE_BYTES: // To set EFUE total used bytes, added by Roger, 2008.12.22.
			pHalData->EfuseUsedBytes = *((u16*)val);
			break;

		case HW_VAR_FIFO_CLEARN_UP:
			{
				#define RW_RELEASE_EN		BIT(18)
				#define RXDMA_IDLE			BIT(17)

				struct pwrctrl_priv *pwrpriv = &padapter->pwrctrlpriv;
				u8 trycnt = 100;

				// pause tx
				rtw_write8(padapter, REG_TXPAUSE, 0xff);

				// keep sn
				padapter->xmitpriv.nqos_ssn = rtw_read16(padapter, REG_NQOS_SEQ);

				if (pwrpriv->bkeepfwalive != _TRUE)
				{
					u32 v32;

					// RX DMA stop
					v32 = rtw_read32(padapter, REG_RXPKT_NUM);
					v32 |= RW_RELEASE_EN;
					rtw_write32(padapter, REG_RXPKT_NUM, v32);
					do {
						v32 = rtw_read32(padapter, REG_RXPKT_NUM) & RXDMA_IDLE;
						if (!v32) break;
					} while (trycnt--);
					if (trycnt == 0) {
						DBG_8192C("Stop RX DMA failed......\n");
					}

					// RQPN Load 0
					rtw_write16(padapter, REG_RQPN_NPQ, 0);
					rtw_write32(padapter, REG_RQPN, 0x80000000);
					rtw_mdelay_os(10);
				}
			}
			break;

		case HW_VAR_CHECK_TXBUF:
#ifdef CONFIG_CONCURRENT_MODE
			{
				u16 v16;
				u32 i;
				u8 RetryLimit = 0x01;

				//rtw_write16(padapter, REG_RL,0x0101);
				v16 = RetryLimit << RETRY_LIMIT_SHORT_SHIFT | RetryLimit << RETRY_LIMIT_LONG_SHIFT;
				rtw_write16(padapter, REG_RL, v16);

				for (i=0; i<1000; i++)
				{
					if (rtw_read32(padapter, 0x200) != rtw_read32(padapter, 0x204))
					{
						//DBG_871X("packet in tx packet buffer - 0x204=%x, 0x200=%x (%d)\n", rtw_read32(padapter, 0x204), rtw_read32(padapter, 0x200), i);
						rtw_msleep_os(10);
					}
					else
					{
						DBG_871X("no packet in tx packet buffer (%d)\n", i);
						break;
					}
				}

				RetryLimit = 0x30;
				v16 = RetryLimit << RETRY_LIMIT_SHORT_SHIFT | RetryLimit << RETRY_LIMIT_LONG_SHIFT;
				rtw_write16(padapter, REG_RL, v16);
			}
#endif
			break;

		case HW_VAR_APFM_ON_MAC:
			pHalData->bMacPwrCtrlOn = *val;
#ifdef PLATFORM_LINUX
			printk("%s: bMacPwrCtrlOn=%d\n", __func__, pHalData->bMacPwrCtrlOn);
#endif
			break;

		case HW_VAR_NAV_UPPER:
			{
				u32 usNavUpper = *((u32*)val);
		
				if (usNavUpper > HAL_8723A_NAV_UPPER_UNIT * 0xFF)
				{
					RT_TRACE(_module_hal_init_c_, _drv_notice_, ("The setting value (0x%08X us) of NAV_UPPER is larger than (%d * 0xFF)!!!\n", usNavUpper, HAL_8723A_NAV_UPPER_UNIT));
					break;
				}
		
				// The value of ((usNavUpper + HAL_8723A_NAV_UPPER_UNIT - 1) / HAL_8723A_NAV_UPPER_UNIT)
				// is getting the upper integer.
				usNavUpper = (usNavUpper + HAL_8723A_NAV_UPPER_UNIT - 1) / HAL_8723A_NAV_UPPER_UNIT;
				rtw_write8(padapter, REG_NAV_UPPER, (u8)usNavUpper);
			}
			break;

		case HW_VAR_C2H_HANDLE:
			C2HCommandHandler(padapter);
			break;

		default:
			break;
	}

_func_exit_;
}

void GetHwReg8723A(PADAPTER padapter, u8 variable, u8 *val)
{
	PHAL_DATA_TYPE pHalData = GET_HAL_DATA(padapter);


	switch (variable)
	{
		case HW_VAR_BASIC_RATE:
			*((u16*)val) = pHalData->BasicRateSet;
			break;

		case HW_VAR_TXPAUSE:
			*val = rtw_read8(padapter, REG_TXPAUSE);
			break;

		case HW_VAR_TX_BCN_DONE:
			{
				u32 xmitbcnDown;
				xmitbcnDown = rtw_read32(padapter, REG_TDECTRL);
				if (xmitbcnDown & BCN_VALID) {
					rtw_write32(padapter, REG_TDECTRL, xmitbcnDown | BCN_VALID); // write 1 to clear, Clear by sw
					*val = _TRUE;
				}
			}
			break;

		case HW_VAR_RF_TYPE:
			*val = pHalData->rf_type;
			break;

		case HW_VAR_DM_FLAG:
			{
				PDM_ODM_T podmpriv = &pHalData->odmpriv;
				*((u32*)val) = podmpriv->SupportAbility;
			}
			break;

		case HW_VAR_FWLPS_RF_ON:
			{
				// When we halt NIC, we should check if FW LPS is leave.
				u32 valRCR;

				if ((padapter->bSurpriseRemoved == _TRUE) ||
					(padapter->pwrctrlpriv.rf_pwrstate == rf_off))
				{
					// If it is in HW/SW Radio OFF or IPS state, we do not check Fw LPS Leave,
					// because Fw is unload.
					*val = _TRUE;
				}
				else
				{
					valRCR = rtw_read32(padapter, REG_RCR);
					valRCR &= 0x00070000;
					if(valRCR)
						*val = _FALSE;
					else
						*val = _TRUE;
				}
			}
			break;

#ifdef CONFIG_ANTENNA_DIVERSITY
		case HW_VAR_CURRENT_ANTENNA:
			*val = pHalData->CurAntenna;
			break;
#endif

		case HW_VAR_EFUSE_BYTES: // To get EFUE total used bytes, added by Roger, 2008.12.22.
			*((u16*)val) = pHalData->EfuseUsedBytes;
			break;

		case HW_VAR_APFM_ON_MAC:
			*val = pHalData->bMacPwrCtrlOn;
			break;
	}
}

#ifdef CONFIG_BT_COEXIST

#if 1
static void
_PHY_SaveAFERegisters(
	PADAPTER	padapter,
	u32		*AFEReg,
	u32		*AFEBackup,
	u32		RegisterNum
	)
{
	u32	i;

//	RTPRINT(FINIT, INIT_IQK, ("Save ADDA parameters.\n"));
	for (i = 0; i < RegisterNum; i++) {
		AFEBackup[i] = PHY_QueryBBReg(padapter, AFEReg[i], bMaskDWord);
	}
}

static void
_PHY_ReloadAFERegisters(
	PADAPTER	padapter,
	u32		*AFEReg,
	u32		*AFEBackup,
	u32		RegiesterNum
	)
{
	u32	i;

//	RTPRINT(FINIT, INIT_IQK, ("Reload ADDA power saving parameters !\n"));
	for (i = 0; i < RegiesterNum; i++)
	{
		PHY_SetBBReg(padapter, AFEReg[i], bMaskDWord, AFEBackup[i]);
	}
}

extern const u16 dB_Invert_Table[8][12];

static u32 ConvertTo_dB(u32 Value)
{
	u8 i;
	u8 j;
	u32 dB;


	Value = Value & 0xFFFF;

	for (i=0; i<8; i++)
	{
		if (Value <= dB_Invert_Table[i][11])
		{
			break;
		}
	}

	if (i >= 8)
	{
		return (96);	// maximum 96 dB
	}

	for (j=0;j<12;j++)
	{
		if (Value <= dB_Invert_Table[i][j])
		{
			break;
		}
	}

	dB = i*12 + j + 1;

	return (dB);
}

static u32 GetPSDData_8192C(
	PADAPTER	padapter,
	u32 		point,
	u8			initial_gain_psd)
{
	u32	psd_report;


	// Set DCO frequency index, offset=(40MHz/SamplePts)*point
	PHY_SetBBReg(padapter, 0x808, 0x3FF, point);

	// Start PSD calculation, Reg808[22]=0->1
	PHY_SetBBReg(padapter, 0x808, BIT(22), 1);
	// Need to wait for HW PSD report
	rtw_udelay_os(30);
	PHY_SetBBReg(padapter, 0x808, BIT(22), 0);
	// Read PSD report, Reg8B4[15:0]
	psd_report = PHY_QueryBBReg(padapter, 0x8B4, bMaskDWord) & 0x0000FFFF;
#if 1//(DEV_BUS_TYPE == RT_PCI_INTERFACE) && ( (RT_PLATFORM == PLATFORM_LINUX) || (RT_PLATFORM == PLATFORM_MACOSX))
	psd_report = (u32) (ConvertTo_dB(psd_report))+(u32)(initial_gain_psd-0x1c);
#else
	psd_report = (int) (20*log10((double)psd_report))+(int)(initial_gain_psd-0x1c);
#endif

	return psd_report;
}

//2 8723A ANT DETECT
//
// Description:
//	Implement IQK single tone for RF DPK loopback and BB PSD scanning. 
//	This function is cooperated with BB team Neil. 
//
// Added by Roger, 2011.12.15
//
static u8 ODM_SingleDualAntennaDetection(PADAPTER padapter, pSWAT_T pDM_SWAT_Table)
{
	PHAL_DATA_TYPE	pHalData = GET_HAL_DATA(padapter);
//	pSWAT_T		pDM_SWAT_Table = &padapter->DM_SWAT_Table;
	u32		CurrentChannel;
	u8		n, i;
	u32		Reg88c, Regc08, Reg874, Regc50;
	u8		initial_gain = 0x5a;
	u32		PSD_report_tmp;
	u32		AntA_report = 0x0, AntB_report = 0x0;
	u8		bResult = _TRUE;
	u32		AFE_Backup[16];
	u32		AFE_REG_8723A[16] = {
					rRx_Wait_CCA, 	rTx_CCK_RFON,
					rTx_CCK_BBON, 	rTx_OFDM_RFON,
					rTx_OFDM_BBON, 	rTx_To_Rx,
					rTx_To_Tx, 		rRx_CCK, 
					rRx_OFDM, 		rRx_Wait_RIFS,
					rRx_TO_Rx,		rStandby,
					rSleep,			rPMPD_ANAEN,
					rFPGA0_XCD_SwitchControl, rBlue_Tooth};


#if 0
	if (!IS_HARDWARE_TYPE_8723A(padapter))
		return bResult;
#endif

#ifdef CONFIG_ANTENNA_DIVERSITY
	if (pHalData->AntDivCfg == 0)
		return bResult;
#endif

	//1 Backup Current RF/BB Settings

	CurrentChannel = PHY_QueryRFReg(padapter, RF_PATH_A, 0x18, bRFRegOffsetMask);

	PHY_SetBBReg(padapter, rFPGA0_XA_RFInterfaceOE, 0x300, Antenna_A);	// change to Antenna A
	// Step 1: USE IQK to transmitter single tone

	rtw_udelay_os(10);

	// Store A Path Register 88c, c08, 874, c50
	Reg88c = PHY_QueryBBReg(padapter, rFPGA0_AnalogParameter4, bMaskDWord);
	Regc08 = PHY_QueryBBReg(padapter, rOFDM0_TRMuxPar, bMaskDWord);
	Reg874 = PHY_QueryBBReg(padapter, rFPGA0_XCD_RFInterfaceSW, bMaskDWord);
	Regc50 = PHY_QueryBBReg(padapter, rOFDM0_XAAGCCore1, bMaskDWord);

	// Store AFE Registers
	_PHY_SaveAFERegisters(padapter, AFE_REG_8723A, AFE_Backup, 16);

	PHY_SetBBReg(padapter, rFPGA0_PSDFunction, BIT(14)|BIT(15), 0x0);	// 128 pts

	// To SET CH1 to do
	PHY_SetRFReg(padapter, RF_PATH_A, 0x18, bRFRegOffsetMask, 0x01);	// Channel 1

	// AFE all on step
	PHY_SetBBReg(padapter, rRx_Wait_CCA, bMaskDWord, 0x6FDB25A4);
	PHY_SetBBReg(padapter, rTx_CCK_RFON, bMaskDWord, 0x6FDB25A4);
	PHY_SetBBReg(padapter, rTx_CCK_BBON, bMaskDWord, 0x6FDB25A4);
	PHY_SetBBReg(padapter, rTx_OFDM_RFON, bMaskDWord, 0x6FDB25A4);
	PHY_SetBBReg(padapter, rTx_OFDM_BBON, bMaskDWord, 0x6FDB25A4);
	PHY_SetBBReg(padapter, rTx_To_Rx, bMaskDWord, 0x6FDB25A4);
	PHY_SetBBReg(padapter, rTx_To_Tx, bMaskDWord, 0x6FDB25A4);
	PHY_SetBBReg(padapter, rRx_CCK, bMaskDWord, 0x6FDB25A4);
	PHY_SetBBReg(padapter, rRx_OFDM, bMaskDWord, 0x6FDB25A4);
	PHY_SetBBReg(padapter, rRx_Wait_RIFS, bMaskDWord, 0x6FDB25A4);
	PHY_SetBBReg(padapter, rRx_TO_Rx, bMaskDWord, 0x6FDB25A4);
	PHY_SetBBReg(padapter, rStandby, bMaskDWord, 0x6FDB25A4);
	PHY_SetBBReg(padapter, rSleep, bMaskDWord, 0x6FDB25A4);
	PHY_SetBBReg(padapter, rPMPD_ANAEN, bMaskDWord, 0x6FDB25A4);
	PHY_SetBBReg(padapter, rFPGA0_XCD_SwitchControl, bMaskDWord, 0x6FDB25A4);
	PHY_SetBBReg(padapter, rBlue_Tooth, bMaskDWord, 0x6FDB25A4);

	// 3 wire Disable
	PHY_SetBBReg(padapter, rFPGA0_AnalogParameter4, bMaskDWord, 0xCCF000C0);

	// BB IQK Setting
	PHY_SetBBReg(padapter, rOFDM0_TRMuxPar, bMaskDWord, 0x000800E4);
	PHY_SetBBReg(padapter, rFPGA0_XCD_RFInterfaceSW, bMaskDWord, 0x22208000);

	// IQK setting tone@ 4.34Mhz
	PHY_SetBBReg(padapter, rTx_IQK_Tone_A, bMaskDWord, 0x10008C1C);
	PHY_SetBBReg(padapter, rTx_IQK, bMaskDWord, 0x01007c00);


	// Page B init
	PHY_SetBBReg(padapter, rConfig_AntA, bMaskDWord, 0x00080000);
	PHY_SetBBReg(padapter, rConfig_AntA, bMaskDWord, 0x0f600000);
	PHY_SetBBReg(padapter, rRx_IQK, bMaskDWord, 0x01004800);
	PHY_SetBBReg(padapter, rRx_IQK_Tone_A, bMaskDWord, 0x10008c1f);
	PHY_SetBBReg(padapter, rTx_IQK_PI_A, bMaskDWord, 0x82150008);
	PHY_SetBBReg(padapter, rRx_IQK_PI_A, bMaskDWord, 0x28150008);
	PHY_SetBBReg(padapter, rIQK_AGC_Rsp, bMaskDWord, 0x001028d0);

	// RF loop Setting
	PHY_SetRFReg(padapter, RF_PATH_A, 0x0, 0xFFFFF, 0x50008);

	// IQK Single tone start
	PHY_SetBBReg(padapter, rFPGA0_IQK, bMaskDWord, 0x80800000);
	PHY_SetBBReg(padapter, rIQK_AGC_Pts, bMaskDWord, 0xf8000000);
	rtw_udelay_os(1000);

	for (n=0; n<10; n++)
	{
		PSD_report_tmp = GetPSDData_8192C(padapter, 14, initial_gain);
		if (PSD_report_tmp > AntA_report)
			AntA_report = PSD_report_tmp;
	}

	PSD_report_tmp = 0x0;

	PHY_SetBBReg(padapter, rFPGA0_XA_RFInterfaceOE, 0x300, Antenna_B);  // change to Antenna B
	rtw_udelay_os(10);

	for (n=0; n<10; n++)
	{
		PSD_report_tmp = GetPSDData_8192C(padapter, 14, initial_gain);
		if (PSD_report_tmp > AntB_report)
			AntB_report = PSD_report_tmp;
	}

	// Close IQK Single Tone function
	PHY_SetBBReg(padapter, rFPGA0_IQK, bMaskDWord, 0x00000000);
	PSD_report_tmp = 0x0;

	//1 Return to antanna A
	PHY_SetBBReg(padapter, rFPGA0_XA_RFInterfaceOE, 0x300, Antenna_A);
	PHY_SetBBReg(padapter, rFPGA0_AnalogParameter4, bMaskDWord, Reg88c);
	PHY_SetBBReg(padapter, rOFDM0_TRMuxPar, bMaskDWord, Regc08);
	PHY_SetBBReg(padapter, rFPGA0_XCD_RFInterfaceSW, bMaskDWord, Reg874);
	PHY_SetBBReg(padapter, rOFDM0_XAAGCCore1, 0x7F, 0x40);
	PHY_SetBBReg(padapter, rOFDM0_XAAGCCore1, bMaskDWord, Regc50);
	PHY_SetRFReg(padapter, RF_PATH_A, RF_CHNLBW, bRFRegOffsetMask,CurrentChannel);

	// Reload AFE Registers
	_PHY_ReloadAFERegisters(padapter, AFE_REG_8723A, AFE_Backup, 16);

	RT_TRACE(_module_hal_init_c_, _drv_notice_, ("psd_report_A[%d]= %d \n", 2416, AntA_report));
	RT_TRACE(_module_hal_init_c_, _drv_notice_, ("psd_report_B[%d]= %d \n", 2416, AntB_report));

	if (AntA_report >= 100)
	{
		if (AntB_report > (AntA_report+1))
		{
			pDM_SWAT_Table->ANTB_ON = _FALSE;
			RT_TRACE(_module_hal_init_c_, _drv_notice_, ("ODM_SingleDualAntennaDetection(): Single Antenna A\n"));
		}
		else
		{
			pDM_SWAT_Table->ANTB_ON = _TRUE;
			RT_TRACE(_module_hal_init_c_, _drv_notice_, ("ODM_SingleDualAntennaDetection(): Dual Antenna is A and B\n"));
		}
	}
	else
	{
		RT_TRACE(_module_hal_init_c_, _drv_notice_, ("ODM_SingleDualAntennaDetection(): Need to check again\n"));
		pDM_SWAT_Table->ANTB_ON = _FALSE; // Set Antenna B off as default
		bResult = _FALSE;
	}

	return bResult;
}
#endif

void rtl8723a_SingleDualAntennaDetection(PADAPTER padapter)
{
	PHAL_DATA_TYPE pHalData;
	PDM_ODM_T pDM_Odm;
	pSWAT_T pDM_SWAT_Table;
	u8 btAntNum;
	u8 i;


	pHalData = GET_HAL_DATA(padapter);
	pDM_Odm = &pHalData->odmpriv;
	pDM_SWAT_Table= &pDM_Odm->DM_SWAT_Table;

	//
	// <Roger_Notes> RTL8723A Single and Dual antenna dynamic detection mechanism when RF power state is on.
	// We should take power tracking, IQK, LCK, RCK RF read/write operation into consideration.
	// 2011.12.15.
	//
	if (IS_HARDWARE_TYPE_8723A(padapter) && !pHalData->bAntennaDetected)
	{
		u8 btAntNum = BT_GetPGAntNum(padapter);

		// Set default antenna B status
		if (btAntNum == Ant_x2)
			pDM_SWAT_Table->ANTB_ON = _TRUE;
		else if (btAntNum == Ant_x1)
			pDM_SWAT_Table->ANTB_ON = _FALSE;
		else
			pDM_SWAT_Table->ANTB_ON = _TRUE;

		if (pHalData->CustomerID != RT_CID_TOSHIBA )
		{
			for (i=0; i<MAX_ANTENNA_DETECTION_CNT; i++)
			{
				if (ODM_SingleDualAntennaDetection(padapter, pDM_SWAT_Table) == _TRUE)
					break;
			}

			// Set default antenna number for BT coexistence
			if (btAntNum == Ant_x2)
				BT_SetBtCoexCurrAntNum(padapter, pDM_SWAT_Table->ANTB_ON ? 2 : 1);
		}
		pHalData->bAntennaDetected = _TRUE;
	}
}
#endif // CONFIG_BT_COEXIST

