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
#define _RTL8723AS_RECV_C_

#include <drv_conf.h>

#if defined (PLATFORM_LINUX) && defined (PLATFORM_WINDOWS)
#error "Shall be Linux or Windows, but not both!\n"
#endif

#include <drv_types.h>
#include <recv_osdep.h>
#include <rtl8723a_hal.h>



static s32 initrecvbuf(struct recv_buf *precvbuf, PADAPTER padapter)
{
	_rtw_init_listhead(&precvbuf->list);
	_rtw_spinlock_init(&precvbuf->recvbuf_lock);

	precvbuf->adapter = padapter;

	return _SUCCESS;
}

static void freerecvbuf(struct recv_buf *precvbuf)
{
	_rtw_spinlock_free(&precvbuf->recvbuf_lock);
}

static void update_recvframe_attrib(
	union recv_frame *precvframe,
	struct recv_stat *prxstat)
{
	struct rx_pkt_attrib	*pattrib;
	struct recv_stat	report;
	PRXREPORT		prxreport;


	report.rxdw0 = le32_to_cpu(prxstat->rxdw0);
	report.rxdw1 = le32_to_cpu(prxstat->rxdw1);
	report.rxdw2 = le32_to_cpu(prxstat->rxdw2);
	report.rxdw3 = le32_to_cpu(prxstat->rxdw3);
	report.rxdw4 = le32_to_cpu(prxstat->rxdw4);
	report.rxdw5 = le32_to_cpu(prxstat->rxdw5);

	prxreport = (PRXREPORT)&report;

	pattrib = &precvframe->u.hdr.attrib;
	_rtw_memset(pattrib, 0, sizeof(struct rx_pkt_attrib));

	// update rx report to recv_frame attribute
	pattrib->pkt_len = (u16)prxreport->pktlen;
	pattrib->drvinfo_sz = (u8)(prxreport->drvinfosize << 3);
	pattrib->physt = (u8)prxreport->physt;

	pattrib->crc_err = (u8)prxreport->crc32;
	pattrib->icv_err = (u8)prxreport->icverr;

	pattrib->bdecrypted = (u8)(prxreport->swdec ? 0 : 1);
	pattrib->encrypt = (u8)prxreport->security;

	pattrib->qos = (u8)prxreport->qos;
	pattrib->priority = (u8)prxreport->tid;

	pattrib->amsdu = (u8)prxreport->amsdu;

	pattrib->seq_num = (u16)prxreport->seq;
	pattrib->frag_num = (u8)prxreport->frag;
	pattrib->mfrag = (u8)prxreport->mf;
	pattrib->mdata = (u8)prxreport->md;

	pattrib->mcs_rate = (u8)prxreport->rxmcs;
	pattrib->rxht = (u8)prxreport->rxht;
}

/*
 * Notice:
 *	Before calling this function,
 *	precvframe->u.hdr.rx_data should be ready!
 */
void update_recvframe_phyinfo(
	union recv_frame	*precvframe,
	struct phy_stat *pphy_status)
{
	PADAPTER 			padapter= precvframe->u.hdr.adapter;
	struct rx_pkt_attrib	*pattrib = &precvframe->u.hdr.attrib;
	HAL_DATA_TYPE		*pHalData = GET_HAL_DATA(padapter);	
	PODM_PHY_INFO_T 	pPHYInfo = (PODM_PHY_INFO_T)(&pattrib->phy_info);
	
	u8			*wlanhdr;
	ODM_PACKET_INFO_T	pkt_info;
	u8 *sa;
	//_irqL		irqL;
	struct sta_priv *pstapriv;
	struct sta_info *psta;
	
	pkt_info.bPacketMatchBSSID =_FALSE;
	pkt_info.bPacketToSelf = _FALSE;
	pkt_info.bPacketBeacon = _FALSE;


	wlanhdr = get_recvframe_data(precvframe);

	pkt_info.bPacketMatchBSSID = ((!IsFrameTypeCtrl(wlanhdr)) &&
		!pattrib->icv_err && !pattrib->crc_err &&
		_rtw_memcmp(get_hdr_bssid(wlanhdr), get_bssid(&padapter->mlmepriv), ETH_ALEN));

	pkt_info.bPacketToSelf = pkt_info.bPacketMatchBSSID && (_rtw_memcmp(get_da(wlanhdr), myid(&padapter->eeprompriv), ETH_ALEN));

	pkt_info.bPacketBeacon = pkt_info.bPacketMatchBSSID && (GetFrameSubType(wlanhdr) == WIFI_BEACON);

	if(pkt_info.bPacketBeacon){
		if(check_fwstate(&padapter->mlmepriv, WIFI_STATION_STATE) == _TRUE){				
			sa = padapter->mlmepriv.cur_network.network.MacAddress;
			#if 0
			{					
				printk("==> rx beacon from AP[%02x:%02x:%02x:%02x:%02x:%02x]\n",
					sa[0],sa[1],sa[2],sa[3],sa[4],sa[5]);					
			}
			#endif
		}
		//to do Ad-hoc
	}
	else{
		sa = get_sa(wlanhdr);
	}			
		
	pkt_info.StationID = 0xFF;
	
	pstapriv = &padapter->stapriv;
	psta = rtw_get_stainfo(pstapriv, sa);
	if (psta)
	{
      		pkt_info.StationID = psta->mac_id;
		//printk("%s ==> StationID(%d)\n",__FUNCTION__,pkt_info.StationID);
	}
	pkt_info.Rate = pattrib->mcs_rate;
		
	#ifdef CONFIG_CONCURRENT_MODE	
	//get Primary adapter's odmpriv
	if(padapter->adapter_type > PRIMARY_ADAPTER){
		pHalData = GET_HAL_DATA(padapter->pbuddy_adapter);		
	}
	#endif	
	//rtl8192c_query_rx_phy_status(precvframe, pphy_status);
	//_enter_critical_bh(&pHalData->odm_stainfo_lock, &irqL);
	ODM_PhyStatusQuery(&pHalData->odmpriv,pPHYInfo,(u8 *)pphy_status,&(pkt_info));
	//_exit_critical_bh(&pHalData->odm_stainfo_lock, &irqL);
	precvframe->u.hdr.psta = NULL;
	if (pkt_info.bPacketMatchBSSID &&
		(check_fwstate(&padapter->mlmepriv, WIFI_AP_STATE) == _TRUE))
	{
		if (psta)
		{ 
			precvframe->u.hdr.psta = psta;
			rtl8192c_process_phy_info(padapter, precvframe);
              }
	}
	else if (pkt_info.bPacketToSelf || pkt_info.bPacketBeacon)
	{
		if (check_fwstate(&padapter->mlmepriv, WIFI_ADHOC_STATE|WIFI_ADHOC_MASTER_STATE) == _TRUE)
		{
			if (psta)
			{
				precvframe->u.hdr.psta = psta;
			}
		}
		rtl8192c_process_phy_info(padapter, precvframe);             
	}
}
static void rtl8723as_recv_tasklet(void *priv)
{
	PADAPTER				padapter;
	PHAL_DATA_TYPE			pHalData;
	struct recv_priv		*precvpriv;
	struct recv_buf			*precvbuf;
	union recv_frame		*precvframe;
	struct recv_frame_hdr	*phdr;
	struct rx_pkt_attrib	*pattrib;
	u8			*ptr;
	_pkt		*ppkt;
	u32			pkt_offset;
	_irqL		irql;


	padapter = (PADAPTER)priv;
	pHalData = GET_HAL_DATA(padapter);
	precvpriv = &padapter->recvpriv;

	do {
		precvbuf = rtw_dequeue_recvbuf(&precvpriv->recv_buf_pending_queue);
		if (NULL == precvbuf) break;

		ptr = precvbuf->pdata;

		while (ptr < precvbuf->ptail)
		{
			precvframe = rtw_alloc_recvframe(&precvpriv->free_recv_queue);
			if (precvframe == NULL) {
				RT_TRACE(_module_rtl871x_recv_c_, _drv_err_, ("rtl8723as_recv_tasklet: no enough recv frame!\n"));
				rtw_enqueue_recvbuf_to_head(precvbuf, &precvpriv->recv_buf_pending_queue);

				// The case of can't allocte recvframe should be temporary,
				// schedule again and hope recvframe is available next time.
#ifdef PLATFORM_LINUX
				tasklet_schedule(&precvpriv->recv_tasklet);
#endif
				return;
			}

			phdr = &precvframe->u.hdr;
			pattrib = &phdr->attrib;

			update_recvframe_attrib(precvframe, (struct recv_stat*)ptr);
			if(pattrib->crc_err){
				DBG_8192C("%s()-%d: RX Warning! rx CRC ERROR !!\n", __FUNCTION__, __LINE__);	
				rtw_free_recvframe(precvframe, &precvpriv->free_recv_queue);
				rtw_enqueue_recvbuf_to_head(precvbuf, &precvpriv->recv_buf_pending_queue);

				// The case of can't allocte skb is serious and may never be recovered,
				// once bDriverStopped is enable, this task should be stopped.
				if (padapter->bDriverStopped == _FALSE) {
#ifdef PLATFORM_LINUX
					tasklet_schedule(&precvpriv->recv_tasklet);
#endif
				}

				return;				
			}	
			
			pkt_offset = RXDESC_SIZE + pattrib->drvinfo_sz + pattrib->pkt_len;
#if 0 // reduce check to speed up
			if ((ptr + pkt_offset) > precvbuf->ptail) {
				RT_TRACE(_module_rtl871x_recv_c_, _drv_err_,
						("%s: next pkt len(%p,%d) exceed ptail(%p)!\n",
						__FUNCTION__, ptr, pkt_offset, precvbuf->ptail));
				rtw_free_recvframe(precvframe, &precvpriv->free_recv_queue);
				break;
			}
#endif

			ppkt = skb_clone(precvbuf->pskb, GFP_ATOMIC);
			if (ppkt == NULL)
			{
				RT_TRACE(_module_rtl871x_recv_c_, _drv_crit_, ("rtl8723as_recv_tasklet: no enough memory to allocate SKB!\n"));
				rtw_free_recvframe(precvframe, &precvpriv->free_recv_queue);
				rtw_enqueue_recvbuf_to_head(precvbuf, &precvpriv->recv_buf_pending_queue);

				// The case of can't allocte skb is serious and may never be recovered,
				// once bDriverStopped is enable, this task should be stopped.
				if (padapter->bDriverStopped == _FALSE) {
#ifdef PLATFORM_LINUX
					tasklet_schedule(&precvpriv->recv_tasklet);
#endif
				}

				return;
			}

			phdr->pkt = ppkt;
			phdr->len = 0;
			phdr->rx_head = precvbuf->phead;
			phdr->rx_data = phdr->rx_tail = precvbuf->pdata;
			phdr->rx_end = precvbuf->pend;
			recvframe_put(precvframe, pkt_offset);
			recvframe_pull(precvframe, RXDESC_SIZE + pattrib->drvinfo_sz);
			if (pHalData->ReceiveConfig & RCR_APPFCS)
				recvframe_pull_tail(precvframe, IEEE80211_FCS_LEN);

			// move to drv info position
			ptr += RXDESC_SIZE;

			// update drv info
			if (pHalData->ReceiveConfig & RCR_APP_BA_SSN) {
//				rtl8723s_update_bassn(padapter, pdrvinfo);
				ptr += 4;
			}
			if (pattrib->physt)
				update_recvframe_phyinfo(precvframe, (struct phy_stat*)ptr);

			if (rtw_recv_entry(precvframe) != _SUCCESS)
			{
				RT_TRACE(_module_rtl871x_recv_c_, _drv_err_, ("rtl8723as_recv_tasklet: rtw_recv_entry(precvframe) != _SUCCESS\n"));
			}

			// Page size of receive package is 128 bytes alignment => DMA agg
			// refer to _InitTransferPageSize()
			pkt_offset = _RND128(pkt_offset);
			precvbuf->pdata += pkt_offset;
			ptr = precvbuf->pdata;
		}

		dev_kfree_skb_any(precvbuf->pskb);
		precvbuf->pskb = NULL;
		rtw_enqueue_recvbuf(precvbuf, &precvpriv->free_recv_buf_queue);
	} while (1);
}

/*
 * Initialize recv private variable for hardware dependent
 * 1. recv buf
 * 2. recv tasklet
 *
 */
s32 rtl8723as_init_recv_priv(PADAPTER padapter)
{
	s32			res;
	u32			i, n;
	struct recv_priv	*precvpriv;
	struct recv_buf		*precvbuf;


	res = _SUCCESS;
	precvpriv = &padapter->recvpriv;

	//3 1. init recv buffer
	_rtw_init_queue(&precvpriv->free_recv_buf_queue);
	_rtw_init_queue(&precvpriv->recv_buf_pending_queue);

	n = NR_RECVBUFF * sizeof(struct recv_buf) + 4;
	precvpriv->pallocated_recv_buf = rtw_zmalloc(n);
	if (precvpriv->pallocated_recv_buf == NULL) {
		res = _FAIL;
		RT_TRACE(_module_rtl871x_recv_c_, _drv_err_, ("alloc recv_buf fail!\n"));
		goto exit;
	}

	precvpriv->precv_buf = (u8*)N_BYTE_ALIGMENT((SIZE_PTR)(precvpriv->pallocated_recv_buf), 4);

	// init each recv buffer
	precvbuf = (struct recv_buf*)precvpriv->precv_buf;
	for (i = 0; i < NR_RECVBUFF; i++)
	{
		res = initrecvbuf(precvbuf, padapter);
		if (res == _FAIL)
			break;

		res = rtw_os_recvbuf_resource_alloc(padapter, precvbuf);
		if (res == _FAIL) {
			freerecvbuf(precvbuf);
			break;
		}

		rtw_list_insert_tail(&precvbuf->list, &precvpriv->free_recv_buf_queue.queue);

		precvbuf++;
	}
	precvpriv->free_recv_buf_queue_cnt = i;

	if (res == _FAIL)
		goto initbuferror;

	//3 2. init tasklet
#ifdef PLATFORM_LINUX
	tasklet_init(&precvpriv->recv_tasklet,
	     (void(*)(unsigned long))rtl8723as_recv_tasklet,
	     (unsigned long)padapter);
#endif

	goto exit;

initbuferror:
	precvbuf = (struct recv_buf*)precvpriv->precv_buf;
	if (precvbuf) {
		n = precvpriv->free_recv_buf_queue_cnt;
		precvpriv->free_recv_buf_queue_cnt = 0;
		for (i = 0; i < n ; i++)
		{
			rtw_list_delete(&precvbuf->list);
			rtw_os_recvbuf_resource_free(padapter, precvbuf);
			freerecvbuf(precvbuf);
			precvbuf++;
		}
		precvpriv->precv_buf = NULL;
	}

	if (precvpriv->pallocated_recv_buf) {
		n = NR_RECVBUFF * sizeof(struct recv_buf) + 4;
		rtw_mfree(precvpriv->pallocated_recv_buf, n);
		precvpriv->pallocated_recv_buf = NULL;
	}

exit:
	return res;
}

/*
 * Free recv private variable of hardware dependent
 * 1. recv buf
 * 2. recv tasklet
 *
 */
void rtl8723as_free_recv_priv(PADAPTER padapter)
{
	u32			i, n;
	struct recv_priv	*precvpriv;
	struct recv_buf		*precvbuf;


	precvpriv = &padapter->recvpriv;

	//3 1. kill tasklet
#ifdef PLATFORM_LINUX
	tasklet_kill(&precvpriv->recv_tasklet);
#endif

	//3 2. free all recv buffers
	precvbuf = (struct recv_buf*)precvpriv->precv_buf;
	if (precvbuf) {
		n = NR_RECVBUFF;
		precvpriv->free_recv_buf_queue_cnt = 0;
		for (i = 0; i < n ; i++)
		{
			rtw_list_delete(&precvbuf->list);
			rtw_os_recvbuf_resource_free(padapter, precvbuf);
			freerecvbuf(precvbuf);
			precvbuf++;
		}
		precvpriv->precv_buf = NULL;
	}

	if (precvpriv->pallocated_recv_buf) {
		n = NR_RECVBUFF * sizeof(struct recv_buf) + 4;
		rtw_mfree(precvpriv->pallocated_recv_buf, n);
		precvpriv->pallocated_recv_buf = NULL;
	}
}

