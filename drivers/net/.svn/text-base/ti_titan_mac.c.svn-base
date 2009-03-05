/*
 * CPMAC driver for the MIPS TI TITAN
 *
 * Copyright: (C) 2006 Texas Instruments Inc.
 *
 * Author: Manish Lachwani <mlachwani@mvista.com>
 * Copyright: (C) 2006 MontaVista Software Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
	
#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/errno.h>
#include <linux/in.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/highmem.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <asm/irq.h>            /* For NR_IRQS only. */
#include <asm/bitops.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/semaphore.h>
	
#include "ti_titan_mac.h"
	
#define CONFIG_CPMAC_NOPHY                                    9999
	
MODULE_AUTHOR("Maintainer: Manish Lachwani <mlachwani@mvista.com>");
MODULE_DESCRIPTION("Driver for TI CPMAC/CPGMAC");
	
static int cfg_link_speed = 0;
MODULE_PARM(cfg_link_speed, "i");
MODULE_PARM_DESC(cfg_link_speed, "Fixed speed of the Link: <100/10>");
	
static char *cfg_link_mode = NULL;
MODULE_PARM(cfg_link_mode, "1-3s");
MODULE_PARM_DESC(cfg_link_mode, "Fixed mode of the Link: <fd/hd>");
	
int debug_mode = 0;
MODULE_PARM(debug_mode, "i");
MODULE_PARM_DESC(debug_mode, "Turn on the debug info: <0/1>. Default is 0 (off)");
	
#define CPMAC_DDA_MAJOR_VERSION         2
#define CPMAC_DDA_MINOR_VERSION         0
	
#define CPMAC_MODULE_VERSION "2.0" 
MODULE_VERSION(CPMAC_MODULE_VERSION);
	
const char cpmac_DDA_version_string[] = "CPMAC Linux DDA version 2.0";

int cpmac_link_status = 0;
int cpmac_debug_mode = 0;
static int g_cfg_start_link_params = CFG_START_LINK_SPEED;
static int g_init_enable_flag = 0;
	
static struct net_device *cpmac_net_dev[CPMAC_MAX_INSTANCES] = {NULL, NULL, NULL, NULL, NULL, NULL};
	
static struct net_device *last_cpmac_device = NULL;
static int cpmac_devices_installed = 0;  
	
int avalanche_is_cpmac_on_vbus(void);

/*
 * Forward Declarations
 */
int cpsw_halcommon_mii_mdio_getloopback(phy_device *phy_dev);
int cpsw_halcommon_mii_mdio_getlinked(phy_device *phy_dev);
int cpsw_halcommon_mii_mdio_get_duplex(phy_device *phy_dev);
int cpsw_halcommon_mii_mdio_getphy_num(phy_device *phy_dev);
int cpsw_halcommon_mii_mdio_tic(phy_device *phy_dev);
int cpsw_halcommon_mii_mdio_get_speed(phy_device *phy_dev);
void cpsw_halcommon_mii_mdio_setphymode(phy_device *phy_dev,unsigned int phy_mode);
	
extern int cpmac_poll(struct net_device *p_dev, int *budget);
static struct proc_dir_entry *gp_stats_file = NULL;
static int __devinit cpmac_probe(struct device *dev)
{
	return 0;
}
	
static struct device_driver cpmac_driver = {
	.name		= "cpmac",
	.bus		= &platform_bus_type,
	.probe		= cpmac_probe,
	.remove		= NULL, /* TODO: Findout when probe would be called. */
	.suspend	= NULL,
	.resume		= NULL,
};
	
static char cpmac_cfg[CPMAC_MAX_INSTANCES][200];
	
#define LOW_CPMAC   0x00001
#define HIGH_CPMAC  0x00002
#define EXT_SWITCH  0x10000
	
static int cpmac_cfg_build(int connect, int external_switch)
{
	unsigned int BaseAddr = 0;
	unsigned int intrLine = 0;
	unsigned int ResetLine = 0;
	unsigned int BusFreq = 0;
	unsigned int speed = 0;
	unsigned int mdioPhyMask = 0;
	unsigned int TxNumBD = 0;
	unsigned int TxServiceMax= 0;
	unsigned int RxNumBD = 0;
	unsigned int RxServiceMax = 0;
	static int cfg_instance = 0;
	
	switch(connect) {
	case LOW_CPMAC:    
		BaseAddr = AVALANCHE_LOW_CPMAC_BASE;
		intrLine = AVALANCHE_LOW_CPMAC_INT;
		ResetLine = AVALANCHE_LOW_CPMAC_RESET_BIT;
		BusFreq = avalanche_is_cpmac_on_vbus( ) ? 
			avalanche_clkc_getfreq(CLKC_VBUS) : avalanche_clkc_getfreq(CLKC_SYS);
		speed = external_switch ?  CONFIG_CPMAC_NOPHY: 0;
		mdioPhyMask = AVALANCHE_LOW_CPMAC_PHY_MASK;
		TxNumBD = LOW_CPMAC_DDA_DEFAULT_TX_NUM_BD;
		RxNumBD = LOW_CPMAC_DDA_DEFAULT_RX_NUM_BD;
		TxServiceMax = LOW_CPMAC_DDA_DEFAULT_TX_MAX_SERVICE;
		RxServiceMax = LOW_CPMAC_DDA_DEFAULT_RX_MAX_SERVICE;
		
		 break;
	
	case HIGH_CPMAC:
		BaseAddr = AVALANCHE_HIGH_CPMAC_BASE;
		intrLine = AVALANCHE_HIGH_CPMAC_INT;
		ResetLine = AVALANCHE_HIGH_CPMAC_RESET_BIT;
		BusFreq = avalanche_is_cpmac_on_vbus( ) ? 
			avalanche_clkc_getfreq(CLKC_VBUS) : avalanche_clkc_getfreq(CLKC_SYS);
		speed = external_switch ?  CONFIG_CPMAC_NOPHY: 0;
		mdioPhyMask = AVALANCHE_HIGH_CPMAC_PHY_MASK;
		TxNumBD = HIGH_CPMAC_DDA_DEFAULT_TX_NUM_BD;
		RxNumBD = HIGH_CPMAC_DDA_DEFAULT_RX_NUM_BD;
		TxServiceMax = HIGH_CPMAC_DDA_DEFAULT_TX_MAX_SERVICE;
		RxServiceMax = HIGH_CPMAC_DDA_DEFAULT_RX_MAX_SERVICE;
		
		break;
	
	default: return (-1);
	}
	
	sprintf(cpmac_cfg[cfg_instance], "%d:%x:%d:%d:%u:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%x:%d:%d:%u:%u:%x:%d",
	cfg_instance, BaseAddr, intrLine, ResetLine, BusFreq, speed, 0, CPMAC_DEFAULT_PROMISCOUS_ENABLE,
	CPMAC_DEFAULT_BROADCAST_ENABLE, CPMAC_DEFAULT_MULTICAST_ENABLE, CPMAC_DDA_DEFAULT_MAX_FRAME_SIZE,
	TxNumBD, TxServiceMax, RxNumBD, RxServiceMax, 0, AVALANCHE_MDIO_BASE, 0, AVALANCHE_MDIO_RESET_BIT,
	avalanche_clkc_getfreq(CLKC_VBUS),
	2200000, mdioPhyMask, 10);
	
	cfg_instance++;
	
	return (0);
}
	
static int cpmac_cfg_probe(void)
{
	int external_switch = 0;
	
#if defined (CONFIG_AVALANCHE_LOW_CPMAC)
	external_switch = AVALANCHE_LOW_CPMAC_HAS_EXT_SWITCH;
	if(cpmac_cfg_build(LOW_CPMAC, external_switch))
		return (-1);
#endif
	
#if defined (CONFIG_AVALANCHE_HIGH_CPMAC)
	external_switch = AVALANCHE_HIGH_CPMAC_HAS_EXT_SWITCH;
	if(cpmac_cfg_build(HIGH_CPMAC, external_switch))
		return (-1);
#endif
	
	return (0);
}
	
static char cpmac_L3_align[] = {0x02, 0x01, 0x00, 0x03};
#define CPMAC_L3_ALIGN(size) cpmac_L3_align[(size) & 0x3]
	
char cpmac_4byte_align[] = {0x0, 0x03, 0x02, 0x1};
#define CPMAC_4BYTE_ALIGN(size) cpmac_4byte_align[(size) & 0x3]
	
static int cpmac_dev_init(struct net_device *p_dev);
static int cpmac_dev_open( struct net_device *dev );
static int cpmac_ioctl (struct net_device *p_dev, struct ifreq *rq, int cmd);
static int cpmac_dev_close(struct net_device *p_dev);
static void cpmac_dev_mcast_set(struct net_device *p_dev);
static void cpmac_tx_timeout(struct net_device *p_dev);
static struct net_device_stats *cpmac_dev_get_net_stats (struct net_device *dev);
	
static int  __init cpmac_p_detect_manual_cfg(int, char*, int);
static int cpmac_p_read_stats(char* buf, char **start, off_t offset, int count, int *eof, void *data);
static int cpmac_p_write_stats (struct file *fp, const char * buf, unsigned long count, void * data);
static int cpmac_p_read_link(char *buf, char **start, off_t offset, int count, int *eof, void *data);
static int cpmac_dump_config(char *buf, char **start, off_t offset, int count, int *eof, void *data);
static int cpmac_p_get_version(char *buf, char **start, off_t offset, int count, int *eof, void *data);
static int cpmac_p_update_statistics(struct net_device *p_dev, char *buf, int limit, int *p_len);
static int cpmac_p_reset_statistics (struct net_device *p_dev);
static int cpmac_p_read_rfc2665_stats(char* buf, char **start, off_t offset, int count, int *eof, void *data);
static int cpmac_p_dev_enable(CpmacNetDevice * hDDA);
static int cpmac_p_dev_disable(CpmacNetDevice * hDDA);
static void cpmac_p_tick_timer_expiry(CpmacNetDevice * hDDA);
static int cpmac_dev_set_mac_addr(struct net_device *p_dev,void * addr);
	
typedef void (*timer_tick_func)(unsigned long);
static int DDA_cpmac_control_cb(CpmacNetDevice* hDDA, int cmd, void* cmdArg, void* param);
static CpmacDDACbIf cpmac_DDA_cb_interface = 
{ 
	{                      
		{                    
			(DDA_ControlCb) DDA_cpmac_control_cb    
		},
		NULL,      
		(DDA_NetAllocRxBufCb) DDA_cpmac_net_alloc_rx_buf,
		(DDA_NetFreeRxBufCb) DDA_cpmac_net_free_rx_buf, 
#ifndef CPMAC_MULTIPACKET_RX_COMPLETE_NOTIFY
		(DDA_NetRxCb)DDA_cpmac_net_rx,        
#else
		NULL,
#endif
#ifdef CPMAC_MULTIPACKET_RX_COMPLETE_NOTIFY
		(DDA_NetRxMultipleCb)DDA_cpmac_net_rx_multiple_cb,
#else
		NULL,                                         
#endif
		(DDA_NetTxCompleteCb)DDA_cpmac_net_tx_complete 
	},          
	(DDA_Printf) printk,    
	(DDA_ErrLog) printk,      
};
	
static unsigned char cpmac_str_to_hexnum(unsigned char c)
{
	if(c >= '0' && c <= '9') 
		return c - '0';

	if(c >= 'a' && c <= 'f') 
		return c - 'a' + 10;

	if(c >= 'A' && c <= 'F') return c - 'A' + 10;
		return 0; 
}
	
static void cpmac_str_to_ethaddr(unsigned char *ea, unsigned char *str)
{
	int i;
	unsigned char num;
	for(i = 0; i < 6; i++) 
	{
		if((*str == '.') || (*str == ':')) 
			str++;
		num = cpmac_str_to_hexnum(*str++) << 4;
		num |= (cpmac_str_to_hexnum(*str++));
		ea[i] = num;
	}
}
	
static int DDA_cpmac_control_cb(CpmacNetDevice* hDDA, int cmd, void* cmdArg, void* param)
{
	switch (cmd)
	{
		case CPMAC_DDA_IOCTL_TIMER_START:
			{
				struct timer_list *p_timer = &hDDA->periodicTimer;
				hDDA->periodicTicks = (CPMAC_DDA_TICKS_PER_SEC * (int)cmdArg) / 1000;
				p_timer->expires = jiffies + hDDA->periodicTicks;
				add_timer(&hDDA->periodicTimer);
				hDDA->timerActive = 1;
			}
			break;
	
		case CPMAC_DDA_IOCTL_TIMER_STOP:
			{
				if (hDDA->timerActive == 1)
				{
					del_timer_sync(&hDDA->periodicTimer);
					hDDA->timerActive = 0;
				}
			}
			break;
	
		case CPMAC_DDA_IOCTL_STATUS_UPDATE:
			{   
				struct net_device *p_dev = hDDA->owner;
				CpmacDDCStatus *status = &hDDA->ddcStatus;
				hDDA->ddcStatus = *((CpmacDDCStatus *)cmdArg);
				if ((status->hwStatus & CPMAC_DDC_TX_HOST_ERROR) == CPMAC_DDC_TX_HOST_ERROR)
				{
					printk("DDA_cpmac_control_cb: TX Host Error. Transmit Stopped %s\n", p_dev->name);
				} 
				if ((status->hwStatus & CPMAC_DDC_RX_HOST_ERROR) == CPMAC_DDC_RX_HOST_ERROR)
				{
					printk("DDA_cpmac_control_cb: RX Host Error. Receive Stopped %s\n", p_dev->name);
				} 
				if (status->PhyLinked)
				{	
					if(!netif_carrier_ok(p_dev))
					{
						netif_carrier_on(p_dev);
					}
					hDDA->linkSpeed = ((status->PhySpeed) ? 100000000 : 10000000);
					hDDA->linkMode  = ((status->PhyDuplex) ? 3 : 2);
	
					if(netif_running(p_dev) && netif_queue_stopped(p_dev))
					{
						netif_wake_queue(p_dev);
					}
				}
				else
				{
					if(netif_carrier_ok(p_dev))
					{
						hDDA->linkSpeed = 100000000;
						hDDA->linkMode  = 1;
						netif_carrier_off(p_dev);	
					}
	
					if(!netif_queue_stopped(p_dev))
					{
						netif_stop_queue(p_dev);  
					}
				}
			}
			break;
	
			case CPMAC_DDA_IOCTL_MIB64_CNT_TIMER_START:
					{
						struct timer_list *p_timer = &hDDA->mibTimer;
						hDDA->mibTicks = (CPMAC_DDA_TICKS_PER_SEC * (int)cmdArg) / 1000;
						p_timer->expires = jiffies + hDDA->mibTicks;
						add_timer(p_timer);
						hDDA->mibTimerActive = 1;
			}
			break;
	
			case CPMAC_DDA_IOCTL_MIB64_CNT_TIMER_STOP:
			{
				if (hDDA->mibTimerActive == 1)
				{
					del_timer_sync(&hDDA->mibTimer);
					hDDA->mibTimerActive = 0;
				}
			}
			break;
	
			default:
				break;
		}   
	
		return (CPMAC_SUCCESS);
}

static int cpmac_net_get_config(CpmacNetDevice *hDDA)
{
#define CPMAC_TOKEN_PARSE(str)	{ if ((tok = (char *)strsep ((str), ":")) == NULL) return -1; }
#define CPMAC_TOKEN_GET_INTEGER	simple_strtoul (tok, NULL, 10)
#define CPMAC_TOKEN_GET_HEX	simple_strtoul (tok, NULL, 16)
	
	{
		CpmacInitConfig *iCfg = &hDDA->initCfg;
		CpmacChInfo     *txChCfg = &hDDA->txChInfo[0];
		CpmacChInfo     *rxChCfg = &hDDA->rxChInfo[0];
		int             speed, duplex, extra;
		char            localStringVal[200];
		char            *localString = NULL;
		char            *tok;
		char            *pHolder = NULL;
	
		char *instanceName = NULL;
		switch(hDDA->instanceNum)
		{
			case 0: 
				instanceName = CPMAC_DDA_CONFIG_A; 
				break;

			case 1: 
				instanceName = CPMAC_DDA_CONFIG_B; 
				break;

			case 2: 
				instanceName = CPMAC_DDA_CONFIG_C; 
				break;

			case 3: 
				instanceName = CPMAC_DDA_CONFIG_D; 
				break;

			case 4: 
				instanceName = CPMAC_DDA_CONFIG_E; 
				break;

			case 5: 
				instanceName = CPMAC_DDA_CONFIG_F; 
				break;
	
			default: instanceName=""; 
				break;
		}
		localString = (char *) prom_getenv(instanceName);
	
		if(localString == NULL) {
			printk("Error getting CPMAC Configuration params for instance:%d\n",hDDA->instanceNum);
			printk("Environment Variable:%s not set in bootloader\n",instanceName);
			printk("Setting Default configuration params for CPMAC instance:%d\n",hDDA->instanceNum);
	
			switch(hDDA->instanceNum) {
			case 0: 
				localString = cpmac_cfg[0]; 
				break;

			case 1: 
				localString = cpmac_cfg[1]; 
				break;

			default: 
				localString= cpmac_cfg[0]; 
				break;
			}
		}

		strcpy(&localStringVal[0], localString);
		localString = &localStringVal[0];
		pHolder = NULL;
		tok = (char *)strsep (&localString, ":");

		if (tok == NULL) 
			return (-1);
	
		iCfg->instId = CPMAC_TOKEN_GET_INTEGER; CPMAC_TOKEN_PARSE(&localString);
		iCfg->baseAddress = CPMAC_TOKEN_GET_HEX; CPMAC_TOKEN_PARSE(&localString);
		iCfg->intrLine = CPMAC_TOKEN_GET_INTEGER; CPMAC_TOKEN_PARSE(&localString);
		iCfg->resetLine = CPMAC_TOKEN_GET_INTEGER; CPMAC_TOKEN_PARSE(&localString);
		iCfg->cpmacBusFrequency = CPMAC_TOKEN_GET_INTEGER; CPMAC_TOKEN_PARSE(&localString);
		speed = CPMAC_TOKEN_GET_INTEGER; CPMAC_TOKEN_PARSE(&localString);
		duplex = CPMAC_TOKEN_GET_INTEGER; CPMAC_TOKEN_PARSE(&localString);
		iCfg->rxCfg.promiscousEnable = CPMAC_TOKEN_GET_INTEGER; CPMAC_TOKEN_PARSE(&localString);
		iCfg->rxCfg.broadcastEnable = CPMAC_TOKEN_GET_INTEGER; CPMAC_TOKEN_PARSE(&localString);
		iCfg->rxCfg.multicastEnable = CPMAC_TOKEN_GET_INTEGER; CPMAC_TOKEN_PARSE(&localString);
		iCfg->rxCfg.maxRxPktLength = CPMAC_TOKEN_GET_INTEGER; CPMAC_TOKEN_PARSE(&localString);
		txChCfg->numBD = CPMAC_TOKEN_GET_INTEGER; CPMAC_TOKEN_PARSE(&localString);
		txChCfg->serviceMax = CPMAC_TOKEN_GET_INTEGER; CPMAC_TOKEN_PARSE(&localString);
		rxChCfg->numBD = CPMAC_TOKEN_GET_INTEGER; CPMAC_TOKEN_PARSE(&localString);
		rxChCfg->serviceMax = CPMAC_TOKEN_GET_INTEGER; CPMAC_TOKEN_PARSE(&localString);
		extra = CPMAC_TOKEN_GET_INTEGER; CPMAC_TOKEN_PARSE(&localString);
		iCfg->mdioBaseAddress =  CPMAC_TOKEN_GET_HEX; CPMAC_TOKEN_PARSE(&localString);
		iCfg->mdioIntrLine =  CPMAC_TOKEN_GET_INTEGER; CPMAC_TOKEN_PARSE(&localString);
		iCfg->mdioResetLine =  CPMAC_TOKEN_GET_INTEGER; CPMAC_TOKEN_PARSE(&localString);
		iCfg->MdioBusFrequency = CPMAC_TOKEN_GET_INTEGER; CPMAC_TOKEN_PARSE(&localString);
		iCfg->MdioClockFrequency = CPMAC_TOKEN_GET_INTEGER; CPMAC_TOKEN_PARSE(&localString);
		iCfg->PhyMask = CPMAC_TOKEN_GET_HEX; CPMAC_TOKEN_PARSE(&localString);
		iCfg->MdioTickMSec = CPMAC_TOKEN_GET_INTEGER;
		iCfg->Mib64CntMsec = CONFIG_CPMAC_MIB_TIMER_TIMEOUT;
		rxChCfg->bufSize = iCfg->rxCfg.maxRxPktLength;
		hDDA->rxBufOffset = CPMAC_L3_ALIGN(extra);
		hDDA->rxBufSize =   (rxChCfg->bufSize + hDDA->rxBufOffset); 
		hDDA->rxBufSize += CPMAC_4BYTE_ALIGN(hDDA->rxBufSize);
		hDDA->napiRxTx.rxMaxService = rxChCfg->serviceMax;
		hDDA->napiRxTx.txMaxService = txChCfg->serviceMax;
				
		if (speed == CONFIG_CPMAC_NOPHY) {
			iCfg->phyMode = SNWAY_NOPHY;
		}
		else {
			if ((speed == 0) && (duplex == 0)) {  
				iCfg->phyMode = SNWAY_AUTOALL;  /* Auto detection */
			}
			else if (speed == 10) {
				if (duplex == 1) { 
					iCfg->phyMode = SNWAY_HD10; 
				}
				else if (duplex == 2) { 
					iCfg->phyMode = SNWAY_FD10;
				}
				else { 
					iCfg->phyMode = SNWAY_HD10 | SNWAY_FD10; 
				}
			}
			else if (speed == 100) {
				if (duplex == 1) { 
					iCfg->phyMode = SNWAY_HD100; 
				}
				else if (duplex == 2) { 
					iCfg->phyMode = SNWAY_FD100; 
				}
				else { 
					iCfg->phyMode = SNWAY_HD100 | SNWAY_FD100; 
				}
			}
			else {
				if (duplex == 1) { 
					iCfg->phyMode = SNWAY_HD10 | SNWAY_HD100 ; 
				}
				else { 
					iCfg->phyMode = SNWAY_FD10 | SNWAY_FD100; 
				}
			}
		}
	
		hDDA->vlanEnable                    = CPMAC_DDA_DEFAULT_VLAN_ENABLE;
		iCfg->numTxChannels                 = CPMAC_DDA_DEFAULT_NUM_TX_CHANNELS;
		iCfg->numRxChannels                 = CPMAC_DDA_DEFAULT_NUM_RX_CHANNELS;
		iCfg->MLinkMask                     = CPMAC_DEFAULT_MLINK_MASK;
		iCfg->rxCfg.passCRC                 = CPMAC_DEFAULT_PASS_CRC;
		iCfg->rxCfg.qosEnable               = CPMAC_DEFAULT_QOS_ENABLE;
		iCfg->rxCfg.noBufferChaining        = CPMAC_DEFAULT_NO_BUFFER_CHAINING;
		iCfg->rxCfg.copyMACControlFramesEnable = CPMAC_DEFAULT_COPY_MAC_CONTROL_FRAMES_ENABLE;
		iCfg->rxCfg.copyShortFramesEnable   = CPMAC_DEFAULT_COPY_SHORT_FRAMES_ENABLE;
		iCfg->rxCfg.copyErrorFramesEnable   = CPMAC_DEFAULT_COPY_ERROR_FRAMES_ENABLE;
		iCfg->rxCfg.promiscousChannel       = CPMAC_DEFAULT_PROMISCOUS_CHANNEL;
		iCfg->rxCfg.broadcastChannel        = CPMAC_DEFAULT_BROADCAST_CHANNEL;
		iCfg->rxCfg.multicastChannel        = CPMAC_DEFAULT_MULTICAST_CHANNEL;
		iCfg->rxCfg.bufferOffset            = CPMAC_DEFAULT_BUFFER_OFFSET;
	
#if (AVALANCHE_CPMAC_HW_MODULE_REV > 0x000c0100)
		iCfg->macCfg.gigForce               = CPMAC_DEFAULT_GIGFORCE;
		iCfg->macCfg.rxFifoFlowEnable       = CPMAC_DEFAULT_RX_FIFO_FLOW_ENABLE;
	
#endif
		iCfg->macCfg.pType                  = CPMAC_TXPRIO_FIXED;
		iCfg->macCfg.txShortGapEnable       = 0;
		if (speed == 1000) 
			iCfg->macCfg.gigaBitEnable      = 1;
		else 
			iCfg->macCfg.gigaBitEnable      = 0;

		iCfg->macCfg.txPacingEnable         = CPMAC_DEFAULT_TX_PACING_ENABLE(hDDA->instanceNum);
		iCfg->macCfg.miiEnable              = CPMAC_DEFAULT_MII_ENABLE;
		iCfg->macCfg.txFlowEnable           = CPMAC_DEFAULT_TX_FLOW_ENABLE;
		iCfg->macCfg.rxFlowEnable           = CPMAC_DEFAULT_RX_FLOW_ENABLE;
		iCfg->macCfg.loopbackEnable         = CPMAC_DEFAULT_LOOPBACK_ENABLE;
		iCfg->macCfg.fullDuplexEnable       = CPMAC_DEFAULT_FULL_DUPLEX_ENABLE;
	
		{	
			char *tx_threshold_ptr;
			int threshold;
	
			switch(hDDA->instanceNum) {
			case 0: 
				tx_threshold_ptr = prom_getenv("threshold0"); 
				break;
			case 1: 
				tx_threshold_ptr = prom_getenv("threshold1"); 
				break;
			default: 
				tx_threshold_ptr = prom_getenv("threshold0");
				break;
			}
	
			if(tx_threshold_ptr) {
				threshold = simple_strtol(tx_threshold_ptr, (char **)NULL, 10);
				if( threshold <= 0 ) {
					iCfg->macCfg.txInterruptDisable     = 0;
					iCfg->macCfg.txIntThresholdValue    = CPMAC_DEFAULT_TX_THRESHOLD_VALUE;
				}
				else {
					iCfg->macCfg.txInterruptDisable     = 1;
					iCfg->macCfg.txIntThresholdValue    = threshold;
				}
			}
			else {
				iCfg->macCfg.txInterruptDisable     = CPMAC_DEFAULT_TX_INTERRUPT_DISABLE;
				iCfg->macCfg.txIntThresholdValue    = CPMAC_DEFAULT_TX_THRESHOLD_VALUE;
			}
		
			if(iCfg->macCfg.txInterruptDisable)
				printk("\n MAC CFG : Tx interrupts off , threshold : %d \n",iCfg->macCfg.txIntThresholdValue);
			else
				printk("\n MAC CFG : Tx interrupts on\n");
		}
	
		txChCfg->chNum                      = CPMAC_DDA_DEFAULT_TX_CHANNEL;
		txChCfg->chDir                      = DDC_NET_CH_DIR_TX;
		txChCfg->chState                    = DDC_NET_CH_UNINITIALIZED;
		rxChCfg->chNum                      = CPMAC_DDA_DEFAULT_RX_CHANNEL;
		rxChCfg->chDir                      = DDC_NET_CH_DIR_RX;
		rxChCfg->chState                    = DDC_NET_CH_UNINITIALIZED;
	}
	return (0);
}
	
static int  __init cpmac_p_detect_manual_cfg(int linkSpeed, char* linkMode, int debug)
{
	char *pSpeed = NULL;
	
	if(debug == 1) {
		cpmac_debug_mode = 1;
	}
	
	if(!linkSpeed && !linkMode) {
		return (0);
	}
	
	if(!linkSpeed || (linkSpeed != 10 && linkSpeed != 100)) {
		pSpeed = "auto";
	}
	
	if((linkSpeed == 10) && (strcmp(linkMode,"fd"))) {
		g_cfg_start_link_params = SNWAY_FD10;
	}
	else if((linkSpeed == 10) && (strcmp(linkMode,"hd"))) {
		g_cfg_start_link_params = SNWAY_HD10;
	}
	else if((linkSpeed == 100) && (strcmp(linkMode,"hd"))) {
		g_cfg_start_link_params = SNWAY_HD100;
	}
	else if((linkSpeed == 100) && (strcmp(linkMode,"fd"))) {
		g_cfg_start_link_params = SNWAY_FD100;
	}
	
	return(0);
}
	
static int cpmac_p_read_link(char *buf, char **start, off_t offset, int count, int *eof, void *data)
{
	int len = 0;
	struct net_device *p_dev;
	CpmacNetDevice *hDDA;
	struct net_device *cpmac_dev_list[cpmac_devices_installed];
	int i;
	
	len += sprintf(buf+len, "CPMAC devices = %d\n",cpmac_devices_installed);
	p_dev = last_cpmac_device;
	
	for(i=0; i< cpmac_devices_installed; i++) {
		cpmac_dev_list[cpmac_devices_installed -(i+1)] = p_dev;
		hDDA = netdev_priv(p_dev);
		p_dev = hDDA->nextDevice;
	}
	
	for(i=0; i< cpmac_devices_installed; i++) {
		p_dev = cpmac_dev_list[i];
		hDDA = netdev_priv(p_dev);
	
		if(netif_carrier_ok(p_dev)) {
			len += sprintf(buf+len,"eth%d: Link State: %s Phy %d, Speed = %s, Duplex = %s\n",
				hDDA->instanceNum, "UP", hDDA->ddcStatus.PhyNum, 
				(hDDA->linkSpeed == 100000000) ? "100":"10",
				(hDDA->linkMode  == 2) ? "Half":"Full");
		}
		else {
			len += sprintf(buf+len,"eth%d: Link State: DOWN\n",hDDA->instanceNum);
		}
		p_dev = hDDA->nextDevice;
	}
	
	return len;
	
}
	
static int cpmac_dump_config(char *buf, char **start, off_t offset, int count, int *eof, void *data)
	{
	int len = 0;
	struct net_device *p_dev;
	CpmacNetDevice *hDDA;
	struct net_device *cpmac_dev_list[cpmac_devices_installed];
	int i;
	
	len += sprintf(buf+len, "CPMAC devices = %d\n",cpmac_devices_installed);
	p_dev = last_cpmac_device;
	
	for(i=0; i< cpmac_devices_installed; i++) {
		cpmac_dev_list[cpmac_devices_installed -(i+1)] = p_dev;
		hDDA = netdev_priv(p_dev);
		p_dev = hDDA->nextDevice;
	}
	
	for(i=0; i< cpmac_devices_installed; i++) {
		p_dev = cpmac_dev_list[i];
		hDDA = netdev_priv(p_dev);
	
		len += sprintf(buf+len,"\nCPMAC Driver internal Config Info for Unit %d\n",hDDA->instanceNum);
		len += sprintf(buf+len,"vlanEnable         = %d\n", hDDA->vlanEnable);
		len += sprintf(buf+len,"rxBufSize          = %d\n", hDDA->rxBufSize);
		len += sprintf(buf+len,"rxBufOffset        = %d\n", hDDA->rxBufOffset);
		len += sprintf(buf+len,"instId             = %d\n", hDDA->initCfg.instId);
		len += sprintf(buf+len,"numTxChannels      = %d\n", hDDA->initCfg.numTxChannels);
		len += sprintf(buf+len,"numRxChannels      = %d\n", hDDA->initCfg.numRxChannels);
		len += sprintf(buf+len,"cpmacBusFrequency  = %d\n", hDDA->initCfg.cpmacBusFrequency);
		len += sprintf(buf+len,"baseAddress        = %08X\n", hDDA->initCfg.baseAddress);
		len += sprintf(buf+len,"intrLine           = %d\n", hDDA->initCfg.intrLine);
		len += sprintf(buf+len,"resetLine          = %d\n", hDDA->initCfg.resetLine);
		len += sprintf(buf+len,"mdioBaseAddress    = %08X\n", hDDA->initCfg.mdioBaseAddress);
		len += sprintf(buf+len,"mdioResetLine      = %d\n", hDDA->initCfg.mdioResetLine);
		len += sprintf(buf+len,"mdioIntrLine       = %d\n", hDDA->initCfg.mdioIntrLine);
		len += sprintf(buf+len,"PhyMask            = %08X\n", hDDA->initCfg.PhyMask);
		len += sprintf(buf+len,"MLinkMask          = %08X\n", hDDA->initCfg.MLinkMask);
		len += sprintf(buf+len,"MdioBusFrequency   = %d\n", hDDA->initCfg.MdioBusFrequency);
		len += sprintf(buf+len,"MdioClockFrequency = %d\n", hDDA->initCfg.MdioClockFrequency);
		len += sprintf(buf+len,"MdioTickMSec       = %d\n", hDDA->initCfg.MdioTickMSec);
		len += sprintf(buf+len,"phyMode            = %d\n", hDDA->initCfg.phyMode);
		len += sprintf(buf+len,"passCRC            = %d\n", hDDA->initCfg.rxCfg.passCRC);
		len += sprintf(buf+len,"qosEnable          = %d\n", hDDA->initCfg.rxCfg.qosEnable);
		len += sprintf(buf+len,"noBufferChaining   = %d\n", hDDA->initCfg.rxCfg.noBufferChaining);
		len += sprintf(buf+len,"copyMACCntrlFrsEne = %d\n", hDDA->initCfg.rxCfg.copyMACControlFramesEnable);
		len += sprintf(buf+len,"copyShortFramesEn  = %d\n", hDDA->initCfg.rxCfg.copyShortFramesEnable);
		len += sprintf(buf+len,"copyErrorFramesEn  = %d\n", hDDA->initCfg.rxCfg.copyErrorFramesEnable);
		len += sprintf(buf+len,"promiscousEnable   = %d\n", hDDA->initCfg.rxCfg.promiscousEnable);
		len += sprintf(buf+len,"promiscousChannel  = %d\n", hDDA->initCfg.rxCfg.promiscousChannel);
		len += sprintf(buf+len,"broadcastEnable    = %d\n", hDDA->initCfg.rxCfg.broadcastEnable);
		len += sprintf(buf+len,"broadcastChannel   = %d\n", hDDA->initCfg.rxCfg.broadcastChannel);
		len += sprintf(buf+len,"multicastEnable    = %d\n", hDDA->initCfg.rxCfg.multicastEnable);
		len += sprintf(buf+len,"multicastChannel   = %d\n", hDDA->initCfg.rxCfg.multicastChannel);
		len += sprintf(buf+len,"maxRxPktLength     = %d\n", hDDA->initCfg.rxCfg.maxRxPktLength);
		len += sprintf(buf+len,"bufferOffset       = %d\n", hDDA->initCfg.rxCfg.bufferOffset);
	
#if (AVALANCHE_CPMAC_HW_MODULE_REV > 0x000c0100)
		len += sprintf(buf+len,"gigForce           = %d\n", hDDA->initCfg.macCfg.gigForce);
		len += sprintf(buf+len,"rxFifoFlowEnable   = %d\n", hDDA->initCfg.macCfg.rxFifoFlowEnable);
#endif
	
		len += sprintf(buf+len,"pType              = %d\n", hDDA->initCfg.macCfg.pType);
		len += sprintf(buf+len,"txShortGapEnable   = %d\n", hDDA->initCfg.macCfg.txShortGapEnable);
		len += sprintf(buf+len,"gigaBitEnable      = %d\n", hDDA->initCfg.macCfg.gigaBitEnable);
		len += sprintf(buf+len,"txPacingEnable     = %d\n", hDDA->initCfg.macCfg.txPacingEnable);
		len += sprintf(buf+len,"miiEnable          = %d\n", hDDA->initCfg.macCfg.miiEnable);
		len += sprintf(buf+len,"txFlowEnable       = %d\n", hDDA->initCfg.macCfg.txFlowEnable);
		len += sprintf(buf+len,"rxFlowEnable       = %d\n", hDDA->initCfg.macCfg.rxFlowEnable);
		len += sprintf(buf+len,"loopbackEnable     = %d\n", hDDA->initCfg.macCfg.loopbackEnable);
		len += sprintf(buf+len,"fullDuplexEnable   = %d\n", hDDA->initCfg.macCfg.fullDuplexEnable);
	
		p_dev = hDDA->nextDevice;
	}
	return len;
}
	
static int cpmac_p_read_stats(char* buf, char **start, off_t offset, int count, int *eof, void *data)
{
	struct net_device *p_dev = last_cpmac_device;
	int len = 0;
	int limit = count - 80;
	int i;
	struct net_device *cpmac_dev_list[cpmac_devices_installed];
	CpmacNetDevice *hDDA;
	CpmacHwStatistics *p_deviceMib;
	
	for(i=0; i< cpmac_devices_installed; i++)
	{
		cpmac_dev_list[cpmac_devices_installed - (i+1)] = p_dev;
		hDDA = netdev_priv(p_dev);
		p_dev = hDDA->nextDevice;
	}
	
	for(i=0; i< cpmac_devices_installed; i++)
	{
		p_dev = cpmac_dev_list[i];
	
		if(!p_dev)
			goto proc_error;
		
		cpmac_p_update_statistics(p_dev, NULL, 0, NULL);
		hDDA = netdev_priv(p_dev);
		p_deviceMib = &hDDA->deviceMib;
			
		len+= sprintf(buf+len, "\nCpmac %d, Address %lx\n",i+1, p_dev->base_addr);
		if(len<=limit)
			len+= sprintf(buf+len, " Transmit Stats\n");
		if(len<=limit)
			len+= sprintf(buf+len, "   Tx Valid Bytes Sent        :%u\n",p_deviceMib->ifOutOctets);
		if(len<=limit)
			len+= sprintf(buf+len, "   Good Tx Frames (Hardware)  :%u\n",p_deviceMib->ifOutGoodFrames);
		if(len<=limit)
			len+= sprintf(buf+len, "   Good Tx Frames (Software)  :%lu\n",hDDA->netDevStats.tx_packets);
		if(len<=limit)
			len+= sprintf(buf+len, "   Good Tx Broadcast Frames   :%u\n",p_deviceMib->ifOutBroadcasts);
		if(len<=limit)
			len+= sprintf(buf+len, "   Good Tx Multicast Frames   :%u\n",p_deviceMib->ifOutMulticasts);
		if(len<=limit)
			len+= sprintf(buf+len, "   Pause Frames Sent          :%u\n",p_deviceMib->ifOutPauseFrames);
		if(len<=limit)
			len+= sprintf(buf+len, "   Collisions                 :%u\n",p_deviceMib->ifCollisionFrames);
		if(len<=limit)
			len+= sprintf(buf+len, "   Tx Error Frames            :%lu\n",hDDA->netDevStats.tx_errors);
		if(len<=limit)
			len+= sprintf(buf+len, "   Carrier Sense Errors       :%u\n",p_deviceMib->ifCarrierSenseErrors);
		if(len<=limit)
			len+= sprintf(buf+len, "\n");
	
		/* Receive Stats */
		if(len<=limit)
			len+= sprintf(buf+len, "\nCpmac %d, Address %lx\n",i+1,p_dev->base_addr);
		if(len<=limit)
			len+= sprintf(buf+len, " Receive Stats\n");
		if(len<=limit)
			len+= sprintf(buf+len, "   Rx Valid Bytes Received    :%u\n",p_deviceMib->ifInOctets);
		if(len<=limit)
			len+= sprintf(buf+len, "   Good Rx Frames (Hardware)  :%u\n",p_deviceMib->ifInGoodFrames);
		if(len<=limit)
			len+= sprintf(buf+len, "   Good Rx Frames (Software)  :%lu\n",hDDA->netDevStats.rx_packets);
		if(len<=limit)
			len+= sprintf(buf+len, "   Good Rx Broadcast Frames   :%u\n",p_deviceMib->ifInBroadcasts);
		if(len<=limit)
			len+= sprintf(buf+len, "   Good Rx Multicast Frames   :%u\n",p_deviceMib->ifInMulticasts);
		if(len<=limit)
			len+= sprintf(buf+len, "   Pause Frames Received      :%u\n",p_deviceMib->ifInPauseFrames);
		if(len<=limit)
			len+= sprintf(buf+len, "   Rx CRC Errors              :%u\n",p_deviceMib->ifInCRCErrors);
		if(len<=limit)
			len+= sprintf(buf+len, "   Rx Align/Code Errors       :%u\n",p_deviceMib->ifInAlignCodeErrors);
		if(len<=limit)
			len+= sprintf(buf+len, "   Rx Jabbers                 :%u\n",p_deviceMib->ifInOversizedFrames);
		if(len<=limit)
			len+= sprintf(buf+len, "   Rx Filtered Frames         :%u\n",p_deviceMib->ifInFilteredFrames);
		if(len<=limit)
			len+= sprintf(buf+len, "   Rx Fragments               :%u\n",p_deviceMib->ifInFragments);
		if(len<=limit)
			len+= sprintf(buf+len, "   Rx Undersized Frames       :%u\n",p_deviceMib->ifInUndersizedFrames);
		if(len<=limit)
			len+= sprintf(buf+len, "   Rx Overruns                :%u\n",p_deviceMib->ifRxDMAOverruns);
	}
	
	return len;
	
	proc_error:
	*eof=1;
	return len;
}
	
static int cpmac_p_write_stats (struct file *fp, const char * buf, unsigned long count, void * data)
{
	char local_buf[31];
	int ret_val = 0;
	
	if(count > 30) {
		printk("Error : Buffer Overflow\n");
		printk("Use \"echo 0 > cpmac_stat\" to reset the statistics\n");
		return -EFAULT;
	}
	
	copy_from_user(local_buf,buf,count);
	local_buf[count-1]='\0'; /* Ignoring last \n char */
	ret_val = count;
	
	if(strcmp("0",local_buf)==0) {
		struct net_device *p_dev = last_cpmac_device;
		int i;
		struct net_device     *cpmac_dev_list[cpmac_devices_installed];
		CpmacNetDevice  *hDDA;
	
		printk("Resetting statistics for CPMAC interface.\n");
	
		for(i=0; i< cpmac_devices_installed; i++) {
			cpmac_dev_list[cpmac_devices_installed - (i+1)] = p_dev;
			hDDA = netdev_priv(p_dev);
			p_dev = hDDA->nextDevice;
		}
	
		for(i=0; i< cpmac_devices_installed; i++) {
			p_dev = cpmac_dev_list[i];
			if(!p_dev) {
				ret_val = -EFAULT;
				break;
			}	
			cpmac_p_reset_statistics(p_dev);
		}
	}
	else {
		printk("Error: Unknown operation on cpmac statistics\n");
		printk("Use \"echo 0 > cpmac_stats\" to reset the statistics\n");
		return -EFAULT;
	}
	return ret_val;
}
	
static int cpmac_p_read_rfc2665_stats(char* buf, char **start, off_t offset, int count, int *eof, void *data)
{
	int limit = count - 80;
	int len = 0;
	struct net_device *p_dev = (struct net_device*)data;
	
	cpmac_p_update_statistics(p_dev, buf, limit, &len);
	*eof = 1;
	
	return len;
}

static int cpmac_p_reset_statistics(struct net_device *p_dev)
{
	CpmacNetDevice *hDDA = netdev_priv(p_dev);
	
	memset(&hDDA->deviceMib, 0, sizeof(CpmacHwStatistics));
	memset(&hDDA->deviceStats, 0, sizeof(CpmacDrvStats));
	memset(&hDDA->netDevStats, 0, sizeof(struct net_device_stats));
	
	if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_CLR_STATISTICS, NULL, NULL) != CPMAC_SUCCESS) {
		printk("cpmac_p_reset_statistics: Error clearing statistics in DDC for %s\n", p_dev->name);
		return (-1);
	}
	
	return(0);
}
	
static int cpmac_p_update_statistics(struct net_device *p_dev, char *buf, int limit, int *p_len)
{
	unsigned long rx_hal_errors   = 0;
	unsigned long rx_hal_discards = 0;
	unsigned long tx_hal_errors   = 0;
	unsigned long ifOutDiscards   = 0;
	unsigned long ifInDiscards    = 0;
	unsigned long ifOutErrors     = 0;
	unsigned long ifInErrors      = 0;
	CpmacNetDevice *hDDA = netdev_priv(p_dev);
	
	CpmacHwStatistics   *p_deviceMib = &hDDA->deviceMib;
	CpmacDrvStats    *p_stats      = &hDDA->deviceStats;
	CpmacHwStatistics   local_mib;
	CpmacHwStatistics   *p_local_mib  = &local_mib;
	
	struct net_device_stats *p_netDevStats = &hDDA->netDevStats;
	
	int len = 0;
	int dev_mib_elem_count      = 0;
	
	if(!test_bit(0, &hDDA->setToClose)) {
		if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_GET_STATISTICS, (void*) p_local_mib, NULL) != CPMAC_SUCCESS) {
			printk("Error getting statistics from DDC for %s\n", p_dev->name);
			return(-1);
		}
	
		dev_mib_elem_count = sizeof(CpmacHwStatistics)/sizeof(unsigned long);
		while(dev_mib_elem_count--) {
			*((unsigned long*) p_deviceMib + dev_mib_elem_count) = 
			*((unsigned long*) p_local_mib  + dev_mib_elem_count); 
		}
	}
	
	rx_hal_errors = p_deviceMib->ifInFragments +
			p_deviceMib->ifInCRCErrors +
			p_deviceMib->ifInAlignCodeErrors +
			p_deviceMib->ifInJabberFrames;
	
	rx_hal_discards = p_deviceMib->ifRxDMAOverruns;
	
	tx_hal_errors = p_deviceMib->ifExcessiveCollisionFrames +
			p_deviceMib->ifLateCollisions +
			p_deviceMib->ifCarrierSenseErrors +
			p_deviceMib->ifOutUnderrun;
	
	if(hDDA->initCfg.rxCfg.copyShortFramesEnable == 0)
		rx_hal_errors += p_deviceMib->ifInUndersizedFrames;
	
	rx_hal_errors += p_deviceMib->ifInOversizedFrames;
	
	if(hDDA->initCfg.rxCfg.promiscousEnable == 0) {
		ifInDiscards  +=  p_deviceMib->ifInFilteredFrames;
	}
	
	ifInDiscards  = rx_hal_discards + p_netDevStats->rx_dropped;
	ifInErrors    = rx_hal_errors;
	ifOutErrors   = tx_hal_errors;
	ifOutDiscards = p_netDevStats->tx_dropped;
	
	hDDA->netDevStats.rx_errors  = ifInErrors;
	hDDA->netDevStats.collisions = p_deviceMib->ifCollisionFrames;
	
	if(buf == NULL || limit == 0)
	{
	return(0);
	}
	
	if(len <= limit)
	len+= sprintf(buf + len, "%-35s: %u\n", "ifSpeed", hDDA->linkSpeed);
	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "dot3StatsDuplexStatus",hDDA->linkMode);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifAdminStatus", (p_dev->flags & IFF_UP ? 1:2));

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifOperStatus", (((p_dev->flags & IFF_UP) && netif_carrier_ok(p_dev)) ? 1:2));

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %lu\n", "ifLastChange", p_stats->start_tick);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %lu\n", "ifInDiscards", ifInDiscards);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %lu\n", "ifInErrors", ifInErrors);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %lu\n", "ifOutDiscards", ifOutDiscards);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %lu\n", "ifOutErrors", ifOutErrors);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifInGoodFrames", p_deviceMib->ifInGoodFrames);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifInBroadcasts", p_deviceMib->ifInBroadcasts);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifInMulticasts", p_deviceMib->ifInMulticasts);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifInPauseFrames", p_deviceMib->ifInPauseFrames);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifInCRCErrors", p_deviceMib->ifInCRCErrors);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifInAlignCodeErrors", p_deviceMib->ifInAlignCodeErrors);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifInOversizedFrames", p_deviceMib->ifInOversizedFrames);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifInJabberFrames", p_deviceMib->ifInJabberFrames);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifInUndersizedFrames", p_deviceMib->ifInUndersizedFrames);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifInFragments", p_deviceMib->ifInFragments);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifInFilteredFrames", p_deviceMib->ifInFilteredFrames);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifInQosFilteredFrames", p_deviceMib->ifInQosFilteredFrames);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifInOctets", p_deviceMib->ifInOctets);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifOutGoodFrames", p_deviceMib->ifOutGoodFrames);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifOutBroadcasts", p_deviceMib->ifOutBroadcasts);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifOutMulticasts", p_deviceMib->ifOutMulticasts);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifOutPauseFrames", p_deviceMib->ifOutPauseFrames);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifDeferredTransmissions", p_deviceMib->ifDeferredTransmissions);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifCollisionFrames", p_deviceMib->ifCollisionFrames);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifSingleCollisionFrames", p_deviceMib->ifSingleCollisionFrames);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifMultipleCollisionFrames", p_deviceMib->ifMultipleCollisionFrames);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifExcessiveCollisionFrames", p_deviceMib->ifExcessiveCollisionFrames);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifLateCollisions", p_deviceMib->ifLateCollisions);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifOutUnderrun", p_deviceMib->ifOutUnderrun);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifCarrierSenseErrors", p_deviceMib->ifCarrierSenseErrors);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifOutOctets", p_deviceMib->ifOutOctets);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "if64OctetFrames", p_deviceMib->if64OctetFrames);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "if65To127POctetFrames", p_deviceMib->if65To127OctetFrames);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "if128To255OctetFrames", p_deviceMib->if128To255OctetFrames);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "if256To511OctetFrames", p_deviceMib->if256To511OctetFrames);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "if512To1023OctetFrames", p_deviceMib->if512To1023OctetFrames);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "if1024ToUpOctetFrames", p_deviceMib->if1024ToUPOctetFrames);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifNetOctets", p_deviceMib->ifNetOctets);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifRxSofOverruns", p_deviceMib->ifRxSofOverruns);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifRxMofOverruns", p_deviceMib->ifRxMofOverruns);

	if(len <= limit)
		len+= sprintf(buf + len, "%-35s: %u\n", "ifRxDMAOverruns", p_deviceMib->ifRxDMAOverruns);
	
	*p_len = len;
	
	return(0);
}
	
static int cpmac_p_get_version(char* buf, char **start, off_t offset, int count,int *eof, void *data)
{
	int  len = 0;
	int  limit = count - 80;
	unsigned int ddc_version = 0;
	char *ddc_version_string = NULL;
	
	ddc_version_string = DDC_cpmacGetVersionInfo(&ddc_version);
	
	len += sprintf(buf+len, "Texas Instruments : %s\n", cpmac_DDA_version_string);
	
	if(len <= limit && ddc_version_string)
		len += sprintf(buf+len, "Texas Instruments : %s\n", ddc_version_string);
	
	return len;
}
	
static void cpmac_p_tick_timer_expiry(CpmacNetDevice * hDDA)
{
	struct timer_list *p_timer = &hDDA->periodicTimer;
	
	if(test_bit(0, &hDDA->setToClose) && !g_init_enable_flag) {
		return;
	}
	
	if (hDDA->timerActive == 1) {
		hDDA->ddcIf->ddctick(hDDA->hDDC, NULL);
	
		p_timer->expires = jiffies + hDDA->periodicTicks;
		add_timer(p_timer);
	}
}
	
static void cpmac_p_mib_timer_expiry(CpmacNetDevice * hDDA)
{
	struct timer_list *p_timer = &hDDA->mibTimer;
	
	if(test_bit(0, &hDDA->setToClose) && !g_init_enable_flag) {
		return;
	}
	
	if (hDDA->mibTimerActive == 1) {
		hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_IF_PARAMS_UPDT, NULL, NULL);
		p_timer->expires = jiffies + hDDA->mibTicks;
		add_timer(p_timer);
	}
}

static int cpmac_p_dev_enable(CpmacNetDevice * hDDA)
{
	int retCode;
	struct net_device *p_dev = hDDA->owner;
	
	retCode = hDDA->ddcIf->ddcNetIf.ddcNetChOpen((DDC_Handle*)hDDA->hDDC, (DDC_NetChInfo *)&hDDA->txChInfo[0], NULL);

	if (retCode != CPMAC_SUCCESS) {
		printk("%s error: Error %08X from CPMAC DDC TX Channel Open()\n", p_dev->name, retCode);
		return (-1);
	}
	
	retCode = hDDA->ddcIf->ddcNetIf.ddcNetChOpen((DDC_Handle*)hDDA->hDDC, (DDC_NetChInfo *)&hDDA->rxChInfo[0], (void*)&hDDA->macAddr[0]);

	if (retCode != CPMAC_SUCCESS) {
		printk("%s error: Error %08X from CPMAC DDC RX Channel Open()\n", p_dev->name, retCode);
		return (-1);
	}
	
	retCode = hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcOpen(hDDA->hDDC, NULL);
	if (retCode != CPMAC_SUCCESS) {
		printk("%s error: Error %08X from CPMAC DDC Open()\n", p_dev->name, retCode);
		return (-1);
	}
			
	return (0);
}
	
static int cpmac_p_dev_disable(CpmacNetDevice * hDDA)
{
	int retCode;
	struct net_device *p_dev = hDDA->owner;
	
	netif_stop_queue(hDDA->owner);
	
	set_bit(0, &hDDA->setToClose);
	
	retCode = hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcClose(hDDA->hDDC, NULL);
	if (retCode != CPMAC_SUCCESS) {
		printk("%s error: Error %08X from CPMAC DDC Close()\n", p_dev->name, retCode);
		return (-1);
	}
	else {
		if (hDDA->timerActive != 0) {
			del_timer_sync(&hDDA->periodicTimer);
			hDDA->timerActive = 0;
		}
	
		hDDA->deviceStats.start_tick = jiffies;
		hDDA->linkSpeed        = 100000000;
		hDDA->linkMode         = 1;
		netif_carrier_off(p_dev);
	}
	return (0);
}
	
static struct net_device_stats *cpmac_dev_get_net_stats (struct net_device *p_dev)
{
	CpmacNetDevice *hDDA = netdev_priv(p_dev);
	
	cpmac_p_update_statistics(p_dev, NULL, 0, NULL);
	
	return &hDDA->netDevStats;
}
	
static void cpmac_dev_mcast_set(struct net_device *p_dev)
{
	CpmacNetDevice *hDDA = netdev_priv(p_dev);
	
	if(p_dev->flags & IFF_PROMISC) {
		hDDA->initCfg.rxCfg.promiscousEnable = 1;
		hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_SET_RXCFG, (void*) &hDDA->initCfg.rxCfg, NULL);
	}
	else if ((p_dev->flags & IFF_ALLMULTI) || (p_dev->mc_count > CPMAC_DDA_DEFAULT_MAX_MULTICAST_ADDRESSES) )   {
		hDDA->initCfg.rxCfg.promiscousEnable = 0;
		hDDA->initCfg.rxCfg.multicastEnable = 1;
		hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_SET_RXCFG, (void*) &hDDA->initCfg.rxCfg, NULL);
		hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_ALL_MULTI, (void*) CPMAC_ALL_MULTI_SET, NULL);
	}
	else if (p_dev->mc_count == 0) {
		hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_ALL_MULTI, (void*) CPMAC_ALL_MULTI_CLR, NULL);
		hDDA->initCfg.rxCfg.promiscousEnable = 0;
		hDDA->initCfg.rxCfg.multicastEnable = 0;
		hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_SET_RXCFG, (void*) &hDDA->initCfg.rxCfg, NULL);
	}
	else if (p_dev->mc_count) {
		struct dev_mc_list *mc_ptr;
	
		hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_ALL_MULTI, (void*) CPMAC_ALL_MULTI_CLR, NULL);
	
		hDDA->initCfg.rxCfg.promiscousEnable = 0;
		hDDA->initCfg.rxCfg.multicastEnable = 1;
		hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_SET_RXCFG, (void*) &hDDA->initCfg.rxCfg, NULL);
	
		for (mc_ptr = p_dev->mc_list; mc_ptr; mc_ptr = mc_ptr->next) {
			hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_MULTICAST_ADDR, (void*) CPMAC_MULTICAST_ADD, (void*) mc_ptr->dmi_addr);
		}
	}
	else {
		/* Do nothing */
	}
}
	
static int cpmac_dev_set_mac_addr(struct net_device *p_dev,void * addr)
{
	int retCode;
	CpmacAddressParams AddressParams;
	struct sockaddr *sa = addr;
	CpmacNetDevice *hDDA = netdev_priv(p_dev);
	
	AddressParams.channel = CPMAC_DDA_DEFAULT_RX_CHANNEL;
	AddressParams.macAddress = sa->sa_data;
	
	retCode = hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_SET_MAC_ADDRESS, (CpmacAddressParams *)&AddressParams, NULL);
	
	if (retCode != CPMAC_SUCCESS) {
		printk("%s error: Error %08X from CPMAC DDC TX Channel Open()\n", p_dev->name, retCode);
		return -EIO;
	}
	
	memcpy(hDDA->macAddr,sa->sa_data,p_dev->addr_len);
	memcpy(p_dev->dev_addr,sa->sa_data,p_dev->addr_len);
	return 0;
}
	
static void  cpmac_tx_timeout(struct net_device *p_dev)
{
	int retCode;
	CpmacNetDevice *hDDA = netdev_priv(p_dev);
	
	hDDA->ddcIf->ddcNetIf.ddcNetChClose(hDDA->hDDC,hDDA->txChInfo[0].chNum,hDDA->txChInfo[0].chDir,0);
	retCode = hDDA->ddcIf->ddcNetIf.ddcNetChOpen((DDC_Handle*)hDDA->hDDC, (DDC_NetChInfo *)&hDDA->txChInfo[0], NULL);
	if (retCode != CPMAC_SUCCESS) {
	printk("%s error: Error %08X from CPMAC DDC TX Channel Open()\n", p_dev->name, retCode);
	}
}
	
static int cpmac_dev_init(struct net_device *p_dev)
{
	int cnt, ddcInitStatus = 0;
	int retCode;
	char *mac_name = NULL;
	char *mac_string = NULL;
	char *default_mac_string = NULL;
	CpmacNetDevice *hDDA = netdev_priv(p_dev);
	int instanceNum = hDDA->instanceNum;
	
	switch(instanceNum) {
	case 0: 
		mac_name=CPMAC_DDA_MAC_ADDR_A; 
		default_mac_string = "08.00.28.32.06.02";
		break;
	
	case 1: 
		mac_name=CPMAC_DDA_MAC_ADDR_B;
		default_mac_string = "08.00.28.32.06.03";
		break;
	
	case 2: 
		mac_name=CPMAC_DDA_MAC_ADDR_C; 
		default_mac_string = "08.00.28.32.06.04";
		break;
	
	case 3: 
		mac_name=CPMAC_DDA_MAC_ADDR_D; 
		default_mac_string = "08.00.28.32.06.05";
		break;
	
	case 4: 
		mac_name=CPMAC_DDA_MAC_ADDR_E;
		default_mac_string = "08.00.28.32.06.06";
		break;
	
	case 5: 
		mac_name=CPMAC_DDA_MAC_ADDR_F; 
		default_mac_string = "08.00.28.32.06.07";
		break;
	
	default: 
		mac_name=""; 
		default_mac_string = "08.00.28.32.06.08";
		break;
	
	}

	if(mac_name)
		mac_string= (char *) prom_getenv(mac_name);
	
	if(!mac_string) {
		mac_string=default_mac_string;
		printk("Cpmac: Error getting mac from Boot enviroment for %s\n",p_dev->name);
		printk("Cpmac: Using default mac address: %s\n",mac_string);
	
		if(mac_name) {
			printk("Use Bootloader command:\n");
			printk("    setenv %s xx.xx.xx.xx.xx.xx\n",mac_name);
			printk("to set mac address\n");
		}
	}

	cpmac_str_to_ethaddr(hDDA->macAddr,mac_string);
	for (cnt=0; cnt <= ETH_ALEN; cnt++) {
		p_dev->dev_addr[cnt] = hDDA->macAddr[cnt];    
	}
	
	hDDA->setToClose = 1;
	
	if (cpmac_net_get_config(hDDA) != 0) {
		printk("Cpmac: Could not fetch configuration information for instance %d\n", instanceNum);
		goto cpmac_dev_init_exit;
	}
	
	retCode = DDC_cpmacCreateInstance(instanceNum,   
			 		hDDA,  
					&cpmac_DDA_cb_interface, 
					(DDC_Handle**)&hDDA->hDDC,  
					&hDDA->ddcIf, 
					NULL); 

	if (retCode != CPMAC_SUCCESS) {
		printk("Cpmac: cpmac_dev_init:%d: Error %08X from DDC_cpmacCreateInstance()\n", instanceNum, retCode);
		goto cpmac_dev_init_exit;
	}
	
	ddcInitStatus = 1;  /* Instance created */
	
	retCode = hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcInit(hDDA->hDDC, &hDDA->initCfg);
	if (retCode != CPMAC_SUCCESS) {
		printk("Cpmac: cpmac_dev_init:%d: Error %08X from DDC Init()\n", instanceNum, retCode);
		goto cpmac_dev_init_exit;
	}
	
	ddcInitStatus = 2;  /* Instance initialized */
	
	hDDA->linkSpeed = 100000000; 
	hDDA->linkMode = 1;
	
	init_timer(&hDDA->periodicTimer);
	hDDA->periodicTicks = 0;
	hDDA->periodicTimer.expires = 0;
	hDDA->timerActive = 0;
	hDDA->periodicTimer.data = (unsigned long)hDDA;
	hDDA->periodicTimer.function = (timer_tick_func)cpmac_p_tick_timer_expiry;
	
	init_timer(&hDDA->mibTimer);
	hDDA->mibTimerActive = 0;
	hDDA->mibTimer.data = (unsigned long)hDDA;
	hDDA->mibTimer.function = (timer_tick_func)cpmac_p_mib_timer_expiry;
	
	hDDA->Clear_EOI = 0;
	
	p_dev->addr_len = 6;
	p_dev->open = cpmac_dev_open;    /*  i.e. Start Device  */
	p_dev->do_ioctl = cpmac_ioctl;
	p_dev->hard_start_xmit = cpmac_dev_tx;
	p_dev->stop = cpmac_dev_close;
	p_dev->get_stats = cpmac_dev_get_net_stats;
	p_dev->set_multicast_list = cpmac_dev_mcast_set;
	p_dev->tx_timeout = cpmac_tx_timeout;
	p_dev->set_mac_address = cpmac_dev_set_mac_addr;
	p_dev->poll = cpmac_poll;
	p_dev->weight = hDDA->napiRxTx.rxMaxService;
	
	p_dev->flags &= ~(IFF_PROMISC | IFF_BROADCAST | IFF_MULTICAST);
	
	if (hDDA->initCfg.rxCfg.broadcastEnable == 1)
	p_dev->flags |= IFF_BROADCAST;
	if (hDDA->initCfg.rxCfg.multicastEnable == 1)
	p_dev->flags |= IFF_MULTICAST;
	
	netif_carrier_off(p_dev);
	
	p_dev->base_addr = hDDA->initCfg.baseAddress;
	request_mem_region(p_dev->base_addr, CPMAC_DDA_DEFAULT_CPMAC_SIZE, p_dev->name);
	
	if(g_init_enable_flag) {
		if (cpmac_p_dev_enable(hDDA)) {
			printk("%s error: cpmac_dev_init: device could not OPEN DDC\n", p_dev->name);
		goto cpmac_dev_init_exit;
		}
	}
	
	return(0);
	
cpmac_dev_init_exit:
	switch (ddcInitStatus) {
	case 2:     /* Deinit DDC */
		retCode = hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcDeinit(hDDA->hDDC, NULL);
		if (retCode != CPMAC_SUCCESS)
			printk("Cpmac: cpmac_dev_init_exit:%s: Error %08X from DDC Deinit()\n", p_dev->name, retCode);

	case 1:     /* Delete DDC Instance */
		retCode = hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcDelInst(hDDA->hDDC, NULL);
		if (retCode != CPMAC_SUCCESS)
			printk("Cpmac: cpmac_dev_init_exit:%s: Error %08X from DDC Delete Instance()\n", p_dev->name, retCode);
		break;
	
	default:
		break;
	}
	
	return (-1);
} /* cpmac_dev_init */
	
static int cpmac_dev_open(struct net_device *p_dev)
{
	CpmacNetDevice *hDDA = netdev_priv(p_dev);
		
	clear_bit(0, &hDDA->setToClose);    
		
	if(!g_init_enable_flag) {
		if (cpmac_p_dev_enable(hDDA)) {
			printk("%s error: cpmac_dev_open: device could not OPEN DDC\n", p_dev->name);
			return (-1);
		}
	}
	
	if(request_irq(LNXINTNUM(hDDA->initCfg.intrLine), cpmac_hal_isr, SA_INTERRUPT, "Cpmac Driver", hDDA)) {
		printk("Failed to register the irq %d for Cpmac %s.\n", hDDA->initCfg.intrLine, p_dev->name); 
		return (-1);          
	}
	
	if(netif_carrier_ok(p_dev))
		netif_start_queue(p_dev);
	else 
		netif_stop_queue(p_dev);
	
	hDDA->deviceStats.start_tick = jiffies;
	return(0);
}
	
static int cpmac_dev_close(struct net_device *p_dev)
{
	CpmacNetDevice *hDDA = netdev_priv(p_dev);
	
	netif_stop_queue(hDDA->owner);    
	free_irq(LNXINTNUM(hDDA->initCfg.intrLine), hDDA);    
	
	if(!g_init_enable_flag)
		cpmac_p_dev_disable(p_dev->priv);
	
	if(hDDA->Clear_EOI) {
		hDDA->ddcIf->pktProcessEnd(hDDA->hDDC, NULL);
		hDDA->Clear_EOI = 0;
	}
	
	set_bit(0, &hDDA->setToClose);
	
	return(0);
}
	
static int cpmac_ioctl (struct net_device *p_dev, struct ifreq *rq, int cmd)
{
	CpmacDrvPrivIoctl privIoctl;
	CpmacNetDevice *hDDA = netdev_priv(p_dev);
	
	if(cmd == SIOCDEVPRIVATE) {
		if(copy_from_user((char *)&privIoctl,(char *)rq->ifr_data, sizeof(CpmacDrvPrivIoctl)))
			return -EFAULT;
	
		switch (privIoctl.cmd) {       
		case CPMAC_DDA_PRIV_FILTERING:
			{
				CpmacType2_3_AddrFilterParams filterParams;
				if(copy_from_user((char *)&filterParams,(char *)privIoctl.data, sizeof(CpmacType2_3_AddrFilterParams)))
					return -EFAULT;
	
				if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_TYPE2_3_FILTERING,  (CpmacType2_3_AddrFilterParams *)&filterParams, NULL) != CPMAC_SUCCESS) {
					printk("Failed to read params (CPMAC_DDA_PRIV_FILTERING) from DDC for  %s.\n", p_dev->name);
					return -EFAULT;
				}
				break;
			}
	
		case CPMAC_DDA_PRIV_MII_READ:
			{
				CpmacPhyParams phyParams;
				unsigned long irq_flags;
	
				if(copy_from_user((char *)&phyParams,(char *)privIoctl.data, sizeof(CpmacPhyParams)))
					return -EFAULT;
	
				save_and_cli(irq_flags);
	
				if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_READ_PHY_REG, (void*) &phyParams, NULL) != CPMAC_SUCCESS) {
					printk("Failed to read params (CPMAC_DDA_PRIV_MII_READ) from DDC for  %s.\n", p_dev->name);	
					return -EFAULT;
				}
	
				if(copy_to_user((char *)privIoctl.data, (char *)&phyParams, sizeof(CpmacPhyParams)))
					return -EFAULT;
	
				restore_flags(irq_flags);
				
				break;
			}

		case CPMAC_DDA_PRIV_MII_WRITE:
			{
				CpmacPhyParams phyParams;
				unsigned long irq_flags;
	
				if(copy_from_user((char *)&phyParams,(char *)privIoctl.data, sizeof(CpmacPhyParams)))
					return -EFAULT;
	
				save_and_cli(irq_flags);
	
				if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_WRITE_PHY_REG, (void*) &phyParams, NULL) != CPMAC_SUCCESS) {
					printk("Failed to read params (CPMAC_DDA_PRIV_MII_READ) from DDC for  %s.\n", p_dev->name);
					return -EFAULT;		
				}
	
				restore_flags(irq_flags);
				break;
			}
	
		case CPMAC_DDA_PRIV_GET_STATS:
			{
				CpmacHwStatistics   stats;
	
				if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_GET_STATISTICS, (void*) &stats, NULL) != CPMAC_SUCCESS) {
					printk("Failed to get statistics (CPMAC_DDA_PRIV_GET_STATS) from DDC for  %s.\n", p_dev->name);
					return (CPMAC_DDA_INTERNAL_FAILURE);
				}
	
				if(copy_to_user((char *)privIoctl.data, (char *)&stats, sizeof(CpmacHwStatistics)))
					return -EFAULT;
	
				break;

			}

		case CPMAC_DDA_PRIV_CLR_STATS:
			{
				if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_CLR_STATISTICS, NULL, NULL) != CPMAC_SUCCESS) {
					printk("Failed to clear statistics (CPMAC_DDA_PRIV_CLR_STATS) in DDC for  %s.\n", p_dev->name);
					return (CPMAC_DDA_INTERNAL_FAILURE);
				}
				break;
			}

		case CPMAC_DDA_ADD_RX_BD:
			{
				unsigned int numOfRxBd = ( unsigned int )privIoctl.data;
		
				if (numOfRxBd <= hDDA->rxChInfo[0].numBD) {
					printk("%s CPMAC_DDA_ADD_RX_BD Capable only to add RX buffer\n",__FUNCTION__);
					return -EFAULT;
				}
	
				numOfRxBd = numOfRxBd - hDDA->rxChInfo[0].numBD;
	
				if(hDDA->ddcIf->AddRxBd((CpmacDDCObj*)hDDA->hDDC,(CpmacChInfo *)&hDDA->rxChInfo[0],numOfRxBd) != CPMAC_SUCCESS) {
					printk("%s CPMAC_DDA_ADD_RX_BD Failed to add RX buffer\n",__FUNCTION__);
					return -EFAULT;
				}
			
				break;	
			}
		default:
			return -EFAULT;
			break;
		} /* End of switch for CpmacDrvPrivIoctl command */
	}
	else if(cmd == SIOTIMIB2) {
		TI_SNMP_CMD_T ti_snmp_cmd;
	
		if(copy_from_user((char *)&ti_snmp_cmd,(char *)rq->ifr_data, sizeof(TI_SNMP_CMD_T)))
			return -EFAULT;
	
		switch (ti_snmp_cmd.cmd) {
		case TI_SIOCGINTFCOUNTERS:
			{
				struct mib2_ifCounters mib_counter;
			
				if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_IF_COUNTERS, (void*) &mib_counter, NULL) != CPMAC_SUCCESS) {
					printk("Failed to get statistics (TI_SIOCGINTFCOUNTERS) from DDC for  %s.\n", p_dev->name);
					return (CPMAC_DDA_INTERNAL_FAILURE);
				}
	
				if(copy_to_user((char *)ti_snmp_cmd.data, (char *)&mib_counter, sizeof(struct mib2_ifCounters)))
					return -EFAULT;
	
				break;
			}
	
		case TI_SIOCGINTFPARAMS:
			{
				struct mib2_ifParams    localParams;
	
				localParams.ifSpeed = hDDA->linkSpeed;
				localParams.ifHighSpeed = (localParams.ifSpeed)/1000000;
				localParams.ifOperStatus = ((p_dev->flags & IFF_UP)?MIB2_STATUS_UP:MIB2_STATUS_DOWN);
				localParams.ifPromiscuousMode           = ((p_dev->flags & IFF_PROMISC)?1:0);
	
				if(copy_to_user((char *)ti_snmp_cmd.data, (char *)&localParams, sizeof(struct mib2_ifParams)))
					       return   -EFAULT;
		        }
			break; 
	
		case TI_SIOCGETHERCOUNTERS:
			{
				struct mib2_phyCounters phy_counter;
	
				if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_ETHER_COUNTERS, (void*) &phy_counter, NULL) != CPMAC_SUCCESS) {
					printk("Failed to get statistics (TI_SIOCGETHERCOUNTERS) from DDC for  %s.\n", p_dev->name);
					return (CPMAC_DDA_INTERNAL_FAILURE);
				}
	
				if(copy_to_user((char *)ti_snmp_cmd.data, (char *)&phy_counter, sizeof(struct mib2_phyCounters)))
					return -EFAULT;
	
				break;
			}
	
		case TI_SIOCGETHERPARAMS:
			{
				struct mib2_ethParams localParams;
	
				localParams.ethDuplexStatus = ((hDDA->linkMode  == 2)?MIB2_FULL_DUPLEX:MIB2_HALF_DUPLEX);
	
				if(copy_to_user((char *)ti_snmp_cmd.data, (char *)&localParams, sizeof(struct mib2_ethParams)))
					return -EFAULT;
	
				break;
			}
		    
		default:
			return -EFAULT;
	
		}
	}
	else {
		return -EFAULT;
	}
	
	return (0);
}
	
static struct platform_device *cpmac_dev;
	
static ssize_t cpmac_show_version(struct device_driver *drv, char *buf)
{
	return cpmac_p_get_version(buf, NULL, 0, 4096, NULL, NULL);
}
	
static DRIVER_ATTR(version, S_IRUGO, cpmac_show_version, NULL);
	
static int __init cpmac_dev_probe(void)
{
	int     retVal         = 0;
	int     err_val;
	int     unit;
	int     instance_count = CONFIG_MIPS_CPMAC_PORTS;
	
	if(cpmac_cfg_probe()) {
		printk("Failed to probe for CPMAC configuration, Exiting.\n");
		return (-1);   
	}
	
	if ((err_val = avalanche_sysprobe_and_prep(AVALANCHE_CPMAC_HW_MODULE_REV, AVALANCHE_HIGH_CPMAC_BASE, 0)) != 0) {
		printk("CPMAC1 support not available\n");
		instance_count = 1;
	}
	
	cpmac_dev = platform_device_register_simple("cpmac", -1, NULL, 0);
	
	if (IS_ERR(cpmac_dev)) {
		return -1;
	}
	
	if (driver_register(&cpmac_driver)) {
		platform_device_unregister(cpmac_dev);
		return -1;
	}
	
	driver_create_file(&cpmac_driver, &driver_attr_version);
	for(unit = 0; unit < instance_count; unit++) {
		struct net_device       *p_dev;
		CpmacNetDevice    *hDDA;
		int                     failed;
	
		if(!(p_dev = alloc_etherdev(sizeof(CpmacNetDevice)))) {
			printk( "CPMAC: Etherdev alloc failed for device inst %d.\n", unit );
			retVal = -ENOMEM;

			break;
		}            
		hDDA = netdev_priv(p_dev);
		hDDA->owner = p_dev;
		hDDA->instanceNum  = unit;
		p_dev->init = cpmac_dev_init;   /* Set the initialization function */
	
		SET_NETDEV_DEV(p_dev, &(cpmac_dev->dev));
		cpmac_net_dev[hDDA->instanceNum] = p_dev;
	
		g_init_enable_flag = 1;
		cpmac_p_detect_manual_cfg(cfg_link_speed, cfg_link_mode, debug_mode);
	
		failed = register_netdev(p_dev);
		if (failed) {
			printk("Cpmac: Could not register device for inst %d: %d.\n", unit, failed);
			retVal = -1;
			free_netdev(p_dev);
				
			platform_device_unregister(cpmac_dev);
			printk("platform device unregistered.\n");

			driver_remove_file(&cpmac_driver, &driver_attr_version);
			printk("driver file removed.\n");
	
			driver_unregister(&cpmac_driver);
			printk("driver unregistered.\n");

			break;
		}           
		else {
			char proc_name[100];
			int  proc_category_name_len = 0;
	
			hDDA->nextDevice = last_cpmac_device;
			last_cpmac_device = p_dev;
			strcpy(proc_name, "avalanche/");
			strcat(proc_name, p_dev->name);
			proc_category_name_len = strlen(proc_name);
			strcpy(proc_name + proc_category_name_len, "_rfc2665_stats");
			create_proc_read_entry(proc_name,0,NULL,cpmac_p_read_rfc2665_stats, p_dev);
		}
	}
	
	if(retVal == 0) {
		gp_stats_file = create_proc_entry("avalanche/cpmac_stats", 0644, NULL);
		if(gp_stats_file) {
			gp_stats_file->read_proc  = cpmac_p_read_stats;
			gp_stats_file->write_proc = cpmac_p_write_stats;
		}
		create_proc_read_entry("avalanche/cpmac_link", 0, NULL, cpmac_p_read_link, NULL);
		create_proc_read_entry("avalanche/cpmac_ver", 0, NULL, cpmac_p_get_version, NULL);
		create_proc_read_entry("avalanche/cpmac_config", 0, NULL, cpmac_dump_config, NULL);
	}
	
	cpmac_devices_installed  = unit;
	printk("TI %s - %s\n", cpmac_DDA_version_string , DDC_cpmacGetVersionInfo(NULL));
	printk("Cpmac: Installed %d instances.\n", unit);
	printk("Cpmac driver is allocating buffer memory at init time.\n");

	return ( (unit >= 0 ) ? 0 : -ENODEV );
}
	
void cpmac_exit(void)
{
	struct net_device *p_dev;
	CpmacNetDevice *hDDA;
	int retCode;
	
	while (cpmac_devices_installed)  {
		char proc_name[100];
		int  proc_category_name_len = 0;
	
		p_dev = last_cpmac_device;
		hDDA = netdev_priv(p_dev);
		if(g_init_enable_flag) {
			cpmac_p_dev_disable(hDDA);
		}
	
		retCode = hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcDeinit(hDDA->hDDC, NULL);
		if (retCode != CPMAC_SUCCESS) {
			printk("Cpmac: cpmac_exit:%s: Error %08X from DDC Deinit()\n", p_dev->name, retCode);
		}
	
		retCode = hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcDelInst(hDDA->hDDC, NULL);
		if (retCode != CPMAC_SUCCESS) {
			printk("Cpmac: cpmac_exit:%s: Error %08X from DDC Delete Instance()\n", p_dev->name, retCode);
		}
	
		strcpy(proc_name, "avalanche/");
		strcat(proc_name, p_dev->name);
		proc_category_name_len = strlen(proc_name);
		strcpy(proc_name + proc_category_name_len, "_rfc2665_stats");
		remove_proc_entry(proc_name, NULL);
	
		release_mem_region(p_dev->base_addr, CPMAC_DDA_DEFAULT_CPMAC_SIZE);
		unregister_netdev(p_dev);
		last_cpmac_device = hDDA->nextDevice;
	
		if (p_dev)
			free_netdev(p_dev);
	
		cpmac_devices_installed--;
	}
	
	if(gp_stats_file)
		remove_proc_entry("avalanche/cpmac_stats", NULL);
	
	remove_proc_entry("avalanche/cpmac_link",  NULL);
	remove_proc_entry("avalanche/cpmac_ver",   NULL);
	remove_proc_entry("avalanche/cpmac_config",NULL);
	
	platform_device_unregister(cpmac_dev);
	printk("platform device unregistered.\n");
	
	driver_remove_file(&cpmac_driver, &driver_attr_version);
	printk("driver file removed.\n");
	
	driver_unregister(&cpmac_driver);
	printk("driver unregistered.\n");
}
	
module_init(cpmac_dev_probe);
module_exit(cpmac_exit);
	
int cpmac_poll(struct net_device *p_dev, int *budget)
{
	CpmacNetDevice *hDDA = netdev_priv(p_dev);
	unsigned int work = min(p_dev->quota, *budget);
	unsigned int pkts_pending = 0;
	
	RxTxParams *napi_params = &hDDA->napiRxTx;   
	
	if(netif_running(p_dev)) {
		napi_params->rxPkts = work;
		napi_params->txPkts = napi_params->txMaxService;
	
		hDDA->ddcIf->pktProcess(hDDA->hDDC, &pkts_pending, napi_params);
	
		*budget -= napi_params->retRxPkts;
		p_dev->quota -= napi_params->retRxPkts;
	
		if(pkts_pending){
			return 1;
		}
	}
	
	netif_rx_complete(p_dev);
	hDDA->ddcIf->pktProcessEnd(hDDA->hDDC, NULL);
	return 0;
}
	
void* DDA_cpmac_net_alloc_rx_buf(CpmacNetDevice* hDDA, int bufSize, 
				DDC_NetDataToken *dataToken, 
				unsigned int channel, void* allocArgs)
{
	struct net_device *p_dev = hDDA->owner;
	struct sk_buff *p_skb;
	
	p_skb = dev_alloc_skb(hDDA->rxBufSize);
	
	if( p_skb != NULL ) {
		p_skb->dev = p_dev;
		skb_reserve(p_skb, hDDA->rxBufOffset);
	
		*dataToken = (DDC_NetDataToken) p_skb;
	}
	else {
		return (NULL);
	}
	
	return p_skb->data;
}
	
int DDA_cpmac_net_free_rx_buf(CpmacNetDevice* hDDA, void* buffer, 
				DDC_NetDataToken dataToken, 
				unsigned int channel, void* freeArgs)
{
	dev_kfree_skb_any((struct sk_buff *) dataToken);
	return (CPMAC_SUCCESS);
}
	
int DDA_cpmac_net_rx_multiple_cb(CpmacNetDevice* hDDA, DDC_NetPktObj *netPktList, 
				int numPkts, void* rxArgs)
{
	unsigned int  cnt;
	
	for (cnt=numPkts; 0 < cnt; cnt--) {
		struct sk_buff *p_skb = (struct sk_buff *) netPktList->pktToken;
	
		skb_put(p_skb, netPktList->pktLength);
		CPMAC_DDA_CACHE_INVALIDATE(p_skb->data, p_skb->len);    
		p_skb->protocol = eth_type_trans(p_skb, hDDA->owner);
		p_skb->dev->last_rx = jiffies;
	
		netif_receive_skb(p_skb);
		hDDA->netDevStats.rx_bytes += netPktList->pktLength;
		++netPktList;
	}
	
	hDDA->netDevStats.rx_packets += numPkts;
	return (0);
}
	
int DDA_cpmac_net_tx_complete(CpmacNetDevice* hDDA, DDC_NetDataToken *netDataTokens,	
				int numTokens, unsigned int channel)
{
	unsigned int  cnt;
	
	if(numTokens && netif_queue_stopped(hDDA->owner))
		netif_start_queue(hDDA->owner);
	
	for (cnt = 0; cnt < numTokens; cnt++) {
		struct sk_buff *skb = (struct sk_buff *) netDataTokens[cnt];
		struct net_device_stats *stat = &hDDA->netDevStats;
	
		if(skb == NULL) 
			continue;
	
		stat->tx_packets++;
		stat->tx_bytes += skb->len;
	
		dev_kfree_skb(skb);
	}
	
	return(0);
}
	
irqreturn_t cpmac_hal_isr(int irq, void *dev_id, struct pt_regs *regs)
{
	CpmacNetDevice *hDDA = (CpmacNetDevice *)dev_id;
	
	++hDDA->isrCount;
	
	if (!hDDA->setToClose) {
		if(netif_rx_schedule_prep(hDDA->owner))
			__netif_rx_schedule(hDDA->owner);    
		else
			hDDA->Clear_EOI++; 
	}
	else {
		/* Do Nothing */
	} 
	
	return IRQ_HANDLED;
}
	
int cpmac_dev_tx( struct sk_buff *skb, struct net_device *p_dev)
{
	int retCode;
	DDC_NetBufObj txBuf;        
	DDC_NetPktObj txPacket;     
	CpmacNetDevice *hDDA = netdev_priv(p_dev);
	
	txPacket.bufList    = &txBuf;
	txPacket.numBufs    = 1;   
	txPacket.pktLength  = skb->len;
	txPacket.pktToken   = (DDC_NetDataToken) skb;
	txBuf.length        = skb->len;
	txBuf.bufToken      = (DDC_NetDataToken) skb;
	txBuf.data       = skb->data;
	
	CPMAC_DDA_CACHE_WRITEBACK((unsigned long)skb->data, skb->len);    
	
	p_dev->trans_start = jiffies;
	
	retCode = hDDA->ddcIf->ddcNetIf.ddcNetSend(hDDA->hDDC, &txPacket, CPMAC_DDA_DEFAULT_TX_CHANNEL, False);
	if (retCode == CPMAC_SUCCESS) {
		return (0);
	}
	else {
		if(retCode == CPMAC_ERR_TX_OUT_OF_BD) 
			netif_stop_queue(hDDA->owner);
	
		hDDA->netDevStats.tx_errors++; 
		hDDA->netDevStats.tx_dropped++;
		return (-1);
	}

}
#define CPMAC_DDC_MAJOR_VERSION         0
#define CPMAC_DDC_MINOR_VERSION         3
	
const static char CpmacDDCVersionString[] = "CPMAC DDC version 0.3";
static Bool CpmacDDCInstCreated[CPMAC_MAX_INSTANCES] = { False, False, False, False, False, False };
static CpmacDDCObj *CpmacDDCObject[CPMAC_MAX_INSTANCES] = {NULL, NULL, NULL, NULL, NULL, NULL};
static unsigned int  CpmacDDCNumInst = 0;
unsigned int CpmacDDCDebug =0;   
	
static int DDC_cpmacInit(CpmacDDCObj *hDDC, CpmacInitConfig *initCfg);
static int DDC_cpmacDeInit (CpmacDDCObj *hDDC, void* param);
static int DDC_cpmacOpen(CpmacDDCObj *hDDC, void* param);
static int DDC_cpmacClose(CpmacDDCObj *hDDC, void* param);
static int DDC_cpmacControl(CpmacDDCObj *hDDC, int cmd, void* cmdArg, void* param);
static int DDC_cpmacDeleteInstance (CpmacDDCObj *hDDC, void* param);
	
static int DDC_cpmacChOpen(CpmacDDCObj *hDDC, CpmacChInfo *chInfo, void* chOpenArgs);
static int DDC_cpmacChClose(CpmacDDCObj *hDDC, int channel, int direction, void* chCloseArgs);
	
static int cpmacWaitForTeardownComplete(CpmacDDCObj *hDDC, unsigned int channel, DDC_NetChDir direction, unsigned int mode);
static int cpmacEnableChannel(CpmacDDCObj *hDDC, unsigned int channel, unsigned int direction);
static int cpmacDisableChannel(CpmacDDCObj *hDDC, unsigned int channel, DDC_NetChDir direction);
static int cpmacInitTxChannel(CpmacDDCObj *hDDC, CpmacChInfo *chInfo, void* chOpenArgs);
static int cpmacInitRxChannel(CpmacDDCObj *hDDC, CpmacChInfo *chInfo, void* chOpenArgs);
static int cpmacUnInitTxChannel(CpmacDDCObj *hDDC, unsigned int channel, void* chCloseArgs);
static int cpmacUnInitRxChannel(CpmacDDCObj *hDDC, unsigned int channel, void* chCloseArgs);
static int cpmacAddRxBd(CpmacDDCObj *hDDC, CpmacChInfo *chInfo,unsigned int numOfBd2Add);
	
static void cpmacSetMacAddress(CpmacDDCObj *hDDC, unsigned int channel, unsigned char* macAddr);
static void cpmacSetSrcMacAddress(CpmacDDCObj *hDDC, unsigned char* macAddress);
static void cpmacDDCIfcntClear(CpmacDDCObj *hDDC);
static void cpmacDDCIfcntUpdt(CpmacDDCObj *hDDC);
static void cpmacDDCPhycnt(CpmacDDCObj *hDDC, unsigned int *cmdArg);
static unsigned int hashGet(unsigned char *addr)
{
	unsigned int  hash;
	unsigned char   tmpval;
	int     cnt;
	
	hash = 0;
	for( cnt=0; cnt<2; cnt++ ) {
		tmpval = *addr++;
		hash  ^= (tmpval>>2)^(tmpval<<4);
		tmpval = *addr++;
		hash  ^= (tmpval>>4)^(tmpval<<2);
		tmpval = *addr++;
		hash  ^= (tmpval>>6)^(tmpval);
	}
	
	return( hash & 0x3F );
}
	
static int hashAdd(CpmacDDCObj *hDDC, unsigned char *macAddress)
{
	unsigned int hashValue;
	unsigned int hashBit;
	unsigned int status = 0;
	
	hashValue = hashGet(macAddress);
	
	if (hashValue >= CPMAC_NUM_MULTICAST_BITS) {
		return (CPMAC_INVALID_PARAM);
	}
	
	if (hDDC->multicastHashCnt[hashValue] == 0) {
		status = 1;
		if(hashValue < 32) {
			hashBit = (1 << hashValue);
			hDDC->MacHash1 |= hashBit;
		}
		else {
			hashBit = (1 << (hashValue-32));
			hDDC->MacHash2 |= hashBit;
		}
	}

	++hDDC->multicastHashCnt[hashValue];
	return (status); 
}
	
static int hashDel(CpmacDDCObj *hDDC, unsigned char *macAddress)
{
	unsigned int hashValue;
	unsigned int hashBit;
	
	hashValue = hashGet(macAddress);
	
	if (hDDC->multicastHashCnt[hashValue] > 0) {
		--hDDC->multicastHashCnt[hashValue];
	}
	
	if (hDDC->multicastHashCnt[hashValue] > 0) {            
		return (0);
	}
	
	if(hashValue < 32) {
		hashBit = (1 << hashValue);
		hDDC->MacHash1 &= ~hashBit;
	}
	else {
		hashBit = (1 << (hashValue-32));
		hDDC->MacHash2 &= ~hashBit;
	}
	
	return (1); 
}
	
void cpmacSingleMulti(CpmacDDCObj *hDDC, CpmacSingleMultiOper oper, unsigned char *addr)
{
	unsigned int  status = -1;   
	switch (oper) {
	case CPMAC_MULTICAST_ADD:
		status = hashAdd(hDDC, addr);
		break;
	case CPMAC_MULTICAST_DEL:
		status = hashDel(hDDC, addr);
		break;
	default:
		break;
	}

	if (status > 0) {
		hDDC->regs->MacHash1 = hDDC->MacHash1;
		hDDC->regs->MacHash2 = hDDC->MacHash2;
	}
}
	
void cpmacAllMulti(CpmacDDCObj *hDDC, CpmacAllMultiOper oper)
{
	switch (oper) {
	case CPMAC_ALL_MULTI_SET:
		hDDC->MacHash1 = CPMAC_ALL_MULTI_REG_VALUE;
		hDDC->MacHash2 = CPMAC_ALL_MULTI_REG_VALUE;
		break;
	
	case CPMAC_ALL_MULTI_CLR:
		hDDC->MacHash1 = 0;
		hDDC->MacHash2 = 0;
		break;
	
	default:
		break;
	}
	
	hDDC->regs->MacHash1 = hDDC->MacHash1;
	hDDC->regs->MacHash2 = hDDC->MacHash2;
}

int cpmacUpdatePhyStatus(CpmacDDCObj *hDDC)
{
	phy_device *PhyDev = hDDC->PhyDev;
	unsigned int setPhyMode;
	
	if (hDDC->ddcObj.state != DDC_OPENED) {
		return (CPMAC_ERR_DEV_NOT_OPEN);
	}
	
	setPhyMode = hDDC->initCfg.phyMode;
	
	if(setPhyMode & SNWAY_NOPHY) {
		hDDC->status.PhyLinked = 1;
		hDDC->status.PhySpeed  = 1;
		hDDC->status.PhyDuplex = 1;
		hDDC->status.PhyNum    = 0xFFFFFFFF;  /* No Phy */
		hDDC->MacControl |= (1 << CSL_CPMAC_MACCONTROL_FULLDUPLEXEN_SHIFT);
		hDDC->regs->MacControl = hDDC->MacControl;
		goto cpmacUpdatePhyStatus_Exit;
	}
	
	if(hDDC->MacControl & CSL_CPMAC_MACCONTROL_LOOPBKEN_MASK) {
		hDDC->status.PhyLinked = 1;
		goto cpmacUpdatePhyStatus_Exit;
	}
	
	if(setPhyMode & SNWAY_LPBK) { 
		hDDC->status.PhyLinked = cpsw_halcommon_mii_mdio_getloopback(PhyDev); 
	}
	else { 
		hDDC->status.PhyLinked = cpsw_halcommon_mii_mdio_getlinked(PhyDev); 
	}
	
	if (hDDC->status.PhyLinked)
	{
		if (setPhyMode & SNWAY_LPBK) { 
			hDDC->status.PhyDuplex = 1; 
		}
		else { 
			hDDC->status.PhyDuplex = cpsw_halcommon_mii_mdio_get_duplex(PhyDev); 
		}
	
		hDDC->status.PhySpeed = cpsw_halcommon_mii_mdio_get_speed(PhyDev);
		hDDC->status.PhySpeed = hDDC->status.PhySpeed >> 10;
		hDDC->status.PhyNum   = cpsw_halcommon_mii_mdio_getphy_num(PhyDev);
	
		if (hDDC->status.PhyDuplex) { 
			hDDC->MacControl |= (1 << CSL_CPMAC_MACCONTROL_FULLDUPLEXEN_SHIFT); 
		}
		else { 
			hDDC->MacControl &= ~(1 << CSL_CPMAC_MACCONTROL_FULLDUPLEXEN_SHIFT); 
		}
	}
	
	hDDC->regs->MacControl = hDDC->MacControl;
	
	cpmacUpdatePhyStatus_Exit:
	
	return (CPMAC_SUCCESS);
}
	
void* cpmacGetPhyDev(CpmacDDCObj *hDDC)
{
	return (void*)hDDC->PhyDev;
}
	
static int cpmacSetPhyMode(CpmacDDCObj *hDDC)
{
	unsigned int setPhyMode;
	unsigned int PhyMode;
	
	if (hDDC->ddcObj.state != DDC_OPENED) {
		return (CPMAC_ERR_DEV_NOT_OPEN);
	}
	
	if (1 == avalanche_is_mdix_on_chip()) {
		hDDC->initCfg.phyMode |= SNWAY_AUTOMDIX;
	}
	
	setPhyMode = hDDC->initCfg.phyMode;
	PhyMode = 0;
	
	if(setPhyMode & SNWAY_AUTO)       
		PhyMode |= NWAY_AUTO;

	if(setPhyMode & SNWAY_FD10)
		PhyMode |= NWAY_FD10;

	if(setPhyMode & SNWAY_FD100)
		PhyMode |= NWAY_FD100;

	if(setPhyMode & SNWAY_HD10)
		PhyMode |= NWAY_HD10;

	if(setPhyMode & SNWAY_HD100)
		PhyMode |= NWAY_HD100;

	if(setPhyMode & SNWAY_LPBK)
		PhyMode |= NWAY_LPBK;

	if(setPhyMode & SNWAY_AUTOMDIX) 
		PhyMode |= NWAY_AUTOMDIX;
	
	if((setPhyMode & SNWAY_FD10) || (setPhyMode & SNWAY_HD10)) {
		/* Do Nothing */
	}
	else 
		if((setPhyMode & SNWAY_FD100) || (setPhyMode & SNWAY_HD100)) {
			/* Do Nothing */
		}
	cpsw_halcommon_mii_mdio_setphymode(hDDC->PhyDev, PhyMode);
	cpmacUpdatePhyStatus(hDDC);
	
	return (CPMAC_SUCCESS);
}

static void cpmacRxUniCast(CpmacDDCObj *hDDC, unsigned int channel, Bool enable)
{
	if(enable == True) {
		hDDC->Rx_Unicast_Set    |=  (1 << channel);
		hDDC->Rx_Unicast_Clear  &= ~(1 << channel);
	}
	else {
		hDDC->Rx_Unicast_Clear  |=  (1 << channel);
		hDDC->Rx_Unicast_Set    &= ~(1 << channel);
	}

	if (hDDC->ddcObj.state == DDC_OPENED) {
		hDDC->regs->Rx_Unicast_Set = hDDC->Rx_Unicast_Set;
		hDDC->regs->Rx_Unicast_Clear = hDDC->Rx_Unicast_Clear;
	}
}
	
static void cpmacAddType0Addr(CpmacDDCObj *hDDC, unsigned int channel, unsigned char* macAddress)
{
	CPMAC_MACADDRLO(hDDC->initCfg.baseAddress, channel) =  CSL_FMK(CPMAC_TYPE_0_MACSRCADDR0,(macAddress[5]));
	CPMAC_MACADDRMID(hDDC->initCfg.baseAddress) = CSL_FMK(CPMAC_TYPE_0_MACSRCADDR1,(macAddress[4]));
	CPMAC_MACADDRHI(hDDC->initCfg.baseAddress) = CSL_FMK(CPMAC_TYPE_0_MACSRCADDR2,(macAddress[3])) | CSL_FMK(CPMAC_TYPE_0_MACSRCADDR3,(macAddress[2])) |
	CSL_FMK(CPMAC_TYPE_0_MACSRCADDR4,(macAddress[1])) |
	CSL_FMK(CPMAC_TYPE_0_MACSRCADDR5,(macAddress[0]));
	
	cpmacRxUniCast(hDDC, channel, True);
}
	
static void cpmacSetSrcMacAddress(CpmacDDCObj *hDDC, unsigned char* macAddress)
{
	
	if((hDDC->RxAddrType == RX_ADDR_TYPE1) || (hDDC->RxAddrType == RX_ADDR_TYPE2)) {
		hDDC->regs->MacSrcAddr_Lo = (macAddress[5] << 8)  |  macAddress[4];
		hDDC->regs->MacSrcAddr_Hi = (macAddress[3] << 24) | (macAddress[2] << 16) | (macAddress[1] << 8)  |(macAddress[0]); 
	}
	
}
	
static void cpmacAddType1Addr(CpmacDDCObj *hDDC, unsigned int channel,
				unsigned char* macAddress)
{
	hDDC->regs->MacIndex =  channel;
	
	hDDC->regs->MacAddr_Hi =  CSL_FMK(CPMAC_TYPE_1_MACSRCADDR2,(macAddress[3])) |
		CSL_FMK(CPMAC_TYPE_1_MACSRCADDR3,(macAddress[2])) |
		CSL_FMK(CPMAC_TYPE_1_MACSRCADDR4,(macAddress[1])) |
		CSL_FMK(CPMAC_TYPE_1_MACSRCADDR5,(macAddress[0]));
	
	hDDC->regs->MacAddr_Lo =  CSL_FMK(CPMAC_TYPE_1_MACSRCADDR0,(macAddress[5])) |
	CSL_FMK(CPMAC_TYPE_1_MACSRCADDR1,(macAddress[4]));
	cpmacRxUniCast(hDDC, channel, True);
}
	
static void cpmacAddType2Addr(CpmacDDCObj *hDDC, unsigned int channel, 
				unsigned char* macAddress, int index, 
				Bool valid, int match)
{
	unsigned int macAddrLo;
	
	hDDC->regs->MacIndex =  index;
	hDDC->regs->MacAddr_Hi =  CSL_FMK(CPGMAC_TYPE_2_3_MACSRCADDR2,(macAddress[3])) |
		CSL_FMK(CPGMAC_TYPE_2_3_MACSRCADDR3,(macAddress[2])) |
		CSL_FMK(CPGMAC_TYPE_2_3_MACSRCADDR4,(macAddress[1])) |
		CSL_FMK(CPGMAC_TYPE_2_3_MACSRCADDR5,(macAddress[0]));

	macAddrLo  = CSL_FMK(CPGMAC_VALID, ((valid == True) ? 0x1 : 0));
	macAddrLo |= CSL_FMK(CPGMAC_MATCH_FILTER, ((valid == True) ? 0x1 : 0));
	macAddrLo |= CSL_FMK(CPGMAC_CHANNEL, channel);
	macAddrLo |= CSL_FMK(CPGMAC_TYPE_2_3_MACSRCADDR0,(macAddress[5]));
	macAddrLo |= CSL_FMK(CPGMAC_TYPE_2_3_MACSRCADDR1,(macAddress[4]));
	
	hDDC->regs->MacAddr_Lo = macAddrLo;
	
	cpmacRxUniCast(hDDC, channel, True);
}
	
static void cpmacType2AddrInit(CpmacDDCObj *hDDC)
{
	unsigned int i = 0; 
	
	for (i = 0; i < 32; i++) {
		hDDC->regs->MacIndex   = i;
		hDDC->regs->MacAddr_Lo = CSL_FMK(CPGMAC_VALID, False);
	}
}
	
static void cpmacTypeXAddrInit(CpmacDDCObj *hDDC)
{
	if(hDDC->RxAddrType == RX_ADDR_TYPE2)
		cpmacType2AddrInit(hDDC);
}
	
void cpmacSetRxHwCfg(CpmacDDCObj *hDDC)
{
	CpmacRxConfig *rxCfg;
	unsigned int rxMbpEnable;
	
	if (hDDC->ddcObj.state != DDC_OPENED) {
		return;
	}
	
	rxCfg = &hDDC->initCfg.rxCfg;
	rxMbpEnable = CSL_FMK(CPMAC_RXMBP_PASSCRC, rxCfg->passCRC) |
		CSL_FMK(CPMAC_RXMBP_QOSEN, rxCfg->qosEnable) |
		CSL_FMK(CPMAC_RXMBP_NOCHAIN, rxCfg->noBufferChaining) |
		CSL_FMK(CPMAC_RXMBP_CMFEN,rxCfg->copyMACControlFramesEnable) |
		CSL_FMK(CPMAC_RXMBP_CSFEN,rxCfg->copyShortFramesEnable) |
		CSL_FMK(CPMAC_RXMBP_CEFEN,rxCfg->copyErrorFramesEnable) |
		CSL_FMK(CPMAC_RXMBP_CAFEN,rxCfg->promiscousEnable) |
		CSL_FMK(CPMAC_RXMBP_PROMCH,rxCfg->promiscousChannel) |
		CSL_FMK(CPMAC_RXMBP_BROADEN,rxCfg->broadcastEnable) |
		CSL_FMK(CPMAC_RXMBP_BROADCH,rxCfg->broadcastChannel) |
		CSL_FMK(CPMAC_RXMBP_MULTIEN,rxCfg->multicastEnable) |
		CSL_FMK(CPMAC_RXMBP_MULTICH,rxCfg->multicastChannel) ;  
	
	if(rxCfg->promiscousEnable) {
		CSL_FINS(rxMbpEnable, CPMAC_RXMBP_BROADEN, False);
		CSL_FINS(rxMbpEnable, CPMAC_RXMBP_MULTIEN, False);
		cpmacRxUniCast(hDDC, (hDDC->rxCppi[0])->chInfo.chNum, False);
	} else {
		cpmacRxUniCast(hDDC, (hDDC->rxCppi[0])->chInfo.chNum, True);
	}
	
	if(hDDC->Rx_MBP_Enable != rxMbpEnable) {
		hDDC->Rx_MBP_Enable = rxMbpEnable;
		hDDC->regs->Rx_MBP_Enable = rxMbpEnable;
	}
	
	hDDC->regs->Rx_Maxlen = CSL_FMK(CPMAC_RX_MAX_LEN, rxCfg->maxRxPktLength);
	hDDC->regs->Rx_Buffer_Offset = CSL_FMK(CPMAC_RX_BUFFER_OFFSET, rxCfg->bufferOffset);
}

void cpmacSetMacHwCfg(CpmacDDCObj *hDDC)
{
	CpmacMacConfig *macCfg;
	unsigned int macControl;
	
	if (hDDC->ddcObj.state != DDC_OPENED) {
		return;
	}
	
	macCfg = &hDDC->initCfg.macCfg;
	
	macControl = CSL_FMK(CPMAC_MACCONTROL_TXSHORTGAPEN, macCfg->txShortGapEnable) |
		CSL_FMK(CPMAC_MACCONTROL_TXPTYPE, ((macCfg->pType == CPMAC_TXPRIO_FIXED) ? 0x1 : 0)) |
		CSL_FMK(CPMAC_MACCONTROL_GIGABITEN, macCfg->gigaBitEnable) |
		CSL_FMK(CPMAC_MACCONTROL_TXPACEEN, macCfg->gigaBitEnable) |
		(hDDC->MacControl & CSL_CPMAC_MACCONTROL_MIIEN_MASK) | 
		CSL_FMK(CPMAC_MACCONTROL_TXFLOWEN, macCfg->txFlowEnable) |
		CSL_FMK(CPMAC_MACCONTROL_RXFLOWEN, macCfg->rxFlowEnable) |
		CSL_FMK(CPMAC_MACCONTROL_LOOPBKEN, macCfg->loopbackEnable) |
		(hDDC->MacControl & CSL_CPMAC_MACCONTROL_FULLDUPLEXEN_MASK);
	
	
	if(hDDC->MacControl != macControl) {
		hDDC->MacControl = macControl;
		hDDC->regs->MacControl = macControl;
	}
}
	
unsigned char* DDC_cpmacGetVersionInfo(unsigned int *swVer)
{
	if (swVer != NULL)
		*swVer = ((CPMAC_DDC_MAJOR_VERSION << 16) | CPMAC_DDC_MINOR_VERSION);
	
	return ((unsigned char*) &CpmacDDCVersionString[0]);
}
	
static CpmacDDCIf CpmacDDCInterface = 
{ 
	{                                  /* DDC Net Class functions */
		{                                 /*   DDC Class functions */
			(DDC_Init)DDC_cpmacInit,         /*      CPMAC Init function */
			(DDC_DeInit)DDC_cpmacDeInit,     /*      CPMAC DeInit function */
			(DDC_Open)DDC_cpmacOpen,         /*      CPMAC Open function */
			(DDC_Close)DDC_cpmacClose,       /*      CPMAC Close function */
			(DDC_Control)DDC_cpmacControl,    /*      CPMAC Control function */
			(DDC_DeleteInstance)DDC_cpmacDeleteInstance 
		},
		(DDC_NetSend)DDC_cpmacSend,       /*   CPMAC Send function */
		NULL,                             /*   Multiple packet send not supported */
		NULL,                             /*   Poll RX not supported */
		NULL,                             /*   Rx Return not supported */
		NULL,                         
		(DDC_NetChOpen)DDC_cpmacChOpen,   /*   CPMAC channel open function */
		(DDC_NetChClose)DDC_cpmacChClose  /*   CPMAC channel close function */
	},          
	(DDC_CpmacTick)cpmacTick,               
	(DDC_CpmacPktProcess)cpmacPktProcess,      
	(DDC_CpmacPktProcessEnd)cpmacPktProcessEnd, 
	(DDC_CpmacGetInterruptCause)cpmacGetInterruptCause,  
	(DDC_CpmacPktTxCompletionProcess)cpmacTxPktCompletionProcess,    
	(DDC_CpmacPktRxProcess)cpmacRxPktProcess,   
	(DDC_CpmacAddRxBd)cpmacAddRxBd              /* DDC CPMAC add Rx BD function */
};
	
int DDC_cpmacCreateInstance(unsigned int instId, DDA_Handle hDDA, CpmacDDACbIf *hDDACbIf,
				DDC_Handle **hDDC, CpmacDDCIf **hDDCIf, void* param)
{
	CpmacDDCObj     *cpmacDDCHandle;
	int      retCode;
	
	if (CpmacDDCInstCreated[instId] == True) {
		return (CPMAC_ERR_DEV_ALREADY_INSTANTIATED(instId));
	}
	
	retCode = avalanche_memalloc(sizeof(CpmacDDCObj), (void* *) &cpmacDDCHandle);  
	if (retCode != 0) {
		return (retCode);
	}
	memset(cpmacDDCHandle, 0, sizeof(CpmacDDCObj)); 
	
	cpmacDDCHandle->ddcObj.versionId = (CPMAC_DDC_MAJOR_VERSION << 16) | CPMAC_DDC_MINOR_VERSION;
	cpmacDDCHandle->ddcObj.instId = instId;
	cpmacDDCHandle->ddcObj.state = DDC_CREATED;
	
	cpmacDDCHandle->ddcIf = &CpmacDDCInterface;
	
	cpmacDDCHandle->ddcObj.hDDA = hDDA;
	cpmacDDCHandle->ddaIf = hDDACbIf;
	
	*hDDC = (DDC_Handle) cpmacDDCHandle;
	*hDDCIf = cpmacDDCHandle->ddcIf;
	
	CpmacDDCInstCreated[instId] = True;
	CpmacDDCObject[instId] = cpmacDDCHandle;
	++CpmacDDCNumInst;
	
	return (CPMAC_SUCCESS);
}
	
static int DDC_cpmacDeleteInstance (CpmacDDCObj *hDDC, void* param)
{
	int retCode = CPMAC_SUCCESS;
	unsigned int instId = hDDC->initCfg.instId;
	
	if (CpmacDDCInstCreated[instId] == False) {
		return (CPMAC_ERR_DEV_NOT_INSTANTIATED);	
	}
	
	CpmacDDCInstCreated[instId] = False;
	CpmacDDCObject[instId] = NULL;
	--CpmacDDCNumInst;
	
	if (hDDC != NULL) {
		retCode = avalanche_memfree(hDDC);
		if (retCode != 0) {
			hDDC->ddcIf = NULL;
		}
		hDDC = NULL;
	}
	
	return (retCode);
}
	
static int DDC_cpmacInit(CpmacDDCObj *hDDC, CpmacInitConfig *initCfg)
{
	if ((initCfg->numTxChannels > CPMAC_MAX_TX_CHANNELS) || 
		(initCfg->numRxChannels > CPMAC_MAX_RX_CHANNELS)) {
		return (CPMAC_INVALID_PARAM);
	}
	
	hDDC->initCfg = *initCfg;   /* Structure copy */
	
	return (CPMAC_SUCCESS);
}
static int DDC_cpmacDeInit (CpmacDDCObj *hDDC, void* param)
{
	return (CPMAC_SUCCESS);
}
	
static int DDC_cpmacOpen(CpmacDDCObj *hDDC, void* param)
{
	int  retCode;
	unsigned int channel;
	unsigned int miiModId,  miiRevMaj,  miiRevMin;
	int retVal;
	CpmacInitConfig *initCfg;
	
	if (hDDC->ddcObj.state == DDC_OPENED) {
		return (CPMAC_ERR_DEV_ALREADY_OPEN);
	}
	
	avalanche_reset_ctrl(hDDC->initCfg.resetLine, IN_RESET);
	avalanche_reset_ctrl(hDDC->initCfg.resetLine,OUT_OF_RESET);
	avalanche_reset_ctrl(hDDC->initCfg.mdioResetLine,  OUT_OF_RESET);
	initCfg = &hDDC->initCfg;
	hDDC->regs = (CSL_CpmacRegsOvly) initCfg->baseAddress;
	for (channel = 0; channel < CPMAC_MAX_TX_CHANNELS; channel++) {
		hDDC->regs->Tx_HDP[channel] = 0;
	}

	for (channel = 0; channel < CPMAC_MAX_RX_CHANNELS; channel++) {
		hDDC->regs->Rx_HDP[channel] = 0;
	}
	hDDC->regs->Tx_Control |= CPMAC_TX_CONTROL_TX_ENABLE_VAL;
	hDDC->regs->Rx_Control |= CPMAC_RX_CONTROL_RX_ENABLE_VAL;
	hDDC->regs->Mac_IntMask_Set = CPMAC_MAC_HOST_ERR_INTMASK_VAL;
	hDDC->ddcObj.state = DDC_OPENED;
	cpmacSetMacHwCfg(hDDC);
	cpsw_halcommon_mii_mdio_getver(initCfg->mdioBaseAddress, &miiModId,  &miiRevMaj,  &miiRevMin);
	
	retCode = avalanche_memalloc(cpsw_halcommon_mii_mdio_getphydevsize(), (void* *)&hDDC->PhyDev);  
	if (retCode != 0) {
		return (retCode);
	}
	cpsw_hal_common_mii_mdio_init (hDDC->PhyDev, initCfg->mdioBaseAddress,  
			hDDC->ddcObj.instId, 
			initCfg->PhyMask, 
			initCfg->MLinkMask, 
			0,                   
			initCfg->mdioResetLine, 
			initCfg->MdioBusFrequency, 
			initCfg->MdioClockFrequency); 
	
	cpmacSetPhyMode(hDDC);
	hDDC->ddaIf->ddaNetIf.ddaFuncTable.ddaControlCb(hDDC->ddcObj.hDDA, CPMAC_DDA_IOCTL_STATUS_UPDATE, (void*) &hDDC->status, NULL);
	
	hDDC->ddaIf->ddaNetIf.ddaFuncTable.ddaControlCb(hDDC->ddcObj.hDDA, CPMAC_DDA_IOCTL_TIMER_START, (void*) initCfg->MdioTickMSec, NULL);
	
	for (channel = 0; channel < CPMAC_MAX_TX_CHANNELS; channel++) {
		if (hDDC->txCppi[channel] != NULL) {
			retVal = cpmacEnableChannel(hDDC, channel, DDC_NET_CH_DIR_TX);
			if (retVal != CPMAC_SUCCESS) {
				return (retVal); 
			}
		}
	}
	
	hDDC->regs->Rx_FilterLowThresh = 0;
	hDDC->regs->Rx_Unicast_Clear = CPMAC_RX_UNICAST_CLEAR_ALL;
	hDDC->regs->MacHash1 = hDDC->MacHash1;
	hDDC->regs->MacHash2 = hDDC->MacHash2;
	cpmacSetRxHwCfg(hDDC);
	hDDC->RxAddrType = (hDDC->regs->Mac_Cfig >> 8) & 0xFF;
	cpmacTypeXAddrInit(hDDC);
	for (channel = 0; channel < CPMAC_MAX_RX_CHANNELS; channel++) {
		if (hDDC->rxCppi[channel] != NULL) {
			retVal = cpmacEnableChannel(hDDC, channel, DDC_NET_CH_DIR_RX);
			if (retVal != CPMAC_SUCCESS) {
				return (retVal); 
			}
		}
	
		hDDC->regs->Rx_FlowThresh[channel] = 0;
		hDDC->regs->Rx_FreeBuffer[channel] = 0;
	}
	
	hDDC->MacControl |= (1 << CSL_CPMAC_MACCONTROL_MIIEN_SHIFT);
	hDDC->regs->MacControl = hDDC->MacControl;
	hDDC->ddaIf->ddaNetIf.ddaFuncTable.ddaControlCb(hDDC->ddcObj.hDDA, CPMAC_DDA_IOCTL_MIB64_CNT_TIMER_START, (void*) initCfg->Mib64CntMsec, NULL);
	return (CPMAC_SUCCESS);  
}
	
static int DDC_cpmacClose(CpmacDDCObj *hDDC, void* param)
{
	int retVal;
	int errVal = CPMAC_SUCCESS;
	unsigned int channel;
	
	if (hDDC->ddcObj.state == DDC_CLOSED) {
		return (CPMAC_ERR_DEV_ALREADY_CLOSED);
	}
	hDDC->ddaIf->ddaNetIf.ddaFuncTable.ddaControlCb(hDDC->ddcObj.hDDA, CPMAC_DDA_IOCTL_TIMER_STOP, NULL, NULL);
	
	hDDC->ddaIf->ddaNetIf.ddaFuncTable.ddaControlCb(hDDC->ddcObj.hDDA, CPMAC_DDA_IOCTL_MIB64_CNT_TIMER_STOP, NULL, NULL);
	
	for (channel = 0; channel < CPMAC_MAX_TX_CHANNELS; channel++) {
		if (hDDC->txCppi[channel] != NULL) {
			retVal = DDC_cpmacChClose (hDDC, channel, DDC_NET_CH_DIR_TX, NULL);
			if (retVal != CPMAC_SUCCESS) {
				errVal = retVal;
			}
		}
	}
	
	for (channel = 0; channel < CPMAC_MAX_RX_CHANNELS; channel++) {
		if (hDDC->rxCppi[channel] != NULL) {
			retVal = DDC_cpmacChClose (hDDC, channel, DDC_NET_CH_DIR_RX, NULL);
			if (retVal != CPMAC_SUCCESS) {
				errVal = retVal;  /* return (retVal); */
			}
		}
	}
	
	avalanche_reset_ctrl(hDDC->initCfg.resetLine, IN_RESET);
	
	if (CpmacDDCNumInst == 1)   {
		avalanche_reset_ctrl(hDDC->initCfg.mdioResetLine,  IN_RESET);
	}

	if(errVal == CPMAC_SUCCESS) {
		hDDC->ddcObj.state = DDC_CLOSED;
	}
	return (errVal);
}
	
static int DDC_cpmacControl(CpmacDDCObj *hDDC, int cmd, void* cmdArg, void* param)
{
	switch (cmd) {
	case CPMAC_DDC_IOCTL_GET_SWVER:
		*((unsigned int *)cmdArg) = (unsigned int) ((CPMAC_DDC_MAJOR_VERSION << 16) | CPMAC_DDC_MINOR_VERSION);
		*((char **)param) = (char *)&CpmacDDCVersionString[0];
		break;
	
	case CPMAC_DDC_IOCTL_GET_HWVER:
		if (hDDC->ddcObj.state == DDC_OPENED) {
			*((unsigned int *)cmdArg) = hDDC->regs->Tx_IdVer;
			*((unsigned int *)param)  = hDDC->regs->Rx_IdVer;
		}
		else  { 
			return (CPMAC_ERR_DEV_NOT_OPEN); 
		}
		break;
	
	case CPMAC_DDC_IOCTL_SET_RXCFG:
		if (cmdArg != NULL) {
			hDDC->initCfg.rxCfg = *((CpmacRxConfig *)cmdArg);
			cpmacSetRxHwCfg(hDDC); 
		}
		else  { 
			return (CPMAC_INVALID_PARAM); 
		}
		break;
	
	case CPMAC_DDC_IOCTL_SET_MACCFG:
		if (cmdArg != NULL) {
			hDDC->initCfg.macCfg = *((CpmacMacConfig *)cmdArg);
			cpmacSetMacHwCfg(hDDC); /* Set the MAC Control register */
		}	
		else  { 
			return (CPMAC_INVALID_PARAM); 
		}
		break;
	
	case CPMAC_DDC_IOCTL_GET_STATUS:
		if (hDDC->ddcObj.state != DDC_OPENED) {
			return (CPMAC_ERR_DEV_NOT_OPEN);	
		}
		{   /* Scope {} introduced here to declare status as local scope variable */
			CpmacDDCStatus *status = (CpmacDDCStatus *)cmdArg;
			*status = hDDC->status; /* structure copy */
		}
		break;
	
	case CPMAC_DDC_IOCTL_READ_PHY_REG:
		if (hDDC->ddcObj.state != DDC_OPENED) {
			return (CPMAC_ERR_DEV_NOT_OPEN);
		}
		{
			CpmacPhyParams *phyParams = (CpmacPhyParams *) cmdArg;
			phyParams->data = _cpsw_halcommon_mii_mdio_user_accessread(hDDC->PhyDev, phyParams->regAddr, phyParams->phyNum);
		}
		break;
	
	case CPMAC_DDC_IOCTL_WRITE_PHY_REG:
		if (hDDC->ddcObj.state != DDC_OPENED) {
			return (CPMAC_ERR_DEV_NOT_OPEN);
		}
		{
			CpmacPhyParams *phyParams = (CpmacPhyParams *) cmdArg;
			_cpsw_halcommon_mii_mdiouser_accesswrite(hDDC->PhyDev, phyParams->regAddr, phyParams->phyNum, phyParams->data);
		}
		break;
	
	case CPMAC_DDC_IOCTL_GET_STATISTICS:
		if (hDDC->ddcObj.state != DDC_OPENED) {
			return (CPMAC_ERR_DEV_NOT_OPEN);
		}
		{   
			unsigned int cnt;
			unsigned int *userStats = (unsigned int *)cmdArg;
			volatile unsigned int *addr = (unsigned int *) &hDDC->regs->RxGoodFrames;
			for (cnt=0; cnt < CPMAC_NUM_STAT_REGS; cnt++, userStats++, addr++) {
				*userStats = *addr;
			} 
		}
		break;
	
	case CPMAC_DDC_IOCTL_CLR_STATISTICS:
		if (hDDC->ddcObj.state != DDC_OPENED) {
			return (CPMAC_ERR_DEV_NOT_OPEN);
		}
		{
			unsigned int cnt;
			volatile unsigned int *addr = (unsigned int *) &hDDC->regs->RxGoodFrames;
			for (cnt = 0; cnt < CPMAC_NUM_STAT_REGS; cnt++, addr++) {
				*addr = CPMAC_STAT_CLEAR; /* 0xFFFFFFFF value */
			} 
	
			cpmacDDCIfcntClear(hDDC);
		}
		break;
	
	case CPMAC_DDC_IOCTL_MULTICAST_ADDR:
		if (hDDC->ddcObj.state != DDC_OPENED) {
			return (CPMAC_ERR_DEV_NOT_OPEN);
		}
		{
			unsigned char *addr = (unsigned char *) param;
			cpmacSingleMulti(hDDC, (CpmacSingleMultiOper)cmdArg, addr);
		}	
		break;
	
	case CPMAC_DDC_IOCTL_ALL_MULTI:
		/* cmdArg= CpmacAllMultiOper enum, param=not used */
		if (hDDC->ddcObj.state != DDC_OPENED) {
			return (CPMAC_ERR_DEV_NOT_OPEN);
		}
		cpmacAllMulti(hDDC, (CpmacAllMultiOper)cmdArg);
		break;
	
	case CPMAC_DDC_IOCTL_TYPE2_3_FILTERING:
		{
			CpmacType2_3_AddrFilterParams *addrParams;
			if (hDDC->ddcObj.state != DDC_OPENED) {
				return (CPMAC_ERR_DEV_NOT_OPEN);
			}
			addrParams = (CpmacType2_3_AddrFilterParams *) cmdArg;
			cpmacAddType2Addr(hDDC, addrParams->channel, addrParams->macAddress, 
			addrParams->index, addrParams->valid, addrParams->match);
		}
		break;
	
	case CPMAC_DDC_IOCTL_SET_MAC_ADDRESS:
		{
			CpmacAddressParams *addrParams;
			CpmacRxCppiCh  *rxCppi;
			int cnt;
	
			if (hDDC->ddcObj.state != DDC_OPENED) {
				return (CPMAC_ERR_DEV_NOT_OPEN);
			}
			addrParams = (CpmacAddressParams *) cmdArg;
	
			rxCppi = hDDC->rxCppi[addrParams->channel];
			if (rxCppi == NULL) {
				return (CPMAC_ERR_RX_CH_INVALID);
			}
	
			for (cnt=0; cnt < 6; cnt++)
				rxCppi->macAddr[cnt] = addrParams->macAddress[cnt];                 
	
			cpmacSetMacAddress(hDDC, addrParams->channel, addrParams->macAddress); 
			cpmacSetSrcMacAddress(hDDC, addrParams->macAddress);
		
		}
		break;
			
	case CPMAC_DDC_IOCTL_IF_COUNTERS:			
	       if (hDDC->ddcObj.state != DDC_OPENED) {
			    return (CPMAC_ERR_DEV_NOT_OPEN);
		}		
	        cpmacDDCIfcntUpdt(hDDC);
		memcpy((char *)cmdArg,
		      (char*)&hDDC->Mib2IfHCCounter.Mib2IfCounter,
		      sizeof(struct mib2_ifCounters));
		break;			    
			
	    case CPMAC_DDC_IOCTL_ETHER_COUNTERS:     		
		if (hDDC->ddcObj.state != DDC_OPENED) {
			return (CPMAC_ERR_DEV_NOT_OPEN);
		}		
		cpmacDDCPhycnt(hDDC, cmdArg);
		break;	    	
			
	    case CPMAC_DDC_IOCTL_IF_PARAMS_UPDT: 	    
		if (hDDC->ddcObj.state != DDC_OPENED) {
			return (CPMAC_ERR_DEV_NOT_OPEN);
		}
		cpmacDDCIfcntUpdt(hDDC);
		break;
				
	default:
		break;
	}   
	
	return (CPMAC_SUCCESS);
}
	
static int DDC_cpmacChOpen(CpmacDDCObj *hDDC, CpmacChInfo *chInfo, void* chOpenArgs)
{
	int retVal;
	
	if (chInfo->chState != DDC_NET_CH_UNINITIALIZED) {
		return (CPMAC_INVALID_PARAM);
	}
	
	if (chInfo->chDir == DDC_NET_CH_DIR_TX) {
		if (chInfo->chNum >= hDDC->initCfg.numTxChannels) {
			return (CPMAC_ERR_TX_CH_INVALID);
		}
	
		if (hDDC->txIsCreated[chInfo->chNum] == True) {
			return (CPMAC_ERR_TX_CH_ALREADY_INIT);
		}
	
		retVal = cpmacInitTxChannel(hDDC, chInfo, chOpenArgs);
		if (retVal != CPMAC_SUCCESS) {
			return (retVal);
		}
	}
	else if (chInfo->chDir == DDC_NET_CH_DIR_RX) {
		if (chInfo->chNum >= hDDC->initCfg.numRxChannels) {
			return (CPMAC_ERR_RX_CH_INVALID);
		}
	
		if (hDDC->rxIsCreated[chInfo->chNum] == True) {
			return (CPMAC_ERR_RX_CH_ALREADY_INIT);	
		}
	
		retVal = cpmacInitRxChannel(hDDC, chInfo, chOpenArgs);
		if (retVal != CPMAC_SUCCESS) {
			return (retVal);
		}
	}
	
	if (hDDC->ddcObj.state == DDC_OPENED) {
		retVal = cpmacEnableChannel(hDDC, chInfo->chNum, chInfo->chDir);
		if (retVal != CPMAC_SUCCESS) {
			return (retVal);
		}
	}
	
	return (CPMAC_SUCCESS);
}
	
static int DDC_cpmacChClose(CpmacDDCObj *hDDC, int channel, int direction, void* chCloseArgs)
{
	int retVal;
	
	if (hDDC->ddcObj.state == DDC_OPENED) {
		retVal = cpmacDisableChannel(hDDC, channel, direction);
		if (retVal != CPMAC_SUCCESS) {
			return (retVal);
		}
	}
	
	if (direction == DDC_NET_CH_DIR_TX) {
		retVal = cpmacUnInitTxChannel(hDDC, channel, chCloseArgs);
		if (retVal != CPMAC_SUCCESS) {
			return (retVal);
		}
	}
	else if (direction == DDC_NET_CH_DIR_RX) {
		retVal = cpmacUnInitRxChannel(hDDC, channel, chCloseArgs);
		if (retVal != CPMAC_SUCCESS) {
			return (retVal);
		}
	}
	
	return (CPMAC_SUCCESS);
}

static int cpmacInitTxChannel(CpmacDDCObj *hDDC, CpmacChInfo *chInfo, void* chOpenArgs)
{
	int retCode;
	unsigned int cnt, bdSize;
	char *allocMem;
	CpmacTxBD *currBD;
	CpmacTxCppiCh *txCppi = NULL;
	
	retCode = avalanche_memalloc(sizeof(CpmacTxCppiCh), (void* *)&txCppi);  
	if (retCode != 0) {
		return (retCode);
	}
	memset(txCppi, 0, sizeof(CpmacTxCppiCh));
	
	hDDC->txCppi[chInfo->chNum] = txCppi;
	
	txCppi->chInfo = *chInfo;      /* Structure copy */
	txCppi->chInfo.chState = DDC_NET_CH_INITIALIZED;
	txCppi->activeQueueHead = 0;
	txCppi->activeQueueTail = 0;
	txCppi->queueActive = False;
	hDDC->txTeardownPending[chInfo->chNum] = False; 
	
	retCode = avalanche_memalloc((chInfo->serviceMax * sizeof(unsigned int)), 
				(void* *)&txCppi->txComplete);  
	if (retCode != 0) {
		avalanche_memfree(txCppi);
		return (retCode);
	}
	memset(txCppi->txComplete, 0, (chInfo->serviceMax * sizeof(unsigned int)));
	bdSize = (sizeof(CpmacTxBD)+0xF) & ~0xF;
	txCppi->allocSize = (bdSize * chInfo->numBD) + 0xF;
	
	retCode = avalanche_memalloc(txCppi->allocSize, (void* *)&txCppi->bdMem);  
	if (retCode != 0) {
		return (retCode);
	}
	memset(txCppi->bdMem, 0, txCppi->allocSize);
	
	allocMem = (char *)(((unsigned int)txCppi->bdMem + 0xF) & ~0xF);
	txCppi->bdPoolHead = 0;
	for (cnt = 0; cnt < chInfo->numBD; cnt++) {
		currBD = (CpmacTxBD *) (allocMem + (cnt * bdSize));
		currBD->next = txCppi->bdPoolHead;
		txCppi->bdPoolHead = currBD;
	}
	
	txCppi->outOfTxBD = 0;
	txCppi->noActivePkts = 0;
	txCppi->activeQueueCount = 0;
	
	hDDC->txIsCreated[chInfo->chNum] = True;
	return (CPMAC_SUCCESS);    
}
static int cpmacUnInitTxChannel(CpmacDDCObj *hDDC, unsigned int channel, void* chCloseArgs)
{
	int retCode;
	CpmacTxCppiCh *txCppi;
	
	if (hDDC->txIsCreated[channel] == False) {
		return (CPMAC_ERR_TX_CH_ALREADY_CLOSED);
	}
	txCppi = hDDC->txCppi[channel];
	
	if (txCppi->bdMem != NULL) {
		retCode = avalanche_memfree(txCppi->bdMem);
		txCppi->bdMem = NULL;
	}
	
	retCode = avalanche_memfree(txCppi->txComplete);
	retCode = avalanche_memfree(txCppi);
	hDDC->txCppi[channel] = NULL;
	hDDC->txIsCreated[channel] = False;
	
	return (CPMAC_SUCCESS);
}
static int cpmacInitRxChannel(CpmacDDCObj *hDDC, CpmacChInfo *chInfo, void* chOpenArgs)
{
	int retCode;
	unsigned int cnt, bdSize;
	char *allocMem;
	CpmacRxBD *currBD;
	CpmacRxCppiCh *rxCppi = NULL;
	
	retCode = avalanche_memalloc(sizeof(CpmacRxCppiCh), (void* *)&rxCppi);  
	if (retCode != 0) {
		return (retCode);
	}
	memset(rxCppi, 0, sizeof(CpmacRxCppiCh));
	
	hDDC->rxCppi[chInfo->chNum] = rxCppi;
	
	rxCppi->hDDC = hDDC;     
	rxCppi->chInfo = *chInfo;      /* Structure copy */
	rxCppi->chInfo.chState = DDC_NET_CH_INITIALIZED;
	hDDC->rxTeardownPending[chInfo->chNum] = False; 
	
	allocMem = (char *)chOpenArgs;  /* reusing allocMem local variable as char pointer */
	for (cnt=0; cnt < 6; cnt++)
		rxCppi->macAddr[cnt] = allocMem[cnt];
	
	bdSize = (sizeof(CpmacRxBD)+0xF) & ~0xF;
	rxCppi->allocSize = (bdSize * chInfo->numBD) + 0xF;
	
	retCode = avalanche_memalloc(rxCppi->allocSize, (void* *)&rxCppi->bdMem);  
	if (retCode != 0) {
		return (retCode);
	}
	memset(rxCppi->bdMem, 0, rxCppi->allocSize);
	
	retCode = avalanche_memalloc((chInfo->serviceMax * sizeof(DDC_NetPktObj)), (void* *)&rxCppi->pktQueue);  
	if (retCode != 0) {
		/* Free tx cppi channel memory */
		avalanche_memfree(rxCppi);
		return (retCode);
	}
	memset(rxCppi->pktQueue, 0, (chInfo->serviceMax * sizeof(DDC_NetPktObj)));
	
	retCode = avalanche_memalloc((chInfo->serviceMax * sizeof(DDC_NetBufObj) * CPMAC_MAX_RX_FRAGMENTS), (void* *)&rxCppi->bufQueue);  
	if (retCode != 0) {
		avalanche_memfree(rxCppi);
		return (retCode);
	}
	memset(rxCppi->bufQueue, 0, (chInfo->serviceMax * sizeof(DDC_NetBufObj) * CPMAC_MAX_RX_FRAGMENTS));
	
	{
		DDC_NetPktObj   *currPkt = &rxCppi->pktQueue[0];
		DDC_NetBufObj   *currBuf = &rxCppi->bufQueue[0];
		/* Bind pkt and buffer queue data structures */
		for (cnt = 0; cnt < chInfo->serviceMax; cnt++) {
			currPkt->bufList = currBuf;
			++currPkt;
		currBuf += CPMAC_MAX_RX_FRAGMENTS;
		}
	}
	
	/* Allocate RX buffer and initialize the BD linked list */
	allocMem = (char *)(((unsigned int)rxCppi->bdMem + 0xF) & ~0xF);
	rxCppi->activeQueueHead = 0;
	rxCppi->activeQueueTail = (CpmacRxBD *) allocMem;
	for (cnt = 0; cnt < chInfo->numBD; cnt++) {
		currBD = (CpmacRxBD *) (allocMem + (cnt * bdSize));
	
		currBD->data = (void*) (hDDC->ddaIf->ddaNetIf.ddaNetAllocRxBufCb(hDDC->ddcObj.hDDA, chInfo->bufSize, (DDC_NetDataToken *)&currBD->bufToken, (void*) currBD));
		if (currBD->data == NULL) {
			return (CPMAC_ERR_RX_BUFFER_ALLOC_FAIL);
		}
	
		currBD->hNext = PAL_CPMAC_VIRT_2_PHYS(rxCppi->activeQueueHead);
		currBD->buff = PAL_CPMAC_VIRT_2_PHYS(currBD->data);
		currBD->off_bLen = chInfo->bufSize;
		currBD->mode = CPMAC_CPPI_OWNERSHIP_BIT;
		PAL_CPMAC_CACHE_WRITEBACK(currBD, CPMAC_BD_LENGTH_FOR_CACHE);
		currBD->next = (void*) rxCppi->activeQueueHead;
		rxCppi->activeQueueHead = currBD;
	}
	hDDC->rxIsCreated[chInfo->chNum] = True;
	
	return (CPMAC_SUCCESS);    
}
static int cpmacUnInitRxChannel(CpmacDDCObj *hDDC, unsigned int channel, void* chCloseArgs)
{
	int retCode;
	CpmacRxCppiCh *rxCppi;
	CpmacRxBD *currBD;
	
	if (hDDC->rxIsCreated[channel] == False) {
		return (CPMAC_ERR_RX_CH_ALREADY_CLOSED);
	}
	rxCppi = hDDC->rxCppi[channel];
	
	currBD = rxCppi->activeQueueHead;
	
	while (currBD) {
		currBD = currBD->next;
	}
	
	if (rxCppi->bdMem != NULL) {
		retCode = avalanche_memfree(rxCppi->bdMem);
		rxCppi->bdMem = NULL;
	}
	
	if(rxCppi->extraBdMem != NULL) {
		while(rxCppi->extraBdMem) {
			    retCode = avalanche_memfree(rxCppi->extraBdMem->extraAllocMem);
			    avalanche_memfree(rxCppi->extraBdMem);
			    rxCppi->extraBdMem = (CpmacBdMemChunk *)rxCppi->extraBdMem->ptr_next;
		}
		rxCppi->extraBdMem = NULL;
	}
	
	retCode = avalanche_memfree(rxCppi);
	hDDC->rxCppi[channel] = NULL;
	hDDC->rxIsCreated[channel] = False;
	
	return (CPMAC_SUCCESS);
}
	
static int cpmacAddRxBd(CpmacDDCObj *hDDC, CpmacChInfo *chInfo,unsigned int numOfBd2Add)
{
	int retCode;
	unsigned int cnt, bdSize,allocSize;
	CpmacRxBD *currBD,*QueueTail,*QueueHead,*allocMem;
	CpmacRxCppiCh *rxCppi;
	CpmacBdMemChunk *MemChunk;
	
	rxCppi = hDDC->rxCppi[chInfo->chNum];
	bdSize = (sizeof(CpmacRxBD)+0xF) & ~0xF;
		allocSize = (bdSize * numOfBd2Add) + 0xF;
	
	retCode = avalanche_memalloc(sizeof(CpmacBdMemChunk), (void* *)&MemChunk);  
	if (retCode != 0) {
		return CPMAC_ERR_RX_BUFFER_ALLOC_FAIL;
	}
	
	memset(MemChunk, 0,sizeof(CpmacBdMemChunk));
	
	retCode = avalanche_memalloc(allocSize, (void* *)&MemChunk->extraAllocMem);  
	if (retCode != 0) {
		avalanche_memfree(MemChunk);
		return CPMAC_ERR_RX_BUFFER_ALLOC_FAIL;
	}
	
	/* Set memory to 0 */
	memset(MemChunk->extraAllocMem, 0,allocSize);
	
	/* Allocate RX buffer and initialize the BD linked list */
	allocMem = (CpmacRxBD *)(((unsigned int)MemChunk->extraAllocMem + 0xF) & ~0xF);
		
	QueueHead = NULL;;
	QueueTail = allocMem;
	
	for (cnt = 0; cnt < numOfBd2Add; cnt++) {
		currBD = allocMem + cnt;
	
		currBD->data = (void*) (hDDC->ddaIf->ddaNetIf.ddaNetAllocRxBufCb(hDDC->ddcObj.hDDA, chInfo->bufSize, (DDC_NetDataToken *)&currBD->bufToken, (void*) currBD));
		if (currBD->data == NULL) {
			goto errorFreeExtraMem;
		}
	
		currBD->hNext = PAL_CPMAC_VIRT_2_PHYS(QueueHead);
		currBD->buff = PAL_CPMAC_VIRT_2_PHYS(currBD->data);
		currBD->off_bLen = chInfo->bufSize;
		currBD->mode = CPMAC_CPPI_OWNERSHIP_BIT;
		PAL_CPMAC_CACHE_WRITEBACK(currBD, CPMAC_BD_LENGTH_FOR_CACHE);
		currBD->next = (void*) QueueHead;
		QueueHead = currBD;
	}
	
	cpmacAddListToRxQueue(hDDC,rxCppi, QueueHead/*first*/, QueueTail/*last*/);
	
	{
		unsigned int flags;
	
		local_irq_save(flags);  /* Start of Receive Critical Section */
		/* Update the number of RX BD */
		chInfo->numBD += numOfBd2Add;
		/* Update the memory list */
		if (rxCppi->extraBdMem == NULL)  {
			rxCppi->extraBdMem = MemChunk;
		}
		else {
			MemChunk->ptr_next = ( struct CpmacBdMemChunk *)rxCppi->extraBdMem;
			rxCppi->extraBdMem = MemChunk;
		}
	
		local_irq_restore(flags);     /* End of Receive Critical Section */
	}
	
	return (CPMAC_SUCCESS);    
	
errorFreeExtraMem:
	while (QueueHead) {
		QueueHead = QueueHead->next;
	}
	
	retCode = avalanche_memfree((void* *)MemChunk->extraAllocMem);
	retCode = avalanche_memfree((void* *)MemChunk);
	
	return (CPMAC_ERR_RX_BUFFER_ALLOC_FAIL);
}
	
static void cpmacSetMacAddress(CpmacDDCObj *hDDC, unsigned int channel, unsigned char* macAddr)
{
	hDDC->regs->Rx_Unicast_Set = (1 << channel);
	
	if (hDDC->RxAddrType == RX_ADDR_TYPE0)
		cpmacAddType0Addr(hDDC, channel, macAddr);
	else if (hDDC->RxAddrType == RX_ADDR_TYPE1)
		cpmacAddType1Addr(hDDC, channel, macAddr);
	else if (hDDC->RxAddrType == RX_ADDR_TYPE2)
		cpmacAddType2Addr(hDDC, channel, macAddr, 0, 1, 1);
}
	
static int cpmacEnableChannel(CpmacDDCObj *hDDC, unsigned int channel, unsigned int direction)
{
	if (direction == DDC_NET_CH_DIR_TX) {
		CpmacTxCppiCh  *txCppi;
	
		txCppi = hDDC->txCppi[channel];

		if (txCppi == NULL) {
			return (CPMAC_ERR_TX_CH_INVALID);
		}
		hDDC->regs->Tx_HDP[channel] = 0;
	
		{	
			CpmacMacConfig  *macCfg;
			macCfg = &hDDC->initCfg.macCfg;
	
			if(macCfg->txInterruptDisable == True) {
				hDDC->regs->Tx_IntMask_Clear = (1 << channel);
				hDDC->txInterruptDisable = True;
				hDDC->txIntThreshold[channel] = macCfg->txIntThresholdValue;
			}
			else {
				hDDC->regs->Tx_IntMask_Set = (1 << channel);
				hDDC->txInterruptDisable = False;
			}
		}

		hDDC->txIsOpen[channel] = True;
		txCppi->chInfo.chState = DDC_NET_CH_OPENED;
	}
	else if (direction == DDC_NET_CH_DIR_RX) {
		CpmacRxCppiCh  *rxCppi;
	
		rxCppi = hDDC->rxCppi[channel];
		if (rxCppi == NULL) {
			return (CPMAC_ERR_RX_CH_INVALID);
		}
	
		cpmacSetMacAddress(hDDC, channel,rxCppi->macAddr);
		cpmacSetSrcMacAddress(hDDC, rxCppi->macAddr);
		hDDC->regs->Rx_IntMask_Set = (1 << channel);
		rxCppi->queueActive = True;
		hDDC->regs->Rx_HDP[channel] = PAL_CPMAC_VIRT_2_PHYS(rxCppi->activeQueueHead);
		hDDC->rxIsOpen[channel] = True;
		rxCppi->chInfo.chState = DDC_NET_CH_OPENED;
	}
	
	return (CPMAC_SUCCESS);
}
	
	
static int cpmacDisableChannel(CpmacDDCObj *hDDC, unsigned int channel, DDC_NetChDir direction)
{
	if (direction == DDC_NET_CH_DIR_TX) {
		hDDC->txTeardownPending[channel] = True;
		hDDC->regs->Tx_Teardown = channel;
	
		if (cpmacWaitForTeardownComplete(hDDC, channel, direction, True) != CPMAC_SUCCESS) {
			/* do nothing */
		}
	
		hDDC->txTeardownPending[channel] = False;   /* Clear the TX teardown pending flag */
		hDDC->regs->Tx_IntMask_Clear = (1 << channel);
		hDDC->txIsOpen[channel] = False;
	}
	else if (direction == DDC_NET_CH_DIR_RX) {
		hDDC->rxTeardownPending[channel] = True;    /* Set the RX teardown pending flag */
		hDDC->regs->Rx_Teardown = channel;
		hDDC->rxTeardownPending[channel] = False;   /* Clear the RX teardown pending flag */
		hDDC->regs->Rx_IntMask_Clear = (1 << channel);
		hDDC->rxIsOpen[channel] = False;
	}
	
	return (CPMAC_SUCCESS);
}

static int cpmacWaitForTeardownComplete(CpmacDDCObj *hDDC, unsigned int channel, DDC_NetChDir direction, Bool blocking)
{
	if (direction == DDC_NET_CH_DIR_TX) {
		CpmacTxBD *currBD;
		CpmacTxCppiCh *txCppi;
	
		do {
		} while ( (hDDC->regs->Tx_CP[channel] & CPMAC_TEARDOWN_VALUE) != CPMAC_TEARDOWN_VALUE);
	
		hDDC->regs->Tx_CP[channel] = CPMAC_TEARDOWN_VALUE;
	
		txCppi = hDDC->txCppi[channel];
		if(txCppi->queueActive == True)  {
			currBD = txCppi->activeQueueHead;
			while (currBD != NULL) {
				hDDC->ddaIf->ddaNetIf.ddaNettxCompleteCb(hDDC->ddcObj.hDDA, &(currBD->bufToken), 1, (void*)channel);            
				if(currBD != txCppi->activeQueueTail) {
					currBD = currBD->next;
				} else {
					break;     
				}
			}
			txCppi->bdPoolHead = txCppi->activeQueueHead;
			txCppi->activeQueueHead = txCppi->activeQueueTail = 0;
		}
	}
	else if (direction == DDC_NET_CH_DIR_RX) {
		do {
		} while ( (hDDC->regs->Rx_CP[channel] & CPMAC_TEARDOWN_VALUE) != CPMAC_TEARDOWN_VALUE);
		hDDC->regs->Rx_CP[channel] = CPMAC_TEARDOWN_VALUE;
	}
	
	return (CPMAC_SUCCESS);
}
	
static void cpmacDDCPhycnt(CpmacDDCObj *hDDC, unsigned int *cmdArg)
{
	int result;		
	CpmacHwStatistics   stats;	 
	struct mib2_phyCounters *mib2PhyCounters = (struct mib2_phyCounters *)cmdArg;
	result = DDC_cpmacControl(hDDC, CPMAC_DDC_IOCTL_GET_STATISTICS, (unsigned int *)&stats, NULL);
	if (result != 0) {
		return ;
	}
		 
	mib2PhyCounters->ethAlignmentErrors = stats.ifInAlignCodeErrors;
	mib2PhyCounters->ethFCSErrors = stats.ifInCRCErrors;
	mib2PhyCounters->ethSingleCollisions = stats.ifSingleCollisionFrames;	
	mib2PhyCounters->ethMultipleCollisions = stats.ifMultipleCollisionFrames;
	mib2PhyCounters->ethSQETestErrors = 0;
	mib2PhyCounters->ethDeferredTxFrames = stats.ifDeferredTransmissions;	
	mib2PhyCounters->ethLateCollisions = stats.ifLateCollisions;
	mib2PhyCounters->ethExcessiveCollisions = stats.ifExcessiveCollisionFrames;	
	mib2PhyCounters->ethInternalMacTxErrors = 0;  
	mib2PhyCounters->ethCarrierSenseErrors = stats.ifCarrierSenseErrors;
	mib2PhyCounters->ethTooLongRxFrames = stats.ifInOversizedFrames;  
	mib2PhyCounters->ethInternalMacRxErrors = 0; 
	mib2PhyCounters->ethSymbolErrors = 0;

	return;
}
		 
unsigned int ts_ddc_stat_dbg=0;	
static void cpmacDDCIfcntClear(CpmacDDCObj *hDDC)
{
	memset((char *)&hDDC->Mib2IfHCCounter, 0, sizeof(hDDC->Mib2IfHCCounter));
}
	
	
static void cpmacDDCIfcntUpdt(CpmacDDCObj *hDDC)
{
	int result;		
	CpmacHwStatistics stats;
		 
	result = DDC_cpmacControl(hDDC, CPMAC_DDC_IOCTL_GET_STATISTICS, (unsigned int *)&stats, NULL);
	
	if (result != 0) {
		return ;
	}
	if(stats.ifInOctets >= hDDC->Mib2IfHCCounter.inBytes) {
		hDDC->Mib2IfHCCounter.inBytesHC += (stats.ifInOctets - hDDC->Mib2IfHCCounter.inBytes);	
	}
	else {
	        hDDC->Mib2IfHCCounter.inBytesHC += 0xffffffff - (hDDC->Mib2IfHCCounter.inBytes - stats.ifInOctets);	
	}
	hDDC->Mib2IfHCCounter.inBytes = stats.ifInOctets;			
		 
	if(stats.ifInGoodFrames >= hDDC->Mib2IfHCCounter.inMulticastPkts +
	 			hDDC->Mib2IfHCCounter.inBroadcastPkts +
				hDDC->Mib2IfHCCounter.inUnicastPkts) {
		 hDDC->Mib2IfHCCounter.inUnicastPktsHC +=((stats.ifInGoodFrames - 
		  (stats.ifInBroadcasts + stats.ifInMulticasts))
		  - hDDC->Mib2IfHCCounter.inUnicastPkts);
	}
	else {
		hDDC->Mib2IfHCCounter.inUnicastPktsHC +=
			0xffffffff - (hDDC->Mib2IfHCCounter.inUnicastPkts -
			(stats.ifInGoodFrames - 
			(stats.ifInBroadcasts + stats.ifInMulticasts)));
	}
	 hDDC->Mib2IfHCCounter.inUnicastPkts = (stats.ifInGoodFrames - 
			(stats.ifInBroadcasts + stats.ifInMulticasts));			
		
			
	if(stats.ifInMulticasts >= hDDC->Mib2IfHCCounter.inMulticastPkts) {
		hDDC->Mib2IfHCCounter.inMulticastPktsHC +=(stats.ifInMulticasts - hDDC->Mib2IfHCCounter.inMulticastPkts);	
	}
	else {
		hDDC->Mib2IfHCCounter.inMulticastPktsHC +=
			0xffffffff - (hDDC->Mib2IfHCCounter.inMulticastPkts -
			stats.ifInMulticasts);	
	}
	hDDC->Mib2IfHCCounter.inMulticastPkts = stats.ifInMulticasts;			
		
	if(stats.ifInBroadcasts >= hDDC->Mib2IfHCCounter.inBroadcastPkts) {
		hDDC->Mib2IfHCCounter.inBroadcastPktsHC +=(stats.ifInBroadcasts - hDDC->Mib2IfHCCounter.inBroadcastPkts);	
	}
	else {
	
		hDDC->Mib2IfHCCounter.inBroadcastPktsHC +=
			0xffffffff - (hDDC->Mib2IfHCCounter.inBroadcastPkts -
			stats.ifInBroadcasts);	
	}
	hDDC->Mib2IfHCCounter.inBroadcastPkts = stats.ifInBroadcasts;					
	if(stats.ifOutOctets >= hDDC->Mib2IfHCCounter.outBytes) {
		hDDC->Mib2IfHCCounter.outBytesHC += (stats.ifOutOctets - hDDC->Mib2IfHCCounter.outBytes);	
	}
	else {
		hDDC->Mib2IfHCCounter.outBytesHC += 0xffffffff - (hDDC->Mib2IfHCCounter.outBytes - stats.ifOutOctets);	
	}

	hDDC->Mib2IfHCCounter.outBytes = stats.ifOutOctets;			
	
	if(stats.ifOutGoodFrames >= hDDC->Mib2IfHCCounter.outMulticastPkts +
				hDDC->Mib2IfHCCounter.outBroadcastPkts +
				hDDC->Mib2IfHCCounter.outUnicastPkts) {
					hDDC->Mib2IfHCCounter.outUnicastPktsHC +=((stats.ifOutGoodFrames - (stats.ifOutBroadcasts + stats.ifOutMulticasts)) - hDDC->Mib2IfHCCounter.outUnicastPkts);	
	}
	else
	{
		hDDC->Mib2IfHCCounter.outUnicastPktsHC +=
			0xffffffff - (hDDC->Mib2IfHCCounter.outUnicastPkts -
			(stats.ifOutGoodFrames - 
			(stats.ifOutBroadcasts + stats.ifOutMulticasts)));	
	}
	hDDC->Mib2IfHCCounter.outUnicastPkts= (stats.ifOutGoodFrames - 
		    (stats.ifOutBroadcasts + stats.ifOutMulticasts));				
			
	if(stats.ifOutMulticasts >= hDDC->Mib2IfHCCounter.outMulticastPkts) {
		hDDC->Mib2IfHCCounter.outMulticastPktsHC +=
			(stats.ifOutMulticasts - hDDC->Mib2IfHCCounter.outMulticastPkts);	
	}
	else {
		hDDC->Mib2IfHCCounter.outMulticastPktsHC +=
			0xffffffff - (hDDC->Mib2IfHCCounter.outMulticastPkts -
			stats.ifOutMulticasts);	
	}
	hDDC->Mib2IfHCCounter.outMulticastPkts = stats.ifOutMulticasts;			
	
	if(stats.ifOutBroadcasts >= hDDC->Mib2IfHCCounter.outBroadcastPkts) {
		hDDC->Mib2IfHCCounter.outBroadcastPktsHC +=
			(stats.ifOutBroadcasts - hDDC->Mib2IfHCCounter.outBroadcastPkts);	
	}
	else {
		hDDC->Mib2IfHCCounter.outBroadcastPktsHC +=
			0xffffffff - (hDDC->Mib2IfHCCounter.outBroadcastPkts -
			stats.ifOutBroadcasts);	
	}
	hDDC->Mib2IfHCCounter.outBroadcastPkts = stats.ifOutBroadcasts;			
	
	hDDC->Mib2IfHCCounter.Mib2IfCounter.inBytesLow = 
			(unsigned long)hDDC->Mib2IfHCCounter.inBytesHC;
	hDDC->Mib2IfHCCounter.Mib2IfCounter.inBytesHigh = 
			(hDDC->Mib2IfHCCounter.inBytesHC >> 32);
	
	hDDC->Mib2IfHCCounter.Mib2IfCounter.inUnicastPktsLow = 
			(unsigned long)hDDC->Mib2IfHCCounter.inUnicastPktsHC;
	hDDC->Mib2IfHCCounter.Mib2IfCounter.inUnicastPktsHigh = 
			(hDDC->Mib2IfHCCounter.inUnicastPktsHC >> 32);
	
	hDDC->Mib2IfHCCounter.Mib2IfCounter.inMulticastPktsLow = 
			(unsigned long)hDDC->Mib2IfHCCounter.inMulticastPktsHC;
	hDDC->Mib2IfHCCounter.Mib2IfCounter.inMulticastPktsHigh = 
			hDDC->Mib2IfHCCounter.inMulticastPktsHC >> 32;
		
	hDDC->Mib2IfHCCounter.Mib2IfCounter.inBroadcastPktsLow = 
			(unsigned long)hDDC->Mib2IfHCCounter.inBroadcastPktsHC;
	hDDC->Mib2IfHCCounter.Mib2IfCounter.inBroadcastPktsHigh = 
			hDDC->Mib2IfHCCounter.inBroadcastPktsHC >> 32;
	
	hDDC->Mib2IfHCCounter.Mib2IfCounter.inDiscardPkts = stats.ifRxDMAOverruns
			+ stats.ifRxMofOverruns
			+ stats.ifRxSofOverruns
			+ stats.ifInCRCErrors 
			+ stats.ifInAlignCodeErrors
			+ stats.ifInJabberFrames
			+ stats.ifInFragments
			+ stats.ifInOversizedFrames
			+ stats.ifInUndersizedFrames
			+ stats.ifInFilteredFrames
			+ stats.ifInQosFilteredFrames;     

	hDDC->Mib2IfHCCounter.Mib2IfCounter.inErrorPkts = stats.ifInCRCErrors 
			+ stats.ifInAlignCodeErrors
			+ stats.ifInJabberFrames
			+ stats.ifInFragments;
		
	hDDC->Mib2IfHCCounter.Mib2IfCounter.inUnknownProtPkts = 0; 
	
	hDDC->Mib2IfHCCounter.Mib2IfCounter.outBytesLow = 
			(unsigned long)hDDC->Mib2IfHCCounter.outBytesHC;
	hDDC->Mib2IfHCCounter.Mib2IfCounter.outBytesHigh = 
			hDDC->Mib2IfHCCounter.outBytesHC >> 32;
	
	
	hDDC->Mib2IfHCCounter.Mib2IfCounter.outUnicastPktsLow = 
			(unsigned long)hDDC->Mib2IfHCCounter.outUnicastPktsHC;
	hDDC->Mib2IfHCCounter.Mib2IfCounter.outUnicastPktsHigh = 
			hDDC->Mib2IfHCCounter.outUnicastPktsHC >> 32;
	
	hDDC->Mib2IfHCCounter.Mib2IfCounter.outMulticastPktsLow = 
			(unsigned long)hDDC->Mib2IfHCCounter.outMulticastPktsHC;
	hDDC->Mib2IfHCCounter.Mib2IfCounter.outMulticastPktsHigh = 
			hDDC->Mib2IfHCCounter.outMulticastPktsHC >> 32;
		
	hDDC->Mib2IfHCCounter.Mib2IfCounter.outBroadcastPktsLow = 
			(unsigned long)hDDC->Mib2IfHCCounter.outBroadcastPktsHC;
	hDDC->Mib2IfHCCounter.Mib2IfCounter.outBroadcastPktsHigh = 
			hDDC->Mib2IfHCCounter.outBroadcastPktsHC >> 32;
	
	hDDC->Mib2IfHCCounter.Mib2IfCounter.outErrorPkts = (stats.ifExcessiveCollisionFrames
			+ stats.ifLateCollisions
			+ stats.ifCarrierSenseErrors);
				
	hDDC->Mib2IfHCCounter.Mib2IfCounter.outDiscardPkts = 
			stats.ifOutUnderrun+hDDC->Mib2IfHCCounter.Mib2IfCounter.outErrorPkts;    	   
	return;		
}
		
#define cpmac_min_val(a,b) ((a > b) ? b : a) 
	
int cpmacTick (CpmacDDCObj *hDDC, void* tickArgs)
{
	/* Verify proper device state */
	if (hDDC->ddcObj.state != DDC_OPENED) {
		return (CPMAC_ERR_DEV_NOT_OPEN);
	}
	
	if( !(hDDC->initCfg.phyMode & SNWAY_NOPHY) )  {
		int tickChange;
	
		tickChange = cpsw_halcommon_mii_mdio_tic(hDDC->PhyDev);
		if(tickChange == 1) {
			cpmacUpdatePhyStatus(hDDC);
			hDDC->ddaIf->ddaNetIf.ddaFuncTable.ddaControlCb(hDDC->ddcObj.hDDA, 
					CPMAC_DDA_IOCTL_STATUS_UPDATE, 
					(void*) &hDDC->status, NULL);
		}
		else if ((hDDC->initCfg.phyMode & SNWAY_AUTOMDIX) && 
				(tickChange & _MIIMDIO_MDIXFLIP)) {
			avalanche_set_mdix_on_chip(hDDC->initCfg.baseAddress, tickChange & 0x1);
		}
	}
	
	return(CPMAC_SUCCESS);
}
	
static void cpmacProcessHostError(CpmacDDCObj *hDDC)
{
	unsigned int channel = 0;
	unsigned int vector = 0;
	unsigned int status = 0;
	
	status = hDDC->regs->MacStatus;
	
	channel = CSL_FEXT(status,CPMAC_MACSTATUS_TXERRCH);
	hDDC->status.hwErrInfo = channel << 16; /* Tx error channel in MSB 16 bits */
	vector = CSL_FEXT(status,CPMAC_MACSTATUS_TXERRCODE);
	
	if (vector)  {
		hDDC->status.hwStatus = CPMAC_DDC_TX_HOST_ERROR;
	}
	
	channel = CSL_FEXT(status,CPMAC_MACSTATUS_RXERRCH);
	hDDC->status.hwErrInfo |= channel; /* Rx error channel in LSB 16 bits */
	vector = CSL_FEXT(status,CPMAC_MACSTATUS_RXERRCODE);
	
	if (vector)  {
		hDDC->status.hwStatus = CPMAC_DDC_RX_HOST_ERROR;
	}
	
	hDDC->ddaIf->ddaNetIf.ddaFuncTable.ddaControlCb(hDDC->ddcObj.hDDA, 
					CPMAC_DDA_IOCTL_STATUS_UPDATE, 
					(void*) &hDDC->status, NULL);
}

int cpmacPktProcess (CpmacDDCObj *hDDC, int *pktsPending, void* pktArgs)
{
	unsigned int channel = 0;
	unsigned int vector = 0;
	unsigned int handlePktsAndStatus = 0;
	int  pktsProcessed       = 0;
	
	vector = hDDC->regs->Mac_In_Vector;
	
	if (vector & CPMAC_MAC_IN_VECTOR_TX_INT_OR) {
		Bool isEOQ;
	
		channel = (vector & 0x7);
	
		handlePktsAndStatus = hDDC->txCppi[channel]->chInfo.serviceMax;
	
		if(pktArgs)
			handlePktsAndStatus = cpmac_min_val(((RxTxParams *)pktArgs)->txPkts, handlePktsAndStatus);
	
		pktsProcessed = cpmacTxBDProc(hDDC, channel, &handlePktsAndStatus, &isEOQ);
		if(pktArgs)
			((RxTxParams *)pktArgs)->retTxPkts = pktsProcessed;
	
		if(hDDC->txInterruptDisable == True) {  
			if( !handlePktsAndStatus && isEOQ)
				hDDC->regs->Tx_IntMask_Clear = (1 << channel); 
		}
	
		*pktsPending = handlePktsAndStatus; /* Status. */
	}
	
	if (vector & CPMAC_MAC_IN_VECTOR_RX_INT_OR) {
		channel = (vector >> 8) & 0x7;
		handlePktsAndStatus = hDDC->rxCppi[channel]->chInfo.serviceMax;
	
		if(pktArgs)
			handlePktsAndStatus = cpmac_min_val(((RxTxParams *)pktArgs)->rxPkts, handlePktsAndStatus);
	
		pktsProcessed = CpmacRxBDProc(hDDC, channel, &handlePktsAndStatus); 
	
		if(pktArgs)
			((RxTxParams *)pktArgs)->retRxPkts = pktsProcessed;

		*pktsPending |= handlePktsAndStatus; /* Status */
	}
	
	if (vector & CPMAC_MAC_IN_VECTOR_HOST_INT) {
		cpmacProcessHostError(hDDC);
	}
	
	return (CPMAC_SUCCESS);
	
}
	
void cpmacGetInterruptCause(CpmacDDCObj *hDDC,Bool *rxPending,Bool *txPending,Bool *errPending)
{
	unsigned int vector = 0;
	
	vector = hDDC->regs->Mac_In_Vector;
	
	if (vector & CPMAC_MAC_IN_VECTOR_TX_INT_OR) {
		*txPending = True;
	}
	else {
		*txPending = False;
	}
	
	if (vector & CPMAC_MAC_IN_VECTOR_RX_INT_OR) {
		*rxPending = True;
	}
	else {
		*rxPending = False;
	}
	
	if (vector & CPMAC_MAC_IN_VECTOR_HOST_INT) {
		*errPending = True;
	}
	else {
		*errPending = False;
	}
	
}
int cpmacTxPktCompletionProcess (CpmacDDCObj *hDDC, int *pktsPending,void* pktArgs)
{
	unsigned int channel = 0;
	unsigned int vector = 0;
	unsigned int handlePktsAndStatus = 0;
	
	vector = hDDC->regs->Mac_In_Vector;
	
	if (vector & CPMAC_MAC_IN_VECTOR_TX_INT_OR) {
		Bool isEOQ;
	
		channel = (vector & 0x7);
	
		handlePktsAndStatus   = hDDC->txCppi[channel]->chInfo.serviceMax;
	
		cpmacTxBDProc(hDDC, channel, &handlePktsAndStatus, &isEOQ);
		*pktsPending = handlePktsAndStatus;
	}
	
	if (vector & CPMAC_MAC_IN_VECTOR_HOST_INT) {
		cpmacProcessHostError(hDDC);
	}
	
	return (CPMAC_SUCCESS);
}
	
int cpmacRxPktProcess (CpmacDDCObj *hDDC, int *pktsPending, void* pktArgs)
{
	unsigned int channel = 0;
	unsigned int vector = 0;
	unsigned int status = 0;
	
	vector = hDDC->regs->Mac_In_Vector;
	
	if (vector & CPMAC_MAC_IN_VECTOR_RX_INT_OR) {
		channel = (vector >> 8) & 0x7;
		CpmacRxBDProc(hDDC, channel, &status);
		*pktsPending = status;
	}
	
	if (vector & CPMAC_MAC_IN_VECTOR_HOST_INT) {
		cpmacProcessHostError(hDDC);
	}
	
	return (CPMAC_SUCCESS);
}
	
int cpmacPktProcessEnd (CpmacDDCObj *hDDC, void* procArgs)
{
	hDDC->regs->Mac_EOI_Vector = 0;
	return (CPMAC_SUCCESS);    
}
	
	
int DDC_cpmacSend(CpmacDDCObj *hDDC, DDC_NetPktObj *pkt, int  channel, void* sendArgs)
{
	int      retVal = CPMAC_SUCCESS;
	CpmacTxBD       *currBD;
	CpmacTxCppiCh   *txCppi;
	DDC_NetBufObj   *bufList;
	unsigned int          flags;
		
	if (hDDC->ddcObj.state != DDC_OPENED)
		return (CPMAC_ERR_DEV_NOT_OPEN);
	
	if (channel > CPMAC_MAX_TX_CHANNELS)
		return (CPMAC_ERR_TX_CH_INVALID);
	
	if (hDDC->txIsOpen[channel] != True)
		return (CPMAC_ERR_TX_CH_NOT_OPEN);
	
	if (!hDDC->status.PhyLinked)
		return (CPMAC_ERR_TX_NO_LINK);
	
	txCppi = hDDC->txCppi[channel];
	bufList = pkt->bufList;     /* Get handle to the buffer array */
	
	if (pkt->pktLength < CPMAC_MIN_ETHERNET_PKT_SIZE) {
		bufList->length += (CPMAC_MIN_ETHERNET_PKT_SIZE - pkt->pktLength);
		pkt->pktLength = CPMAC_MIN_ETHERNET_PKT_SIZE;
	}
	
	local_irq_save(flags);  /* Start of Send Critical Section */
	
	currBD = txCppi->bdPoolHead;
	if (currBD == NULL) {
		retVal = CPMAC_ERR_TX_OUT_OF_BD;
		hDDC->txIntThreshold[channel] = 0;
		goto Exit_DDC_cpmacSend;
	}
	txCppi->bdPoolHead = currBD->next;
	
	currBD->bufToken = bufList->bufToken;
	currBD->buff  = PAL_CPMAC_VIRT_2_PHYS((int *)bufList->data);
	currBD->off_bLen = bufList->length;
	currBD->hNext = 0;
	currBD->next  = 0;
	currBD->mode  = (CPMAC_CPPI_SOP_BIT | CPMAC_CPPI_OWNERSHIP_BIT | CPMAC_CPPI_EOP_BIT | pkt->pktLength);
	
	if ((Bool)sendArgs == True) 
		currBD->mode |= CPMAC_CPPI_PASS_CRC_BIT;
	
	PAL_CPMAC_CACHE_WRITEBACK(currBD, CPMAC_BD_LENGTH_FOR_CACHE);
	
	if (txCppi->activeQueueHead == 0) {
		txCppi->activeQueueHead = currBD;
		txCppi->activeQueueTail = currBD;
		if (txCppi->queueActive != True) {
			hDDC->regs->Tx_HDP[channel] = PAL_CPMAC_VIRT_2_PHYS(currBD);
			txCppi->queueActive = True;
		}
	}
	else {
		register volatile CpmacTxBD *tailBD;
		register unsigned int frameStatus;
	
		tailBD = txCppi->activeQueueTail;
		tailBD->next = currBD;
		txCppi->activeQueueTail = currBD;
	
		tailBD = PAL_CPMAC_VIRT_NOCACHE(tailBD);
		tailBD->hNext = (int) PAL_CPMAC_VIRT_2_PHYS(currBD);
		frameStatus = tailBD->mode;
		if (frameStatus & CPMAC_CPPI_EOQ_BIT) {
			frameStatus &= ~(CPMAC_CPPI_EOQ_BIT);
			tailBD->mode = frameStatus;
			if(hDDC->txInterruptDisable == True) {
			        volatile unsigned int txCompletionPtr;
			      	txCompletionPtr = hDDC->regs->Tx_CP[channel];
				hDDC->regs->Tx_CP[channel] = txCompletionPtr;
			}
	
			hDDC->regs->Tx_HDP[channel] = PAL_CPMAC_VIRT_2_PHYS(currBD);
		}
		else {
			if(hDDC->txInterruptDisable == True) {
				/* Enable Channel interrupt */
				hDDC->regs->Tx_IntMask_Set = (1 << channel);
			}
		}
	}
	
Exit_DDC_cpmacSend:
	if(hDDC->txInterruptDisable == True) {
		if( --hDDC->txIntThreshold[channel] <=  0) {
			Bool isEOQ;
			unsigned int handlePktsAndStatus;
				
			handlePktsAndStatus   = hDDC->txCppi[channel]->chInfo.serviceMax;
	
			cpmacTxBDProc(hDDC, channel, &handlePktsAndStatus, &isEOQ);
			hDDC->txIntThreshold[channel] = hDDC->initCfg.macCfg.txIntThresholdValue;
		}
	}
	
	local_irq_restore(flags);     /* End of Send Critical Section */
	
	return (retVal);    
}
	
int cpmacTxBDProc(CpmacDDCObj *hDDC,  unsigned int channel, unsigned int *handlePktsAndStatus, Bool * isEOQ)
{
	CpmacTxBD *currBD;
	CpmacTxCppiCh *txCppi;
	unsigned int frameStatus;
	unsigned int pktsProcessed = 0;
	unsigned int flags;
	unsigned int pktsToProcess = *handlePktsAndStatus;
	
#ifdef CPMAC_MULTIPACKET_TX_COMPLETE_NOTIFY
	unsigned int txCompleteCnt = 0;
	unsigned int *txCompletePtr;
#endif
	
	*handlePktsAndStatus = 0; /* Status. */
	
	*isEOQ = True;
	
	if (hDDC->txIsOpen[channel] == False)
		return (CPMAC_ERR_TX_CH_NOT_OPEN);
	
	if (hDDC->txTeardownPending[channel] == True) {
		return (CPMAC_SUCCESS);     /* Dont handle any packet completions */
	}
	txCppi = hDDC->txCppi[channel];
#ifdef CPMAC_MULTIPACKET_TX_COMPLETE_NOTIFY
	txCompletePtr = &txCppi->txComplete[0];
#endif
	
	local_irq_save(flags);  /* Start of Send Critical Section */
	
	currBD = txCppi->activeQueueHead;
	if (currBD == 0) {
		local_irq_restore(flags);     /* End of Send Critical Section */
		hDDC->regs->Tx_CP[channel] = PAL_CPMAC_VIRT_2_PHYS(txCppi->lastHwBDProcessed); 
		return(CPMAC_SUCCESS);
	}
	
	PAL_CPMAC_CACHE_INVALIDATE(currBD, CPMAC_BD_LENGTH_FOR_CACHE);
	frameStatus = currBD->mode;
	
	while ( (currBD) &&
		((frameStatus & CPMAC_CPPI_OWNERSHIP_BIT) == 0) && 
		(pktsProcessed < pktsToProcess)) {
			hDDC->regs->Tx_CP[channel] = PAL_CPMAC_VIRT_2_PHYS(currBD); 
			txCppi->activeQueueHead = currBD->next;
	
			if (frameStatus & CPMAC_CPPI_EOQ_BIT) {
				if (currBD->next) {
					hDDC->regs->Tx_HDP[channel] = currBD->hNext;
					hDDC->regs->Tx_IntMask_Set = (1 << channel);
				}
				else {
					txCppi->queueActive = False;
				}
			}
	
			*txCompletePtr = (unsigned int) currBD->bufToken;
			++txCompletePtr;
			++txCompleteCnt;
			currBD->next = txCppi->bdPoolHead;
			txCppi->bdPoolHead = currBD;
			pktsProcessed++;
	
			txCppi->lastHwBDProcessed = currBD;
			currBD = txCppi->activeQueueHead;
			if (currBD) {
				PAL_CPMAC_CACHE_INVALIDATE(currBD, CPMAC_BD_LENGTH_FOR_CACHE);
				frameStatus = currBD->mode;
			}
	} /* End of while loop */
	
	if ( (currBD) && ((frameStatus & CPMAC_CPPI_OWNERSHIP_BIT) == 0) ) {
		*handlePktsAndStatus = 1;
	}
	
	if (currBD)  
		*isEOQ = False;
	
	hDDC->ddaIf->ddaNetIf.ddaNettxCompleteCb(hDDC->ddcObj.hDDA, &txCppi->txComplete[0], txCompleteCnt, (void*) channel);
	
	local_irq_restore(flags);     /* End of Send Critical Section */
	return (pktsProcessed);
}
	
void cpmacAddListToRxQueue(CpmacDDCObj *hDDC, CpmacRxCppiCh *rxCppi, CpmacRxBD *first, CpmacRxBD *last)
{
	last->hNext = 0;
	last->next = 0;
	
	PAL_CPMAC_CACHE_WRITEBACK(last, CPMAC_BD_LENGTH_FOR_CACHE);
	
	if (rxCppi->activeQueueHead == 0) {
		rxCppi->activeQueueHead = first;
		rxCppi->activeQueueTail = last;
		if (rxCppi->queueActive != False) {
			rxCppi->hDDC->regs->Rx_HDP[rxCppi->chInfo.chNum] = PAL_CPMAC_VIRT_2_PHYS(rxCppi->activeQueueHead);
			rxCppi->queueActive = True;
		}
	}
	else {
		CpmacRxBD   *tailBD;
		unsigned int      frameStatus;
		unsigned int      flags;
	
		local_irq_save(flags);  /* Start of Receive Critical Section */
		tailBD = rxCppi->activeQueueTail;
		rxCppi->activeQueueTail = last;
		tailBD->next = (void*) first;
		tailBD = PAL_CPMAC_VIRT_NOCACHE(tailBD);
		tailBD->hNext = PAL_CPMAC_VIRT_2_PHYS(first);
		frameStatus = tailBD->mode;
		if (frameStatus & CPMAC_CPPI_EOQ_BIT) {
			hDDC->regs->Rx_HDP[rxCppi->chInfo.chNum] = PAL_CPMAC_VIRT_2_PHYS(first);
			frameStatus &= ~(CPMAC_CPPI_EOQ_BIT);
			tailBD->mode = frameStatus;
		}
	local_irq_restore(flags);     /* End of Receive Critical Section */
	}
}
	
int CpmacRxBDProc(CpmacDDCObj *hDDC,  unsigned int channel, int *handlePktsAndStatus)
{
	CpmacRxCppiCh *rxCppi;
	CpmacRxBD *currBD, *lastBD = NULL, *firstBD;
	unsigned int frameStatus;
	char *newBuffer;
	DDC_NetDataToken newBufToken;
	DDC_NetBufObj *rxBufObj;
	unsigned int pktsProcessed;
	DDC_NetPktObj *currPkt;
	unsigned int recycle_pkt;
	unsigned int pktsToBeProcessed = *handlePktsAndStatus;
	unsigned int rxCompleteCnt = 0;
	
	if (hDDC->rxIsOpen[channel] == False) {
		*handlePktsAndStatus = 0;
		return (CPMAC_ERR_RX_CH_NOT_OPEN);
	}
	
	rxCppi = hDDC->rxCppi[channel];
	if (hDDC->rxTeardownPending[channel] == True) {
		*handlePktsAndStatus = 0;
		return (0);
	}
	
	*handlePktsAndStatus = 0;
	
	pktsProcessed = 0;
	currPkt = &rxCppi->pktQueue[0];
	
	firstBD = currBD = rxCppi->activeQueueHead;
	PAL_CPMAC_CACHE_INVALIDATE(currBD, CPMAC_BD_LENGTH_FOR_CACHE);
	frameStatus = currBD->mode;
	
	while ( ((frameStatus & CPMAC_CPPI_OWNERSHIP_BIT) == 0) && 
		 (currBD) &&
		(pktsProcessed < pktsToBeProcessed) ) {
			DDA_NetFuncTable* NetIf = &hDDC->ddaIf->ddaNetIf;
	
			recycle_pkt = 0;
	
			newBuffer = NetIf->ddaNetAllocRxBufCb(hDDC->ddcObj.hDDA, rxCppi->chInfo.bufSize, &newBufToken, NULL);
			if (newBuffer == NULL) {
				    newBuffer   = currBD->data;
				    newBufToken = currBD->bufToken;   
	
				    recycle_pkt = 1;
			}
			else {
				rxBufObj = &currPkt->bufList[0];
				rxBufObj->data  = (char *)currBD->data;
				rxBufObj->length   = currBD->off_bLen & CPMAC_RX_BD_BUF_SIZE;
				rxBufObj->bufToken = currBD->bufToken;
				currPkt->pktToken = currPkt->bufList->bufToken;
				currPkt->numBufs = 1;
				currPkt->pktLength = (frameStatus & CPMAC_RX_BD_PKT_LENGTH_MASK);
			}
	
			hDDC->regs->Rx_CP[channel] = PAL_CPMAC_VIRT_2_PHYS(currBD); 
	
			lastBD = currBD;
			currBD = lastBD->next;
			rxCppi->activeQueueHead = currBD;
	
			if (frameStatus & CPMAC_CPPI_EOQ_BIT) {
				if (currBD)  {
					hDDC->regs->Rx_HDP[channel] = PAL_CPMAC_VIRT_2_PHYS(lastBD->hNext);
				}
				else  {
					rxCppi->queueActive = False;
				}
			}
	
			if(recycle_pkt)
			    goto Recycle_CpmacRxBDProc;
	
			++currPkt;
			++rxCompleteCnt;
		    
Recycle_CpmacRxBDProc:
	
			lastBD->buff = PAL_CPMAC_VIRT_2_PHYS(newBuffer);
			lastBD->off_bLen = rxCppi->chInfo.bufSize;
			lastBD->mode = CPMAC_CPPI_OWNERSHIP_BIT;
			lastBD->data = newBuffer;
			lastBD->bufToken = newBufToken;
			PAL_CPMAC_CACHE_WRITEBACK(lastBD, CPMAC_BD_LENGTH_FOR_CACHE);
	
			++pktsProcessed;
	
			if (currBD) {
				PAL_CPMAC_CACHE_INVALIDATE(currBD, CPMAC_BD_LENGTH_FOR_CACHE);
				frameStatus = currBD->mode;
			}
	} /* End of while receive packet processing loop */
	
	if ( (currBD) && ((frameStatus & CPMAC_CPPI_OWNERSHIP_BIT) == 0) ) {
		*handlePktsAndStatus = 1;
	}
	
	if (rxCompleteCnt > 0) 
		hDDC->ddaIf->ddaNetIf.ddaNetrxMultipleCb(hDDC->ddcObj.hDDA, &rxCppi->pktQueue[0], rxCompleteCnt, (void*) channel);
	
	if(pktsProcessed)
		cpmacAddListToRxQueue(hDDC, rxCppi, firstBD, lastBD);
	
	return (pktsProcessed); 
}
