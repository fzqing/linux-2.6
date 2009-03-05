/*
 * evm_audio_info.c - EVM specific layout
 *
 *
 * Copyright (C) 2007  Texas Instruments, India
 * Author:Nirmal Pandey <n-pandey@ti.com>,
 *        Suresh Rajashekara <suresh.r@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */


#include <asm/arch/evm_audio_info.h>
#include <asm/arch/mcasp.h>

extern struct asp_file_operations mcasp_fops;
extern codec_fops_t aic32_fops;

#define MCASP0 			0
#define MCASP0_REG_BASE		0x01D01000
#define MCASP0_TX_DMA_PORT	0x01D01400
#define MCASP0_RX_DMA_PORT	0x01D01400
#define MCASP0_TX_DMA_EVT	6
#define MCASP0_RX_DMA_EVT	9
#define MCASP0_TX_IRQ		28
#define MCASP0_RX_IRQ		29
#define MCASP0_SERIALIZER_COUNT	4

#define MCASP1			1
#define MCASP1_REG_BASE		0x01D01800
#define MCASP1_TX_DMA_PORT	0x01D01C00
#define MCASP1_TX_DMA_EVT	12
#define MCASP1_TX_IRQ		32
#define MCASP1_SERIALIZER_COUNT	1


asp_info_t mcasp_info[] = {
	[0] = {
		.id = MCASP0,
		.reg_base = IO_ADDRESS(MCASP0_REG_BASE),
		.tx_data_port = MCASP0_TX_DMA_PORT,
		.rx_data_port = MCASP0_RX_DMA_PORT,
		.tx_dma_evt = MCASP0_TX_DMA_EVT,
		.rx_dma_evt = MCASP0_RX_DMA_EVT,
		.tx_irq = MCASP0_TX_IRQ,
		.rx_irq = MCASP0_RX_IRQ,
		.serializer_count = MCASP0_SERIALIZER_COUNT,
		.left_dit_channel_count = 0,
		.right_dit_channel_count = 0,
		.lpsc = "McASPCLK0",
		.flags = TX_SUPPORT | RX_SUPPORT,
	},
	[1] = {
		.id = MCASP1,
		.reg_base = IO_ADDRESS(MCASP1_REG_BASE),
		.tx_data_port = MCASP1_TX_DMA_PORT,
		.rx_data_port = -1,
		.tx_dma_evt = MCASP1_TX_DMA_EVT,
		.rx_dma_evt = -1,
		.tx_irq = MCASP1_TX_IRQ,
		.rx_irq = -1,
		.serializer_count = MCASP1_SERIALIZER_COUNT,
		.left_dit_channel_count = 0,
		.right_dit_channel_count = 0,
		.lpsc = "McASPCLK1",
		.flags = TX_SUPPORT,
	},
};


struct codec_info aic32_info = {
	.lpsc = NULL,
	.flags = 0,
};

evm_audio_info_t evm_aud_info = {
	.asp_count = 2,
	.asp[0] = {
		.id = 0,
		.type = MCASP,
		/* For Davinci-HD if ASP needs to be made as master, then
		   need_tx_for_rx has to be 1, because of the issue we have with
		   the clock interference of ACLKX and ACLKR. This is not
		   necessary for S/PDIF*/
		.master_mode = ASP_SLAVE,
		.is_configured = FALSE,
		.is_initialized = FALSE,
		.is_in_use = FALSE,
		.a_op = &mcasp_fops,
		.a_info = (struct asp_info *)&mcasp_info,
		.dit_enabled = 0,
		.need_tx_for_rx = 0,
		.codec_count = 1,
		.codec[0] = {
			.id = 0,
			.type = CODEC_AIC32,
			.is_configured = FALSE,
			.is_initialized = FALSE,
			.is_in_use = FALSE,
			.c_op = &aic32_fops,
			.c_info = &aic32_info,
			.device_instance = 0,
			.pid = 0,
		}
	},
	.asp[1] = {
		.id = 1,
		.type = MCASP,
		.master_mode = ASP_MASTER,
		.is_configured = FALSE,
		.is_initialized = FALSE,
		.is_in_use = FALSE,
		.a_op = &mcasp_fops,
		.a_info = (struct asp_info *)&mcasp_info,
		.dit_enabled = 1,
		.need_tx_for_rx = 0,
		.codec_count = 0,
		.codec[0] = {
			.id = 1,
			.type = CODEC_UNKNOWN,
			.is_configured = FALSE,
			.is_initialized = FALSE,
			.is_in_use = FALSE,
			.c_op = NULL,
			.c_info = NULL,
			.device_instance = 0,
			.pid = 1,
		}
	},
};

s8 find_audio_device ( u8 dev_instance, u8 *asp_id_ptr, u8 *codec_id_ptr )
{
	s8 ret_code = 0;
	u8 asp_count, count, ccount, codec_count;
	u8 found = FALSE;
	u8 asp_id;

	/* Find the ASP and codec for the given device number */
	if (evm_aud_info.asp_count == 0) {
		ret_code = -ERR_NULL_AUDIO_CONFIG;
	} else {
		asp_count = evm_aud_info.asp_count;
		/* Search for each ASP */
		for (count = 0; count < asp_count; count++) {
			asp_id = evm_aud_info.asp[count].id;
			codec_count = evm_aud_info.asp[count].codec_count;
			/* NK: make sure that if codec_count is zero, dit is
			   enabled */
			if ( (codec_count == 0) &&
			     (evm_aud_info.asp[count].dit_enabled != 1) ) {
				/* This McASP is not having a codec or DIT */
				/* There might be some problem with the config
				 */
				/* Lets go ahed for the time being */
				continue; /* No codec connected with this ASP */
			} else {
				/* search each codec within given ASP */
				/* NK: Lets make sure that if codec_count is
				   zero, dit is enabled, make codec count as 1
				*/
				if (evm_aud_info.asp[count].dit_enabled == 1) {
					/* making codec count as one. This will
					   enable us to search for the given ASP
					*/
					codec_count = 1;
				}
				for ( ccount = 0; ccount < codec_count; ccount++
					) {
					if ( dev_instance ==
					     evm_aud_info.asp[count].
					     codec[ccount].device_instance ) {
						/* codec with desired device
						   number has been found */
						*asp_id_ptr = count;
						*codec_id_ptr = ccount;
						found = TRUE;
						break;
					}
				}

				if ( found ) {
					break;
				}
			}
		}
	}

	if (!found) {
		ret_code = -ERR_AUDIO_DEVICE_NOT_FOUND;
		printk(KERN_INFO"McASP with minor number %d not found"
		       , dev_instance);
	}

	return(ret_code);
}
EXPORT_SYMBOL(find_audio_device);

u8 is_dit_enabled (u8 device_instance)
{
	u8 asp_id = 0, codec_id = 0;

	find_audio_device (device_instance, &asp_id, &codec_id);
	return evm_aud_info.asp[asp_id].dit_enabled;
}

unsigned long get_tx_dma_dest_address (u8 device_instance)
{
	u8 asp_id = 0, codec_id = 0;

	find_audio_device (device_instance, &asp_id, &codec_id);
	return evm_aud_info.asp[asp_id].a_info[asp_id].tx_data_port;
}
EXPORT_SYMBOL(get_tx_dma_dest_address);

unsigned long get_rx_dma_src_address (u8 device_instance)
{
	u8 asp_id = 0, codec_id = 0;

	find_audio_device (device_instance, &asp_id, &codec_id);
	return evm_aud_info.asp[asp_id].a_info[asp_id].rx_data_port;
}
EXPORT_SYMBOL(get_rx_dma_src_address);

asp_fops_t *get_asp_fops(u8 device_instance)
{
	u8 asp_id = 0, codec_id = 0;

	find_audio_device (device_instance, &asp_id, &codec_id);
	return evm_aud_info.asp[asp_id].a_op;
}
EXPORT_SYMBOL(get_asp_fops);

codec_fops_t *get_codec_fops(u8 device_instance)
{
	u8 asp_id = 0, codec_id = 0;

	find_audio_device (device_instance, &asp_id, &codec_id);
	return evm_aud_info.asp[asp_id].codec[codec_id].c_op;
}
EXPORT_SYMBOL(get_codec_fops);

static s8 validate_evm_codec_config (audio_codec_t *codec)
{
	s8 retcode = 0;

	if ( codec->id > MAX_CODEC ) {
		retcode = -ERR_INVALID_CODEC_ID;
	} else {
		codec->is_configured = FALSE;
		codec->is_initialized = FALSE;
		codec->is_in_use = FALSE;
		codec->device_instance = 0;

		if ( codec->c_info == NULL && codec->c_op != NULL ) {
			retcode = -ERR_INVALID_CODEC_FOPS_OR_INFO;
		}
	}

	return retcode;
}


static s8 validate_evm_asp_config (audio_serial_port_t *asp)
{
	s8 retcode = 0;
	u8 cnt = 0;
	u8 *codec_id_arr = NULL;

	codec_id_arr = (u8 *) kmalloc (asp->codec_count, GFP_KERNEL);

	if (codec_id_arr == NULL) {
		retcode = -ENOMEM;
	} else {
		memset(codec_id_arr, 0, asp->codec_count);

		if ( asp->id > MAX_ASP ) {
			retcode = -ERR_INVALID_ASP_ID;
		} else {
			asp->is_configured = FALSE;
			asp->is_initialized = FALSE;
			asp->is_in_use = FALSE;

			if ( asp->a_info == NULL && asp->a_op != NULL ) {
				retcode = -ERR_INVALID_ASP_FOPS_OR_INFO;
			} else {

				if ( asp->dit_enabled == 1 && asp->codec_count
				     != 0 ) {
					retcode = -ERR_DIT_ENABLED_WITH_CODECS;
				} else {
					for (cnt = 0; cnt < asp->codec_count;
					     cnt++) {

						retcode =
						validate_evm_codec_config
							(&asp->codec[cnt]);
						if (retcode < 0 ) {
							break;
						}

						if (
						codec_id_arr[asp->codec[cnt].id]
						== TRUE ) {
							retcode =
							-ERR_DUP_CODEC_ID;
							break;
						} else {
							codec_id_arr[
								asp->codec[
									cnt].id]
								= TRUE;
						}

						asp->codec[cnt].pid = asp->id;
					}
				}
			}
		}

		kfree(codec_id_arr);
	}

	return retcode;
}

s8 validate_evm_audio_config (evm_audio_info_t **evm_info)
{
	s8 retcode = 0;
	u8 cnt = 0;
	u8 asp_id_arr[MAX_ASP] = {0};
	evm_audio_info_t *info;

	info = &evm_aud_info;

	if (unlikely(info->asp_count > MAX_ASP)) {
		info = NULL;
		retcode = -ERR_INVAL_ASP_CNT;
	} else {
		for (cnt = 0; cnt < info->asp_count; cnt++) {
			retcode = validate_evm_asp_config (&info->asp[cnt]);
			if (retcode < 0 ) {
				info = NULL;
				break;
			}

			if (asp_id_arr[info->asp[cnt].id] == TRUE) {
				info = NULL;
				retcode = -ERR_DUP_ASP_ID;
				break;
			} else {
				asp_id_arr[info->asp[cnt].id] = TRUE;
			}
		}
	}

	*evm_info = info;
	return retcode;
}
EXPORT_SYMBOL(validate_evm_audio_config);
