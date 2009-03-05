#ifndef __DAVINCI_AUDIO_CONFIG_H_
#define __DAVINCI_AUDIO_CONFIG_H_

/*
 * davinci-audio-config.h - An interface to make the configuration of ASP and
 *                          codec easy.
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

#define CODEC_IS_MASTER                    0
#define CODEC_IS_SLAVE                     1

#define SERIALIZER_IS_TX                1
#define SERIALIZER_IS_RX                2
#define SERIALIZER_IS_INACTIVE          0

#define MAX_SERIALIZER_COUNT           16 /* Serializer Count */

typedef struct audio_config {
    u8 mcasp_id;
    u8 mode;
    u8 channel_size;
    u8 serializer_count;
    u8 serializer_mode[MAX_SERIALIZER_COUNT];
    u8 loopback;
    u8 amute;
    s16 tdm_slots;
    u32 sample_rate;
}audio_config_t;

#endif /* __DAVINCI_AUDIO_CONFIG_H_ */
