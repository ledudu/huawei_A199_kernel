/* add cmi lcd driver */
/* Copyright (c), Code HUAWEI. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "lcd_hw_debug.h"
#ifndef HW_LCD_COMMON_H
#define HW_LCD_COMMON_H

/* Move from the every LCD file ,those are common */
#define TRACE_LCD_DEBUG 1
#if TRACE_LCD_DEBUG
#define LCD_DEBUG(x...) printk(KERN_ERR "[LCD_DEBUG] " x)
#else
#define LCD_DEBUG(x...) do {} while (0)
#endif
/* LCD_MDELAY will select mdelay or msleep according value */
#define LCD_MDELAY(time_ms)   	\
	do							\
	{ 							\
		if (time_ms>10)			\
			msleep(time_ms);	\
		else					\
			mdelay(time_ms);	\
	}while(0)	

#define LCD_HW_ID_STATUS_LOW     0
#define LCD_HW_ID_STATUS_HIGH    1
#define LCD_HW_ID_STATUS_FLOAT   2

#define MAX_LCD_PANEL_NAME_LEN  40
#define MIPI_MAX_BUFFER 128

/* MIPI_DCS_COMMAND : == TYPE_COMMAND MIPI DCS COMMAND
 * MIPI_GEN_COMMAND : 4 MIPI GCS COMMAND
 * MIPI_TYPE_END: 0XFF 
 */
#define MIPI_DCS_COMMAND (1<<0)
#define TYPE_PARAMETER   (1<<1)
#define MIPI_GEN_COMMAND (1<<2)
#define MIPI_TYPE_END    0XFF
#define ARRY_SIZE(A) (sizeof(A)/sizeof(A[0]))

#define MIPI_CMD_OTM1282A_HD  "mipi_cmd_otm1282a_hd"

/* Add auo name */
#define DEFAULT_PANEL_NAME  "mipi_cmd_otm1282a_cmi_hd"
#define MIPI_CMD_OTM1282A_CMI_HD  "mipi_cmd_otm1282a_cmi_hd"
#define MIPI_CMD_OTM1282A_AUO_HD  "mipi_cmd_otm1282a_auo_hd"
	/* move struct sequence to lcd_hw_debug.h file */


void process_mipi_table(struct sequence *table, unsigned int count, struct k3_fb_data_type *k3fd);
int lcd_detect_panel(const char *pstring);
/* assert varible */
extern char panel_name[MAX_LCD_PANEL_NAME_LEN];
#endif

