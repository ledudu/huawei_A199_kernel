/* add cmi lcd driver */
/* Copyright (c) 2009, Code HUAWEI. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "k3_fb.h"
#include "k3_fb_def.h"
#include "mipi_dsi.h"
#include "mipi_reg.h"
#include <linux/hw_lcd_common.h>
#include <mach/gpio.h>
#include <hsad/config_mgr.h>

#define MIPI_MAX_BUFFER 128
static char mipi_packet_struct[MIPI_MAX_BUFFER];
/* use global varible */
char panel_name[MAX_LCD_PANEL_NAME_LEN];


/*****************************************
  @brief : transfor struct sequence to struct mipi packet,  
  @param reg:register and param, value: reg type.
  @return none
******************************************/
void mipi_lcd_register_write(unsigned int reg,unsigned int value, struct k3_fb_data_type *k3fd)
{
	static bool packet_ok = false; 
	static unsigned int param_num = 0;
	static unsigned int last_datatype = 0;
	unsigned int datatype = 0;
	
	struct dsi_cmd_desc dsi_cmd;
	
	if (( (MIPI_DCS_COMMAND == last_datatype) || (MIPI_GEN_COMMAND == last_datatype) )
		&&( TYPE_PARAMETER != value ))
	{
		packet_ok = true;
	}
	else
	{
		packet_ok = false;
	}
	
	if(packet_ok)
	{
		switch (param_num)
   		{
    		case 1:
				if (MIPI_DCS_COMMAND == last_datatype)
				{
					/*DCS MODE*/					
					datatype = DTYPE_DCS_WRITE;
				}
				else if (MIPI_GEN_COMMAND == last_datatype)
				{
					/*GCS MODE*/					
					datatype = DTYPE_GEN_WRITE1;					
				}								
				
				break;
			case 2:		
				if (MIPI_DCS_COMMAND == last_datatype)
				{
					/*DCS MODE*/
					datatype = DTYPE_DCS_WRITE1;
				}
				else if (MIPI_GEN_COMMAND == last_datatype)
				{
					/*GCS MODE*/					
					datatype = DTYPE_GEN_WRITE2;					
				}

				break;
			default:	
				if (MIPI_DCS_COMMAND == last_datatype)
				{
					/*DCS MODE*/
					datatype = DTYPE_DCS_LWRITE;
				}
				else if (MIPI_GEN_COMMAND == last_datatype)
				{
					/*GCS MODE*/					
					datatype = DTYPE_GEN_LWRITE;					
				}
				
				break;
		}
	
		dsi_cmd.dtype = datatype;
		dsi_cmd.vc = 0;
		dsi_cmd.wait = 0;
		dsi_cmd.dlen = param_num;
		dsi_cmd.payload = mipi_packet_struct;
		
		mipi_dsi_cmds_tx(&dsi_cmd,1,k3fd->edc_base);
		packet_ok = false;
		param_num = 0;
		last_datatype = 0;
	}  
	
    switch (value)
    {
    	case MIPI_DCS_COMMAND:
		case MIPI_GEN_COMMAND:				
			last_datatype = value;	
			mipi_packet_struct[param_num] = reg;
			param_num++;
			break;
		case TYPE_PARAMETER:
			mipi_packet_struct[param_num] = reg;
			param_num++;
			break;
		case MIPI_TYPE_END:
			packet_ok = false;
			param_num = 0;
			last_datatype = 0;
			break;
		default :
			break;

    }
	
}

/*****************************************
  @brief   process mipi sequence table
  @param table: lcd init code, count: sizeof(table), lcd_panel: lcd type
			    mfd:mipi need ,tp: process mipi buffer.
  @return none
******************************************/
void process_mipi_table(struct sequence *table, unsigned int count, struct k3_fb_data_type *k3fd)
{
	unsigned int i = 0;
	unsigned int reg = 0;
	unsigned int value = 0;
	unsigned int time = 0;
	for (i = 0; i < count; i++)
	{
	    reg = table[i].reg;
        value = table[i].value;
        time = table[i].time;		
		mipi_lcd_register_write(reg,value,k3fd);
		if ((time > 0)&&(time <= MAX_UDELAY_MS*1000))
        {
            udelay(time);
        }
		else if( time > MAX_UDELAY_MS*1000)
		{
			mdelay(time/1000);
		}
	}
			
}
/****************************************************************
function: get lcd id by gpio

*data structure*
*	   ID1	  ID0   * 
 *	----------------- * 
 *	|   |   |   |   | * 
 *	|   |   |   |   | * 
 *	----------------- *
 For each Gpio :
		00 means low  ,
		01 means high ,
		10 means float,
		11 is not defined,

 lcd id(hex):
 0	:ID0 low,	ID1 low
 1	:ID0 high,	ID1 low
 2	:ID0 float,	ID1 low

 4	:ID0 low,	ID1 high
 5	:ID0 high,	ID1 high
 6	:ID0 float,	ID1 high

 8	:ID0 low,	ID1 float
 9	:ID0 high,	ID1 float
 A	:ID0 float,	ID1 float, used for emulator
 ***************************************************************/
/* optimize this part code */
int hw_get_lcd_id(void)
{
	int lcd_id=0;
	int gpio_id0,gpio_id1;
	bool pullup_read,pulldown_read;
	struct iomux_pin *gpio_lcd_id0;
	struct iomux_pin *gpio_lcd_id1;
	pullup_read = 0;
	pulldown_read = 0;
	gpio_id0 = GPIO_16_1;
	gpio_id1 = GPIO_16_2;
	
	gpio_request(gpio_id0, "gpio_lcd_id0");
	gpio_request(gpio_id1, "gpio_lcd_id1");
	gpio_lcd_id0=iomux_get_pin("gpio_129");
	gpio_lcd_id1=iomux_get_pin("gpio_130");
		
	/*config lcd_id to pull down and read*/
	pinmux_setpullupdown(gpio_lcd_id0, PULLDOWN);
	gpio_direction_input(gpio_id0);
	udelay(10);//necessary for a delay, else following gpio_get_value always get 0
	pulldown_read = gpio_get_value(gpio_id0);
	
	/*config lcd_id to pull up and read*/
	pinmux_setpullupdown(gpio_lcd_id0, PULLUP);
	gpio_direction_input(gpio_id0);
	udelay(10);
	pullup_read = gpio_get_value(gpio_id0);
	
	if(pulldown_read != pullup_read)//float
	{
		pinmux_setpullupdown(gpio_lcd_id0, NOPULL);
		lcd_id = lcd_id | LCD_HW_ID_STATUS_FLOAT;

	}
	else if((pulldown_read == LCD_HW_ID_STATUS_LOW)&&(pullup_read == LCD_HW_ID_STATUS_LOW))
    {
		pinmux_setpullupdown(gpio_lcd_id0, PULLDOWN);		
        /*low status*/
        lcd_id = lcd_id | LCD_HW_ID_STATUS_LOW;
    }
    else if((pulldown_read == LCD_HW_ID_STATUS_HIGH)&&(pullup_read == LCD_HW_ID_STATUS_HIGH))
    {
		pinmux_setpullupdown(gpio_lcd_id0, PULLUP);
        /*high status*/
        lcd_id = lcd_id | LCD_HW_ID_STATUS_HIGH;
    }
	
    /* get id0 status */
	/*config lcd_id to pull down and read*/
	pinmux_setpullupdown(gpio_lcd_id1, PULLDOWN);
	gpio_direction_input(gpio_id1);
	udelay(10);//necessary for a delay, else following gpio_get_value always get 0
	pulldown_read = gpio_get_value(gpio_id1);
	
	/*config lcd_id to pull up and read*/
	pinmux_setpullupdown(gpio_lcd_id1, PULLUP);
	gpio_direction_input(gpio_id1);
	udelay(10);
	pullup_read = gpio_get_value(gpio_id1);

	if(pulldown_read != pullup_read)//float
	{
		pinmux_setpullupdown(gpio_lcd_id1, NOPULL);
		lcd_id = lcd_id | (LCD_HW_ID_STATUS_FLOAT<<2);
	}
	else if((pulldown_read == LCD_HW_ID_STATUS_LOW)&&(pullup_read == LCD_HW_ID_STATUS_LOW))
    {
		pinmux_setpullupdown(gpio_lcd_id1, PULLDOWN);
        /*low status*/
        lcd_id = lcd_id | (LCD_HW_ID_STATUS_LOW<<2);
    }
    else if((pulldown_read == LCD_HW_ID_STATUS_HIGH)&&(pullup_read == LCD_HW_ID_STATUS_HIGH))
    {
		pinmux_setpullupdown(gpio_lcd_id1, PULLUP);
        /*high status*/
        lcd_id = lcd_id | (LCD_HW_ID_STATUS_HIGH<<2);
    }
	gpio_free(gpio_id0);
	gpio_free(gpio_id1);
	k3fb_loge(" lcd_id=%d\n",lcd_id);
	return lcd_id;
}
int get_lcd_name(char *pname,int length)
{
    char *psKey = NULL;
    int id = 0;
    int ret = 0;
	psKey = kmalloc(48, GFP_KERNEL);
    if (NULL == psKey)  
    {
		ret = false;
		return ret;
    }
    memset(psKey, 0, 48);
    id = hw_get_lcd_id();
    sprintf(psKey, "lcd/id%d", id);
    ret = get_hw_config_string(psKey, pname, length, NULL);
    kfree(psKey);
	if(false == ret)
	{
		printk(KERN_ERR "get LCD name fail!\n");
	}
	return ret;
}
int lcd_detect_panel(const char *pstring)
{
    int ret;
    static bool detected = false;
	/* remove one line */
	
    if(detected == true)
    {
        return -EBUSY;
    }
	ret = get_lcd_name(panel_name, MAX_LCD_PANEL_NAME_LEN);
    if(ret == false)
    {
		strncpy(panel_name,DEFAULT_PANEL_NAME,MAX_LCD_PANEL_NAME_LEN);
    }
    ret = strncmp(pstring,panel_name,MAX_LCD_PANEL_NAME_LEN);
    if(ret)
    {
        ret = -ENODEV;
        goto err_detect_panel;
    }

    detected = true;

err_detect_panel:
    return ret;
}


