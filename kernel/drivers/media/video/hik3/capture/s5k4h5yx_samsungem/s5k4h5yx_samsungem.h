/*
 *  S5K4H5YX camera driver head file
 *
 *  CopyRight (C) Hisilicon Co., Ltd.
 *	Author :
 *  Version:  1.2
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _S5K4H5YX_H
#define _S5K4H5YX_H

#include "../isp/k3_isp_io.h"

#define S5K4H5YXREG_SOFTWARE_RESET	0x0103

#if 0
/* High 4 bits are clock, low 4 bits are reset */
#define S5K4H5YXREG_CLKRST0		0x3016
#define S5K4H5YXREG_CLKRST1		0x3017
#define S5K4H5YXREG_CLKRST2		0x3018


#define S5K4H5YXREG_PLL1_MULTIPLIER	0x0307
#define S5K4H5YXREG_PLL1_PREDIV		0x0305
#define S5K4H5YXREG_PLL1_OP_PIX_DIV	0x30b5
#define S5K4H5YXREG_PLL1_OS_SYS_DIV	0x30b6

#define S5K4H5YXREG_PLL2_PREDIV		0x3090
#define S5K4H5YXREG_PLL2_MULTIPLIER	0x3091
#define S5K4H5YXREG_PLL2_DIVS			0x3092
#define S5K4H5YXREG_PLL2_SELD5		0x3093

#define S5K4H5YXREG_PLL3_PREDIV		0x3098
#define S5K4H5YXREG_PLL3_MULT1		0x309c
#define S5K4H5YXREG_PLL3_MULT2		0x3099
#define S5K4H5YXREG_PLL3_DIVS		0x309a
#define S5K4H5YXREG_PLL3_DIV		0x309b

#define S5K4H5YXREG_WIDTH			0x3808
#define S5K4H5YXREG_HEIGHT		0x380a

//#define S5K4H5YXREG_HTS			0x380c
//#define S5K4H5YXREG_VTS			0x380e

#define S5K4H5YXREG_X_ADDR_START		0x3800
#define S5K4H5YXREG_Y_ADDR_START		0x3802
#define S5K4H5YXREG_X_ADDR_END		0x3804
#define S5K4H5YXREG_Y_ADDR_END		0x3806

#define S5K4H5YXREG_H_OFFSET		0x3810
#define S5K4H5YXREG_V_OFFSET		0x3812

/*
 * bit[7:4]: odd inc
 * bit[3:0]: even inc
 */
#define S5K4H5YXREG_X_INC			0x3814
#define S5K4H5YXREG_Y_INC			0x3815

#endif

/*
 * bit[6]: digital vertical flip
 * bit[1]: array vertical flip
 * bit[0]: vertical binning
 */
#define S5K4H5YXREG_TIMING_FORMAT1	0x3820

/*
 * bit[2:1]: 00-normal;11-horizontal mirror
 * bit[0]: horizontal binning
 */
#define S5K4H5YXREG_TIMING_FORMAT2	0x3821


/* exposure: three bytes */
#define S5K4H5YXREG_LONG_EXPOSURE	0x3500

/*
 * bit[5]: Gain delay option: 0-one frame latch; 1-delay one frame latch
 * bit[4]: choose delay option: 0-delay disable; 1-delay enable
 * bit[1]: AGC manual enable
 * bit[0]: AEC manual enable
 */
#define S5K4H5YXREG_AEC_MANUAL		0x3503

/* exposure: three bytes */
#define S5K4H5YXREG_SHORT_EXPOSURE	0x3506

#define S5K4H5YXREG_GAIN_CONVERT		0x3509

#define S5K4H5YXREG_AEC_GAIN		0x350a

/*
 * bit[7:4]: lane num
 * bit[0]: 1-mipi;0-dvp
 */
#define S5K4H5YXREG_MIPI_SC_CTRL0		0x3011

/*
 * bit[7:4]: disable each lane
 * bit[3]: phy mode; 1-mipi;0-dvp
 */
#define S5K4H5YXREG_MIPI_SC_CTRL2		0x3015

#define S5K4H5YXREG_GENERAL_COLORBAR	0x5e00
#define S5K4H5YXREG_GROUP_ACCESS	0x3208

#define MIPI_4LANE

#define ORIENTATION_PHONE

/* For Huawei OTP */
struct otp_struct {
	//module info
	u8 year;
	u8 month;
	u8 day;
	u8 user_id_1;
	u8 user_id_2;
	u8 user_id_3;
	u8 cam_code;
	u8 vendor_version;
	//awb
	u8 rg_h;
	u8 rg_l;
	u8 bg_h;
	u8 bg_l;
	u8 rg_coff;
	u8 bg_coff;
	//vcm
	u8 start_curr;
	u8 end_curr;
	//lenc
	u8 lenc[62];
};

typedef enum {
	S5K4H5YX_MODULE_SAMSUNG = 0,
	S5K4H5YX_MODULE_FOXCONN,
	S5K4H5YX_MODULE_MAX,
} flash_moudle_t ;

#define OTP_BUFFER_START_ADDRESS		0x3d00
#define RG_TYPICAL 0x400
#define BG_TYPICAL 0x400

/*OVISP register*/
#define REG_D65_LIMIT		0x6620d
#define REG_D65_SPLIT		0x6620f
#define REG_A_LIMIT			0x6620e
#define REG_A_SPLIT			0x66210
#define REG_CWF_X			0x66209
#define REG_CWF_Y			0x6620a
#define REG_CENTER_CT		0x1c1c8
#define REG_LEFT_CT			0x1c1cc
#define REG_RIGHT_CT		0x1c1d0
#define REG_K_Y2X			0x6620b
#define REG_K_X2Y			0x6620c
#define REG_CT_A			0x1ccd5
#define REG_CT_CWF			0x1ccd6
#define REG_CT_D			0x1ccd7
#define REG_CT_HIGH_LIGHT_1		0x1c5d4
#define REG_CT_HIGH_LIGHT_2		0x1c5d5
#define REG_CT_HIGH_LIGHT_D50_B	0x1c5d6
#define REG_CT_HIGH_LIGHT_D50_R	0x1c5d7

int s5k4h5yx_check_otp_group(u16 index);
void s5k4h5yx_read_otp_mi_wb_vcm(u16 index, struct otp_struct *potp);
void s5k4h5yx_update_otp_wb(u16 otp_index);
void s5k4h5yx_update_otp_lenc(u16 otp_index);
void s5k4h5yx_update_blc(void);


/***********************************************************************
 *
 * s5k4h5yx init sensor registers list
 *
 ***********************************************************************/
/* default is 1600*1200 9fps 2lane */
const struct _sensor_reg_t s5k4h5yx_init_regs[] = {
//Quarter size 
////////////////////////////////////////////////////
//  name:  S5K4H5YX EVT1 setfile
//  v0.00 Temporary version
//  Image size : Full size - 1632X1224, 30 fps
//  EXTCLK : 20 MHz
//  System PLL output : 560 MHz
//  Output PLL output : 700 Mbps
//  GCLK (ATOP) : 280 MHz
//  DCLK_4 (ATOP) : 70 MHz
//  Like 3H7 (3268x2452, v-blank 79 line), Embedded line off
////////////////////////////////////////////////////

//$MIPI[Width:1632,Height:1224,Format:Raw10,Lane:4,ErrorCheck:0,PolarityData:0,PolarityClock:0,Buffer:4]


// Streaming off -----------------------------------
{0x0100,0x00},		//	[0]	mode_select

// Image Orientation -------------------------------
{0x0101,0x00},		//	[1:0]	image_orientation ([0] mirror en, [1] flip en)

// Analog Gain -------------------------------------
{0x0204,0x00},		//	[15:8]	analogue_gain_code_global H
{0x0205,0x20},		//	[7:0]	analogue_gain_code_global L

// Exposure Time -----------------------------------
{0x0200,0x14},		//	[15:8]	fine_integration_time H
{0x0201,0x04},		//	[7:0]	fine_integration_time L
{0x0202,0x04},		//	[15:8]	coarse_integration_time H
{0x0203,0xE2},		//	[7:0]	coarse_integration_time L

// Frame Rate --------------------------------------
{0x0340,0x04},//09		//	[15:8]	frame_length_lines H
{0x0341,0xF2},//E3		//	[7:0]	frame_length_lines L
{0x0342,0x1D},//12//0F//0E		//	[15:8]	line_length_pck H
{0x0343,0x50},//DE//6F//68		//	[7:0]	line_length_pck L

// Image Size --------------------------------------
{0x0344,0x00},		//	[11:8]	x_addr_start H
{0x0345,0x08},		//	[7:0]	x_addr_start L
{0x0346,0x00},		//	[11:8]	y_addr_start H
{0x0347,0x00},		//	[7:0]	y_addr_start L
{0x0348,0x0C},		//	[11:8]	x_addr_end H
{0x0349,0xC7},		//	[7:0]	x_addr_end L
{0x034A,0x09},		//	[11:8]	y_addr_end H
{0x034B,0x9F},		//	[7:0]	y_addr_end L
{0x034C,0x06},//0C		//	[11:8]	x_output_size H
{0x034D,0x60},//C0		//	[7:0]	x_output_size L
{0x034E,0x04},//09		//	[11:8]	y_output_size H
{0x034F,0xD0},//90		//	[7:0]	y_output_size L

// Analog Binning ----------------------------------
{0x0390,0x01},		//	[7:0]	binning_mode ([0] binning enable)
{0x0391,0x22},		//	[7:0]	binning_type (22h : 2x2 binning, 44h : 4x4 binning)

// Sub-Sampling Ratio ------------------------------
{0x0381,0x01},		//	[4:0]	x_even_inc
{0x0383,0x03},		//	[4:0]	x_odd_inc
{0x0385,0x01},		//	[4:0]	y_even_inc
{0x0387,0x03},		//	[4:0]	y_odd_inc

// PLL control -------------------------------------
{0x0301,0x02},		//	[3:0]	vt_pix_clk_div
{0x0303,0x01},		//	[3:0]	vt_sys_clk_div
{0x0305,0x05},		//	[5:0]	pre_pll_clk_div
{0x0306,0x00},		//	[9:8]	pll_multiplier H
{0x0307,0x8C},		//	[7:0]	pll_multiplier L
{0x0309,0x02},		//	[3:0]	op_pix_clk_div
{0x030B,0x01},		//	[3:0]	op_sys_clk_div
{0x3C59,0x00},		//	[2:0]	reg_PLL_S
{0x030D,0x05},		//	[5:0]	out_pre_pll_clk_div
{0x030E,0x00},		//	[9:8]	out_pll_multiplier H 
{0x030F,0xAF},//A5		//	[7:0]	out_pll_multiplier L
{0x3C5A,0x00},		//	[2:0]	reg_out_PLL_S
{0x0310,0x01},		//	[0]	pll_mode (01h : 2-PLL, 00h : 1-PLL)
{0x3C50,0x53},		//	[7:4]	reg_DIV_DBR
		//	[3:0]	reg_DIV_G
{0x3C62,0x02},		//	[31:24]	requested_link_bit_rate_mbps HH
{0x3C63,0xBC},//94		//	[23:16]	requested_link_bit_rate_mbps HL
{0x3C64,0x00},		//	[15:8]	requested_link_bit_rate_mbps LH
{0x3C65,0x00},		//	[7:0]	requested_link_bit_rate_mbps LL

// Embedded line on/off ----------------------------
{0x3C1E,0x00},		//	[3]	reg_isp_fe_TN_SMIA_sync_sel
		//	[2]	reg_pat_TN_SMIA_sync_sel
		//	[1]	reg_bpc_TN_SMIA_sync_sel
		//	[0]	reg_outif_TN_SMIA_sync_sel




	
{0x302A,0x0A},	
{0x303D,0x06},	
{0x304B,0x2A},	
	
	
{0x3205,0x84},	
{0x3207,0x85},	
{0x3214,0x94},	
{0x3216,0x95},	
{0x303a,0x9f},	
{0x3201,0x07},	
{0x3051,0xff},	
{0x3052,0xff},	
{0x3054,0xF0},	
{0x3037,0x12}, 
{0x305C,0x8F},	
{0x302D,0x7F}, 


{0x3915,0x00},	
{0x3916,0x4E},	
{0x3917,0x21},	
{0x393C,0x0F},	
{0x393D,0xA1},	
{0x3C12,0x00},	
{0x3C13,0x55},	
{0x3C14,0xF1},	
{0x3C15,0x00},	
{0x3C16,0x4E},	
{0x3C17,0x21},	

// Analog Tuning End


// Streaming On ------------------------------------
{0x0100,0x01},		//	[0]	mode_select
};

const struct _sensor_reg_t s5k4h5yx_framesize_1632x1224[] = {
//Quarter size 
////////////////////////////////////////////////////
//  name:  S5K4H5YX EVT1 setfile
//  v0.00 Temporary version
//  Image size : Full size - 1632X1224, 30 fps
//  EXTCLK : 20 MHz
//  System PLL output : 560 MHz
//  Output PLL output : 700 Mbps
//  GCLK (ATOP) : 280 MHz
//  DCLK_4 (ATOP) : 70 MHz
//  Like 3H7 (3268x2452, v-blank 79 line), Embedded line off
////////////////////////////////////////////////////




// Streaming off -----------------------------------
{0x0100,0x00},		//	[0]	mode_select

// Image Orientation -------------------------------
{0x0101,0x00},		//	[1:0]	image_orientation ([0] mirror en, [1] flip en)

// Analog Gain -------------------------------------
{0x0204,0x00},		//	[15:8]	analogue_gain_code_global H
{0x0205,0xA0},		//	[7:0]	analogue_gain_code_global L

// Exposure Time -----------------------------------
{0x0200,0x14},		//	[15:8]	fine_integration_time H
{0x0201,0x04},		//	[7:0]	fine_integration_time L
{0x0202,0x05},		//	[15:8]	coarse_integration_time H
{0x0203,0xF2},		//	[7:0]	coarse_integration_time L

// Frame Rate --------------------------------------
{0x0340,0x04},//09		//	[15:8]	frame_length_lines H
{0x0341,0xF2},//E3		//	[7:0]	frame_length_lines L
{0x0342,0x1D},//12//0F//0E		//	[15:8]	line_length_pck H
{0x0343,0x50},//DE//6F//68		//	[7:0]	line_length_pck L

// Image Size --------------------------------------
{0x0344,0x00},		//	[11:8]	x_addr_start H
{0x0345,0x08},		//	[7:0]	x_addr_start L
{0x0346,0x00},		//	[11:8]	y_addr_start H
{0x0347,0x00},		//	[7:0]	y_addr_start L
{0x0348,0x0C},		//	[11:8]	x_addr_end H
{0x0349,0xC7},		//	[7:0]	x_addr_end L
{0x034A,0x09},		//	[11:8]	y_addr_end H
{0x034B,0x9F},		//	[7:0]	y_addr_end L
{0x034C,0x06},//0C		//	[11:8]	x_output_size H
{0x034D,0x60},//C0		//	[7:0]	x_output_size L
{0x034E,0x04},//09		//	[11:8]	y_output_size H
{0x034F,0xD0},//90		//	[7:0]	y_output_size L

// Analog Binning ----------------------------------
{0x0390,0x01},		//	[7:0]	binning_mode ([0] binning enable)
{0x0391,0x22},		//	[7:0]	binning_type (22h : 2x2 binning, 44h : 4x4 binning)

// Sub-Sampling Ratio ------------------------------
{0x0381,0x01},		//	[4:0]	x_even_inc
{0x0383,0x03},		//	[4:0]	x_odd_inc
{0x0385,0x01},		//	[4:0]	y_even_inc
{0x0387,0x03},		//	[4:0]	y_odd_inc

// PLL control -------------------------------------
{0x0301,0x02},		//	[3:0]	vt_pix_clk_div
{0x0303,0x01},		//	[3:0]	vt_sys_clk_div
{0x0305,0x05},		//	[5:0]	pre_pll_clk_div
{0x0306,0x00},		//	[9:8]	pll_multiplier H
{0x0307,0x8C},		//	[7:0]	pll_multiplier L
{0x0309,0x02},		//	[3:0]	op_pix_clk_div
{0x030B,0x01},		//	[3:0]	op_sys_clk_div
{0x3C59,0x00},		//	[2:0]	reg_PLL_S
{0x030D,0x05},		//	[5:0]	out_pre_pll_clk_div
{0x030E,0x00},		//	[9:8]	out_pll_multiplier H 
{0x030F,0xAF},//A5		//	[7:0]	out_pll_multiplier L
{0x3C5A,0x00},		//	[2:0]	reg_out_PLL_S
{0x0310,0x01},		//	[0]	pll_mode (01h : 2-PLL, 00h : 1-PLL)
{0x3C50,0x53},		//	[7:4]	reg_DIV_DBR
		//	[3:0]	reg_DIV_G
{0x3C62,0x02},		//	[31:24]	requested_link_bit_rate_mbps HH
{0x3C63,0xBC},//94		//	[23:16]	requested_link_bit_rate_mbps HL
{0x3C64,0x00},		//	[15:8]	requested_link_bit_rate_mbps LH
{0x3C65,0x00},		//	[7:0]	requested_link_bit_rate_mbps LL

// Embedded line on/off ----------------------------
{0x3C1E,0x00},		//	[3]	reg_isp_fe_TN_SMIA_sync_sel
		//	[2]	reg_pat_TN_SMIA_sync_sel
		//	[1]	reg_bpc_TN_SMIA_sync_sel
		//	[0]	reg_outif_TN_SMIA_sync_sel




	
{0x302A,0x0A},	
{0x303D,0x06},	
{0x304B,0x2A},	


{0x3205,0x84},	
{0x3207,0x85},	
{0x3214,0x94},	
{0x3216,0x95},	
{0x303a,0x9f},	
{0x3201,0x07},	
{0x3051,0xff},	
{0x3052,0xff},	
{0x3054,0xF0},	
{0x3037,0x12}, 
{0x305C,0x8F},
{0x302D,0x7F}, 


{0x3915,0x00},	
{0x3916,0x4E},	
{0x3917,0x21},	
{0x393C,0x0F},	
{0x393D,0xA1},	
{0x3C12,0x00},	
{0x3C13,0x55},	
{0x3C14,0xF1},	
{0x3C15,0x00},	
{0x3C16,0x4E},	
{0x3C17,0x21},	

// Analog Tuning End


// Streaming On ------------------------------------
{0x0100,0x01},		//	[0]	mode_select
};

const struct _sensor_reg_t s5k4h5yx_framesize_full_30fps[] = {
	//$MIPI[Width:3264,Height:2448,Format:Raw10,Lane:4,ErrorCheck:0,PolarityData:0,PolarityClock:0,Buffer:4]
////////////////////////////////////////////////////
//  name:  S5K4H5YX EVT1 setfile
//  v0.00 Temporary version
//  Image size : 3264x2448, 20 fps
//  EXTCLK : 20 MHz
//  System PLL output : 560 MHz
//  Output PLL output : 500 Mbps
//  GCLK (ATOP) : 280 MHz
//  DCLK_4 (ATOP) : 70 MHz
////////////////////////////////////////////////////
// Streaming off -----------------------------------
{0x0100,0x00},		//	[0]	mode_select

// Image Orientation -------------------------------
{0x0101,0x00},		//	[1:0]	image_orientation ([0] mirror en, [1] flip en)

// Analog Gain -------------------------------------
{0x0204,0x00},		//	[15:8]	analogue_gain_code_global H
{0x0205,0x1F},		//	[7:0]	analogue_gain_code_global L

// Exposure Time -----------------------------------
{0x0200,0x14},		//	[15:8]	fine_integration_time H
{0x0201,0x04},		//	[7:0]	fine_integration_time L
{0x0202,0x03},		//	[15:8]	coarse_integration_time H
{0x0203,0xD2},		//	[7:0]	coarse_integration_time L

// Frame Rate --------------------------------------
{0x0340,0x09},		//	[15:8]	frame_length_lines H
{0x0341,0xE3},		//	[7:0]	frame_length_lines L
{0x0342,0x1C},		//	[15:8]	line_length_pck H
{0x0343,0x70},		//	[7:0]	line_length_pck L

// Image Size --------------------------------------
{0x0344,0x00},		//	[11:8]	x_addr_start H
{0x0345,0x08},		//	[7:0]	x_addr_start L
{0x0346,0x00},		//	[11:8]	y_addr_start H
{0x0347,0x08},		//	[7:0]	y_addr_start L
{0x0348,0x0C},		//	[11:8]	x_addr_end H
{0x0349,0xC7},		//	[7:0]	x_addr_end L
{0x034A,0x09},		//	[11:8]	y_addr_end H
{0x034B,0x97},		//	[7:0]	y_addr_end L
{0x034C,0x0C},		//	[11:8]	x_output_size H
{0x034D,0xC0},		//	[7:0]	x_output_size L
{0x034E,0x09},		//	[11:8]	y_output_size H
{0x034F,0x90},		//	[7:0]	y_output_size L

// Analog Binning ----------------------------------
{0x0390,0x00},		//	[7:0]	binning_mode ([0] binning enable)
{0x0391,0x00},		//	[7:0]	binning_type (22h : 2x2 binning, 44h : 4x4 binning)

// Sub-Sampling Ratio ------------------------------
{0x0381,0x01},		//	[4:0]	x_even_inc
{0x0383,0x01},		//	[4:0]	x_odd_inc
{0x0385,0x01},		//	[4:0]	y_even_inc
{0x0387,0x01},		//	[4:0]	y_odd_inc

// PLL control -------------------------------------
{0x0301,0x02},		//	[3:0]	vt_pix_clk_div
{0x0303,0x01},		//	[3:0]	vt_sys_clk_div
{0x0305,0x06},		//	[5:0]	pre_pll_clk_div
{0x0306,0x00},		//	[9:8]	pll_multiplier H
{0x0307,0x8C},		//	[7:0]	pll_multiplier L
{0x0309,0x02},		//	[3:0]	op_pix_clk_div
{0x030B,0x01},		//	[3:0]	op_sys_clk_div
{0x3C59,0x00},		//	[2:0]	reg_PLL_S
{0x030D,0x06},		//	[5:0]	out_pre_pll_clk_div
{0x030E,0x00},		//	[9:8]	out_pll_multiplier H
{0x030F,0x7D},		//	[7:0]	out_pll_multiplier L
{0x3C5A,0x00},		//	[2:0]	reg_out_PLL_S
{0x0310,0x01},		//	[0]	pll_mode (01h : 2-PLL, 00h : 1-PLL)
{0x3C50,0x53},		//	[7:4]	reg_DIV_DBR
		//	[3:0]	reg_DIV_G
{0x3C62,0x01},		//	[31:24]	requested_link_bit_rate_mbps HH
{0x3C63,0xF4},		//	[23:16]	requested_link_bit_rate_mbps HL
{0x3C64,0x00},		//	[15:8]	requested_link_bit_rate_mbps LH
{0x3C65,0x00},		//	[7:0]	requested_link_bit_rate_mbps LL

// Embedded line on/off ----------------------------
{0x3C1E,0x00},		//	[3]	reg_isp_fe_TN_SMIA_sync_sel
		//	[2]	reg_pat_TN_SMIA_sync_sel
		//	[1]	reg_bpc_TN_SMIA_sync_sel
		//	[0]	reg_outif_TN_SMIA_sync_sel

// Out_if control ----------------------------------
{0x3915,0x00},		//	[1:0]	outif_mld_ulpm_rxinit_limit_2
{0x3916,0x4E},		//	[7:0]	outif_mld_ulpm_rxinit_limit_1
{0x3917,0x21},		//	[7:0]	outif_mld_ulpm_rxinit_limit_0
{0x393C,0x0F},		//	[7:0]	reg_outif_init_time_up
{0x393D,0xA1},		//	[7:0]	reg_outif_init_time_dn
{0x3C12,0x00},		//	[1:0]	reg_streaming_enable_time_2
{0x3C13,0x55},		//	[7:0]	reg_streaming_enable_time_1
{0x3C14,0xF1},		//	[7:0]	reg_streaming_enable_time_0
{0x3C15,0x00},		//	[1:0]	reg_outif_enable_time_2
{0x3C16,0x4E},		//	[7:0]	reg_outif_enable_time_1
{0x3C17,0x21},		//	[7:0]	reg_outif_enable_time_0

// Analog Tuning Start
// revision history
// 2012.11.21 EVT1 initial setfile
// 2012.12.10 dshut_en=1
// 2013.1.3 dbs_option 00h -> 12h

{0x0202,0x00},	
{0x302A,0x0A},	
{0x303D,0x06},	
{0x304B,0x2A},	
{0x0204,0x02},	
{0x0205,0x00},	
{0x3205,0x84},	
{0x3207,0x85},	
{0x3214,0x94},	
{0x3216,0x95},	
{0x303a,0x9f},	
{0x3201,0x07},	
{0x3051,0xff},	
{0x3052,0xff},	
{0x3054,0xF0},	
{0x3037,0x12}, 
{0x305C,0x8F},	
{0x302D,0x7F}, 

// Analog Tuning End

// Streaming On ------------------------------------
{0x0100,0x01},		//	[0]	mode_select
  
};

/*RES_3264x2448 full size 24fps*/
const struct _sensor_reg_t s5k4h5yx_framesize_full_24fps[] = {
	//Full size - 24 fps
////////////////////////////////////////////////////
//  name:  S5K4H5YX EVT1 setfile
//  v0.00 Temporary version
//  Image size : Full size - Full size - 24  fps
//  EXTCLK : 20 MHz
//  System PLL output : 560 MHz
//  Output PLL output : 700 Mbps
//  GCLK (ATOP) : 280 MHz
//  DCLK_4 (ATOP) : 70 MHz
//  Like 3H7 (3268x2452, v-blank 79 line), Embedded line off
////////////////////////////////////////////////////




// Streaming off -----------------------------------
{0x0100,0x00},		//	[0]	mode_select

// Image Orientation -------------------------------
{0x0101,0x00},		//	[1:0]	image_orientation ([0] mirror en, [1] flip en)

// Analog Gain -------------------------------------
{0x0204,0x00},		//	[15:8]	analogue_gain_code_global H
{0x0205,0x20},		//	[7:0]	analogue_gain_code_global L

// Exposure Time -----------------------------------
{0x0200,0x14},		//	[15:8]	fine_integration_time H
{0x0201,0x04},		//	[7:0]	fine_integration_time L
{0x0202,0x04},		//	[15:8]	coarse_integration_time H
{0x0203,0xE2},		//	[7:0]	coarse_integration_time L

// Frame Rate --------------------------------------
{0x0340,0x0C},//09		//	[15:8]	frame_length_lines H
{0x0341,0x5B},//E3		//	[7:0]	frame_length_lines L
{0x0342,0x0E},//12//0F//0E		//	[15:8]	line_length_pck H
{0x0343,0x68},//DE//6F//68		//	[7:0]	line_length_pck L

// Image Size --------------------------------------
{0x0344,0x00},		//	[11:8]	x_addr_start H
{0x0345,0x08},		//	[7:0]	x_addr_start L
{0x0346,0x00},		//	[11:8]	y_addr_start H
{0x0347,0x00},		//	[7:0]	y_addr_start L
{0x0348,0x0C},		//	[11:8]	x_addr_end H
{0x0349,0xC7},		//	[7:0]	x_addr_end L
{0x034A,0x09},		//	[11:8]	y_addr_end H
{0x034B,0x9F},		//	[7:0]	y_addr_end L
{0x034C,0x0C},//0C		//	[11:8]	x_output_size H
{0x034D,0xC0},//C0		//	[7:0]	x_output_size L
{0x034E,0x09},//09		//	[11:8]	y_output_size H
{0x034F,0x90},//90		//	[7:0]	y_output_size L

// Analog Binning ----------------------------------
{0x0390,0x00},		//	[7:0]	binning_mode ([0] binning enable)
{0x0391,0x00},		//	[7:0]	binning_type (22h : 2x2 binning, 44h : 4x4 binning)

// Sub-Sampling Ratio ------------------------------
{0x0381,0x01},		//	[4:0]	x_even_inc
{0x0383,0x01},		//	[4:0]	x_odd_inc
{0x0385,0x01},		//	[4:0]	y_even_inc
{0x0387,0x01},		//	[4:0]	y_odd_inc

// PLL control -------------------------------------
{0x0301,0x02},		//	[3:0]	vt_pix_clk_div
{0x0303,0x01},		//	[3:0]	vt_sys_clk_div
{0x0305,0x05},		//	[5:0]	pre_pll_clk_div
{0x0306,0x00},		//	[9:8]	pll_multiplier H
{0x0307,0x8C},		//	[7:0]	pll_multiplier L
{0x0309,0x02},		//	[3:0]	op_pix_clk_div
{0x030B,0x01},		//	[3:0]	op_sys_clk_div
{0x3C59,0x00},		//	[2:0]	reg_PLL_S
{0x030D,0x05},		//	[5:0]	out_pre_pll_clk_div
{0x030E,0x00},		//	[9:8]	out_pll_multiplier H 
{0x030F,0xAF},//A5		//	[7:0]	out_pll_multiplier L
{0x3C5A,0x00},		//	[2:0]	reg_out_PLL_S
{0x0310,0x01},		//	[0]	pll_mode (01h : 2-PLL, 00h : 1-PLL)
{0x3C50,0x53},		//	[7:4]	reg_DIV_DBR
		//	[3:0]	reg_DIV_G
{0x3C62,0x02},		//	[31:24]	requested_link_bit_rate_mbps HH
{0x3C63,0xBC},//94		//	[23:16]	requested_link_bit_rate_mbps HL
{0x3C64,0x00},		//	[15:8]	requested_link_bit_rate_mbps LH
{0x3C65,0x00},		//	[7:0]	requested_link_bit_rate_mbps LL

// Embedded line on/off ----------------------------
{0x3C1E,0x00},		//	[3]	reg_isp_fe_TN_SMIA_sync_sel
		//	[2]	reg_pat_TN_SMIA_sync_sel
		//	[1]	reg_bpc_TN_SMIA_sync_sel
		//	[0]	reg_outif_TN_SMIA_sync_sel





{0x302A,0x0A},
{0x303D,0x06},
{0x304B,0x2A},


{0x3205,0x84},
{0x3207,0x85},
{0x3214,0x94},
{0x3216,0x95},
{0x303a,0x9f},
{0x3201,0x07},
{0x3051,0xff},
{0x3052,0xff},
{0x3054,0xF0},
{0x3037,0x12},
{0x305C,0x8F},
{0x302D,0x7F},


{0x3915,0x00},	
{0x3916,0x4E},	
{0x3917,0x21},	
{0x393C,0x0F},	
{0x393D,0xA1},	
{0x3C12,0x00},	
{0x3C13,0x55},	
{0x3C14,0xF1},	
{0x3C15,0x00},	
{0x3C16,0x4E},	
{0x3C17,0x21},	

// Analog Tuning End


// Streaming On ------------------------------------
{0x0100,0x01},		//	[0]	mode_select
	
};

//support solutions and fps in datasheet P23.
const struct _sensor_reg_t s5k4h5yx_framesize_full_15fps[] = {
	//Full size - 15 fps
////////////////////////////////////////////////////
//  name:  S5K4H5YX EVT1 setfile
//  v0.00 Temporary version
//  Image size : Full size - 1632X1224, 30 fps
//  EXTCLK : 20 MHz
//  System PLL output : 560 MHz
//  Output PLL output : 700 Mbps
//  GCLK (ATOP) : 280 MHz
//  DCLK_4 (ATOP) : 70 MHz
//  Like 3H7 (3268x2452, v-blank 79 line), Embedded line off
////////////////////////////////////////////////////

//$MIPI[Width:1632,Height:1224,Format:Raw10,Lane:4,ErrorCheck:0,PolarityData:0,PolarityClock:0,Buffer:4]


// Streaming off -----------------------------------
{0x0100,0x00},		//	[0]	mode_select

// Image Orientation -------------------------------
{0x0101,0x00},		//	[1:0]	image_orientation ([0] mirror en, [1] flip en)

// Analog Gain -------------------------------------
{0x0204,0x00},		//	[15:8]	analogue_gain_code_global H
{0x0205,0x20},		//	[7:0]	analogue_gain_code_global L

// Exposure Time -----------------------------------
{0x0200,0x14},		//	[15:8]	fine_integration_time H
{0x0201,0x04},		//	[7:0]	fine_integration_time L
{0x0202,0x04},		//	[15:8]	coarse_integration_time H
{0x0203,0xE2},		//	[7:0]	coarse_integration_time L

// Frame Rate --------------------------------------
{0x0340,0x13},//09		//	[15:8]	frame_length_lines H
{0x0341,0xC6},//E3		//	[7:0]	frame_length_lines L
{0x0342,0x0E},//12//0F//0E		//	[15:8]	line_length_pck H
{0x0343,0x68},//DE//6F//68		//	[7:0]	line_length_pck L

// Image Size --------------------------------------
{0x0344,0x00},		//	[11:8]	x_addr_start H
{0x0345,0x08},		//	[7:0]	x_addr_start L
{0x0346,0x00},		//	[11:8]	y_addr_start H
{0x0347,0x00},		//	[7:0]	y_addr_start L
{0x0348,0x0C},		//	[11:8]	x_addr_end H
{0x0349,0xC7},		//	[7:0]	x_addr_end L
{0x034A,0x09},		//	[11:8]	y_addr_end H
{0x034B,0x9F},		//	[7:0]	y_addr_end L
{0x034C,0x0C},//0C		//	[11:8]	x_output_size H
{0x034D,0xC0},//C0		//	[7:0]	x_output_size L
{0x034E,0x09},//09		//	[11:8]	y_output_size H
{0x034F,0x90},//90		//	[7:0]	y_output_size L

// Analog Binning ----------------------------------
{0x0390,0x00},		//	[7:0]	binning_mode ([0] binning enable)
{0x0391,0x00},		//	[7:0]	binning_type (22h : 2x2 binning, 44h : 4x4 binning)

// Sub-Sampling Ratio ------------------------------
{0x0381,0x01},		//	[4:0]	x_even_inc
{0x0383,0x01},		//	[4:0]	x_odd_inc
{0x0385,0x01},		//	[4:0]	y_even_inc
{0x0387,0x01},		//	[4:0]	y_odd_inc

// PLL control -------------------------------------
{0x0301,0x02},		//	[3:0]	vt_pix_clk_div
{0x0303,0x01},		//	[3:0]	vt_sys_clk_div
{0x0305,0x05},		//	[5:0]	pre_pll_clk_div
{0x0306,0x00},		//	[9:8]	pll_multiplier H
{0x0307,0x8C},		//	[7:0]	pll_multiplier L
{0x0309,0x02},		//	[3:0]	op_pix_clk_div
{0x030B,0x01},		//	[3:0]	op_sys_clk_div
{0x3C59,0x00},		//	[2:0]	reg_PLL_S
{0x030D,0x05},		//	[5:0]	out_pre_pll_clk_div
{0x030E,0x00},		//	[9:8]	out_pll_multiplier H 
{0x030F,0xAF},//A5		//	[7:0]	out_pll_multiplier L
{0x3C5A,0x00},		//	[2:0]	reg_out_PLL_S
{0x0310,0x01},		//	[0]	pll_mode (01h : 2-PLL, 00h : 1-PLL)
{0x3C50,0x53},		//	[7:4]	reg_DIV_DBR
		//	[3:0]	reg_DIV_G
{0x3C62,0x02},		//	[31:24]	requested_link_bit_rate_mbps HH
{0x3C63,0xBC},//94		//	[23:16]	requested_link_bit_rate_mbps HL
{0x3C64,0x00},		//	[15:8]	requested_link_bit_rate_mbps LH
{0x3C65,0x00},		//	[7:0]	requested_link_bit_rate_mbps LL

// Embedded line on/off ----------------------------
{0x3C1E,0x00},		//	[3]	reg_isp_fe_TN_SMIA_sync_sel
		//	[2]	reg_pat_TN_SMIA_sync_sel
		//	[1]	reg_bpc_TN_SMIA_sync_sel
		//	[0]	reg_outif_TN_SMIA_sync_sel





{0x302A,0x0A},
{0x303D,0x06},
{0x304B,0x2A},


{0x3205,0x84},
{0x3207,0x85},
{0x3214,0x94},
{0x3216,0x95},
{0x303a,0x9f},
{0x3201,0x07},
{0x3051,0xff},
{0x3052,0xff},
{0x3054,0xF0},
{0x3037,0x12},
{0x305C,0x8F},
{0x302D,0x7F},


{0x3915,0x00},	
{0x3916,0x4E},	
{0x3917,0x21},	
{0x393C,0x0F},	
{0x393D,0xA1},	
{0x3C12,0x00},	
{0x3C13,0x55},	
{0x3C14,0xF1},	
{0x3C15,0x00},	
{0x3C16,0x4E},	
{0x3C17,0x21},	

// Analog Tuning End


// Streaming On ------------------------------------
{0x0100,0x01},		//	[0]	mode_select
	
};

const struct _sensor_reg_t s5k4h5yx_framesize_full_es[] = {

};

/* j00179721_2012-05-07 */
/*RES_3264x1836 crop 30fps */	
const struct _sensor_reg_t s5k4h5yx_framesize_3264x1840[] = {
	//Full size 3264x1836 - 30 fps
////////////////////////////////////////////////////
//  name:  S5K4H5YX EVT1 setfile
//  v0.00 Temporary version
//  Image size : Full size - 3264x1836 - 30 fps
//  EXTCLK : 20 MHz
//  System PLL output : 560 MHz
//  Output PLL output : 700 Mbps
//  GCLK (ATOP) : 280 MHz
//  DCLK_4 (ATOP) : 70 MHz
//  Like 3H7 (3268x2452, v-blank 79 line), Embedded line off
////////////////////////////////////////////////////




// Streaming off -----------------------------------
{0x0100,0x00},		//	[0]	mode_select

// Image Orientation -------------------------------
{0x0101,0x00},		//	[1:0]	image_orientation ([0] mirror en, [1] flip en)

// Analog Gain -------------------------------------
{0x0204,0x00},		//	[15:8]	analogue_gain_code_global H
{0x0205,0x20},		//	[7:0]	analogue_gain_code_global L

// Exposure Time -----------------------------------
{0x0200,0x14},		//	[15:8]	fine_integration_time H
{0x0201,0x04},		//	[7:0]	fine_integration_time L
{0x0202,0x04},		//	[15:8]	coarse_integration_time H
{0x0203,0xE2},		//	[7:0]	coarse_integration_time L

// Frame Rate --------------------------------------
{0x0340,0x09},//09		//	[15:8]	frame_length_lines H
{0x0341,0xE3},//E3		//	[7:0]	frame_length_lines L
{0x0342,0x0E},//12//0F//0E		//	[15:8]	line_length_pck H
{0x0343,0x68},//DE//6F//68		//	[7:0]	line_length_pck L

// Image Size --------------------------------------
{0x0344,0x00},		//	[11:8]	x_addr_start H
{0x0345,0x08},		//	[7:0]	x_addr_start L
{0x0346,0x01},		//	[11:8]	y_addr_start H
{0x0347,0x38},		//	[7:0]	y_addr_start L
{0x0348,0x0C},		//	[11:8]	x_addr_end H
{0x0349,0xC7},		//	[7:0]	x_addr_end L
{0x034A,0x08},		//	[11:8]	y_addr_end H
{0x034B,0x67},		//	[7:0]	y_addr_end L
{0x034C,0x0C},//0C		//	[11:8]	x_output_size H
{0x034D,0xC0},//C0		//	[7:0]	x_output_size L
{0x034E,0x07},//09		//	[11:8]	y_output_size H
{0x034F,0x30},//90		//	[7:0]	y_output_size L

// Analog Binning ----------------------------------
{0x0390,0x00},		//	[7:0]	binning_mode ([0] binning enable)
{0x0391,0x00},		//	[7:0]	binning_type (22h : 2x2 binning, 44h : 4x4 binning)

// Sub-Sampling Ratio ------------------------------
{0x0381,0x01},		//	[4:0]	x_even_inc
{0x0383,0x01},		//	[4:0]	x_odd_inc
{0x0385,0x01},		//	[4:0]	y_even_inc
{0x0387,0x01},		//	[4:0]	y_odd_inc

// PLL control -------------------------------------
{0x0301,0x02},		//	[3:0]	vt_pix_clk_div
{0x0303,0x01},		//	[3:0]	vt_sys_clk_div
{0x0305,0x05},		//	[5:0]	pre_pll_clk_div
{0x0306,0x00},		//	[9:8]	pll_multiplier H
{0x0307,0x8C},		//	[7:0]	pll_multiplier L
{0x0309,0x02},		//	[3:0]	op_pix_clk_div
{0x030B,0x01},		//	[3:0]	op_sys_clk_div
{0x3C59,0x00},		//	[2:0]	reg_PLL_S
{0x030D,0x05},		//	[5:0]	out_pre_pll_clk_div
{0x030E,0x00},		//	[9:8]	out_pll_multiplier H 
{0x030F,0xAF},//A5		//	[7:0]	out_pll_multiplier L
{0x3C5A,0x00},		//	[2:0]	reg_out_PLL_S
{0x0310,0x01},		//	[0]	pll_mode (01h : 2-PLL, 00h : 1-PLL)
{0x3C50,0x53},		//	[7:4]	reg_DIV_DBR
		//	[3:0]	reg_DIV_G
{0x3C62,0x02},		//	[31:24]	requested_link_bit_rate_mbps HH
{0x3C63,0xBC},//94		//	[23:16]	requested_link_bit_rate_mbps HL
{0x3C64,0x00},		//	[15:8]	requested_link_bit_rate_mbps LH
{0x3C65,0x00},		//	[7:0]	requested_link_bit_rate_mbps LL

// Embedded line on/off ----------------------------
{0x3C1E,0x00},		//	[3]	reg_isp_fe_TN_SMIA_sync_sel
		//	[2]	reg_pat_TN_SMIA_sync_sel
		//	[1]	reg_bpc_TN_SMIA_sync_sel
		//	[0]	reg_outif_TN_SMIA_sync_sel





{0x302A,0x0A},
{0x303D,0x06},
{0x304B,0x2A},


{0x3205,0x84},
{0x3207,0x85},
{0x3214,0x94},
{0x3216,0x95},
{0x303a,0x9f},
{0x3201,0x07},
{0x3051,0xff},
{0x3052,0xff},
{0x3054,0xF0},
{0x3037,0x12},
{0x305C,0x8F},
{0x302D,0x7F},


{0x3915,0x00},	
{0x3916,0x4E},	
{0x3917,0x21},	
{0x393C,0x0F},	
{0x393D,0xA1},	
{0x3C12,0x00},	
{0x3C13,0x55},	
{0x3C14,0xF1},	
{0x3C15,0x00},	
{0x3C16,0x4E},	
{0x3C17,0x21},	

// Analog Tuning End


// Streaming On ------------------------------------
{0x0100,0x01},		//	[0]	mode_select

};


//h00206029_20120120
const struct _sensor_reg_t s5k4h5yx_framesize_2592x1944[] = {

};
/*2048x1536*/
const struct _sensor_reg_t s5k4h5yx_framesize_2048x1536[] = {


};


/*1600x1200*/
const struct _sensor_reg_t s5k4h5yx_framesize_1600x1200[] = {

};


/*1920x1080*/
const struct _sensor_reg_t s5k4h5yx_framesize_1080p[] = {
	//Full size:1920x1080 - 30 fps
////////////////////////////////////////////////////
//  name:  S5K4H5YX EVT1 setfile
//  v0.00 Temporary version
//  Image size : Full size - 1632X1224, 30 fps
//  EXTCLK : 20 MHz
//  System PLL output : 560 MHz
//  Output PLL output : 700 Mbps
//  GCLK (ATOP) : 280 MHz
//  DCLK_4 (ATOP) : 70 MHz
//  Like 3H7 (3268x2452, v-blank 79 line), Embedded line off
////////////////////////////////////////////////////

//$MIPI[Width:1632,Height:1224,Format:Raw10,Lane:4,ErrorCheck:0,PolarityData:0,PolarityClock:0,Buffer:4]


// Streaming off -----------------------------------
{0x0100,0x00},		//	[0]	mode_select

// Image Orientation -------------------------------
{0x0101,0x00},		//	[1:0]	image_orientation ([0] mirror en, [1] flip en)

// Analog Gain -------------------------------------
{0x0204,0x00},		//	[15:8]	analogue_gain_code_global H
{0x0205,0x20},		//	[7:0]	analogue_gain_code_global L

// Exposure Time -----------------------------------
{0x0200,0x14},		//	[15:8]	fine_integration_time H
{0x0201,0x04},		//	[7:0]	fine_integration_time L
{0x0202,0x04},		//	[15:8]	coarse_integration_time H
{0x0203,0xE2},		//	[7:0]	coarse_integration_time L

// Frame Rate --------------------------------------
{0x0340,0x09},//09		//	[15:8]	frame_length_lines H
{0x0341,0xE3},//E3		//	[7:0]	frame_length_lines L
{0x0342,0x0E},//12//0F//0E		//	[15:8]	line_length_pck H
{0x0343,0x68},//DE//6F//68		//	[7:0]	line_length_pck L

// Image Size --------------------------------------
{0x0344,0x02},		//	[11:8]	x_addr_start H
{0x0345,0xA8},		//	[7:0]	x_addr_start L
{0x0346,0x02},		//	[11:8]	y_addr_start H
{0x0347,0xB4},		//	[7:0]	y_addr_start L
{0x0348,0x0A},		//	[11:8]	x_addr_end H
{0x0349,0x27},		//	[7:0]	x_addr_end L
{0x034A,0x06},		//	[11:8]	y_addr_end H
{0x034B,0xEB},		//	[7:0]	y_addr_end L
{0x034C,0x07},//0C		//	[11:8]	x_output_size H
{0x034D,0x80},//C0		//	[7:0]	x_output_size L
{0x034E,0x04},//09		//	[11:8]	y_output_size H
{0x034F,0x38},//90		//	[7:0]	y_output_size L

// Analog Binning ----------------------------------
{0x0390,0x00},		//	[7:0]	binning_mode ([0] binning enable)
{0x0391,0x00},		//	[7:0]	binning_type (22h : 2x2 binning, 44h : 4x4 binning)

// Sub-Sampling Ratio ------------------------------
{0x0381,0x01},		//	[4:0]	x_even_inc
{0x0383,0x01},		//	[4:0]	x_odd_inc
{0x0385,0x01},		//	[4:0]	y_even_inc
{0x0387,0x01},		//	[4:0]	y_odd_inc

// PLL control -------------------------------------
{0x0301,0x02},		//	[3:0]	vt_pix_clk_div
{0x0303,0x01},		//	[3:0]	vt_sys_clk_div
{0x0305,0x05},		//	[5:0]	pre_pll_clk_div
{0x0306,0x00},		//	[9:8]	pll_multiplier H
{0x0307,0x8C},		//	[7:0]	pll_multiplier L
{0x0309,0x02},		//	[3:0]	op_pix_clk_div
{0x030B,0x01},		//	[3:0]	op_sys_clk_div
{0x3C59,0x00},		//	[2:0]	reg_PLL_S
{0x030D,0x05},		//	[5:0]	out_pre_pll_clk_div
{0x030E,0x00},		//	[9:8]	out_pll_multiplier H 
{0x030F,0xAF},//A5		//	[7:0]	out_pll_multiplier L
{0x3C5A,0x00},		//	[2:0]	reg_out_PLL_S
{0x0310,0x01},		//	[0]	pll_mode (01h : 2-PLL, 00h : 1-PLL)
{0x3C50,0x53},		//	[7:4]	reg_DIV_DBR
		//	[3:0]	reg_DIV_G
{0x3C62,0x02},		//	[31:24]	requested_link_bit_rate_mbps HH
{0x3C63,0xBC},//94		//	[23:16]	requested_link_bit_rate_mbps HL
{0x3C64,0x00},		//	[15:8]	requested_link_bit_rate_mbps LH
{0x3C65,0x00},		//	[7:0]	requested_link_bit_rate_mbps LL

// Embedded line on/off ----------------------------
{0x3C1E,0x00},		//	[3]	reg_isp_fe_TN_SMIA_sync_sel
		//	[2]	reg_pat_TN_SMIA_sync_sel
		//	[1]	reg_bpc_TN_SMIA_sync_sel
		//	[0]	reg_outif_TN_SMIA_sync_sel




	
{0x302A,0x0A},	
{0x303D,0x06},	
{0x304B,0x2A},	
	
	
{0x3205,0x84},	
{0x3207,0x85},	
{0x3214,0x94},	
{0x3216,0x95},	
{0x303a,0x9f},	
{0x3201,0x07},	
{0x3051,0xff},	
{0x3052,0xff},	
{0x3054,0xF0},	
{0x3037,0x12}, 
{0x305C,0x8F},	
{0x302D,0x7F}, 


{0x3915,0x00},	
{0x3916,0x4E},	
{0x3917,0x21},	
{0x393C,0x0F},	
{0x393D,0xA1},	
{0x3C12,0x00},	
{0x3C13,0x55},	
{0x3C14,0xF1},	
{0x3C15,0x00},	
{0x3C16,0x4E},	
{0x3C17,0x21},	

// Analog Tuning End


// Streaming On ------------------------------------
{0x0100,0x01},		//	[0]	mode_select

};

/*2112x1200*/
const struct _sensor_reg_t s5k4h5yx_framesize_1080pEIS[] = {



};

#endif /* S5K4H5YX_H_INCLUDED */

/************************** END ***************************/
