/*
 *  OV8850 camera driver source file
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/videodev2.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <asm/div64.h>
#include <mach/hisi_mem.h>
#include "mach/hardware.h"
#include <mach/boardid.h>
#include <mach/gpio.h>
#include "../isp/sensor_common.h"
#include "ov8850.h"
/*#include "../isp/k3_isp_io.h"*/
#include <asm/bug.h>
#include <linux/device.h>
#include <../isp/cam_util.h>
#define LOG_TAG "OV8850"
/* #define DEBUG_DEBUG 1 */
#include "../isp/cam_log.h"
#include <hsad/config_interface.h>

/*
 * OV8850 module vendor code, according to OTP RULE
 *vendor		0x3D07 [7:4]
 *Sunny			0010
 *Foxconn		0011
 *Liteon		0100
 *SEMCO			0101
 */
#define OV8850_SUNNY_FACTORY_ID		0x02
#define OV8850_FOXCONN_FACTORY_ID	0x03
#define OV8850_LITEON_FACTORY_ID	0x04
#define OV8850_SEMCO_FACTORY_ID		0x05

#define OV8850_SLAVE_ADDRESS_20 0x20
#define OV8850_SLAVE_ADDRESS_6C 0x6c	//SID is HIGH connected to DOVDD
#define OV8850_CHIP_ID       (0x8850)	//chip id

#define OV8850_NO_FLIP	0x00
#define OV8850_H_FLIP	0x01
#define OV8850_V_FLIP	0x02
#define OV8850_HV_FLIP	0x03

//move the marco of register related to ov8850.h
#define OV8850_APERTURE_FACTOR		240 // F2.4
#define OV8850_EQUIVALENT_FOCUS		35  // view-angle equivalent to 35mm camera

/* camera sensor override parameters, define in binning preview mode */
#define OV8850_MAX_ISO			800
#define OV8850_MIN_ISO			100

#define OV8850_AUTO_FPS_MAX_GAIN	0x40
#define OV8850_AUTO_FPS_MIN_GAIN	0x16
#define OV8850_MAX_FRAMERATE		30
#define OV8850_MIN_FRAMERATE		10
#define OV8850_MIN_CAP_FRAMERATE	10

#define OV8850_FLASH_TRIGGER_GAIN	0x80

#define OV8850_SHARPNESS_PREVIEW	0x10
#define OV8850_SHARPNESS_CAPTURE	0x18

/*camera af param which are associated with sensor*/
#define OV8850_AF_MIN_HEIGHT_RATIO	(5)
#define OV8850_AF_MAX_FOCUS_STEP	(4)
#define OV8850_AF_GSENSOR_INTERVAL_THRESHOLD	(50)
#define OV8850_AF_WIDTH_PERCENT	(20)
#define OV8850_AF_HEIGHT_PERCENT	(25)

#define BOARD_ID_CS_G710C		0x30

//#define OV8850_AP_WRITEAE_MODE

static camera_capability ov8850_cap[] = {
	{V4L2_CID_FLASH_MODE, THIS_FLASH},
	{V4L2_CID_FOCUS_MODE, THIS_FOCUS_MODE},
};


/*move the isp initial code to ov8850.h*/

static u16 ov8850_vcm_start = 0;
static u16 ov8850_vcm_end = 0;
static int ov8850_otp_index = -1;
/*initial the var with a unnormal value*/
static u8 factory_id = 0x12;
/*the framesize should be sorted by size, so must be modified in furture*/
/*new setting is for ov8850, so delete invalid framesizes setting for ov8850*/
/*delete other non-use sensor size setting*/
static framesize_s ov8850_framesizes[] = {
	/* 1600x1200*/
	{0, 0, 1600, 1200, 3608, 1858, 30, 25, 0x22a, 0x1cd, 0x100, VIEW_FULL, RESOLUTION_4_3, true, {ov8850_framesize_1600x1200_new, ARRAY_SIZE(ov8850_framesize_1600x1200_new)} },
	/* 1080P, 1920*1088 25fps*/
	{0, 0, 1920, 1088, 3608, 2200, 25, 25, 0x22a, 0x1cd, 0x200, VIEW_FULL, RESOLUTION_16_9, false, {ov8850_framesize_1920X1088, ARRAY_SIZE(ov8850_framesize_1920X1088)} },
	/* 3264x2448 */
	{0, 0, 3264, 2448, 3788, 2637, 15, 13, 0x18b, 0x149, 0x16D, VIEW_FULL, RESOLUTION_4_3, false, {ov8850_framesize_3264x2448_new, ARRAY_SIZE(ov8850_framesize_3264x2448_new)} },
};
static camera_sensor ov8850_sensor;
static vcm_info_s ov8850_vcm;	//sensor_common.c DW9714
static void ov8850_set_default(void);
static void ov8850_get_vcm_otp(u16 *vcm_start, u16 *vcm_end);
bool ov8850_read_otp_vcm(u16 index, u8 *vcm_start, u8 *vcm_end);

/*
 **************************************************************************
 * FunctionName: ov8850_read_reg;
 * Description : read ov8850 reg by i2c;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int ov8850_read_reg(u16 reg, u8 *val)
{
	return k3_ispio_read_reg(ov8850_sensor.i2c_config.index,
				 ov8850_sensor.i2c_config.addr, reg, (u16*)val, ov8850_sensor.i2c_config.val_bits);
}

/*
 **************************************************************************
 * FunctionName: ov8850_write_reg;
 * Description : write ov8850 reg by i2c;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int ov8850_write_reg(u16 reg, u8 val, u8 mask)
{
	return k3_ispio_write_reg(ov8850_sensor.i2c_config.index,
			ov8850_sensor.i2c_config.addr, reg, val, ov8850_sensor.i2c_config.val_bits, mask);
}

/*
 **************************************************************************
 * FunctionName: ov8850_write_seq;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int ov8850_write_seq(const struct _sensor_reg_t *seq, u32 size, u8 mask)
{
	print_debug("Enter %s, seq[%#x], size=%d", __func__, (int)seq, size);
	return k3_ispio_write_seq(ov8850_sensor.i2c_config.index,
			ov8850_sensor.i2c_config.addr, seq, size, ov8850_sensor.i2c_config.val_bits, mask);
}

/*
 **************************************************************************
 * FunctionName: ov8850_write_isp_seq;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static void ov8850_write_isp_seq(const struct isp_reg_t *seq, u32 size)
{
	print_debug("Enter %s, seq[%#x], size=%d", __func__, (int)seq, size);
	k3_ispio_write_isp_seq(seq, size);
}

/*
 **************************************************************************
 * FunctionName: ov8850_read_isp_seq;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static void ov8850_read_isp_seq(struct isp_reg_t *seq, u32 size)
{
	print_debug("Enter %s, seq[%#x], size=%d", __func__, (int)seq, size);
	k3_ispio_read_isp_seq(seq, size);
}
/*
 **************************************************************************
 * FunctionName: ov8850_read_isp_reg;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : last modified j00212990;
 **************************************************************************
 */
static u32 ov8850_read_isp_reg(u32 reg_addr, u32 size)
{
	struct isp_reg_t reg_seq[4];
	int i = 0;
	u32 reg_value = 0x00;
	print_debug("Enter %s, size=%d", __func__, size);
	/*initialize buffer */
	for (i = 0; i < size; i++) {
		reg_seq[i].subaddr = reg_addr;
		reg_seq[i].value = 0x00;
		reg_seq[i].mask = 0x00;
		reg_addr++;
	}
	/*read register of isp for ov8850 */
	ov8850_read_isp_seq(reg_seq, size);
	/*construct return value */
	do {
		i--;
		reg_value = reg_value << 8;
		reg_value |= reg_seq[i].value;
		print_debug("reg_seq[%d].value:%x", i, reg_seq[i].value);
	} while (i > 0);
	return reg_value;
}
/*
 **************************************************************************
 * FunctionName: ov8850_write_isp_reg;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : last modified j00212990;
 **************************************************************************
 */
static void ov8850_write_isp_reg(u32 reg_addr, u32 data, u32 size)
{
	struct isp_reg_t reg_seq[4];
	int i = 0;
	print_debug("Enter %s, size=%d", __func__, size);
	print_debug("data:%x", data);
	/*initialize buffer */
	for (i = 0; i < size; i++) {
		reg_seq[i].subaddr = reg_addr;
		reg_seq[i].value = data & 0xFF;
		reg_seq[i].mask = 0x00;
		reg_addr++;
		data = data >> 8;
		print_debug("reg_seq[%d].subaddr:%x", i, reg_seq[i].subaddr);
		print_debug("reg_seq[%d].value:%x", i, reg_seq[i].value);
	}
	/*write register of isp for ov8850 */
	ov8850_write_isp_seq(reg_seq, size);
}
/*
 **************************************************************************
 * FunctionName: ov8850_enum_frame_intervals;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int ov8850_enum_frame_intervals(struct v4l2_frmivalenum *fi)
{
	assert(fi);

	print_debug("enter %s", __func__);
	if (fi->index > CAMERA_MAX_FRAMERATE) {
		return -EINVAL;
	}

	fi->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fi->discrete.numerator = 1;
	/* y36721:if denominator=0, means sensor support auto frame rate control. */
	fi->discrete.denominator = fi->index;
	return 0;
}

/*
 **************************************************************************
 * FunctionName: ov8850_get_format;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int ov8850_get_format(struct v4l2_fmtdesc *fmt)
{
	if (fmt->type == V4L2_BUF_TYPE_VIDEO_OVERLAY) {
		fmt->pixelformat = ov8850_sensor.fmt[STATE_PREVIEW];
	} else {
		fmt->pixelformat = ov8850_sensor.fmt[STATE_CAPTURE];
	}
	return 0;
}

/*
 **************************************************************************
 * FunctionName: ov8850_enum_framesizes;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int ov8850_enum_framesizes(struct v4l2_frmsizeenum *framesizes)
{
	u32 max_index = ARRAY_SIZE(camera_framesizes) - 1;
	u32 this_max_index = ARRAY_SIZE(ov8850_framesizes) - 1;

	assert(framesizes);

	print_debug("enter %s; ", __func__);

	if (framesizes->index > max_index) {
		print_error("framesizes->index = %d error", framesizes->index);
		return -EINVAL;
	}

	if ((camera_framesizes[framesizes->index].width > ov8850_framesizes[this_max_index].width)
		|| (camera_framesizes[framesizes->index].height > ov8850_framesizes[this_max_index].height)) {
		print_error("framesizes->index = %d error", framesizes->index);
		return -EINVAL;
	}

	framesizes->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	framesizes->discrete.width = ov8850_framesizes[this_max_index].width;
	framesizes->discrete.height = ov8850_framesizes[this_max_index].height;
	return 0;
}

/*
 **************************************************************************
 * FunctionName: ov8850_try_framesizes;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int ov8850_try_framesizes(struct v4l2_frmsizeenum *framesizes)
{
	int max_index = ARRAY_SIZE(ov8850_framesizes) - 1;

	assert(framesizes);

	print_debug("Enter Function:%s  ", __func__);

	if ((framesizes->discrete.width <= ov8850_framesizes[max_index].width)
	    && (framesizes->discrete.height <= ov8850_framesizes[max_index].height)) {
		print_debug("===========width = %d", framesizes->discrete.width);
		print_debug("===========height = %d", framesizes->discrete.height);
		return 0;
	}

	print_error("frame size too large, [%d,%d]",
		    framesizes->discrete.width, framesizes->discrete.height);
	return -EINVAL;
}

/*
 **************************************************************************
 * FunctionName: ov8850_set_framesizes;
 * Description : NA;
 * Input       : flag: if 1, set framesize to sensor,
 *					   if 0, only store framesize to camera_interface;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int ov8850_set_framesizes(camera_state state,
				 struct v4l2_frmsize_discrete *fs, int flag, camera_setting_view_type view_type)
{
	int i = 0;
	bool match = false;

	assert(fs);

	print_debug("Enter Function:%s State(%d), flag=%d, width=%d, height=%d",
		    __func__, state, flag, fs->width, fs->height);

	if (VIEW_FULL == view_type) {
		for (i = 0; i < ARRAY_SIZE(ov8850_framesizes); i++) {
			if ((ov8850_framesizes[i].width >= fs->width)
			    && (ov8850_framesizes[i].height >= fs->height)
			    && (VIEW_FULL == ov8850_framesizes[i].view_type)
			    && (camera_get_resolution_type(fs->width, fs->height)
			    <= ov8850_framesizes[i].resolution_type)) {
				fs->width = ov8850_framesizes[i].width;
				fs->height = ov8850_framesizes[i].height;
				match = true;
				break;
			}
		}
	}

	if (false == match) {
		for (i = 0; i < ARRAY_SIZE(ov8850_framesizes); i++) {
			if ((ov8850_framesizes[i].width >= fs->width)
			    && (ov8850_framesizes[i].height >= fs->height)
			    && (camera_get_resolution_type(fs->width, fs->height)
			    <= ov8850_framesizes[i].resolution_type)) {
				fs->width = ov8850_framesizes[i].width;
				fs->height = ov8850_framesizes[i].height;
				break;
			}
		}
	}

	if (i >= ARRAY_SIZE(ov8850_framesizes)) {
		print_error("request resolution larger than sensor's max resolution");
		return -EINVAL;
	}
	if (state == STATE_PREVIEW) {
		ov8850_sensor.preview_frmsize_index = i;
	} else {
		ov8850_sensor.capture_frmsize_index = i;
	}

	return 0;
}

/*
 **************************************************************************
 * FunctionName: ov8850_get_framesizes;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int ov8850_get_framesizes(camera_state state,
				 struct v4l2_frmsize_discrete *fs)
{
	int frmsize_index;

	assert(fs);

	if (state == STATE_PREVIEW) {
		frmsize_index = ov8850_sensor.preview_frmsize_index;
	} else if (state == STATE_CAPTURE) {
		frmsize_index = ov8850_sensor.capture_frmsize_index;
	} else {
		return -EINVAL;
	}
	fs->width = ov8850_framesizes[frmsize_index].width;
	fs->height = ov8850_framesizes[frmsize_index].height;

	return 0;
}

/*
 **************************************************************************
 * FunctionName: ov8850_init_reg;
 * Description : download initial seq for sensor init;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int ov8850_init_reg(void)
{
	int size = 0, i;
	print_debug("Enter Function:%s  , initsize=%d",
		    __func__, sizeof(ov8850_init_regs_new));

	if (0 != k3_ispio_init_csi(ov8850_sensor.mipi_index,
				 ov8850_sensor.mipi_lane_count, ov8850_sensor.lane_clk)) {
		return -EFAULT;
	}

	ov8850_write_reg(0x0100, 0x00, 0x00);
	ov8850_write_reg(0x0103, 0x01, 0x00);
	msleep(5);
	size = ARRAY_SIZE(ov8850_init_regs_new);
	if (0 != ov8850_write_seq(ov8850_init_regs_new, size, 0x00)) {
		print_error("fail to init ov8850 sensor");
		return -EFAULT;
	}

	/*make sure sensor stream on before OTP read/write*/
	ov8850_write_reg(0x0100, 0x01, 0x00);
	if (ov8850_otp_index != -1) {
		ov8850_update_otp_wb(ov8850_otp_index);
		ov8850_update_otp_lenc(ov8850_otp_index);
	} else {
		print_info("Failed to read ov8850 sensor otp!");
	}
	ov8850_update_blc();
	//ov8850_write_reg(0x0100, 0x00, 0x00);
	return 0;
}
static int ov8850_get_capability(u32 id, u32 *value)
{
	int i;
	for (i = 0; i < sizeof(ov8850_cap) / sizeof(ov8850_cap[0]); ++i) {
		if (id == ov8850_cap[i].id) {
			*value = ov8850_cap[i].value;
			break;
		}
	}
	return 0;
}

static int ov8850_set_hflip(int flip)
{
	print_debug("enter %s flip=%d", __func__, flip);
	ov8850_sensor.hflip = flip;
	return 0;
}
static int ov8850_get_hflip(void)
{
	print_debug("enter %s", __func__);

	return ov8850_sensor.hflip;
}
static int ov8850_set_vflip(int flip)
{
	print_debug("enter %s flip=%d", __func__, flip);

	ov8850_sensor.vflip = flip;

	return 0;
}
static int ov8850_get_vflip(void)
{
	print_debug("enter %s", __func__);
	return ov8850_sensor.vflip;
}
static int ov8850_update_flip(u16 width, u16 height)
{
	u8 new_flip = ((ov8850_sensor.vflip << 1) | ov8850_sensor.hflip);
    int flip_type = E_CAMERA_SENSOR_FLIP_TYPE_NONE;
	
	print_debug("Enter %s  ", __func__);

	flip_type = get_primary_sensor_flip_type();
	k3_ispio_update_flip((ov8850_sensor.old_flip ^ new_flip) & 0x03, width, height, PIXEL_ORDER_NO_CHANGED);

	ov8850_sensor.old_flip = new_flip;
	print_debug("%s:flip_type = %d, new_flip = %d", __func__, flip_type, new_flip);

    switch (flip_type) {
	case E_CAMERA_SENSOR_FLIP_TYPE_NONE:
    	ov8850_write_reg(OV8850REG_TIMING_FORMAT1, ov8850_sensor.vflip ? 0x42 : 0x00, ~0x42);
    	ov8850_write_reg(OV8850REG_TIMING_FORMAT2, ov8850_sensor.hflip ? 0x06 : 0x00, ~0x06);
		break;
	case E_CAMERA_SENSOR_FLIP_TYPE_H_V:
    	ov8850_write_reg(OV8850REG_TIMING_FORMAT1, ov8850_sensor.vflip ? 0x00 : 0x42, ~0x42);
    	ov8850_write_reg(OV8850REG_TIMING_FORMAT2, ov8850_sensor.hflip ? 0x00 : 0x06, ~0x06);
		break;
    case E_CAMERA_SENSOR_FLIP_TYPE_H:
    	ov8850_write_reg(OV8850REG_TIMING_FORMAT1, ov8850_sensor.vflip ? 0x42 : 0x00, ~0x42);
    	ov8850_write_reg(OV8850REG_TIMING_FORMAT2, ov8850_sensor.hflip ? 0x00 : 0x06, ~0x06);
		break;
    case E_CAMERA_SENSOR_FLIP_TYPE_V:
    	ov8850_write_reg(OV8850REG_TIMING_FORMAT1, ov8850_sensor.vflip ? 0x00 : 0x42, ~0x42);
    	ov8850_write_reg(OV8850REG_TIMING_FORMAT2, ov8850_sensor.hflip ? 0x06 : 0x00, ~0x06);
		break;
	default:
    	ov8850_write_reg(OV8850REG_TIMING_FORMAT1, ov8850_sensor.vflip ? 0x42 : 0x00, ~0x42);
    	ov8850_write_reg(OV8850REG_TIMING_FORMAT2, ov8850_sensor.hflip ? 0x06 : 0x00, ~0x06);
		break;
	}

	return 0;
}
void ov8850_set_vts(u16 vts)
{
	print_debug("Enter %s  ", __func__);
	ov8850_write_reg(OV8850_VTS_REG_H, (vts >> 8) & 0xff, 0x00);
	ov8850_write_reg(OV8850_VTS_REG_L, vts & 0xff, 0x00);
}

/*
 **************************************************************************
 * FunctionName: ov8850_framesize_switch;
 * Description : switch frame size, used by preview and capture
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int ov8850_framesize_switch(camera_state state)
{
	u8 next_frmsize_index;
	u16 vts;

	if (state == STATE_PREVIEW)
		next_frmsize_index = ov8850_sensor.preview_frmsize_index;
	else
		next_frmsize_index = ov8850_sensor.capture_frmsize_index;

	print_info("Enter Function:%s frm index=%d", __func__, next_frmsize_index);

	if (next_frmsize_index >= ARRAY_SIZE(ov8850_framesizes)){
		print_error("Unsupport sensor setting index: %d",next_frmsize_index);
		return -ETIME;
	}

	if (0 != ov8850_write_seq(ov8850_sensor.frmsize_list[next_frmsize_index].sensor_setting.setting
		,ov8850_sensor.frmsize_list[next_frmsize_index].sensor_setting.seq_size, 0x00)) {
		print_error("fail to init ov8850 sensor");
		return -ETIME;
	}
	msleep(3);

	/* use auto fps level at coolboot */
	vts = ov8850_sensor.frmsize_list[next_frmsize_index].vts *
		ov8850_sensor.frmsize_list[next_frmsize_index].fps / ov8850_sensor.fps;
	ov8850_set_vts(vts);
	ov8850_update_flip(ov8850_framesizes[next_frmsize_index].width,
			   ov8850_framesizes[next_frmsize_index].height);
	return 0;
}

/*
 **************************************************************************
 * FunctionName: ov8850_stream_on;
 * Description : download preview seq for sensor preview;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int ov8850_stream_on(camera_state state)
{
	print_debug("Enter Function:%s ", __func__);
	return ov8850_framesize_switch(state);
}
/*delete the i2c addr table because of no-use*/

/*delete ov8850_flash_awb and use boardid to distinguish*/
static awb_gain_t ov8850_flash_awb_g710c = {0xb4, 0x80, 0x80, 0xf2};

static void ov8850_get_flash_awb(flash_platform_t type, awb_gain_t *flash_awb)
{
	unsigned int boardid = get_boardid();
	
	if(BOARD_ID_CS_G710C == boardid)
	{
		*flash_awb = ov8850_flash_awb_g710c;
	}
	else
	{
		*flash_awb = ov8850_flash_awb_g710c;
	}
}
/* DTS20130302xxxx0 yuhaitao 20130301 end> */

static ccm_gain_t cap_gain[FRAMESIZE_BINNING_MAX] = {{0x80, 0x80, 0x80}, {0x84, 0x80, 0x84}};
static ccm_gain_t preview_gain ={0x84, 0x80, 0x84};

#define OV8850_AE_TH_HIGH	481120 /* 496640 1940 * 0x80 *2, max AE_value in 720P@30fps, binning should x2 */
#define OV8850_AE_TH_LOW	62000 /* 62000 is 4band(1vts)@0x10 gain, binning should x2. */

#define CCM_GAIN_ORI 	0x80
#define R_GAIN_SHIFT	0x85
#define B_GAIN_SHIFT	0x85

#define R_GAIN_MAX	0xb0
#define B_GAIN_MIN	0x60

static void ov8850_awb_dynamic_ccm_gain(u32 frame_index, u32 ae, awb_gain_t  *awb_gain, ccm_gain_t *ccm_gain)
{
	u32 gain_delta;

	u32 ae_value = ae;
	u32 current_rbratio = 0x100;
	bool binning;

	u32 a_light_rbratio = 0x50;
	u32 h_light_rbratio =0x30;

	u32 r_gain_base = CCM_GAIN_ORI;
	u32 b_gain_base = CCM_GAIN_ORI;

	if (frame_index >= ARRAY_SIZE(ov8850_framesizes)) {
		print_error("Unsupport sensor setting index: %d", frame_index);
		return;
	}

	binning = ov8850_framesizes[frame_index].binning;

	if ((awb_gain->b_gain != 0) && (awb_gain->r_gain != 0)) {
		current_rbratio = 0x8000 / (awb_gain->b_gain * 0x100 / awb_gain->r_gain);
	}

	if (binning == true) {
		ae_value *= 2;
		r_gain_base = R_GAIN_SHIFT;
		b_gain_base = B_GAIN_SHIFT;
	}

	/* adjust current ae value */
	ae_value = (ae_value > OV8850_AE_TH_HIGH) ? OV8850_AE_TH_HIGH : ae_value;

	if ((current_rbratio >= a_light_rbratio) || (ae_value < OV8850_AE_TH_LOW)) {
	/* (1) R/B larger than A' light or expo_line*GAIN lower than TH_LOW */
			ccm_gain->r_gain = r_gain_base;
			ccm_gain->b_gain = b_gain_base;
	} else if ((current_rbratio < h_light_rbratio) && (ae_value >= OV8850_AE_TH_LOW)) {
	/* (2)R/B lower than H light and expo_line*GAIN larger than TH_LOW*/
		gain_delta = (ae_value -OV8850_AE_TH_LOW) * (R_GAIN_MAX - r_gain_base) / (OV8850_AE_TH_HIGH - OV8850_AE_TH_LOW);
		ccm_gain->r_gain = gain_delta + r_gain_base;

		gain_delta = (ae_value -OV8850_AE_TH_LOW) * (b_gain_base - B_GAIN_MIN) / (OV8850_AE_TH_HIGH - OV8850_AE_TH_LOW);
		ccm_gain->b_gain = b_gain_base - gain_delta;
 	} else if ((current_rbratio <= a_light_rbratio) && (current_rbratio >= h_light_rbratio)) {
	/* (3)R/B is between A' light and H light*/
		gain_delta = (a_light_rbratio - current_rbratio) * (R_GAIN_MAX -r_gain_base) /(a_light_rbratio -h_light_rbratio);
		ccm_gain->r_gain = gain_delta * (ae_value -OV8850_AE_TH_LOW) / (OV8850_AE_TH_HIGH - OV8850_AE_TH_LOW) + r_gain_base;

		gain_delta = (a_light_rbratio - current_rbratio) * (b_gain_base - B_GAIN_MIN) /(a_light_rbratio -h_light_rbratio);
		ccm_gain->b_gain = b_gain_base - gain_delta * (ae_value -OV8850_AE_TH_LOW) / (OV8850_AE_TH_HIGH - OV8850_AE_TH_LOW);
	} else {
		print_info("NO COVERRING SCENE.\n");
		return;
	}

	//for preview
	preview_gain.r_gain = ccm_gain->r_gain;
	preview_gain.b_gain = ccm_gain->b_gain;

	print_debug("CURRENT SCENE : r_gain = 0x%x, b_gain = 0x%x, current_value = %d\n\n",
		ccm_gain->r_gain, ccm_gain->b_gain, ae_value);

	//for capture
	if (current_rbratio >= a_light_rbratio) {
		cap_gain[FRAMESIZE_NOBINNING].r_gain = CCM_GAIN_ORI;
		cap_gain[FRAMESIZE_NOBINNING].b_gain = CCM_GAIN_ORI;
		cap_gain[FRAMESIZE_BINNING].r_gain = R_GAIN_SHIFT;
		cap_gain[FRAMESIZE_BINNING].b_gain = B_GAIN_SHIFT;
	} else {
		/* check preview size is binning or not. */
		if (binning == true) {
			cap_gain[FRAMESIZE_NOBINNING].r_gain = ccm_gain->r_gain - (R_GAIN_SHIFT - CCM_GAIN_ORI);
			cap_gain[FRAMESIZE_NOBINNING].b_gain = ccm_gain->b_gain - (B_GAIN_SHIFT - CCM_GAIN_ORI);
			cap_gain[FRAMESIZE_BINNING].r_gain = ccm_gain->r_gain;
			cap_gain[FRAMESIZE_BINNING].b_gain = ccm_gain->b_gain;
		} else {
			cap_gain[FRAMESIZE_NOBINNING].r_gain = ccm_gain->r_gain;
			cap_gain[FRAMESIZE_NOBINNING].b_gain = ccm_gain->b_gain;
			cap_gain[FRAMESIZE_BINNING].r_gain = ccm_gain->r_gain + (R_GAIN_SHIFT - CCM_GAIN_ORI);
			cap_gain[FRAMESIZE_BINNING].b_gain = ccm_gain->b_gain + (B_GAIN_SHIFT - CCM_GAIN_ORI);
		}
	}

	print_debug("cap_gain[NOBINNING] = 0x%x, 0x%x; cap_gain[BINNING] = 0x%x, 0x%x\n",
		cap_gain[FRAMESIZE_NOBINNING].r_gain, cap_gain[FRAMESIZE_NOBINNING].b_gain,
		cap_gain[FRAMESIZE_BINNING].r_gain, cap_gain[FRAMESIZE_BINNING].b_gain);
}

static void ov8850_get_ccm_pregain(camera_state state, u32 frame_index, u8 *bgain, u8 *rgain)
{
	bool binning;

	if (state >= STATE_MAX)
		return;

	if (frame_index >= ARRAY_SIZE(ov8850_framesizes)) {
		print_error("Unsupport sensor setting index: %d", frame_index);
		return;
	}

	binning = ov8850_framesizes[frame_index].binning;

	if (state == STATE_PREVIEW) {
		*bgain = preview_gain.b_gain;
		*rgain = preview_gain.r_gain;
	} else if (state == STATE_CAPTURE) {
		/* check capture size is binning or not. */
		if (binning == true) {
			*bgain = cap_gain[FRAMESIZE_BINNING].b_gain;
			*rgain = cap_gain[FRAMESIZE_BINNING].r_gain;
		} else {
			*bgain = cap_gain[FRAMESIZE_NOBINNING].b_gain;
			*rgain = cap_gain[FRAMESIZE_NOBINNING].r_gain;
		}
	} else
		return;
}

/*  **************************************************************************
* FunctionName: ov8850_check_sensor;
* Description : NA;
* Input       : NA;
* Output      : NA;
* ReturnValue : NA;
* Other       : NA;
***************************************************************************/
static int ov8850_check_sensor(void)
{
	u8 regl = 0;
	u8 regh = 0;
	u16 id = 0;
	int size;
	int i;

	//1  1.check sensor id.
	ov8850_read_reg(0x300A, &regh);
	ov8850_read_reg(0x300B, &regl);

	id = ((regh << 8) | regl);
	print_info("ov8850 product id:0x%x", id);
	if (OV8850_CHIP_ID != id) {
		print_error("Invalid product id ,Could not load sensor ov8850");
		return -ENODEV;
	}

	//1 2.check otp info and vendor id.
	/* Frist should out LP11 */
	ov8850_write_reg(0x0100, 0x01, 0x00);

	for (i = 2; i >=0; i--) {
		if (true == ov8850_check_otp_group(i)) {
			ov8850_read_otp_vcm(i, &ov8850_vcm_start, &ov8850_vcm_end);
			print_info("Successfully read ov8850 otp_index %d, vcm start 0x%x, vcm end 0x%x",
				i, ov8850_vcm_start, ov8850_vcm_end);
			ov8850_otp_index = i;
			break;
		}
	}
	if(-1 == ov8850_otp_index)
		print_info("have no any otp data");
	else
		print_info("ov8850_otp_index=%d, factory_id=%d", ov8850_otp_index, factory_id);

	//1 3.config vcm info according to vendor id.
 	if (OV8850_FOXCONN_FACTORY_ID == factory_id) {
		memcpy(ov8850_sensor.vcm, &vcm_dw9714_ov8850_f, sizeof(vcm_info_s));
	} else {
		memcpy(ov8850_sensor.vcm, &vcm_dw9714_ov8850_f, sizeof(vcm_info_s));
	}
	ov8850_sensor.vcm->get_vcm_otp = ov8850_get_vcm_otp;
	/* then enter LP11 again */
	ov8850_write_reg(0x0100, 0x00, 0x00);

	//1 4.write isp sequence according to vendor id.
#ifndef OVISP_DEBUG_MODE
	if (OV8850_FOXCONN_FACTORY_ID == factory_id) {
		size = ARRAY_SIZE(isp_init_regs_ov8850_foxconn);
		ov8850_write_isp_seq(isp_init_regs_ov8850_foxconn, size);
	} else {
		size = ARRAY_SIZE(isp_init_regs_ov8850_foxconn);
		ov8850_write_isp_seq(isp_init_regs_ov8850_foxconn, size);
	}
#endif

	return 0;
}
/****************************************************************************
* FunctionName: ov8850_check_sensor;
* Description : NA;
* Input       : NA;
* Output      : NA;
* ReturnValue : NA;
* Other       : NA;
***************************************************************************/
int ov8850_power(camera_power_state power)
{
	int ret = 0;
	print_debug("enter %s", __func__);
	if (power == POWER_ON) {

		k3_ispldo_power_sensor(power, "pri-cameralog-vcc");	//avdd-ldo19

		k3_ispldo_power_sensor(power, "camera-vcc");		//iovdd-ldo18


		k3_ispgpio_power_sensor(&ov8850_sensor, power);	//pwdn and af vcm

		k3_ispio_ioconfig(&ov8850_sensor, power);			//isp i2c, rst, 

		ret = camera_power_core_ldo(power);				//core ldo, gpio073,

		k3_ispldo_power_sensor(power, "cameravcm-vcc");	//af vdd-ldo20

		k3_ispldo_power_sensor(power, "sec-cameralog-vcc");//scam avdd-ldo11
		msleep(2); 
	} else {
		k3_ispio_deinit_csi(ov8850_sensor.mipi_index);

		k3_ispldo_power_sensor(power, "sec-cameralog-vcc");
		k3_ispldo_power_sensor(power, "cameravcm-vcc");
		camera_power_core_ldo(power);
		udelay(200);

		k3_ispio_ioconfig(&ov8850_sensor, power);
		k3_ispgpio_power_sensor(&ov8850_sensor, power);

		k3_ispldo_power_sensor(power, "camera-vcc");
		udelay(10);
		k3_ispldo_power_sensor(power, "pri-cameralog-vcc");
	}
	return ret;
}
/*
 * Here gain is in unit 1/16 of sensor gain,
 * y36721 todo, temporarily if sensor gain=0x10, ISO is 100
 * in fact we need calibrate an ISO-ET-gain table.
 */
u32 ov8850_gain_to_iso(int gain)
{
	return (gain * 100) / 0x10;
}

u32 ov8850_iso_to_gain(int iso)
{
	return (iso * 0x10) / 100;
}

void ov8850_set_gain(u32 gain)
{
	ov8850_write_reg(OV8850_GAIN_REG_H, (gain >> 8) & 0xff, 0x00);
	ov8850_write_reg(OV8850_GAIN_REG_L, gain & 0xff, 0x00);
}

u32 ov8850_get_gain(void)
{
	u8 gain_h = 0;
	u8 gain_l = 0;
	u32 gain;
	ov8850_read_reg(OV8850_GAIN_REG_H, &gain_h);
	ov8850_read_reg(OV8850_GAIN_REG_L, &gain_l);
	gain = (gain_h << 8) | gain_l;
	return gain;
}

static int ov8850_get_af_param(camera_af_param_t type)
{
	int ret;

	switch(type){
	case AF_MIN_HEIGHT_RATIO:
		ret = OV8850_AF_MIN_HEIGHT_RATIO;
		break;
	case AF_MAX_FOCUS_STEP:
		ret = OV8850_AF_MAX_FOCUS_STEP;
		break;
	case AF_GSENSOR_INTERVAL_THRESHOLD:
		ret = OV8850_AF_GSENSOR_INTERVAL_THRESHOLD;
		break;
	case AF_WIDTH_PERCENT:
		ret = OV8850_AF_WIDTH_PERCENT;
		break;
	case AF_HEIGHT_PERCENT:
		ret = OV8850_AF_HEIGHT_PERCENT;
		break;
	default:
		print_error("%s:invalid argument",__func__);
		ret = -1;
		break;
	}

	return ret;
}

/*delete ov8850_get_iso_limit func, replace by ov8850_get_override_param*/
static u32 ov8850_get_override_param(camera_override_type_t type)
{
	u32 ret_val = sensor_override_params[type];

	switch (type) {
	case OVERRIDE_ISO_HIGH:
		ret_val = OV8850_MAX_ISO;
		break;

	case OVERRIDE_ISO_LOW:
		ret_val = OV8850_MIN_ISO;
		break;

	case OVERRIDE_AUTO_FPS_GAIN_HIGH:
		ret_val = OV8850_AUTO_FPS_MAX_GAIN;
		break;

	case OVERRIDE_AUTO_FPS_GAIN_LOW:
		ret_val = OV8850_AUTO_FPS_MIN_GAIN;
		break;

	case OVERRIDE_FPS_MAX:
		ret_val = OV8850_MAX_FRAMERATE;
		break;

	case OVERRIDE_FPS_MIN:
		ret_val = OV8850_MIN_FRAMERATE;
		break;

	case OVERRIDE_CAP_FPS_MIN:
		ret_val = OV8850_MIN_CAP_FRAMERATE;
		break;

	case OVERRIDE_FLASH_TRIGGER_GAIN:
		ret_val = OV8850_FLASH_TRIGGER_GAIN;
		break;

	case OVERRIDE_SHARPNESS_PREVIEW:
		ret_val = OV8850_SHARPNESS_PREVIEW;
		break;

	case OVERRIDE_SHARPNESS_CAPTURE:
		ret_val = OV8850_SHARPNESS_CAPTURE;
		break;

	default:
		print_error("%s:not override or invalid type %d, use default",__func__, type);
	}

	return ret_val;
}

u32 ov8850_get_vts_reg_addr(void)
{
	return OV8850_VTS_REG_H;
}

void ov8850_set_exposure(u32 exposure)
{
	ov8850_write_reg(OV8850_EXPOSURE_REG_0, (exposure >> 16) & 0x0f, 0x00);
	ov8850_write_reg(OV8850_EXPOSURE_REG_1, (exposure >> 8) & 0xff, 0x00);
	ov8850_write_reg(OV8850_EXPOSURE_REG_2, exposure & 0xf0, 0x00);	/*fraction part not used */
}

u32 ov8850_get_exposure(void)
{
	u8 exposure[3] = { 0, 0, 0 };
	u32 expo;

	ov8850_read_reg(OV8850_EXPOSURE_REG_0, &exposure[0]);
	ov8850_read_reg(OV8850_EXPOSURE_REG_1, &exposure[1]);
	ov8850_read_reg(OV8850_EXPOSURE_REG_2, &exposure[2]);
	expo = (exposure[0] << 16) | (exposure[1] << 8) | exposure[2];

	return expo;
}

/*
 **************************************************************************
 * FunctionName: ov8850_reset;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static int ov8850_reset(camera_power_state power_state)
{
	print_debug("enter %s", __func__);

	if (POWER_ON == power_state) {
		k3_isp_io_enable_mclk(MCLK_ENABLE, ov8850_sensor.sensor_index);
		k3_ispgpio_reset_sensor(ov8850_sensor.sensor_index, power_state,
					ov8850_sensor.power_conf.reset_valid);
		
	} else {
		k3_ispgpio_reset_sensor(ov8850_sensor.sensor_index, power_state,
					ov8850_sensor.power_conf.reset_valid);
		udelay(20);
		k3_isp_io_enable_mclk(MCLK_DISABLE, ov8850_sensor.sensor_index);
	}

	return 0;
}

/*
 **************************************************************************
 * FunctionName: ov8850_init;
 * Description : ov8850 init function;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : Error code indicating success or failure;
 * Other       : NA;
 **************************************************************************
*/
static int ov8850_init(void)
{
	static bool ov8850_check = false;
	print_debug("enter %s", __func__);

	if (false == ov8850_check)
	{
		if (check_suspensory_camera("OV8850") != 1)
		{
			return -ENODEV;
		}
		ov8850_check = true;
	}

	if (!camera_timing_is_match(1)) {
		print_error("%s: sensor timing don't match.\n", __func__);
		return -ENODEV;
	}

	if (ov8850_sensor.owner && !try_module_get(ov8850_sensor.owner)) {
		print_error("%s: try_module_get fail", __func__);
		return -ENOENT;
	}

	k3_ispio_power_init("pri-cameralog-vcc", LDO_VOLTAGE_28V, LDO_VOLTAGE_28V);	/*analog 2.85V */
	k3_ispio_power_init("camera-vcc", LDO_VOLTAGE_18V, LDO_VOLTAGE_18V);		/*IO 1.8V */
	k3_ispio_power_init("cameravcm-vcc", LDO_VOLTAGE_28V, LDO_VOLTAGE_28V);	/*AF 2.85V */
	k3_ispio_power_init("sec-cameralog-vcc", LDO_VOLTAGE_28V, LDO_VOLTAGE_28V);	/*analog 2.85V */

	print_debug("exit %s", __func__);
	return 0;
}

/*
 **************************************************************************
 * FunctionName: ov8850_exit;
 * Description : ov8850 exit function;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static void ov8850_exit(void)
{
	print_debug("enter %s", __func__);

	k3_ispio_power_deinit();

	if (ov8850_sensor.owner) {
		module_put(ov8850_sensor.owner);
	}
	print_debug("exit %s", __func__);
}

/*
 **************************************************************************
 * FunctionName: ov8850_shut_down;
 * Description : ov8850 shut down function;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static void ov8850_shut_down(void)
{
	print_debug("enter %s", __func__);
	k3_ispgpio_power_sensor(&ov8850_sensor, POWER_OFF);
}

/*
 **************************************************************************
 * FunctionName: ov8850_check_otp_group;
 * Description : check whether the group of index is valid;
 * Input       : index of otp group. (0, 1, 2);
 * Output      : 0, group of index is empty;
 *				 1, group of index has valid data;
 *				-1, unexpected error;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
int ov8850_check_otp_group(u16 index)
{
	u8 temp1 = 0;
	u8 temp2 = 0;
	u8 temp3 = 0;
	u8 i;
	u8 bank;

	switch (index) {
		case 0:
			bank = 1;
			break;
		case 1:
			bank = 6;
			break;
		case 2:
			bank = 11;
			break;
		default:
			return -1;
	}

	/* select bank  */
	ov8850_write_reg(0x3d84, 0xc0 | bank, 0x00);
	/* load otp to buffer */
	ov8850_write_reg(0x3d81, 0x01, 0x00);
	/* delay 10ms if needed */
	mdelay(1);
	/* disable otp read */
	ov8850_write_reg(0x3d81, 0x00, 0x00);

	ov8850_read_reg(0x3d06, &temp1);
	ov8850_read_reg(0x3d08, &temp2);
	ov8850_read_reg(0x3d07, &temp3);
	factory_id = (temp3&0xF0)>>4;
	
	if (temp1 | temp2)
		return true;
	else
		return false;
}
/*
 **************************************************************************
 * FunctionName: ov8850_read_otp_mi_wb_vcm;
 * Description : read out calibration datas of module info, wb and vcm;
 * Input       : index of otp group. (0, 1, 2);
 * Output      : true, ok;
 *				 false, unexpected error;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
void ov8850_read_otp_mi_wb_vcm(u16 index, struct otp_struct *potp)
{
	u8 i;
	u8 bank;

	switch (index) {
		case 0:
			bank = 1;
			break;
		case 1:
			bank = 6;
			break;
		case 2:
			bank = 11;
			break;
		default:
			return;
	}

	/* select bank */
	ov8850_write_reg(0x3d84, 0xc0 | bank, 0x00);
	/* load otp to buffer */
	ov8850_write_reg(0x3d81, 0x01, 0x00);
	/* delay 10ms if needed */
	mdelay(1);
	/* disable otp read */
	ov8850_write_reg(0x3d81, 0x00, 0x00);

	//ov8850_read_reg(0x3d00, &(potp->year));
	//ov8850_read_reg(0x3d01, &(potp->month));
	//ov8850_read_reg(0x3d02, &(potp->day));
	//ov8850_read_reg(0x3d03, &(potp->user_id_1));
	//ov8850_read_reg(0x3d04, &(potp->user_id_2));
	//ov8850_read_reg(0x3d05, &(potp->user_id_3));
	//ov8850_read_reg(0x3d06, &(potp->cam_code));
	ov8850_read_reg(0x3d07, &(potp->vendor_version));
	//ov8850_read_reg(0x3d08, &(potp->rg_h));
	//ov8850_read_reg(0x3d09, &(potp->rg_l));
	//ov8850_read_reg(0x3d0a, &(potp->bg_h));
	//ov8850_read_reg(0x3d0b, &(potp->bg_l));
	//ov8850_read_reg(0x3d0c, &(potp->gbgr_h));
	//ov8850_read_reg(0x3d0d, &(potp->gbgr_l));
	//ov8850_read_reg(0x3d0e, &(potp->start_curr));
	//ov8850_read_reg(0x3d0f, &(potp->end_curr));
}

/*
 **************************************************************************
 * FunctionName: ov8850_read_otp_wb;
 * Description : read out calibration datas of wb;
 * Input       : index of otp group. (0, 1, 2);
 * Output      : true, ok;
 *				 false, unexpected error;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
bool ov8850_read_otp_wb(u16 index, struct otp_struct *potp)
{
	u8 i;
	u8 bank;

	switch (index) {
		case 0:
			bank = 1;
			break;
		case 1:
			bank = 6;
			break;
		case 2:
			bank = 11;
			break;
		default:
			return false;
	}

	/* select bank */
	ov8850_write_reg(0x3d84, 0xc0 | bank, 0x00);
	/* load otp to buffer */
	ov8850_write_reg(0x3d81, 0x01, 0x00);
	/* delay 10ms if needed */
	mdelay(1);
	/* disable otp read */
	ov8850_write_reg(0x3d81, 0x00, 0x00);

	ov8850_read_reg(0x3d08, &(potp->rg_h));
	ov8850_read_reg(0x3d09, &(potp->rg_l));
	ov8850_read_reg(0x3d0a, &(potp->bg_h));
	ov8850_read_reg(0x3d0b, &(potp->bg_l));
#if 0	//ov8850 have no light-source calibration parameters
	//select bank 5
	ov8850_write_reg(0x3d84, 0xc0 | (bank + 4), 0x00);
	// load otp to buffer
	ov8850_write_reg(0x3d81, 0x01, 0x00);
	mdelay(1);
	// disable otp read
	ov8850_write_reg(0x3d81, 0x00, 0x00);
	ov8850_read_reg(0x3d0e, &(potp->rg_coff));
	ov8850_read_reg(0x3d0f, &(potp->bg_coff));
#endif
	return true;
}

bool ov8850_read_otp_vcm(u16 index, u8 *vcm_start, u8 *vcm_end)
{
	u8 bank;

	switch (index) {
		case 0:
			bank = 1;
			break;
		case 1:
			bank = 6;
			break;
		case 2:
			bank = 11;
			break;
		default:
			return false;
	}

	/* select bank */
	ov8850_write_reg(0x3d84, 0xc0 | bank, 0x00);
	/* load otp to buffer */
	ov8850_write_reg(0x3d81, 0x01, 0x00);
	/* delay 10ms if needed */
	mdelay(1);
	/* disable otp read */
	ov8850_write_reg(0x3d81, 0x00, 0x00);

	ov8850_read_reg(0x3d0e, vcm_start);
	ov8850_read_reg(0x3d0f, vcm_end);

	return true;
}

/*
 **************************************************************************
 * FunctionName: ov8850_read_otp_lenc;
 * Description : read out calibration datas of lenc;
 * Input       : index of otp group. (0, 1, 2);
 * Output      : true, ok;
 *				 false, unexpected error;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
bool ov8850_read_otp_lenc(u16 index, struct otp_struct *potp)
{
	u8 i, bank;
	u8 otp_part;
	switch (index) {
		case 0:
			bank = 2;
			break;
		case 1:
			bank = 7;
			break;
		case 2:
			bank = 12;
			break;
		default:
			return false;
	}

	/*
	 * read out part 1
	 * clear otp buffer
	 */
	for (otp_part = 0; otp_part < 4; otp_part++) {
		/* select bank */
		ov8850_write_reg(0x3d84, 0xc0 | (bank + otp_part), 0x00);
		/* load otp to buffer */
		ov8850_write_reg(0x3d81, 0x01, 0x00);
		/* delay 10ms if needed */
		mdelay(1);
		/* disable otp read */
		ov8850_write_reg(0x3d81, 0x00, 0x00);

		if (otp_part < 3) {
			for (i = 0; i < 16; i++)
				ov8850_read_reg(OTP_BUFFER_START_ADDRESS + i, &(potp->lenc[i + 16 * otp_part]));
		} else {
			for (i = 0; i < 14; i++)
				ov8850_read_reg(OTP_BUFFER_START_ADDRESS + i, &(potp->lenc[i + 16 * otp_part]));
		}
	}

	return true;
}

/*
 **************************************************************************
 * FunctionName: ov8850_update_wb_gain;
 * Description : NA;
 * Input       : R_gain: red gain of sensor MWB, 0x400 = 1;
 *				 G_gain: green gain of sensor MWB, 0x400 = 1;
 *				 B_gain: blue gain of sensor MWB, 0x400 = 1;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
void ov8850_update_wb_gain(u16 R_gain, u16 G_gain, u16 B_gain)
{
	u8 temp = 0;
	if (R_gain > 0x400) {
		ov8850_write_reg(0x3400, R_gain>>8, 0x00);
		ov8850_write_reg(0x3401, R_gain & 0x00ff, 0x00);
	}

	if (G_gain > 0x400) {
		ov8850_write_reg(0x3402, G_gain>>8, 0x00);
		ov8850_write_reg(0x3403, G_gain & 0x00ff, 0x00);
	}

	if (B_gain > 0x400) {
		ov8850_write_reg(0x3404, B_gain>>8, 0x00);
		ov8850_write_reg(0x3405, B_gain & 0x00ff, 0x00);
	}

	/* enable mwb */
	ov8850_read_reg(0x3406, &temp);
	ov8850_write_reg(0x3406, temp|0x01, 0x00);
}

void ov8850_update_lenc(unsigned char *plenc)
{
	u16 i;
	u8 temp = 0;

	for(i=0; i<62; i++) {
		ov8850_write_reg(0x5800+i, *(plenc+i), 0x00);
	}
	/*enable LenC*/
	ov8850_read_reg(0x5000, &temp);
	ov8850_write_reg(0x5000, temp|0x80, 0x00);
}

/*
 **************************************************************************
 * FunctionName: ov8850_update_otp_wb;
 * Description : R/G and B/G of typical camera module is defined here,
 * 			  	 call this function after ov8850 initialization to calibrate white balance;
 * Input       : otp_index;
 * Output      : NA;
 * ReturnValue : 1 update success;0, no OTP;-1, unexpected error
 * Other       : last modified j00212990;
 **************************************************************************
*/
void ov8850_update_otp_wb(u16 otp_index)
{
	/* sensor method */

	struct otp_struct current_otp;

	u32 rg_ratio, bg_ratio;
	u32 R_gain, G_gain, B_gain, G_gain_R, G_gain_B;

	u32 RG_Ratio_typical; //RG_TYPICAL;
	u32 BG_Ratio_typical; //BG_TYPICAL;

	if (factory_id == OV8850_FOXCONN_FACTORY_ID) 
	{
		/*Foxconn typical code according to mass-production*/
		/*golden sample code:1165#,foxconn told this value is typical*/
		RG_Ratio_typical = 0X22F;//559;
		BG_Ratio_typical = 0X2B1;//689;
	}
	else
	{
		RG_Ratio_typical = 0x267;
		BG_Ratio_typical = 0x261;
	}

	/* R/G and B/G of current camera module is read out from sensor OTP */
	ov8850_read_otp_wb(otp_index, &current_otp);
	rg_ratio = ((current_otp.rg_h << 8) & 0xff00) | (current_otp.rg_l & 0xff);/* OTP_RG' */
	bg_ratio = ((current_otp.bg_h << 8) & 0xff00) | (current_otp.bg_l & 0xff);/* OTP_BG' */
	print_debug("%s:read from otp rg_ratio=0x%x, bg_ratio=0x%x", __func__, rg_ratio, bg_ratio);
#if 0	//ov8850 have no light-source calibration parameters
	if (current_otp.rg_coff != 0)
		rg_ratio = rg_ratio * (current_otp.rg_coff + 128) / 256;

	if (current_otp.bg_coff != 0)
		bg_ratio = bg_ratio * (current_otp.bg_coff + 128) / 256;
#endif
	//calculate gain
	//0x400 = 1x gain
	if (bg_ratio < BG_Ratio_typical) {
		if (rg_ratio < RG_Ratio_typical) {
			// bg_ratio < BG_Ratio_typical &&
			// rg_ratio < RG_Ratio_typical
			G_gain = 0x400;
			B_gain = 0x400 * BG_Ratio_typical / bg_ratio;
			R_gain = 0x400 * RG_Ratio_typical / rg_ratio;
		} else {
			// bg_ratio < BG_Ratio_typical &&
			// rg_ratio >= RG_Ratio_typical
			R_gain = 0x400;
			G_gain = 0x400 * rg_ratio / RG_Ratio_typical;
			B_gain = G_gain * BG_Ratio_typical / bg_ratio;
		}
	} else {
		if (rg_ratio < RG_Ratio_typical) {
			// bg_ratio >= BG_Ratio_typical &&
			// rg_ratio < RG_Ratio_typical
			B_gain = 0x400;
			G_gain = 0x400 * bg_ratio / BG_Ratio_typical;
			R_gain = G_gain * RG_Ratio_typical / rg_ratio;
		} else {
			// bg_ratio >= BG_Ratio_typical &&
			// rg_ratio >= RG_Ratio_typical
			G_gain_B = 0x400 * bg_ratio / BG_Ratio_typical;
			G_gain_R = 0x400 * rg_ratio / RG_Ratio_typical;

			if (G_gain_B > G_gain_R) {
				B_gain = 0x400;
				G_gain = G_gain_B;
				R_gain = G_gain * RG_Ratio_typical / rg_ratio;
			} else {
				R_gain = 0x400;
				G_gain = G_gain_R;
				B_gain = G_gain * BG_Ratio_typical / bg_ratio;
			}
    		}
	}
	print_debug("%s:R_gain=0x%x, G_gain=0x%x, B_gain=0x%x", __func__, R_gain, G_gain, B_gain);
	// write sensor wb gain to registers
	ov8850_update_wb_gain(R_gain, G_gain, B_gain);
}

/*
 **************************************************************************
 * FunctionName: ov8850_update_otp_lenc;
 * Description : call this function after ov8850 initialization to calibrate lens correction;
 * Input       : R_gain: red gain of sensor MWB, 0x400 = 1;
 *				 G_gain: green gain of sensor MWB, 0x400 = 1;
 *				 B_gain: blue gain of sensor MWB, 0x400 = 1;
 * Output      : NA;
 * ReturnValue : 1 update success
 *				 0, no OTP
 *				 -1, unexpected error;
 * Other       : NA;
 **************************************************************************
*/
void ov8850_update_otp_lenc(u16 otp_index)
{
	struct otp_struct current_otp;

	print_debug("Enter function %s", __func__);
	ov8850_read_otp_lenc(otp_index, &current_otp);

	ov8850_update_lenc(current_otp.lenc);
}

void ov8850_update_blc(void)
{
	u8 blc_check = 0;
	u8 bank = 0x1f;	//blc info is in bank31
	u8 temp;
	/* clear otp buffer */
	ov8850_write_reg(0x3d0a, 0x00, 0x00);
	ov8850_write_reg(0x3d0b, 0x00, 0x00);

	ov8850_write_reg(0x3d84, 0xc0|bank, 0x00);
	ov8850_write_reg(0x3d81, 0x01, 0x00);
	mdelay(1);
	ov8850_read_reg(0x3d0b, &blc_check);
	ov8850_read_reg(0x4000, &temp);
	if (0 == blc_check) {
		temp |= 0x08;//manual load mode
		ov8850_read_reg(0x3d0a, &blc_check);
		mdelay(1);
		print_debug("%s, blc not calculated 0x3d0a = %x", __func__, blc_check);
		ov8850_write_reg(0x4006, blc_check, 0x00);
	} else {
		temp &= 0xf7;//auto load mode
		print_debug("%s, blc calculated ", __func__);
	}
	
	//enable blc ratio auto/manual mode
	ov8850_write_reg(0x4000, temp, 0x00);
	mdelay(1);
	//enable blc ratio
	ov8850_read_reg(0x4008, &temp);
	temp &= 0xfb;
	ov8850_write_reg(0x4008, temp, 0x00);
	//set blc target
	ov8850_write_reg(0x4009, 0x10, 0x00);
}
#define OV8850_VCM_V2H_OFFSET	0xa0

static void ov8850_get_vcm_otp(u16 *vcm_start, u16 *vcm_end)
{
	if (ov8850_vcm_start > (ov8850_sensor.vcm->startCurrentOffset >> 2))
		*vcm_start = (ov8850_vcm_start << 2) - ov8850_sensor.vcm->startCurrentOffset;
	else
		*vcm_start = 0;

	//if (ov8850_vcm_end > (OV8850_VCM_V2H_OFFSET >> 2))
	//	*vcm_end = (ov8850_vcm_end << 2) - OV8850_VCM_V2H_OFFSET;
	//else
	//	*vcm_end = 0;
	*vcm_end = (ov8850_vcm_end << 2) ;
}

static int ov8850_get_sensor_aperture()
{
	print_info("enter %s", __func__);
	return OV8850_APERTURE_FACTOR;
}

static int ov8850_get_equivalent_focus()
{
	print_info("enter %s", __func__);
	return OV8850_EQUIVALENT_FOCUS;
}

/*
 **************************************************************************
 * FunctionName: ov8850_set_default;
 * Description : init ov8850_sensor;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
*/
static void ov8850_set_default(void)
{
	unsigned int chip_id;
	int i;
	ov8850_sensor.init = ov8850_init;
	ov8850_sensor.exit = ov8850_exit;
	
	ov8850_sensor.shut_down = ov8850_shut_down;
	
	ov8850_sensor.reset = ov8850_reset;
	
	ov8850_sensor.power = ov8850_power;
	
	ov8850_sensor.check_sensor = ov8850_check_sensor;
	ov8850_sensor.init_reg = ov8850_init_reg;
	//ov8850_sensor.init_isp_reg = ov8850_init_isp_reg;
	ov8850_sensor.stream_on = ov8850_stream_on;

	ov8850_sensor.set_flash = NULL;
	ov8850_sensor.get_flash = NULL;

	ov8850_sensor.set_scene = NULL;
	ov8850_sensor.get_scene = NULL;

	ov8850_sensor.set_hflip = ov8850_set_hflip;
	ov8850_sensor.get_hflip = ov8850_get_hflip;
	ov8850_sensor.set_vflip = ov8850_set_vflip;
	ov8850_sensor.get_vflip = ov8850_get_vflip;
	ov8850_sensor.update_flip = ov8850_update_flip;
	ov8850_sensor.hflip = 0;
	ov8850_sensor.vflip = 0;
	ov8850_sensor.old_flip = 0;

	ov8850_sensor.get_format = ov8850_get_format;

	ov8850_sensor.enum_framesizes = ov8850_enum_framesizes;
	ov8850_sensor.try_framesizes = ov8850_try_framesizes;
	ov8850_sensor.set_framesizes = ov8850_set_framesizes;
	ov8850_sensor.get_framesizes = ov8850_get_framesizes;

	ov8850_sensor.enum_frame_intervals = ov8850_enum_frame_intervals;
	ov8850_sensor.try_frame_intervals = NULL;
	ov8850_sensor.set_frame_intervals = NULL;
	ov8850_sensor.get_frame_intervals = NULL;

	ov8850_sensor.get_capability = ov8850_get_capability;
	ov8850_sensor.skip_frames = 1;
	ov8850_sensor.interface_type = MIPI1;
#ifdef MIPI_4LANE
	ov8850_sensor.mipi_lane_count = CSI_LINES_4;
#else
	ov8850_sensor.mipi_lane_count = CSI_LINES_2;
#endif
	ov8850_sensor.mipi_index = CSI_INDEX_0;
	ov8850_sensor.power_conf.pd_valid = LOW_VALID;
	ov8850_sensor.power_conf.reset_valid = LOW_VALID;
	ov8850_sensor.power_conf.vcmpd_valid = LOW_VALID;
	ov8850_sensor.sensor_index = CAMERA_SENSOR_PRIMARY;
	
	ov8850_sensor.i2c_config.index = I2C_PRIMARY;
	ov8850_sensor.i2c_config.speed = I2C_SPEED_400;
	ov8850_sensor.i2c_config.addr = OV8850_SLAVE_ADDRESS_6C;	//for FOXCONN module
	ov8850_sensor.i2c_config.addr_bits = 16;
	ov8850_sensor.i2c_config.val_bits = I2C_8BIT;

	ov8850_sensor.sensor_type = SENSOR_OV;
#ifdef OV8850_AP_WRITEAE_MODE /* just an example and test case for AP write AE mode */
	ov8850_sensor.aec_addr[0] = 0;
	ov8850_sensor.aec_addr[1] = 0;
	ov8850_sensor.aec_addr[2] = 0;
	ov8850_sensor.agc_addr[0] = 0;
	ov8850_sensor.agc_addr[1] = 0;
	ov8850_sensor.ap_writeAE_delay = 1500; /* 5 expo and gain registers, 1500us is enough */
#else
	ov8850_sensor.aec_addr[0] = OV8850_EXPOSURE_REG_0;
	ov8850_sensor.aec_addr[1] = OV8850_EXPOSURE_REG_1;
	ov8850_sensor.aec_addr[2] = OV8850_EXPOSURE_REG_2;
	ov8850_sensor.agc_addr[0] = OV8850_GAIN_REG_H;
	ov8850_sensor.agc_addr[1] = OV8850_GAIN_REG_L;
#endif
	ov8850_sensor.isp_location = CAMERA_USE_K3ISP;
	ov8850_sensor.sensor_tune_ops = NULL;

	ov8850_sensor.sensor_gain_to_iso = ov8850_gain_to_iso;
	ov8850_sensor.sensor_iso_to_gain = ov8850_iso_to_gain;
	ov8850_sensor.get_ccm_pregain = ov8850_get_ccm_pregain;
	ov8850_sensor.get_flash_awb = ov8850_get_flash_awb;

	ov8850_sensor.set_gain = ov8850_set_gain;
	ov8850_sensor.get_gain = ov8850_get_gain;
	ov8850_sensor.set_exposure = ov8850_set_exposure;
	ov8850_sensor.get_exposure = ov8850_get_exposure;
	//ov8850_sensor.set_awb_gain = ov8850_set_awb_gain;

	ov8850_sensor.set_vts = ov8850_set_vts;
	ov8850_sensor.get_vts_reg_addr = ov8850_get_vts_reg_addr;

	ov8850_sensor.set_effect = NULL;
	ov8850_sensor.set_awb = NULL;
	
	//ov8850_sensor.set_anti_banding = ov8850_set_anti_banding;

	//ov8850_sensor.update_framerate = ov8850_update_frame_rate;
	
#ifndef OVISP_DEBUG_MODE
	ov8850_sensor.awb_dynamic_ccm_gain = NULL;//ov8850_awb_dynamic_ccm_gain;
#else
	ov8850_sensor.awb_dynamic_ccm_gain = NULL;
#endif

	ov8850_sensor.get_sensor_aperture = ov8850_get_sensor_aperture;
	ov8850_sensor.get_equivalent_focus = ov8850_get_equivalent_focus;
	//ov8850_sensor.get_awb_offset	 = ov8850_get_awb_offset;
	ov8850_sensor.get_af_param = ov8850_get_af_param;
	ov8850_sensor.get_override_param = ov8850_get_override_param;
	ov8850_sensor.fmt[STATE_PREVIEW] = V4L2_PIX_FMT_RAW10;
	ov8850_sensor.fmt[STATE_CAPTURE] = V4L2_PIX_FMT_RAW10;
	ov8850_sensor.preview_frmsize_index = 0;
	ov8850_sensor.capture_frmsize_index = 1;
	ov8850_sensor.frmsize_list = ov8850_framesizes;

	//ov8850_sensor.pclk= ???;
	
	ov8850_sensor.fps_max = 30;	//??
	ov8850_sensor.fps_min = 15;	//??
	ov8850_sensor.fps = 24;		//??

	//ov8850_sensor.get_iso_limit = ov8850_get_iso_limit;

	/* auto focus parameters and image settings remove to function check_sensor() */
	ov8850_sensor.af_enable = 1;
	ov8850_sensor.vcm = &vcm_dw9714_ov8850_f;	//which may be changed in check_sensor

	//ov8850_sensor.image_setting.lensc_param = ov8850_lensc_param;
	//ov8850_sensor.image_setting.ccm_param = ov8850_ccm_param;
	//ov8850_sensor.image_setting.awb_param = ov8850_awb_param;

	ov8850_sensor.owner = THIS_MODULE;

	/* Following parameters should be revised according to sensor */
	ov8850_sensor.info.facing = CAMERA_FACING_BACK;
	ov8850_sensor.info.orientation = 270;
	ov8850_sensor.info.focal_length = 439;	/* 4.39mm */
	ov8850_sensor.info.h_view_angle = 63;	/*  66.1 degree */	//63.2
	ov8850_sensor.info.v_view_angle = 50;	//49.7
	strcpy(ov8850_sensor.info.name,"23060108FA-OV-F");
	ov8850_sensor.lane_clk = CLK_800M;	//CLK_800M
	/*delete codes for non-mass product chip*/
}

/*
 **************************************************************************
 * FunctionName: ov8850_module_init;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static __init int ov8850_module_init(void)
{
	ov8850_set_default();
	return register_camera_sensor(ov8850_sensor.sensor_index, &ov8850_sensor);
}

/*
 **************************************************************************
 * FunctionName: ov8850_module_exit;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static void __exit ov8850_module_exit(void)
{
	unregister_camera_sensor(ov8850_sensor.sensor_index, &ov8850_sensor);
}

MODULE_AUTHOR("Hisilicon");
module_init(ov8850_module_init);
module_exit(ov8850_module_exit);
MODULE_LICENSE("GPL");
/********************************** END **********************************************/
