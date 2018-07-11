/*
 * Driver for the imx230 camera sensor.
 *
 * Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
 *
 * Based on:
 * - the imx230 driver from QC msm-3.10 kernel on codeaurora.org:
 *   https://us.codeaurora.org/cgit/quic/la/kernel/msm-3.10/tree/drivers/
 *       media/platform/msm/camera_v2/sensor/imx230.c?h=LA.BR.1.2.4_rb1.41
 * - the imx230 driver posted on linux-media:
 *   https://www.mail-archive.com/linux-media%40vger.kernel.org/msg92671.html
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-ctrls.h>
//#include <media/v4l2-of.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

static DEFINE_MUTEX(imx230_lock);

#define IMX230_VOLTAGE_ANALOG               2800000
#define IMX230_VOLTAGE_DIGITAL_CORE         1500000
#define IMX230_VOLTAGE_DIGITAL_IO           1800000

#define IMX230_CHIP_ID_HIGH		0x0016
#define		IMX230_CHIP_ID_HIGH_BYTE	0x02
#define IMX230_CHIP_ID_LOW		0x0017
#define		IMX230_CHIP_ID_LOW_BYTE		0x30
#define IMX230_SYSTEM_CTRL0		0x3008
#define	IMX230_SYSTEM_CTRL0_STOP	0x42
#define IMX230_SC_MODE_SELECT		0x0100
#define IMX230_SC_MODE_SELECT_SW_STANDBY	0x00
#define IMX230_SC_MODE_SELECT_STREAMING		0x01

struct reg_value {
	u16 reg;
	u8 val;
};

struct imx230_mode_info {
	u32 width;
	u32 height;
	const struct reg_value *data;
	u32 data_size;
	u32 pixel_clock;
	u32 link_freq;
	u16 exposure_max;
	u16 exposure_def;
	struct v4l2_fract timeperframe;
};
/*
struct imx230_ctrls {
	struct v4l2_ctrl_handler handler;
	struct {
		struct v4l2_ctrl *auto_exp;
		struct v4l2_ctrl *exposure;
	};
	struct {
		struct v4l2_ctrl *auto_wb;
		struct v4l2_ctrl *blue_balance;
		struct v4l2_ctrl *red_balance;
	};
	struct {
		struct v4l2_ctrl *auto_gain;
		struct v4l2_ctrl *gain;
	};
	struct v4l2_ctrl *brightness;
	struct v4l2_ctrl *saturation;
	struct v4l2_ctrl *contrast;
	struct v4l2_ctrl *hue;
	struct v4l2_ctrl *test_pattern;
};
*/
struct imx230 {
	struct i2c_client *i2c_client;
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep;
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_rect crop;
	struct clk *xclk;

	struct regulator *io_regulator;
	struct regulator *core_regulator;
	struct regulator *analog_regulator;

	const struct imx230_mode_info *current_mode;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *pixel_clock;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;

//	struct imx230_ctrls imx230_ctrls;

	/* Cached register values */
	u8 aec_pk_manual;
	u8 pre_isp_00;
	u8 timing_format1;
	u8 timing_format2;
//	u8 timing_tc_reg20;
//	u8 timing_tc_reg21;

	struct mutex power_lock; /* lock to protect power state */
//	int power_count;
	bool power_on;

	struct gpio_desc *enable_gpio;
	struct gpio_desc *rst_gpio;
};

static inline struct imx230 *to_imx230(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx230, sd);
}

static const struct reg_value imx230_global_init_setting[] = {
  {0x0136, 0x18},
  {0x0137, 0x00},
  /* Global Setting */
  {0x4800, 0x0E},
  {0x4890, 0x01},
  {0x4D1E, 0x01},
  {0x4D1F, 0xFF},
  {0x4FA0, 0x00},
  {0x4FA1, 0x00},
  {0x4FA2, 0x00},
  {0x4FA3, 0x83},
  {0x6153, 0x01},
  {0x6156, 0x01},
  {0x69BB, 0x01},
  {0x69BC, 0x05},
  {0x69BD, 0x05},
  {0x69C1, 0x00},
  {0x69C4, 0x01},
  {0x69C6, 0x01},
  {0x7300, 0x00},
  {0x9009, 0x1A},
  {0xB040, 0x90},
  {0xB041, 0x14},
  {0xB042, 0x6B},
  {0xB043, 0x43},
  {0xB044, 0x63},
  {0xB045, 0x2A},
  {0xB046, 0x68},
  {0xB047, 0x06},
  {0xB048, 0x68},
  {0xB049, 0x07},
  {0xB04A, 0x68},
  {0xB04B, 0x04},
  {0xB04C, 0x68},
  {0xB04D, 0x05},
  {0xB04E, 0x68},
  {0xB04F, 0x16},
  {0xB050, 0x68},
  {0xB051, 0x17},
  {0xB052, 0x68},
  {0xB053, 0x74},
  {0xB054, 0x68},
  {0xB055, 0x75},
  {0xB056, 0x68},
  {0xB057, 0x76},
  {0xB058, 0x68},
  {0xB059, 0x77},
  {0xB05A, 0x68},
  {0xB05B, 0x7A},
  {0xB05C, 0x68},
  {0xB05D, 0x7B},
  {0xB05E, 0x68},
  {0xB05F, 0x0A},
  {0xB060, 0x68},
  {0xB061, 0x0B},
  {0xB062, 0x68},
  {0xB063, 0x08},
  {0xB064, 0x68},
  {0xB065, 0x09},
  {0xB066, 0x68},
  {0xB067, 0x0E},
  {0xB068, 0x68},
  {0xB069, 0x0F},
  {0xB06A, 0x68},
  {0xB06B, 0x0C},
  {0xB06C, 0x68},
  {0xB06D, 0x0D},
  {0xB06E, 0x68},
  {0xB06F, 0x13},
  {0xB070, 0x68},
  {0xB071, 0x12},
  {0xB072, 0x90},
  {0xB073, 0x0E},
  {0xD000, 0xDA},
  {0xD001, 0xDA},
  {0xD002, 0x7B},
  {0xD003, 0x00},
  {0xD004, 0x55},
  {0xD005, 0x34},
  {0xD006, 0x21},
  {0xD007, 0x00},
  {0xD008, 0x1C},
  {0xD009, 0x80},
  {0xD00A, 0xFE},
  {0xD00B, 0xC5},
  {0xD00C, 0x55},
  {0xD00D, 0xDC},
  {0xD00E, 0xB6},
  {0xD00F, 0x00},
  {0xD010, 0x31},
  {0xD011, 0x02},
  {0xD012, 0x4A},
  {0xD013, 0x0E},
 /* Load Setting */
  {0x5869, 0x01},
  /* DPC2D Setting */
  {0x6953, 0x01},
  {0x6962, 0x3A},
  {0x69CD, 0x3A},
  {0x9258, 0x00},
  {0x9906, 0x00},
  {0x9907, 0x28},
  {0x9976, 0x0A},
  {0x99B0, 0x20},
  {0x99B1, 0x20},
  {0x99B2, 0x20},
  {0x99C6, 0x6E},
  {0x99C7, 0x6E},
  {0x99C8, 0x6E},
  {0x9A1F, 0x0A},
  {0x9AB0, 0x20},
  {0x9AB1, 0x20},
  {0x9AB2, 0x20},
  {0x9AC6, 0x6E},
  {0x9AC7, 0x6E},
  {0x9AC8, 0x6E},
  {0x9B01, 0x08},
  {0x9B03, 0x1B},
  {0x9B05, 0x20},
  {0x9B07, 0x28},
  {0x9B08, 0x01},
  {0x9B09, 0x33},
  {0x9B0A, 0x01},
  {0x9B0B, 0x40},
  {0x9B13, 0x10},
  {0x9B15, 0x1D},
  {0x9B17, 0x20},
  {0x9B25, 0x60},
  {0x9B27, 0x60},
  {0x9B29, 0x60},
  {0x9B2B, 0x40},
  {0x9B2D, 0x40},
  {0x9B2F, 0x40},
  {0x9B37, 0x80},
  {0x9B39, 0x80},
  {0x9B3B, 0x80},
  {0x9B5D, 0x08},
  {0x9B5E, 0x0E},
  {0x9B60, 0x08},
  {0x9B61, 0x0E},
  {0x9B76, 0x0A},
  {0x9BB0, 0x20},
  {0x9BB1, 0x20},
  {0x9BB2, 0x20},
  {0x9BC6, 0x6E},
  {0x9BC7, 0x6E},
  {0x9BC8, 0x6E},
  {0x9BCC, 0x20},
  {0x9BCD, 0x20},
  {0x9BCE, 0x20},
  {0x9C01, 0x10},
  {0x9C03, 0x1D},
  {0x9C05, 0x20},
  {0x9C13, 0x10},
  {0x9C15, 0x10},
  {0x9C17, 0x10},
  {0x9C19, 0x04},
  {0x9C1B, 0x67},
  {0x9C1D, 0x80},
  {0x9C1F, 0x0A},
  {0x9C21, 0x29},
  {0x9C23, 0x32},
  {0x9C27, 0x56},
  {0x9C29, 0x60},
  {0x9C39, 0x67},
  {0x9C3B, 0x80},
  {0x9C3D, 0x80},
  {0x9C3F, 0x80},
  {0x9C41, 0x80},
  {0x9C55, 0xC8},
  {0x9C57, 0xC8},
  {0x9C59, 0xC8},
  {0x9C87, 0x48},
  {0x9C89, 0x48},
  {0x9C8B, 0x48},
  {0x9CB0, 0x20},
  {0x9CB1, 0x20},
  {0x9CB2, 0x20},
  {0x9CC6, 0x6E},
  {0x9CC7, 0x6E},
  {0x9CC8, 0x6E},
  {0x9D13, 0x10},
  {0x9D15, 0x10},
  {0x9D17, 0x10},
  {0x9D19, 0x04},
  {0x9D1B, 0x67},
  {0x9D1F, 0x0A},
  {0x9D21, 0x29},
  {0x9D23, 0x32},
  {0x9D55, 0xC8},
  {0x9D57, 0xC8},
  {0x9D59, 0xC8},
  {0x9D91, 0x20},
  {0x9D93, 0x20},
  {0x9D95, 0x20},
  {0x9E01, 0x10},
  {0x9E03, 0x1D},
  {0x9E13, 0x10},
  {0x9E15, 0x10},
  {0x9E17, 0x10},
  {0x9E19, 0x04},
  {0x9E1B, 0x67},
  {0x9E1D, 0x80},
  {0x9E1F, 0x0A},
  {0x9E21, 0x29},
  {0x9E23, 0x32},
  {0x9E25, 0x30},
  {0x9E27, 0x56},
  {0x9E29, 0x60},
  {0x9E39, 0x67},
  {0x9E3B, 0x80},
  {0x9E3D, 0x80},
  {0x9E3F, 0x80},
  {0x9E41, 0x80},
  {0x9E55, 0xC8},
  {0x9E57, 0xC8},
  {0x9E59, 0xC8},
  {0x9E91, 0x20},
  {0x9E93, 0x20},
  {0x9E95, 0x20},
  {0x9F8F, 0xA0},
  {0xA027, 0x67},
  {0xA029, 0x80},
  {0xA02D, 0x67},
  {0xA02F, 0x80},
  {0xA031, 0x80},
  {0xA033, 0x80},
  {0xA035, 0x80},
  {0xA037, 0x80},
  {0xA039, 0x80},
  {0xA03B, 0x80},
  {0xA067, 0x20},
  {0xA068, 0x20},
  {0xA069, 0x20},
  {0xA071, 0x48},
  {0xA073, 0x48},
  {0xA075, 0x48},
  {0xA08F, 0xA0},
  {0xA091, 0x3A},
  {0xA093, 0x3A},
  {0xA095, 0x0A},
  {0xA097, 0x0A},
  {0xA099, 0x0A},
  /* AE Setting */
  {0x9012, 0x00},
  {0x9098, 0x1A},
  {0x9099, 0x04},
  {0x909A, 0x20},
  {0x909B, 0x20},
  {0x909C, 0x13},
  {0x909D, 0x13},
  {0xA716, 0x13},
  {0xA801, 0x08},
  {0xA803, 0x0C},
  {0xA805, 0x10},
  {0xA806, 0x00},
  {0xA807, 0x18},
  {0xA808, 0x00},
  {0xA809, 0x20},
  {0xA80A, 0x00},
  {0xA80B, 0x30},
  {0xA80C, 0x00},
  {0xA80D, 0x40},
  {0xA80E, 0x00},
  {0xA80F, 0x60},
  {0xA810, 0x00},
  {0xA811, 0x80},
  {0xA812, 0x00},
  {0xA813, 0xC0},
  {0xA814, 0x01},
  {0xA815, 0x00},
  {0xA816, 0x01},
  {0xA817, 0x80},
  {0xA818, 0x02},
  {0xA819, 0x00},
  {0xA81A, 0x03},
  {0xA81B, 0x00},
  {0xA81C, 0x03},
  {0xA81D, 0xAC},
  {0xA838, 0x03},
  {0xA83C, 0x28},
  {0xA83D, 0x5F},
  {0xA881, 0x08},
  {0xA883, 0x0C},
  {0xA885, 0x10},
  {0xA886, 0x00},
  {0xA887, 0x18},
  {0xA888, 0x00},
  {0xA889, 0x20},
  {0xA88A, 0x00},
  {0xA88B, 0x30},
  {0xA88C, 0x00},
  {0xA88D, 0x40},
  {0xA88E, 0x00},
  {0xA88F, 0x60},
  {0xA890, 0x00},
  {0xA891, 0x80},
  {0xA892, 0x00},
  {0xA893, 0xC0},
  {0xA894, 0x01},
  {0xA895, 0x00},
  {0xA896, 0x01},
  {0xA897, 0x80},
  {0xA898, 0x02},
  {0xA899, 0x00},
  {0xA89A, 0x03},
  {0xA89B, 0x00},
  {0xA89C, 0x03},
  {0xA89D, 0xAC},
  {0xA8B8, 0x03},
  {0xA8BB, 0x13},
  {0xA8BC, 0x28},
  {0xA8BD, 0x25},
  {0xA8BE, 0x1D},
  {0xA8C0, 0x3A},
  {0xA8C1, 0xE0},
  {0xB24F, 0x80},
  /* RMSC Setting */
  {0x8858, 0x00},
  /* LSC Setting */
  {0x6B42, 0x40},
  {0x6B46, 0x00},
  {0x6B47, 0x4B},
  {0x6B4A, 0x00},
  {0x6B4B, 0x4B},
  {0x6B4E, 0x00},
  {0x6B4F, 0x4B},
  {0x6B44, 0x00},
  {0x6B45, 0x8C},
  {0x6B48, 0x00},
  {0x6B49, 0x8C},
  {0x6B4C, 0x00},
  {0x6B4D, 0x8C},
};

static const struct reg_value imx230_setting_full[] = {
  /* Mode: 5344x4016 Full 24fps */
  /* Preset Settings*/
  {0x9004, 0x00},
  {0x9005, 0x00},
  /* Mode Settings*/
  {0x0114, 0x03},
  {0x0220, 0x00},
  {0x0221, 0x11},
  {0x0222, 0x01},
  {0x0340, 0x10},
  {0x0341, 0x36},
  {0x0342, 0x17},
  {0x0343, 0x88},
  {0x0344, 0x00},
  {0x0345, 0x00},
  {0x0346, 0x00},
  {0x0347, 0x00},
  {0x0348, 0x14},
  {0x0349, 0xDF},
  {0x034A, 0x0F},
  {0x034B, 0xAF},
  {0x0381, 0x01},
  {0x0383, 0x01},
  {0x0385, 0x01},
  {0x0387, 0x01},
  {0x0900, 0x00},
  {0x0901, 0x11},
  {0x0902, 0x00},
  {0x3000, 0x74},
  {0x3001, 0x00},
  {0x305C, 0x11},
  /* Output Size Settings */
  {0x0112, 0x0A},
  {0x0113, 0x0A},
  {0x034C, 0x14},
  {0x034D, 0xE0},
  {0x034E, 0x0F},
  {0x034F, 0xB0},
  {0x0401, 0x00},
  {0x0404, 0x00},
  {0x0405, 0x10},
  {0x0408, 0x00},
  {0x0409, 0x00},
  {0x040A, 0x00},
  {0x040B, 0x00},
  {0x040C, 0x14},
  {0x040D, 0xE0},
  {0x040E, 0x0F},
  {0x040F, 0xB0},
  /* Clock Settings */
  {0x0301, 0x04},
  {0x0303, 0x02},
  {0x0305, 0x04},
  {0x0306, 0x00},
  {0x0307, 0xC8},
  {0x0309, 0x0A},
  {0x030B, 0x01},
  {0x030D, 0x0C},
  {0x030E, 0x02},
  {0x030F, 0xC6},
  {0x0310, 0x01},
  /* Data Rate Settings */
  {0x0820, 0x16},
  {0x0821, 0x30},
  {0x0822, 0x00},
  {0x0823, 0x00},
  /* Integration Time Settings */
  {0x0202, 0x10},
  {0x0203, 0x2C},
  {0x0224, 0x01},
  {0x0225, 0xF4},
  /* Gain Settings */
  {0x0204, 0x00},
  {0x0205, 0x00},
  {0x0216, 0x00},
  {0x0217, 0x00},
  {0x020E, 0x01},
  {0x020F, 0x00},
  {0x0210, 0x01},
  {0x0211, 0x00},
  {0x0212, 0x01},
  {0x0213, 0x00},
  {0x0214, 0x01},
  {0x0215, 0x00},
  /* HDR Settings */
  {0x3006, 0x01},
  {0x3007, 0x02},
  {0x31E0, 0x03},
  {0x31E1, 0xFF},
  {0x31E4, 0x02},
  /* DPC2D Settings */
  {0x3A22, 0x20},
  {0x3A23, 0x14},
  {0x3A24, 0xE0},
  {0x3A25, 0x0F},
  {0x3A26, 0xB0},
  {0x3A2F, 0x00},
  {0x3A30, 0x00},
  {0x3A31, 0x00},
  {0x3A32, 0x00},
  {0x3A33, 0x14},
  {0x3A34, 0xDF},
  {0x3A35, 0x0F},
  {0x3A36, 0xAF},
  {0x3A37, 0x00},
  {0x3A38, 0x00},
  {0x3A39, 0x00},
  /* LSC Settings */
  {0x3A21, 0x00},
  /* Stats Settings */
  {0x3011, 0x00},
  {0x3013, 0x01},
  /* MIPI Global Timing Settings*/
  {0x080A, 0x00},
  {0x080B, 0xA7},
  {0x080C, 0x00},
  {0x080D, 0x6F},
  {0x080E, 0x00},
  {0x080F, 0x9F},
  {0x0810, 0x00},
  {0x0811, 0x5F},
  {0x0812, 0x00},
  {0x0813, 0x5F},
  {0x0814, 0x00},
  {0x0815, 0x6F},
  {0x0816, 0x01},
  {0x0817, 0x7F},
  {0x0818, 0x00},
  {0x0819, 0x4F},
};

static const struct reg_value imx230_setting_4k2k[] = {
  /* Mode: 4272x2404 4k2k cropped 16:9 30 fps */
  /* Mode Setting */
  {0x0114, 0x03},
  {0x0220, 0x00},
  {0x0221, 0x11},
  {0x0222, 0x01},
  {0x0340, 0x09},
  {0x0341, 0xBE},
  {0x0342, 0x17},
  {0x0343, 0x88},
  {0x0344, 0x00},
  {0x0345, 0x00},
  {0x0346, 0x03},
  {0x0347, 0x26},
  {0x0348, 0x14},
  {0x0349, 0xDF},
  {0x034A, 0x0C},
  {0x034B, 0x89},
  {0x0381, 0x01},
  {0x0383, 0x01},
  {0x0385, 0x01},
  {0x0387, 0x01},
  {0x0900, 0x00},
  {0x0901, 0x11},
  {0x0902, 0x00},
  {0x3000, 0x74},
  {0x3001, 0x00},
  {0x305C, 0x11},
  /* Output Size Setting */
  {0x0112, 0x0A},
  {0x0113, 0x0A},
  {0x034C, 0x10},
  {0x034D, 0xB0},
  {0x034E, 0x09},
  {0x034F, 0x64},
  {0x0401, 0x00},
  {0x0404, 0x00},
  {0x0405, 0x10},
  {0x0408, 0x02},
  {0x0409, 0x18},
  {0x040A, 0x00},
  {0x040B, 0x00},
  {0x040C, 0x10},
  {0x040D, 0xB0},
  {0x040E, 0x09},
  {0x040F, 0x64},
  /* Clock Setting */
  {0x0301, 0x04},
  {0x0303, 0x02},
  {0x0305, 0x04},
  {0x0306, 0x00},
  {0x0307, 0x98},
  {0x0309, 0x0A},
  {0x030B, 0x01},
  {0x030D, 0x0F},
  {0x030E, 0x02},
  {0x030F, 0xCE},
  {0x0310, 0x01},
  /* Data Rate Setting */
  {0x0820, 0x11},
  {0x0821, 0xF3},
  {0x0822, 0x33},
  {0x0823, 0x33},
  /* Integration Time Setting */
  {0x0202, 0x09},
  {0x0203, 0xB4},
  {0x0224, 0x01},
  {0x0225, 0xF4},
  /* Gain Setting */
  {0x0204, 0x00},
  {0x0205, 0x00},
  {0x0216, 0x00},
  {0x0217, 0x00},
  {0x020E, 0x01},
  {0x020F, 0x00},
  {0x0210, 0x01},
  {0x0211, 0x00},
  {0x0212, 0x01},
  {0x0213, 0x00},
  {0x0214, 0x01},
  {0x0215, 0x00},
  /* HDR Setting */
  {0x3006, 0x01},
  {0x3007, 0x02},
  {0x31E0, 0x03},
  {0x31E1, 0xFF},
  {0x31E4, 0x02},
  /* DPC2D Setting */
  {0x3A22, 0x20},
  {0x3A23, 0x14},
  {0x3A24, 0xE0},
  {0x3A25, 0x09},
  {0x3A26, 0x64},
  {0x3A2F, 0x00},
  {0x3A30, 0x00},
  {0x3A31, 0x03},
  {0x3A32, 0x26},
  {0x3A33, 0x14},
  {0x3A34, 0xDF},
  {0x3A35, 0x0C},
  {0x3A36, 0x89},
  {0x3A37, 0x00},
  {0x3A38, 0x00},
  {0x3A39, 0x00},
  /* LSC Setting */
  {0x3A21, 0x00},
  /* Stats Setting */
  {0x3011, 0x00},
  {0x3013, 0x00},
  /* MIPI Global Timing Settings*/
  {0x080A, 0x00},
  {0x080B, 0xA7},
  {0x080C, 0x00},
  {0x080D, 0x6F},
  {0x080E, 0x00},
  {0x080F, 0x9F},
  {0x0810, 0x00},
  {0x0811, 0x5F},
  {0x0812, 0x00},
  {0x0813, 0x5F},
  {0x0814, 0x00},
  {0x0815, 0x6F},
  {0x0816, 0x01},
  {0x0817, 0x7F},
  {0x0818, 0x00},
  {0x0819, 0x4F},
};

static const struct reg_value imx230_setting_1080[] = {
  /* Mode: 2136x1202 1080p 16:9 30 fps */
  /* Preset Settings*/
  {0x9004, 0x00},
  {0x9005, 0x00},
  /* Mode Settings */
  {0x0114, 0x03},
  {0x0220, 0x00},
  {0x0221, 0x11},
  {0x0222, 0x01},
  {0x0340, 0x0A},
  {0x0341, 0x18},
  {0x0342, 0x17},
  {0x0343, 0x88},
  {0x0344, 0x00},
  {0x0345, 0x00},
  {0x0346, 0x03},
  {0x0347, 0x28},
  {0x0348, 0x14},
  {0x0349, 0xDF},
  {0x034A, 0x0C},
  {0x034B, 0x8B},
  {0x0381, 0x01},
  {0x0383, 0x01},
  {0x0385, 0x01},
  {0x0387, 0x01},
  {0x0900, 0x01},
  {0x0901, 0x22},
  {0x0902, 0x00},
  {0x3000, 0x74},
  {0x3001, 0x00}, 
  {0x305C, 0x11},
  /* Output Size Settings */
  {0x0112, 0x0A},
  {0x0113, 0x0A},
  {0x034C, 0x08},
  {0x034D, 0x58},
  {0x034E, 0x04},
  {0x034F, 0xB2},
  {0x0401, 0x00},
  {0x0404, 0x00},
  {0x0405, 0x10},
  {0x0408, 0x01},
  {0x0409, 0x0C},
  {0x040A, 0x00},
  {0x040B, 0x00},
  {0x040C, 0x08},
  {0x040D, 0x58},
  {0x040E, 0x04},
  {0x040F, 0xB2},
  /* Clock Settings */
  {0x0301, 0x04},
  {0x0303, 0x02},
  {0x0305, 0x04},
  {0x0306, 0x00},
  {0x0307, 0x9C},
  {0x0309, 0x0A},
  {0x030B, 0x01},
  {0x030D, 0x0F},
  {0x030E, 0x02},
  {0x030F, 0xCE},
  {0x0310, 0x01},
  /* Data Rate Settings */
  {0x0820, 0x11},
  {0x0821, 0xF3},
  {0x0822, 0x33},
  {0x0823, 0x33},
  /* Integration Time Settings */
  {0x0202, 0x05},
  {0x0203, 0x02},
  {0x0224, 0x01},
  {0x0225, 0xF4},
  /* Gain Setting */
  {0x0204, 0x00},
  {0x0205, 0x00},
  {0x0216, 0x00},
  {0x0217, 0x00},
  {0x020E, 0x01},
  {0x020F, 0x00},
  {0x0210, 0x01},
  {0x0211, 0x00},
  {0x0212, 0x01},
  {0x0213, 0x00},
  {0x0214, 0x01},
  {0x0215, 0x00},
  /* HDR Settings */
  {0x3006, 0x01},
  {0x3007, 0x02},
  {0x31E0, 0x03},
  {0x31E1, 0xFF},
  {0x31E4, 0x02},
  /* DPC2D Settings */
  {0x3A22, 0x20},
  {0x3A23, 0x14},
  {0x3A24, 0xE0},
  {0x3A25, 0x04},
  {0x3A26, 0xB2},
  {0x3A2F, 0x00},
  {0x3A30, 0x00},
  {0x3A31, 0x03},
  {0x3A32, 0x28},
  {0x3A33, 0x14},
  {0x3A34, 0xDF},
  {0x3A35, 0x0C},
  {0x3A36, 0x8B},
  {0x3A37, 0x00},
  {0x3A38, 0x01},
  {0x3A39, 0x00},
  /* LSC Settings */
  {0x3A21, 0x00},
  /* Stats Setting */
  {0x3011, 0x00},
  {0x3013, 0x00},
  /* MIPI Global Timing Settings*/
  {0x080A, 0x00},
  {0x080B, 0xA7},
  {0x080C, 0x00},
  {0x080D, 0x6F},
  {0x080E, 0x00},
  {0x080F, 0x9F},
  {0x0810, 0x00},
  {0x0811, 0x5F},
  {0x0812, 0x00},
  {0x0813, 0x5F},
  {0x0814, 0x00},
  {0x0815, 0x6F},
  {0x0816, 0x01},
  {0x0817, 0x7F},
  {0x0818, 0x00},
  {0x0819, 0x4F},
};

static const struct reg_value imx230_setting_720[] = {
  /* Mode: 1316x740 120 fps*/
  /* Preset Settings */
  {0x9004, 0x00},
  {0x9005, 0x00},
  /* Mode Settings */
  {0x0114, 0x03},
  {0x0220, 0x00},
  {0x0221, 0x11},
  {0x0222, 0x01},
  {0x0340, 0x03},
  {0x0341, 0x3E},
  {0x0342, 0x17},
  {0x0343, 0x88},
  {0x0344, 0x00},
  {0x0345, 0x00},
  {0x0346, 0x04},
  {0x0347, 0xF4},
  {0x0348, 0x14},
  {0x0349, 0xDF},
  {0x034A, 0x0A},
  {0x034B, 0xBB},
  {0x0381, 0x01},
  {0x0383, 0x01},
  {0x0385, 0x01},
  {0x0387, 0x01},
  {0x0900, 0x01},
  {0x0901, 0x22},
  {0x0902, 0x00},
  {0x3000, 0x74},
  {0x3001, 0x00},
  {0x305C, 0x11},
  /* Output Size Settings */
  {0x0112, 0x0A},
  {0x0113, 0x0A},
  {0x034C, 0x05},
  {0x034D, 0x24},
  {0x034E, 0x02},
  {0x034F, 0xE4},
  {0x0401, 0x00},
  {0x0404, 0x00},
  {0x0405, 0x10},
  {0x0408, 0x02},
  {0x0409, 0xA6},
  {0x040A, 0x00},
  {0x040B, 0x00},
  {0x040C, 0x05},
  {0x040D, 0x24},
  {0x040E, 0x02},
  {0x040F, 0xE4},
  /* Clock Settings */
  {0x0301, 0x04},
  {0x0303, 0x02},
  {0x0305, 0x04},
  {0x0306, 0x00},
  {0x0307, 0xC8},
  {0x0309, 0x0A},
  {0x030B, 0x01},
  {0x030D, 0x0F},
  {0x030E, 0x02},
  {0x030F, 0xCE},
  {0x0310, 0x01},
  /* Data Rate Settings */
  {0x0820, 0x11},
  {0x0821, 0xF3},
  {0x0822, 0x33},
  {0x0823, 0x33},
  /* Integration Time Settings */
  {0x0202, 0x03},
  {0x0203, 0x34},
  {0x0224, 0x01},
  {0x0225, 0xF4},
  /* Gain Setting */
  {0x0204, 0x00},
  {0x0205, 0x00},
  {0x0216, 0x00},
  {0x0217, 0x00},
  {0x020E, 0x01},
  {0x020F, 0x00},
  {0x0210, 0x01},
  {0x0211, 0x00},
  {0x0212, 0x01},
  {0x0213, 0x00},
  {0x0214, 0x01},
  {0x0215, 0x00},
  /* HDR Settings */
  {0x3006, 0x01},
  {0x3007, 0x02},
  {0x31E0, 0x03},
  {0x31E1, 0xFF},
  {0x31E4, 0x02},
  /* DPC2D Settings */
  {0x3A22, 0x20},
  {0x3A23, 0x14},
  {0x3A24, 0xE0},
  {0x3A25, 0x02},
  {0x3A26, 0xE4},
  {0x3A2F, 0x00},
  {0x3A30, 0x00},
  {0x3A31, 0x04},
  {0x3A32, 0xF4},
  {0x3A33, 0x14},
  {0x3A34, 0xDF},
  {0x3A35, 0x0A},
  {0x3A36, 0xBB},
  {0x3A37, 0x00},
  {0x3A38, 0x01},
  {0x3A39, 0x00},
  /* LSC Settings */
  {0x3A21, 0x00},
  /* Stats Setting */
  {0x3011, 0x00},
  {0x3013, 0x00},
  /* MIPI Global Timing Settings*/
  {0x080A, 0x00},
  {0x080B, 0xA7},
  {0x080C, 0x00},
  {0x080D, 0x6F},
  {0x080E, 0x00},
  {0x080F, 0x9F},
  {0x0810, 0x00},
  {0x0811, 0x5F},
  {0x0812, 0x00},
  {0x0813, 0x5F},
  {0x0814, 0x00},
  {0x0815, 0x6F},
  {0x0816, 0x01},
  {0x0817, 0x7F},
  {0x0818, 0x00},
  {0x0819, 0x4F},
};

static const s64 link_freq[] = {
	240000000,
	240000000,
};

static const struct imx230_mode_info imx230_mode_info_data[] = {
	{
		.width = 5344,
		.height = 4016,
		.data = imx230_setting_full,
		.data_size = ARRAY_SIZE(imx230_setting_full),
		.pixel_clock = 568000000,
		.link_freq = 0, /* an index in link_freq[] */
		.exposure_max = 1704,
		.exposure_def = 504,
		.timeperframe = {
			.numerator = 100,
			.denominator = 2400,
		}
	},
        {
                .width = 4272,
                .height = 2404,
                .data = imx230_setting_4k2k,
                .data_size = ARRAY_SIZE(imx230_setting_4k2k),
                .pixel_clock = 459520000,
                .link_freq = 0, /* an index in link_freq[] */
                .exposure_max = 1704,
                .exposure_def = 504,
                .timeperframe = {
                        .numerator = 100,
                        .denominator = 2400,
                }
        },
	{
		.width = 2136,
		.height = 1202,
		.data = imx230_setting_1080,
		.data_size = ARRAY_SIZE(imx230_setting_1080),
		.pixel_clock = 459520000,
		.link_freq = 0, /* an index in link_freq[] */
		.exposure_max = 840,
		.exposure_def = 504,
		.timeperframe = {
			.numerator = 30,
			.denominator = 1000,
		}
	},
        {
                .width = 1316,
                .height = 740,
                .data = imx230_setting_720,
                .data_size = ARRAY_SIZE(imx230_setting_720),
                .pixel_clock = 459520000,
                .link_freq = 0, /* an index in link_freq[] */
                .exposure_max = 840,
                .exposure_def = 504,
                .timeperframe = {
                        .numerator = 100,
                        .denominator = 12000,
                }
        },
#if 0
	{
		.width = 640,
		.height = 480,
		.data = imx230_global_init_setting,//imx230_setting_vga_90fps,
		.data_size = ARRAY_SIZE(imx230_global_init_setting),//ARRAY_SIZE(imx230_setting_vga_90fps),
		.pixel_clock = 48000000,
		.link_freq = 0, /* an index in link_freq[] */
		.exposure_max = 552,
		.exposure_def = 504,
		.timeperframe = {
			.numerator = 100,
			.denominator = 9043
		}
	},
#endif
};

static int imx230_read_reg(struct imx230 *imx230, u16 reg, u8 *val)
{
	u8 regbuf[2];
	int ret;

	regbuf[0] = reg >> 8;
	regbuf[1] = reg & 0xff;

	ret = i2c_master_send(imx230->i2c_client, regbuf, 2);
	if (ret < 0) {
		dev_err(imx230->dev, "%s: write reg error %d: reg=%x\n",
			__func__, ret, reg);
		return ret;
	}

	ret = i2c_master_recv(imx230->i2c_client, val, 1);
	if (ret < 0) {
		dev_err(imx230->dev, "%s: read reg error %d: reg=%x\n",
			__func__, ret, reg);
		return ret;
	}

	return 0;
}

static int imx230_write_reg(struct imx230 *imx230, u16 reg, u8 val)
{
	u8 regbuf[3];
	int ret;

	regbuf[0] = reg >> 8;
	regbuf[1] = reg & 0xff;
	regbuf[2] = val;

	ret = i2c_master_send(imx230->i2c_client, regbuf, 3);
	if (ret < 0) {
		dev_err(imx230->dev, "%s: write reg error %d: reg=%x, val=%x\n",
			__func__, ret, reg, val);
		return ret;
	}

	return 0;
}

static int imx230_regulators_enable(struct imx230 *imx230)
{
	int ret;

	ret = regulator_enable(imx230->io_regulator);
	if (ret < 0) {
		dev_err(imx230->dev, "set io voltage failed\n");
		return ret;
	}

	ret = regulator_enable(imx230->analog_regulator);
	if (ret) {
		dev_err(imx230->dev, "set analog voltage failed\n");
		goto err_disable_io;
	}

	ret = regulator_enable(imx230->core_regulator);
	if (ret) {
		dev_err(imx230->dev, "set core voltage failed\n");
		goto err_disable_analog;
	}

	return 0;

err_disable_analog:
	regulator_disable(imx230->analog_regulator);
err_disable_io:
	regulator_disable(imx230->io_regulator);

	return ret;
}

static void imx230_regulators_disable(struct imx230 *imx230)
{
	int ret;

	ret = regulator_disable(imx230->core_regulator);
	if (ret < 0)
		dev_err(imx230->dev, "core regulator disable failed\n");

	ret = regulator_disable(imx230->analog_regulator);
	if (ret < 0)
		dev_err(imx230->dev, "analog regulator disable failed\n");

	ret = regulator_disable(imx230->io_regulator);
	if (ret < 0)
		dev_err(imx230->dev, "io regulator disable failed\n");
}

static int imx230_set_register_array(struct imx230 *imx230,
				     const struct reg_value *settings,
				     unsigned int num_settings)
{
	unsigned int i;
	int ret;

	for (i = 0; i < num_settings; ++i, ++settings) {
		ret = imx230_write_reg(imx230, settings->reg, settings->val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int imx230_set_power_on(struct imx230 *imx230)
{
	int ret;

	dev_err(imx230->dev,"AKHIL___%s__\n",__func__);
	ret = imx230_regulators_enable(imx230);
	if (ret < 0) {
		return ret;
	}

	ret = clk_prepare_enable(imx230->xclk);
	if (ret < 0) {
		dev_err(imx230->dev, "clk prepare enable failed\n");
		imx230_regulators_disable(imx230);
		return ret;
	}

	usleep_range(5000, 15000);
	gpiod_set_value_cansleep(imx230->enable_gpio, 1);

	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(imx230->rst_gpio, 0);

	//msleep(20);
	//gpiod_set_value_cansleep(imx230->rst_gpio, 1);
	msleep(20);

	return 0;
}

static void imx230_set_power_off(struct imx230 *imx230)
{
	gpiod_set_value_cansleep(imx230->rst_gpio, 1);
	gpiod_set_value_cansleep(imx230->enable_gpio, 0);
	clk_disable_unprepare(imx230->xclk);
	imx230_regulators_disable(imx230);
}

static int imx230_s_power(struct v4l2_subdev *sd, int on)
{
	struct imx230 *imx230 = to_imx230(sd);
	int ret = 0;

	mutex_lock(&imx230->power_lock);

	/* If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */

	dev_err(imx230->dev,"AKHIL___%s__\n",__func__);
//	if (imx230->power_count == !on) {
	if (imx230->power_on == !!on){
		goto exit;
	}

		if (on) {
			mutex_lock(&imx230_lock);

			ret = imx230_set_power_on(imx230);
			if (ret < 0)
				goto exit;
/*
			ret = imx230_write_reg_to(imx230, 0x3100,
						imx230->i2c_client->addr << 1, 0x3c);
			if (ret < 0) {
				dev_err(imx230->dev,
					"could not change i2c address\n");
				imx230_set_power_off(imx230);
				mutex_unlock(&imx230_lock);
				goto exit;
			}
*/
			mutex_unlock(&imx230_lock);

			ret = imx230_set_register_array(imx230,
					imx230_global_init_setting,
					ARRAY_SIZE(imx230_global_init_setting));
			if (ret < 0) {
				dev_err(imx230->dev,
					"could not set init registers\n");
				imx230_set_power_off(imx230);
				goto exit;
			}
/*
			ret = imx230_write_reg(imx230, IMX230_SYSTEM_CTRL0,
					       IMX230_SYSTEM_CTRL0_STOP);
			if (ret < 0) {
				imx230_set_power_off(imx230);
				goto exit;
			}
*/
			imx230->power_on = true;
		} else {
			imx230_set_power_off(imx230);
			imx230->power_on = false;
		}
//	}

	/* Update the power count. */
//	imx230->power_count += on ? 1 : -1;
//	WARN_ON(imx230->power_count < 0);

exit:
	mutex_unlock(&imx230->power_lock);

	dev_err(imx230->dev,"AKHIL___%s2__ret==%d__\n",__func__,ret);
	return ret;
}

static struct v4l2_mbus_framefmt *
__imx230_get_pad_format(struct imx230 *imx230,
			struct v4l2_subdev_pad_config *cfg,
			unsigned int pad,
			enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(&imx230->sd, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &imx230->fmt;
	default:
		return NULL;
	}
}

static int imx230_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *format)
{
	struct imx230 *imx230 = to_imx230(sd);

	format->format = *__imx230_get_pad_format(imx230, cfg, format->pad,
						  format->which);

	return 0;
}

static const struct imx230_mode_info *
imx230_find_mode_by_size(unsigned int width, unsigned int height)
{
	unsigned int max_dist_match = (unsigned int) -1;
	int i, n = 0;

	for (i = 0; i < ARRAY_SIZE(imx230_mode_info_data); i++) {
		unsigned int dist = min(width, imx230_mode_info_data[i].width)
				* min(height, imx230_mode_info_data[i].height);

		dist = imx230_mode_info_data[i].width *
				imx230_mode_info_data[i].height +
				width * height - 2 * dist;

		if (dist < max_dist_match) {
			n = i;
			max_dist_match = dist;
		}
	}

	return &imx230_mode_info_data[n];
}

static const struct v4l2_ctrl_ops imx230_ctrl_ops = {
	.s_ctrl = NULL,//imx230_s_ctrl,
};

static int imx230_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SBGGR10_1X10;

	return 0;
}

static int imx230_enum_frame_size(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->code != MEDIA_BUS_FMT_SBGGR10_1X10)
		return -EINVAL;

	if (fse->index >= ARRAY_SIZE(imx230_mode_info_data))
		return -EINVAL;

	fse->min_width = imx230_mode_info_data[fse->index].width;
	fse->max_width = imx230_mode_info_data[fse->index].width;
	fse->min_height = imx230_mode_info_data[fse->index].height;
	fse->max_height = imx230_mode_info_data[fse->index].height;

	return 0;
}

static int imx230_enum_frame_ival(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_interval_enum *fie)
{
	int index = fie->index;
	int i;

	for (i = 0; i < ARRAY_SIZE(imx230_mode_info_data); i++) {
		if (fie->width != imx230_mode_info_data[i].width ||
		    fie->height != imx230_mode_info_data[i].height)
			continue;

		if (index-- == 0) {
			fie->interval = imx230_mode_info_data[i].timeperframe;
			return 0;
		}
	}

	return -EINVAL;
}

static struct v4l2_rect *
__imx230_get_pad_crop(struct imx230 *imx230, struct v4l2_subdev_pad_config *cfg,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&imx230->sd, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &imx230->crop;
	default:
		return NULL;
	}
}

static inline u32 avg_fps(const struct v4l2_fract *t)
{
	return (t->denominator + (t->numerator >> 1)) / t->numerator;
}

static const struct imx230_mode_info *
imx230_find_mode_by_ival(struct imx230 *imx230, struct v4l2_fract *timeperframe)
{
	const struct imx230_mode_info *mode = imx230->current_mode;
	int fps_req = avg_fps(timeperframe);
	unsigned int max_dist_match = (unsigned int) -1;
	int i, n = 0;

	for (i = 0; i < ARRAY_SIZE(imx230_mode_info_data); i++) {
		unsigned int dist;
		int fps_tmp;

		if (mode->width != imx230_mode_info_data[i].width ||
		    mode->height != imx230_mode_info_data[i].height)
			continue;

		fps_tmp = avg_fps(&imx230_mode_info_data[i].timeperframe);

		dist = abs(fps_req - fps_tmp);

		if (dist < max_dist_match) {
			n = i;
			max_dist_match = dist;
		}
	}

	return &imx230_mode_info_data[n];
}

static int imx230_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *format)
{
	struct imx230 *imx230 = to_imx230(sd);
	struct v4l2_mbus_framefmt *__format;
	struct v4l2_rect *__crop;
	const struct imx230_mode_info *new_mode;
	int ret;

	__crop = __imx230_get_pad_crop(imx230, cfg, format->pad, format->which);

	new_mode = imx230_find_mode_by_size(format->format.width,
					    format->format.height);

	__crop->width = new_mode->width;
	__crop->height = new_mode->height;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		ret = v4l2_ctrl_s_ctrl_int64(imx230->pixel_clock,
					     new_mode->pixel_clock);
		if (ret < 0)
			return ret;

		ret = v4l2_ctrl_s_ctrl(imx230->link_freq,
				       new_mode->link_freq);
		if (ret < 0)
			return ret;

		ret = v4l2_ctrl_modify_range(imx230->exposure,
					     1, new_mode->exposure_max,
					     1, new_mode->exposure_def);
		if (ret < 0)
			return ret;

		ret = v4l2_ctrl_s_ctrl(imx230->exposure,
				       new_mode->exposure_def);
		if (ret < 0)
			return ret;

		ret = v4l2_ctrl_s_ctrl(imx230->gain, 16);
		if (ret < 0)
			return ret;

		imx230->current_mode = new_mode;
	}

	__format = __imx230_get_pad_format(imx230, cfg, format->pad,
					   format->which);
	__format->width = __crop->width;
	__format->height = __crop->height;
	__format->code = MEDIA_BUS_FMT_SBGGR10_1X10;
	__format->field = V4L2_FIELD_NONE;
	__format->colorspace = V4L2_COLORSPACE_SRGB;
	__format->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(__format->colorspace);
	__format->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
				__format->colorspace, __format->ycbcr_enc);
	__format->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(__format->colorspace);

	format->format = *__format;

	return 0;
}

static int imx230_entity_init_cfg(struct v4l2_subdev *subdev,
				  struct v4l2_subdev_pad_config *cfg)
{
	struct v4l2_subdev_format fmt = { 0 };

	fmt.which = cfg ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.format.width = 1316;
	fmt.format.height = 740;

	imx230_set_format(subdev, cfg, &fmt);

	return 0;
}

static int imx230_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
{
	struct imx230 *imx230 = to_imx230(sd);

	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	sel->r = *__imx230_get_pad_crop(imx230, cfg, sel->pad,
					sel->which);
	return 0;
}

static int imx230_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct imx230 *imx230 = to_imx230(subdev);
	int ret;

	dev_err(imx230->dev, "AKHIL::start stream\n");
	if (enable) {
		ret = imx230_set_register_array(imx230,
					imx230->current_mode->data,
					imx230->current_mode->data_size);
		if (ret < 0) {
			dev_err(imx230->dev, "could not set mode %dx%d\n",
				imx230->current_mode->width,
				imx230->current_mode->height);
			return ret;
		}
		ret = v4l2_ctrl_handler_setup(&imx230->ctrls);
		if (ret < 0) {
			dev_err(imx230->dev, "could not sync v4l2 controls\n");
			return ret;
		}
		ret = imx230_write_reg(imx230, IMX230_SC_MODE_SELECT,
				       IMX230_SC_MODE_SELECT_STREAMING);
		if (ret < 0)
			return ret;
		dev_err(imx230->dev, "start stream success\n");
	} else {
		ret = imx230_write_reg(imx230, IMX230_SC_MODE_SELECT,
				       IMX230_SC_MODE_SELECT_SW_STANDBY);
		dev_err(imx230->dev, "start stream failed\n");
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int imx230_get_frame_interval(struct v4l2_subdev *subdev,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct imx230 *imx230 = to_imx230(subdev);

	fi->interval = imx230->current_mode->timeperframe;

	return 0;
}

static int imx230_set_frame_interval(struct v4l2_subdev *subdev,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct imx230 *imx230 = to_imx230(subdev);
	const struct imx230_mode_info *new_mode;

	new_mode = imx230_find_mode_by_ival(imx230, &fi->interval);

	if (new_mode != imx230->current_mode) {
		int ret;

		ret = v4l2_ctrl_s_ctrl_int64(imx230->pixel_clock,
					     new_mode->pixel_clock);
		if (ret < 0)
			return ret;

		ret = v4l2_ctrl_s_ctrl(imx230->link_freq,
				       new_mode->link_freq);
		if (ret < 0)
			return ret;

		ret = v4l2_ctrl_modify_range(imx230->exposure,
					     1, new_mode->exposure_max,
					     1, new_mode->exposure_def);
		if (ret < 0)
			return ret;

		ret = v4l2_ctrl_s_ctrl(imx230->exposure,
				       new_mode->exposure_def);
		if (ret < 0)
			return ret;

		ret = v4l2_ctrl_s_ctrl(imx230->gain, 16);
		if (ret < 0)
			return ret;

		imx230->current_mode = new_mode;
	}

	fi->interval = imx230->current_mode->timeperframe;

	return 0;
}

static const struct v4l2_subdev_core_ops imx230_core_ops = {
	.s_power = imx230_s_power,
};

static const struct v4l2_subdev_video_ops imx230_video_ops = {
	.s_stream = imx230_s_stream,
	.g_frame_interval = NULL,//imx230_get_frame_interval,
	.s_frame_interval = NULL,//imx230_set_frame_interval,
};

static const struct v4l2_subdev_pad_ops imx230_subdev_pad_ops = {
	.init_cfg = imx230_entity_init_cfg,
	.enum_mbus_code = imx230_enum_mbus_code,
	.enum_frame_size = imx230_enum_frame_size,
	.enum_frame_interval = imx230_enum_frame_ival,
	.get_fmt = imx230_get_format,
	.set_fmt = imx230_set_format,
	.get_selection = imx230_get_selection,
};

static const struct v4l2_subdev_ops imx230_subdev_ops = {
	.core = &imx230_core_ops,
	.video = &imx230_video_ops,
	.pad = &imx230_subdev_pad_ops,
};

static int imx230_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	//struct device_node *endpoint;
	struct imx230 *imx230;
	u8 chip_id_high, chip_id_low;
	u32 xclk_freq;
	int ret;
//	struct imx230_ctrls *imx230_ctrls;

	imx230 = devm_kzalloc(dev, sizeof(struct imx230), GFP_KERNEL);
	if (!imx230)
		return -ENOMEM;

	imx230->i2c_client = client;
	imx230->dev = dev;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(endpoint, &imx230->ep);
	fwnode_handle_put(endpoint);

	if (ret < 0) {
		dev_err(dev, "parsing endpoint node failed\n");
		return ret;
	}


	if (imx230->ep.bus_type != V4L2_MBUS_CSI2) {
		dev_err(dev, "invalid bus type, must be CSI2\n");
		return -EINVAL;
	}

	/* get system clock (xclk) */
	imx230->xclk = devm_clk_get(dev, "xclk");
	if (IS_ERR(imx230->xclk)) {
		dev_err(dev, "could not get xclk");
		return PTR_ERR(imx230->xclk);
	}

//	ret = of_property_read_u32(dev->of_node, "clock-frequency", &xclk_freq);

	ret = fwnode_property_read_u32(dev_fwnode(dev), "clock-frequency",
				       &xclk_freq);
	if (ret) {
		dev_err(dev, "could not get xclk frequency\n");
		return ret;
	}

	if (xclk_freq < 23760000 || xclk_freq > 24240000) {
		dev_err(dev, "external clock frequency %u is not supported\n",
			xclk_freq);
		return -EINVAL;
	}

	ret = clk_set_rate(imx230->xclk, xclk_freq);
	if (ret) {
		dev_err(dev, "could not set xclk frequency\n");
		return ret;
	}

	imx230->io_regulator = devm_regulator_get(dev, "vdddo");
	if (IS_ERR(imx230->io_regulator)) {
		dev_err(dev, "cannot get io regulator\n");
		return PTR_ERR(imx230->io_regulator);
	}

	ret = regulator_set_voltage(imx230->io_regulator,
				    IMX230_VOLTAGE_DIGITAL_IO,
				    IMX230_VOLTAGE_DIGITAL_IO);
	if (ret < 0) {
		dev_err(dev, "cannot set io voltage\n");
		return ret;
	}

	imx230->core_regulator = devm_regulator_get(dev, "vddd");
	if (IS_ERR(imx230->core_regulator)) {
		dev_err(dev, "cannot get core regulator\n");
		return PTR_ERR(imx230->core_regulator);
	}

	ret = regulator_set_voltage(imx230->core_regulator,
				    IMX230_VOLTAGE_DIGITAL_CORE,
				    IMX230_VOLTAGE_DIGITAL_CORE);
	if (ret < 0) {
		dev_err(dev, "cannot set core voltage\n");
		return ret;
	}

	imx230->analog_regulator = devm_regulator_get(dev, "vdda");
	if (IS_ERR(imx230->analog_regulator)) {
		dev_err(dev, "cannot get analog regulator\n");
		return PTR_ERR(imx230->analog_regulator);
	}

	ret = regulator_set_voltage(imx230->analog_regulator,
				    IMX230_VOLTAGE_ANALOG,
				    IMX230_VOLTAGE_ANALOG);
	if (ret < 0) {
		dev_err(dev, "cannot set analog voltage\n");
		return ret;
	}

	imx230->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(imx230->enable_gpio)) {
		dev_err(dev, "cannot get enable gpio\n");
		return PTR_ERR(imx230->enable_gpio);
	}

	imx230->rst_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(imx230->rst_gpio)) {
		dev_err(dev, "cannot get reset gpio\n");
		return PTR_ERR(imx230->rst_gpio);
	}

	mutex_init(&imx230->power_lock);

//	imx230_ctrls = &(imx230->imx230_ctrls);
	v4l2_ctrl_handler_init(&imx230->ctrls, 7);
/*
	imx230_ctrls->test_pattern = v4l2_ctrl_new_std_menu_items(&imx230->ctrls, &imx230_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx230_test_pattern_menu) - 1,/////
				     0, 0, imx230_test_pattern_menu);
	imx230->pixel_clock = v4l2_ctrl_new_std(&imx230->ctrls,
						&imx230_ctrl_ops,
						V4L2_CID_PIXEL_RATE,
						1, INT_MAX, 1, 1);
	imx230->link_freq = v4l2_ctrl_new_int_menu(&imx230->ctrls,
						   &imx230_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(link_freq) - 1,
						   0, link_freq);
	if (imx230->link_freq)
		imx230->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
*/
	v4l2_ctrl_new_std(&imx230->ctrls, &imx230_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&imx230->ctrls, &imx230_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);
	imx230->exposure = v4l2_ctrl_new_std(&imx230->ctrls, &imx230_ctrl_ops,
					     V4L2_CID_EXPOSURE, 1, 32, 1, 32);
	imx230->gain = v4l2_ctrl_new_std(&imx230->ctrls, &imx230_ctrl_ops,
					 V4L2_CID_GAIN, 16, 1023, 1, 16);
//	v4l2_ctrl_new_std_menu_items(&imx230->ctrls, &imx230_ctrl_ops,
//				     V4L2_CID_TEST_PATTERN,
//				     ARRAY_SIZE(imx230_test_pattern_menu) - 1,
//				     0, 0, imx230_test_pattern_menu);
	imx230->pixel_clock = v4l2_ctrl_new_std(&imx230->ctrls,
						&imx230_ctrl_ops,
						V4L2_CID_PIXEL_RATE,
						1, INT_MAX, 1, 1);
	imx230->link_freq = v4l2_ctrl_new_int_menu(&imx230->ctrls,
						   &imx230_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(link_freq) - 1,
						   0, link_freq);
	if (imx230->link_freq)
		imx230->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx230->sd.ctrl_handler = &imx230->ctrls;

	if (imx230->ctrls.error) {
		dev_err(dev, "%s: control initialization error %d\n",
		       __func__, imx230->ctrls.error);
		ret = imx230->ctrls.error;
		goto free_ctrl;
	}

	v4l2_i2c_subdev_init(&imx230->sd, client, &imx230_subdev_ops);
	imx230->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx230->pad.flags = MEDIA_PAD_FL_SOURCE;
	imx230->sd.dev = &client->dev;
	imx230->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ret = media_entity_pads_init(&imx230->sd.entity, 1, &imx230->pad);
	if (ret < 0) {
		dev_err(dev, "could not register media entity\n");
		goto free_ctrl;
	}

	ret = imx230_s_power(&imx230->sd, true);
	if (ret < 0) {
		dev_err(dev, "could not power up imx230\n");
		goto free_entity;
	}

	ret = imx230_read_reg(imx230, IMX230_CHIP_ID_HIGH, &chip_id_high);
	if (ret < 0 || chip_id_high != IMX230_CHIP_ID_HIGH_BYTE) {
		dev_err(dev, "could not read ID high\n");
		ret = -ENODEV;
		goto power_down;
	}
	ret = imx230_read_reg(imx230, IMX230_CHIP_ID_LOW, &chip_id_low);
	if (ret < 0 || chip_id_low != IMX230_CHIP_ID_LOW_BYTE) {
		dev_err(dev, "could not read ID low\n");
		ret = -ENODEV;
		goto power_down;
	}

	dev_info(dev, "imx230 detected at address 0x%02x\n", client->addr);
/*
	ret = imx230_read_reg(imx230, imx230_AEC_PK_MANUAL,
			      &imx230->aec_pk_manual);
	if (ret < 0) {
		dev_err(dev, "could not read AEC/AGC mode\n");
		ret = -ENODEV;
		goto power_down;
	}

	ret = imx230_read_reg(imx230, imx230_TIMING_TC_REG20,
			      &imx230->timing_tc_reg20);
	if (ret < 0) {
		dev_err(dev, "could not read vflip value\n");
		ret = -ENODEV;
		goto power_down;
	}

	ret = imx230_read_reg(imx230, imx230_TIMING_TC_REG21,
			      &imx230->timing_tc_reg21);
	if (ret < 0) {
		dev_err(dev, "could not read hflip value\n");
		ret = -ENODEV;
		goto power_down;
	}
*/
	imx230_s_power(&imx230->sd, false);

	ret = v4l2_async_register_subdev(&imx230->sd);
	if (ret < 0) {
		dev_err(dev, "could not register v4l2 device\n");
		goto free_entity;
	}

	imx230_entity_init_cfg(&imx230->sd, NULL);

	return 0;

power_down:
	imx230_s_power(&imx230->sd, false);
free_entity:
	media_entity_cleanup(&imx230->sd.entity);
free_ctrl:
	v4l2_ctrl_handler_free(&imx230->ctrls);
	mutex_destroy(&imx230->power_lock);

	return ret;
}

static int imx230_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx230 *imx230 = to_imx230(sd);

	v4l2_async_unregister_subdev(&imx230->sd);
	media_entity_cleanup(&imx230->sd.entity);
	v4l2_ctrl_handler_free(&imx230->ctrls);
	mutex_destroy(&imx230->power_lock);

	return 0;
}

static const struct i2c_device_id imx230_id[] = {
	{ "imx230", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, imx230_id);

static const struct of_device_id imx230_of_match[] = {
	{ .compatible = "sony,imx230" },
	{ /* inforce */ }
};
MODULE_DEVICE_TABLE(of, imx230_of_match);

static struct i2c_driver imx230_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(imx230_of_match),
		.name  = "imx230",
	},
	.probe  = imx230_probe,
	.remove = imx230_remove,
	.id_table = imx230_id,
};

module_i2c_driver(imx230_i2c_driver);

MODULE_DESCRIPTION("Sony imx230 Camera Driver");
MODULE_AUTHOR("Akhil Xavier <akhilxavier@inforcecomputing.com>");
MODULE_LICENSE("GPL v2");
