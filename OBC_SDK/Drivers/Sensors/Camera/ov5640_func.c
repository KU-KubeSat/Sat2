/*!
*********************************************************************************************
* @file ov5640_func.c
* @brief Support for camera OV5640-YUV
*********************************************************************************************
* @author            Kolio
* @version           1.0.0
* @date              2018.07.04
*
* @copyright         (C) Copyright Endurosat
*
*                    Contents and presentations are protected world-wide.
*                    Any kind of using, copying etc. is prohibited without prior permission.
*                    All rights - incl. industrial property rights - are reserved.
*
* @history
* @revision{         1.0.0  , 2018.07.04, author Kolio, Initial revision }
* @endhistory
*********************************************************************************************
*/

/*
*********************************************************************************************
* INCLUDES
*********************************************************************************************
*/
#include "stm32f7xx_hal.h"
#include "main.h"
#include "ov5640_func.h"
#include "cmsis_os.h"
#include "MX_I2C.h"
#include "es_exeh.h"

/*
*********************************************************************************************
* INTERNAL DEFINES
*********************************************************************************************
*/
/* No Internal defines */

/*
*********************************************************************************************
* INTERNAL TYPES DEFINITION
*********************************************************************************************
*/
/* No Internal types definition */

/*
*********************************************************************************************
* EXTERNAL VARIABLES DEFINITION
*********************************************************************************************
*/
const struct sensor_reg OV5640YUV_Sensor_Dvp_Init[] =
{
		{ 0x4740, 0x20 },

		//{ 0x4050, 0x6e },
	    //{ 0x4051, 0x8f },

		{ 0x3008, 0x42 },
		{ 0x3103, 0x03 },
		{ 0x3017, 0x7f },
		{ 0x3018, 0xff },
		{ 0x302c, 0x02 },
		{ 0x3108, 0x01 },
		{ 0x3630, 0x2e },//2e
		{ 0x3632, 0xe2 },
		{ 0x3633, 0x23 },//23
		{ 0x3621, 0xe0 },
		{ 0x3704, 0xa0 },
		{ 0x3703, 0x5a },
		{ 0x3715, 0x78 },
		{ 0x3717, 0x01 },
		{ 0x370b, 0x60 },
		{ 0x3705, 0x1a },
		{ 0x3905, 0x02 },
		{ 0x3906, 0x10 },
		{ 0x3901, 0x0a },
		{ 0x3731, 0x12 },
		{ 0x3600, 0x08 },
		{ 0x3601, 0x33 },
		{ 0x302d, 0x60 },
		{ 0x3620, 0x52 },
		{ 0x371b, 0x20 },
		{ 0x471c, 0x50 },

		{ 0x3a18, 0x00 },
		{ 0x3a19, 0xf8 },

		{ 0x3635, 0x1c },//1c
		{ 0x3634, 0x40 },
		{ 0x3622, 0x01 },

		{ 0x3c04, 0x28 },
		{ 0x3c05, 0x98 },
		{ 0x3c06, 0x00 },
		{ 0x3c07, 0x08 },
		{ 0x3c08, 0x00 },
		{ 0x3c09, 0x1c },
		{ 0x3c0a, 0x9c },
		{ 0x3c0b, 0x40 },

		{ 0x3820, 0x41 },
		{ 0x3821, 0x01 }, //07

		//windows setup
		{ 0x3800, 0x00 },
		{ 0x3801, 0x00 },
		{ 0x3802, 0x00 },
		{ 0x3803, 0x04 },
		{ 0x3804, 0x0a },
		{ 0x3805, 0x3f },
		{ 0x3806, 0x07 },
		{ 0x3807, 0x9b },
		{ 0x3808, 0x05 },
		{ 0x3809, 0x00 },
		{ 0x380a, 0x03 },
		{ 0x380b, 0xc0 },
		{ 0x3810, 0x00 },
		{ 0x3811, 0x10 },
		{ 0x3812, 0x00 },
		{ 0x3813, 0x06 },
		{ 0x3814, 0x31 },
		{ 0x3815, 0x31 },

		{ 0x3034, 0x1a },
		{ 0x3035, 0x21 }, //15fps
		{ 0x3036, 0x46 },
		{ 0x3037, 0x13 },
		{ 0x3038, 0x00 },
		{ 0x3039, 0x00 },

		{ 0x380c, 0x07 },
		{ 0x380d, 0x68 },
		{ 0x380e, 0x03 }, //03
		{ 0x380f, 0xd8 }, //d8

		{ 0x3c01, 0xb4 },
		{ 0x3c00, 0x04 },
		{ 0x3a08, 0x00 },
		{ 0x3a09, 0x93 },
		{ 0x3a0e, 0x06 },
		{ 0x3a0a, 0x00 },
		{ 0x3a0b, 0x7b },
		{ 0x3a0d, 0x08 },

		{ 0x3a00, 0x3c }, //15fps-10fps
		{ 0x3a02, 0x05 },
		{ 0x3a03, 0xc4 },
		{ 0x3a14, 0x05 },
		{ 0x3a15, 0xc4 },

		{ 0x3618, 0x00 },
		{ 0x3612, 0x29 },
		{ 0x3708, 0x64 },
		{ 0x3709, 0x52 },
		{ 0x370c, 0x03 },

		{ 0x4001, 0x02 },
		{ 0x4004, 0x02 },
		{ 0x3000, 0x00 },
		{ 0x3002, 0x1c },
		{ 0x3004, 0xff },
		{ 0x3006, 0xc3 },
		{ 0x300e, 0x58 },
		{ 0x302e, 0x00 },
		{ 0x4300, 0x30 },
		{ 0x501f, 0x00 },
		{ 0x4713, 0x03 },
		{ 0x4407, 0x04 },
		{ 0x460b, 0x35 },
		{ 0x460c, 0x22 },//add by bright
	  { 0x3824, 0x01 },//add by bright
		{ 0x5001, 0xa3 },

		{ 0x3406, 0x01 },//awbinit
		{ 0x3400, 0x06 },
		{ 0x3401, 0x80 },
		{ 0x3402, 0x04 },
		{ 0x3403, 0x00 },
		{ 0x3404, 0x06 },
		{ 0x3405, 0x00 },
	  //awb
		{ 0x5180, 0xff },
		{ 0x5181, 0xf2 },
		{ 0x5182, 0x00 },
		{ 0x5183, 0x14 },
		{ 0x5184, 0x25 },
		{ 0x5185, 0x24 },
		{ 0x5186, 0x16 },
		{ 0x5187, 0x16 },
		{ 0x5188, 0x16 },
		{ 0x5189, 0x62 },
		{ 0x518a, 0x62 },
		{ 0x518b, 0xf0 },
		{ 0x518c, 0xb2 },
		{ 0x518d, 0x50 },
		{ 0x518e, 0x30 },
		{ 0x518f, 0x30 },
		{ 0x5190, 0x50 },
		{ 0x5191, 0xf8 },
		{ 0x5192, 0x04 },
		{ 0x5193, 0x70 },
		{ 0x5194, 0xf0 },
		{ 0x5195, 0xf0 },
		{ 0x5196, 0x03 },
		{ 0x5197, 0x01 },
		{ 0x5198, 0x04 },
		{ 0x5199, 0x12 },
		{ 0x519a, 0x04 },
		{ 0x519b, 0x00 },
		{ 0x519c, 0x06 },
		{ 0x519d, 0x82 },
		{ 0x519e, 0x38 },
		//color matrix
		{ 0x5381, 0x1e },
		{ 0x5382, 0x5b },
		{ 0x5383, 0x14 },
		{ 0x5384, 0x06 },
		{ 0x5385, 0x82 },
		{ 0x5386, 0x88 },
		{ 0x5387, 0x7c },
		{ 0x5388, 0x60 },
		{ 0x5389, 0x1c },
		{ 0x538a, 0x01 },
		{ 0x538b, 0x98 },
		//sharp&noise
		{ 0x5300, 0x08 },
		{ 0x5301, 0x30 },
		{ 0x5302, 0x3f },
		{ 0x5303, 0x10 },
		{ 0x5304, 0x08 },
		{ 0x5305, 0x30 },
		{ 0x5306, 0x18 },
		{ 0x5307, 0x28 },
		{ 0x5309, 0x08 },
		{ 0x530a, 0x30 },
		{ 0x530b, 0x04 },
		{ 0x530c, 0x06 },
		//gamma
		{ 0x5480, 0x01 },
		{ 0x5481, 0x06 },
		{ 0x5482, 0x12 },
		{ 0x5483, 0x24 },
		{ 0x5484, 0x4a },
		{ 0x5485, 0x58 },
		{ 0x5486, 0x65 },
		{ 0x5487, 0x72 },
		{ 0x5488, 0x7d },
		{ 0x5489, 0x88 },
		{ 0x548a, 0x92 },
		{ 0x548b, 0xa3 },
		{ 0x548c, 0xb2 },
		{ 0x548d, 0xc8 },
		{ 0x548e, 0xdd },
		{ 0x548f, 0xf0 },
		{ 0x5490, 0x15 },
		//UV adjust
		{ 0x5580, 0x06 },
		{ 0x5583, 0x40 },
		{ 0x5584, 0x20 },
		{ 0x5589, 0x10 },
		{ 0x558a, 0x00 },
		{ 0x558b, 0xf8 },
		//lens shading
		{ 0x5000, 0xa7 },
		{ 0x5800, 0x20 },
		{ 0x5801, 0x19 },
		{ 0x5802, 0x17 },
		{ 0x5803, 0x16 },
		{ 0x5804, 0x18 },
		{ 0x5805, 0x21 },
		{ 0x5806, 0x0F },
		{ 0x5807, 0x0A },
		{ 0x5808, 0x07 },
		{ 0x5809, 0x07 },
		{ 0x580a, 0x0A },
		{ 0x580b, 0x0C },
		{ 0x580c, 0x0A },
		{ 0x580d, 0x03 },
		{ 0x580e, 0x01 },
		{ 0x580f, 0x01 },
		{ 0x5810, 0x03 },
		{ 0x5811, 0x09 },
		{ 0x5812, 0x0A },
		{ 0x5813, 0x03 },
		{ 0x5814, 0x01 },
		{ 0x5815, 0x01 },
		{ 0x5816, 0x03 },
		{ 0x5817, 0x08 },
		{ 0x5818, 0x10 },
		{ 0x5819, 0x0A },
		{ 0x581a, 0x06 },
		{ 0x581b, 0x06 },
		{ 0x581c, 0x08 },
		{ 0x581d, 0x0E },
		{ 0x581e, 0x22 },
		{ 0x581f, 0x18 },
		{ 0x5820, 0x13 },
		{ 0x5821, 0x12 },
		{ 0x5822, 0x16 },
		{ 0x5823, 0x1E },
		{ 0x5824, 0x64 },
		{ 0x5825, 0x2A },
		{ 0x5826, 0x2C },
		{ 0x5827, 0x2A },
		{ 0x5828, 0x46 },
		{ 0x5829, 0x2A },
		{ 0x582a, 0x26 },
		{ 0x582b, 0x24 },
		{ 0x582c, 0x26 },
		{ 0x582d, 0x2A },
		{ 0x582e, 0x28 },
		{ 0x582f, 0x42 },
		{ 0x5830, 0x40 },
		{ 0x5831, 0x42 },
		{ 0x5832, 0x08 },
		{ 0x5833, 0x28 },
		{ 0x5834, 0x26 },
		{ 0x5835, 0x24 },
		{ 0x5836, 0x26 },
		{ 0x5837, 0x2A },
		{ 0x5838, 0x44 },
		{ 0x5839, 0x4A },
		{ 0x583a, 0x2C },
		{ 0x583b, 0x2a },
		{ 0x583c, 0x46 },
		{ 0x583d, 0xCE },

		{ 0x5688, 0x22 },
		{ 0x5689, 0x22 },
		{ 0x568a, 0x42 },
		{ 0x568b, 0x24 },
		{ 0x568c, 0x42 },
		{ 0x568d, 0x24 },
		{ 0x568e, 0x22 },
		{ 0x568f, 0x22 },

		{ 0x5025, 0x00 },

		{ 0x3a0f, 0x30 },
		{ 0x3a10, 0x28 },
		{ 0x3a1b, 0x30 },
		{ 0x3a1e, 0x28 },
		{ 0x3a11, 0x61 },
		{ 0x3a1f, 0x10 },

		{ 0x4005, 0x1a },
		{ 0x3406, 0x00 },//awbinit
    { 0x3503, 0x00 },//awbinit
		{ 0x3008, 0x02 },
		{ 0xffff, 0xff },
};


const struct sensor_reg ov5640_vga_preview[] =
{
		// YUV VGA 30fps, night mode 5fps
		// Input Clock = 24Mhz, PCLK = 56MHz
		{ 0x3035, 0x11 }, // PLL
		{ 0x3036, 0x46 }, // PLL
		{ 0x3c07, 0x08 }, // light meter 1 threshold [7:0]
		{ 0x3820, 0x41 }, // Sensor flip off, ISP flip on
		{ 0x3821, 0x01 }, // Sensor mirror on, ISP mirror on, H binning on
		{ 0x3814, 0x31 }, // X INC
		{ 0x3815, 0x31 }, // Y INC
		{ 0x3800, 0x00 }, // HS
		{ 0x3801, 0x00 }, // HS
		{ 0x3802, 0x00 }, // VS
		{ 0x3803, 0x04 }, // VS
		{ 0x3804, 0x0a }, // HW (HE)
		{ 0x3805, 0x3f }, // HW (HE)
		{ 0x3806, 0x07 }, // VH (VE)
		{ 0x3807, 0x9b }, // VH (VE)
		{ 0x3808, 0x02 }, // DVPHO
		{ 0x3809, 0x80 }, // DVPHO
		{ 0x380a, 0x01 }, // DVPVO
		{ 0x380b, 0xe0 }, // DVPVO
		{ 0x380c, 0x07 }, // HTS
		{ 0x380d, 0x68 }, // HTS
		{ 0x380e, 0x03 }, // VTS
		{ 0x380f, 0xd8 }, // VTS
		{ 0x3813, 0x06 }, // Timing Voffset
		{ 0x3618, 0x00 },
		{ 0x3612, 0x29 },
		{ 0x3709, 0x52 },
		{ 0x370c, 0x03 },
		{ 0x3a02, 0x17 }, // 60Hz max exposure, night mode 5fps
		{ 0x3a03, 0x10 }, // 60Hz max exposure
		{ 0x3a14, 0x17 }, // 50Hz max exposure, night mode 5fps
		{ 0x3a15, 0x10 }, // 50Hz max exposure
		{ 0x4004, 0x02 }, // BLC 2 lines
		{ 0x3002, 0x1c }, // reset JFIFO, SFIFO, JPEG
		{ 0x3006, 0xc3 }, // disable clock of JPEG2x, JPEG
		{ 0x4713, 0x03 }, // JPEG mode 3
		{ 0x4407, 0x04 }, // Quantization scale
		{ 0x460b, 0x35 },
		{ 0x460c, 0x22 },
		{ 0x4837, 0x22 }, // DVP CLK divider
		{ 0x3824, 0x02 }, // DVP CLK divider
		{ 0x5001, 0xa3 }, // SDE on, scale on, UV average off, color matrix on, AWB on
		{ 0x3503, 0x00 }, // AEC/AGC on
                { 0xffff, 0xff },
};

const struct sensor_reg OV5640_RGB_QVGA[]  =
{
		{0x3008, 0x02},
		{0x3035, 0x41},
		{0x4740, 0x21},
		{0x4300, 0x61},
		{0x3808, 0x01},
		{0x3809, 0x40},
		{0x380a, 0x00},
		{0x380b, 0xf0},
		{0x501f, 0x01},
		{0xffff, 0xff},
};

//2592x1944 QSXGA
const struct sensor_reg OV5640_JPEG_QSXGA[]  =
{
		{0x3820 ,0x40},
		{0x3821 ,0x26},
		{0x3814 ,0x11},
		{0x3815 ,0x11},
		{0x3803 ,0x00},
		{0x3807 ,0x9f},
		{0x3808 ,0x0a},
		{0x3809 ,0x20},
		{0x380a ,0x07},
		{0x380b ,0x98},
		{0x380c ,0x0b},
		{0x380d ,0x1c},
		{0x380e ,0x07},
		{0x380f ,0xb0},
		{0x3813 ,0x04},
		{0x3618 ,0x04},
		{0x3612 ,0x4b},
		{0x3708 ,0x64},
		{0x3709 ,0x12},
		{0x370c ,0x00},
		{0x3a02 ,0x07},
		{0x3a03 ,0xb0},
		{0x3a0e ,0x06},
		{0x3a0d ,0x08},
		{0x3a14 ,0x07},
		{0x3a15 ,0xb0},
		{0x4001 ,0x02},
		{0x4004 ,0x06},
		{0x3002 ,0x00},
		{0x3006 ,0xff},
		{0x3824 ,0x04},
		{0x5001 ,0x83},
		{0x3036 ,0x69},
		{0x3035 ,0x31},
		{0x4005 ,0x1A},
		{0xffff, 0xff},
};

//5MP
const struct sensor_reg OV5640_5MP_JPEG[]  =
{
		{0x3800 ,0x00},
		{0x3801 ,0x00},
		{0x3802 ,0x00},
		{0x3803 ,0x00},
		{0x3804 ,0xA },
		{0x3805 ,0x3f},
		{0x3806 ,0x7 },
		{0x3807 ,0x9f},
		{0x3808 ,0xA },
		{0x3809 ,0x20},
		{0x380a ,0x7 },
		{0x380b ,0x98},
		{0x380c ,0xc },
		{0x380d ,0x80},
		{0x380e ,0x7 },
		{0x380f ,0xd0},
		{0x5001 ,0xa3},
		{0x5680 ,0x0 },
		{0x5681 ,0x0 },
		{0x5682 ,0xA },
		{0x5683 ,0x20},
		{0x5684 ,0x0 },
		{0x5685 ,0x0 },
		{0x5686 ,0x7 },
		{0x5687 ,0x98},
		{0xffff, 0xff},
};

//320x240 QVGA
const struct sensor_reg OV5640_QSXGA2QVGA[]  =
{
		{0x3800 ,0x00},
		{0x3801 ,0x00},
		{0x3802 ,0x00},
		{0x3803 ,0x00},
		{0x3804 ,0xA },
		{0x3805 ,0x3f},
		{0x3806 ,0x7 },
		{0x3807 ,0x9f},
		{0x3808 ,0x1 },
		{0x3809 ,0x40},
		{0x380a ,0x0 },
		{0x380b ,0xf0},
		{0x380c ,0xc },
		{0x380d ,0x80},
		{0x380e ,0x7 },
		{0x380f ,0xd0},
		{0x5001 ,0xa3},
		{0x5680 ,0x0 },
		{0x5681 ,0x0 },
		{0x5682 ,0xA },
		{0x5683 ,0x20},
		{0x5684 ,0x0 },
		{0x5685 ,0x0 },
		{0x5686 ,0x7 },
		{0x5687 ,0x98},
		{0xffff, 0xff},
};

//640x480 VGA
const struct sensor_reg OV5640_QSXGA2VGA[]  =
{
		{0x3800 ,0x00},
		{0x3801 ,0x00},
		{0x3802 ,0x00},
		{0x3803 ,0x00},
		{0x3804 ,0xA },
		{0x3805 ,0x3f},
		{0x3806 ,0x7 },
		{0x3807 ,0x9f},
		{0x3808 ,0x2 },
		{0x3809 ,0x80},
		{0x380a ,0x1 },
		{0x380b ,0xe0},
		{0x380c ,0xc },
		{0x380d ,0x80},
		{0x380e ,0x7 },
		{0x380f ,0xd0},
		{0x5001 ,0xa3},
		{0x5680 ,0x0 },
		{0x5681 ,0x0 },
		{0x5682 ,0xA },
		{0x5683 ,0x20},
		{0x5684 ,0x0 },
		{0x5685 ,0x0 },
		{0x5686 ,0x7 },
		{0x5687 ,0x98},
		{0xffff, 0xff},
};

//800x480 WVGA
const struct sensor_reg OV5640_QSXGA2WVGA[]  =
{
		{0x3800 ,0x00},
		{0x3801 ,0x00},
		{0x3802 ,0x00},
		{0x3803 ,0x00},
		{0x3804 ,0xA },
		{0x3805 ,0x3f},
		{0x3806 ,0x7 },
		{0x3807 ,0x9f},
		{0x3808 ,0x3 },
		{0x3809 ,0x20},
		{0x380a ,0x1 },
		{0x380b ,0xe0},
		{0x380c ,0xc },
		{0x380d ,0x80},
		{0x380e ,0x7 },
		{0x380f ,0xd0},
	  {0x3810, 0x00},
	  {0x3811, 0x10},
	  {0x3812, 0x01},
	  {0x3813, 0x48},
		{0x5001 ,0xa3},
		{0x5680 ,0x0 },
		{0x5681 ,0x0 },
		{0x5682 ,0xA },
		{0x5683 ,0x20},
		{0x5684 ,0x0 },
		{0x5685 ,0x0 },
		{0x5686 ,0x7 },
		{0x5687 ,0x98},
		{0xffff, 0xff},
};

//1280x960 SXGA
const struct sensor_reg OV5640_QSXGA2SXGA[]  =
{
		{0x3800 ,0x00},
		{0x3801 ,0x00},
		{0x3802 ,0x00},
		{0x3803 ,0x00},
		{0x3804 ,0xA },
		{0x3805 ,0x3f},
		{0x3806 ,0x7 },
		{0x3807 ,0x9f},
		{0x3808 ,0x5 },
		{0x3809 ,0x0 },
		{0x380a ,0x3 },
		{0x380b ,0xc0},
		{0x380c ,0xc },
		{0x380d ,0x80},
		{0x380e ,0x7 },
		{0x380f ,0xd0},
		{0x5001 ,0xa3},
		{0x5680 ,0x0 },
		{0x5681 ,0x0 },
		{0x5682 ,0xA },
		{0x5683 ,0x20},
		{0x5684 ,0x0 },
		{0x5685 ,0x0 },
		{0x5686 ,0x7 },
		{0x5687 ,0x98},
		{0xffff, 0xff},

};

//2048x1536 QXGA
const struct sensor_reg OV5640_QSXGA2QXGA[]  =
{
		{0x3800 ,0x00},
		{0x3801 ,0x00},
		{0x3802 ,0x00},
		{0x3803 ,0x00},
		{0x3804 ,0xA },
		{0x3805 ,0x3f},
		{0x3806 ,0x7 },
		{0x3807 ,0x9f},
		{0x3808 ,0x8 },
		{0x3809 ,0x0 },
		{0x380a ,0x6 },
		{0x380b ,0x0 },
		{0x380c ,0xc },
		{0x380d ,0x80},
		{0x380e ,0x7 },
		{0x380f ,0xd0},
		{0x5001 ,0xa3},
		{0x5680 ,0x0 },
		{0x5681 ,0x0 },
		{0x5682 ,0xA },
		{0x5683 ,0x20},
		{0x5684 ,0x0 },
		{0x5685 ,0x0 },
		{0x5686 ,0x7 },
		{0x5687 ,0x98},
		{0xffff, 0xff},
};


//1600x1200 UXGA
const struct sensor_reg OV5640_QSXGA2UXGA[]  =
{
		{0x3800 ,0x00},
		{0x3801 ,0x00},
		{0x3802 ,0x00},
		{0x3803 ,0x00},
		{0x3804 ,0xA },
		{0x3805 ,0x3f},
		{0x3806 ,0x7 },
		{0x3807 ,0x9f},
		{0x3808 ,0x6 },
		{0x3809 ,0x40},
		{0x380a ,0x4 },
		{0x380b ,0xb0},
		{0x380c ,0xc },
		{0x380d ,0x80},
		{0x380e ,0x7 },
		{0x380f ,0xd0},
		{0x5001 ,0xa3},
		{0x5680 ,0x0 },
		{0x5681 ,0x0 },
		{0x5682 ,0xA },
		{0x5683 ,0x20},
		{0x5684 ,0x0 },
		{0x5685 ,0x0 },
		{0x5686 ,0x7 },
		{0x5687 ,0x98},
		{0xffff, 0xff},
};

//1024x768 XGA
const struct sensor_reg OV5640_QSXGA2XGA[]  =
{
		{0x3800 ,0x00},
		{0x3801 ,0x00},
		{0x3802 ,0x00},
		{0x3803 ,0x00},
		{0x3804 ,0xA },
		{0x3805 ,0x3f},
		{0x3806 ,0x7 },
		{0x3807 ,0x9f},
		{0x3808 ,0x4 },
		{0x3809 ,0x0 },
		{0x380a ,0x3 },
		{0x380b ,0x0 },
		{0x380c ,0xc },
		{0x380d ,0x80},
		{0x380e ,0x7 },
		{0x380f ,0xd0},
		{0x5001 ,0xa3},
		{0x5680 ,0x0 },
		{0x5681 ,0x0 },
		{0x5682 ,0xA },
		{0x5683 ,0x20},
		{0x5684 ,0x0 },
		{0x5685 ,0x0 },
		{0x5686 ,0x7 },
		{0x5687 ,0x98},
		{0xffff, 0xff},
};


/*
*********************************************************************************************
* INTERNAL (STATIC) VARIABLES DEFINITION/DECLARATION 
*********************************************************************************************
*/

/*
*********************************************************************************************
* INTERNAL (STATIC) ROUTINES DECLARATION
*********************************************************************************************
*/
/* No Internal routines declaration */

/*
*********************************************************************************************
* EXTERNAL (NONE STATIC) ROUTINES DEFINITION
*********************************************************************************************
*/
void InitCAM(uint8_t m_fmt)
{
  osDelay(100);

  HAL_StatusTypeDef I2C_retStat = HAL_ERROR;
  I2C_retStat = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);
  if(I2C_retStat == HAL_OK)
  {
      if (m_fmt == JPG)
      {
        CamWriteI2C(0x3103, 0x11);
        CamWriteI2C(0x3008, 0x82);
        osDelay(100);
        CamWriteRegsI2C(OV5640YUV_Sensor_Dvp_Init);
        osDelay(500);
        CamWriteRegsI2C(OV5640_JPEG_QSXGA);
        CamWriteRegsI2C(OV5640_QSXGA2QVGA);
        CamWriteI2C(0x4407, 0x04);
      }
      else
      {
        CamWriteI2C(0x3103, 0x11);
        CamWriteI2C(0x3008, 0x82);
        osDelay(500);
        CamWriteRegsI2C(OV5640YUV_Sensor_Dvp_Init);
        CamWriteRegsI2C(OV5640_RGB_QVGA);
      }

      MX_I2C_Release(MX_I2C_BUS_SYSTEM);
  }else{
      EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_ARDU_CAM_COMM, eEXEH_ARDU_CAM_EXCEPTION_ID_COMM_ERROR, __LINE__);
  }
}


void OV5640_set_JPEG_size(uint8_t size)
{
    HAL_StatusTypeDef I2C_retStat = HAL_ERROR;
    I2C_retStat = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);
    if(I2C_retStat == HAL_OK)
    {
        switch (size)
        {
          case OV5640_320x240:
            CamWriteRegsI2C(OV5640_QSXGA2QVGA);
            break;
          case OV5640_640x480:
            CamWriteRegsI2C(OV5640_QSXGA2VGA);
            break;
          case OV5640_800x480:
            CamWriteRegsI2C(OV5640_QSXGA2WVGA);
            break;
          case OV5640_1024x768:
            CamWriteRegsI2C(OV5640_QSXGA2XGA);
            break;
          case OV5640_1280x960:
            CamWriteRegsI2C(OV5640_QSXGA2SXGA);
            break;
          case OV5640_1600x1200:
            CamWriteRegsI2C(OV5640_QSXGA2UXGA);
            break;
          case OV5640_2048x1536:
            CamWriteRegsI2C(OV5640_QSXGA2QXGA);
            break;
          case OV5640_2592x1944:
            CamWriteRegsI2C(OV5640_JPEG_QSXGA);
            break;
          default: //320x240
            CamWriteRegsI2C(OV5640_QSXGA2QVGA);
            break;
        }

        MX_I2C_Release(MX_I2C_BUS_SYSTEM);
    }else{
        EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_ARDU_CAM_COMM, eEXEH_ARDU_CAM_EXCEPTION_ID_COMM_ERROR, __LINE__);
    }


}

void OV5640_set_Light_Mode(uint8_t Light_Mode)
{
    HAL_StatusTypeDef I2C_retStat = HAL_ERROR;
    I2C_retStat = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);
    if(I2C_retStat == HAL_OK)
    {
        switch(Light_Mode)
        {
            case Auto:
                    CamWriteI2C(0x3212, 0x03); // start group 3
                    CamWriteI2C(0x3406, 0x00);
                    CamWriteI2C(0x3400, 0x04);
                    CamWriteI2C(0x3401, 0x00);
                    CamWriteI2C(0x3402, 0x04);
                    CamWriteI2C(0x3403, 0x00);
                    CamWriteI2C(0x3404, 0x04);
                    CamWriteI2C(0x3405, 0x00);
                    CamWriteI2C(0x3212, 0x13); // end group 3
                    CamWriteI2C(0x3212, 0xa3); // lanuch group 3
                    CamWriteI2C(0x5183 ,0x0 );
              break;
            case Sunny:
                    CamWriteI2C(0x3212, 0x03); // start group 3
                    CamWriteI2C(0x3406, 0x01);
                    CamWriteI2C(0x3400, 0x06);
                    CamWriteI2C(0x3401, 0x1c);
                    CamWriteI2C(0x3402, 0x04);
                    CamWriteI2C(0x3403, 0x00);
                    CamWriteI2C(0x3404, 0x04);
                    CamWriteI2C(0x3405, 0xf3);
                    CamWriteI2C(0x3212, 0x13); // end group 3
                    CamWriteI2C(0x3212, 0xa3); // lanuch group 3
              break;
              case Office:
                    CamWriteI2C(0x3212, 0x03); // start group 3
                    CamWriteI2C(0x3406, 0x01);
                    CamWriteI2C(0x3400, 0x05);
                    CamWriteI2C(0x3401, 0x48);
                    CamWriteI2C(0x3402, 0x04);
                    CamWriteI2C(0x3403, 0x00);
                    CamWriteI2C(0x3404, 0x07);
                    CamWriteI2C(0x3405, 0xcf);
                    CamWriteI2C(0x3212, 0x13); // end group 3
                    CamWriteI2C(0x3212, 0xa3); // lanuch group 3
                    CamWriteI2C(0x3212, 0x03); // start group 3
                    CamWriteI2C(0x3406, 0x01);
                    CamWriteI2C(0x3400, 0x06);
                    CamWriteI2C(0x3401, 0x48);
                    CamWriteI2C(0x3402, 0x04);
                    CamWriteI2C(0x3403, 0x00);
                    CamWriteI2C(0x3404, 0x04);
                    CamWriteI2C(0x3405, 0xd3);
                    CamWriteI2C(0x3212, 0x13); // end group 3
                    CamWriteI2C(0x3212, 0xa3); // lanuch group 3
              break;
              case Cloudy:
              CamWriteI2C(0x3212, 0x03); // start group 3
                    CamWriteI2C(0x3406, 0x01);
                    CamWriteI2C(0x3400, 0x06);
                    CamWriteI2C(0x3401, 0x48);
                    CamWriteI2C(0x3402, 0x04);
                    CamWriteI2C(0x3403, 0x00);
                    CamWriteI2C(0x3404, 0x04);
                    CamWriteI2C(0x3405, 0xd3);
                    CamWriteI2C(0x3212, 0x13); // end group 3
                    CamWriteI2C(0x3212, 0xa3); // lanuch group 3
                    break;
              case Home:
              CamWriteI2C(0x3212, 0x03); // start group 3
                    CamWriteI2C(0x3406, 0x01);
                    CamWriteI2C(0x3400, 0x04);
                    CamWriteI2C(0x3401, 0x10);
                    CamWriteI2C(0x3402, 0x04);
                    CamWriteI2C(0x3403, 0x00);
                    CamWriteI2C(0x3404, 0x08);
                    CamWriteI2C(0x3405, 0x40);
                    CamWriteI2C(0x3212, 0x13); // end group 3
                    CamWriteI2C(0x3212, 0xa3); // lanuch group 3
              break;
              default :
              break;
        }

        MX_I2C_Release(MX_I2C_BUS_SYSTEM);
    }else{
        EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_ARDU_CAM_COMM, eEXEH_ARDU_CAM_EXCEPTION_ID_COMM_ERROR, __LINE__);
    }


}

void OV5640_set_Color_Saturation(uint8_t Color_Saturation)
{
    HAL_StatusTypeDef I2C_retStat = HAL_ERROR;
    I2C_retStat = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);
    if(I2C_retStat == HAL_OK)
    {
        switch(Color_Saturation)
        {
        case Saturation3:
                CamWriteI2C(0x3212, 0x03); // start group 3
                CamWriteI2C(0x5381, 0x1c);
                CamWriteI2C(0x5382, 0x5a);
                CamWriteI2C(0x5383, 0x06);
                CamWriteI2C(0x5384, 0x2b);
                CamWriteI2C(0x5385, 0xab);
                CamWriteI2C(0x5386, 0xd6);
                CamWriteI2C(0x5387, 0xda);
                CamWriteI2C(0x5388, 0xd6);
                CamWriteI2C(0x5389, 0x04);
                CamWriteI2C(0x538b, 0x98);
                CamWriteI2C(0x538a, 0x01);
                CamWriteI2C(0x3212, 0x13); // end group 3
                CamWriteI2C(0x3212, 0xa3); // launch group 3
        break;
        case Saturation2:
                CamWriteI2C(0x3212, 0x03); // start group 3
                CamWriteI2C(0x5381, 0x1c);
                CamWriteI2C(0x5382, 0x5a);
                CamWriteI2C(0x5383, 0x06);
                CamWriteI2C(0x5384, 0x24);
                CamWriteI2C(0x5385, 0x8f);
                CamWriteI2C(0x5386, 0xb3);
                CamWriteI2C(0x5387, 0xb6);
                CamWriteI2C(0x5388, 0xb3);
                CamWriteI2C(0x5389, 0x03);
                CamWriteI2C(0x538b, 0x98);
                CamWriteI2C(0x538a, 0x01);
                CamWriteI2C(0x3212, 0x13); // end group 3
                CamWriteI2C(0x3212, 0xa3); // launch group 3
        break;
        case Saturation1:
                CamWriteI2C(0x3212, 0x03); // start group 3
                CamWriteI2C(0x5381, 0x1c);
                CamWriteI2C(0x5382, 0x5a);
                CamWriteI2C(0x5383, 0x06);
                CamWriteI2C(0x5384, 0x1f);
                CamWriteI2C(0x5385, 0x7a);
                CamWriteI2C(0x5386, 0x9a);
                CamWriteI2C(0x5387, 0x9c);
                CamWriteI2C(0x5388, 0x9a);
                CamWriteI2C(0x5389, 0x02);
                CamWriteI2C(0x538b, 0x98);
                CamWriteI2C(0x538a, 0x01);
                CamWriteI2C(0x3212, 0x13); // end group 3
                CamWriteI2C(0x3212, 0xa3); // launch group 3
        break;
        case Saturation0:
                CamWriteI2C(0x3212, 0x03); // start group 3
                CamWriteI2C(0x5381, 0x1c);
                CamWriteI2C(0x5382, 0x5a);
                CamWriteI2C(0x5383, 0x06);
                CamWriteI2C(0x5384, 0x1a);
                CamWriteI2C(0x5385, 0x66);
                CamWriteI2C(0x5386, 0x80);
                CamWriteI2C(0x5387, 0x82);
                CamWriteI2C(0x5388, 0x80);
                CamWriteI2C(0x5389, 0x02);
                CamWriteI2C(0x538b, 0x98);
                CamWriteI2C(0x538a, 0x01);
                CamWriteI2C(0x3212, 0x13); // end group 3
                CamWriteI2C(0x3212, 0xa3); // launch group 3
        break;
        case Saturation_1:
                CamWriteI2C(0x3212, 0x03); // start group 3
                CamWriteI2C(0x5381, 0x1c);
                CamWriteI2C(0x5382, 0x5a);
                CamWriteI2C(0x5383, 0x06);
                CamWriteI2C(0x5384, 0x15);
                CamWriteI2C(0x5385, 0x52);
                CamWriteI2C(0x5386, 0x66);
                CamWriteI2C(0x5387, 0x68);
                CamWriteI2C(0x5388, 0x66);
                CamWriteI2C(0x5389, 0x02);
                CamWriteI2C(0x538b, 0x98);
                CamWriteI2C(0x538a, 0x01);
                CamWriteI2C(0x3212, 0x13); // end group 3
                CamWriteI2C(0x3212, 0xa3); // launch group 3
        break;
                case Saturation_2:
                CamWriteI2C(0x3212, 0x03); // start group 3
                CamWriteI2C(0x5381, 0x1c);
                CamWriteI2C(0x5382, 0x5a);
                CamWriteI2C(0x5383, 0x06);
                CamWriteI2C(0x5384, 0x10);
                CamWriteI2C(0x5385, 0x3d);
                CamWriteI2C(0x5386, 0x4d);
                CamWriteI2C(0x5387, 0x4e);
                CamWriteI2C(0x5388, 0x4d);
                CamWriteI2C(0x5389, 0x01);
                CamWriteI2C(0x538b, 0x98);
                CamWriteI2C(0x538a, 0x01);
                CamWriteI2C(0x3212, 0x13); // end group 3
                CamWriteI2C(0x3212, 0xa3); // launch group 3
        break;
                case Saturation_3:
                CamWriteI2C(0x3212, 0x03); // start group 3
                CamWriteI2C(0x5381, 0x1c);
                CamWriteI2C(0x5382, 0x5a);
                CamWriteI2C(0x5383, 0x06);
                CamWriteI2C(0x5384, 0x0c);
                CamWriteI2C(0x5385, 0x30);
                CamWriteI2C(0x5386, 0x3d);
                CamWriteI2C(0x5387, 0x3e);
                CamWriteI2C(0x5388, 0x3d);
                CamWriteI2C(0x5389, 0x01);
                CamWriteI2C(0x538b, 0x98);
                CamWriteI2C(0x538a, 0x01);
                CamWriteI2C(0x3212, 0x13); // end group 3
                CamWriteI2C(0x3212, 0xa3); // launch group 3
        break;

        default:
            break;
        }

        MX_I2C_Release(MX_I2C_BUS_SYSTEM);
    }else{
        EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_ARDU_CAM_COMM, eEXEH_ARDU_CAM_EXCEPTION_ID_COMM_ERROR, __LINE__);
    }
}

void OV5640_set_Brightness(uint8_t Brightness)
{
    HAL_StatusTypeDef I2C_retStat = HAL_ERROR;
    I2C_retStat = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);
    if(I2C_retStat == HAL_OK)
    {
        switch(Brightness)
        {
            case Brightness4:
                    CamWriteI2C(0x3212, 0x03); // start group 3
                    CamWriteI2C(0x5587, 0x40);
                    CamWriteI2C(0x5588, 0x01);
                    CamWriteI2C(0x3212, 0x13); // end group 3
                    CamWriteI2C(0x3212, 0xa3); // launch group 3
                    break;
             case Brightness3:
                    CamWriteI2C(0x3212, 0x03); // start group 3
                    CamWriteI2C(0x5587, 0x30);
                    CamWriteI2C(0x5588, 0x01);
                    CamWriteI2C(0x3212, 0x13); // end group 3
                    CamWriteI2C(0x3212, 0xa3); // launch group 3
                      break;
             case Brightness2:
                    CamWriteI2C(0x3212, 0x03); // start group 3
                    CamWriteI2C(0x5587, 0x20);
                    CamWriteI2C(0x5588, 0x01);
                    CamWriteI2C(0x3212, 0x13); // end group 3
                    CamWriteI2C(0x3212, 0xa3); // launch group 3
                    break;
            case Brightness1:
                    CamWriteI2C(0x3212, 0x03); // start group 3
                    CamWriteI2C(0x5587, 0x10);
                    CamWriteI2C(0x5588, 0x01);
                    CamWriteI2C(0x3212, 0x13); // end group 3
                    CamWriteI2C(0x3212, 0xa3); // launch group 3
            break;
            case Brightness0:
                    CamWriteI2C(0x3212, 0x03); // start group 3
                    CamWriteI2C(0x5587, 0x00);
                    CamWriteI2C(0x5588, 0x01);
                    CamWriteI2C(0x3212, 0x13); // end group 3
                    CamWriteI2C(0x3212, 0xa3); // launch group 3
                    break;
             case Brightness_1:
                    CamWriteI2C(0x3212, 0x03); // start group 3
                    CamWriteI2C(0x5587, 0x10);
                    CamWriteI2C(0x5588, 0x09);
                    CamWriteI2C(0x3212, 0x13); // end group 3
                    CamWriteI2C(0x3212, 0xa3); // launch group 3
                    break;
             case Brightness_2:
                    CamWriteI2C(0x3212, 0x03); // start group 3
                    CamWriteI2C(0x5587, 0x20);
                    CamWriteI2C(0x5588, 0x09);
                    CamWriteI2C(0x3212, 0x13); // end group 3
                    CamWriteI2C(0x3212, 0xa3); // launch group 3
                    break;
                    case Brightness_3:
                    CamWriteI2C(0x3212, 0x03); // start group 3
                    CamWriteI2C(0x5587, 0x30);
                    CamWriteI2C(0x5588, 0x09);
                    CamWriteI2C(0x3212, 0x13); // end group 3
                    CamWriteI2C(0x3212, 0xa3); // launch group 3
                    break;
            case Brightness_4:
                    CamWriteI2C(0x3212, 0x03); // start group 3
                    CamWriteI2C(0x5587, 0x40);
                    CamWriteI2C(0x5588, 0x09);
                    CamWriteI2C(0x3212, 0x13); // end group 3
                    CamWriteI2C(0x3212, 0xa3); // launch group 3
                    break;

            default:
                break;
        }

        MX_I2C_Release(MX_I2C_BUS_SYSTEM);
    }else{
        EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_ARDU_CAM_COMM, eEXEH_ARDU_CAM_EXCEPTION_ID_COMM_ERROR, __LINE__);
    }

}

void OV5640_set_Contrast(uint8_t Contrast)
{
    HAL_StatusTypeDef I2C_retStat = HAL_ERROR;
    I2C_retStat = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);
    if(I2C_retStat == HAL_OK)
    {
        switch(Contrast)
        {
        case Contrast3:
                CamWriteI2C(0x3212, 0x03); // start group 3
                CamWriteI2C(0x5586, 0x2c);
                CamWriteI2C(0x5585, 0x1c);
                CamWriteI2C(0x3212, 0x13); // end group 3
                CamWriteI2C(0x3212, 0xa3); // launch group 3
        break;
        case Contrast2:
                CamWriteI2C(0x3212, 0x03); // start group 3
                CamWriteI2C(0x5586, 0x28);
                CamWriteI2C(0x5585, 0x18);
                CamWriteI2C(0x3212, 0x13); // end group 3
                CamWriteI2C(0x3212, 0xa3); // launch group 3
        break;
        case Contrast1:
                CamWriteI2C(0x3212, 0x03); // start group 3
                CamWriteI2C(0x5586, 0x24);
                CamWriteI2C(0x5585, 0x10);
                CamWriteI2C(0x3212, 0x13); // end group 3
                CamWriteI2C(0x3212, 0xa3); // launch group 3
        break;
        case Contrast0:
                CamWriteI2C(0x3212, 0x03); // start group 3
                CamWriteI2C(0x3212, 0x03); // start group 3
                CamWriteI2C(0x5586, 0x20);
                CamWriteI2C(0x5585, 0x00);
                CamWriteI2C(0x3212, 0x13); // end group 3
                CamWriteI2C(0x3212, 0xa3); // launch group 3
        break;
        case Contrast_1:
                CamWriteI2C(0x3212, 0x03); // start group 3
                CamWriteI2C(0x5586, 0x1c);
                CamWriteI2C(0x5585, 0x1c);
                CamWriteI2C(0x3212, 0x13); // end group 3
                CamWriteI2C(0x3212, 0xa3); // launch group 3
        break;
        case Contrast_2:
                CamWriteI2C(0x3212, 0x03); // start group 3
                CamWriteI2C(0x5586, 0x18);
                CamWriteI2C(0x5585, 0x18);
                CamWriteI2C(0x3212, 0x13); // end group 3
                CamWriteI2C(0x3212, 0xa3); // launch group 3
        break;
        case Contrast_3:
                CamWriteI2C(0x3212, 0x03); // start group 3
                CamWriteI2C(0x5586, 0x14);
                CamWriteI2C(0x5585, 0x14);
                CamWriteI2C(0x3212, 0x13); // end group 3
                CamWriteI2C(0x3212, 0xa3); // launch group 3
        break;

        default:
            break;
        }

        MX_I2C_Release(MX_I2C_BUS_SYSTEM);
    }else{
        EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_ARDU_CAM_COMM, eEXEH_ARDU_CAM_EXCEPTION_ID_COMM_ERROR, __LINE__);
    }

}

void OV5640_set_Special_effects(uint8_t Special_effect)
{
    HAL_StatusTypeDef I2C_retStat = HAL_ERROR;
    I2C_retStat = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);
    if(I2C_retStat == HAL_OK)
    {
        switch(Special_effect)
        {
            case Normal:
                    CamWriteI2C(0x3212, 0x03); // start group 3
                    CamWriteI2C(0x5580, 0x06);
                    CamWriteI2C(0x5583, 0x40); // sat U
                    CamWriteI2C(0x5584, 0x10); // sat V
                    CamWriteI2C(0x5003, 0x08);
                    CamWriteI2C(0x3212, 0x13); // end group 3
                    CamWriteI2C(0x3212, 0xa3); // launch group
            break;
            case Blueish:
                    CamWriteI2C(0x3212, 0x03); // start group 3
                    CamWriteI2C(0x5580, 0x1e);
                    CamWriteI2C(0x5583, 0xa0);
                    CamWriteI2C(0x5584, 0x40);
                    CamWriteI2C(0x5003, 0x08);
                    CamWriteI2C(0x3212, 0x13); // end group 3
                    CamWriteI2C(0x3212, 0xa3); // launch group 3
            break;
            case Reddish:
                    CamWriteI2C(0x3212, 0x03); // start group 3
                    CamWriteI2C(0x5580, 0x1e);
                    CamWriteI2C(0x5583, 0x80);
                    CamWriteI2C(0x5584, 0xc0);
                    CamWriteI2C(0x5003, 0x08);
                    CamWriteI2C(0x3212, 0x13); // end group 3
                    CamWriteI2C(0x3212, 0xa3); // launch group 3
            break;
            case BW:
                    CamWriteI2C(0x3212, 0x03); // start group 3
                    CamWriteI2C(0x5580, 0x1e);
                    CamWriteI2C(0x5583, 0x80);
                    CamWriteI2C(0x5584, 0x80);
                    CamWriteI2C(0x5003, 0x08);
                    CamWriteI2C(0x3212, 0x13); // end group 3
                    CamWriteI2C(0x3212, 0xa3); // launch group 3
            break;
            case Sepia:
                    CamWriteI2C(0x3212, 0x03); // start group 3
                    CamWriteI2C(0x5580, 0x1e);
                    CamWriteI2C(0x5583, 0x40);
                    CamWriteI2C(0x5584, 0xa0);
                    CamWriteI2C(0x5003, 0x08);
                    CamWriteI2C(0x3212, 0x13); // end group 3
                    CamWriteI2C(0x3212, 0xa3); // launch group 3
            break;

                    case Negative:
                    CamWriteI2C(0x3212, 0x03); // start group 3
                    CamWriteI2C(0x5580, 0x40);
                    CamWriteI2C(0x5003, 0x08);
                    CamWriteI2C(0x5583, 0x40); // sat U
                    CamWriteI2C(0x5584, 0x10); // sat V
                    CamWriteI2C(0x3212, 0x13); // end group 3
                    CamWriteI2C(0x3212, 0xa3); // launch group 3
            break;
            case Greenish:
                    CamWriteI2C(0x3212, 0x03); // start group 3
                    CamWriteI2C(0x5580, 0x1e);
                    CamWriteI2C(0x5583, 0x60);
                    CamWriteI2C(0x5584, 0x60);
                    CamWriteI2C(0x5003, 0x08);
                    CamWriteI2C(0x3212, 0x13); // end group 3
                    CamWriteI2C(0x3212, 0xa3); // launch group 3
            break;
            case Overexposure:
                    CamWriteI2C(0x3212, 0x03); // start group 3
                    CamWriteI2C(0x5580, 0x1e);
                    CamWriteI2C(0x5583, 0xf0);
                    CamWriteI2C(0x5584, 0xf0);
                    CamWriteI2C(0x5003, 0x08);
                    CamWriteI2C(0x3212, 0x13); // end group 3
                    CamWriteI2C(0x3212, 0xa3); // launch group 3
            break;
            case Solarize:
                    CamWriteI2C(0x3212, 0x03); // start group 3
                    CamWriteI2C(0x5580, 0x06);
                    CamWriteI2C(0x5583, 0x40); // sat U
                    CamWriteI2C(0x5584, 0x10); // sat V
                    CamWriteI2C(0x5003, 0x09);
                    CamWriteI2C(0x3212, 0x13); // end group 3
                    CamWriteI2C(0x3212, 0xa3); // launch group 3
            break;

            default:
                break;
        }
        MX_I2C_Release(MX_I2C_BUS_SYSTEM);
    }else{
        EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_ARDU_CAM_COMM, eEXEH_ARDU_CAM_EXCEPTION_ID_COMM_ERROR, __LINE__);
    }

}

void OV5640_set_Night_Mode(uint8_t Night_mode)
{
    uint8_t reg_val;
    HAL_StatusTypeDef I2C_retStat = HAL_ERROR;
    I2C_retStat = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);
    if(I2C_retStat == HAL_OK)
    {
        switch(Night_mode)
        {
                case Night_Mode_On:
                  reg_val = CamReadI2C(0x3a00) | 0x04;
                  CamWriteI2C(0x3a00, reg_val);
                  break;
                case Night_Mode_Off:
                  reg_val = CamReadI2C(0x3a00) & 0xFB;
                  CamWriteI2C(0x3a00, reg_val);
                  break;

                default:
                    break;
        }

        MX_I2C_Release(MX_I2C_BUS_SYSTEM);
    }else{
        EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_ARDU_CAM_COMM, eEXEH_ARDU_CAM_EXCEPTION_ID_COMM_ERROR, __LINE__);
    }
}

void OV5640_set_Banding_Filter(uint8_t Banding_Filter)
{
    uint8_t reg_val;
    HAL_StatusTypeDef I2C_retStat = HAL_ERROR;
    I2C_retStat = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);
    if(I2C_retStat == HAL_OK)
    {
        switch(Banding_Filter)
        {
            case Off:
              reg_val = CamReadI2C(0x3a00) & 0xdf; // turn off banding filter
              CamWriteI2C(0x3a00, reg_val);
              break;
            case Manual_50HZ:
              CamWriteI2C(0x3c00, 04); // set to 50Hz
              CamWriteI2C(0x3c01, 80); // manual banding filter
              reg_val = CamReadI2C(0x3a00) | 0x20; // turn on banding filter
              CamWriteI2C(0x3a00, reg_val);
              break;
            case Manual_60HZ:
              CamWriteI2C(0x3c00, 00); // set to 60Hz
              CamWriteI2C(0x3c01, 80); // manual banding filter
              reg_val = CamReadI2C(0x3a00) | 0x20; // turn on banding filter
              CamWriteI2C(0x3a00, reg_val);
              break;
            case Auto_Detection:
              CamWriteI2C(0x3c01, 00); // auto banding filter
              reg_val = CamReadI2C(0x3a00) & 0xdf; // turn off banding filter
              CamWriteI2C(0x3a00, reg_val);
              break;

            default:
                break;
        }

        MX_I2C_Release(MX_I2C_BUS_SYSTEM);
    }else{
        EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_ARDU_CAM_COMM, eEXEH_ARDU_CAM_EXCEPTION_ID_COMM_ERROR, __LINE__);
    }
}

void OV5640_set_EV(uint8_t EV)
{
     HAL_StatusTypeDef I2C_retStat = HAL_ERROR;
     I2C_retStat = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);
     if(I2C_retStat == HAL_OK)
     {
         switch(EV)
         {
             case EV3:
                     CamWriteI2C(0x3a0f, 0x60);
                     CamWriteI2C(0x3a10, 0x58);
                     CamWriteI2C(0x3a11, 0xa0);
                     CamWriteI2C(0x3a1b, 0x60);
                     CamWriteI2C(0x3a1e, 0x58);
                     CamWriteI2C(0x3a1f, 0x20);
                     break;
             case EV2:
                     CamWriteI2C(0x3a0f, 0x50);
                     CamWriteI2C(0x3a10, 0x48);
                     CamWriteI2C(0x3a11, 0x90);
                     CamWriteI2C(0x3a1b, 0x50);
                     CamWriteI2C(0x3a1e, 0x48);
                     CamWriteI2C(0x3a1f, 0x20);
                     break;
             case EV1:
                     CamWriteI2C(0x3a0f, 0x40);
                     CamWriteI2C(0x3a10, 0x38);
                     CamWriteI2C(0x3a11, 0x71);
                     CamWriteI2C(0x3a1b, 0x40);
                     CamWriteI2C(0x3a1e, 0x38);
                     CamWriteI2C(0x3a1f, 0x10);
                     break;
             case EV0:
                     CamWriteI2C(0x3a0f, 0x38);
                     CamWriteI2C(0x3a10, 0x30);
                     CamWriteI2C(0x3a11, 0x61);
                     CamWriteI2C(0x3a1b, 0x38);
                     CamWriteI2C(0x3a1e, 0x30);
                     CamWriteI2C(0x3a1f, 0x10);
                     break;
             case EV_1:
                     CamWriteI2C(0x3a0f, 0x30);
                     CamWriteI2C(0x3a10, 0x28);
                     CamWriteI2C(0x3a11, 0x61);
                     CamWriteI2C(0x3a1b, 0x30);
                     CamWriteI2C(0x3a1e, 0x28);
                     CamWriteI2C(0x3a1f, 0x10);
                     break;
             case EV_2:
                     CamWriteI2C(0x3a0f, 0x20);
                     CamWriteI2C(0x3a10, 0x18);
                     CamWriteI2C(0x3a11, 0x41);
                     CamWriteI2C(0x3a1b, 0x20);
                     CamWriteI2C(0x3a1e, 0x18);
                     CamWriteI2C(0x3a1f, 0x10);
                     break;
             case EV_3:
                     CamWriteI2C(0x3a0f, 0x10);
                     CamWriteI2C(0x3a10, 0x08);
                     CamWriteI2C(0x3a1b, 0x10);
                     CamWriteI2C(0x3a1e, 0x08);
                     CamWriteI2C(0x3a11, 0x20);
                     CamWriteI2C(0x3a1f, 0x10);
                     break;

             default:
                 break;
         }

         MX_I2C_Release(MX_I2C_BUS_SYSTEM);
     }else{
         EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_ARDU_CAM_COMM, eEXEH_ARDU_CAM_EXCEPTION_ID_COMM_ERROR, __LINE__);
     }
}


/*
*********************************************************************************************
* INTERNAL (STATIC) ROUTINES DEFINITION
*********************************************************************************************
*/
/* No Internal routines definition */

/* ******************************************************************************************* */
