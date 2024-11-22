/*!
*********************************************************************************************
* @file ov5640_func.h
* @brief Header of ov5640_func.c
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
#ifndef OV5640_FUNC_H
#define OV5640_FUNC_H

/*
*********************************************************************************************
* INCLUDES
*********************************************************************************************
*/
#include "Camera.h"

/*
*********************************************************************************************
* EXTERNAL DEFINES
*********************************************************************************************
*/
/* No External defines*/

/*
*********************************************************************************************
* EXTERNAL TYPES DECLARATIONS
*********************************************************************************************
*/
/* No External types declarations */

/*
*********************************************************************************************
* EXTERNAL VARIABLES DECLARATIONS
*********************************************************************************************
*/
extern const struct sensor_reg OV5640YUV_Sensor_Dvp_Init[];
extern const struct sensor_reg ov5640_vga_preview[];
extern const struct sensor_reg OV5640_RGB_QVGA[];

//2592x1944 QSXGA
extern const struct sensor_reg OV5640_JPEG_QSXGA[];

//5MP
extern const struct sensor_reg OV5640_5MP_JPEG[];

//320x240 QVGA
extern const struct sensor_reg OV5640_QSXGA2QVGA[];

//640x480 VGA
extern const struct sensor_reg OV5640_QSXGA2VGA[];

//800x480 WVGA
extern const struct sensor_reg OV5640_QSXGA2WVGA[];

//1280x960 SXGA
extern const struct sensor_reg OV5640_QSXGA2SXGA[];

//2048x1536 QXGA
extern const struct sensor_reg OV5640_QSXGA2QXGA[];


//1600x1200 UXGA
extern const struct sensor_reg OV5640_QSXGA2UXGA[];

//1024x768 XGA
extern const struct sensor_reg OV5640_QSXGA2XGA[];

/*
*********************************************************************************************
* EXTERNAL ROUTINES DECLARATIONS 
*********************************************************************************************
*/
void OV5640_set_JPEG_size(uint8_t size);

#endif    /* OV5640_FUNC_H */
/* ******************************************************************************************* */
