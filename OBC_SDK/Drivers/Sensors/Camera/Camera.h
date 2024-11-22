/*!
*********************************************************************************************
* @file Camera.h
* @brief Header of Camera.c
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
#ifndef _CAMERA_H_
#define _CAMERA_H_

/*
*********************************************************************************************
* INCLUDES
*********************************************************************************************
*/
#include "User_types.h"


/*
*********************************************************************************************
* EXTERNAL DEFINES
*********************************************************************************************
*/

/****************************************************/
/* Sensor related definition 						*/
/****************************************************/
#define CAM_ADDR                0x78
#define OV5640_CHIPID_HIGH      0x300a
#define OV5640_CHIPID_LOW       0x300b

#define JPG	0
#define BMP 	1
#define RAW	2

#define OV5640_320x240 	     0	//320x240
#define OV5640_640x480 	     1	//640x480
#define OV5640_800x480	     2	//800x480
#define OV5640_1024x768	     3	//1024x768
#define OV5640_1280x960	     4	//1280x960
#define OV5640_1600x1200	 5	//1600x1200
#define OV5640_2048x1536	 6   //2048x1536
#define OV5640_2592x1944	 7	//2592x1944

//Light Mode
#define Auto                 0
#define Sunny                1
#define Cloudy               2
#define Office               3
#define Home                 4

#define Advanced_AWB         0
#define Simple_AWB           1
#define Manual_day           2
#define Manual_A             3
#define Manual_cwf           4
#define Manual_cloudy        5

//Color Saturation
#define Saturation4          0
#define Saturation3          1
#define Saturation2          2
#define Saturation1          3
#define Saturation0          4
#define Saturation_1         5
#define Saturation_2         6
#define Saturation_3         7
#define Saturation_4         8

//Brightness
#define Brightness4          0
#define Brightness3          1
#define Brightness2          2
#define Brightness1          3
#define Brightness0          4
#define Brightness_1         5
#define Brightness_2         6
#define Brightness_3         7
#define Brightness_4         8

//Contrast
#define Contrast4            0
#define Contrast3            1
#define Contrast2            2
#define Contrast1            3
#define Contrast0            4
#define Contrast_1           5
#define Contrast_2           6
#define Contrast_3           7
#define Contrast_4           8

#define degree_180           0
#define degree_150           1
#define degree_120           2
#define degree_90            3
#define degree_60            4
#define degree_30            5
#define degree_0             6
#define degree30             7
#define degree60             8
#define degree90             9
#define degree120            10
#define degree150            11

//Special effects
#define Antique                0
#define Bluish                 1
#define Greenish               2
#define Reddish                3
#define BW                     4
#define Negative               5
#define BWnegative             6
#define Normal                 7
#define Sepia                  8
#define Overexposure           9
#define Solarize               10
#define  Blueish               11
#define Yellowish              12

#define Exposure_17_EV         0
#define Exposure_13_EV         1
#define Exposure_10_EV         2
#define Exposure_07_EV         3
#define Exposure_03_EV         4
#define Exposure_default       5
#define Exposure07_EV          6
#define Exposure10_EV          7
#define Exposure13_EV          8
#define Exposure17_EV          9
#define Exposure03_EV          10

#define Auto_Sharpness_default 0
#define Auto_Sharpness1        1
#define Auto_Sharpness2        2
#define Manual_Sharpnessoff    3
#define Manual_Sharpness1      4
#define Manual_Sharpness2      5
#define Manual_Sharpness3      6
#define Manual_Sharpness4      7
#define Manual_Sharpness5      8

#define Sharpness1             0
#define Sharpness2             1
#define Sharpness3             2
#define Sharpness4             3
#define Sharpness5             4
#define Sharpness6             5
#define Sharpness7             6
#define Sharpness8             7
#define Sharpness_auto         8


#define EV3                    0
#define EV2                    1
#define EV1                    2
#define EV0                    3
#define EV_1                   4
#define EV_2                   5
#define EV_3                   6

#define MIRROR                 0
#define FLIP                   1
#define MIRROR_FLIP            2


#define high_quality           0
#define default_quality        1
#define low_quality            2

#define Color_bar              0
#define Color_square           1
#define BW_square              2
#define DLI                    3

#define Night_Mode_On          0
#define Night_Mode_Off         1

#define Off                    0
#define Manual_50HZ            1
#define Manual_60HZ            2
#define Auto_Detection         3

/****************************************************/
/* ArduChip related definition 						*/
/****************************************************/
#define ARDUCHIP_TEST1      0x00  //TEST register
#define ARDUCHIP_TEST2      0x01  //TEST register

#define ARDUCHIP_MODE      	0x02  //Mode register
#define MCU2LCD_MODE       	0x00
#define CAM2LCD_MODE       	0x01
#define LCD2MCU_MODE       	0x02

#define ARDUCHIP_TIM       	0x03  //Timming control
#define HREF_LEVEL_MASK    	0x01  //0 = High active , 1 = Low active
#define VSYNC_LEVEL_MASK   	0x02  //0 = High active , 1 = Low active
#define LCD_BKEN_MASK      	0x04  //0 = Enable, 	1 = Disable
#define DELAY_MASK         	0x08  //0 = no delay, 1 = delay one clock
#define MODE_MASK          	0x10  //0 = LCD mode, 	1 = FIFO mode
#define FIFO_PWRDN_MASK	   	0x20  //0 = Normal operation, 1 = FIFO power down

#define ARDUCHIP_FIFO      	0x04  //FIFO and I2C control
#define FIFO_CLEAR_MASK    	0x01
#define FIFO_START_MASK    	0x02
#define FIFO_RDPTR_RST_MASK 0x10
#define FIFO_WRPTR_RST_MASK 0x20

#define ARDUCHIP_GPIO		0x06  //GPIO Write Register
#define GPIO_RESET_MASK		0x01  //0 = default state, 1 =  Sensor reset IO value
#define GPIO_PWDN_MASK		0x02  //0 = Sensor power down IO value, 1 = Sensor power enable IO value

#define BURST_FIFO_READ		0x3C  //Burst FIFO read operation

#define ARDUCHIP_REV       	0x40  //ArduCHIP revision
#define VER_LOW_MASK       	0x3F
#define VER_HIGH_MASK      	0xC0

#define ARDUCHIP_TRIG      	0x41  //Trigger source
#define VSYNC_MASK         	0x01
#define SHUTTER_MASK       	0x02
#define CAP_DONE_MASK      	0x08

#define FIFO_SIZE1			0x42  //Camera write FIFO size[7:0] for burst to read
#define FIFO_SIZE2			0x43  //Camera write FIFO size[15:8]
#define FIFO_SIZE3			0x44  //Camera write FIFO size[18:16]
#define FIFO_LEN_MASK       0x000FFFFF

/****************************************************/

/*
*********************************************************************************************
* EXTERNAL TYPES DECLARATIONS
*********************************************************************************************
*/
struct sensor_reg {
  uint16_t reg;
  uint16_t val;
};

/*
*********************************************************************************************
* EXTERNAL VARIABLES DECLARATIONS
*********************************************************************************************
*/
extern status_t comstat;
extern uint8_t CamCtrl;

/*
*********************************************************************************************
* EXTERNAL ROUTINES DECLARATIONS 
*********************************************************************************************
*/
void CamBurstReadSPI(uint8_t *buff, uint16_t size);
uint8_t CamReadSPI(uint8_t Addr);
void CamWriteSPI(uint8_t Addr, uint8_t Data);
uint8_t CamReadI2C(uint16_t Addr);
uint8_t CamWriteI2C(uint16_t Addr, uint8_t Val);
uint8_t CamWriteRegsI2C(const struct sensor_reg reglist[]);
void CamInit(uint8_t m_fmt);
status_t CamSetup(void);
uint32_t CamReadFifoLen(void);
void PrintReg(uint8_t Reg);
int CamCapture(uint8_t Ctrl);

void InitCameraModule(void);
uint8_t Camm_IsCammReady(void);
void Camm_MakePicture(void);
uint8_t Camm_GetStatus(void);
void AppProcessCameraData(void);

#endif    /* _CAMERA_H_ */
/* ******************************************************************************************* */
