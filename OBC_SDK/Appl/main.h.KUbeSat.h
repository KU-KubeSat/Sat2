/*!
********************************************************************************************
* @file main.h
* @brief Header of main.c
********************************************************************************************
* @author            Vassil Milev
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
* @revision{         1.0.0  , 2018.07.04, author Vassil Milev, Initial revision }
* @endhistory
********************************************************************************************
*/
#ifndef MAIN_H
#define MAIN_H

/*
*********************************************************************************************
* INCLUDES
*********************************************************************************************
*/
#include <MCU_Init.h>
#include <stdio.h>
#include "cmsis_os.h"

/*
*********************************************************************************************
* EXTERNAL DEFINES
*********************************************************************************************
*/

#if !defined(__FILE_defined)
//typedef __FILE FILE;
# define __FILE_defined
#endif

/* LED related definitions */
#define GREEN_LED_ON() LED_G_GPIO_Port->ODR |= LED_G_Pin;
#define GREEN_LED_OFF() LED_G_GPIO_Port->ODR &= ~LED_G_Pin;
#define GREEN_LED_XOR() LED_G_GPIO_Port->ODR ^= LED_G_Pin;

#define AMBER_LED_ON()  LED_Y_GPIO_Port->ODR |= LED_Y_Pin;
#define AMBER_LED_OFF() LED_Y_GPIO_Port->ODR &= ~LED_Y_Pin;
#define AMBER_LED_XOR() LED_Y_GPIO_Port->ODR ^= LED_Y_Pin;
/* End of LED related definitions */

/* ESTTC related definitions */
#define EPS_I_I2C_ADDRESS       0x18
#define EPS_OUT_PAR_NUM         19
#define EPS_MAX_PAR_NUM         (60)
#define ANT_I2C_ADDRESS         0x33
#define UHF_I2C_ADDRESS         0x23 //0x22
#define UHF_MAX_PAR_NUM         8
#define OBC_I2C_ADDRESS         0x11
#define IMTQ_ADDRESS            0x50

//KUbeSat Added
#define CLYDE_BAT_ADDRESS       0x2A
#define CLYDE_EPS_ADDRESS       0x2B
/* End of ESTTC related definitions */

/* Bootloader related definitions */
#define APPL_ADDRESS            0x08100000
#define BOOT_ADDRESS            0x08000000
#define RTC_INIT_ADDRESS        0x40002850
#define MAILBOX_ADDRESS         0x40002854
#define MAILBOX_VAL_BOOT        0xB007B007
#define MAILBOX_VAL_APPL        0xA552A552
#define MAILBOX_VAL_AUTO_FLASH  0x2AA52AAF
#define MAILBOX_VAL_HARD        0x0BAD0BAD

/* End of Bootloader related definitions */

/* EPS related definitions */
#define BATT_LOW_3_5V                   1489
#define BATT_LOW_3_6V                   1531
#define BATT_LOW_3_7V                   1574
#define BATT_LOW_3_75V                  ((BATT_LOW_3_7V+BATT_LOW_3_8V)/2)
#define BATT_LOW_3_8V                   1617
#define BATT_LOW_3_85V                  ((BATT_LOW_3_8V+BATT_LOW_3_9V)/2)
#define BATT_LOW_3_9V                   1659
#define BATT_LOW_4_0V                   1702
#define BATT_LOW_4_1V                   1744

#define BATT_CURRENT_250mA              83
#define BATT_CURRENT_500mA              167
#define BATT_CURRENT_1000mA             333
/* End of EPS related definitions */

/* Gyroscope related definitions */
#define POWER_ON_GYRO_1_4  { HAL_GPIO_WritePin(PAN1_4_VGY_GPIO_Port, PAN1_4_VGY_Pin, GPIO_PIN_RESET); }
#define POWER_ON_GYRO_2_5  { HAL_GPIO_WritePin(PAN2_5_VGY_GPIO_Port, PAN2_5_VGY_Pin, GPIO_PIN_RESET); }
#define POWER_ON_GYRO_3_6  { HAL_GPIO_WritePin(GPIOB, PAN3_6_VGY_Pin, GPIO_PIN_RESET); }

#define POWER_OFF_GYRO_1_4  { HAL_GPIO_WritePin(PAN1_4_VGY_GPIO_Port, PAN1_4_VGY_Pin, GPIO_PIN_SET); }
#define POWER_OFF_GYRO_2_5  { HAL_GPIO_WritePin(PAN2_5_VGY_GPIO_Port, PAN2_5_VGY_Pin, GPIO_PIN_SET); }
#define POWER_OFF_GYRO_3_6  { HAL_GPIO_WritePin(GPIOB, PAN3_6_VGY_Pin, GPIO_PIN_SET); }

/* End of Gyroscope related definitions */

/* Accelerometer related definitions */
#define POWER_ON_ACC_1   { HAL_GPIO_WritePin(ACC_EN_GPIO_Port, ACC_EN_Pin, GPIO_PIN_RESET);  }
#define POWER_OFF_ACC_1  { HAL_GPIO_WritePin(ACC_EN_GPIO_Port, ACC_EN_Pin, GPIO_PIN_SET);  }
/* End of Accelerometer related definitions */

/* UHF related definitions */
// Turn ON UHF
#define POWER_ON_UHF_1  { HAL_GPIO_WritePin(OBC_OUT5_GPIO_Port, OBC_OUT5_Pin, GPIO_PIN_SET); }
#define POWER_ON_UHF_2  { HAL_GPIO_WritePin(OBC_OUT4_6_GPIO_Port, OBC_OUT4_6_Pin, GPIO_PIN_RESET); }
// Turn OFF UHF
#define POWER_OFF_UHF_1  { HAL_GPIO_WritePin(OBC_OUT5_GPIO_Port, OBC_OUT5_Pin, GPIO_PIN_RESET); }
#define POWER_OFF_UHF_2  { HAL_GPIO_WritePin(OBC_OUT4_6_GPIO_Port, OBC_OUT4_6_Pin, GPIO_PIN_SET); }
/* End of UHF related definitions */

/* ON/OFF the redundant OBC related definitions */
#define POWER_ON_SECOND_OBC  { HAL_GPIO_WritePin(OBC_OUT4_6_GPIO_Port, OBC_OUT4_6_Pin, GPIO_PIN_RESET); }
#define POWER_OFF_SECOND_OBC { HAL_GPIO_WritePin(OBC_OUT4_6_GPIO_Port, OBC_OUT4_6_Pin, GPIO_PIN_SET); }
/* End of ON/OFF the redundant OBC */

#define	__disable_interrupt()	__disable_irq()
#define	__enable_interrupt()	__enable_irq()


#define INVALID_INTERFACE   ((void *)-1)

#define sysDelay(a)          if(osKernelGetState() == osKernelRunning){osDelay(a);}else{ HAL_Delay(a);}

/*
*********************************************************************************************
* EXTERNAL TYPES DECLARATIONS
*********************************************************************************************
*/

/*
*********************************************************************************************
* EXTERNAL VARIABLES DECLARATIONS
*********************************************************************************************
*/
#define SYSCON     (FILE*)&huart8

#if (ENABLE_CAMERA_OV5640 == 1)
  #define PAYLOAD  INVALID_INTERFACE
#else
  #define PAYLOAD  (FILE*)&huart7
#endif

#if (ENABLE_GNSS_OEM719 == 1)
  #define COMM     INVALID_INTERFACE
  #define COM_GNSS (FILE*)&huart5
#else
  #define COMM     (FILE*)&huart5
  #define COM_GNSS INVALID_INTERFACE
#endif

/**
 * @brief RTC calendar format
 */
#define CALENDAR_FORMAT RTC_FORMAT_BIN

/*
*********************************************************************************************
* EXTERNAL ROUTINES DECLARATIONS 
*********************************************************************************************
*/
void Error_Handler(void);

#endif    /* MAIN_H */
/* **************************************************************************************** */
