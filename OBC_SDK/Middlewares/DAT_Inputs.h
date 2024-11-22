/*!
********************************************************************************************
* @file DAT_Inputs.h
* @brief Header of DAT_Inputs.c
********************************************************************************************
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
********************************************************************************************
*/
#ifndef DAT_INPUTS_H
#define DAT_INPUTS_H

/*
*********************************************************************************************
* INCLUDES
*********************************************************************************************
*/
#include "User_types.h"
#include <stdint.h>

/*
*********************************************************************************************
* EXTERNAL DEFINES
*********************************************************************************************
*/

#define LIS3MDL_4GAUSS_LSB  ((double)(1.00/6842.00))
#define LIS3MDL_8GAUSS_LSB  ((double)(1.00/3421.00))
#define LIS3MDL_12GAUSS_LSB ((double)(1.00/2281.00))
#define LIS3MDL_16GAUSS_LSB ((double)(1.00/1711.00))

#define LIS3MDL_CONV_GAUSS_TO_TESLA ((double)0.0001)
#define LIS3MDL_CONV_GAUSS_TO_nTESLA ((double)100000)

/*
*********************************************************************************************
* EXTERNAL TYPES DECLARATIONS
*********************************************************************************************
*/
typedef enum
{
    OBCSENSORID_ACCELEROMETER,
    OBCSENSORID_MAGNETOMETER,
    OBCSENSORID_GYROSCOPE,
    OBCSENSORID_MAGNETORQUER,
    OBCSENSORID_TEMPERATURE_SENSOR,
    OBCSENSORID_SUN_SENSOR,
    OBCSENSORID_MAX
} ObcSensorId_t;

typedef enum tag_MagnetoDataFormat
{
    MAGNETODATAFORMAT_RAW,
    MAGNETODATAFORMAT_GAUSS,
    MAGNETODATAFORMAT_nTESLA,
    MAGNETODATAFORMAT_TESLA,
    MAGNETODATAFORMAT_MAX
} eMagnetoDataFormat_t;

/*
*********************************************************************************************
* EXTERNAL VARIABLES DECLARATIONS
*********************************************************************************************
*/
/* No External variables declarations */

/*
*********************************************************************************************
* EXTERNAL ROUTINES DECLARATIONS 
*********************************************************************************************
*/
ES_ReturnType Magnitometers_LIS3MDL_Init(uint8_t Dev_No);
ES_ReturnType Magnitometers_LIS3MDL_Read_Data (Compass_Axis_t *MagOutData, uint8_t Dev_No, eMagnetoDataFormat_t u8DataFormat);

void sensorSem_Take(ObcSensorId_t sensorId);
void sensorSem_Release(ObcSensorId_t sensorId, uint8_t FullForceEnable);
uint8_t sensorSem_ReadState(ObcSensorId_t sensorId);

//
//  Legacy ESTTC macros (to be removed when ESTTC code is refactored to use the new interfaces above)
//
#define SunSens_Take()                             sensorSem_Take(OBCSENSORID_SUN_SENSOR)
#define SunSens_Release(FullForceEnable)           sensorSem_Release(OBCSENSORID_SUN_SENSOR, FullForceEnable)
#define SunSens_ReadState()                        sensorSem_ReadState(OBCSENSORID_SUN_SENSOR)

#define Magnetometers_Take()                       sensorSem_Take(OBCSENSORID_MAGNETOMETER)
#define Magnetometers_Release(FullForceEnable)     sensorSem_Release(OBCSENSORID_MAGNETOMETER, FullForceEnable)
#define Magnetometers_ReadState()                  sensorSem_ReadState(OBCSENSORID_MAGNETOMETER)


#define Gyro_Take()                                sensorSem_Take(OBCSENSORID_GYROSCOPE)
#define Gyro_Release(FullForceEnable)              sensorSem_Release(OBCSENSORID_GYROSCOPE, FullForceEnable)
#define Gyro_ReadState()                           sensorSem_ReadState(OBCSENSORID_GYROSCOPE)

#define PanTempSens_Take()                         sensorSem_Take(OBCSENSORID_TEMPERATURE_SENSOR)
#define PanTempSens_Release(FullForceEnable)       sensorSem_Release(OBCSENSORID_TEMPERATURE_SENSOR, FullForceEnable)
#define PanTempSens_ReadState()                    sensorSem_ReadState(OBCSENSORID_TEMPERATURE_SENSOR)

#define MagnTrq_Take()                             sensorSem_Take(OBCSENSORID_MAGNETORQUER)
#define MagnTrq_Release(FullForceEnable)           sensorSem_Release(OBCSENSORID_MAGNETORQUER, FullForceEnable)
#define MagnTrq_ReadState()                        sensorSem_ReadState(OBCSENSORID_MAGNETORQUER)

#endif    /* DAT_INPUTS_H */
/* **************************************************************************************** */
