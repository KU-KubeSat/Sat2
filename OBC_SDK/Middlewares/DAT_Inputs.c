/*!
********************************************************************************************
* @file DAT_Inputs.c
* @brief Input data (data from sensors)
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

/*
*********************************************************************************************
* INCLUDES
*********************************************************************************************
*/
#include "User_types.h"
#include "DAT_Inputs.h"
#include "panels.h"
#include "cmsis_os2.h"
#include "MCU_Init.h"
#include "LIS3MDL_MAG_driver.h"

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
/* No External variables definition */

/*
*********************************************************************************************
* INTERNAL (STATIC) VARIABLES DEFINITION/DECLARATION 
*********************************************************************************************
*/
static uint8_t countSemaphores[OBCSENSORID_MAX] = { (uint8_t) 0 };

/*
*********************************************************************************************
* INTERNAL (STATIC) ROUTINES DECLARATION
*********************************************************************************************
*/
static void sensorInit(ObcSensorId_t sensorId)
{
    switch (sensorId)
    {
        case OBCSENSORID_GYROSCOPE:
            MCU_GyrosInit();
        break;

        case OBCSENSORID_MAGNETOMETER:
            MCU_MagnetometersInit();
        break;

        case OBCSENSORID_MAGNETORQUER:
            MCU_MagnetorqersInit();
        break;

        case OBCSENSORID_TEMPERATURE_SENSOR:
            MCU_TemprInit();
            osDelay(TMP122_CONVERSION_TIME);
        break;

        case OBCSENSORID_SUN_SENSOR:
            MCU_SunSensInit();
        break;

        default:
        break;
    }
}

static void sensorDeinit(ObcSensorId_t sensorId)
{
    switch (sensorId)
    {
        case OBCSENSORID_GYROSCOPE:
            MCU_GyrosDeInit();
        break;

        case OBCSENSORID_MAGNETOMETER:
            MCU_MagnetometersDeInit();
        break;

        case OBCSENSORID_MAGNETORQUER:
            MCU_MagnetorqersDeInit();
        break;

        case OBCSENSORID_TEMPERATURE_SENSOR:
            MCU_TemprDeInit();
        break;

        case OBCSENSORID_SUN_SENSOR:
            MCU_SunSensDeInit();
        break;

        default:
        break;
    }
}

/*
*********************************************************************************************
* EXTERNAL (NONE STATIC) ROUTINES DEFINITION
*********************************************************************************************
*/

/*!
*********************************************************************************************
* @brief Initialises LIS3MDL magnetometer component
*********************************************************************************************
* @param[input]      uint8_t Dev_No - magnetometer module No
* @param[output]     none
* @return            E_INV_CFG or E_OK
* @note              Magnetometers at power-up:
*                       - Operating mode (OM0 and OM1 ; OMZ0 and OMZ1; all are zeros): low-power
*                       - Output data rate (CTRL_REG1, DO2 only set): 10Hz
*                       - Measurement mode (MD0, MD1 both set): IDLE mode
*********************************************************************************************
*/
ES_ReturnType Magnitometers_LIS3MDL_Init(uint8_t Dev_No) {
    uint8_t val;  
    ES_ReturnType ESRetValue = E_OK; 

    // Turn on device and change measurement mode to continuous
    if ( !(LIS3MDL_MAG_W_SystemOperatingMode (Dev_No, LIS3MDL_MAG_MD_CONTINUOUS)))
        return E_INV_CFG;

    // Update output value 10 times per second
    if ( !(LIS3MDL_MAG_W_OutputDataRate (Dev_No, LIS3MDL_MAG_DO_10Hz)))
        return E_INV_CFG;

    // Set range to 4 Gauss
    if ( !(LIS3MDL_MAG_W_FullScale(Dev_No, LIS3MDL_MAG_FS_4Ga )))
        return E_INV_CFG;

    // Set operating mode to ultra-high performance for the X and Y axis ... to get smallest RMS noise figure
    if ( !(LIS3MDL_MAG_W_OperatingModeXY(Dev_No, LIS3MDL_MAG_OM_ULTRA_HIGH )))
        return E_INV_CFG;

    // Set operating mode to ultra-high performance for the Z axis ... to get smallest RMS noise figure
    if ( !(LIS3MDL_MAG_W_OperatingModeZ(Dev_No, LIS3MDL_MAG_OMZ_ULTRA_HIGH )))
        return E_INV_CFG;

    // Block data update
    if ( !(LIS3MDL_MAG_W_BlockDataUpdate(Dev_No,LIS3MDL_MAG_BDU_ENABLE)))
        return E_INV_CFG;

    // Check status
    if ( !(LIS3MDL_MAG_R_WHO_AM_I_(Dev_No, &val )))
        return E_INV_CFG;

    // invalid configuration if device magnetometer answered with id different than 0x3D
    if (val ^ 0x3D)
        return E_INV_CFG;

    return ESRetValue;
}

/*!
*********************************************************************************************
* @brief Reads magnetometer data from selected LIS3MDL magnetometer component
*********************************************************************************************
* @param[input]      uint8_t Dev_No - magnetometer module No
* @param[output]     Compass_Axis_t *MagOutData - magnetometer data vector
* @return            E_INV_CFG or E_OK
* @note              none
*********************************************************************************************
*/
ES_ReturnType Magnitometers_LIS3MDL_Read_Data (Compass_Axis_t *MagOutData, uint8_t Dev_No, eMagnetoDataFormat_t u8DataFormat) {
  
  int16_t pDataRaw[3];

  ES_ReturnType ESRetValue = E_OK;   
  
  uint8_t regValue[6] = {0, 0, 0, 0, 0, 0};
  
  if ( !(LIS3MDL_MAG_Get_Magnetic(Dev_No, (uint8_t*)regValue )))
   return E_INV_CFG;   

  /* Format the data. */
  pDataRaw[0] = ( ( ( ( int16_t )regValue[1] ) << 8 ) + ( int16_t )regValue[0] );
  pDataRaw[1] = ( ( ( ( int16_t )regValue[3] ) << 8 ) + ( int16_t )regValue[2] );
  pDataRaw[2] = ( ( ( ( int16_t )regValue[5] ) << 8 ) + ( int16_t )regValue[4] );

  if ( MAGNETODATAFORMAT_GAUSS == u8DataFormat )
  {
      MagOutData->AXIS_X = ( pDataRaw[0] * LIS3MDL_4GAUSS_LSB );
      MagOutData->AXIS_Y = ( pDataRaw[1] * LIS3MDL_4GAUSS_LSB );
      MagOutData->AXIS_Z = ( pDataRaw[2] * LIS3MDL_4GAUSS_LSB );
  }
  else
  if ( MAGNETODATAFORMAT_nTESLA == u8DataFormat )
  {
      MagOutData->AXIS_X = ( pDataRaw[0] * LIS3MDL_4GAUSS_LSB ) * LIS3MDL_CONV_GAUSS_TO_nTESLA;
      MagOutData->AXIS_Y = ( pDataRaw[1] * LIS3MDL_4GAUSS_LSB ) * LIS3MDL_CONV_GAUSS_TO_nTESLA;
      MagOutData->AXIS_Z = ( pDataRaw[2] * LIS3MDL_4GAUSS_LSB ) * LIS3MDL_CONV_GAUSS_TO_nTESLA;
  }
  else
  if ( MAGNETODATAFORMAT_TESLA == u8DataFormat )
  {
      MagOutData->AXIS_X = ( pDataRaw[0] * LIS3MDL_4GAUSS_LSB ) * LIS3MDL_CONV_GAUSS_TO_TESLA;
      MagOutData->AXIS_Y = ( pDataRaw[1] * LIS3MDL_4GAUSS_LSB ) * LIS3MDL_CONV_GAUSS_TO_TESLA;
      MagOutData->AXIS_Z = ( pDataRaw[2] * LIS3MDL_4GAUSS_LSB ) * LIS3MDL_CONV_GAUSS_TO_TESLA;
  }
  else
  {
      MagOutData->AXIS_X = ( pDataRaw[0] );
      MagOutData->AXIS_Y = ( pDataRaw[1] );
      MagOutData->AXIS_Z = ( pDataRaw[2] );
  }

  return ESRetValue;
  
}

void sensorSem_Take(ObcSensorId_t sensorId)
{
    if (sensorId < sizeof(countSemaphores) / sizeof(countSemaphores[0]))
        if(countSemaphores[sensorId] < 255)
        {
            // if this is the first time we use the sensor, initialise related
            // peripherals
            if (countSemaphores[sensorId] == 0)
                sensorInit(sensorId);

            ++countSemaphores[sensorId];
        }
}

void sensorSem_Release(ObcSensorId_t sensorId, uint8_t FullForceEnable)
{
    if (sensorId < sizeof(countSemaphores) / sizeof(countSemaphores[0]))
    {
        if (FullForceEnable == 0)
        {
            if (countSemaphores[sensorId] > 0)
                --countSemaphores[sensorId];
        }
        else
            countSemaphores[sensorId] = 0;

        // if there are no more users of this sensor, de-initialise
        // related peripherals to save power
        if (countSemaphores[sensorId] == 0)
            sensorDeinit(sensorId);
    }
}

uint8_t sensorSem_ReadState(ObcSensorId_t sensorId)
{
    return (sensorId < sizeof(countSemaphores) / sizeof(countSemaphores[0])) ?
           (countSemaphores[sensorId]) : 0;
}

/*
*********************************************************************************************
* INTERNAL (STATIC) ROUTINES DEFINITION
*********************************************************************************************
*/
/* No Internal routines definition */

/* **************************************************************************************** */
