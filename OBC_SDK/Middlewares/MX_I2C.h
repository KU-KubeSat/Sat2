/*!
*********************************************************************************************
* @file MX_I2C.h
* @brief Header of MX_I2C.c
*********************************************************************************************
* @author            Vassil Milev
* @version           1.0.0
* @date              2020.11.14
*
* @copyright         (C) Copyright Endurosat
*
*                    Contents and presentations are protected world-wide.
*                    Any kind of using, copying etc. is prohibited without prior permission.
*                    All rights - incl. industrial property rights - are reserved.
*
* @history
* @revision{         1.0.0  , 2020.11.14, author Vassil Milev, Initial revision }
* @endhistory
*********************************************************************************************
*/
#ifndef MX_I2C_H
#define MX_I2C_H

/*
*********************************************************************************************
* INCLUDES
*********************************************************************************************
*/
#include <main.h>          /* Standard types */
#include "cmsis_os.h"

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
typedef enum
{
    MX_I2C_BUS_SYSTEM,
    MX_I2C_BUS_SENSORS,
    MX_I2C_BUS_PAYLOAD,
    MX_I2C_BUS_COUNT
} MX_I2Cbus_enum;

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
void MX_I2C_Init(void);
void MX_I2C_DeInit(void);

HAL_StatusTypeDef MX_I2C_Take(MX_I2Cbus_enum bus, TickType_t maxTicksToWait);
void MX_I2C_Release(MX_I2Cbus_enum bus);

HAL_StatusTypeDef MX_I2C_BlockingTransmit(MX_I2Cbus_enum bus, uint16_t devAddress, uint8_t *pData, uint16_t size);
HAL_StatusTypeDef MX_I2C_BlockingReceive(MX_I2Cbus_enum bus, uint16_t devAddress, uint8_t *pData, uint16_t size);
HAL_StatusTypeDef MX_I2C_BlockingMemWrite(MX_I2Cbus_enum bus, uint16_t devAddress, uint16_t writeAddr, uint16_t wAddrSize, uint8_t *pData, uint16_t size);
HAL_StatusTypeDef MX_I2C_BlockingMemRead(MX_I2Cbus_enum bus, uint16_t devAddress, uint16_t writeAddr, uint16_t wAddrSize, uint8_t *pData, uint16_t size);

void MX_I2C_BusInit(MX_I2Cbus_enum bus);
void MX_I2C_BusDeInit(MX_I2Cbus_enum bus);
void MX_I2C_BusReset(MX_I2Cbus_enum bus);


void SYSTEM_I2C_Reset(void);
void SENSORS_I2C_Reset(void);
void PAYLOAD_I2C_Reset(void);

// Daniel Added this
//static I2C_HandleTypeDef *MX_I2C_GetI2cHandler(MX_I2Cbus_enum bus);

//HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c);
//HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c);

#endif    /* MX_I2C_H */
/* ******************************************************************************************* */
