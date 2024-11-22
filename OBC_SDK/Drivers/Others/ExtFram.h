/*!
********************************************************************************************
* @file ExtFram.h
* @brief Header of ExtFram.c
********************************************************************************************
* @author            Vassil Milev
* @version           1.0.0
* @date              2020.09.02
*
* @copyright         (C) Copyright Endurosat
*
*                    Contents and presentations are protected world-wide.
*                    Any kind of using, copying etc. is prohibited without prior permission.
*                    All rights - incl. industrial property rights - are reserved.
*
* @history
* @revision{         1.0.0  , 2020.09.02, author Vassil Milev, Initial revision }
* @endhistory
********************************************************************************************
*/
#ifndef EXT_FRAM_H
#define EXT_FRAM_H

/*
********************************************************************************************
* INCLUDES
********************************************************************************************
*/
#include <main.h>          /* Standard types */

/*
********************************************************************************************
* EXTERNAL DEFINES
********************************************************************************************
*/
/* No External defines*/

/*
********************************************************************************************
* EXTERNAL TYPES DECLARATIONS
********************************************************************************************
*/
/* No External types declarations */

/*
********************************************************************************************
* EXTERNAL VARIABLES DECLARATIONS
********************************************************************************************
*/
/* No External variables declarations */

/*
********************************************************************************************
* EXTERNAL ROUTINES DECLARATIONS 
********************************************************************************************
*/
void ExtFram_Init(void);
void FRAM_WriteEnable(void);
void FRAM_WriteDisable(void);
void FRAM_WriteStatusReg(uint8_t status);
uint8_t FRAM_ReadStatusReg(void);
void FRAM_ReadData(uint32_t address,uint8_t size, uint8_t * dataBuff);
void FRAM_FastReadData(uint8_t size, uint8_t * dataBuff);
void FRAM_WriteData(uint32_t address,uint8_t size, uint8_t * dataBuff);
void FRAM_EnSleep(void);
void FRAM_Wake(void);
void FRAM_Read_DevID(uint8_t * buffer);

#endif    /* EXT_FRAM_H */
/* ******************************************************************************************* */
