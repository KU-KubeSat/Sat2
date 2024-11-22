/*!
********************************************************************************************
* @file ExtFram.c
* @brief Driver for external 2-Mbit (256 K × 8) FRAM CY15B102Q
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

/*
********************************************************************************************
* INCLUDES
********************************************************************************************
*/
#include <ExtFram.h>          /* Own header file */
#include <string.h>           /* memcpy, memcmp */
#include "cmsis_os.h"

/*
********************************************************************************************
* INTERNAL DEFINES
********************************************************************************************
*/
#define EXT_FRAM_RX_BUFF_SIZE   (256)
#define EXT_FRAM_MAX_RETRY      (3)

/*
********************************************************************************************
* INTERNAL TYPES DEFINITION
********************************************************************************************
*/
/* No Internal types definition */

/*
********************************************************************************************
* EXTERNAL VARIABLES DEFINITION
********************************************************************************************
*/
/* No External variables definition */

/**
* @brief Variable description
*/


/*
********************************************************************************************
* INTERNAL (STATIC) VARIABLES DEFINITION/DECLARATION 
********************************************************************************************
*/
static uint8_t ExtFram_RxBuffer[2][EXT_FRAM_RX_BUFF_SIZE];
static uint8_t ExtFram_TxBuffer[EXT_FRAM_RX_BUFF_SIZE + 4];

/*
********************************************************************************************
* INTERNAL (STATIC) ROUTINES DECLARATION
********************************************************************************************
*/
static void ExtFram_Transfer(uint8_t * txBuffer, uint8_t txSize, uint8_t * rxBuffer, uint8_t rxSize);
static uint8_t ExtFram_SafeRead(uint8_t * txBuffer, uint8_t txSize, uint8_t * rxBuffer, uint8_t rxSize);

/*
********************************************************************************************
* EXTERNAL (NONE STATIC) ROUTINES DEFINITION
********************************************************************************************
*/
/*!
********************************************************************************************
* @brief Init routine for the ExtFram component
********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
********************************************************************************************
*/
void ExtFram_Init(void)
{
    MCU_ExtFlashInit();
}

/*!
********************************************************************************************
* @brief Enabled writing latch (disable the write protection)
********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
********************************************************************************************
*/
void FRAM_WriteEnable(void)
{
    uint8_t TxData = 0x06; //Opcode RDSR (Read status register)
    uint8_t RxData;

    (void)ExtFram_Transfer(&TxData, 1, &RxData, 0);
}


/*!
********************************************************************************************
* @brief Disabled writing latch (Enable the write protection)
********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
********************************************************************************************
*/
void FRAM_WriteDisable(void)
{
    uint8_t TxData = 0x04; //Opcode RDSR (Read status register)
    uint8_t RxData;

    (void)ExtFram_Transfer(&TxData, 1, &RxData, 0);
}

/*!
********************************************************************************************
* @brief Write the status register
********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
********************************************************************************************
*/
void FRAM_WriteStatusReg(uint8_t status)
{
    uint8_t TxData[2] = {0x01, status}; //Opcode RDSR (Read status register)
    uint8_t RxData;

    (void)ExtFram_Transfer(TxData, 2, &RxData, 0); //VMI_TODO_EXT_FRAM Use safe write
}


/*!
********************************************************************************************
* @brief Read the status register of the External FRAM
********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            uint8_t - returns the value of the status register
* @note              none
********************************************************************************************
*/
uint8_t FRAM_ReadStatusReg(void)
{
    uint8_t TxData = 0x05; //Opcode RDSR (Read status register)
    uint8_t RxData = 0;

    (void)ExtFram_SafeRead(&TxData, 1, &RxData, 1);

    return RxData;
}


/*!
********************************************************************************************
* @brief Read data
********************************************************************************************
* @param[input]      address
* @param[input]      size
* @param[output]     dataBuff
* @return            none
* @note              none
********************************************************************************************
*/
void FRAM_ReadData(uint32_t address,uint8_t size, uint8_t * dataBuff)
{
    uint8_t TxData[4] = {0x03, (uint8_t)(address >> 16), (uint8_t)(address >> 8), (uint8_t)(address)}; //Opcode READ (Read memory data)

    (void)ExtFram_SafeRead(TxData, 4, dataBuff, size);
}

/*!
********************************************************************************************
* @brief Fast Read data
********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
********************************************************************************************
*/
void FRAM_FastReadData(uint8_t size, uint8_t * dataBuff)
{
    //VMI_TODO_EXT_FRAM FRAM fast read not needed for now
}

/*!
********************************************************************************************
* @brief Write data
********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
********************************************************************************************
*/
void FRAM_WriteData(uint32_t address,uint8_t size, uint8_t * dataBuff)
{
    uint8_t TxData[4] = {0x02, (uint8_t)(address >> 16), (uint8_t)(address >> 8), (uint8_t)(address)}; //Opcode READ (Read memory data)

    memcpy(ExtFram_TxBuffer,TxData, 4);
    memcpy(&ExtFram_TxBuffer[4],dataBuff, size);

    FRAM_WriteEnable();

    for(uint8_t i = 0; i < EXT_FRAM_MAX_RETRY; i++)
    {
        (void)ExtFram_Transfer(ExtFram_TxBuffer, 4+size, dataBuff, 0);

        FRAM_ReadData(address,size, ExtFram_RxBuffer[1]);

        if( memcmp(&ExtFram_TxBuffer[4], ExtFram_RxBuffer[1], size) == 0)
        {
            break;
        }
    }

    FRAM_WriteDisable();
}

/*!
********************************************************************************************
* @brief Enter Sleep Mode
********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
********************************************************************************************
*/
void FRAM_EnSleep(void)
{
    uint8_t TxData = 0xB9; //Opcode SLEEP (Enter sleep mode)
    uint8_t RxData = 0;

    (void)ExtFram_Transfer(&TxData, 1, &RxData, 0);
}

/*!
********************************************************************************************
* @brief Wake up (send dumy read and wait 450 �s)
********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
********************************************************************************************
*/
void FRAM_Wake(void)
{
    (void)FRAM_ReadStatusReg();
    osDelay(1);
}

/*!
********************************************************************************************
* @brief Read the Device ID of the External FRAM
********************************************************************************************
* @param[input]      none
* @param[output]     buffer - read the ID into the buffer
* @return            none
* @note              none
********************************************************************************************
*/
void FRAM_Read_DevID(uint8_t * buffer)
{
#define FRAM_DEV_ID_LENTH (9)

    uint8_t TxData = 0x9F; //Opcode RDSR (Read status register)

    (void)ExtFram_Transfer(&TxData, 1, buffer, FRAM_DEV_ID_LENTH);
}


/*
********************************************************************************************
* INTERNAL (STATIC) ROUTINES DEFINITION
********************************************************************************************
*/

/*!
********************************************************************************************
* @brief Read the Device ID of the External FRAM
********************************************************************************************
* @param[input]      txBuffer - write data
* @param[input]      txSize - number of bytes to send
* @param[input]      rxSize - number of bytes to receive
* @param[output]     rxBuffer - read data
* @return            uint8_t:
*                       + 1 - OK
*                       - 0 - Failed
* @note              none
********************************************************************************************
*/
static void ExtFram_Transfer(uint8_t * txBuffer, uint8_t txSize, uint8_t * rxBuffer, uint8_t rxSize)
{
    HAL_GPIO_WritePin(SPI4_FRAM_CHIP_SELECT_GPIO_Port, SPI4_FRAM_CHIP_SELECT_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi4, txBuffer, txSize, 50); //VMI_TODO_EXT_FRAM -> check for max timeout ( 50 is set just like that )
    HAL_SPI_Receive(&hspi4, rxBuffer, rxSize, 50);
    HAL_GPIO_WritePin(SPI4_FRAM_CHIP_SELECT_GPIO_Port, SPI4_FRAM_CHIP_SELECT_Pin, GPIO_PIN_SET);
}

/*!
********************************************************************************************
* @brief Read the Device ID of the External FRAM
********************************************************************************************
* @param[input]      txBuffer - write data
* @param[input]      txSize - number of bytes to send
* @param[input]      rxSize - number of bytes to receive
* @param[output]     rxBuffer - read data
* @return            uint8_t:
*                       + 1 - OK
*                       - 0 - Failed
* @note              none
********************************************************************************************
*/
static uint8_t ExtFram_SafeRead(uint8_t * txBuffer, uint8_t txSize, uint8_t * rxBuffer, uint8_t rxSize)
{
    int readStatus;
    uint8_t retStat = 0;

    if(rxSize <= EXT_FRAM_RX_BUFF_SIZE)
    {
        for(uint8_t i = 0; i < EXT_FRAM_MAX_RETRY; i++)
        {
            ExtFram_Transfer(txBuffer, txSize, ExtFram_RxBuffer[0], rxSize);

            if(rxSize <= 0)
            {
                // if no bytes are going to be read, there is nothing to compare
                break;
            }

            ExtFram_Transfer(txBuffer, txSize, ExtFram_RxBuffer[1], rxSize);

            readStatus = memcmp(ExtFram_RxBuffer[0], ExtFram_RxBuffer[1], rxSize);

            if(readStatus == 0)
            {
                memcpy(rxBuffer, ExtFram_RxBuffer[0], rxSize);
                retStat = 1;
                break;
            }
        }
    }

    return retStat;
}

/* ******************************************************************************************* */
