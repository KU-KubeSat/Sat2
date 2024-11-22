/*!
********************************************************************************************
* @file UHF.c
* @brief UHF
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
* @revision{         1.0.0  , 2020.06.23, author Vassil Milev, Initial revision }
* @endhistory
********************************************************************************************
*/

/*
********************************************************************************************
* INCLUDES
********************************************************************************************
*/
#include <UHF.h>            /* Include own header file */

#if (ENABLE_UHF_II_SUPPORT == 1)

#include "cmsis_os.h"
#include "main.h"
#include "string.h"
#include "es_crc32.h"
#include "MX_I2C.h"
#include "es_exeh.h"        /* Include DTC error handling */
#include "TaskMonitor.h"
#include "semphr.h"

/*
********************************************************************************************
* INTERNAL DEFINES
********************************************************************************************
*/
#define UHF_I2C_TIME_BETWEEN_PACS   (3)
#define UHF_I2C_TRANSMITION_RETRY   (5)


/*
********************************************************************************************
* INTERNAL TYPES DEFINITION
********************************************************************************************
*/
typedef struct
{
    uint8_t initalStr[3];
    uint8_t opperation;
    uint8_t address[2];
    uint8_t command[2];
    uint8_t AddCRCString[11];
} __attribute__((__packed__)) UHF_TxReadCmd_struct;

typedef struct
{
    uint8_t msb;
    uint8_t lsb;
}UHF_EsttcCmdValues;

//Keep alligned with UHFcmdsRead_Enum
static const UHF_EsttcCmdValues UHF_EsttcReadCmdNumb[UHF_CMD_READ_STOPPER] =
{
    /* UHF_CMD_READ_RSSI       */ {'0','0'},
    /* UHF_CMD_READ_SCW        */ {'0','0'},
    /* UHF_CMD_READ_UPTIME     */ {'0','2'},
    /* UHF_CMD_READ_TX_PACKETS */ {'0','3'},
    /* UHF_CMD_READ_RX_PACKETS */ {'0','4'},
    /* UHF_CMD_READ_CRC_ERR    */ {'0','5'},
    /* UHF_CMD_READ_TEMP_SENS  */ {'0','A'}
};

//Keep alligned with UHFcmdsWrite_Enum
static const UHF_EsttcCmdValues UHF_EsttcWriteCmdNumb[UHF_CMD_WRITE_STOPPER] =
{
    /* UHF_CMD_WRITE_SCW        */ {'0','0'},
    /* UHF_CMD_WRITE_BEACON     */ {'E','F'},
};


/*
********************************************************************************************
* EXTERNAL VARIABLES DEFINITION
********************************************************************************************
*/


/*
********************************************************************************************
* INTERNAL (STATIC) VARIABLES DEFINITION/DECLARATION
********************************************************************************************
*/
UHFactive_Enum UHF_activeUhfNumber;

static const uint8_t UHF_address[UHF_II_STOPPER] = UHF_ADDRESSES_CONFIG;
static uint16_t BCN_UHF_Scw;
static uint32_t BCN_UHF_Uptime;
static uint32_t BCN_UHF_TxPacks;
static uint32_t BCN_UHF_RxPacks;
static uint32_t BCN_UHF_CRCErr;
static int16_t  BCN_UHF_TemperatureRaw;
static uint8_t  BCN_UHF_Temperature[6];


static uint8_t uhfTxRxBuffer[128];   // max beacon size + command + CRC


/*
********************************************************************************************
* INTERNAL (STATIC) ROUTINES DECLARATION
********************************************************************************************
*/

//////////////////////////////////////////////////////////////
static uint8_t UHF_HexToBin(uint8_t hb, uint8_t lb);
//static void UHF_BinToHex(uint8_t hexNumb, uint8_t *hb, uint8_t *lb);
static UHFcmdStatus_Enum BCN_ReadUHF_Data(const UHFcmdsRead_Enum command);
static UHFcmdStatus_Enum BCN_WriteUHF_Data(const UHFcmdsWrite_Enum command,const void * data,const uint8_t size);


/*
********************************************************************************************
* EXTERNAL (NONE STATIC) ROUTINES DEFINITION
********************************************************************************************
*/
/*!
*********************************************************************************************
* @brief Init routine for the UHF component
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void UHF_Init(void)
{
    UHF_activeUhfNumber = UHF_II_NUMBER_A;
}

void UHF_SelectActive(UHFactive_Enum uhfNumber)
{
    if (uhfNumber < UHF_II_STOPPER)
    {
        UHF_activeUhfNumber = uhfNumber;
    }
}


/*!
*********************************************************************************************
* @brief Send beacon instant message
*********************************************************************************************
* @param[input]      data  - data to be transmitted as a beacon
* @param[output]     size - number of bytes to be send
*
* @return            UHFcmdStatus_Enum - status of the reading command
* @note              none
*********************************************************************************************
*/
UHFcmdStatus_Enum UHF_Set_Beacon(uint8_t *data)   //VMI_TODO - test when the fix in UHF is ready
{
    UHFcmdStatus_Enum retVal;

    retVal = BCN_WriteUHF_Data(UHF_CMD_WRITE_BEACON, data, 0);

    return retVal;
}


/////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Read ///////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*!
*********************************************************************************************
* @brief Read the Status Control Word of the UHF
*********************************************************************************************
* @param[input]      ReadType  + 0 - Get last read value from the buffer.
*                              + 1 - Read the registers from the UHF and refresh the values from the buffer
* @param[output]     Scw - the value that was just read
*
* @return            UHFcmdStatus_Enum - status of the reading command
* @note              none
*********************************************************************************************
*/
UHFcmdStatus_Enum UHF_Get_SCW(uint8_t ReadType, uint16_t *Scw)
{
    static UHFcmdStatus_Enum retVal = UHF_CMD_STAT_TIMEOUT; // if online read, return the last read state

    if (ReadType == pdTRUE)
    {
        retVal = BCN_ReadUHF_Data(UHF_CMD_READ_SCW);

        if (UHF_CMD_STAT_COMPLETED_OK == retVal)
        {
            if (Scw != NULL)
                *Scw = BCN_UHF_Scw;
        }
    }
    else
    {
        if (Scw != NULL)
        {
            *Scw = BCN_UHF_Scw;
            retVal = UHF_CMD_STAT_COMPLETED_OK;
        }
        else
        {
            retVal = UHF_CMD_STAT_ERROR;
        }
    }

    return retVal;
}

/*!
*********************************************************************************************
* @brief Read the time since power up of the UHF
*********************************************************************************************
* @param[input]      ReadType  + 0 - Get last read value from the buffer.
*                              + 1 - Read the registers from the UHF and refresh the values from the buffer
* @param[output]     Uptime - the value that was just read
*
* @return            UHFcmdStatus_Enum - status of the reading command
* @note              none
*********************************************************************************************
*/
UHFcmdStatus_Enum UHF_Get_Uptime(uint8_t ReadType, uint32_t *Uptime)
{
    static UHFcmdStatus_Enum retVal = UHF_CMD_STAT_TIMEOUT; // if online read, return the last read state

    if (ReadType == pdTRUE)
    {
        retVal = BCN_ReadUHF_Data(UHF_CMD_READ_UPTIME);

        if (UHF_CMD_STAT_COMPLETED_OK == retVal)
        {
            if (Uptime != NULL)
                *Uptime = BCN_UHF_Uptime;
        }
    }
    else
    {
        if (Uptime != NULL)
        {
            *Uptime = BCN_UHF_Uptime;
            retVal = UHF_CMD_STAT_COMPLETED_OK;
        }
        else
        {
            retVal = UHF_CMD_STAT_ERROR;
        }
    }

    return retVal;
}

/*!
*********************************************************************************************
* @brief Read the transmitted number of packets of the UHF
*********************************************************************************************
* @param[input]      ReadType  + 0 - Get last read value from the buffer.
*                              + 1 - Read the registers from the UHF and refresh the values from the buffer
* @param[output]     TxPackets - the value that was just read
*
* @return            UHFcmdStatus_Enum - status of the reading command
* @note              none
*********************************************************************************************
*/
UHFcmdStatus_Enum UHF_Get_TxPacks(uint8_t ReadType, uint32_t *TxPackets)
{
    static UHFcmdStatus_Enum retVal = UHF_CMD_STAT_TIMEOUT; // if online read, return the last read state

    if (ReadType == pdTRUE)
    {
        retVal = BCN_ReadUHF_Data(UHF_CMD_READ_TX_PACKETS);

        if (UHF_CMD_STAT_COMPLETED_OK == retVal)
        {
            if (TxPackets != NULL)
                *TxPackets = BCN_UHF_TxPacks;
        }
    }
    else
    {
        if (TxPackets != NULL)
        {
            *TxPackets = BCN_UHF_TxPacks;
            retVal = UHF_CMD_STAT_COMPLETED_OK;
        }
        else
        {
            retVal = UHF_CMD_STAT_ERROR;
        }
    }

    return retVal;
}

/*!
*********************************************************************************************
* @brief Read the received number of packets of the UHF
*********************************************************************************************
* @param[input]      ReadType  + 0 - Get last read value from the buffer.
*                              + 1 - Read the registers from the UHF and refresh the values from the buffer
* @param[output]     RxPackets - the value that was just read
*
* @return            UHFcmdStatus_Enum - status of the reading command
* @note              none
*********************************************************************************************
*/
UHFcmdStatus_Enum UHF_Get_RxPacks(uint8_t ReadType, uint32_t *RxPackets)
{
    static UHFcmdStatus_Enum retVal = UHF_CMD_STAT_TIMEOUT; // if online read, return the last read state

    if (ReadType == pdTRUE)
    {
        retVal = BCN_ReadUHF_Data(UHF_CMD_READ_RX_PACKETS);

        if (UHF_CMD_STAT_COMPLETED_OK == retVal)
        {
            if (RxPackets != NULL)
                *RxPackets = BCN_UHF_RxPacks;
        }
    }
    else
    {
        if (RxPackets != NULL)
        {
            *RxPackets = BCN_UHF_RxPacks;
            retVal = UHF_CMD_STAT_COMPLETED_OK;
        }
        else
        {
            retVal = UHF_CMD_STAT_ERROR;
        }
    }

    return retVal;
}


/*!
*********************************************************************************************
* @brief Read the number of packets with CRC error of the UHF
*********************************************************************************************
* @param[input]      ReadType  + 0 - Get last read value from the buffer.
*                              + 1 - Read the registers from the UHF and refresh the values from the buffer
* @param[output]     CrcErrs - the value that was just read
*
* @return            UHFcmdStatus_Enum - status of the reading command
* @note              none
*********************************************************************************************
*/
UHFcmdStatus_Enum UHF_Get_CrcErrs(uint8_t ReadType, uint32_t *CrcErrs)
{
    static UHFcmdStatus_Enum retVal = UHF_CMD_STAT_TIMEOUT; // if online read, return the last read state

    if (ReadType == pdTRUE)
    {
        retVal = BCN_ReadUHF_Data(UHF_CMD_READ_CRC_ERR);

        if (UHF_CMD_STAT_COMPLETED_OK == retVal)
        {
            if (CrcErrs != NULL)
                *CrcErrs = BCN_UHF_CRCErr;
        }
    }
    else
    {
        if (CrcErrs != NULL)
        {
            *CrcErrs = BCN_UHF_CRCErr;
            retVal = UHF_CMD_STAT_COMPLETED_OK;
        }
        else
        {
            retVal = UHF_CMD_STAT_ERROR;
        }
    }
    return retVal;
}

/*!
*********************************************************************************************
* @brief Read internal temperature sensor of the UHF as a string
*********************************************************************************************
* @param[input]      ReadType  + 0 - Get last read value from the buffer.
*                              + 1 - Read the registers from the UHF and refresh the values from the buffer
* @param[output]     TemperSensors - the value that was just read
*
* @return            UHFcmdStatus_Enum - status of the reading command
* @note              none
*********************************************************************************************
*/
UHFcmdStatus_Enum UHF_Get_TemerSens(uint8_t ReadType, uint8_t *TemperSensors)
{
    static UHFcmdStatus_Enum retVal = UHF_CMD_STAT_TIMEOUT; // if online read, return the last read state

    if (ReadType == pdTRUE)
    {
        retVal = BCN_ReadUHF_Data(UHF_CMD_READ_TEMP_SENS);

        if (UHF_CMD_STAT_COMPLETED_OK == retVal)
        {
            if (TemperSensors != NULL)
                memcpy(TemperSensors, BCN_UHF_Temperature, sizeof(BCN_UHF_Temperature));
        }
    }
    else
    {
        if (TemperSensors != NULL)
        {
            memcpy(TemperSensors, BCN_UHF_Temperature, sizeof(BCN_UHF_Temperature));
            retVal = UHF_CMD_STAT_COMPLETED_OK;
        }
        else
        {
            retVal = UHF_CMD_STAT_ERROR;
        }
    }

    return retVal;
}

/*!
*********************************************************************************************
* @brief Read internal temperature sensor of the UHF
*********************************************************************************************
* @param[input]      ReadType  + 0 - Get last read value from the buffer.
*                              + 1 - Read the registers from the UHF and refresh the values from the buffer
* @param[output]     TemperSensorsRaw - the value that was just read
*
* @return            UHFcmdStatus_Enum - status of the reading command
* @note              none
*********************************************************************************************
*/
UHFcmdStatus_Enum UHF_Get_TemerSensRaw(uint8_t ReadType, int16_t *TemperSensorsRaw)
{
    static UHFcmdStatus_Enum retVal = UHF_CMD_STAT_TIMEOUT; // if online read, return the last read state

    if (ReadType == pdTRUE)
    {

        retVal = BCN_ReadUHF_Data(UHF_CMD_READ_TEMP_SENS);

        if (UHF_CMD_STAT_COMPLETED_OK == retVal)
        {
            if (TemperSensorsRaw != NULL)
                *TemperSensorsRaw = BCN_UHF_TemperatureRaw;
        }
    }
    else
    {
        if (TemperSensorsRaw != NULL)
        {
            *TemperSensorsRaw = BCN_UHF_TemperatureRaw;
            retVal = UHF_CMD_STAT_COMPLETED_OK;
        }
        else
        {
            retVal = UHF_CMD_STAT_ERROR;
        }
    }

    return retVal;
}


/*
********************************************************************************************
* INTERNAL (STATIC) ROUTINES DEFINITION
********************************************************************************************
*/

/*!
*********************************************************************************************
* @brief Hexadecimal ASCII to binary number converted
*********************************************************************************************
* @param[input]      hb - high significant part of the byte
*                    lb - low significant part of the byte
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
static uint8_t UHF_HexToBin(uint8_t hb, uint8_t lb)
{
    uint8_t thb = hb, tlb = lb;

    if (thb > '9')
        thb += 9;
    if (tlb > '9')
        tlb += 9;

    return (thb << 4) + (tlb & 0x0f);
}

/*!
*********************************************************************************************
* @brief Read command to UHF (through I2C/UART interface), return after the packet is received
*********************************************************************************************
* @param[input]      command - number of the command
* @param[output]     none
*
* @return            UHFcmdStatus_Enum - status of the reading command
* @note              none
*********************************************************************************************
*/
static UHFcmdStatus_Enum BCN_ReadUHF_Data(const UHFcmdsRead_Enum command)
{
    UHFcmdStatus_Enum retVal = UHF_CMD_STAT_ERROR;
    uint32_t CRC_value_calc1;
    uint8_t temp_reg_buff[26];

    if (command >= UHF_CMD_READ_STOPPER)
    {
        //retStatus = UHF_CMD_STAT_ERROR;
        //requestedEnable = UHF_REQ_CMD_STATE_RESULT_ERROR;

        EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_UHF_COMM, eEXEH_UHF_EXCEPTION_ID_COMM_INVALID_PARAMS_ERROR, __LINE__);
        Error_Handler();
    }
    else
    {
        HAL_StatusTypeDef I2C_stat;

        uint8_t address = UHF_address[UHF_activeUhfNumber];
        uint8_t addressHigh = (address >> 4) + '0';
        uint8_t addressLow = (address & 0xF) + '0';

        UHF_TxReadCmd_struct txCommandBuffer =
        {
            .initalStr =
            {
                'E',
                'S',
                '+'
            },
            .opperation = 'R',
            .address =
            {
                addressHigh,
                addressLow
            }
        };

        I2C_stat = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);

        if (I2C_stat == HAL_OK)
        {
            txCommandBuffer.command[0] = UHF_EsttcReadCmdNumb[command].msb;
            txCommandBuffer.command[1] = UHF_EsttcReadCmdNumb[command].lsb;

            CRC_value_calc1 = crc32(0, (BYTE*)&txCommandBuffer, sizeof(UHF_TxReadCmd_struct) - 11);

            sprintf((char*)&txCommandBuffer.AddCRCString, " %08X\r", (unsigned int)CRC_value_calc1);

            for (uint8_t i = 0; i < UHF_I2C_TRANSMITION_RETRY; i++)
            {
                //VMI_TODO_1UPLATFORM Send using USART and if not working then I2C -> also receiving
                I2C_stat = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, UHF_I2C_ADDRESS, (uint8_t*)&txCommandBuffer, 18);
                osDelay(UHF_I2C_TIME_BETWEEN_PACS);

                if (I2C_stat == HAL_OK)
                {
                    I2C_stat =
                        MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, UHF_I2C_ADDRESS, &temp_reg_buff[0], 23);
                    osDelay(UHF_I2C_TIME_BETWEEN_PACS); //wait after receiving to prevent to frequent request

                    if ((HAL_OK == I2C_stat) && ((temp_reg_buff[0] == 'O') && (temp_reg_buff[1] == 'K')))
                    {
                        break;
                    }
                }
            }

            if ((HAL_OK == I2C_stat) && ((temp_reg_buff[0] == 'O') && (temp_reg_buff[1] == 'K')))
            {
                switch (command)
                {
                    case UHF_CMD_READ_UPTIME:
                    {
                        //last 8 ( 5 - 13 )
                        BCN_UHF_Uptime = UHF_HexToBin(temp_reg_buff[5], temp_reg_buff[6]) << 24 | UHF_HexToBin(temp_reg_buff[7], temp_reg_buff[8]) << 16 | UHF_HexToBin(temp_reg_buff[9], temp_reg_buff[10]) << 8 | UHF_HexToBin(temp_reg_buff[11], temp_reg_buff[12]) << 0;
                        retVal = UHF_CMD_STAT_COMPLETED_OK;
                    }
                    break;

                    case UHF_CMD_READ_TX_PACKETS:
                    {
                        //last 8 ( 5 - 13 )
                        BCN_UHF_TxPacks = UHF_HexToBin(temp_reg_buff[5], temp_reg_buff[6]) << 24 | UHF_HexToBin(temp_reg_buff[7], temp_reg_buff[8]) << 16 | UHF_HexToBin(temp_reg_buff[9], temp_reg_buff[10]) << 8 | UHF_HexToBin(temp_reg_buff[11], temp_reg_buff[12]) << 0;
                        retVal = UHF_CMD_STAT_COMPLETED_OK;
                    }
                    break;

                    case UHF_CMD_READ_RX_PACKETS:
                    {
                        //last 8 ( 5 - 13 )
                        BCN_UHF_RxPacks = UHF_HexToBin(temp_reg_buff[5], temp_reg_buff[6]) << 24 | UHF_HexToBin(temp_reg_buff[7], temp_reg_buff[8]) << 16 | UHF_HexToBin(temp_reg_buff[9], temp_reg_buff[10]) << 8 | UHF_HexToBin(temp_reg_buff[11], temp_reg_buff[12]) << 0;
                        retVal = UHF_CMD_STAT_COMPLETED_OK;
                    }
                    break;

                    case UHF_CMD_READ_CRC_ERR:
                    {
                        //last 8 ( 5 - 13 )
                        BCN_UHF_CRCErr = UHF_HexToBin(temp_reg_buff[5], temp_reg_buff[6]) << 24 | UHF_HexToBin(temp_reg_buff[7], temp_reg_buff[8]) << 16 | UHF_HexToBin(temp_reg_buff[9], temp_reg_buff[10]) << 8 | UHF_HexToBin(temp_reg_buff[11], temp_reg_buff[12]) << 0;
                        retVal = UHF_CMD_STAT_COMPLETED_OK;
                    }
                    break;

                    case UHF_CMD_READ_TEMP_SENS:
                    {
                        //bytes 3 - 7 (string)
                        memcpy(BCN_UHF_Temperature, &temp_reg_buff[3], 5);
                        BCN_UHF_Temperature[5] = 0; //end character
                        int8_t wholePart = (temp_reg_buff[4] - '0') * 10 + (temp_reg_buff[5] - '0');

                        //convert from string to a integer value
                        if (temp_reg_buff[3] == '-')
                        {
                            wholePart = wholePart * (-1);
                        }
                        uint8_t decimalPart = temp_reg_buff[7] - '0';
                        if (decimalPart > 9)
                            decimalPart = 0;

                        BCN_UHF_TemperatureRaw = (wholePart * 10) + decimalPart;

                        retVal = UHF_CMD_STAT_COMPLETED_OK;
                    }
                    break;

                    default:
                    {
                        //retVal = UHF_CMD_STAT_ERROR;  //Already set
                    }
                    break;
                }
            }
            else
            {
                //retVal = UHF_CMD_STAT_ERROR;  //Already set
            }

            MX_I2C_Release(MX_I2C_BUS_SYSTEM);
        }

        if ((I2C_stat != HAL_OK) || (retVal != UHF_CMD_STAT_COMPLETED_OK))
        {
            EXEH_vException(eEXEHSeverity_Warning, eEXEHModuleID_UHF_COMM, eEXEH_UHF_EXCEPTION_ID_COMM_I2C_ERROR, __LINE__);
        }
    }

    return retVal;
}

/*!
*********************************************************************************************
* @brief Write command to UHF (through I2C/UART interface), return after the packet is received
*********************************************************************************************
* @param[input]      command - number of the command
* @param[input]      data - pointer to the data to be written
* @param[input]      size - number of bytes to be written (needed for commands with not fixed size of data)
* @param[output]     none
*
* @return            UHFcmdStatus_Enum - status of the reading command
* @note              none
*********************************************************************************************
*/
static UHFcmdStatus_Enum BCN_WriteUHF_Data(const UHFcmdsWrite_Enum command, const void * data,const uint8_t size)
{
    UHFcmdStatus_Enum retVal = UHF_CMD_STAT_ERROR;
    uint32_t CRC_value_calc;

    if ((command >= UHF_CMD_WRITE_STOPPER) || ((size + 10 + 10) >= sizeof(uhfTxRxBuffer)))
    {
        //retStatus = UHF_CMD_STAT_ERROR;
        //requestedEnable = UHF_REQ_CMD_STATE_RESULT_ERROR;

        EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_UHF_COMM, eEXEH_UHF_EXCEPTION_ID_COMM_INVALID_PARAMS_ERROR, __LINE__);
        Error_Handler();
    }
    else
    {
        uint8_t address = UHF_address[UHF_activeUhfNumber];
        uint8_t addressHigh = (address >> 4) + '0';
        uint8_t addressLow = (address & 0xF) + '0';

        UHF_TxReadCmd_struct txCommandBuffer =
        {
            .initalStr =
            {
                'E',
                'S',
                '+'
            },
            .opperation = 'W',
            .address =
            {
                addressHigh,
                addressLow
            }
        };

        txCommandBuffer.command[0] = UHF_EsttcWriteCmdNumb[command].msb;
        txCommandBuffer.command[1] = UHF_EsttcWriteCmdNumb[command].lsb;

        switch (command)
        {
            case UHF_CMD_WRITE_BEACON:
            {
                memcpy(uhfTxRxBuffer, &txCommandBuffer, 8);
                sprintf((char*)&uhfTxRxBuffer[8], "%02X", size);
                memcpy(&uhfTxRxBuffer[10], data, size);
            }
            break;

            default:
            {
                //retVal = UHF_CMD_STAT_ERROR;
            }
            break;
        }

        CRC_value_calc = crc32(0, (BYTE*)uhfTxRxBuffer, 10 + size);
        sprintf((char*)&uhfTxRxBuffer[10 + size], " %08X\r", (unsigned int)CRC_value_calc); /* Attach the calcuated CRC at the end of the string */

#if 1   //Disable UART transmittion
        HAL_StatusTypeDef I2C_stat;

        I2C_stat = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);

        if (I2C_stat == HAL_OK)
        {
            for (uint8_t i = 0; i < UHF_I2C_TRANSMITION_RETRY; i++)
            {
                I2C_stat = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, UHF_I2C_ADDRESS, (uint8_t*)&uhfTxRxBuffer, 10 + 10 + size);
                if (I2C_stat == HAL_OK)
                {
                    osDelay(UHF_I2C_TIME_BETWEEN_PACS);
                    I2C_stat = MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, UHF_I2C_ADDRESS, &uhfTxRxBuffer[0], 23);
                    osDelay(UHF_I2C_TIME_BETWEEN_PACS); //wait after receiving to prevent to frequent request

                    if (I2C_stat == HAL_OK)
                    {
                        if ((uhfTxRxBuffer[0] != 'O') || (uhfTxRxBuffer[1] != 'K'))
                        {
                            //retVal = UHF_CMD_STAT_ERROR;
                        }
                        else
                        {
                            retVal = UHF_CMD_STAT_COMPLETED_OK;
                            break;
                        }
                    }
                }
                else
                {
                    //retVal = UHF_CMD_STAT_ERROR;  //Already set
                }
            }

            MX_I2C_Release(MX_I2C_BUS_SYSTEM);
        }
        else
        {
            //retVal = UHF_CMD_STAT_ERROR;  //Already set
        }

        if ((I2C_stat != HAL_OK) || (retVal != UHF_CMD_STAT_COMPLETED_OK))
        {
            EXEH_vException(eEXEHSeverity_Warning, eEXEHModuleID_UHF_COMM, eEXEH_UHF_EXCEPTION_ID_COMM_I2C_ERROR, __LINE__);
            //retVal = UHF_CMD_STAT_ERROR
        }
#else
        fwrite(uhfTxRxBuffer, 0, 10+10+size, COMM );  /* print the symbols without any formating */

        //VMI_TODO_1UPLATFORM Send using USART and if not working then I2C -> receiving not done
#endif
    }

    return retVal;
}

#endif
