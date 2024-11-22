/*!
********************************************************************************************
* @file UHF.h
* @brief Header of UHF.
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
#ifndef UHF_H
#define UHF_H

/*
********************************************************************************************
* INCLUDES
********************************************************************************************
*/
#include <main.h>

/*
********************************************************************************************
* EXTERNAL DEFINES
********************************************************************************************
*/

#define UHF_ADDRESSES_CONFIG \
{                            \
    0x22,                    \
}



/*
********************************************************************************************
* EXTERNAL TYPES DECLARATIONS
********************************************************************************************
*/
//Keep "UHF_EsttcReadCmdNumb" aligned to that enum
typedef enum
{
    UHF_CMD_READ_RSSI,
    UHF_CMD_READ_SCW,
    UHF_CMD_READ_UPTIME,
    UHF_CMD_READ_TX_PACKETS,
    UHF_CMD_READ_RX_PACKETS,
    UHF_CMD_READ_CRC_ERR,
    UHF_CMD_READ_TEMP_SENS,
    UHF_CMD_READ_STOPPER    //Keep this at the end
} UHFcmdsRead_Enum;

//Keep "UHF_EsttcWriteCmdNumb" aligned to that enum
typedef enum
{
    UHF_CMD_WRITE_SCW,
    UHF_CMD_WRITE_BEACON,
    UHF_CMD_WRITE_PING,
    UHF_CMD_WRITE_STOPPER    //Keep this at the end
} UHFcmdsWrite_Enum;

typedef enum
{
    UHF_CMD_STAT_COMPLETED_OK,
    UHF_CMD_STAT_TIMEOUT,
    UHF_CMD_STAT_ERROR,
    UHF_CMD_STAT_STOPPER    //Keep this at the end
} UHFcmdStatus_Enum;

typedef enum
{
    UHF_II_NUMBER_A,
    UHF_II_STOPPER    //Keep this at the end
} UHFactive_Enum;


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
void UHF_Init(void);
void UHF_SelectActive(UHFactive_Enum uhfNumber);
UHFcmdStatus_Enum UHF_Set_Beacon(uint8_t *data);
UHFcmdStatus_Enum UHF_Get_Uptime(uint8_t ReadType, uint32_t *Uptime);
UHFcmdStatus_Enum UHF_Get_TxPacks(uint8_t ReadType, uint32_t *TxPackets);
UHFcmdStatus_Enum UHF_Get_RxPacks(uint8_t ReadType, uint32_t *RxPackets);
UHFcmdStatus_Enum UHF_Get_CrcErrs(uint8_t ReadType, uint32_t *CrcErrs);
UHFcmdStatus_Enum UHF_Get_TemerSens(uint8_t ReadType, uint8_t *TemperSensors);
UHFcmdStatus_Enum UHF_Get_TemerSensRaw(uint8_t ReadType, int16_t *TemperSensorsRaw);


#endif    /* UHF_H */
/* ******************************************************************************************* */
