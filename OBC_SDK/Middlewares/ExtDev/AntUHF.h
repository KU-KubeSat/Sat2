/*!
 ********************************************************************************************
 * @file AntUHF.h
 * @brief Header of AntUHF.c
 ********************************************************************************************
 * @author            Vassil Milev
 * @version           1.0.0
 * @date              2018.01.11
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
#ifndef ANTUHF_H
#define ANTUHF_H

/*
 *********************************************************************************************
 * INCLUDES
 *********************************************************************************************
 */
#include <stdint.h>
#include "stm32f7xx_hal.h"

/*
 *********************************************************************************************
 * EXTERNAL DEFINES
 *********************************************************************************************
 */
#define ANTUHF_POWER_UP_MIN_TIME      (0x0A)    /* Minimum possible time until the deployment is allowed to start */

/*
 *********************************************************************************************
 * EXTERNAL TYPES DECLARATIONS
 *********************************************************************************************
 */
/* Keeps data about the Errors, faults, CRC of the Flash memory */
typedef union
{
    struct
    {
        uint8_t AntUHFEnable   :1; /* Enable the UHF module */
        uint8_t AntUHFFrstDepl :1; /* Enable the First deployment algorithm */
        uint8_t Out2Deployment :1; /* Use Out2 */
        uint8_t Reserve1       :4; /* Reserved */
        uint8_t LowBatCheckEn  :1; /* Check for low battery (if low postpone the deployment) */
        uint8_t PowerUpDelay;      /* Time after power up, until the deployment algorithm is started */
        uint16_t LowBatThreshold;  /* Minimum battery voltage to release the antenna */
    } Byte;
    uint32_t u32Settings;
} AntUHFSettings_Struct;

typedef enum
{
    UHFANTRESULT_SUCCESS,
    UHFANTRESULT_ERROR,
    UHFANTRESULT_INVALID_ARGS
} UHFAntResult;

typedef enum
{
    UHFANTRESULT_REG_STATUS     = 0,
    UHFANTRESULT_REG_HEETERS    = 1,
    UHFANTRESULT_REG_TIMEOUT    = 2,
    UHFANTRESULT_REG_SWITCHES   = 3,
    UHFANTRESULT_REG_COUNT      = 4 /* Number of registers of the UHF antenna */
} UHFAntRegs_enum;

typedef union
{
    struct
    {
        //Byte 0:
        uint8_t Algorithm_1   :1; /* Algorithm 1 activity status */
        uint8_t Algorithm_2   :1; /* Algorithm 1 activity status */
        uint8_t testMode      :1; /* Status of test mode */
        uint8_t reserved      :1; /* reserved bit */
        uint8_t doorsStatus   :4; /* Status of all doors */

        //Byte 1:
        uint8_t backupHeaters :4; /* Status of all backup heaters */
        uint8_t mainHeaters   :4; /* Status of all main heaters */

        //Byte 2:
        uint8_t heatersTimeout;   /* Timeout to stop the heaters */

        //Byte 3:
        uint8_t switchesStatuses; /* Open/close status of the switches on the doors */
    } Byte;
    uint8_t ant_io[UHFANTRESULT_REG_COUNT];
} AntUHFRegs_Struct;

typedef enum
{
    ANTUHFREADTYPE_CACHED_VALUE = 0U,
    ANTUHFREADTYPE_ACTUAL_VALUE = 1U,
    ANTUHFREADTYPE_MAX
} eAntUHFReadType_t;
/*
 *********************************************************************************************
 * EXTERNAL VARIABLES DECLARATIONS
 *********************************************************************************************
 */
extern AntUHFRegs_Struct antRegs; /* Backup of the registers of the UHF Antenna */

/*
 *********************************************************************************************
 * EXTERNAL ROUTINES DECLARATIONS
 *********************************************************************************************
 */
void AntUHF_Handler(void);
void AntUHF_Init(void);
void AntUHF_ChangePowerUpDelay(uint8_t Delay);
HAL_StatusTypeDef AntUHF_Read_Data(eAntUHFReadType_t ReadType , AntUHFRegs_Struct * registers);
HAL_StatusTypeDef AntUHF_Write_Data(uint8_t u8RegValue);
AntUHFSettings_Struct AntUHF_ReadSettings(void);
UHFAntResult AntUHF_UpdateSettings(const AntUHFSettings_Struct *pNewSettings);

#endif    /* ANTUHF_H */
/* **************************************************************************************** */

