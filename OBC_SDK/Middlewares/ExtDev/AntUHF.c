/*!
********************************************************************************************
* @file  AntUHF.c
* @brief All functions serve to deploy all rods of the UHF antenna
********************************************************************************************
* @author            Vassil Milev
* @version           1.0.0
* @date              2019.06.19
*
* @copyright         (C) Copyright Endurosat
*
*                    Contents and presentations are protected world-wide.
*                    Any kind of using, copying etc. is prohibited without prior permission.
*                    All rights - incl. industrial property rights - are reserved.
*
* @history
* @revision{         1.0.0  , 2019.06.19, author Vassil Milev, Initial revision }
* @endhistory
********************************************************************************************
*/

/*
*********************************************************************************************
* INCLUDES
*********************************************************************************************
*/
#include "AntUHF.h"       /* Include own header file */
#include "GlobalConfig.h"
#if (ENABLE_UHF_ANT_SUPPORT == 1)

#include <string.h>
#include "MCU_Init.h"     /* Include I2C Interfaces to communicate with the UHF Antenna */
#include "EEPROM_Emul.h"  /* Include EEPROM emulation interfaces */
#include "PwrMng.h"
#include "MX_I2C.h"
#include "es_exeh.h"
#include "EPS_I.h"
#include "AppTasks.h"

/*
*********************************************************************************************
* INTERNAL DEFINES
*********************************************************************************************
*/
#define ANT_UHF_CALL_PERIOD        (SERVICE_TASK_PERIOD)
#define ANT_UHF_1000MS             (1000)
#define I2C_WRITE_DATA_BUS_WAIT_TIMEOUT_TICKS   (50)
#define ANT_UHF_DEFAULT_LOW_VOLTAGE             (1645)    //Default low voltage 3.85V. / 2.3394775
#define ANT_UHF_ALL_RODS_RELEASED               (0x0F)

/* At the 35 minute the UHF may try (if set to do that) to deploy the antenna rods if they are not deployed */
#define ANT_INIT_DELAY             (0xFFFE)//((60*0xFF) + ANT_ALG1_TIMEOUT + ANT_ALG2_TIMEOUT+1)  // Max time that needs to be count - 15486

#define DELAY_30_MINUTES           (60*30)  /* Wait 30 minutes after power up to deploy the antenna rods */
#define ANT_ALG1_TIMEOUT           (62)     /* Maximum time to do Algorithm 1 in seconds */
#define ANT_ALG2_TIMEOUT           (124)    /* Maximum time to do Algorithm 2 in seconds */
#define ANT_UHF_TRIALS             (15)     /* Number of attempts to retry */

/* Register 1 masks: */
#define ANT_UHF_ALGRTM_MODE_MASK   (0x03)    /* Bits S1 and S2 - Running mode: OFF, Algorithm 1, Algorithm 2 */
#define ANT_UHF_TEST_MODE_MASK     (0x04)    /* Test mode: 0 - Disabled, 1 - Enabled */
#define ANT_UHF_DOORS_MASK         (0xF0)    /* D1-D4 - Position status of all 4 doors */

#define EXEH_CURRENT_MODULE_ID     (eEXEHModuleID_UHF_ANT_COMM)
/*
*********************************************************************************************
* INTERNAL TYPES DEFINITION
*********************************************************************************************
*/
/* Enumeration with all states to do the deployment */
typedef enum
{
    ANT_STATE_CONNECTING,
    ANT_STATE_WAITING,
    ANT_STATE_FIRST_DEPLOYMENT,
    ANT_STATE_FIRST_DEPLOYMENT_EXECUTING,
    ANT_STATE_ALG1,
    ANT_STATE_ALG1_EXECUTING,
    ANT_STATE_ALG2,
    ANT_STATE_ALG2_EXECUTING,
    ANT_STATE_IDLE,
    ANT_STATE_NUMBER
} AntUHF_States_enum;

/*
*********************************************************************************************
* EXTERNAL VARIABLES DEFINITION
*********************************************************************************************
*/
AntUHFRegs_Struct antRegs;              /* Mirror to the registers of the UHF Antenna */

/*
*********************************************************************************************
* INTERNAL (STATIC) VARIABLES DEFINITION/DECLARATION 
*********************************************************************************************
*/
static int16_t AntPeriod = 0;                                   /* Counter for elapsed ms */
static uint16_t AntCnt = 0;                                     /* Time counter */
static uint16_t AntRetryCnt = 0;                                /* Time counter */
static uint8_t FirstDeploymentStatus;                           /* If that flag is set, the First deployment algorithm is enabled. A flag for that is set only if a command 0x47 has been received before last power up  */
static AntUHF_States_enum AntUHF_State = ANT_STATE_CONNECTING;  /* State of the deployment algorithm */
static uint16_t AntUHF_PowerUpDelay    = DELAY_30_MINUTES;      /* State of the deployment algorithm */
static uint16_t AntUHF_LowBatThreshold = ANT_UHF_DEFAULT_LOW_VOLTAGE;     /* Default low voltage */

/*
*********************************************************************************************
* INTERNAL (STATIC) ROUTINES DECLARATION
*********************************************************************************************
*/
static void AntUHF_StartOutputsAnalogDeployment(void);
static HAL_StatusTypeDef AntUHF_GetStatus(void);
static uint8_t AntUHF_SetCommand(uint8_t cmd);
static uint8_t AntUHF_IsBattLow(void);



/*
*********************************************************************************************
* EXTERNAL (NONE STATIC) ROUTINES DEFINITION
*********************************************************************************************
*/
/*!
*********************************************************************************************
* @brief Deploys the antenna after 0.5 hours after the power up
*********************************************************************************************
* @param[input]      argument - not used
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void AntUHF_Handler(void)
{
    uint8_t TempData;
    uint8_t OneSecondFlag = 0;

    if ((EEPROM_emul_DataTemp.AntUHFSettings.Byte.AntUHFEnable) && (AntUHF_State != ANT_STATE_IDLE))
    {
        uint16_t callPeriod;

        //Enter here on normal period at which the task is called
        callPeriod = ANT_UHF_CALL_PERIOD;

        AntPeriod += callPeriod;
        if (AntPeriod >= ANT_UHF_1000MS)
        {
            OneSecondFlag = 1; /* at lease one second has passed */

            if (AntCnt <= AntUHF_PowerUpDelay)
            {    //Prevent overflow in case of low bat
                AntCnt += AntPeriod / ANT_UHF_1000MS;
            }
            AntPeriod %= ANT_UHF_1000MS;

            if (OneSecondFlag == 1)
            {
                switch (AntUHF_State)
                {
                    case ANT_STATE_CONNECTING:
                    {
                        //AntCnt++;                /* +1 second */
                        /* Connect to antenna */
                        if (HAL_OK == AntUHF_GetStatus())
                        {
                            /* connection to the antenna is established so continue to next step */
                            AntUHF_State = ANT_STATE_WAITING;
                        }
                        else
                        {
                            if (AntCnt > AntUHF_PowerUpDelay)
                            {
                                if (FirstDeploymentStatus)
                                {
                                    if (AntUHF_IsBattLow() == pdFALSE)
                                    {
                                        AntUHF_StartOutputsAnalogDeployment();

                                        /* Clear the flag for that request */
                                        FirstDeploymentStatus = 0;

                                        /* First deployment algorithm is done, clear the flags */
                                        EEPROM_emul_DataTemp.AntUHFSettings.Byte.AntUHFFrstDepl = 0;
                                        /* Sync EEPROM emulated memory with the buffer "EEPROM_emul_DataTemp" */
                                        EEPROM_Emul_SyncInfo();
                                    }
                                }
                            }
                        }
                    } break;

                    case ANT_STATE_WAITING:
                    {
                        //AntCnt++;                /* +1 second */
                        /* Wait until 30 minutes after power up has elapsed */
                        if (AntCnt > AntUHF_PowerUpDelay)
                        {
                            if (AntUHF_IsBattLow() == pdFALSE)
                            {
                                AntCnt = 0;
                                /*          Check for command 0x47           */
                                if (FirstDeploymentStatus == 1)
                                {
                                    FirstDeploymentStatus = (antRegs.ant_io[0] >> 4);
                                    AntUHF_State = ANT_STATE_FIRST_DEPLOYMENT;
                                }
                                else
                                {
                                    AntUHF_State = ANT_STATE_ALG1;
                                }
                            }
                        }
                    } break;

                    case ANT_STATE_FIRST_DEPLOYMENT:
                    {
                        AntRetryCnt++;

                        /* try to send the command for deployment several times maximum until done successful */
                        if (AntRetryCnt <= ANT_UHF_TRIALS)
                        {
                            /* Send command to open all deployed rods */
                            TempData = AntUHF_SetCommand(0x20 | FirstDeploymentStatus);

                            if (TempData != (0x20 | FirstDeploymentStatus))
                            {
                                /* The command has failed - try again */
                                //AntCnt++;
                            }
                            else
                            {
                                /* Successive packet to a UHF antenna */

                                /* wait until the command is done */
                                AntUHF_State = ANT_STATE_FIRST_DEPLOYMENT_EXECUTING; /* exit the cycle */
                                AntCnt = 0;
                            }
                        }
                        else
                        {
                            /* The connection with the Antenna is not working */
                            AntUHF_State = ANT_STATE_CONNECTING;
                            AntCnt = 0;
                            AntRetryCnt = 0;
                        }
                    } break;

                    case ANT_STATE_FIRST_DEPLOYMENT_EXECUTING:
                    {
                        /* Read antenna status */
                        if (HAL_OK == AntUHF_GetStatus())
                        {
                            if ((AntCnt <= (ANT_ALG1_TIMEOUT + callPeriod / 1000)) && ((antRegs.ant_io[UHFANTRESULT_REG_HEETERS] != 0) || (antRegs.Byte.heatersTimeout != 0)))
                            {
                                /* Command not finished */

                                /* Check again next time */
                                //AntCnt++;
                            }
                            else
                            {
                                /* Successive packet to a UHF antenna */
                                AntUHF_State = ANT_STATE_ALG1; /* exit the cycle */
                                AntCnt = 0;

                                /* Clear the flag for that request */
                                FirstDeploymentStatus = 0;

                                /* First deployment algorithm is done, clear the flags */
                                EEPROM_emul_DataTemp.AntUHFSettings.Byte.AntUHFFrstDepl = 0;
                                /* Sync EEPROM emulated memory with the buffer "EEPROM_emul_DataTemp" */
                                EEPROM_Emul_SyncInfo();
                            }
                        }
                        else
                        {
                            AntRetryCnt++;
                            if (AntRetryCnt > ANT_UHF_TRIALS)
                            {
                                /* The connection with the Antenna is not working */
                                AntUHF_State = ANT_STATE_CONNECTING;
                                AntCnt = 0;
                                AntRetryCnt = 0;
                            }
                        }
                    } break;

                    case ANT_STATE_ALG1:
                    {
                        /* Read antenna status */
                        if (HAL_OK == AntUHF_GetStatus())
                        {
                            /* Start deployment Algorithm 1 */
                            if ((antRegs.ant_io[UHFANTRESULT_REG_STATUS] >> 4) != ANT_UHF_ALL_RODS_RELEASED) /* if any rods are not deployed */
                            {
                                AntRetryCnt++;

                                /* try to send the command for deployment several times maximum until done successful */
                                if (AntRetryCnt <= ANT_UHF_TRIALS)
                                {
                                    osDelay(5); // wait some time between the read and write commands

                                    /* Send command to open all rods that are not deployed */
                                    TempData = AntUHF_SetCommand(0x10 | ((antRegs.Byte.doorsStatus) ^ ANT_UHF_ALL_RODS_RELEASED)); /* 0x10 = Algorithm 1 */
                                    if (TempData != (0x10 | ((antRegs.Byte.doorsStatus) ^ ANT_UHF_ALL_RODS_RELEASED)))
                                    {
                                        /* The command has failed - try again */
                                        //AntCnt++;
                                    }
                                    else
                                    {
                                        /* Successive packet to a UHF antenna */

                                        /* wait until the command is done */
                                        AntUHF_State = ANT_STATE_ALG1_EXECUTING; /* exit the cycle */
                                    }
                                }
                                else
                                {
                                    /* The connection with the Antenna is not working */
                                    AntUHF_State = ANT_STATE_CONNECTING;
                                    AntCnt = 0;
                                    AntRetryCnt = 0;
                                }
                            }
                            else
                            {
                                AntUHF_State = ANT_STATE_IDLE;
                            }
                        }
                        else if (AntRetryCnt > ANT_UHF_TRIALS)
                        {
                            /* The connection with the Antenna is not working */
                            AntUHF_State = ANT_STATE_CONNECTING;
                            AntCnt = 0;
                        }
                    } break;

                    case ANT_STATE_ALG1_EXECUTING:
                    {
                        /* Read antenna status */
                        if (HAL_OK == AntUHF_GetStatus())
                        {
                            if ((AntCnt <= (ANT_ALG1_TIMEOUT + callPeriod / 1000)) && ((antRegs.ant_io[UHFANTRESULT_REG_HEETERS] != 0) || (antRegs.Byte.heatersTimeout != 0)))
                            {
                                /* Command not finished */

                                /* Check again next time */
                                //AntCnt++;
                            }
                            else
                            {
                                /* Successive packet to a UHF antenna */
                                AntUHF_State = ANT_STATE_ALG2; /* exit the cycle */
                                AntCnt = 0;
                            }
                        }
                        else
                        {
                            AntRetryCnt++;

                            if (AntRetryCnt > ANT_UHF_TRIALS)
                            {
                                /* The connection with the Antenna is not working */
                                AntUHF_State = ANT_STATE_CONNECTING;
                                AntCnt = 0;
                                AntRetryCnt = 0;
                            }
                        }
                    } break;

                    case ANT_STATE_ALG2:
                    {
                        AntRetryCnt++;

                        /* Read antenna status                  */
                        if (HAL_OK == AntUHF_GetStatus())
                        {
                            /* Start deployment Algorithm 2         */
                            if ((antRegs.Byte.doorsStatus) != ANT_UHF_ALL_RODS_RELEASED) /* if any rods are not deployed */
                            {
                                /* try to send the command for deployment several times maximum until done successful */
                                if (AntRetryCnt <= ANT_UHF_TRIALS)
                                {
                                    osDelay(5); // wait some time between the read and write commands

                                    /* Send command to open all rods that are not deployed */
                                    TempData = AntUHF_SetCommand(0x20 | ((antRegs.Byte.doorsStatus) ^ ANT_UHF_ALL_RODS_RELEASED)); /* 0x20 = Algorithm 2 */
                                    if (TempData != (0x20 | ((antRegs.Byte.doorsStatus) ^ ANT_UHF_ALL_RODS_RELEASED)))
                                    {
                                        /* The command has failed - try again */
                                        //AntCnt++;
                                    }
                                    else
                                    {
                                        /* Successive packet to a UHF antenna */

                                        /* wait until the command is done */
                                        AntUHF_State = ANT_STATE_ALG2_EXECUTING; /* exit the cycle */
                                    }
                                }
                                else
                                {
                                    /* The connection with the Antenna is not working */
                                    AntUHF_State = ANT_STATE_CONNECTING;
                                    AntCnt = 0;
                                }

                            }
                            else
                            {
                                AntUHF_State = ANT_STATE_IDLE;
                                AntCnt = 0;
                            }
                        }
                        else if (AntRetryCnt > ANT_UHF_TRIALS)
                        {
                            /* The connection with the Antenna is not working */
                            AntUHF_State = ANT_STATE_CONNECTING;
                            AntCnt = 0;
                            AntRetryCnt = 0;
                        }
                    } break;

                    case ANT_STATE_ALG2_EXECUTING:
                    {
                        /* Read antenna status                  */
                        if (HAL_OK == AntUHF_GetStatus())
                        {
                            if ((AntCnt <= (ANT_ALG2_TIMEOUT + callPeriod / 1000)) && ((antRegs.ant_io[UHFANTRESULT_REG_HEETERS] != 0) || (antRegs.Byte.heatersTimeout != 0)))
                            {
                                /* Command not finished */

                                /* Check again next time */
                                //AntCnt++;
                            }
                            else
                            {
                                /* Successive packet to a UHF antenna */
                                AntUHF_State = ANT_STATE_IDLE; /* exit the cycle */
                            }
                        }
                        else
                        {
                            AntRetryCnt++;

                            if (AntRetryCnt > ANT_UHF_TRIALS)
                            {
                                /* The connection with the Antenna is not working */
                                AntUHF_State = ANT_STATE_CONNECTING;
                                AntCnt = 0;
                                AntRetryCnt = 0;
                            }
                        }
                    } break;

                    case ANT_STATE_IDLE:
                    case ANT_STATE_NUMBER:
                    default:
                    {
                    }
                }
            }
        }
    }
}

/*!
*********************************************************************************************
* @brief Initialisation routine of the UHF antenna
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void AntUHF_Init(void)
{
    AntPeriod = 0;
    AntCnt = 0;
    AntRetryCnt = 0;
    FirstDeploymentStatus = EEPROM_emul_DataTemp.AntUHFSettings.Byte.AntUHFFrstDepl;
    AntUHF_State = ANT_STATE_CONNECTING;
    if (EEPROM_emul_DataTemp.AntUHFSettings.Byte.PowerUpDelay >= ANTUHF_POWER_UP_MIN_TIME) //Invalid settings
    {
        AntUHF_PowerUpDelay = EEPROM_emul_DataTemp.AntUHFSettings.Byte.PowerUpDelay * 60;
        AntUHF_LowBatThreshold = EEPROM_emul_DataTemp.AntUHFSettings.Byte.LowBatThreshold;
    }
}

/*!
*********************************************************************************************
* @brief Set the delay period after power up
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void AntUHF_ChangePowerUpDelay(uint8_t Delay)
{
    if (Delay >= ANTUHF_POWER_UP_MIN_TIME)
    {
        AntUHF_PowerUpDelay = Delay * 60;
    }
}

/*!
*********************************************************************************************
* @brief Updates the UHF antenna register
*********************************************************************************************
* @param[input]      u8RegValue: register value to be written
* @return            HAL_StatusTypeDef - Status of the requested operation
* @note              none
*********************************************************************************************
*/
HAL_StatusTypeDef AntUHF_Write_Data(uint8_t u8RegValue)
{
    HAL_StatusTypeDef I2C_retStat = HAL_ERROR;

    I2C_retStat = MX_I2C_Take(MX_I2C_BUS_SYSTEM, I2C_WRITE_DATA_BUS_WAIT_TIMEOUT_TICKS);

    if (I2C_retStat == HAL_OK)
    {
        I2C_retStat = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, ANT_I2C_ADDRESS, &u8RegValue, sizeof(uint8_t));
        MX_I2C_Release(MX_I2C_BUS_SYSTEM);
    }

    if (I2C_retStat != HAL_OK)
        EXEH_HANDLE(eEXEHSeverity_Error, eEXEH_UHF_ANT_EXCEPTION_ID_COMM_ERROR);

    return I2C_retStat;
}

/*!
*********************************************************************************************
* @brief Read all status registers of the Antenna
*********************************************************************************************
* @param[input]      ReadType  + 0 - Get last read value from the buffer.
*                              + 1 - Read the registers from the UHF antenna and refresh the values from the buffer
* @param[output]     *Status - the value of the requested register
* @return            HAL_StatusTypeDef - Status of the requested operation
* @note              none
*********************************************************************************************
*/
HAL_StatusTypeDef AntUHF_Read_Data(eAntUHFReadType_t ReadType , AntUHFRegs_Struct * registers)
{
    HAL_StatusTypeDef readStatus = HAL_ERROR;

    if (ReadType == ANTUHFREADTYPE_ACTUAL_VALUE)
    {
        readStatus = AntUHF_GetStatus();
        if (readStatus == HAL_OK)
            memcpy(registers,&antRegs,sizeof(AntUHFRegs_Struct));
    }
    else
    {
        memcpy(registers,&antRegs,sizeof(AntUHFRegs_Struct));
        readStatus = HAL_OK;
    }

    return readStatus;
}

AntUHFSettings_Struct AntUHF_ReadSettings(void)
{
    AntUHFSettings_Struct settings;

    (void*)memcpy(&settings, &EEPROM_emul_DataTemp.AntUHFSettings, sizeof(AntUHFSettings_Struct));

    return settings;
}

UHFAntResult AntUHF_UpdateSettings(const AntUHFSettings_Struct *pNewSettings)
{
    UHFAntResult res = UHFANTRESULT_INVALID_ARGS;

    if (pNewSettings != NULL)
    {
        if (pNewSettings->Byte.PowerUpDelay >= ANTUHF_POWER_UP_MIN_TIME)
        {
            EEPROM_emul_DataTemp.AntUHFSettings.u32Settings = pNewSettings->u32Settings;

            // Change the delay right away
            AntUHF_ChangePowerUpDelay(pNewSettings->Byte.PowerUpDelay);
            AntUHF_LowBatThreshold = pNewSettings->Byte.LowBatThreshold;

            // Sync EEPROM emulated memory with the buffer "EEPROM_emul_DataTemp"
            EEPROM_Emul_SyncInfo();

            res = UHFANTRESULT_SUCCESS;
        }
    }

    return res;
}

/*
*********************************************************************************************
* INTERNAL (STATIC) ROUTINES DEFINITION
*********************************************************************************************
*/
static void AntUHF_StartOutputsAnalogDeployment(void)
{
    MCU_Init_OutStates_struct states;
    if (EEPROM_emul_DataTemp.AntUHFSettings.Byte.Out2Deployment == 1)
    {
        states.OutToChange = EPS_I_OUTPUTS_MASKS_OUT2;
        states.OutState = EPS_I_OUTPUTS_MASKS_OUT2;
        MX_GPIO_Outputs_Set(&states);

        sysDelay(20000); //wait 20 sec sharp - this time should not be dependent of other waiting periods

        states.OutState = 0;
        MX_GPIO_Outputs_Set(&states);
    }
}

/*!
*********************************************************************************************
* @brief Send a command to the UHF Antenna
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            uint8_t - repeats the command number on success
*                            - 0xFF on fail
* @note              none
*********************************************************************************************
*/
static uint8_t AntUHF_SetCommand(uint8_t cmd)
{
    HAL_StatusTypeDef I2C_retStat = 0;
    uint8_t retVal;

    I2C_retStat = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);
    if (I2C_retStat == HAL_OK)
    {
        I2C_retStat = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, ANT_I2C_ADDRESS, &cmd, sizeof(uint8_t));

        MX_I2C_Release(MX_I2C_BUS_SYSTEM);
    }

    if (I2C_retStat != HAL_OK)
    {
        EXEH_HANDLE(eEXEHSeverity_Error, eEXEH_UHF_ANT_EXCEPTION_ID_COMM_ERROR);
    }

    if (I2C_retStat == HAL_OK)
        retVal = cmd;
    else
        retVal = 0xFF;

    return retVal;
}

/*!
*********************************************************************************************
* @brief Read all status registers of the Antenna
*********************************************************************************************
* @param[input]      none
* @param[output]     ant_io[] - on successful read the status registers are written at that array
* @return            HAL_StatusTypeDef - Status of the requested operation
* @note              none
*********************************************************************************************
*/
static HAL_StatusTypeDef AntUHF_GetStatus(void)
{
    AntUHFRegs_Struct tmp;
    HAL_StatusTypeDef hal_res = 0;

    hal_res = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);
    if (hal_res == HAL_OK)
    {
        hal_res = MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, ANT_I2C_ADDRESS, &tmp.ant_io[0], UHFANTRESULT_REG_COUNT);

        MX_I2C_Release(MX_I2C_BUS_SYSTEM);
    }

    if (hal_res != HAL_OK)
    {
        EXEH_HANDLE(eEXEHSeverity_Error, eEXEH_UHF_ANT_EXCEPTION_ID_COMM_ERROR);
    }

    if (hal_res == HAL_OK)
    {
        memcpy(&antRegs, &tmp, sizeof(AntUHFRegs_Struct));
    }

    return hal_res;
}

static uint8_t AntUHF_IsBattLow(void)
{
    uint8_t retStat = pdFALSE;

    if (EEPROM_emul_DataTemp.AntUHFSettings.Byte.LowBatCheckEn == pdTRUE)
    {
#if (ENABLE_EPS_I_SUPPORT == 1)
        static uint8_t lowVoltageCounter = 0;

        uint16_t OpModes_VbatIdeal_Data = 0;

        HAL_StatusTypeDef readStatus = EPS_Read_Register(EPS_I_R_REG_VBAT_IDEAL, pdTRUE, &OpModes_VbatIdeal_Data);

        if (readStatus == HAL_OK)
        {

            if ((OpModes_VbatIdeal_Data > EPS_I_MIN_VALID_VOLTAGE) && // grater then 1.5 V - ignore impossible values
                (OpModes_VbatIdeal_Data < AntUHF_LowBatThreshold)) // lower then the programmed threshold
            {
                lowVoltageCounter++;
                if (lowVoltageCounter >= 10)
                {
                    lowVoltageCounter = 0;
                    retStat = pdTRUE;
                }
                else
                {
                    retStat = pdFALSE;
                }
            }
            else
            {
                lowVoltageCounter = 0;
                return pdFALSE;
            }
        }
#endif
    }
    else
    {
        //retStat = pdFALSE; //already set to false
    }

    return retStat;
}


#endif
/* **************************************************************************************** */
