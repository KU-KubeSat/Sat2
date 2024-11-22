/*!
********************************************************************************************
* @file EPS.h
* @brief Header of EPS.c
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
#ifndef EPS_I_H
#define EPS_I_H

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
#define EPS_I_OUTPUTS_MASKS_OUT1   (1<< MCU_INIT_OUT1  )
#define EPS_I_OUTPUTS_MASKS_OUT2   (1<< MCU_INIT_OUT2  )
#define EPS_I_OUTPUTS_MASKS_OUT3   (1<< MCU_INIT_OUT3  )
#define EPS_I_OUTPUTS_MASKS_OUT5   (1<< MCU_INIT_OUT5  )
#define EPS_I_OUTPUTS_MASKS_OUT4_6 (1<< MCU_INIT_OUT4_6)
#define EPS_I_OUTPUTS_MASKS_ALL    (EPS_I_OUTPUTS_MASKS_OUT1 | EPS_I_OUTPUTS_MASKS_OUT2 | EPS_I_OUTPUTS_MASKS_OUT3 | EPS_I_OUTPUTS_MASKS_OUT5)  // mask OBC outputs for EPSI

#define EPS_I_MIN_VALID_VOLTAGE    (641)       //Minimum valid voltage

#define EPS_I_ROW_TO_MV_CONST      (2.3394775)

/*
********************************************************************************************
* EXTERNAL TYPES DECLARATIONS
********************************************************************************************
*/
typedef enum
{
    EPS_I_R_REG_dummy,
    EPS_I_R_REG_BATTERY_VOLTAGE,
    EPS_I_R_REG_BATTERY_CURRENT,
    EPS_I_R_REG_BCR_VOLTAGE,
    EPS_I_R_REG_BCR_CURRENT,
    EPS_I_R_REG_X_VOLTAGE,
    EPS_I_R_REG_X_M_CURRENT,
    EPS_I_R_REG_X_P_CURRENT,
    EPS_I_R_REG_Y_VOLTAGE,
    EPS_I_R_REG_Y_M_CURRENT,
    EPS_I_R_REG_Y_P_CURRENT,
    EPS_I_R_REG_Z_VOLTAGE,
    EPS_I_R_REG_Z_M_CURRENT,
    EPS_I_R_REG_Z_P_CURRENT,
    EPS_I_R_REG_3_3V_CURRENT,
    EPS_I_R_REG_5V_CURRENT,
    EPS_I_R_REG_LUP_3_3V,
    EPS_I_R_REG_LUP_5V,
    EPS_I_R_REG_MCU_TEMP,
    EPS_I_R_REG_BATT_TEMP_SENS_1,
    EPS_I_R_REG_BATT_TEMP_SENS_2,
    EPS_I_R_REG_BATT_TEMP_SENS_3,
    EPS_I_R_REG_BATT_TEMP_SENS_4,
    EPS_I_R_REG_INPUT_CONDITION,
    EPS_I_R_REG_OUTPUTS_CONDITIONS,
    EPS_I_R_REG_OUTPUTS_CONDITIONS_2,
    EPS_I_R_REG_POWER_ON_CYCLES,
    EPS_I_R_REG_V_UNDER_VOLTAGE,
    EPS_I_R_REG_V_SHORT_CIRCUIT,
    EPS_I_R_REG_V_OVER_TEMPERATURE,
    EPS_I_R_REG_MAX_TEMP_1,
    EPS_I_R_REG_MAX_TEMP_2,
    EPS_I_R_REG_MAX_TEMP_3,
    EPS_I_R_REG_MAX_TEMP_4,
    EPS_I_R_REG_MIN_TEMP_1,
    EPS_I_R_REG_MIN_TEMP_2,
    EPS_I_R_REG_MIN_TEMP_3,
    EPS_I_R_REG_MIN_TEMP_4,
    EPS_I_R_REG_TEMP_SENSOR_5,
    EPS_I_R_REG_TEMP_SENSOR_6,
    EPS_I_R_REG_TEMP_SENSOR_7,
    EPS_I_R_REG_RESERVE_TEMP_SENS_8,
    EPS_I_R_REG_SW_VERSION,
    EPS_I_R_REG_DEFAULTS_1,
    EPS_I_R_REG_DEFAULTS_2,
    EPS_I_R_REG_RBAT_INIT,
    EPS_I_R_REG_RBAT,
    EPS_I_R_REG_VBAT_IDEAL,
    EPS_I_R_REG_CHARGE_CYCLES,
    EPS_I_R_REG_BUS_3V3,
    EPS_I_R_REG_BUS_5V,
    EPS_I_R_REG_MSB_UPTIMER,
    EPS_I_R_REG_lSB_UPTIMER,
    EPS_I_R_REG_NUMBER
} EPS_ReadRegs_enum;

typedef enum
{
    EPS_I_W_REG_SWSelfLock,
    EPS_I_W_REG_VBATTEN,
    EPS_I_W_REG_BCROutEN,
    EPS_I_W_REG_SHD_3_3V,
    EPS_I_W_REG_SHD_5V,
    EPS_I_W_REG_LUP3_3V,
    EPS_I_W_REG_LUP5V,
    EPS_I_W_REG_SHDChrg,
    EPS_I_W_REG_Chrg_I1,
    EPS_I_W_REG_Chrg_I2,
    EPS_I_W_REG_OUT1,
    EPS_I_W_REG_OUT2,
    EPS_I_W_REG_OUT3,
    EPS_I_W_REG_OUT4,
    EPS_I_W_REG_OUT5,
    EPS_I_W_REG_OUT6,
    EPS_I_W_REG_HEETER1,
    EPS_I_W_REG_HEETER2,
    EPS_I_W_REG_HEETER3,
    EPS_I_W_REG_10KPULLUP,
    EPS_I_W_REG_4K7PULLUP,
    EPS_I_W_REG_R_TERMIN,
    EPS_I_W_REG_NUMBER
} EPS_WriteRegs_enum;

typedef enum
{
    OFF = 0, /* Off            (0) */
    ON = 1, /* On             (1) */
    I_OFF = 2, /* Hardcored OFF  (2) */
    I_ON = 3, /* Hardcored ON   (3) */
    Mass_eLAST /* Last num       (*) */
} EPS_eOUTPUT_STAT;

typedef enum
{
    EPS_I_CMD_STAT_COMPLETED_OK,
    EPS_I_CMD_STAT_TIMEOUT,
    EPS_I_CMD_STAT_ERROR,
    EPS_I_CMD_STAT_STOPPER    //Keep this at the end
} EPS_I_cmdStatus_Enum;

//Keep "UHF_EsttcReadCmdNumb" aligned to that enum
typedef enum
{
    EPS_I_CMD_READ_SW_INFO,
    EPS_I_CMD_READ_BATT_INFO,
    EPS_I_CMD_READ_BASES_INFO,
    EPS_I_CMD_READ_SOLAR_PANS_INFO,
    EPS_I_CMD_READ_STATISTICS,
    EPS_I_CMD_READ_TEMPERATURES,
    EPS_I_CMD_READ_CONFIGURATIONS,

    EPS_I_CMD_READ_STOPPER    //Keep this at the end
} EPS_I_cmdsRead_Enum;

//Keep "UHF_EsttcWriteCmdNumb" aligned to that enum
typedef enum
{
    EPS_I_CMD_WRITE_OUTPUT_CONTROL,
    EPS_I_CMD_WRITE_SET_INIT_RBAT,
    EPS_I_CMD_WRITE_STOPPER    //Keep this at the end
} EPS_I_cmdsWrite_Enum;


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
void EPS_I_Init(void);
void EPS_I_HwReset(void);

// low level I2C functions
HAL_StatusTypeDef EPS_Read_Register(EPS_ReadRegs_enum paramNumber, uint8_t ReadType, uint16_t * ReadData);
HAL_StatusTypeDef EPS_ReadConvertedValue(EPS_ReadRegs_enum paramNumber, uint8_t ReadType, float * convertedValue);
HAL_StatusTypeDef EPS_Write_Register(EPS_WriteRegs_enum paramNumber, uint8_t WriteData);
HAL_StatusTypeDef EPS_VerifyWrite_Register(EPS_WriteRegs_enum paramNumber, EPS_eOUTPUT_STAT WriteData);
void EPS_I_SetEpsOutputs(MCU_Init_OutStates_struct *states);
float EPS_I_GetRawDataConvValue(EPS_ReadRegs_enum eParamNumber);


#endif    /* EPS_I_H */
/* ******************************************************************************************* */
