/*!
********************************************************************************************
* @file EPS.c
* @brief Driver for Endurosat Electric Power System I (EPS I).
********************************************************************************************
* @author            Vassil Milev
* @version           1.0.0
* @date              2020.06.23
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
#include <EPS_I.h>          /* Include own header file */

#if (ENABLE_EPS_I_SUPPORT == 1)

#include "cmsis_os.h"       /* Include OS interfaces */
#include "string.h"         /* Include OS interfaces */
#include "MX_I2C.h"         /* Include OS interfaces */
#include "es_exeh.h"        /* Include DTC error handling */
#include "semphr.h"


/*
********************************************************************************************
* INTERNAL DEFINES
********************************************************************************************
*/
#define PACKED_STRUCT __attribute__((packed))

#define EPS_I_I2C_TIME_BETWEEN_PACS         (2)
#define EPS_I_I2C_MAX_RETRY                 (5)  //max retry to reset the I2C interface
#define EPS_I_BAT_INFO_NUMBER_OLD_CMDS      (5)  //corresponding to MAC CMD: EPS1_GETBATTERYINFO_FUNC_ID
#define EPS_I_BUSSES_INFO_NUMBER_OLD_CMDS   (6)  //corresponding to MAC CMD: EPS1_GETBUSESINFO_FUNC_ID
#define EPS_I_PANS_INFO_NUMBER_OLD_CMDS     (9)  //corresponding to MAC CMD: EPS1_GETSOLARPANELSINFO_FUNC_ID
#define EPS_I_NUMBER_STATISTICS_CMDS        (17) //corresponding to MAC CMD: EPS1_GETSTATISTICINFO_FUNC_ID
#define EPS_I_TEMPERATURES_CMDS             (9)  //corresponding to MAC CMD: EPS1_GETTEMPERATURESINFO_FUNC_ID
#define EPS_I_CONFIGS_CMDS                  (5)  //corresponding to MAC CMD: EPS1_GETCONFIGURATIONINFO_FUNC_ID

/*
********************************************************************************************
* INTERNAL TYPES DEFINITION
********************************************************************************************
*/

typedef struct
{
    float multiplier;
    void* rowData;
    uint8_t dataSize;
} EPS_I_convertData_struct;

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

typedef uint8_t EPS1_VersionVariant_t;

typedef struct
{
    EPS1_VersionVariant_t eVer_variant;
    uint16_t u16Ver_number;
    int8_t ai8Timestamp[14];
} PACKED_STRUCT EPS1getSoftwareVersionResponseData_t;

typedef struct
{
    uint16_t u16Volt;
    uint16_t u16Curr;
} PACKED_STRUCT EPS1_VA_Pair_t;

typedef struct
{
    EPS1_VA_Pair_t sBatt;
    EPS1_VA_Pair_t sBcr;
    uint16_t u16Vbat_ideal;
} PACKED_STRUCT EPS1_BatteryInfo_t;

typedef struct
{
    EPS1_BatteryInfo_t sBattery;
} PACKED_STRUCT EPS1getBatteryInfoResponseData_t;

typedef struct
{
    EPS1_VA_Pair_t sBus_3v3;
    EPS1_VA_Pair_t sBus_5v;
    uint16_t u16Lup_3v3_volt;
    uint16_t u16Lup_5v_volt;
} PACKED_STRUCT EPS1_BusesInfo_t;

typedef struct
{
    EPS1_BusesInfo_t sBuses;
} PACKED_STRUCT EPS1getBusesInfoResponseData_t;

typedef struct
{
    uint16_t u16Volt;
    uint16_t u16Curr_neg;
    uint16_t u16Curr_pos;
} PACKED_STRUCT EPS1_VAA_Pack_t;

typedef struct
{
    EPS1_VAA_Pack_t sX;
    EPS1_VAA_Pack_t sY;
    EPS1_VAA_Pack_t sZ;
} PACKED_STRUCT EPS1_SolarPanelsInfo_t;

typedef struct
{
    EPS1_SolarPanelsInfo_t sSolar;
} PACKED_STRUCT EPS1getSolarPanelsInfoResponseData_t;

typedef struct
{
    uint32_t u32PowerOn;
    uint32_t u32ChargeCycle;
    uint32_t u32UnderVoltage;
    uint32_t u32ShortcutCircuit;
    uint32_t u32Overtemperature;
} PACKED_STRUCT EPS1_Statistic_Counters_t;

typedef struct
{
    EPS1_Statistic_Counters_t sCounters;
    uint16_t au16MaxTemp[4];
    uint16_t au16MinTemp[4];
    uint16_t u16Rbat_initial;
    uint16_t u16Rbat_current;
    uint16_t u16Reserved;
    uint32_t u32Uptime;
} PACKED_STRUCT EPS1_StatisticInfo_t;

typedef struct
{
    EPS1_StatisticInfo_t sStatistics;
} PACKED_STRUCT EPS1getStatisticInfoResponseData_t;

typedef struct
{
    uint16_t u16Mcu;
    uint16_t au16Battery[8];
} PACKED_STRUCT EPS1_TemperatureInfo_t;

typedef struct
{
    EPS1_TemperatureInfo_t sTemperatures;
} PACKED_STRUCT EPS1getTemperaturesInfoResponseData_t;

typedef struct
{
    uint16_t u16InputConditions;
    uint16_t au16OutputConditions[2];
    uint16_t au16DefaultOutputs[2];
} PACKED_STRUCT EPS1_ConfigurationInfo_t;

typedef struct
{
    EPS1_ConfigurationInfo_t sConfig;
} PACKED_STRUCT EPS1getConfigurationInfoResponseData_t;

static EPS1getSoftwareVersionResponseData_t EPS_I_VersionInfo;
static EPS1getBatteryInfoResponseData_t EPS_I_BatInfo;
static EPS1getBusesInfoResponseData_t EPS_I_BusesInfo;
static EPS1getSolarPanelsInfoResponseData_t EPS_I_SolarPanels;
static EPS1getStatisticInfoResponseData_t EPS_I_Statistics;
static EPS1getTemperaturesInfoResponseData_t EPS_I_Temperaturess;
static EPS1getConfigurationInfoResponseData_t EPS_I_Config;
static uint16_t EPS_RawData[EPS_I_R_REG_NUMBER];

const EPS_ReadRegs_enum cmdParamsAddresses_BatInfo[EPS_I_BAT_INFO_NUMBER_OLD_CMDS] =
{
    EPS_I_R_REG_BATTERY_VOLTAGE,
    EPS_I_R_REG_BATTERY_CURRENT,
    EPS_I_R_REG_BCR_VOLTAGE,
    EPS_I_R_REG_BCR_CURRENT,
    EPS_I_R_REG_VBAT_IDEAL
};

const EPS_ReadRegs_enum cmdParamsAddresses_BussesInfo[EPS_I_BUSSES_INFO_NUMBER_OLD_CMDS] =
{
    EPS_I_R_REG_BUS_3V3,
    EPS_I_R_REG_3_3V_CURRENT,
    EPS_I_R_REG_BUS_5V,
    EPS_I_R_REG_5V_CURRENT,
    EPS_I_R_REG_LUP_3_3V,
    EPS_I_R_REG_LUP_5V
};

const EPS_ReadRegs_enum cmdParamsAddresses_PanlesInfo[EPS_I_PANS_INFO_NUMBER_OLD_CMDS] =
{
    EPS_I_R_REG_X_VOLTAGE,
    EPS_I_R_REG_X_M_CURRENT,
    EPS_I_R_REG_X_P_CURRENT,
    EPS_I_R_REG_Y_VOLTAGE,
    EPS_I_R_REG_Y_M_CURRENT,
    EPS_I_R_REG_Y_P_CURRENT,
    EPS_I_R_REG_Z_VOLTAGE,
    EPS_I_R_REG_Z_M_CURRENT,
    EPS_I_R_REG_Z_P_CURRENT
};

const EPS_ReadRegs_enum cmdParamsAddresses_Statistics[EPS_I_NUMBER_STATISTICS_CMDS] =
{
    EPS_I_R_REG_POWER_ON_CYCLES,
    EPS_I_R_REG_CHARGE_CYCLES,
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
    EPS_I_R_REG_RBAT_INIT,
    EPS_I_R_REG_RBAT,
    EPS_I_R_REG_MSB_UPTIMER,
    EPS_I_R_REG_lSB_UPTIMER
};

const EPS_ReadRegs_enum cmdParamsAddresses_Temperatures[EPS_I_TEMPERATURES_CMDS] =
{
    EPS_I_R_REG_MCU_TEMP,
    EPS_I_R_REG_BATT_TEMP_SENS_1,
    EPS_I_R_REG_BATT_TEMP_SENS_2,
    EPS_I_R_REG_BATT_TEMP_SENS_3,
    EPS_I_R_REG_BATT_TEMP_SENS_4,
    EPS_I_R_REG_TEMP_SENSOR_5,
    EPS_I_R_REG_TEMP_SENSOR_6,
    EPS_I_R_REG_TEMP_SENSOR_7,
    EPS_I_R_REG_RESERVE_TEMP_SENS_8
};

const EPS_ReadRegs_enum cmdParamsAddresses_Configurations[EPS_I_CONFIGS_CMDS] =
{
    EPS_I_R_REG_INPUT_CONDITION,
    EPS_I_R_REG_OUTPUTS_CONDITIONS,
    EPS_I_R_REG_OUTPUTS_CONDITIONS_2,
    EPS_I_R_REG_DEFAULTS_1,
    EPS_I_R_REG_DEFAULTS_2
};

static EPS_I_convertData_struct const EPS_RowToFloatConstants[EPS_I_R_REG_NUMBER] =
{
/* EPS_I_R_REG_dummy                */  /* dummy */                 {1             , NULL, 0},
/* EPS_I_R_REG_BATTERY_VOLTAGE      */  /* Battery Voltage      */  {0.0023394775  , &EPS_I_BatInfo.sBattery.sBatt.u16Volt                     , sizeof(EPS_I_BatInfo.sBattery.sBatt.u16Volt                     )},
/* EPS_I_R_REG_BATTERY_CURRENT      */  /* Battery Current      */  {3.0517578     , &EPS_I_BatInfo.sBattery.sBatt.u16Curr                     , sizeof(EPS_I_BatInfo.sBattery.sBatt.u16Curr                     )},
/* EPS_I_R_REG_BCR_VOLTAGE          */  /* BCR Voltage          */  {0.0023394775  , &EPS_I_BatInfo.sBattery.sBcr.u16Volt                      , sizeof(EPS_I_BatInfo.sBattery.sBcr.u16Volt                      )},
/* EPS_I_R_REG_BCR_CURRENT          */  /* BCR Current          */  {1.5258789     , &EPS_I_BatInfo.sBattery.sBcr.u16Curr                      , sizeof(EPS_I_BatInfo.sBattery.sBcr.u16Curr                      )},
/* EPS_I_R_REG_X_VOLTAGE            */  /* X   Voltage          */  {0.0024414063  , &EPS_I_SolarPanels.sSolar.sX.u16Volt                      , sizeof(EPS_I_SolarPanels.sSolar.sX.u16Volt                      )},
/* EPS_I_R_REG_X_M_CURRENT          */  /* X-  Current          */  {0.6103516     , &EPS_I_SolarPanels.sSolar.sX.u16Curr_neg                  , sizeof(EPS_I_SolarPanels.sSolar.sX.u16Curr_neg                  )},
/* EPS_I_R_REG_X_P_CURRENT          */  /* X+ Current           */  {0.6103516     , &EPS_I_SolarPanels.sSolar.sX.u16Curr_pos                  , sizeof(EPS_I_SolarPanels.sSolar.sX.u16Curr_pos                  )},
/* EPS_I_R_REG_Y_VOLTAGE            */  /* Y   Voltage          */  {0.0024414063  , &EPS_I_SolarPanels.sSolar.sY.u16Volt                      , sizeof(EPS_I_SolarPanels.sSolar.sY.u16Volt                      )},
/* EPS_I_R_REG_Y_M_CURRENT          */  /* Y-  Current          */  {0.6103516     , &EPS_I_SolarPanels.sSolar.sY.u16Curr_neg                  , sizeof(EPS_I_SolarPanels.sSolar.sY.u16Curr_neg                  )},
/* EPS_I_R_REG_Y_P_CURRENT          */  /* Y+ Current           */  {0.6103516     , &EPS_I_SolarPanels.sSolar.sY.u16Curr_pos                  , sizeof(EPS_I_SolarPanels.sSolar.sY.u16Curr_pos                  )},
/* EPS_I_R_REG_Z_VOLTAGE            */  /* Z   Voltage          */  {0.0024414063  , &EPS_I_SolarPanels.sSolar.sZ.u16Volt                      , sizeof(EPS_I_SolarPanels.sSolar.sZ.u16Volt                      )},
/* EPS_I_R_REG_Z_M_CURRENT          */  /* Z-  Current          */  {0.6103516     , &EPS_I_SolarPanels.sSolar.sZ.u16Curr_neg                  , sizeof(EPS_I_SolarPanels.sSolar.sZ.u16Curr_neg                  )},
/* EPS_I_R_REG_Z_P_CURRENT          */  /* Z+ Current           */  {0.6103516     , &EPS_I_SolarPanels.sSolar.sZ.u16Curr_pos                  , sizeof(EPS_I_SolarPanels.sSolar.sZ.u16Curr_pos                  )},
/* EPS_I_R_REG_3_3V_CURRENT         */  /* 3.3V Current         */  {2.0345052     , &EPS_I_BusesInfo.sBuses.sBus_3v3.u16Curr                  , sizeof(EPS_I_BusesInfo.sBuses.sBus_3v3.u16Curr                  )},
/* EPS_I_R_REG_5V_CURRENT           */  /* 5V Current           */  {2.0345052     , &EPS_I_BusesInfo.sBuses.sBus_5v.u16Curr                   , sizeof(EPS_I_BusesInfo.sBuses.sBus_5v.u16Curr                   )},
/* EPS_I_R_REG_LUP_3_3V             */  /* LUP 3.3V             */  {0.0023394775  , &EPS_I_BusesInfo.sBuses.u16Lup_3v3_volt                   , sizeof(EPS_I_BusesInfo.sBuses.u16Lup_3v3_volt                   )},
/* EPS_I_R_REG_LUP_5V               */  /* LUP 5V               */  {0.0023394775  , &EPS_I_BusesInfo.sBuses.u16Lup_5v_volt                    , sizeof(EPS_I_BusesInfo.sBuses.u16Lup_5v_volt                    )},
/* EPS_I_R_REG_MCU_TEMP             */  /* MCU Temp             */  {0.0006103516  , &EPS_I_Temperaturess.sTemperatures.u16Mcu                 , sizeof(EPS_I_Temperaturess.sTemperatures.u16Mcu                 )}, // - 0.986 )/0.00355
/* EPS_I_R_REG_BATT_TEMP_SENS_1     */  /* Batt. Temp. Sens. 1  */  {0.00390625    , &EPS_I_Temperaturess.sTemperatures.au16Battery[0]         , sizeof(EPS_I_Temperaturess.sTemperatures.au16Battery[0]         )},
/* EPS_I_R_REG_BATT_TEMP_SENS_2     */  /* Batt. Temp. Sens. 2  */  {0.00390625    , &EPS_I_Temperaturess.sTemperatures.au16Battery[1]         , sizeof(EPS_I_Temperaturess.sTemperatures.au16Battery[1]         )},
/* EPS_I_R_REG_BATT_TEMP_SENS_3     */  /* Batt. Temp. Sens. 3  */  {0.00390625    , &EPS_I_Temperaturess.sTemperatures.au16Battery[2]         , sizeof(EPS_I_Temperaturess.sTemperatures.au16Battery[2]         )},
/* EPS_I_R_REG_BATT_TEMP_SENS_4     */  /* Batt. Temp. Sens. 4  */  {0.00390625    , &EPS_I_Temperaturess.sTemperatures.au16Battery[3]         , sizeof(EPS_I_Temperaturess.sTemperatures.au16Battery[3]         )},
/* EPS_I_R_REG_INPUT_CONDITION      */  /* Input Condition      */  {1             , &EPS_I_Config.sConfig.u16InputConditions                  , sizeof(EPS_I_Config.sConfig.u16InputConditions                  )},
/* EPS_I_R_REG_OUTPUTS_CONDITIONS   */  /* Outputs Conditions   */  {1             , &EPS_I_Config.sConfig.au16OutputConditions[0]             , sizeof(EPS_I_Config.sConfig.au16OutputConditions[0]             )},
/* EPS_I_R_REG_OUTPUTS_CONDITIONS_2 */  /* Outputs Conditions 2 */  {1             , &EPS_I_Config.sConfig.au16OutputConditions[1]             , sizeof(EPS_I_Config.sConfig.au16OutputConditions[1]             )},
/* EPS_I_R_REG_POWER_ON_CYCLES      */  /* Power_ON_Cycles      */  {1             , &EPS_I_Statistics.sStatistics.sCounters.u32PowerOn        , sizeof(EPS_I_Statistics.sStatistics.sCounters.u32PowerOn        )},
/* EPS_I_R_REG_V_UNDER_VOLTAGE      */  /* V_Under_Voltage      */  {1             , &EPS_I_Statistics.sStatistics.sCounters.u32UnderVoltage   , sizeof(EPS_I_Statistics.sStatistics.sCounters.u32UnderVoltage   )},
/* EPS_I_R_REG_V_SHORT_CIRCUIT      */  /* V_Short_Circuit      */  {1             , &EPS_I_Statistics.sStatistics.sCounters.u32ShortcutCircuit, sizeof(EPS_I_Statistics.sStatistics.sCounters.u32ShortcutCircuit)},
/* EPS_I_R_REG_V_OVER_TEMPERATURE   */  /* V_Over Temperature   */  {1             , &EPS_I_Statistics.sStatistics.sCounters.u32Overtemperature, sizeof(EPS_I_Statistics.sStatistics.sCounters.u32Overtemperature)},
/* EPS_I_R_REG_MAX_TEMP_1           */  /* MAX_Temp 1           */  {0.00390625    , &EPS_I_Statistics.sStatistics.au16MaxTemp[0]              , sizeof(EPS_I_Statistics.sStatistics.au16MaxTemp[0]              )},
/* EPS_I_R_REG_MAX_TEMP_2           */  /* MAX_Temp 2           */  {0.00390625    , &EPS_I_Statistics.sStatistics.au16MaxTemp[1]              , sizeof(EPS_I_Statistics.sStatistics.au16MaxTemp[1]              )},
/* EPS_I_R_REG_MAX_TEMP_3           */  /* MAX_Temp 3           */  {0.00390625    , &EPS_I_Statistics.sStatistics.au16MaxTemp[2]              , sizeof(EPS_I_Statistics.sStatistics.au16MaxTemp[2]              )},
/* EPS_I_R_REG_MAX_TEMP_4           */  /* MAX_Temp 4           */  {0.00390625    , &EPS_I_Statistics.sStatistics.au16MaxTemp[3]              , sizeof(EPS_I_Statistics.sStatistics.au16MaxTemp[3]              )},
/* EPS_I_R_REG_MIN_TEMP_1           */  /* MIN_Temp 1           */  {0.00390625    , &EPS_I_Statistics.sStatistics.au16MinTemp[0]              , sizeof(EPS_I_Statistics.sStatistics.au16MinTemp[0]              )},
/* EPS_I_R_REG_MIN_TEMP_2           */  /* MIN_Temp 2           */  {0.00390625    , &EPS_I_Statistics.sStatistics.au16MinTemp[1]              , sizeof(EPS_I_Statistics.sStatistics.au16MinTemp[1]              )},
/* EPS_I_R_REG_MIN_TEMP_3           */  /* MIN_Temp 3           */  {0.00390625    , &EPS_I_Statistics.sStatistics.au16MinTemp[2]              , sizeof(EPS_I_Statistics.sStatistics.au16MinTemp[2]              )},
/* EPS_I_R_REG_MIN_TEMP_4           */  /* MIN_Temp 4           */  {0.00390625    , &EPS_I_Statistics.sStatistics.au16MinTemp[3]              , sizeof(EPS_I_Statistics.sStatistics.au16MinTemp[3]              )},
/* EPS_I_R_REG_TEMP_SENSOR_5        */  /* Temp Sensor 5        */  {0.00390625    , &EPS_I_Temperaturess.sTemperatures.au16Battery[4]         , sizeof(EPS_I_Temperaturess.sTemperatures.au16Battery[4]         )},
/* EPS_I_R_REG_TEMP_SENSOR_6        */  /* Temp Sensor 6        */  {0.00390625    , &EPS_I_Temperaturess.sTemperatures.au16Battery[5]         , sizeof(EPS_I_Temperaturess.sTemperatures.au16Battery[5]         )},
/* EPS_I_R_REG_TEMP_SENSOR_7        */  /* Temp Sensor 7        */  {0.00390625    , &EPS_I_Temperaturess.sTemperatures.au16Battery[6]         , sizeof(EPS_I_Temperaturess.sTemperatures.au16Battery[6]         )},
/* EPS_I_R_REG_RESERVE_TEMP_SENS_8  */  /* Reserve- Temp Sens 8 */  {0.00390625    , &EPS_I_Temperaturess.sTemperatures.au16Battery[7]         , sizeof(EPS_I_Temperaturess.sTemperatures.au16Battery[7]         )},
/* EPS_I_R_REG_SW_VERSION           */  /* SW version           */  {1             , &EPS_I_VersionInfo.u16Ver_number                          , sizeof(EPS_I_VersionInfo.u16Ver_number                          )},
/* EPS_I_R_REG_DEFAULTS_1           */  /* Defaults 1           */  {1             , &EPS_I_Config.sConfig.au16DefaultOutputs[0]               , sizeof(EPS_I_Config.sConfig.au16DefaultOutputs[0]               )},
/* EPS_I_R_REG_DEFAULTS_2           */  /* Defaults 2           */  {1             , &EPS_I_Config.sConfig.au16DefaultOutputs[1]               , sizeof(EPS_I_Config.sConfig.au16DefaultOutputs[1]               )},
/* EPS_I_R_REG_RBAT_INIT            */  /* Rbat initial         */  {1.4972656     , &EPS_I_Statistics.sStatistics.u16Rbat_initial             , sizeof(EPS_I_Statistics.sStatistics.u16Rbat_initial             )},
/* EPS_I_R_REG_RBAT                 */  /* Rbat                 */  {1.4972656     , &EPS_I_Statistics.sStatistics.u16Rbat_current             , sizeof(EPS_I_Statistics.sStatistics.u16Rbat_current             )},
/* EPS_I_R_REG_VBAT_IDEAL           */  /* Vbat ideal           */  {2.3394775     , &EPS_I_BatInfo.sBattery.u16Vbat_ideal                     , sizeof(EPS_I_BatInfo.sBattery.u16Vbat_ideal                     )},
/* EPS_I_R_REG_CHARGE_CYCLES        */  /* Reseved cmd          */  {1             , &EPS_I_Statistics.sStatistics.sCounters.u32ChargeCycle    , sizeof(EPS_I_Statistics.sStatistics.sCounters.u32ChargeCycle    )},
/* EPS_I_R_REG_BUS_3V3              */  /* V bus 3.3V           */  {2.3394775     , &EPS_I_BusesInfo.sBuses.sBus_3v3.u16Volt                  , sizeof(EPS_I_BusesInfo.sBuses.sBus_3v3.u16Volt                  )},
/* EPS_I_R_REG_BUS_5V               */  /* V bus 5V             */  {2.3394775     , &EPS_I_BusesInfo.sBuses.sBus_5v.u16Volt                   , sizeof(EPS_I_BusesInfo.sBuses.sBus_5v.u16Volt                   )},
/* EPS_I_R_REG_MSB_UPTIMER          */  /* MSB uptime           */  {1             , &EPS_I_Statistics.sStatistics.u32Uptime                   , sizeof(EPS_I_Statistics.sStatistics.u32Uptime                   )},
/* EPS_I_R_REG_lSB_UPTIMER          */  /* LSB uptime           */  {1             , &EPS_I_Statistics.sStatistics.u32Uptime                   , sizeof(EPS_I_Statistics.sStatistics.u32Uptime                   )}
};


/*
********************************************************************************************
* INTERNAL (STATIC) ROUTINES DECLARATION
********************************************************************************************
*/

/*
********************************************************************************************
* EXTERNAL (NONE STATIC) ROUTINES DEFINITION
********************************************************************************************
*/
void EPS_I_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_GPIO_WritePin(EPS_RST_GPIO_Port, EPS_RST_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = EPS_RST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EPS_RST_GPIO_Port, &GPIO_InitStruct);
}



void EPS_I_HwReset(void)
{
    HAL_GPIO_WritePin(EPS_RST_GPIO_Port, EPS_RST_Pin, GPIO_PIN_RESET);
    osDelay(2);
    HAL_GPIO_WritePin(EPS_RST_GPIO_Port, EPS_RST_Pin, GPIO_PIN_SET);
}


/*!
********************************************************************************************
* @brief Writes a register for the EPS and then reads the value back to make sure it is the same
********************************************************************************************
* @param[input]      paramNumber - a register number from enum EPS_WriteRegs_enum
* @param[input]      WriteData - the value of the register
* @param[output]     none
* @return            HAL_StatusTypeDef - Status of the requested operation
* @note              none
********************************************************************************************
*/
HAL_StatusTypeDef EPS_VerifyWrite_Register(EPS_WriteRegs_enum paramNumber, EPS_eOUTPUT_STAT WriteData)
{
    HAL_StatusTypeDef retVal = HAL_ERROR;

    if ((paramNumber < EPS_I_W_REG_NUMBER) && (WriteData < Mass_eLAST))
    {
        static const struct RwLut
        {
            uint8_t regNumb;
            uint8_t bitNumer;
        } VerifyLUT[EPS_I_W_REG_NUMBER] =
            {
                /*  EPS_I_W_REG_SWSelfLock */  { 24, 0x00 },
                /*  EPS_I_W_REG_VBATTEN    */  { 24, 0x01 },
                /*  EPS_I_W_REG_BCROutEN   */  { 24, 0x02 },
                /*  EPS_I_W_REG_SHD_3_3V   */  { 24, 0x03 },
                /*  EPS_I_W_REG_SHD_5V     */  { 24, 0x04 },
                /*  EPS_I_W_REG_LUP3_3V    */  { 25, 0x00 },
                /*  EPS_I_W_REG_LUP5V      */  { 25, 0x01 },
                /*  EPS_I_W_REG_SHDChrg    */  { 25, 0x02 },
                /*  EPS_I_W_REG_Chrg_I1    */  { 25, 0x03 },
                /*  EPS_I_W_REG_Chrg_I2    */  { 25, 0x04 },
                /*  EPS_I_W_REG_OUT1       */  { 24, 0x07 },
                /*  EPS_I_W_REG_OUT2       */  { 24, 0x08 },
                /*  EPS_I_W_REG_OUT3       */  { 24, 0x09 },
                /*  EPS_I_W_REG_OUT4       */  { 24, 0x0A },
                /*  EPS_I_W_REG_OUT5       */  { 24, 0x0B },
                /*  EPS_I_W_REG_OUT6       */  { 24, 0x0C },
                /*  EPS_I_W_REG_HEETER1    */  { 24, 0x0D },
                /*  EPS_I_W_REG_HEETER2    */  { 24, 0x0E },
                /*  EPS_I_W_REG_HEETER3    */  { 24, 0x0F },
                /*  EPS_I_W_REG_10KPULLUP  */  { 25, 0x05 },
                /*  EPS_I_W_REG_4K7PULLUP  */  { 25, 0x06 },
                /*  EPS_I_W_REG_R_TERMIN   */  { 25, 0x07 }
            };


        for (uint8_t i = 0; i < EPS_I_I2C_MAX_RETRY; i++)
        {
            retVal = EPS_Write_Register(paramNumber, WriteData);

            if (HAL_OK == retVal)
            {
                uint16_t regVal;
                retVal = EPS_Read_Register(VerifyLUT[paramNumber].regNumb, 1, &regVal);

                if (((regVal >> VerifyLUT[paramNumber].bitNumer) & 0x01) == (WriteData & 0x01))
                {
                    retVal = HAL_OK;
                    break;
                }
            }
        }
    }

    return retVal;
}


/*!
********************************************************************************************
* @brief Set a output of the EPS and read back its state
********************************************************************************************
* @param[input]      MCU_Init_OutStates_struct - bit-field with allowed outputs to be changed and the state to change into
* @param[output]     none
* @return            none
* @note              none
********************************************************************************************
*/
void EPS_I_SetEpsOutputs(MCU_Init_OutStates_struct *states)
{
    uint8_t outsToChange = states->OutToChange & EPS_I_OUTPUTS_MASKS_ALL;

    if (outsToChange)
    {
        if ((outsToChange >> MCU_INIT_OUT1) & 0x01)
        {
            EPS_VerifyWrite_Register(EPS_I_W_REG_OUT1, (states->OutState >> MCU_INIT_OUT1) & 0x01);
        }

        if ((outsToChange >> MCU_INIT_OUT2) & 0x01)
        {
            EPS_VerifyWrite_Register(EPS_I_W_REG_OUT2, (states->OutState >> MCU_INIT_OUT2) & 0x01);
        }

        if ((outsToChange >> MCU_INIT_OUT3) & 0x01)
        {
            EPS_VerifyWrite_Register(EPS_I_W_REG_OUT3, (states->OutState >> MCU_INIT_OUT3) & 0x01);
        }

        // Do not change the output of the EPS that switches the On/Off the power of the OBC (prevent cutting off the own power)
        //    if((outsToChange >> MCU_INIT_OUT4_6)&0x01)
        //    {
        //        EPS_VerifyWrite_Register(EPS_I_W_REG_OUT6, (states->OutState >> MCU_INIT_OUT4_6)&0x01);
        //    }

        if ((outsToChange >> MCU_INIT_OUT5) & 0x01)
        {
            EPS_VerifyWrite_Register(EPS_I_W_REG_OUT5, (states->OutState >> MCU_INIT_OUT5) & 0x01);
        }
    }
}


/*!
********************************************************************************************
* @brief Read a register for the EPS and converts the row value to a float number
********************************************************************************************
* @param[input]      paramNumber - Number of the register. First register is 0x01, the last one
*                     is 48. For more information, please see the I2C user manual of the EPS
* @param[input]      ReadType  + 0 - Get last read value from the buffer.
*                              + 1 - Read the registers from the UHF antenna and refresh the values from the buffer
* @param[output]     *convertedValue - the
* @param[output]     none
* @return            HAL_StatusTypeDef:
*                               + HAL_OK -> the value is converted to human readable value
*                               + HAL_ERROR -> wrong parameter, or chosen parameter that has no float value (like a counter)
* @note              none
********************************************************************************************
*/
volatile uint8_t debug_paramNumber;

HAL_StatusTypeDef EPS_ReadConvertedValue(EPS_ReadRegs_enum paramNumber, uint8_t ReadType, float * convertedValue)
{
    HAL_StatusTypeDef retStat = HAL_ERROR;
    debug_paramNumber = paramNumber;

    float tempCovertValue = 0;

    if ((paramNumber < EPS_I_R_REG_NUMBER) && (paramNumber > EPS_I_R_REG_dummy))
    {
        if (EPS_RowToFloatConstants[paramNumber].multiplier != 1)
        {
            uint16_t ReadData;

            if (ReadType != 0)   //read the value or use the last ever read
            {
                //refresh the register value in the buffer
                retStat = EPS_Read_Register(paramNumber, 1, &ReadData);

                if (HAL_OK != retStat)
                {
                    return retStat;
                }
            }

            tempCovertValue = EPS_RawData[paramNumber] * EPS_RowToFloatConstants[paramNumber].multiplier;

            if (paramNumber == EPS_I_R_REG_MCU_TEMP) // if CPU voltage is calculated the equation is more complicated
            {
                tempCovertValue = (tempCovertValue - 0.986) / 0.00355;
            }

            *convertedValue = tempCovertValue;

        }
        // else - the parameter does not have a float value
    }

    return retStat;
}


/*!
********************************************************************************************
* @brief Read a register for the EPS
********************************************************************************************
* @param[input]      paramNumber - Number of the register. First register is 0x01, the last one
*                     is 48. For more information, please see the I2C user manual of the EPS
* @param[input]      ReadType  + 0 - Get last read value from the buffer.
*                              + 1 - Read the registers from the EPS and refresh the values from the buffer
* @param[output]     *Status - the value of the requested register
* @param[output]     *ReadData - the value of the register
* @return            HAL_StatusTypeDef - Status of the requested operation
* @note              none
********************************************************************************************
*/
HAL_StatusTypeDef EPS_Read_Register(EPS_ReadRegs_enum paramNumber, uint8_t ReadType, uint16_t * ReadData)
{
    HAL_StatusTypeDef I2C_retStat = HAL_ERROR;
    uint8_t temp_reg_buff[2];

    if ((paramNumber < EPS_I_R_REG_NUMBER) && (paramNumber > EPS_I_R_REG_dummy))
    {
        if (ReadType == 0)
        {
            *ReadData = EPS_RawData[paramNumber];
        }
        else
        {
            I2C_retStat = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);
            if (I2C_retStat == HAL_OK)
            {
                for (uint8_t i = 0; i < EPS_I_I2C_MAX_RETRY; i++)
                {
                    I2C_retStat = MX_I2C_BlockingMemRead(MX_I2C_BUS_SYSTEM, EPS_I_I2C_ADDRESS, (uint16_t)paramNumber, sizeof(uint8_t), temp_reg_buff, sizeof(uint16_t));
                    osDelay(EPS_I_I2C_TIME_BETWEEN_PACS); //give some delay between packages just in case of overload of the MCU
                    if (I2C_retStat == HAL_OK)
                    {
                        break;
                    }
                }
                MX_I2C_Release(MX_I2C_BUS_SYSTEM);
            }

            if (HAL_OK == I2C_retStat)
            {
                //the register is read successfully

                //Refresh the buffer and return the new data
                *ReadData = EPS_RawData[paramNumber] = (temp_reg_buff[0] << 8) + temp_reg_buff[1];
                //return HAL_OK;
            }
            else
            {
                //return I2C_retStat;
                EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_EPS_COMM, eEXEH_EPS_EXCEPTION_ID_COMM_ERROR, __LINE__);
            }
        }
    }
    else
    {
        I2C_retStat = HAL_ERROR;
    }

    return I2C_retStat;
}

/*!
********************************************************************************************
* @brief Read a register for the EPS and converts the row value to a float number
********************************************************************************************
* @param[input]      paramNumber - a register number from enum EPS_WriteRegs_enum
* @param[input]      WriteData - the value of the register
* @param[output]     none
* @return            HAL_StatusTypeDef - Status of the requested operation
* @note              none
********************************************************************************************
*/
HAL_StatusTypeDef EPS_Write_Register(EPS_WriteRegs_enum paramNumber, uint8_t WriteData)
{
    HAL_StatusTypeDef I2C_retStat;
    uint8_t temp_reg_buff[2];

    temp_reg_buff[0] = paramNumber;
    temp_reg_buff[1] = WriteData;

    if (paramNumber < EPS_I_W_REG_NUMBER)
    {
        I2C_retStat = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);
        if (I2C_retStat == HAL_OK)
        {
            for (uint8_t i = 0; i < EPS_I_I2C_MAX_RETRY; i++)
            {
                I2C_retStat = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, EPS_I_I2C_ADDRESS, temp_reg_buff, sizeof(uint16_t));
                osDelay(EPS_I_I2C_TIME_BETWEEN_PACS); //give some delay between packages just in case of overload of the MCU

                if (I2C_retStat == HAL_OK)
                {
                    break;
                }
            }
            MX_I2C_Release(MX_I2C_BUS_SYSTEM);
        }

        if (I2C_retStat != HAL_OK)
        {
            EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_EPS_COMM, eEXEH_EPS_EXCEPTION_ID_COMM_ERROR, __LINE__);
        }
    }
    else
    {
        I2C_retStat = HAL_ERROR;
    }

    return I2C_retStat;
}


/*!
********************************************************************************************
* @brief Get EPS_I rawData conversion value
********************************************************************************************
* @param[input]      EPS_ReadRegs_enum eParamNumber - parameter number
* @param[output]     none
* @return            float - the conversion value (multiplier)
* @note              none
********************************************************************************************
*/
float EPS_I_GetRawDataConvValue(EPS_ReadRegs_enum eParamNumber)
{
    return EPS_RowToFloatConstants[eParamNumber].multiplier;
}


#endif

/* ******************************************************************************************* */
