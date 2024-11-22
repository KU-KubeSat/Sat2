/*!
*********************************************************************************************
* @file CommandsApi.c
* @brief This file provides the interfaces for all commands that can be executed through any communication
*********************************************************************************************
* @author            Vassil Milev
* @version           1.0.0
* @date              2020.07.14
*
* @copyright         (C) Copyright Endurosat
*
*                    Contents and presentations are protected world-wide.
*                    Any kind of using, copying etc. is prohibited without prior permission.
*                    All rights - incl. industrial property rights - are reserved.
*
* @history
* @revision{         1.0.0  , 2020.07.14, author Vassil Milev, Initial revision }
* @endhistory
*********************************************************************************************
*/

/*
*********************************************************************************************
* INCLUDES
*********************************************************************************************
*/
#include "CommandsApi.h"
#include "DAT_Inputs.h"
#include "panels.h"
#include "EEPROM_emul.h"
#include "main.h"
#include "AppTasks.h"
#include "TaskMonitor.h"
#include "BootLdr.h"
#include "LIS3MDL_MAG_driver.h"
#include <string.h>

/*
*********************************************************************************************
* INTERNAL DEFINES
*********************************************************************************************
*/
#ifndef MIN
  #define MIN(a, b)   (((a) <= (b)) ? ((a)) : ((b)))
#endif

/*
*********************************************************************************************
* INTERNAL TYPES DEFINITION
*********************************************************************************************
*/
/* No Internal types definition */
typedef struct
{
    GPIO_TypeDef *outPortAddr;
    uint16_t pinMask;
} writeHandlerGpioConfig_t;

/*
*********************************************************************************************
* EXTERNAL VARIABLES DEFINITION
*********************************************************************************************
*/
/* No External variables definition */

/**
* @brief Variable description
*/


/*
*********************************************************************************************
* INTERNAL (STATIC) VARIABLES DEFINITION/DECLARATION 
*********************************************************************************************
*/
/* No Internal variables definition/declaration */
static const uint8_t compassAddr[COMPASSID_MAX] =
{
    LIS3MDL_MAG_I2C_ADDRESS_LOW,
    LIS3MDL_MAG_I2C_ADDRESS_HIGH
};

/*
*********************************************************************************************
* INTERNAL (STATIC) ROUTINES DECLARATION
*********************************************************************************************
*/
/* No Internal routines declaration */

/*
*********************************************************************************************
* EXTERNAL (NONE STATIC) ROUTINES DEFINITION
*********************************************************************************************
*/
/*!
*********************************************************************************************
* @brief Init routine for the CommandsApi component
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void CommandsApi_Init(void)
{
	
}

CommApi_Compass_XYZ_Cmd_str ReadHandler_Compass_XYZ(CompassId_t id)
{
    CommApi_Compass_XYZ_Cmd_str retValue;

    if (id < sizeof(compassAddr) / sizeof(compassAddr[0]))
    {
        Magnetometers_Take();

        if (Magnitometers_LIS3MDL_Read_Data(&retValue.data, compassAddr[id], MAGNETODATAFORMAT_RAW) != E_OK)
        {
            retValue.status = SEN_ERROR;
        }
        else
            retValue.status = SEN_SUCCESS;

        Magnetometers_Release(DISABLE);
    }
    else
        retValue.status = SEN_ERROR;

    return retValue;
}

CommApi_Compass_reg_Cmd_str ReadHandler_Compass_Reg(CompassId_t id, uint8_t registerNum)
{
    CommApi_Compass_reg_Cmd_str retValue;

    if (id < sizeof(compassAddr) / sizeof(compassAddr[0]))
    {
        Magnetometers_Take();

        retValue.status = LIS3MDL_MAG_ReadReg(compassAddr[id], registerNum, (uint8_t *) &retValue.data);

        Magnetometers_Release(DISABLE);
    }
    else
        retValue.status = SEN_ERROR;

    return retValue;
}

status_t WriteHandler_Compass_Reg(CompassId_t id, uint8_t registerNum, uint8_t writeData)
{
    status_t retValue;

    if (id < sizeof(compassAddr) / sizeof(compassAddr[0]))
    {
        if (Magnetometers_ReadState() == 0)
            retValue = SEN_DISABLED;
        else
        {
            retValue = LIS3MDL_MAG_WriteReg(compassAddr[id], registerNum, writeData);
        }
    }
    else
        retValue = SEN_ERROR;

    return retValue;
}

CommApi_Gyro_axis_Cmd_str ReadHandler_Gyro_Metric(uint8_t panel)
{
    CommApi_Gyro_axis_Cmd_str retValue;
    status_t readValue;
    int16_t axisValue;

    Gyro_Take();

    readValue = ADIS16265_GetAxesRate(panel, &axisValue);

    if ((readValue == SEN_SUCCESS) && (axisValue != 0x7FFF))
    {
        retValue.data = axisValue;
        retValue.status = SEN_SUCCESS;
    }
    else
    {
        retValue.data = (int16_t)0x0000;
        retValue.status = SEN_ERROR;
    }

    Gyro_Release(DISABLE);

    return retValue;
}

CommApi_Gyro_axis_Cmd_str ReadHandler_Gyro_Angle(uint8_t panel)
{
    CommApi_Gyro_axis_Cmd_str retValue;
    status_t readValue;
    int16_t axisValue;

    Gyro_Take();

    readValue = ADIS16265_GetAxesAngle(panel, &axisValue);

    if ((readValue == SEN_SUCCESS) && (axisValue != 0x7FFF))
    {
        retValue.data = axisValue;
        retValue.status = SEN_SUCCESS;
    }
    else
    {
        retValue.data = (int16_t)0x0000;
        retValue.status = SEN_ERROR;
    }

    Gyro_Release(DISABLE);

    return retValue;
}

CommApi_Compass_reg_Cmd_str ReadHandler_Gyro_Reg(PanId_t panId, uint8_t registerNum)
{
    CommApi_Compass_reg_Cmd_str retValue;
    uint16_t utmp;

    if (panId < MAX_PAN)
    {
        Gyro_Take();

        if (Panels_IsAttached(panId))
        {
            retValue.status = ADIS16265_ReadReg16(registerNum, (uint16_t *) &utmp, panId);
            retValue.data = utmp;
        }
        else
            retValue.status = SEN_ERROR;

        Gyro_Release(DISABLE);
    }
    else
        retValue.status = SEN_ERROR;

    return retValue;
}

status_t WriteHandler_Gyro_Reg(PanId_t panId, uint8_t registerNum, uint16_t writeData)
{
    uint8_t retValue;

    if ((panId < MAX_PAN) && (Panels_IsAttached(panId)))
    {
        if (Gyro_ReadState() == 0)
            retValue = SEN_DISABLED;
        else
            retValue = ADIS16265_WriteReg16(registerNum, writeData, panId);
    }
    else
        retValue = SEN_ERROR;

    return retValue;
}

CommApi_Magnetorq_Pwr_Cmd_str ReadHandler_MagnetorquerData(uint8_t panel)
{
    CommApi_Magnetorq_Pwr_Cmd_str retValue;
    PANLE_TRQ_str data;

    retValue.state = MagnTrq_ReadState();
    if (retValue.state)
    {
        retValue.status = GetMagnetorque(panel, &data);
        retValue.power = data.power;
        retValue.direction = data.dir;
    }
    else
    {
        retValue.status = SEN_DISABLED;
        retValue.power = 0;
        retValue.direction = 0;
    }

    return retValue;
}

CommApi_Temp_Cmd_str ReadHandler_Temp(void)
{
    const int16_t invalidRawTempValue = (int16_t) 0xFFFF;
    CommApi_Temp_Cmd_str retValue;
    status_t status;
    uint8_t idx;
    int16_t rawDataVec[6];
    Temperature_t tmpTemp;

    // initialise return values
    (void) memset(retValue.statusVec, (int) SEN_ERROR, sizeof(retValue.statusVec));
    (void) memset(retValue.rawDataVec, (int) invalidRawTempValue, sizeof(retValue.rawDataVec));
    (void) memset(retValue.degCDataVec, (int) invalidRawTempValue, sizeof(retValue.degCDataVec));

    PanTempSens_Take();

    status = TMP122_GetTemperatureP(&tmpTemp);

    // Plus sensors
    if (status == SEN_SUCCESS)
    {
        rawDataVec[0] = tmpTemp.Temp_X;
        rawDataVec[1] = tmpTemp.Temp_Y;
        rawDataVec[2] = tmpTemp.Temp_Z;

        // first half of the result vector
        for (idx = 0; idx < (MAX_PAN / 2); idx++)
        {
            if (Panels_IsAttached(idx))
            {
                retValue.statusVec[idx] = SEN_SUCCESS;
                retValue.rawDataVec[idx] = rawDataVec[idx];
                retValue.degCDataVec[idx] = TMP122_getTemperatureInDegC(rawDataVec[idx]);
            }
            else
                retValue.statusVec[idx] = SEN_ERROR;
        }
    }

    // Minus sensors
    status = TMP122_GetTemperatureM(&tmpTemp);

    if (status == SEN_SUCCESS)
    {
        rawDataVec[3] = tmpTemp.Temp_X;
        rawDataVec[4] = tmpTemp.Temp_Y;
        rawDataVec[5] = tmpTemp.Temp_Z;

        // second half of result vector
        for (idx = (MAX_PAN / 2); idx < MAX_PAN; idx++)
        {
            if (Panels_IsAttached(idx))
            {
                retValue.statusVec[idx] = SEN_SUCCESS;
                retValue.rawDataVec[idx] = rawDataVec[idx];
                retValue.degCDataVec[idx] = TMP122_getTemperatureInDegC(rawDataVec[idx]);
            }
            else
                retValue.statusVec[idx] = SEN_ERROR;
        }
    }

    PanTempSens_Release(DISABLE);

    return retValue;
}

PanPhotometricInfo_t ReadHandler_PhotometricInfo(void)
{
    PanPhotometricInfo_t data;

    SunSens_Take();

    Panel_GetPhotodiodesLum();

    (void) memcpy(data.sensorReadings, PanelLight, MIN(sizeof(data.sensorReadings), sizeof(PanelLight)));

    SunSens_Release(DISABLE);

    return data;
}

GpioStat_t ReadHandler_GpOutputState(void)
{
    GpioStat_t retValue = { .gpioStatusBitField = 0x00 };

    retValue.gpioStatusBitField  = HAL_GPIO_ReadPin(OBC_OUT1_GPIO_Port, OBC_OUT1_Pin) << MCU_INIT_OUT1;
    retValue.gpioStatusBitField |= HAL_GPIO_ReadPin(OBC_OUT2_GPIO_Port, OBC_OUT2_Pin) << MCU_INIT_OUT2;
    retValue.gpioStatusBitField |= HAL_GPIO_ReadPin(OBC_OUT3_GPIO_Port, OBC_OUT3_Pin) << MCU_INIT_OUT3;
    retValue.gpioStatusBitField |= HAL_GPIO_ReadPin(OBC_OUT5_GPIO_Port, OBC_OUT5_Pin) << MCU_INIT_OUT5;
    retValue.gpioStatusBitField |= HAL_GPIO_ReadPin(OBC_OUT4_6_GPIO_Port, OBC_OUT4_6_Pin) << MCU_INIT_OUT4_6;
    retValue.gpioStatusBitField |= HAL_GPIO_ReadPin(OBC_OUT7_GPIO_Port, OBC_OUT7_Pin) << MCU_INIT_OUT7;
    retValue.gpioStatusBitField |= HAL_GPIO_ReadPin(OBC_OUT8_GPIO_Port, OBC_OUT8_Pin) << MCU_INIT_OUT8;

    return retValue;
}

HwRes_t WriteHandler_GpOutputState(uint8_t pin, uint8_t value)
{
    HwRes_t status = HWRES_ERROR;
    MCU_Init_OutStates_struct states;

    if (pin < MCU_INIT_OUT_MAX)
    {
        states.OutToChange = 1 << pin;
        states.OutState = value << pin;
        MX_GPIO_Outputs_Set(&states);
        status = HWRES_SUCCESS;
    }

    return status;
}

I2CPUState_t ReadHandler_I2CPullUpsState(void)
{
    I2CPUState_t result =
    {
        .SystemBus = EEPROM_emul_DataTemp.I2cPullUpResistors[MCU_INIT_I2C_PULL_UP_SYS],
        .PayloadBus = EEPROM_emul_DataTemp.I2cPullUpResistors[MCU_INIT_I2C_PULL_UP_PAY]
    };

    return result;
}

void WriteHandler_I2CPullUpsState(I2CPUState_t newState)
{
    EEPROM_emul_DataTemp.I2cPullUpResistors[MCU_INIT_I2C_PULL_UP_SYS] = newState.SystemBus;
    EEPROM_emul_DataTemp.I2cPullUpResistors[MCU_INIT_I2C_PULL_UP_PAY] = newState.PayloadBus;

    // Sync EEPROM emulated memory with the buffer "EEPROM_emul_DataTemp"
    EEPROM_Emul_SyncInfo();

    // Change the state of the Pull Up resistors right away
    MCU_Init_I2cPullUps();
}

RtcTm_t ReadHandler_RTCTime(void)
{
    RtcTm_t retTime =
    {
        .isTimeValid = false
    };

    MX_RTC_ReloadShadowRegs();

    if (HAL_OK == HAL_RTC_GetTime(&hrtc, &sTime, CALENDAR_FORMAT))
    {
        retTime.hours = sTime.Hours;
        retTime.minutes = sTime.Minutes;
        retTime.seconds = sTime.Seconds;
        retTime.isTimeValid = true;
    }

    // this call is needed after each HAL_RTC_GetTime() call to synchronize RTC date/time shadow registers
    HAL_RTC_GetDate(&hrtc, &sDate, CALENDAR_FORMAT);

    return retTime;
}

static inline bool isTimeValid(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    return (hours < 24) && (minutes < 60) && (seconds < 60);
}

StdResult_t WriteHandler_RTCTime(RtcTm_t newTime)
{
    StdResult_t res;

    if (isTimeValid(newTime.hours, newTime.minutes, newTime.seconds))
    {
        sTime.Hours   = newTime.hours;
        sTime.Minutes = newTime.minutes;
        sTime.Seconds = newTime.seconds;

        res = (HAL_OK == HAL_RTC_SetTime(&hrtc, &sTime, CALENDAR_FORMAT)) ? STDRESULT_SUCCESS : STDRESULT_ERROR;
    }
    else
        res = STDRESULT_INVALID_ARGS;

    return res;
}

RtcDt_t ReadHandler_RTCDate(void)
{
    RtcDt_t retDate =
    {
        .isDateValid = false
    };

    MX_RTC_ReloadShadowRegs();

    if (HAL_OK == HAL_RTC_GetDate(&hrtc, &sDate, CALENDAR_FORMAT))
    {
        retDate.day = sDate.Date;
        retDate.month = sDate.Month;
        retDate.year = sDate.Year;
        retDate.dayOfWeek = sDate.WeekDay;
        retDate.isDateValid = true;
    }

    return retDate;
}

static bool isDateValid(uint16_t year, uint16_t month, uint16_t day)
{
    static const uint8_t daysInMonth[12] =
    {
       (uint8_t) 31,  // Jan
       (uint8_t) 28,  // Feb
       (uint8_t) 31,  // Mar
       (uint8_t) 30,  // Apr
       (uint8_t) 31,  // May
       (uint8_t) 30,  // Jun
       (uint8_t) 31,  // Jul
       (uint8_t) 31,  // Aug
       (uint8_t) 30,  // Sep
       (uint8_t) 31,  // Oct
       (uint8_t) 30,  // Nov
       (uint8_t) 31   // Dec
    };

    uint8_t maxDays;
    bool isValid = false;

    if ((month >= 1) && (month <= 12) && (day >= 1))
    {
        maxDays = daysInMonth[month - 1U];

        // check if year is leap
        if ((month == 2) && ((((year % 4 == 0) && (year % 100 != 0))) || (year % 400 == 0)))
        {
            ++maxDays;
        }

        isValid = (day <= maxDays);
    }

    return isValid;
}


StdResult_t WriteHandler_RTCDate(RtcDt_t newDate)
{
    StdResult_t res;

    if ((isDateValid(newDate.year, newDate.month, newDate.day)) &&
        (newDate.dayOfWeek >= RTC_WEEKDAY_MONDAY) &&
        (newDate.dayOfWeek <= RTC_WEEKDAY_SUNDAY))
    {
        sDate.Year  = newDate.year;
        sDate.Month = newDate.month;
        sDate.Date  = newDate.day;
        sDate.WeekDay = newDate.dayOfWeek;

        res = (HAL_OK == HAL_RTC_SetDate(&hrtc, &sDate, CALENDAR_FORMAT)) ?
                STDRESULT_SUCCESS :
                STDRESULT_ERROR;
    }
    else
        res = STDRESULT_INVALID_ARGS;

    return res;
}

RtcDtTm_t ReadHandler_RTCDateTime(void)
{
    RtcDtTm_t retDateTime =
    {
        {
            .isDateValid = false
        },
        {
            .isTimeValid = false
        }
    };

    MX_RTC_ReloadShadowRegs();

    if (HAL_OK == HAL_RTC_GetTime(&hrtc, &sTime, CALENDAR_FORMAT))
    {
        retDateTime.time.hours = sTime.Hours;
        retDateTime.time.minutes = sTime.Minutes;
        retDateTime.time.seconds = sTime.Seconds;
        retDateTime.time.isTimeValid = true;
    }

    // this call is needed after each HAL_RTC_GetTime() call to synchronize RTC date/time shadow registers
    if (HAL_OK == HAL_RTC_GetDate(&hrtc, &sDate, CALENDAR_FORMAT))
    {
        retDateTime.date.day = sDate.Date;
        retDateTime.date.month = sDate.Month;
        retDateTime.date.year = sDate.Year;
        retDateTime.date.dayOfWeek = sDate.WeekDay;
        retDateTime.date.isDateValid = true;
    }

    return retDateTime;
}

StdResult_t WriteHandler_RTCDateTime(RtcDtTm_t newDateTime)
{
    StdResult_t res;

    if (isTimeValid(newDateTime.time.hours, newDateTime.time.minutes, newDateTime.time.seconds) &&
        (isDateValid(newDateTime.date.year, newDateTime.date.month, newDateTime.date.day)) &&
        (newDateTime.date.dayOfWeek >= RTC_WEEKDAY_MONDAY) &&
        (newDateTime.date.dayOfWeek <= RTC_WEEKDAY_SUNDAY))
    {
        sTime.Hours   = newDateTime.time.hours;
        sTime.Minutes = newDateTime.time.minutes;
        sTime.Seconds = newDateTime.time.seconds;

        res = (HAL_OK == HAL_RTC_SetTime(&hrtc, &sTime, CALENDAR_FORMAT)) ? STDRESULT_SUCCESS : STDRESULT_ERROR;

        if (res == STDRESULT_SUCCESS)
        {
            sDate.Year  = newDateTime.date.year;
            sDate.Month = newDateTime.date.month;
            sDate.Date  = newDateTime.date.day;
            sDate.WeekDay = newDateTime.date.dayOfWeek;

            res = (HAL_OK == HAL_RTC_SetDate(&hrtc, &sDate, CALENDAR_FORMAT)) ? STDRESULT_SUCCESS : STDRESULT_ERROR;
        }
    }
    else
        res = STDRESULT_INVALID_ARGS;

    return res;
}

UpTimeInfo_t ReadHandler_Uptime(void)
{
    UpTimeInfo_t uptime =
    {
        .days = up_day,
        .hours = up_hrs,
        .minutes = up_min,
        .seconds = up_sec
    };

    return uptime;
}

StdResult_t WriteHandler_Uptime(UpTimeInfo_t newUptime)
{
    StdResult_t res = STDRESULT_ERROR;

    if (isTimeValid(newUptime.hours, newUptime.minutes, newUptime.seconds))
    {
        up_day = newUptime.days;
        up_hrs = newUptime.hours;
        up_min = newUptime.minutes;
        up_sec = newUptime.seconds;

        res = STDRESULT_SUCCESS;
    }
    else
        res = STDRESULT_INVALID_ARGS;

    return res;
}

bool ReadHandler_BeaconNumber(uint8_t beaconNo)
{
    bool res = 0;

    return res;
}

StdResult_t WriteHandler_FaultTest(FaultTstId_t id)
{
    StdResult_t res = STDRESULT_SUCCESS;

#if (ENABLE_ESTTC_FAULT_TESTS == 1)
    typedef void(*func)(void);
	
    switch (id)
    {
        case FAULTTSTID_WWD_RESET:
        {
            hwwdg.Instance = WWDG;
            hwwdg.Init.Prescaler = WWDG_PRESCALER_1;
            hwwdg.Init.Counter = 64;
            hwwdg.Init.Window = 64;
            hwwdg.Init.EWIMode = WWDG_EWI_ENABLE;

            if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
                Error_Handler();
        } break;

        case FAULTTSTID_IWD_RESET:
        {
            hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
            hiwdg.Init.Reload = 1;

            if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
                Error_Handler();
        } break;

        case FAULTTSTID_HARDFAULT: //SVC CALL or any other
        {
            // Disable all fault handlers (only hard fault enabled)
            SCB->SHCSR &= ~((uint32_t)(SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk));
            func function = (func)0xFFFFFFFF;
            function();
        } break;

        case FAULTTSTID_MEMFAULT:
        {
            // branch into the System region (XN region) (0xE0000000 - 0xFFFFFFFF)
            func function = (func)0xFFFFFFFF;
            function();
        } break;

        case FAULTTSTID_BUSFAULT:
        {
            volatile uint8_t* byte = (uint8_t*)0xFFFFFFFF;
            *byte = (uint8_t)0;
        } break;

        case FAULTTSTID_USAGEFAULT:
        {
            volatile float a = 1.f;
            volatile float b = 0.f;
            volatile float c = a / b;

            (void)c;
        } break;

        case FAULTTSTID_MAX:
        default:
        {
            res = STDRESULT_INVALID_ARGS;
        } break;
    }
#else
    res = STDRESULT_ERROR;
#endif

    return res;
}


#if (ENABLE_POWER_MANAGER == 1)

PowerModeInfo_t ReadHandler_PowerModeInfo(void)
{
    PowerModeInfo_t res;

    res.pwMode = PwrMng_GetMode();
    res.settings.sleepPeriod = PwrMng_GetSleepPeriod();
    res.settings.stayAwakePeriod = PwrMng_GetStayAwakePeriod();

    return res;
}

PwrModeSettingsResult WriteHandler_PowerModeSettings(const PwrModeSettings_t *pNewConfig)
{
    PwrModeSettingsResult res = PWRMODESETTINGSRESULT_ERROR;
    uint8_t awakePeriodCurrent;

    if (pNewConfig != NULL)
    {
        awakePeriodCurrent = PwrMng_GetSleepPeriod();

        if (PwrMng_SetSleepPeriod(pNewConfig->sleepPeriod))
        {
            if (PwrMng_SetStayAwakePeriod(pNewConfig->stayAwakePeriod))
            {
                res = PWRMODESETTINGSRESULT_SUCCESS;
            }
            else
            {
                // restore sleep period in case of error
                PwrMng_SetSleepPeriod(awakePeriodCurrent);
                res = PWRMODESETTINGSRESULT_INVALID_STAYAWAKE_TIME;
            }
        }
        else
            res = PWRMODESETTINGSRESULT_INVALID_SLEEP_TIME;
    }

    return res;
}

#endif

StdResult_t WriteHandler_ResetInBootOrAppMode(const AppMode_t appMode)
{
    StdResult_t res = STDRESULT_SUCCESS;

    switch (appMode)
    {
        case APPMODE_APPLICATION:
            BootData->Mailbox = MAILBOX_VAL_APPL;
        break;

        case APPMODE_BOOTLOADER:
            BootData->Mailbox = MAILBOX_VAL_BOOT;
        break;

        case APPMODE_AUTO_FW_UPDATE:
            BootData->Mailbox = MAILBOX_VAL_AUTO_FLASH;
        break;

        case APPMODE_MAX:
        default:
            res = STDRESULT_INVALID_ARGS;
        break;
    }

    if (res == STDRESULT_SUCCESS)
    {
        Svc_Rtc_ClearResetCounter(RSTCOUNTERID_ALL);
        BootData->RebootRequest = pdTRUE;
        // set valid checksum for the RTC backup registers after changing the mailbox
        RTC_SetRTC_BKP_CRC();

        // Reset the OBC
        TaskMonitor_TriggerDelayedReset(1000U);
    }

    return res;
}

SensInUseData_t ReadHandler_SensorsInUse(const ObcSensorId_t sensor)
{
    SensInUseData_t result =
    {
        .isSensorValid = false,
        .u8UsersCount = (uint8_t) 0
    };

    if (sensor < OBCSENSORID_MAX)
    {
        result.isSensorValid = true;
        result.u8UsersCount = sensorSem_ReadState((ObcSensorId_t) sensor);
    }

    return result;
}

SensInUseData_t WriteHandler_SensorsInUse(const ObcSensorId_t sensor, uint8_t cmdId)
{
    SensInUseData_t result =
    {
        .isSensorValid = false,
        .u8UsersCount = (uint8_t) 0
    };

    if (sensor >= OBCSENSORID_MAX)
        return result;

    if ((cmdId == SENCTRL_RELEASE_FOR_ONE_USER) ||
        (cmdId == SENCTRL_RESERVE_FOR_ONE_USER) ||
        (cmdId == SENCTRL_RELEASE_FOR_ALL_AND_OFF))
    {
        switch (cmdId)
        {
            case SENCTRL_RESERVE_FOR_ONE_USER:
                sensorSem_Take(sensor);

                result.isSensorValid = true;
                result.u8UsersCount = sensorSem_ReadState(sensor);
            break;

            case SENCTRL_RELEASE_FOR_ONE_USER:
                sensorSem_Release(sensor, DISABLE);

                result.isSensorValid = true;
                result.u8UsersCount = sensorSem_ReadState(sensor);
            break;

            case SENCTRL_RELEASE_FOR_ALL_AND_OFF:
                sensorSem_Release(sensor, ENABLE);

                result.isSensorValid = true;
                result.u8UsersCount = sensorSem_ReadState(sensor);
            break;

            default:
            break;
        }
    }

    return result;
}


/*
*********************************************************************************************
* INTERNAL (STATIC) ROUTINES DEFINITION
*********************************************************************************************
*/
/* No Internal routines definition */

/* ******************************************************************************************* */
