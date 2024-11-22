/*!
*********************************************************************************************
* @file CommandsApi.h
* @brief Header of CommandsApi.c .
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
#ifndef COMMANDSAPI_H
#define COMMANDSAPI_H

/*
*********************************************************************************************
* INCLUDES
*********************************************************************************************
*/
#include "User_types.h"
#include "PwrMng.h"
#include "panels.h"
#include "DAT_inputs.h"


/*
*********************************************************************************************
* EXTERNAL DEFINES
*********************************************************************************************
*/
#define SENCTRL_RELEASE_FOR_ONE_USER    ((uint8_t) 0x00)
#define SENCTRL_RESERVE_FOR_ONE_USER    ((uint8_t) 0x01)
#define SENCTRL_RELEASE_FOR_ALL_AND_OFF ((uint8_t) 0x5A)

#define DATA_VALID_DAY_OF_WEEK          ((uint8_t) 0x01)

/*
*********************************************************************************************
* EXTERNAL TYPES DECLARATIONS
*********************************************************************************************
*/
typedef enum
{
    COMPASSID_LOW,
    COMPASSID_HIGH,
    COMPASSID_MAX
} CompassId_t;

#pragma pack(push)
#pragma pack(1)

typedef struct
{
    status_t status;
    Compass_Axis_t data;
} CommApi_Compass_XYZ_Cmd_str;

typedef struct
{
    status_t status;
    uint16_t data;
} CommApi_Compass_reg_Cmd_str;

typedef struct
{
    status_t status;
    int16_t data;
} CommApi_Gyro_axis_Cmd_str;

typedef struct
{
    status_t status;
    uint8_t power;
    uint8_t direction;
    uint8_t state;
} CommApi_Magnetorq_Pwr_Cmd_str;

typedef struct
{
    status_t statusVec[MAX_PAN];
    int16_t rawDataVec[MAX_PAN];
    int16_t degCDataVec[MAX_PAN];
} CommApi_Temp_Cmd_str;

typedef enum
{
    HWRES_SUCCESS,
    HWRES_ERROR,
    HWRES_DISABLED
} HwRes_t;

typedef enum
{
    STDRESULT_SUCCESS,
    STDRESULT_ERROR,
    STDRESULT_INVALID_ARGS
} StdResult_t;

typedef struct
{
    uint8_t gpioStatusBitField;
} GpioStat_t;

typedef struct
{
    uint16_t sensorReadings[MAX_PAN];
} PanPhotometricInfo_t;

typedef struct
{
    uint8_t SystemBus;
    uint8_t PayloadBus;
} I2CPUState_t;

typedef struct
{
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    bool isTimeValid;
} RtcTm_t;

typedef struct
{
    uint8_t day;
    uint8_t month;
    uint16_t year;
    uint8_t dayOfWeek;
    bool isDateValid;
} RtcDt_t;

typedef struct
{
    RtcDt_t date;
    RtcTm_t time;
} RtcDtTm_t;

typedef struct
{
    uint32_t days;
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
} UpTimeInfo_t;

typedef enum
{
    FAULTTSTID_WWD_RESET,
    FAULTTSTID_IWD_RESET,
    FAULTTSTID_HARDFAULT,
    FAULTTSTID_MEMFAULT,
    FAULTTSTID_BUSFAULT,
    FAULTTSTID_USAGEFAULT,
    FAULTTSTID_MAX
} FaultTstId_t;

typedef struct
{
    uint16_t sleepPeriod;
    uint16_t stayAwakePeriod;
} PwrModeSettings_t;

typedef struct
{
    PwrMng_modes_enum pwMode;
    PwrModeSettings_t settings;
} PowerModeInfo_t;

typedef enum
{
    PWRMODESETTINGSRESULT_SUCCESS,
    PWRMODESETTINGSRESULT_ERROR,
    PWRMODESETTINGSRESULT_INVALID_SLEEP_TIME,
    PWRMODESETTINGSRESULT_INVALID_STAYAWAKE_TIME,
    PWRMODESETTINGSRESULT_MAX
} PwrModeSettingsResult;

typedef enum
{
    APPMODE_APPLICATION,
    APPMODE_BOOTLOADER,
    APPMODE_AUTO_FW_UPDATE,
    APPMODE_MAX
} AppMode_t;

typedef struct
{
    bool isSensorValid;
    uint8_t u8UsersCount;
} SensInUseData_t;

typedef enum
{
    SLEEPMODESETTINGSRESULT_SUCCESS,
    SLEEPMODESETTINGSRESULT_ERROR
} SetSleepModeSettingsResult;

typedef enum
{
    IDLEMODESETTINGSRESULT_SUCCESS,
    IDLEMODESETTINGSRESULT_ERROR
} SetIdleModeSettingsResult;


#pragma pack(pop)

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
void CommandsApi_Init(void);

CommApi_Compass_XYZ_Cmd_str ReadHandler_Compass_XYZ(CompassId_t id);
CommApi_Compass_reg_Cmd_str ReadHandler_Compass_Reg(CompassId_t id, uint8_t registerNum);
status_t WriteHandler_Compass_Reg(CompassId_t id, uint8_t registerNum, uint8_t writeData);
CommApi_Gyro_axis_Cmd_str ReadHandler_Gyro_Metric(PANLE_GYROS_AXIS axisId);
CommApi_Gyro_axis_Cmd_str ReadHandler_Gyro_Angle(PANLE_GYROS_AXIS axisId);
CommApi_Compass_reg_Cmd_str ReadHandler_Gyro_Reg(PanId_t panId, uint8_t registerNum);
status_t WriteHandler_Gyro_Reg(PanId_t panId, uint8_t registerNum, uint16_t writeData);
CommApi_Magnetorq_Pwr_Cmd_str ReadHandler_MagnetorquerData(uint8_t panel);
CommApi_Temp_Cmd_str ReadHandler_Temp(void);
// FIDL
PanPhotometricInfo_t ReadHandler_PhotometricInfo(void);
GpioStat_t ReadHandler_GpOutputState(void);
HwRes_t WriteHandler_GpOutputState(uint8_t pin, uint8_t value);
I2CPUState_t ReadHandler_I2CPullUpsState(void);
void WriteHandler_I2CPullUpsState(I2CPUState_t newState);
RtcTm_t ReadHandler_RTCTime(void);
StdResult_t WriteHandler_RTCTime(RtcTm_t newTime);
RtcDt_t ReadHandler_RTCDate(void);
StdResult_t WriteHandler_RTCDate(RtcDt_t newDate);
RtcDtTm_t ReadHandler_RTCDateTime(void);
StdResult_t WriteHandler_RTCDateTime(RtcDtTm_t newDateTime);
UpTimeInfo_t ReadHandler_Uptime(void);
StdResult_t WriteHandler_Uptime(UpTimeInfo_t newUptime);
bool ReadHandler_BeaconNumber(uint8_t beaconNo);
StdResult_t WriteHandler_FaultTest(FaultTstId_t id);
StdResult_t WriteHandler_ResetInBootOrAppMode(const AppMode_t appMode);
SensInUseData_t ReadHandler_SensorsInUse(const ObcSensorId_t sensor);
SensInUseData_t WriteHandler_SensorsInUse(const ObcSensorId_t sensor, uint8_t cmdId);
#if (ENABLE_POWER_MANAGER == 1)
PowerModeInfo_t ReadHandler_PowerModeInfo(void);
PwrModeSettingsResult WriteHandler_PowerModeSettings(const PwrModeSettings_t *pNewConfig);
#endif


#endif    /* COMMANDSAPI_H */
/* ******************************************************************************************* */
