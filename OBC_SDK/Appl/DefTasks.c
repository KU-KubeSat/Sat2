/*!
********************************************************************************************
* @file DefTasks.c
* @brief Implementation of common purposes task-related logic
********************************************************************************************
* @author            Georgi Georgiev
* @version           1.0.0
* @date              2020.01.16
*
* @copyright         (C) Copyright Endurosat
*
*                    Contents and presentations are protected world-wide.
*                    Any kind of using, copying etc. is prohibited without prior permission.
*                    All rights - incl. industrial property rights - are reserved.
*
* @history
* @revision{         1.0.0  , 2020.01.16, author Georgi Georgie, Initial revision }
* @endhistory
********************************************************************************************
*/

/*
********************************************************************************************
* INCLUDES
********************************************************************************************
*/
#include <time.h>
#include "AppTasks.h"
#include "TaskMonitor.h"
#include "AntUHF.h"
#include "BootLdr.h"
#include "Svc_RTC.h"
#include "PwrMng.h"
#include "CommandsApi.h"
#include "SdMngr.h"
#include <stdint.h>

/*
********************************************************************************************
* INTERNAL DEFINES
********************************************************************************************
*/
#define APP_LED_ON_TIME             (50)                                /* given time in ms */
#define APP_TASK_CALL_PERIOD        (1000)                              /* given time in ms */

/*
********************************************************************************************
* INTERNAL TYPES DEFINITION
********************************************************************************************
*/
#define UPTIME_1000MS        (1000)

/*
********************************************************************************************
* EXTERNAL VARIABLES DEFINITION
********************************************************************************************
*/
uint8_t  up_sec=0, up_min=0, up_hrs=0;
uint32_t up_day=0;
uint32_t obcUptimeInSeconds=0;

/*
********************************************************************************************
* INTERNAL (STATIC) VARIABLES DEFINITION/DECLARATION
********************************************************************************************
*/
static uint32_t ServicesTask_RTC_StartTime;

static osThreadId_t ServicesTask_TaskHandle;
static const osThreadAttr_t ServicesTask_attributes = {
  .name = "ServicesTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 12
};

/*
********************************************************************************************
* INTERNAL (STATIC) ROUTINES DECLARATION
********************************************************************************************
*/
static void InitCompleted(void);
static void ServicesTask(void * argument);

/*
********************************************************************************************
* EXTERNAL (NONE STATIC) ROUTINES DEFINITION
********************************************************************************************
*/
/*!
*********************************************************************************************
* @brief Init routine for the UhfKeepAlive component
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void ServicesTask_Init(void)
{
    ServicesTask_RTC_StartTime = ServicesTask_RTC_Seconds();

    ServicesTask_TaskHandle = osThreadNew(ServicesTask, NULL, &ServicesTask_attributes);
    configASSERT( ServicesTask_TaskHandle );
}

/*!
********************************************************************************************
* @brief Routine utilised by the task, that handles all service operations.
********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
********************************************************************************************
*/
static void ServicesTask(void * argument)
{
    SdMngr_RequestSdCardInit(SD_MNGR_CARD_BOTH, pdTRUE);

    HAL_RTC_GetTime(&hrtc, &sTime, CALENDAR_FORMAT);
    HAL_RTC_GetDate(&hrtc, &sDate, CALENDAR_FORMAT);

    CpuTemprInit();
    MCU_SunSensInit();

    MCU_MagnetometersInit();

    MCU_TemprInit();
    MCU_GyrosInit();

    MCU_MagnetorqersInit();
    MCU_SunSensInit();

    TaskMonitor_TaskInitialized(TASK_MONITOR_SERVICES);   /* The task is initialized and is ready */

    while(1)
    {
        TaskMonitor_IamAlive(TASK_MONITOR_SERVICES); /* Prevent from WatchDog reset */

        Svc_RtcTask();

        InitCompleted();

#if (ENABLE_UHF_ANT_SUPPORT == 1)
        AntUHF_Handler();
#endif

        osDelay(SERVICE_TASK_PERIOD);

#if defined(BOOTLOADER)
        BootLdr_CheckAutoFlashCmd();
#endif

        vApplicationLowStackCheck(TASK_MONITOR_SERVICES);
    }
}

void vApplicationIdleHook( void )
{
    // Ram / Rom check
    // Some functions in service task can run here
}


/*!
********************************************************************************************
* @brief Counts time since power up
********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
********************************************************************************************
*/
void Uptime(void)
{
    static uint16_t delayPeriod = 0;

    delayPeriod += SERVICE_TASK_PERIOD;

    while( delayPeriod >= UPTIME_1000MS )
    {
        /* +1 second */
        up_sec++;
        obcUptimeInSeconds++;

        if (up_sec >= 60)
        {
          up_sec = 0;
          up_min++;
          if (up_min >= 60)
          {
            up_min = 0;
            up_hrs++;
            if (up_hrs >= 24)
            {
              up_hrs = 0;
              up_day++;
            }
          }
        }

        if( delayPeriod >= UPTIME_1000MS )
            delayPeriod -= UPTIME_1000MS; /* - 1 second */
    }
}

/*!
********************************************************************************************
* @brief Read the number of seconds since power up
********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
********************************************************************************************
*/
uint32_t GetObcUptimeSeconds(void)
{
    return obcUptimeInSeconds;
}

/*
********************************************************************************************
* INTERNAL (STATIC) ROUTINES DEFINITION
********************************************************************************************
*/
/*!
********************************************************************************************
* @brief Check if all tasks has been initialised and if yes - prints a string once.
********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
********************************************************************************************
*/
static void InitCompleted(void)
{
    static uint8_t Complete = 0;

    if( Complete == 0 )
    {
        if( TaskMonitor_CheckTasksInit() )
        {
            fprintf(SYSCON, "OK+OBC initialisation completed\r\n"); /* OBC GUI application needs that string to end or start a operation */
            fprintf(SYSCON,
                "RST Counters: "
                "0.WWD = %03d / "
                "1.IWD = %03d / "
                "2.LPR = %03d / "
                "3.POR = %03d / "
                "4.RstPin = %03d / "
                "5.BOR = %03d / "
                "6.HardFault = %03d / "
                "7.MemFault = %03d / "
                "8.BusFault = %03d / "
                "9.UsageFault = %03d\r\n",
                    (int) BootData->RST_WWD,
                    (int) BootData->RST_IWD,
                    (int) BootData->RST_LPR,
                    (int) BootData->RST_POR,
                    (int) BootData->RST_RstPin,
                    (int) BootData->RST_BOR,
                    (int) BootData->RST_HardFault,
                    (int) BootData->RST_MemFault,
                    (int) BootData->RST_BusFault,
                    (int) BootData->RST_UsageFault);

            fprintf(COMM ,"OK+OBC initialisation completed\r\n"); /* OBC GUI application needs that string to end or start a operation */
            fprintf(COMM,
                "RST Counters: "
                "0.WWD = %03d / "
                "1.IWD = %03d / "
                "2.LPR = %03d / "
                "3.POR = %03d / "
                "4.RstPin = %03d / "
                "5.BOR = %03d / "
                "6.HardFault = %03d / "
                "7.MemFault = %03d / "
                "8.BusFault = %03d / "
                "9.UsageFault = %03d\r\n",
                    (int) BootData->RST_WWD,
                    (int) BootData->RST_IWD,
                    (int) BootData->RST_LPR,
                    (int) BootData->RST_POR,
                    (int) BootData->RST_RstPin,
                    (int) BootData->RST_BOR,
                    (int) BootData->RST_HardFault,
                    (int) BootData->RST_MemFault,
                    (int) BootData->RST_BusFault,
                    (int) BootData->RST_UsageFault);

            Complete = 1;
        }
    }

}

uint32_t ServicesTask_RTC_Seconds(void)
{
    taskENTER_CRITICAL();

    RtcDtTm_t rtcDateTime = ReadHandler_RTCDateTime();

    taskEXIT_CRITICAL();

    struct tm tm;
    tm.tm_year = rtcDateTime.date.year + 100;
    tm.tm_mon = rtcDateTime.date.month - 1;
    tm.tm_mday = rtcDateTime.date.day;
    tm.tm_hour = rtcDateTime.time.hours;
    tm.tm_min = rtcDateTime.time.minutes;
    tm.tm_sec = rtcDateTime.time.seconds;

    uint32_t ret = mktime(&tm);

    return ret;
}

uint32_t ServicesTask_UptimeSeconds(void)
{
    return ( ServicesTask_RTC_Seconds() - ServicesTask_RTC_StartTime );
}

/* **************************************************************************************** */
