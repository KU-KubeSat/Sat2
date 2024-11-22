/*!
 *********************************************************************************************
 * @file OEM719.c
 * @brief Driver for GNSS receiver NOVATEL OEM719
 *********************************************************************************************
 * @author
 * @date
 *
 * @copyright         (C) Copyright Endurosat
 *
 *                    Contents and presentations are protected world-wide.
 *                    Any kind of using, copying etc. is prohibited without prior permission.
 *                    All rights - incl. industrial property rights - are reserved.
 *********************************************************************************************
 */
#include "OEM719.h"
#include "GlobalConfig.h"
#if (ENABLE_GNSS_OEM719 == 1)
#include "cmsis_os.h"
#include "semphr.h"
#include "TaskMonitor.h"
#include "MCU_Init.h"
#include "main.h"
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <ctype.h>


/** Power to OEM719 (GNSS) */
#define OEM719_ENABLE_GPIO_PORT                (En_GNSS_GPIO_Port)              /** GPIO port */
#define OEM719_ENABLE_PIN                      (En_GNSS_Pin)                    /** GPIO pin number */


/** OBC to GNSS/USER communication related definitions */
#define OEM719_UART                            ((UART_HandleTypeDef*)COM_GNSS)  /** OBC to GNSS uart connection */
#define OEM719_FILE                            ((FILE*)COM_GNSS)                /** OBC to GNSS FILE* used for formatting data */
#define OEM719_PRINT_TO_USER(f, fmt, ...)      { if (f != NULL)           fprintf(f, fmt, ##__VA_ARGS__);                        }
#define OEM719_WRITE_TO_USER(f, data, data_sz) { if (f != NULL)           fwrite(data, 0, data_sz, f);                           }
#define OEM719_PRINT_TO_GNSS(fmt, ...)         { if (OEM719_FILE != NULL) OEM719_PRINT_TO_USER(OEM719_FILE, fmt, ##__VA_ARGS__); }
#define OEM719_WRITE_TO_GNSS(cmd, cmd_sz)      { if (OEM719_FILE != NULL) OEM719_WRITE_TO_USER(OEM719_FILE, cmd, cmd_sz);        }


/**
 * NOTE:
 * Enabling long commands support requires a lot of preallocated memory.
 */
#define OEM719_LONG_CMD_SUPPORT                (0)

#if (OEM719_LONG_CMD_SUPPORT == 1)
  #define OEM719_RX_BUF_SZ                     (50 * 512)
#else
  #define OEM719_RX_BUF_SZ                     ( 4 * 512)
#endif

#define OEM719_FORBIDDEN_CMDS_COUNT            (6)
#define OEM719_LONG_CMDS_COUNT                 (2)


/** main task notifications list */
typedef enum
{
    _OEM719_NOTIFICATION_CMD       = 0x01, /** notify task for regular command request      */
    _OEM719_NOTIFICATION_CMD_LONG  = 0x02, /** notify task for regular long command request */
    _OEM719_NOTIFICATION_POWER_ON  = 0x04, /** notify task for power on command request     */
    _OEM719_NOTIFICATION_POWER_OFF = 0x08, /** notify task for power off command request    */
} _eNotificationvalues_t;


/**
 * List of forbidden commands.
 * Note that this list is only a recommendation.
 * If you are familiar with OEM719 feel free to remove/edit this list.
 * When adding new elements to this list make sure they are all upper case.
 */
static const char* _oem719ForbiddenCmdsList[OEM719_FORBIDDEN_CMDS_COUNT] =
{
    "ONNEW",
    "ONCHANGED",
    "ONNEXT",
    "ONTIME",
    "ONMARK",
    "SERIALCONFIG",
};


/**
 * List of long commands.
 * These commands require large input buffer on OBC to be able to store incoming data.
 * By default they are treated as forbidden commands.
 * To enable their usage OEM719_LONG_CMD_SUPPORT must be set to 1
 * When adding new elements to this list make sure they are all upper case.
 */
static const char* _oem719LongCmdsList[OEM719_FORBIDDEN_CMDS_COUNT] =
{
    "RXCONFIG",
    "LOG LOGLIST",
};


/** buffer to store data coming from GNSS. modified from interrupt context */
static uint8_t  _oem719RxBuf[OEM719_RX_BUF_SZ] = {0};
static uint8_t  _oem719Byte;
static uint16_t _oem719RxCount  = 0;


/** OBC to user interface */
static FILE* _oem719UserCom = NULL;


/** main task handle and attributes */
static osThreadId_t _oem719TaskHandle;
static const osThreadAttr_t _oem719TaskAttr =
{
    .name       = "OEM719_Task",
    .priority   = (osPriority_t) osPriorityNormal,
    .stack_size = 128 * 8
};


/** semaphore used to prevent multiple access to GNSS functionality */
static SemaphoreHandle_t _oem719SemphrHandle;
static StaticSemaphore_t _oem719SemphrBuf;


static void _OEM719_Turnon (void);
static void _OEM719_Turnoff(void);
static void _OEM719_RxIsrStop (void);
static void _OEM719_RxIsrStart(void);
static bool _OEM719_IsGnssOn  (void);
static void _OEM719_TryExecuteCmdOn(FILE* com);
static void _OEM719_ExecuteCmdOff  (FILE* com);
static void _OEM719_TryExecuteRegularCmd (FILE* com, const uint8_t* rawCmd, const uint32_t rawCmdSz);
static bool _OEM719_IsCmdAbbreviatedAscii(const uint8_t* buf, const uint32_t bufSz);
static void _OEM719_LowerToUpperChar     (uint8_t* buf, const uint32_t bufSz);
static bool _OEM719_CheckForForbiddenCmds(const uint8_t* buf);
static bool _OEM719_CheckForLongCmds     (const uint8_t* buf);


void OEM719_Init(void)
{
    MX_UART5_Init();
    _OEM719_RxIsrStop();

    _oem719SemphrHandle = xSemaphoreCreateBinaryStatic(&_oem719SemphrBuf);
    configASSERT(_oem719SemphrHandle);
    
    _oem719TaskHandle = osThreadNew(OEM719_Task, NULL, &_oem719TaskAttr);
    configASSERT(_oem719TaskHandle);

    xTaskNotify(_oem719TaskHandle, _OEM719_NOTIFICATION_POWER_ON, eSetValueWithoutOverwrite);
    xSemaphoreGive(_oem719SemphrHandle);

    _OEM719_Turnon();
}


void OEM719_Task(void* args)
{
    TaskMonitor_TaskInitialized(TASK_MONITOR_GNSS_OEM719);
    uint32_t notifyVal;

    for (;;)
    {
        // prevent OBC reset
        TaskMonitor_IamAlive(TASK_MONITOR_GNSS_OEM719);

        // a delay is added to prevent the task from executing too often
        // fill free to increase/decrease it
        osDelay(pdMS_TO_TICKS(250));

        if (xSemaphoreTake(_oem719SemphrHandle, 0) == pdTRUE)
        {
            notifyVal = ulTaskNotifyTake(pdTRUE, 0);

            if (notifyVal == _OEM719_NOTIFICATION_POWER_ON)
            {
                // prevent OBC reset and give GNSS some time to boot (~5sec)
                // but also monitor for POWER OFF command
                for (uint8_t i = 0; i < 6; i++)
                {
                    TaskMonitor_IamAlive(TASK_MONITOR_GNSS_OEM719);
                    notifyVal = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
                    if (notifyVal == _OEM719_NOTIFICATION_POWER_OFF)
                        break;
                }
            }

            // give some time for RX buffer to accumulate data and log the result
            // but also monitor for POWER OFF command
            if (notifyVal == _OEM719_NOTIFICATION_CMD)
            {
                notifyVal = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1500));

                _OEM719_RxIsrStop();

                if (_oem719RxCount == 0)
                    OEM719_PRINT_TO_USER(_oem719UserCom, "<ERROR:rx\r\n")
                else
                    OEM719_WRITE_TO_USER(_oem719UserCom, _oem719RxBuf, _oem719RxCount)

            }
            else if (notifyVal == _OEM719_NOTIFICATION_CMD_LONG)
            {
                for (uint8_t i = 0; i < 30; i++)
                {
                    // prevent OBC reset
                    TaskMonitor_IamAlive(TASK_MONITOR_GNSS_OEM719);
                    notifyVal = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
                    if (notifyVal == _OEM719_NOTIFICATION_POWER_OFF)
                        break;
                }

                _OEM719_RxIsrStop();

                if (_oem719RxCount == 0)
                    OEM719_PRINT_TO_USER(_oem719UserCom, "<ERROR:rx\r\n")
                else
                    OEM719_WRITE_TO_USER(_oem719UserCom, _oem719RxBuf, _oem719RxCount)
            }

            if (notifyVal == _OEM719_NOTIFICATION_POWER_OFF)
            {
                // delay is added to prevent excessive calls to POWER ON/OFF
                osDelay(pdMS_TO_TICKS(1000));
            }

            xSemaphoreGive(_oem719SemphrHandle);
        }

        vApplicationLowStackCheck(TASK_MONITOR_GNSS_OEM719);
    }
}


void OEM719_ReceiveChar(void)
{
    if (sizeof(_oem719RxBuf) > _oem719RxCount)
    {
        _oem719RxBuf[_oem719RxCount] = _oem719Byte;
        _oem719RxCount++;
    }

    _OEM719_RxIsrStart();
}


void OEM719_SendRawCmd(FILE* com, uint8_t* rawCmd, uint32_t rawCmdSz)
{
#if (ENABLE_GNSS_OEM719 != 1)
    OEM719_PRINT_TO_USER(com, "<ERROR:usage\r\n")
    return;
#endif

    if (com == NULL)
        return;

    if (rawCmd == NULL || rawCmdSz == 0)
    {
        OEM719_PRINT_TO_USER(com, "<ERROR:usage\r\n")
        return;
    }

    if (!_OEM719_IsCmdAbbreviatedAscii(rawCmd, rawCmdSz))
    {
        OEM719_PRINT_TO_USER(com, "<ERROR:usage\r\n")
        return;
    }

    _OEM719_LowerToUpperChar(rawCmd, rawCmdSz);

    if (_OEM719_CheckForForbiddenCmds(rawCmd))
    {
        OEM719_PRINT_TO_USER(com, "<ERROR:usage\r\n")
        return;
    }

    if (!strcmp((char*)rawCmd, "ON"))
        _OEM719_TryExecuteCmdOn(com);
    else if (!strcmp((char*)rawCmd, "OFF"))
        _OEM719_ExecuteCmdOff(com);
    else
        _OEM719_TryExecuteRegularCmd(com, rawCmd, rawCmdSz);
}


static void _OEM719_TryExecuteCmdOn(FILE* com)
{
    if (_OEM719_IsGnssOn())
    {
        OEM719_PRINT_TO_USER(com, "<OK:on\r\n")
        return;
    }

    if (xSemaphoreTake(_oem719SemphrHandle, 0) == pdFALSE)
    {
        OEM719_PRINT_TO_USER(com, "<OK:busy\r\n")
        return;
    }

    // try to notify the task
    if (pdFAIL == xTaskNotify(_oem719TaskHandle, _OEM719_NOTIFICATION_POWER_ON, eSetValueWithoutOverwrite))
    {
        xSemaphoreGive(_oem719SemphrHandle);
        OEM719_PRINT_TO_USER(com, "<OK:busy\r\n")
        return;
    }

    _OEM719_RxIsrStop();
    _OEM719_Turnon();
    xSemaphoreGive(_oem719SemphrHandle);

    OEM719_PRINT_TO_USER(com, "<OK:on\r\n")
}


static void _OEM719_ExecuteCmdOff(FILE* com)
{
    if (!_OEM719_IsGnssOn())
    {
        OEM719_PRINT_TO_USER(com, "<OK:off\r\n")
        return;
    }

    // notify the task no matter what exactly it is currently doing
    if (pdFAIL == xTaskNotify(_oem719TaskHandle, _OEM719_NOTIFICATION_POWER_OFF, eSetValueWithOverwrite))
    {
        OEM719_PRINT_TO_USER(com, "<ERROR:off\r\n")
        return;
    }

    _OEM719_RxIsrStop();
    _OEM719_Turnoff();

    OEM719_PRINT_TO_USER(com, "<OK:off\r\n")
}


static void _OEM719_TryExecuteRegularCmd(FILE* com, const uint8_t* rawCmd, const uint32_t rawCmdSz)
{
    if (!_OEM719_IsGnssOn())
    {
        OEM719_PRINT_TO_USER(com, "<ERROR:off\r\n")
        return;
    }

    if (xSemaphoreTake(_oem719SemphrHandle, 0) == pdFALSE)
    {
        OEM719_PRINT_TO_USER(com, "<OK:busy\r\n")
        return;
    }

    if (_OEM719_CheckForLongCmds(rawCmd))
    {
        // try to notify the task
        if (pdFAIL == xTaskNotify(_oem719TaskHandle, _OEM719_NOTIFICATION_CMD_LONG, eSetValueWithoutOverwrite))
        {
            xSemaphoreGive(_oem719SemphrHandle);
            OEM719_PRINT_TO_USER(com, "<OK:busy\r\n")
            return;
        }
    }
    else
    {
        // try to notify the task
        if (pdFAIL == xTaskNotify(_oem719TaskHandle, _OEM719_NOTIFICATION_CMD, eSetValueWithoutOverwrite))
        {
            xSemaphoreGive(_oem719SemphrHandle);
            OEM719_PRINT_TO_USER(com, "<OK:busy\r\n")
            return;
        }
    }

    _oem719UserCom = com;

    // prepare for RX interrupt
    memset(_oem719RxBuf, 0, sizeof(_oem719RxBuf));
    _oem719RxCount = 0;

    _OEM719_RxIsrStart();

    // send command to GNSS
    OEM719_WRITE_TO_GNSS(rawCmd, rawCmdSz)
    OEM719_PRINT_TO_GNSS("\r\n")

    xSemaphoreGive(_oem719SemphrHandle);
}


static bool _OEM719_IsCmdAbbreviatedAscii(const uint8_t* buf, const uint32_t bufSz)
{
    for (uint32_t i = 0; i < bufSz; i++)
    {
        if ((buf[i] <  32 ||
             buf[i] > 126) && // (ASCII value 32) and '~' (ASCII value 126) inclusive
            (buf[i] !=  9) && // vertical tab (ASCII value 9)
            (buf[i] != 10) && // line feed (ASCII value 10)
            (buf[i] != 11) && // horizontal tab (ASCII value 11)
            (buf[i] != 13))   // carriage return (ASCII value 13
        {
            return false;
        }

        if (buf[i] == '#')
            return false;
    }

    return true;
}


static void _OEM719_LowerToUpperChar(uint8_t* buf, const uint32_t bufSz)
{
    for (uint32_t i = 0; i < bufSz; i++)
        if (isalpha(buf[i]))
            buf[i] = toupper((char)buf[i]);
}


static bool _OEM719_CheckForForbiddenCmds(const uint8_t* buf)
{
    for (uint32_t i = 0; i < OEM719_FORBIDDEN_CMDS_COUNT; i++)
        if (strstr((char*)buf, _oem719ForbiddenCmdsList[i]))
            return true;

#if (OEM719_LONG_CMD_SUPPORT != 1)
    return _OEM719_CheckForLongCmds(buf);
#else
    return false;
#endif
}


static bool _OEM719_CheckForLongCmds(const uint8_t* buf)
{
    for (uint32_t i = 0; i < OEM719_LONG_CMDS_COUNT; i++)
        if (strstr((char*)buf, _oem719LongCmdsList[i]))
            return true;

    return false;
}


static void _OEM719_RxIsrStop(void)
{
    HAL_StatusTypeDef err = HAL_UART_AbortReceive(OEM719_UART);
    if (HAL_OK != err)
        Error_Handler();
}


static void _OEM719_RxIsrStart(void)
{
    HAL_StatusTypeDef err = HAL_UART_Receive_IT(OEM719_UART, &_oem719Byte, 1);
    if (HAL_OK != err)
        Error_Handler();
}


static void _OEM719_Turnon(void)
{
    HAL_GPIO_WritePin(OEM719_ENABLE_GPIO_PORT, OEM719_ENABLE_PIN, GPIO_PIN_SET);
}


static void _OEM719_Turnoff(void)
{
    HAL_GPIO_WritePin(OEM719_ENABLE_GPIO_PORT, OEM719_ENABLE_PIN, GPIO_PIN_RESET);
}


static bool _OEM719_IsGnssOn(void)
{
    if (GPIO_PIN_SET == HAL_GPIO_ReadPin(OEM719_ENABLE_GPIO_PORT, OEM719_ENABLE_PIN))
        return true;

    return false;
}


#endif

