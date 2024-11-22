/*!
********************************************************************************************
* @file TaskMonitor.h
* @brief Header of TaskMonitor.c
********************************************************************************************
* @author            Vassil Milev
* @version           1.0.0
* @date              2019.02.19
*
* @copyright         (C) Copyright Endurosat
*
*                    Contents and presentations are protected world-wide.
*                    Any kind of using, copying etc. is prohibited without prior permission.
*                    All rights - incl. industrial property rights - are reserved.
*
* @history
* @revision{         1.0.0  , 2019.02.19, author Vassil Milev, Initial revision }
* @endhistory
********************************************************************************************
*/
#ifndef TASKMONITOR_H
#define TASKMONITOR_H

/*
*********************************************************************************************
* INCLUDES
*********************************************************************************************
*/
#include <stdint.h>
#include "GlobalConfig.h"

/*
*********************************************************************************************
* EXTERNAL DEFINES
*********************************************************************************************
*/
/* No External defines*/

/*
*********************************************************************************************
* EXTERNAL TYPES DECLARATIONS
*********************************************************************************************
*/
typedef enum
{
    TASK_MONITOR_ESTTC = 0x00,
    TASK_MONITOR_TASK,
#if (ENABLE_EXEH_PERSIST == 1)
    TASK_MONITOR_EXEH_PERSISTOR,
#endif
    TASK_MONITOR_APP_TASK,
    TASK_MONITOR_SERVICES,
    TASK_MONITOR_SD_MANAGER,
#if (ENABLE_GNSS_OEM719 == 1)
    TASK_MONITOR_GNSS_OEM719,
#endif
    TASK_MONITOR_TASKS_NUMBER
} TASK_MONITOR_TasksEnum;

typedef enum
{
    TASK_MONITOR_SW_RESET,
    TASK_MONITOR_FAULT_RESET,
    TASK_MONITOR_USER_DELAYED_RESET,
    TASK_MONITOR_TASK_TIMEOUT,
    TASK_MONITOR_RESET_NUMBER
} TASK_MONITOR_resetEnum;

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
void TaskMonitor_Init(void);
void TaskMonitor_Task(void * argument);
void TaskMonitor_IamAlive(TASK_MONITOR_TasksEnum task);
void TaskMonitor_TaskInitialized(TASK_MONITOR_TasksEnum task);
uint8_t TaskMonitor_CheckTasksInit(void);
void TaskMonitor_ImmediatReset(TASK_MONITOR_resetEnum ResetType);
void vApplicationLowStackCheck(TASK_MONITOR_TasksEnum taskNumber);
void vApplicationLowTotalHeapCheck(void);
void TaskMonitor_TriggerDelayedReset(const uint32_t u32WaitMs);

#ifdef DEBUG_ENABLED
void vRamTest_StackFill(void);
void vRamTest_StackUsageCheck(void);
#endif

#endif    /* TASKMONITOR_H */
/* **************************************************************************************** */
