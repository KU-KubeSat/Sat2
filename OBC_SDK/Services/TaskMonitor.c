/*!
********************************************************************************************
* @file TaskMonitor.c
* @brief Monitors if all task are running.
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

/*
*********************************************************************************************
* INCLUDES
*********************************************************************************************
*/
#include "GlobalConfig.h"
#include "TaskMonitor.h"          /* Include own header file */
#include "PwrMng.h"
#include "es_exeh.h"
#include "cmsis_os.h"


/*
*********************************************************************************************
* INTERNAL DEFINES
*********************************************************************************************
*/
/* The Window of the "Window Watchdog" is set to minimum time 21ms and maximum 83ms */
/* It is chosen 35 ms so it can fit two two time in the window, to be bigger the the minimum time, and more then twice smaller then the maximum time */
#define TASK_MONITOR_REFRESH_WDG_PERIOD  (100)                                          /* Period in ms at which the watchdog timer will be refreshed: see hwwdg.Init.Window 7ms min, 10 is with 50% reserve */

//To set that value keep in mind for constants: SD_TIMEOUT, SDMMC_MAXERASETIMEOUT, SDMMC_CMDTIMEOUT and so on
#define TASK_MONITOR_TASK_CHECK_PERIOD   ((4*60*1000)/TASK_MONITOR_REFRESH_WDG_PERIOD)  /* Period in ms at which all task will be check if they are alive */

#define TASK_MONITOR_LOW_MEMORY_STACK_PER_TASK   (50)
#define TASK_MONITOR_LOW_MEMORY_TOTAL_HEAP       (1024)

#define EXEH_CURRENT_MODULE_ID     (eEXEHModuleID_TASK_MONITOR)
/*
*********************************************************************************************
* INTERNAL TYPES DEFINITION
*********************************************************************************************
*/
// Used instead of a bool to increase the Hamming distance between the Inactive and Active state
// Therefore unexpected bit-flips could not easily go from Inactive to Active state and trigger
// a reset
typedef enum
{
    ECOUNTDOWN_INACTIVE = 0,
    ECOUNTDOWN_ACTIVE = 0xFF
} eResetCountdownActivationFlags_t;

typedef struct
{
    uint32_t u32MsUntilReset;
    eResetCountdownActivationFlags_t eDelayedResetCountdownActive;
} sDelayedResetCtx_t;

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
static uint32_t     TaskMonitor_timer;
static uint8_t      TaskMonitor_RunCounter[TASK_MONITOR_TASKS_NUMBER];
static uint8_t      TaskMonitor_InitComplete[TASK_MONITOR_TASKS_NUMBER];
static uint16_t     Task_UnusedStackSize[TASK_MONITOR_TASKS_NUMBER];
static sDelayedResetCtx_t sDelayedResetData =
{
    .u32MsUntilReset = 0U,
    .eDelayedResetCountdownActive = ECOUNTDOWN_INACTIVE
};

static osThreadId_t TaskMonitor_TaskHandle;
static const osThreadAttr_t TaskMonitor_attributes =
{
    .name = "TaskMonitor_Task",
    .priority = (osPriority_t)osPriorityRealtime,
    .stack_size = 640
};

/*
*********************************************************************************************
* INTERNAL (STATIC) ROUTINES DECLARATION
*********************************************************************************************
*/
static uint8_t TaskMonitor_CheckTasks(void);
static void fs_vManageDelayedReset(const uint32_t u32MsSinceLastCall);
static inline const char* const fs_pcGetResetTypeDesc(const TASK_MONITOR_resetEnum eResetType);

/*
*********************************************************************************************
* EXTERNAL (NONE STATIC) ROUTINES DEFINITION
*********************************************************************************************
*/
/*!
*********************************************************************************************
* @brief Init routine for the TaskMonitor component
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void TaskMonitor_Init(void)
{
    sDelayedResetData.eDelayedResetCountdownActive = ECOUNTDOWN_INACTIVE;
    sDelayedResetData.u32MsUntilReset = 0U;

    for (uint8_t i = 0; i < TASK_MONITOR_TASKS_NUMBER; i++)
    {
        TaskMonitor_RunCounter[i] = 0;
        TaskMonitor_InitComplete[i] = 0;
        Task_UnusedStackSize[i] = -1;
    }
    Task_UnusedStackSize[TASK_MONITOR_TASKS_NUMBER] = -1;

    TaskMonitor_timer = 0;

#if (defined(DEBUG_ENABLED) && (ENABLE_WATCHDOG_AND_TASK_HANDLER == 1)) || !defined(DEBUG_ENABLED)
    TaskMonitor_TaskHandle = osThreadNew(TaskMonitor_Task, NULL, &TaskMonitor_attributes);
    configASSERT(TaskMonitor_TaskHandle);
#endif

    EXEH_INIT_MODULE_FILENAME();
}

/*!
*********************************************************************************************
* @brief Task routine of the Task monitor
*********************************************************************************************
* @param[input]      argument - not used
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void TaskMonitor_Task(void *argument)
{
    TaskMonitor_TaskInitialized(TASK_MONITOR_TASK);

    while (1)
    {
        osDelay(TASK_MONITOR_REFRESH_WDG_PERIOD);
        fs_vManageDelayedReset(TASK_MONITOR_REFRESH_WDG_PERIOD);

        TaskMonitor_timer++;

        if (TASK_MONITOR_TASK_CHECK_PERIOD <= TaskMonitor_timer)
        {
            uint8_t u8LastTaskChecked = TaskMonitor_CheckTasks();

            if (u8LastTaskChecked == TASK_MONITOR_TASKS_NUMBER)
            {
                HAL_IWDG_Refresh(&hiwdg);
                TaskMonitor_timer = 0;
            }
            else
            {
                const char* const resetTypeDescr = fs_pcGetResetTypeDesc(TASK_MONITOR_TASK_TIMEOUT);

                fprintf(COMM, "%s (tid: %d)\r\n", resetTypeDescr, u8LastTaskChecked);
                fprintf(SYSCON, "%s (tid: %d)\r\n", resetTypeDescr, u8LastTaskChecked);
                TaskMonitor_ImmediatReset(TASK_MONITOR_TASK_TIMEOUT);
            }
        }
        else
        {
            HAL_IWDG_Refresh(&hiwdg);
        }

        TaskMonitor_IamAlive(TASK_MONITOR_TASK);
        vApplicationLowStackCheck(TASK_MONITOR_TASK);
#ifdef DEBUG_ENABLED
        vApplicationLowTotalHeapCheck();
#endif
    }
}

/*!
*********************************************************************************************
* @brief Calling that function from all task listed in TASK_MONITOR_TasksEnum will prevent from watchdog reset
*********************************************************************************************
* @param[input]      task - the number of the task according to the TASK_MONITOR_TasksEnum
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void TaskMonitor_IamAlive(TASK_MONITOR_TasksEnum task)
{
    if (task < TASK_MONITOR_TASKS_NUMBER)
    {
        if (TaskMonitor_RunCounter[task] < 0xFE) /* Prevent from rolling over */
        {
            /* Set the task as alive (run one more time) */
            TaskMonitor_RunCounter[task]++;
        }
    }
}

/*!
*********************************************************************************************
* @brief Set flag that the current task is initialised and ready
*********************************************************************************************
* @param[input]      task - the number of the task according to the TASK_MONITOR_TasksEnum
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void TaskMonitor_TaskInitialized(TASK_MONITOR_TasksEnum task)
{
    if (task < TASK_MONITOR_TASKS_NUMBER)
    {
        TaskMonitor_InitComplete[task] = 1;
    }
}

/*!
*********************************************************************************************
* @brief Check if all tasks are initialised
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            1 - All task are ready
*                    0 - At least one task is not ready
* @note              none
*********************************************************************************************
*/
uint8_t TaskMonitor_CheckTasksInit(void)
{
    uint8_t i;
    uint8_t retVal = 1; /* assume all task has passed at least once */

    for (i = 0; i < TASK_MONITOR_TASKS_NUMBER; i++)
    {
        if (TaskMonitor_InitComplete[i] == 0)
        {
            /* One task is not alive */
            retVal = 0;
            break;
        }
    }

    return retVal;
}

/*!
*********************************************************************************************
* @brief Calling that function clauses immediate reset
*********************************************************************************************
* @param[input]      ResetType:
*                    TASK_MONITOR_SW_RESET - Software reset
*                    TASK_MONITOR_WD_RESET - Watchdog reset
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void TaskMonitor_ImmediatReset(TASK_MONITOR_resetEnum ResetType)
{
    if (ResetType != TASK_MONITOR_TASK_TIMEOUT)
    {
        const char* const resetTypeDescr = fs_pcGetResetTypeDesc(ResetType);

        fprintf(COMM, "%s\r\n", resetTypeDescr);
        fprintf(SYSCON, "%s\r\n", resetTypeDescr);
    }

#ifdef DEBUG_ENABLED
    if (ResetType == TASK_MONITOR_FAULT_RESET)
    {
        __BKPT();
    }
#endif

    NVIC_SystemReset();
}

/*!
*********************************************************************************************
* @brief This function is called to check the unused stack for the task it is called from
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void vApplicationLowStackCheck(TASK_MONITOR_TasksEnum taskNumber)
{
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL);

    if (Task_UnusedStackSize[taskNumber] > (uint16_t)uxHighWaterMark)
    {
        Task_UnusedStackSize[taskNumber] = (uint16_t)uxHighWaterMark;
    }

    if (Task_UnusedStackSize[taskNumber] < 2 * TASK_MONITOR_LOW_MEMORY_STACK_PER_TASK)
        EXEH_HANDLE(eEXEHSeverity_Warning, eEXEH_TASK_MONITOR_TASK_STACK_WARN | ((uint32_t ) taskNumber << 24U));

    if (Task_UnusedStackSize[taskNumber] < TASK_MONITOR_LOW_MEMORY_STACK_PER_TASK) //Minimum tack size over the needed
    {
        Error_Handler();
    }
}

void vApplicationLowTotalHeapCheck(void)
{
    size_t totalFreeHeap = xPortGetFreeHeapSize();

    if (totalFreeHeap < TASK_MONITOR_LOW_MEMORY_TOTAL_HEAP) //Minimum tack size over the needed
    {
        Error_Handler();
    }
}

/*!
*********************************************************************************************
* @brief The function triggers a CPU reset after a specified number of milliseconds
*********************************************************************************************
* @param[input]      u32WaitMs: milliseconds to wait before performing a reset
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void TaskMonitor_TriggerDelayedReset(const uint32_t u32WaitMs)
{
    // order is important since this information is used in the TaskMonitor_Task() and
    // the bDelayedResetCountdownActive flag could be checked before setting the u32MsUntilReset
    // value causing an immediate reset - no protection is needed here since bDelayedResetCountdownActive
    // is read atomically
    sDelayedResetData.u32MsUntilReset = u32WaitMs;
    sDelayedResetData.eDelayedResetCountdownActive = ECOUNTDOWN_ACTIVE;
}

/*
*********************************************************************************************
* INTERNAL (STATIC) ROUTINES DEFINITION
*********************************************************************************************
*/

/*!
*********************************************************************************************
* @brief Check if all task are still running, but not stuck
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            1 - All tasks are alive
*                    0 - At least one frozen task has been found
* @note              none
*********************************************************************************************
*/
static uint8_t TaskMonitor_CheckTasks(void)
{
    uint8_t i;
    uint8_t u8LastTaskChecked = TASK_MONITOR_TASKS_NUMBER; /* assume all task has passed at least once */

    for (i = 0; i < TASK_MONITOR_TASKS_NUMBER; i++)
    {
        if (TaskMonitor_RunCounter[i] == 0)
        {
            // report task which was not alive
            u8LastTaskChecked = i;
            break;
        }
        else
        {
            TaskMonitor_RunCounter[i] = 0;
        }
    }

    return u8LastTaskChecked;
}

/*!
*********************************************************************************************
* @brief Manage the delayed reset mechanism
*********************************************************************************************
* @param[input]      u32MsSinceLastCall: milliseconds passed since last call of this function
*                                        in the main task
* @param[output]     none
* @return            N/A
* @note              none
*********************************************************************************************
*/
static void fs_vManageDelayedReset(const uint32_t u32MsSinceLastCall)
{
    if (sDelayedResetData.eDelayedResetCountdownActive == ECOUNTDOWN_ACTIVE)
    {
        if (sDelayedResetData.u32MsUntilReset >= u32MsSinceLastCall)
        {
            sDelayedResetData.u32MsUntilReset -= u32MsSinceLastCall;
        }
        else
            TaskMonitor_ImmediatReset(TASK_MONITOR_USER_DELAYED_RESET);
    }
}
/* **************************************************************************************** */

static inline const char* const fs_pcGetResetTypeDesc(const TASK_MONITOR_resetEnum eResetType)
{
    static const char* cResetTypesDescription[TASK_MONITOR_RESET_NUMBER] =
    {
        "SW_RESET",
        "FAULT_RESET",
        "USER_DELAYED_RESET",
        "TASK_TIMEOUT"
    };

    if (eResetType < TASK_MONITOR_RESET_NUMBER)
        return cResetTypesDescription[(uint8_t)eResetType];
    else
        return "UNKNOWN";
}

#ifdef DEBUG_ENABLED
//VMI_TODO_ONEDAY - move this in the RAM check component
#define STACK_PATERN 0x52
#define HEAP_PATERN  0x51
#define STACK_SIZE 0x400 //_Min_Stack_Size
#define HEAP_SIZE  0x100 //_Min_Heap_Size
#define END_OF_RAM_POSITION 0x20080000
#define START_OF_STACK_POSITION (END_OF_RAM_POSITION-STACK_SIZE)
#define START_OF_HEAP_POSITION  (END_OF_RAM_POSITION-STACK_SIZE-HEAP_SIZE)
#define MIN_FREE_SIZE_STACK 50
#define MIN_FREE_SIZE_HEAP  50

#include "string.h"

volatile uint16_t freeStackSize = 0;
volatile uint16_t freeHeapSize = 0;

void vRamTest_StackFill(void)
{
    memset((char*)(END_OF_RAM_POSITION - STACK_SIZE), STACK_PATERN, STACK_SIZE - (16 * 4)); //do not overwrite last 10 pointer (with size of uint32_t)
    memset((char*)(END_OF_RAM_POSITION - STACK_SIZE - HEAP_SIZE), HEAP_PATERN, HEAP_SIZE - (8 * 4));
}

void vRamTest_StackUsageCheck(void)
{
    char* i;

    for (i = (char*)(START_OF_STACK_POSITION); i < (char*)((START_OF_STACK_POSITION) + STACK_SIZE); i++)
    {
        if (*i != STACK_PATERN)
        {
            break;
        }
    }
    freeStackSize = (uint32_t)i - (START_OF_STACK_POSITION);

    if (freeStackSize < MIN_FREE_SIZE_STACK)
    {
        Error_Handler();
    }

    for (i = (char*)(START_OF_HEAP_POSITION); i < (char*)((START_OF_HEAP_POSITION) + HEAP_SIZE); i++)
    {
        if (*i != HEAP_PATERN)
        {
            break;
        }
    }
    freeHeapSize = (uint32_t)i - (START_OF_HEAP_POSITION);

    if (freeHeapSize < MIN_FREE_SIZE_HEAP)
    {
        Error_Handler();
    }
}
#endif
