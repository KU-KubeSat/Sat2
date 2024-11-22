/*!
********************************************************************************************
* @file AppTasks.c
* @brief Implementation of common task primitives
*********************************************************************************************
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
* @revision{         1.0.0  , 2018.07.04, author Vassil Milev, Initial revision }
* @revision{         1.0.1  , 2020.01.16, author Georgi Georgiev, Moved everything, except StartAppTask() to DefTasks.c }
* @endhistory
*********************************************************************************************
*/

/*
*********************************************************************************************
* INCLUDES
*********************************************************************************************
*/
#include "AppTasks.h"
#include "TaskMonitor.h"
#include "main.h"
#include "cmsis_os.h"

#if (ENABLE_CAMERA_OV5640 == 1)
#include "camera.h"
#endif

/*
*********************************************************************************************
* INTERNAL DEFINES
*********************************************************************************************
*/
#define APP_LED_ON_TIME             (5)                                /* given time in ms */
#define APP_TASK_CALL_PERIOD        (1000-APP_LED_ON_TIME)             /* given time in ms */




/*
*********************************************************************************************
* INTERNAL TYPES DEFINITION
*********************************************************************************************
*/

/*
*********************************************************************************************
* EXTERNAL VARIABLES DEFINITION
*********************************************************************************************
*/

/*
*********************************************************************************************
* INTERNAL (STATIC) VARIABLES DEFINITION/DECLARATION 
*********************************************************************************************
*/
static osThreadId_t StartAppTask_TaskHandle;
static const osThreadAttr_t StartAppTask_attributes = {
  .name = "StartAppTask",
  .priority = (osPriority_t) osPriorityNormal,
#if (ENABLE_CAMERA_OV5640 == 1)
  .stack_size = 128 * 8
#else
  .stack_size = 128 * 4
#endif
};


/*
*********************************************************************************************
* INTERNAL (STATIC) ROUTINES DECLARATION
*********************************************************************************************
*/
static void StartAppTask(void * argument);

/*
*********************************************************************************************
* EXTERNAL (NONE STATIC) ROUTINES DEFINITION
*********************************************************************************************
*/
void Uptime(void);

void AppTask_Init(void)
{
    StartAppTask_TaskHandle = osThreadNew(StartAppTask, NULL, &StartAppTask_attributes);
    configASSERT( StartAppTask_TaskHandle );
}

/*!
*********************************************************************************************
* @brief That is a task used as example to configure all sensors and actuators and blinks the green LED.
* That can be changed freely as needed depending on the project.
*********************************************************************************************
* @param[input]      argument - not used
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
static void StartAppTask(void * argument)
{
#if (ENABLE_CAMERA_OV5640 == 1)
      InitCameraModule();
#endif
 
      TaskMonitor_TaskInitialized(TASK_MONITOR_APP_TASK);   /* The task is initialized and is ready */

      for( ; ; )
      {
          TaskMonitor_IamAlive(TASK_MONITOR_APP_TASK); /* Prevent from WatchDog reset */

          /* blink the Green LED for 50ms to indicate the OBC is running */
          GREEN_LED_ON();
          osDelay(APP_LED_ON_TIME);
          GREEN_LED_OFF();

          Uptime();

#if (ENABLE_CAMERA_OV5640 == 1)
          AppProcessCameraData();
#endif
 
          osDelay(APP_TASK_CALL_PERIOD);    /* Give processing time for the other tasks */

          vApplicationLowStackCheck(TASK_MONITOR_APP_TASK);
      }
}


/*
*********************************************************************************************
* INTERNAL (STATIC) ROUTINES DEFINITION
*********************************************************************************************
*/

/********************************************************************************************* */
