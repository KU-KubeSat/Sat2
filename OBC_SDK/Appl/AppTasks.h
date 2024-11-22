/*!
********************************************************************************************
* @file AppTasks.h
* @brief Header of AppTasks.c
********************************************************************************************
* @author            Vassil MIlev
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
* @endhistory
********************************************************************************************
*/
#ifndef APPTASKS_H
#define APPTASKS_H

/*
*********************************************************************************************
* INCLUDES
*********************************************************************************************
*/
#include <stdint.h>

/*
*********************************************************************************************
* EXTERNAL DEFINES
*********************************************************************************************
*/
#define SERVICE_TASK_PERIOD  (1000)

/*
*********************************************************************************************
* EXTERNAL TYPES DECLARATIONS
*********************************************************************************************
*/

/*
*********************************************************************************************
* EXTERNAL VARIABLES DECLARATIONS
*********************************************************************************************
*/
extern uint8_t  up_sec, up_min, up_hrs;
extern uint32_t up_day;
extern uint32_t obcUptimeInSeconds;

/*
*********************************************************************************************
* EXTERNAL ROUTINES DECLARATIONS 
*********************************************************************************************
*/
void ServicesTask_Init(void);
void AppTask_Init(void);
uint32_t GetObcUptimeSeconds(void);
uint32_t ServicesTask_RTC_Seconds(void);
uint32_t ServicesTask_UptimeSeconds(void);

#endif    /* APPTASKS_H */
/********************************************************************************************* */
