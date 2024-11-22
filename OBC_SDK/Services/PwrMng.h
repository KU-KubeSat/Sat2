/*!
 *********************************************************************************************
 * @file PwrMng.h
 * @brief Header of PwrMng.c
 *********************************************************************************************
 * @author            Vassil Milev
 * @version           1.0.0
 * @date              2020.04.21
 *
 * @copyright         (C) Copyright Endurosat
 *
 *                    Contents and presentations are protected world-wide.
 *                    Any kind of using, copying etc. is prohibited without prior permission.
 *                    All rights - incl. industrial property rights - are reserved.
 *
 * @history
 * @revision{         1.0.0  , 2020.04.21, author Vassil Milev, Initial revision }
 * @endhistory
 *********************************************************************************************
 */
#ifndef PWRMNG_H
#define PWRMNG_H

/*
 *********************************************************************************************
 * INCLUDES
 *********************************************************************************************
 */
#include <main.h>

#if defined(ENABLE_PWRMNG_DEBUG_TRACE)
  #define PWRM_DBG_PRINT(formatStr, ...)   DBG_SYSCON(formatStr OPT_VA_ARGS(__VA_ARGS__));
#else
  #define PWRM_DBG_PRINT(fmtString, ...)   asm("nop");
#endif


/*
 *********************************************************************************************
 * EXTERNAL DEFINES
 *********************************************************************************************
 */
#define PWRMNG_SLEEP_DEFAULT_PERIOD    (30)    //Seconds - default period
#define PWRMNG_STAY_AWAKE_PERIOD       (60)    // Default period // in ms after [X* OPMODES_TASK_PERIOD]

/*
 *********************************************************************************************
 * EXTERNAL TYPES DECLARATIONS
 *********************************************************************************************
 */
typedef enum
{
    PWRMNG_DYNAMIC_RUN_MODE,
    PWRMNG_STOP_MODE,
    PWRMNG_MODE_NUMBER
} PwrMng_modes_enum;

typedef enum
{
    PWRMNG_ISR_NONE,
    PWRMNG_ISR_PACKET_EXTI,
    PWRMNG_ISR_RTC,
    PWRMNG_ISR_NUMBER
} PwrMng_isrs_enum;


/*
 *********************************************************************************************
 * EXTERNAL VARIABLES DECLARATIONS
 *********************************************************************************************
 */

/*
 *********************************************************************************************
 * EXTERNAL ROUTINES DECLARATIONS
 *********************************************************************************************
 */
void PwrMng_Init(void);
void PwrMng_SetMode(PwrMng_modes_enum mode);
PwrMng_modes_enum PwrMng_GetMode(void);
uint8_t PwrMng_GetWakeUpExterDevice(void);
void PwrMng_ClearWakeUpExterDevice(void);
void PwrMng_SetWakeUpExterDevice(PwrMng_isrs_enum type);
uint8_t PwrMng_SetSleepPeriod(uint16_t period);
uint16_t PwrMng_GetSleepPeriod(void);
uint16_t PwrMng_GetLastLowPowerPeriod(void);
uint8_t PwrMng_SetStayAwakePeriod(uint16_t period);
uint16_t PwrMng_GetStayAwakePeriod(void);

#endif    /* PWRMNG_H */
/* ******************************************************************************************* */
