/*!
********************************************************************************************
* @file XXX.h
* @brief Header of XXX.
********************************************************************************************
* @author            XXX
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
* @revision{         1.0.0  , 2018.07.04, author XXX, Initial revision }
* @endhistory
********************************************************************************************
*/
#ifndef ES_IF_DEBUG_H
#define ES_IF_DEBUG_H

#include <stdint.h>
#include <stdbool.h>
#include "GlobalConfig.h"

void Error_Handler(void);

#ifdef __cplusplus
extern "C" {
#endif  // #ifdef __cplusplus
/*
********************************************************************************************
* INCLUDES
********************************************************************************************
*/
/* No Includes */

/*
********************************************************************************************
* EXTERNAL DEFINES
********************************************************************************************
*/
// Please use the following macros to access debug lib functionality to be able to completely
// remove this code in your release builds
#define DBG_PRINTF_BUF_SIZE           ((uint8_t) 70)

#if (ENABLE_DEBUG_LIB == 1)
  #define DBG_initTestPins()          _DBG_initTestPins()
  #define DBG_setTestPin(pid, state)  _DBG_setTestPin(pid, state)
  #define DBG_toggleTestPin(pid)      _DBG_toggleTestPin(pid)
  #define DBG_printf(formatStr, ...)  _DBG_printf(formatStr, __VA_ARGS__)
  #define DBG_TRACE(formatStr, ...)
  #ifndef DBG_ASSERT
    #define DBG_ASSERT(condition)     { if (!((bool) (condition))) Error_Handler(); }
  #endif
  #define OPT_VA_ARGS_SYSCON(...) , ##__VA_ARGS__
  #define DBG_SYSCON(formatStr, ...)   fprintf(SYSCON, formatStr OPT_VA_ARGS_SYSCON(__VA_ARGS__));
#else
  #define DBG_initTestPins()
  #define DBG_setTestPin(pid, state)
  #define DBG_toggleTestPin(pid)
  #define DBG_printf(formatStr, ...)
  #define DBG_TRACE(formatStr, ...)
  #ifndef DBG_ASSERT
    #define DBG_ASSERT(condition)
  #endif  // #ifndef DBG_ASSERT
  #define DBG_SYSCON(formatStr, ...)
#endif  // #if (ENABLE_DEBUG_LIB == 1)

/*
********************************************************************************************
* EXTERNAL TYPES DECLARATIONS
********************************************************************************************
*/
/* No External types declarations */
typedef enum
{
    eTEST_OUT_0,
    eTEST_OUT_1
} tPinId;

/*
********************************************************************************************
* EXTERNAL VARIABLES DECLARATIONS
********************************************************************************************
*/
/* No External variables declarations */

/*
********************************************************************************************
* EXTERNAL ROUTINES DECLARATIONS 
********************************************************************************************
*/
#if (ENABLE_DEBUG_LIB == 1)
void _DBG_initTestPins(void);
void _DBG_setTestPin(tPinId pid, uint8_t state);
void _DBG_toggleTestPin(tPinId pid);
void _DBG_printf(const char *formatStr, ...);
#endif

#ifdef __cplusplus
}
#endif  // #ifdef __cplusplus

#endif    /* ES_IF_DEBUG_H */
/* ******************************************************************************************* */
