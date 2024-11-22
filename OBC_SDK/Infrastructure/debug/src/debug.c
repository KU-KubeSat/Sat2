/*!
********************************************************************************************
* @file debug.c
* @brief Common functions used for debugging and measurement sessions
********************************************************************************************
* @author            ipetrov
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

#include "debug.h"
#if (ENABLE_DEBUG_LIB == 1)
/*
********************************************************************************************
* INCLUDES
********************************************************************************************
*/
#include <stdio.h>
#include <stdarg.h>
#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"
#include "MCU_Init.h"

/*
********************************************************************************************
* INTERNAL DEFINES
********************************************************************************************
*/
/* No Internal defines */

/*
********************************************************************************************
* INTERNAL TYPES DEFINITION
********************************************************************************************
*/
/* No Internal types definition */

/*
********************************************************************************************
* EXTERNAL VARIABLES DEFINITION
********************************************************************************************
*/
/* No External variables definition */

/**
* @brief Variable description
*/


/*
********************************************************************************************
* INTERNAL (STATIC) VARIABLES DEFINITION/DECLARATION 
********************************************************************************************
*/
/* No Internal variables definition/declaration */

/*
********************************************************************************************
* INTERNAL (STATIC) ROUTINES DECLARATION
********************************************************************************************
*/
/* No Internal routines declaration */

/*
********************************************************************************************
* EXTERNAL (NONE STATIC) ROUTINES DEFINITION
********************************************************************************************
*/

/*!
********************************************************************************************
* @brief Performs initialization of the test pins clock and state
********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
********************************************************************************************
*/
void _DBG_initTestPins(void)
{
    _DBG_setTestPin(eTEST_OUT_0, 0);
    _DBG_setTestPin(eTEST_OUT_1, 0);
}

/*!
********************************************************************************************
* @brief Updates the state of a test pin as requested
********************************************************************************************
* @param[input]      pid - test pin logical identifier
* @param[input]      state - requested pin state to set
* @param[output]     none
* @return            none
* @note              none
********************************************************************************************
*/
void _DBG_setTestPin(tPinId pid, uint8_t state)
{
    switch (pid)
    {
        case eTEST_OUT_0:
            HAL_GPIO_WritePin(OBC_OUT1_GPIO_Port, OBC_OUT1_Pin, (state == 0) ? (GPIO_PIN_RESET) : (GPIO_PIN_SET));
        break;

        case eTEST_OUT_1:
            HAL_GPIO_WritePin(OBC_OUT3_GPIO_Port, OBC_OUT3_Pin, (state == 0) ? (GPIO_PIN_RESET) : (GPIO_PIN_SET));
        break;

        default:
        break;
    }
}

/*!
********************************************************************************************
* @brief Reads the current state of a test pin and toggles it
********************************************************************************************
* @param[input]      pid - test pin logical identifier
* @param[output]     none
* @return            none
* @note              none
********************************************************************************************
*/
void _DBG_toggleTestPin(tPinId pid)
{
    GPIO_PinState pinState;

    switch (pid)
    {
        case eTEST_OUT_0:
            pinState = HAL_GPIO_ReadPin(OBC_OUT1_GPIO_Port, OBC_OUT1_Pin);

            HAL_GPIO_WritePin(OBC_OUT1_GPIO_Port, OBC_OUT1_Pin, (pinState == GPIO_PIN_RESET) ? (GPIO_PIN_SET) : (GPIO_PIN_RESET));
        break;

        case eTEST_OUT_1:
            pinState = HAL_GPIO_ReadPin(OBC_OUT5_GPIO_Port, OBC_OUT5_Pin);

            HAL_GPIO_WritePin(OBC_OUT5_GPIO_Port, OBC_OUT5_Pin, (pinState == GPIO_PIN_RESET) ? (GPIO_PIN_SET) : (GPIO_PIN_RESET));
        break;

        default:
        break;
    }
}

/*!
********************************************************************************************
* @brief Prints a formatted string to the ARM ITM port 0 via a SWV-enabled debugger
********************************************************************************************
* @param[input]      formatStr - printf-like format string
* @param[input]      ... - variable-argument list corresponding to formatStr parameters
* @param[output]     none
* @return            none
* @note              ITM port 0 has to be enabled before this function would output anything
*					 in the debugger trace console (your IDE has to provide support for this)
********************************************************************************************
*/
void _DBG_printf(const char *formatStr, ...)
{
    va_list valist;
    char buff[DBG_PRINTF_BUF_SIZE] = { '\0' };
    uint8_t i;
    uint8_t chCount = (uint8_t) 0;

    va_start(valist, formatStr);

    chCount = vsnprintf(buff, sizeof(buff), formatStr, valist);

    if ((chCount > 0) && (chCount < sizeof(buff)))
        for (i = 0; i < chCount; i++)
            ITM_SendChar(buff[i]);

    va_end(valist);
}

/*
********************************************************************************************
* INTERNAL (STATIC) ROUTINES DEFINITION
********************************************************************************************
*/
/* No Internal routines definition */

/* ******************************************************************************************* */

#endif  // #if (ENABLE_DEBUG_LIB == 1)
