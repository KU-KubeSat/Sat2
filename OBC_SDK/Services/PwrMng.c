/*!
*********************************************************************************************
* @file PwdMng.c
* @brief Manages switching between different power saving modes
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

/*
*********************************************************************************************
* INCLUDES
*********************************************************************************************
*/
#include "PwrMng.h"

#if (ENABLE_POWER_MANAGER == 1)

#include "cmsis_os.h"
#include "ESTTC.h"
#include "EEPROM_Emul.h"
#include "stm32f7xx_ll_exti.h"
#include "stm32f7xx_ll_rtc.h"
#include "es_exeh.h"
#include "ExtFram.h"
#include "panels.h"
#include "DAT_inputs.h"

/*
*********************************************************************************************
* INTERNAL DEFINES
*********************************************************************************************
*/
#define SLEEP_MIN_PERIOD           (2)        //Seconds - minimum allowed period
#define STAY_MIN_PERIOD            (2)        //Seconds - minimum allowed period

/*
*********************************************************************************************
* INTERNAL TYPES DEFINITION
*********************************************************************************************
*/
/* No Internal types definition */

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
static PwrMng_modes_enum PwrMng_powerMode;          // the current mode that is in the moment
static PwrMng_isrs_enum  PwrMng_wakeFromISR;        // Flag used to mark a wake up from external device
static uint16_t          PwrMng_sleepPeriod;        // Period to sleep. After the period the CPU will wake up.
static uint16_t          PwrMng_stayAwakePeriod;    // Period to sleep. After the period the CPU will wake up. // in ms after [X* OPMODES_TASK_PERIOD]
static uint16_t          PwrMng_lastLowPowerPeriod; // Time spent in low power state

/*
*********************************************************************************************
* INTERNAL (STATIC) ROUTINES DECLARATION
*********************************************************************************************
*/
static void PwrMng_ExitFromStopMode(void);
static void PwrMng_EnterDynamicRunMode(void);
static void EnterStopMode(void);
static void PwrMng_DeinitAll(void);

/*
*********************************************************************************************
* EXTERNAL (NONE STATIC) ROUTINES DEFINITION
*********************************************************************************************
*/
/*!
*********************************************************************************************
* @brief Init routine for the PwrMng component
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void PwrMng_Init(void)
{
    PwrMng_powerMode = PWRMNG_DYNAMIC_RUN_MODE;
    PwrMng_lastLowPowerPeriod = 0;
    PwrMng_sleepPeriod = PWRMNG_SLEEP_DEFAULT_PERIOD;
    PwrMng_stayAwakePeriod = PWRMNG_STAY_AWAKE_PERIOD;
}

void PwrMng_SetMode(PwrMng_modes_enum mode)
{
    switch (mode)
    {
        case PWRMNG_STOP_MODE:
        {
            EnterStopMode();//Stay here while sleeping

            //After wake up: reinit
            PwrMng_ExitFromStopMode();
        } break;

        case PWRMNG_DYNAMIC_RUN_MODE:
        default:
        {
            PwrMng_EnterDynamicRunMode();

            HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
            PwrMng_powerMode = PWRMNG_DYNAMIC_RUN_MODE;
        } break;
    }
}

PwrMng_modes_enum PwrMng_GetMode(void)
{
    return PwrMng_powerMode;
}

/*!
*********************************************************************************************
* @brief Get the flag for wake up from external device
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
PwrMng_isrs_enum PwrMng_GetWakeUpExterDevice(void)
{
    return PwrMng_wakeFromISR;
}

/*!
*********************************************************************************************
* @brief Clear the flag for wake up from external device
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void PwrMng_ClearWakeUpExterDevice(void)
{
    PwrMng_wakeFromISR = PWRMNG_ISR_NONE;
}

/*!
*********************************************************************************************
* @brief Set the flag for wake up from external device
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void PwrMng_SetWakeUpExterDevice(PwrMng_isrs_enum type)
{
    if (type < PWRMNG_ISR_NUMBER)
    {
        PwrMng_wakeFromISR = type;
    }
}

/*!
*********************************************************************************************
* @brief Set the sleep period
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
uint8_t PwrMng_SetSleepPeriod(uint16_t period)
{
    if (period >= SLEEP_MIN_PERIOD)
    {
        PwrMng_sleepPeriod = period;
        return 1;
    }
    return 0;
}

/*!
*********************************************************************************************
* @brief Get the sleep period
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
uint16_t PwrMng_GetSleepPeriod(void)
{
    return PwrMng_sleepPeriod;
}

/*!
*********************************************************************************************
* @brief Get the sleep period
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
uint16_t PwrMng_GetLastLowPowerPeriod(void)
{
    return PwrMng_lastLowPowerPeriod;
}



/*!
*********************************************************************************************
* @brief Set period to stay awake on wake up from package
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
uint8_t PwrMng_SetStayAwakePeriod(uint16_t period)
{
    if (period >= STAY_MIN_PERIOD)
    {
        PwrMng_stayAwakePeriod = period; //convert to milliseconds
        return 1;
    }
    return 0;
}

/*!
*********************************************************************************************
* @brief Get period to stay awake on wake up from package
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
uint16_t PwrMng_GetStayAwakePeriod(void)
{
    return PwrMng_stayAwakePeriod;
}

/*
*********************************************************************************************
* INTERNAL (STATIC) ROUTINES DEFINITION
*********************************************************************************************
*/
static void PwrMng_ExitFromStopMode(void)
{
    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

    HAL_Init();
    SystemClock_IdleConfig();
    MX_GPIO_Init();

    HAL_ResumeTick();

    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

    FRAM_Wake();

    ESTTC_Init();

    Stop_Magnetorquers();
    MagnTrq_Release(true);
	
#if defined(ENABLE_PWRMNG_DEBUG_TRACE)
    RtcTm_t rtc = ReadHandler_RTCTime();
    PWRM_DBG_PRINT("\r\n|%d| [%02d:%02d:%02d] \t\t----------------> ExitFromStopMode() <----------------\r\n", OBC_PRIORITY, rtc.hours, rtc.minutes, rtc.seconds);
#endif
}


static void PwrMng_EnterDynamicRunMode(void)
{
    // VMI_TODO_6UPLATFORM SD cards does not work with D cache - only with I-Cache
    /* Enable I-Cache */
    //SCB_EnableICache();

    /* Enable D-Cache */
    //SCB_EnableDCache();

    HAL_Init();
    SystemClock_IdleConfig();
    MX_GPIO_Init();

    HAL_ResumeTick();

    /* Exit Ethernet Phy from low power mode */
    //ETH_PhyExitFromPowerDownMode();

    ESTTC_Init();
}


static void EnterStopMode(void)
{
    HAL_StatusTypeDef halStat;
    GPIO_InitTypeDef GPIO_InitStruct = {0};

#if defined(ENABLE_PWRMNG_DEBUG_TRACE)
    RtcTm_t rtc = ReadHandler_RTCTime();
    PWRM_DBG_PRINT("\r\n|%d| [%02d:%02d:%02d] \t\t----------------> EnterStopMode() <----------------\r\n", OBC_PRIORITY, rtc.hours, rtc.minutes, rtc.seconds);
#endif  // #if defined(ENABLE_PWRMNG_DEBUG_TRACE)

    for (uint8_t i = 0; i < 3; i++)
    {
        halStat = HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, PwrMng_sleepPeriod - 1, RTC_WAKEUPCLOCK_CK_SPRE_16BITS); // if time is 0 it will be 1 second

        if (HAL_OK == halStat)
        {
            break;
        }
        else
        {
            osDelay(1);
        }
    }

    FRAM_EnSleep();
    PwrMng_DeinitAll();

    ////////////////////////////////////////////
    //////    Set up wake up sources     ///////
    ////////////////////////////////////////////

    // Enable Wake up on packet from USART1, USART3, UART8, I2C1
    GPIO_InitStruct.Pin = SYS_WAKE1_Pin;  /*Configure GPIO pin : PE0 */
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SYS_WAKE1_GPIO_Port, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
    HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    // Enable Wake up on packet from SYSTEM UART
    GPIO_InitStruct.Pin = U5_PROTO_USB_SYSTEM_RX_Pin;  /*Configure GPIO pin : PB5 */
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(U5_PROTO_USB_SYSTEM_RX_GPIO_Port, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
    HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    //HAL_PWR_DisableWakeUpPin( PWR_WAKEUP_PIN1 );
    //__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

    /* Enable WKUP pin */
    //HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);  //LL_PWR_EnableWakeUpPin(LL_PWR_WAKEUP_PIN1);

    // Wake up on RCC Wake up pin
    //HAL_NVIC_SetPriority(RCC_IRQn, 2, 0);
    //HAL_NVIC_EnableIRQ(RCC_IRQn);

    // Wake up on RTC timer on period = PwrMng_SleepPeriod [sec]
    __HAL_RTC_WAKEUPTIMER_EXTI_ENABLE_IT();
    HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

    // Time-stamp
    MX_RTC_ReloadShadowRegs();
    RTC_TimeTypeDef sTimeBeforeSleep = {0};
    HAL_RTC_GetTime(&hrtc, &sTimeBeforeSleep, RTC_FORMAT_BIN);

    HAL_SuspendTick();

    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_ALL);

    if (HAL_OK == halStat)
    {
        PwrMng_ClearWakeUpExterDevice();
        PwrMng_powerMode = PWRMNG_STOP_MODE;
        HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI); //Enter Stop mode here
    }
    else
    {
        PwrMng_powerMode = PWRMNG_DYNAMIC_RUN_MODE;
        EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_PWR_MNG, eEXEH_PWR_MNG_EXCEPTION_SET_WAKEUP_TIME_FAIL, __LINE__);
    }

    if (PwrMng_wakeFromISR == PWRMNG_ISR_RTC)
    {
        PwrMng_lastLowPowerPeriod = PwrMng_sleepPeriod;
    }
    else
    {
        //Time-stamp
        MX_RTC_ReloadShadowRegs();
        RTC_TimeTypeDef sTimeAfterSleep = {0};
        HAL_RTC_GetTime(&hrtc, &sTimeAfterSleep, RTC_FORMAT_BIN);

        if (sTimeBeforeSleep.Minutes > sTimeAfterSleep.Minutes)
        {
            sTimeAfterSleep.Minutes += 60;
        }

        uint16_t beforeSleepSeconds = sTimeBeforeSleep.Minutes * 60 + sTimeBeforeSleep.Seconds;

        uint16_t afterSleepSeconds = sTimeAfterSleep.Minutes * 60 + sTimeAfterSleep.Seconds;

        PwrMng_lastLowPowerPeriod = afterSleepSeconds - beforeSleepSeconds;
    }
}


static void PwrMng_DeinitAll(void)
{
    ESTTC_DeInit();

    /* Disable Ethernet Clock */
    //__HAL_RCC_ETH_CLK_DISABLE();

    // Disable max periphery, leave all unused pins as analog inputs for minimum power consumption
    MX_AllPeripheryAndGpio_DeInit();

    HAL_PWREx_EnableFlashPowerDown();
}

/* ******************************************************************************************* */
#endif
