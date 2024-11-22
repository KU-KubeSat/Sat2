/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f7xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32f7xx.h"
#include "stm32f7xx_it.h"
#include "cmsis_os.h"
#include "MCU_Init.h"
#include "ESTTC.h"
#include "TaskMonitor.h"
#include "Svc_RTC.h"
#include "PwrMng.h"
#include "OEM719.h"
#include "SdMngr.h"
#include "csp_example_config.h"
#include "../../KUbeSat/TestUART.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
    /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

    /* USER CODE END NonMaskableInt_IRQn 0 */
    /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

    /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
    /* USER CODE BEGIN HardFault_IRQn 0 */
    BootData->RST_HardFault++;
    RTC_SetRTC_BKP_CRC(); /* set valid checksum for the RTC backup registers after changing the mailbox */
    /* USER CODE END HardFault_IRQn 0 */

    while (1)
    {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        TaskMonitor_ImmediatReset(TASK_MONITOR_FAULT_RESET);
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
    /* USER CODE BEGIN HardFault_IRQn 1 */

    /* USER CODE END HardFault_IRQn 1 */
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
    /* USER CODE BEGIN MemoryManagement_IRQn 0 */
    BootData->RST_MemFault++;
    RTC_SetRTC_BKP_CRC(); /* set valid checksum for the RTC backup registers after changing the mailbox */
    /* USER CODE END MemoryManagement_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
        TaskMonitor_ImmediatReset(TASK_MONITOR_FAULT_RESET);
        /* USER CODE END W1_MemoryManagement_IRQn 0 */
    }
    /* USER CODE BEGIN MemoryManagement_IRQn 1 */

    /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
    /* USER CODE BEGIN BusFault_IRQn 0 */
    BootData->RST_BusFault++;
    RTC_SetRTC_BKP_CRC(); /* set valid checksum for the RTC backup registers after changing the mailbox */
    /* USER CODE END BusFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_BusFault_IRQn 0 */
        TaskMonitor_ImmediatReset(TASK_MONITOR_FAULT_RESET);
        /* USER CODE END W1_BusFault_IRQn 0 */
    }
    /* USER CODE BEGIN BusFault_IRQn 1 */

    /* USER CODE END BusFault_IRQn 1 */
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
    /* USER CODE BEGIN UsageFault_IRQn 0 */
    BootData->RST_UsageFault++;
    RTC_SetRTC_BKP_CRC(); /* set valid checksum for the RTC backup registers after changing the mailbox */
    /* USER CODE END UsageFault_IRQn 0 */
    while (1)
    {
        /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
        TaskMonitor_ImmediatReset(TASK_MONITOR_FAULT_RESET);
        /* USER CODE END W1_UsageFault_IRQn 0 */
    }
    /* USER CODE BEGIN UsageFault_IRQn 1 */

    /* USER CODE END UsageFault_IRQn 1 */
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{
    /* USER CODE BEGIN DebugMonitor_IRQn 0 */

    /* USER CODE END DebugMonitor_IRQn 0 */
    /* USER CODE BEGIN DebugMonitor_IRQn 1 */

    /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
    /* USER CODE BEGIN SysTick_IRQn 0 */

    /* USER CODE END SysTick_IRQn 0 */
    HAL_IncTick();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
#endif /* INCLUDE_xTaskGetSchedulerState */
        xPortSysTickHandler();
    /* USER CODE BEGIN SysTick_IRQn 1 */

    /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles PVD interrupt through EXTI line 16.
 */
void PVD_IRQHandler(void)
{
    /* USER CODE BEGIN PVD_IRQn 0 */

    /* USER CODE END PVD_IRQn 0 */
    HAL_PWR_PVD_IRQHandler();
    /* USER CODE BEGIN PVD_IRQn 1 */

    /* USER CODE END PVD_IRQn 1 */
}

/**
 * @brief This function handles Flash global interrupt.
 */
void FLASH_IRQHandler(void)
{
    /* USER CODE BEGIN FLASH_IRQn 0 */

    /* USER CODE END FLASH_IRQn 0 */
    HAL_FLASH_IRQHandler();
    /* USER CODE BEGIN FLASH_IRQn 1 */

    /* USER CODE END FLASH_IRQn 1 */
}

void RTC_WKUP_IRQHandler(void)
{
    HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);
#if (ENABLE_POWER_MANAGER == 1)
    PwrMng_SetWakeUpExterDevice(PWRMNG_ISR_RTC);
#endif
}

/**
 * @brief This function handles RCC global interrupt.
 */
void RCC_IRQHandler(void)
{
    /* USER CODE BEGIN RCC_IRQn 0 */

    /* USER CODE END RCC_IRQn 0 */
    /* USER CODE BEGIN RCC_IRQn 1 */

    /* USER CODE END RCC_IRQn 1 */
}
/**
 * @brief This function handles DMA1 stream1 global interrupt.
 */
void DMA1_Stream1_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

    /* USER CODE END DMA1_Stream1_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_usart3_rx);
    /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

    /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
 * @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
 */
void ADC_IRQHandler(void)
{
    /* USER CODE BEGIN ADC_IRQn 0 */

    /* USER CODE END ADC_IRQn 0 */
    HAL_ADC_IRQHandler(&hadc1);
    /* USER CODE BEGIN ADC_IRQn 1 */

    /* USER CODE END ADC_IRQn 1 */
}

/**
 * @brief This function handles Window watchdog interrupt.
 */
void WWDG_IRQHandler(void)
{
    /* USER CODE BEGIN WWDG_IRQn 0 */
    BootData->RST_WWD++;
    RTC_SetRTC_BKP_CRC(); /* set valid checksum for the RTC backup registers after changing the mailbox */
    /* USER CODE END HardFault_IRQn 0 */

    while (1)
    {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        TaskMonitor_ImmediatReset(TASK_MONITOR_FAULT_RESET);
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
    /* USER CODE END WWDG_IRQn 0 */
    HAL_WWDG_IRQHandler(&hwwdg);
    /* USER CODE BEGIN WWDG_IRQn 1 */

    /* USER CODE END WWDG_IRQn 1 */
}

void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg)
{
    BootData->RST_WWD++;
    RTC_SetRTC_BKP_CRC(); /* set valid checksum for the RTC backup registers after changing the mailbox */
    /* USER CODE END HardFault_IRQn 0 */

    while (1)
    {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        TaskMonitor_ImmediatReset(TASK_MONITOR_FAULT_RESET);
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
}

/**
 * @brief This function handles CAN1 TX interrupts.
 */
void CAN1_TX_IRQHandler(void)
{
    /* USER CODE BEGIN CAN1_TX_IRQn 0 */

    /* USER CODE END CAN1_TX_IRQn 0 */
    HAL_CAN_IRQHandler(&hcan1);
    /* USER CODE BEGIN CAN1_TX_IRQn 1 */

    /* USER CODE END CAN1_TX_IRQn 1 */
}

/**
 * @brief This function handles CAN1 RX0 interrupts.
 */
void CAN1_RX0_IRQHandler(void)
{
    /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

    /* USER CODE END CAN1_RX0_IRQn 0 */
    HAL_CAN_IRQHandler(&hcan1);
    /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

    /* USER CODE END CAN1_RX0_IRQn 1 */
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1)
    {
#if (ENABLE_CSP_EXAMPLE == 1)
        cspex_can_rx_clbk();
#endif
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1)
    {
#if (ENABLE_CSP_EXAMPLE == 1)
        cspex_can_err_clbk();
#endif
    }
}

/**
 * @brief This function handles CAN1 RX1 interrupt.
 */
void CAN1_RX1_IRQHandler(void)
{
    /* USER CODE BEGIN CAN1_RX1_IRQn 0 */

    /* USER CODE END CAN1_RX1_IRQn 0 */
    HAL_CAN_IRQHandler(&hcan1);
    /* USER CODE BEGIN CAN1_RX1_IRQn 1 */

    /* USER CODE END CAN1_RX1_IRQn 1 */
}

/**
 * @brief This function handles CAN1 SCE interrupt.
 */
void CAN1_SCE_IRQHandler(void)
{
    /* USER CODE BEGIN CAN1_SCE_IRQn 0 */

    /* USER CODE END CAN1_SCE_IRQn 0 */
    HAL_CAN_IRQHandler(&hcan1);
    /* USER CODE BEGIN CAN1_SCE_IRQn 1 */

    /* USER CODE END CAN1_SCE_IRQn 1 */
}

/**
 * @brief This function handles EXTI line[0] interrupts.
 */
void EXTI0_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI15_10_IRQn 1 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);     //COM USART Rx ISR
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_All);   //UHF Rx ISR
    /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
 * @brief This function handles EXTI line[9:5] interrupts.
 */
void EXTI9_5_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI9_5_IRQn 1 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);     //COM USART Rx ISR
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_All);   //UHF Rx ISR
    /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
 * @brief This function handles EXTI line[15:10] interrupts.
 */
void EXTI15_10_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI15_10_IRQn 0 */
    /* USER CODE END EXTI15_10_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);    // SD card 1 detection ISR
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);    // SD card 2 detection ISR
    /* USER CODE BEGIN EXTI15_10_IRQn 1 */
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_All);
    /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
 * @brief This Callback handles EXTI line[15:10] interrupts.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if ( GPIO_PIN_15 == GPIO_Pin)
    {
        // interrupt if SD card 1 is inserted or ejected
        SD_Card_Eject_Detected(SD_MNGR_CARD_0);
    }
    else if ( GPIO_PIN_13 == GPIO_Pin)
    {
        // interrupt if SD card 2 is inserted or ejected
        SD_Card_Eject_Detected(SD_MNGR_CARD_1);
    }
    else if (( GPIO_PIN_0 == GPIO_Pin) || ( GPIO_PIN_5 == GPIO_Pin))
    {
        // interrupt if on the Rx pin the sygnal changed level during stop mode
#if (ENABLE_POWER_MANAGER == 1)
        PwrMng_SetWakeUpExterDevice(PWRMNG_ISR_PACKET_EXTI);
#endif
    }
}

/**
 * @brief This function handles SPI2 global interrupt.
 */
void SPI2_IRQHandler(void)
{
    /* USER CODE BEGIN SPI2_IRQn 0 */

    /* USER CODE END SPI2_IRQn 0 */
    HAL_SPI_IRQHandler(&hspi2);
    /* USER CODE BEGIN SPI2_IRQn 1 */

    /* USER CODE END SPI2_IRQn 1 */
}

/**
 * @brief This function handles UART4 global interrupt.
 */
void UART4_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart4);
}

/**
 * @brief This function handles UART5 global interrupt.
 */
void UART5_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart5);
}

/**
 * @brief This function handles USART3 global interrupt.
 */
void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
}

#if (ENABLE_CAMERA_OV5640 != 1)
/**
 * @brief This function handles UART4 global interrupt.
 */
void UART7_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart7);
}
#endif

void UART8_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart8);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart8)
    {
        ESTTC_ReceiveData(ESTTC_SYSCOMM_INTEFACE);
        ESTTC_NotifyTaskFromISR();
    }
    else if (huart == &huart7)
    {
        ESTTC_ReceiveData(ESTTC_PAYLOAD_INTEFACE);
        ESTTC_NotifyTaskFromISR();
    }
    else if (huart == &huart5)
    {
#if (ENABLE_GNSS_OEM719 != 1)
        ESTTC_ReceiveData(ESTTC_COMM_INTEFACE);
        ESTTC_NotifyTaskFromISR();
#else
        OEM719_ReceiveChar();
#endif
    }
    else if (huart == &huart4)
    {
        // Added by Daniel
        recieveData2();
        ESTTC_NotifyTaskFromISR();

#if (ENABLE_CSP_EXAMPLE == 1)
        cspex_uart_err_clbk();
#endif
    }
}


/*!
 *********************************************************************************************
 * @brief UART HAL error callback
 *********************************************************************************************
 * @param[input]      UART_HandleTypeDef *huart - the UART/USART interface object
 * @param[output]     none
 * @return            none
 * @note              none
 *********************************************************************************************
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart8)
    {
        HAL_UART_DeInit(huart);
        MX_UART8_Init();
        HAL_UART_Abort_IT(huart);
        HAL_UART_Receive_IT(huart, (uint8_t*)&USART_rx_data_dummy, 1);
    }
    else if (huart == &huart7)
    {
#if (ENABLE_CAMERA_OV5640 == 1)
#else
        HAL_UART_DeInit(huart);
        MX_UART7_Init();
        HAL_UART_Abort_IT(huart);
        HAL_UART_Receive_IT(huart, (uint8_t*)&USART_rx_data_dummy, 1);
#endif
    }
    else if (huart == &huart5)
    {
        HAL_UART_DeInit(huart);
        MX_UART5_Init();
        HAL_UART_Abort_IT(huart);
        HAL_UART_Receive_IT(huart, (uint8_t*)&USART_rx_data_dummy, 1);
    }
    else if (huart == &huart4)
    {
#if (ENABLE_CSP_EXAMPLE == 1)
        cspex_uart_err_clbk();
#else
        HAL_UART_DeInit(huart);
        MX_UART4_Init();
        HAL_UART_Abort_IT(huart);
        HAL_UART_Receive_IT(huart, (uint8_t*)&USART_rx_data_dummy, 1);
#endif
    }
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles DMA1 stream3 global interrupt.
 */
void DMA1_Stream3_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

    /* USER CODE END DMA1_Stream3_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_usart3_rx);
    /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

    /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
 * @brief This function handles SDIO global interrupt.
 */
void SDMMC1_IRQHandler(void)
{
    /* USER CODE BEGIN SDMMC1_IRQn 0 */

    /* USER CODE END SDMMC1_IRQn 0 */
    HAL_SD_IRQHandler(&hsd1);
    /* USER CODE BEGIN SDMMC1_IRQn 1 */

    /* USER CODE END SDMMC1_IRQn 1 */
}

/**
 * @brief This function handles SDMMC2 global interrupt.
 */
void SDMMC2_IRQHandler(void)
{
    /* USER CODE BEGIN SDMMC2_IRQn 0 */

    /* USER CODE END SDMMC2_IRQn 0 */
    HAL_SD_IRQHandler(&hsd2);
    /* USER CODE BEGIN SDMMC2_IRQn 1 */

    /* USER CODE END SDMMC2_IRQn 1 */
}

/**
 * @brief This function handles TIM5 global interrupt.
 */
void TIM5_IRQHandler(void)
{
    /* USER CODE BEGIN TIM5_IRQn 0 */

    /* USER CODE END TIM5_IRQn 0 */
    HAL_TIM_IRQHandler(&htim5);
    /* USER CODE BEGIN TIM5_IRQn 1 */

    /* USER CODE END TIM5_IRQn 1 */
}

/**
 * @brief This function handles DMA2 stream0 global interrupt.
 */
void DMA2_Stream0_IRQHandler(void)
{
    /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

    /* USER CODE END DMA2_Stream0_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_sdmmc2_rx);
    /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

    /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
 * @brief This function handles DMA2 stream3 global interrupt.
 */
void DMA2_Stream3_IRQHandler(void)
{
    /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

    /* USER CODE END DMA2_Stream3_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_sdmmc1_rx);
    /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

    /* USER CODE END DMA2_Stream3_IRQn 1 */
}

/**
 * @brief This function handles Ethernet global interrupt.
 */
void ETH_IRQHandler(void)
{
    /* USER CODE BEGIN ETH_IRQn 0 */

    /* USER CODE END ETH_IRQn 0 */
    HAL_ETH_IRQHandler(&heth);
    /* USER CODE BEGIN ETH_IRQn 1 */

    /* USER CODE END ETH_IRQn 1 */
}

/**
 * @brief This function handles Ethernet wake-up interrupt through EXTI line 19.
 */
void ETH_WKUP_IRQHandler(void)
{
    /* USER CODE BEGIN ETH_WKUP_IRQn 0 */

    /* USER CODE END ETH_WKUP_IRQn 0 */
    HAL_ETH_IRQHandler(&heth);
    /* USER CODE BEGIN ETH_WKUP_IRQn 1 */

    /* USER CODE END ETH_WKUP_IRQn 1 */
}

/**
 * @brief This function handles DMA2 stream5 global interrupt.
 */
void DMA2_Stream5_IRQHandler(void)
{
    /* USER CODE BEGIN DMA2_Stream5_IRQn 0 */

    /* USER CODE END DMA2_Stream5_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_sdmmc2_tx);
    /* USER CODE BEGIN DMA2_Stream5_IRQn 1 */

    /* USER CODE END DMA2_Stream5_IRQn 1 */
}

/**
 * @brief This function handles DMA2 stream6 global interrupt.
 */
void DMA2_Stream6_IRQHandler(void)
{
    /* USER CODE BEGIN DMA2_Stream6_IRQn 0 */

    /* USER CODE END DMA2_Stream6_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_sdmmc1_tx);
    /* USER CODE BEGIN DMA2_Stream6_IRQn 1 */

    /* USER CODE END DMA2_Stream6_IRQn 1 */
}

/**
 * @brief This function handles I2C1 event interrupt.
 */
void I2C1_EV_IRQHandler(void)
{
    /* USER CODE BEGIN I2C1_EV_IRQn 0 */

    /* USER CODE END I2C1_EV_IRQn 0 */
    HAL_I2C_EV_IRQHandler(&hi2c1);
    /* USER CODE BEGIN I2C1_EV_IRQn 1 */

    /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
 * @brief This function handles I2C1 error interrupt.
 */
void I2C1_ER_IRQHandler(void)
{
    /* USER CODE BEGIN I2C1_ER_IRQn 0 */

    /* USER CODE END I2C1_ER_IRQn 0 */
    HAL_I2C_ER_IRQHandler(&hi2c1);
    /* USER CODE BEGIN I2C1_ER_IRQn 1 */

    /* USER CODE END I2C1_ER_IRQn 1 */
}

/**
 * @brief This function handles I2C2 event interrupt.
 */
void I2C2_EV_IRQHandler(void)
{
    /* USER CODE BEGIN I2C2_EV_IRQn 0 */

    /* USER CODE END I2C2_EV_IRQn 0 */
    HAL_I2C_EV_IRQHandler(&SENSORS_I2C_HANDLER);
    /* USER CODE BEGIN I2C2_EV_IRQn 1 */

    /* USER CODE END I2C2_EV_IRQn 1 */
}

/**
 * @brief This function handles I2C2 error interrupt.
 */
void I2C2_ER_IRQHandler(void)
{
    /* USER CODE BEGIN I2C2_ER_IRQn 0 */

    /* USER CODE END I2C2_ER_IRQn 0 */
    HAL_I2C_ER_IRQHandler(&SENSORS_I2C_HANDLER);
    /* USER CODE BEGIN I2C2_ER_IRQn 1 */

    /* USER CODE END I2C2_ER_IRQn 1 */
}

/**
 * @brief This function handles I2C3 event interrupt.
 */
void I2C3_EV_IRQHandler(void)
{
    /* USER CODE BEGIN I2C3_EV_IRQn 0 */

    /* USER CODE END I2C3_EV_IRQn 0 */
    HAL_I2C_EV_IRQHandler(&hi2c3);
    /* USER CODE BEGIN I2C3_EV_IRQn 1 */

    /* USER CODE END I2C3_EV_IRQn 1 */
}

/**
 * @brief This function handles I2C3 error interrupt.
 */
void I2C3_ER_IRQHandler(void)
{
    /* USER CODE BEGIN I2C3_ER_IRQn 0 */

    /* USER CODE END I2C3_ER_IRQn 0 */
    HAL_I2C_ER_IRQHandler(&hi2c3);
    /* USER CODE BEGIN I2C3_ER_IRQn 1 */

    /* USER CODE END I2C3_ER_IRQn 1 */
}

/**
 * @brief This function handles FPU global interrupt.
 */
void FPU_IRQHandler(void)
{
    /* USER CODE BEGIN FPU_IRQn 0 */
    uint32_t reg = __get_FPSCR();
    if (reg & 0x0002)
    {
        BootData->RST_UsageFault++;
        RTC_SetRTC_BKP_CRC();
        TaskMonitor_ImmediatReset(TASK_MONITOR_FAULT_RESET);
    }

    /* USER CODE END FPU_IRQn 0 */
    /* USER CODE BEGIN FPU_IRQn 1 */

    /* USER CODE END FPU_IRQn 1 */
}

/**
 * @brief This function handles SPI4 global interrupt.
 */
void SPI4_IRQHandler(void)
{
    /* USER CODE BEGIN SPI4_IRQn 0 */

    /* USER CODE END SPI4_IRQn 0 */
    HAL_SPI_IRQHandler(&hspi4);
    /* USER CODE BEGIN SPI4_IRQn 1 */

    /* USER CODE END SPI4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
