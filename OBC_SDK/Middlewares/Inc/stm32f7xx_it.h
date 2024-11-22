/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f7xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F7xx_IT_H
#define __STM32F7xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
void SysTick_Handler(void);
void WWDG_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void UART4_IRQHandler(void);
void USART1_IRQHandler(void);
void USART6_IRQHandler(void);
void UART7_IRQHandler(void);
//void CUSTOM_UART_Receive_IT_1(void);
void CUSTOM_UART_Receive_IT_8(void);
void CUSTOM_UART_Receive_IT_7(void);
void CUSTOM_UART_Receive_IT_5(void);
void CUSTOM_UART_Receive_IT_4(void);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void DMA2_Stream0_IRQHandler(void);
void DMA2_Stream3_IRQHandler(void);

void DMA2_Stream5_IRQHandler(void);
void DMA2_Stream6_IRQHandler(void);

void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);
void I2C2_EV_IRQHandler(void);
void I2C2_ER_IRQHandler(void);
void I2C3_EV_IRQHandler(void);
void I2C3_ER_IRQHandler(void);

void PVD_IRQHandler(void);
void USART3_IRQHandler(void);
void SDMMC1_IRQHandler(void);
void TIM5_IRQHandler(void);
void UART8_IRQHandler(void);
void SDMMC2_IRQHandler(void);
#ifdef __cplusplus
}
#endif
 
void CANIT_SetActiveHandle(CAN_HandleTypeDef *hcan);
 
#endif /* __STM32F7xx_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
