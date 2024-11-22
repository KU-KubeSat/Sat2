/*!
********************************************************************************************
* @file UnhandledISRs.c
* @brief Defines all not defined Interrupt routines
********************************************************************************************
* @author            Vassil Milev
* @version           1.0.0
* @date              2020.04.09
*
* @copyright         (C) Copyright Endurosat
*
*                    Contents and presentations are protected world-wide.
*                    Any kind of using, copying etc. is prohibited without prior permission.
*                    All rights - incl. industrial property rights - are reserved.
*
* @history
* @revision{         1.0.0  , 2020.04.09, author Vassil Milev, Initial revision }
* @endhistory
********************************************************************************************
*/

#include "stm32f7xx_hal.h"

extern void Error_Handler(void);	// That functions have to be externaly defined by the user

__weak void NMI_Handler                   (void)  {	Error_Handler(); }

__weak void HardFault_Handler             (void)  {	Error_Handler(); }

__weak void MemManage_Handler             (void)  {	Error_Handler(); }

__weak void BusFault_Handler              (void)  {	Error_Handler(); }

__weak void UsageFault_Handler            (void)  {	Error_Handler(); }

__weak void SVC_Handler                   (void)  {	Error_Handler(); }

__weak void DebugMon_Handler              (void)  {	Error_Handler(); }

__weak void PendSV_Handler                (void)  {	Error_Handler(); }

__weak void SysTick_Handler               (void)  {	Error_Handler(); }

__weak void WWDG_IRQHandler               (void)  {	Error_Handler(); }

__weak void PVD_IRQHandler                (void)  {	Error_Handler(); }

__weak void TAMP_STAMP_IRQHandler         (void)  {	Error_Handler(); }

__weak void RTC_WKUP_IRQHandler           (void)  {	Error_Handler(); }

__weak void FLASH_IRQHandler              (void)  {	Error_Handler(); }

__weak void RCC_IRQHandler                (void)  {	Error_Handler(); }

__weak void EXTI0_IRQHandler              (void)  {	Error_Handler(); }

__weak void EXTI1_IRQHandler              (void)  {	Error_Handler(); }

__weak void EXTI2_IRQHandler              (void)  {	Error_Handler(); }

__weak void EXTI3_IRQHandler              (void)  {	Error_Handler(); }

__weak void EXTI4_IRQHandler              (void)  {	Error_Handler(); }

__weak void DMA1_Stream0_IRQHandler       (void)  {	Error_Handler(); }

__weak void DMA1_Stream1_IRQHandler       (void)  {	Error_Handler(); }

__weak void DMA1_Stream2_IRQHandler       (void)  {	Error_Handler(); }

__weak void DMA1_Stream3_IRQHandler       (void)  {	Error_Handler(); }

__weak void DMA1_Stream4_IRQHandler       (void)  {	Error_Handler(); }

__weak void DMA1_Stream5_IRQHandler       (void)  {	Error_Handler(); }

__weak void DMA1_Stream6_IRQHandler       (void)  {	Error_Handler(); }

__weak void ADC_IRQHandler                (void)  {	Error_Handler(); }

__weak void CAN1_TX_IRQHandler            (void)  {	Error_Handler(); }

__weak void CAN1_RX0_IRQHandler           (void)  {	Error_Handler(); }

__weak void CAN1_RX1_IRQHandler           (void)  {	Error_Handler(); }

__weak void CAN1_SCE_IRQHandler           (void)  {	Error_Handler(); }

__weak void EXTI9_5_IRQHandler            (void)  {	Error_Handler(); }

__weak void TIM1_BRK_TIM9_IRQHandler      (void)  {	Error_Handler(); }

__weak void TIM1_UP_TIM10_IRQHandler      (void)  {	Error_Handler(); }

__weak void TIM1_TRG_COM_TIM11_IRQHandler (void)  {	Error_Handler(); }

__weak void TIM1_CC_IRQHandler            (void)  {	Error_Handler(); }

__weak void TIM2_IRQHandler               (void)  {	Error_Handler(); }

__weak void TIM3_IRQHandler               (void)  {	Error_Handler(); }

__weak void TIM4_IRQHandler               (void)  {	Error_Handler(); }

__weak void I2C1_EV_IRQHandler            (void)  {	Error_Handler(); }

__weak void I2C1_ER_IRQHandler            (void)  {	Error_Handler(); }

__weak void I2C2_EV_IRQHandler            (void)  {	Error_Handler(); }

__weak void I2C2_ER_IRQHandler            (void)  {	Error_Handler(); }

__weak void SPI1_IRQHandler               (void)  {	Error_Handler(); }

__weak void SPI2_IRQHandler               (void)  {	Error_Handler(); }

__weak void USART1_IRQHandler             (void)  {	Error_Handler(); }

__weak void USART2_IRQHandler             (void)  {	Error_Handler(); }

__weak void USART3_IRQHandler             (void)  {	Error_Handler(); }

__weak void EXTI15_10_IRQHandler          (void)  {	Error_Handler(); }

__weak void RTC_Alarm_IRQHandler          (void)  {	Error_Handler(); }

__weak void OTG_FS_WKUP_IRQHandler        (void)  {	Error_Handler(); }

__weak void TIM8_BRK_TIM12_IRQHandler     (void)  {	Error_Handler(); }

__weak void TIM8_UP_TIM13_IRQHandler      (void)  {	Error_Handler(); }

__weak void TIM8_TRG_COM_TIM14_IRQHandler (void)  {	Error_Handler(); }

__weak void TIM8_CC_IRQHandler            (void)  {	Error_Handler(); }

__weak void DMA1_Stream7_IRQHandler       (void)  {	Error_Handler(); }

__weak void FMC_IRQHandler                (void)  {	Error_Handler(); }

__weak void SDMMC1_IRQHandler             (void)  {	Error_Handler(); }

__weak void TIM5_IRQHandler               (void)  {	Error_Handler(); }

__weak void SPI3_IRQHandler               (void)  {	Error_Handler(); }

__weak void UART4_IRQHandler              (void)  {	Error_Handler(); }

__weak void UART5_IRQHandler              (void)  {	Error_Handler(); }

__weak void TIM6_DAC_IRQHandler           (void)  {	Error_Handler(); }

__weak void TIM7_IRQHandler               (void)  {	Error_Handler(); }

__weak void DMA2_Stream0_IRQHandler       (void)  {	Error_Handler(); }

__weak void DMA2_Stream1_IRQHandler       (void)  {	Error_Handler(); }

__weak void DMA2_Stream2_IRQHandler       (void)  {	Error_Handler(); }

__weak void DMA2_Stream3_IRQHandler       (void)  {	Error_Handler(); }

__weak void DMA2_Stream4_IRQHandler       (void)  {	Error_Handler(); }

__weak void ETH_IRQHandler                (void)  {	Error_Handler(); }

__weak void ETH_WKUP_IRQHandler           (void)  {	Error_Handler(); }

__weak void CAN2_TX_IRQHandler            (void)  {	Error_Handler(); }

__weak void CAN2_RX0_IRQHandler           (void)  {	Error_Handler(); }

__weak void CAN2_RX1_IRQHandler           (void)  {	Error_Handler(); }

__weak void CAN2_SCE_IRQHandler           (void)  {	Error_Handler(); }

__weak void OTG_FS_IRQHandler             (void)  {	Error_Handler(); }

__weak void DMA2_Stream5_IRQHandler       (void)  {	Error_Handler(); }

__weak void DMA2_Stream6_IRQHandler       (void)  {	Error_Handler(); }

__weak void DMA2_Stream7_IRQHandler       (void)  {	Error_Handler(); }

__weak void USART6_IRQHandler             (void)  {	Error_Handler(); }

__weak void I2C3_EV_IRQHandler            (void)  {	Error_Handler(); }

__weak void I2C3_ER_IRQHandler            (void)  {	Error_Handler(); }

__weak void OTG_HS_EP1_OUT_IRQHandler     (void)  {	Error_Handler(); }

__weak void OTG_HS_EP1_IN_IRQHandler      (void)  {	Error_Handler(); }

__weak void OTG_HS_WKUP_IRQHandler        (void)  {	Error_Handler(); }

__weak void OTG_HS_IRQHandler             (void)  {	Error_Handler(); }

__weak void DCMI_IRQHandler               (void)  {	Error_Handler(); }

__weak void CRYP_IRQHandler               (void)  {	Error_Handler(); }

__weak void HASH_RNG_IRQHandler           (void)  {	Error_Handler(); }

__weak void FPU_IRQHandler                (void)  {	Error_Handler(); }

__weak void UART7_IRQHandler              (void)  {	Error_Handler(); }

__weak void UART8_IRQHandler              (void)  {	Error_Handler(); }

__weak void SPI4_IRQHandler               (void)  {	Error_Handler(); }

__weak void SPI5_IRQHandler               (void)  {	Error_Handler(); }

__weak void SPI6_IRQHandler               (void)  {	Error_Handler(); }

__weak void SAI1_IRQHandler               (void)  {	Error_Handler(); }

__weak void LTDC_IRQHandler               (void)  {	Error_Handler(); }

__weak void LTDC_ER_IRQHandler            (void)  {	Error_Handler(); }

__weak void DMA2D_IRQHandler              (void)  {	Error_Handler(); }

__weak void SAI2_IRQHandler               (void)  {	Error_Handler(); }

__weak void QUADSPI_IRQHandler            (void)  {	Error_Handler(); }

__weak void LPTIM1_IRQHandler             (void)  {	Error_Handler(); }

__weak void CEC_IRQHandler                (void)  {	Error_Handler(); }

__weak void I2C4_EV_IRQHandler            (void)  {	Error_Handler(); }

__weak void I2C4_ER_IRQHandler            (void)  {	Error_Handler(); }

__weak void SPDIF_RX_IRQHandler           (void)  {	Error_Handler(); }

__weak void DFSDM1_FLT0_IRQHandler        (void)  {	Error_Handler(); }

__weak void DFSDM1_FLT1_IRQHandler        (void)  {	Error_Handler(); }

__weak void DFSDM1_FLT2_IRQHandler        (void)  {	Error_Handler(); }

__weak void DFSDM1_FLT3_IRQHandler        (void)  {	Error_Handler(); }

__weak void SDMMC2_IRQHandler             (void)  {	Error_Handler(); }

__weak void CAN3_TX_IRQHandler            (void)  {	Error_Handler(); }

__weak void CAN3_RX0_IRQHandler           (void)  {	Error_Handler(); }

__weak void CAN3_RX1_IRQHandler           (void)  {	Error_Handler(); }

__weak void CAN3_SCE_IRQHandler           (void)  {	Error_Handler(); }

__weak void JPEG_IRQHandler               (void)  {	Error_Handler(); }

__weak void MDIOS_IRQHandler              (void)  {	Error_Handler(); }

