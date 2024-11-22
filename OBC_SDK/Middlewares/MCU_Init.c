/*!
********************************************************************************************
* @file MCU_Init.c
* @brief All initialization routines needed after Power up
********************************************************************************************
* @author            Vassil Milev
* @version           1.0.0
* @date              2019.01.10
*
* @copyright         (C) Copyright Endurosat
*
*                    Contents and presentations are protected world-wide.
*                    Any kind of using, copying etc. is prohibited without prior permission.
*                    All rights - incl. industrial property rights - are reserved.
*
* @history
* @revision{         1.0.0  , 2019.01.10, author Vassil Milev, Initial revision }
* @endhistory
********************************************************************************************
*/

/*
*********************************************************************************************
* INCLUDES
*********************************************************************************************
*/
#include "MCU_Init.h"
#include "cmsis_os.h"
#include "SdMngr.h"
#include "string.h"
#include "DAT_Inputs.h"
#include "Panels.h"
#include "EEPROM_emul.h"
#include "stm32f7xx_ll_rcc.h"
#include "stm32f7xx_ll_rtc.h"
#include "es_exeh.h"                /* Include DTC error handling */
#include "MX_I2C.h"
#include "debug.h"
#include "LIS3MDL_MAG_driver.h"
#include "AppTasks.h"

/*
*********************************************************************************************
* INTERNAL DEFINES
*********************************************************************************************
*/
/* No Internal defines */

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
uint32_t USART_rx_data_dummy;

/**
 * @brief Peripheral HAL interfaces in use
 */
CAN_HandleTypeDef hcan1;
ETH_HandleTypeDef heth;

ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SD_HandleTypeDef hsd1;
SD_HandleTypeDef hsd2;

DMA_HandleTypeDef hdma_sdmmc1_rx;
DMA_HandleTypeDef hdma_sdmmc1_tx;
DMA_HandleTypeDef hdma_sdmmc2_rx;
DMA_HandleTypeDef hdma_sdmmc2_tx;

DMA_HandleTypeDef hdma_usart3_rx;

SPI_HandleTypeDef hspi5;

SPI_HandleTypeDef hspi2;

SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart3;

WWDG_HandleTypeDef hwwdg;
IWDG_HandleTypeDef hiwdg;
//SRAM_HandleTypeDef hsram1;
RTC_HandleTypeDef hrtc;
RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef sDate = {0};
RTC_AlarmTypeDef sAlarm = {0};
extern Disk_drvTypeDef  disk;

/*
*********************************************************************************************
* INTERNAL (STATIC) VARIABLES DEFINITION/DECLARATION 
*********************************************************************************************
*/
volatile static uint8_t I2C2_Used = 0;
volatile static uint8_t PANLE_SPI_Used = 0;

/*
*********************************************************************************************
* INTERNAL (STATIC) ROUTINES DECLARATION
*********************************************************************************************
*/
/* No Internal routines declaration */

/*
*********************************************************************************************
* EXTERNAL (NONE STATIC) ROUTINES DEFINITION
*********************************************************************************************
*/
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
static void RCC_SDIO1_RESET(void);
static void RCC_SDIO2_RESET(void);
static void RCC_DMA2_RESET(void);

/*!
*********************************************************************************************
 * @brief System Clock Configuration
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void SystemClock_MaxConfig(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_HSI);

    /** Configure LSE Drive Capability
    */
    HAL_PWR_EnableBkUpAccess();

    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_HIGH);

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 432;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }
    /** Activate the Over-Drive mode
    */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
      Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
    {
      Error_Handler();
    }
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                                |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USART6
                                |RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_UART5
                                |RCC_PERIPHCLK_UART7|RCC_PERIPHCLK_UART8
                                |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2
                                |RCC_PERIPHCLK_I2C3|RCC_PERIPHCLK_SDMMC1
                                |RCC_PERIPHCLK_SDMMC2|RCC_PERIPHCLK_CLK48;
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_SYSCLK;
    PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
    PeriphClkInitStruct.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
    PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
    PeriphClkInitStruct.Uart7ClockSelection = RCC_UART7CLKSOURCE_PCLK1;
    PeriphClkInitStruct.Uart8ClockSelection = RCC_UART8CLKSOURCE_PCLK1;
    PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
    PeriphClkInitStruct.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
    PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
    PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
    PeriphClkInitStruct.Sdmmc2ClockSelection = RCC_SDMMC2CLKSOURCE_CLK48;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    LL_RCC_HSE_EnableCSS();
}
/*!
*********************************************************************************************
 * @brief System Clock Configuration
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void SystemClock_IdleConfig(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_HSI);

    /** Configure LSE Drive Capability
    */
    HAL_PWR_EnableBkUpAccess();

    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_HIGH);

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);//__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the CPU, AHB and APB busses clocks
    */

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 432;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }
    /** Activate the Over-Drive mode
    */
    if (HAL_PWREx_DisableOverDrive() != HAL_OK)
    {
      Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;//RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3 /*FLASH_LATENCY_7*/) != HAL_OK)
    {
      Error_Handler();
    }
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                                |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USART6
                                |RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_UART5
                                |RCC_PERIPHCLK_UART7|RCC_PERIPHCLK_UART8
                                |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2
                                |RCC_PERIPHCLK_I2C3|RCC_PERIPHCLK_SDMMC1
                                |RCC_PERIPHCLK_SDMMC2|RCC_PERIPHCLK_CLK48;
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_SYSCLK;
    PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
    PeriphClkInitStruct.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
    PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
    PeriphClkInitStruct.Uart7ClockSelection = RCC_UART7CLKSOURCE_PCLK1;
    PeriphClkInitStruct.Uart8ClockSelection = RCC_UART8CLKSOURCE_PCLK1;
    PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
    PeriphClkInitStruct.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
    PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
    PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
    PeriphClkInitStruct.Sdmmc2ClockSelection = RCC_SDMMC2CLKSOURCE_CLK48;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    LL_RCC_HSE_EnableCSS();
}

/*!
*********************************************************************************************
 * @brief General purpose I/O lines initialisation
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /*Configure GPIO pin Output Level */

    HAL_GPIO_WritePin(EN_CAM_GPIO_Port, EN_CAM_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : EN_CAM_Pin */
    GPIO_InitStruct.Pin = EN_CAM_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EN_CAM_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, LED_G_Pin, GPIO_PIN_RESET);    //Turn Off the Green LED

    /*Configure GPIO pins : LED_G_Pin */
    GPIO_InitStruct.Pin = LED_G_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_G_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_RESET);
    /*Configure GPIO pins : PCPin PCPin */
    GPIO_InitStruct.Pin = LED_Y_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_Y_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOG, EPS_RST_Pin, GPIO_PIN_SET);    //Do not reset the EPS at start up

    /*Configure GPIO pins : EPS_RST_Pin */
    GPIO_InitStruct.Pin = EPS_RST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EPS_RST_GPIO_Port, &GPIO_InitStruct);

    MCU_Init_I2cPullUps();
}

/*!
*********************************************************************************************
 * @brief Deinit all General purpose I/O lines
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MX_AllPeripheryAndGpio_DeInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

/////////////// Disable peripheries ///////////////////////
  GREEN_LED_OFF();
  AMBER_LED_OFF();

  MX_USART4_UART_DeInit();
  MX_UART5_DeInit();
  MX_UART7_DeInit();
  MX_UART8_DeInit();

  Panels_ChipSelect2_DeInit();

  MCU_MagnetometersDeInit();
  MCU_MagnetorqersDeInit();
  MCU_GyrosDeInit();
  MCU_TemprDeInit();
  MCU_SunSensDeInit();

//  MX_SENSORS_I2C_DeInit(); // already done in the actual deinit of the sensors
//  MX_SYSTEM_I2C_DeInit(); //Never deinit System I2C
  MX_I2C_BusDeInit(MX_I2C_BUS_PAYLOAD);

/////////////// UART 1 RS485 disable transmit ///////////////////////
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);  //USART1_RTS_RS485_HS1_DE_SYS1_Pin
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/////////////// UART 6 RS485 disable transmit ///////////////////////
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_RESET);   //USART6_RTS_RS485_PAY1_Pin
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/////////////// EPS reset pin ///////////////////////
  HAL_GPIO_WritePin(EPS_RST_GPIO_Port, EPS_RST_Pin, GPIO_PIN_SET);
  /*Configure GPIO pins :  EPS_RST_Pin USART6_RTS_RS485_PAY1_Pin */
  GPIO_InitStruct.Pin = EPS_RST_Pin/*USART6_RTS_RS485_PAY1_Pin*/;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/////////////// GNSS control pins ///////////////////////
  HAL_GPIO_WritePin(En_GNSS_GPIO_Port, En_GNSS_Pin, GPIO_PIN_RESET);
  /*Configure GPIO pins : En_GNSS_Pin */
  GPIO_InitStruct.Pin = En_GNSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

/////////////// Ardu camera pins ///////////////////////
  HAL_GPIO_WritePin(EN_CAM_GPIO_Port, EN_CAM_Pin, GPIO_PIN_RESET);
  /*Configure GPIO pin : EN_CAM_Pin */
  GPIO_InitStruct.Pin = EN_CAM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EN_CAM_GPIO_Port, &GPIO_InitStruct);

/////////////// RS 422 pins ///////////////////////
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5 | GPIO_PIN_12, GPIO_PIN_RESET);   // leave it with Rx enable so it can wake up the MCU

  GPIO_InitStruct.Pin = RS422_HS1_nRE_UART1_SYS1_Pin  //RS422_HS1_nRE
                       |RS422_HS2_nRE_UART6_PAY1_Pin; //RS422_HS2_nRE
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  //DE1 -> PF10 RS422_HS1_DE
  //DE2 -> PE10 RS422_HS2_DE
  HAL_GPIO_WritePin(RS422_HS1_DE_UART1_SYS1_GPIO_Port, RS422_HS1_DE_UART1_SYS1_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = RS422_HS1_DE_UART1_SYS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS422_HS1_DE_UART1_SYS1_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(RS422_HS2_DE_UART6_PAY1_GPIO_Port, RS422_HS2_DE_UART6_PAY1_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = RS422_HS2_DE_UART6_PAY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS422_HS2_DE_UART6_PAY1_GPIO_Port, &GPIO_InitStruct);
/////////////   Ethernet pins ////////////////////

  GPIO_InitStruct.Pin = GPIO_PIN_1   //ETH_MDC
                          |GPIO_PIN_2   //ETH_TXD2
                          |GPIO_PIN_3   //ETH_TX_CLK
                          |GPIO_PIN_4   //ETH_RXD0
                          |GPIO_PIN_5;  //ETH_RXD1
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_10;     //ETH_RX_ER
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2
                           PA5 PA6 PA7 PA9
                           PA11
                           USART1_RTS_RS485_HS1_DE_SYS1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1  //ETH_RX_CLK
                          |GPIO_PIN_2   //ETH_MDIO
                          |GPIO_PIN_7;  //ETH_RX_DV
                          /*|USART1_RTS_RS485_HS1_DE_SYS1_Pin;*/
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins :  PB1 PB10 PB11
                           PB12 PB13 PB5 PB6
                           PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_1  //ETH_RXD3
                          |GPIO_PIN_11  //ETH0TX_EN
                          |GPIO_PIN_12  //ETH_TXD0
                          |GPIO_PIN_13; //ETH_TXD1
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);   //*******

  /*Configure GPIO pins : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2; //ETH_TXD3
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);


  /*Configure GPIO pins : PH2 PH6 PH14 */
  GPIO_InitStruct.Pin = ETH_RSTn_Pin
                          |GPIO_PIN_6   //ETH_RXD2
                          |ETH_EN_Pin; //CAN1_RX
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

////////////////////////////// CAN Pins ////////////////////////////

  HAL_GPIO_WritePin(GPIOA, CAN_RS_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_14; //CAN1_RX
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : CAN_RS_Pin  */
  GPIO_InitStruct.Pin = CAN_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1;   //CAN1_TX
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/*!
*********************************************************************************************
 * @brief Initialize the GPIOS for OBC outputs
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MX_GPIO_Outputs_Set(MCU_Init_OutStates_struct *states)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    if ((states->OutToChange >> MCU_INIT_OUT1) & 0x01)
    {
        HAL_GPIO_WritePin(OBC_OUT1_GPIO_Port, OBC_OUT1_Pin, (states->OutState >> MCU_INIT_OUT1) & 0x01);
    }

    if ((states->OutToChange >> MCU_INIT_OUT2) & 0x01)
    {
        HAL_GPIO_WritePin(OBC_OUT2_GPIO_Port, OBC_OUT2_Pin, (states->OutState >> MCU_INIT_OUT2) & 0x01);
    }

    if ((states->OutToChange >> MCU_INIT_OUT3) & 0x01)
    {
        HAL_GPIO_WritePin(OBC_OUT3_GPIO_Port, OBC_OUT3_Pin, (states->OutState >> MCU_INIT_OUT3) & 0x01);
    }

    if ((states->OutToChange >> MCU_INIT_OUT5) & 0x01)
    {
        HAL_GPIO_WritePin(OBC_OUT5_GPIO_Port, OBC_OUT5_Pin, (states->OutState >> MCU_INIT_OUT5) & 0x01);
    }

    if ((states->OutToChange >> MCU_INIT_OUT4_6) & 0x01)
    {
        HAL_GPIO_WritePin(OBC_OUT4_6_GPIO_Port, OBC_OUT4_6_Pin, (states->OutState >> MCU_INIT_OUT4_6) & 0x01);
    }

    if ((states->OutToChange >> MCU_INIT_OUT7) & 0x01)
    {
        HAL_GPIO_WritePin(OBC_OUT7_GPIO_Port, OBC_OUT7_Pin, (states->OutState >> MCU_INIT_OUT7) & 0x01);
    }
    if ((states->OutToChange >> MCU_INIT_OUT8) & 0x01)
    {
        HAL_GPIO_WritePin(OBC_OUT8_GPIO_Port, OBC_OUT8_Pin, (states->OutState >> MCU_INIT_OUT8) & 0x01);
    }

    /*Configure GPIO pins : OBC_OUT1_Pin */
    GPIO_InitStruct.Pin = OBC_OUT1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : OBC_OUT5_Pin */
    GPIO_InitStruct.Pin = OBC_OUT5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /*Configure GPIO pins : OBC_OUT4_6_Pin OBC_OUT3_Pin OBC_OUT2_Pin */
    GPIO_InitStruct.Pin = OBC_OUT4_6_Pin|OBC_OUT3_Pin|OBC_OUT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = OBC_OUT7_Pin|OBC_OUT8_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
}

/*!
*********************************************************************************************
 * @brief DeInitialize the GPIOS for OBC outputs
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MX_GPIO_Outputs_DeInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();


    /*Configure GPIO pins : OBC_OUT1_Pin */
    GPIO_InitStruct.Pin = OBC_OUT1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : OBC_OUT5_Pin */
    GPIO_InitStruct.Pin = OBC_OUT5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /*Configure GPIO pins : OBC_OUT4_6_Pin OBC_OUT3_Pin OBC_OUT2_Pin */
    GPIO_InitStruct.Pin = OBC_OUT4_6_Pin|OBC_OUT3_Pin|OBC_OUT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    // DeInit GPIOs
    GPIO_InitStruct.Pin = OBC_OUT7_Pin|OBC_OUT8_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
}

/*!
*********************************************************************************************
 * @brief ADC1 (Analog-to-Digital Converter Module #1) initialisation
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
      Error_Handler();
    }
    /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
    sConfig.Channel = ADC_CHANNEL_3;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
      Error_Handler();
    }
}

/*!
*********************************************************************************************
 * @brief SDIO (Secure Digital Input Output Module) initialisation
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
static void MX_SDMMC1_SD_Init(void)
{
    // Enable GPIO and SPI Clocks
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /* USER CODE BEGIN SDMMC1_Init 0 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = SD_DET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SD_DET_GPIO_Port, &GPIO_InitStruct);//ADD exti13 for SD2 -> PC13

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0); //ADD exti13 for SD2
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    /* USER CODE END SDMMC1_Init 0 */

    /* USER CODE BEGIN SDMMC1_Init 1 */

    /* USER CODE END SDMMC1_Init 1 */
    hsd1.Instance = SDMMC1;
    hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
    hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
    hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
    hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd1.Init.ClockDiv = 0;
    /* USER CODE BEGIN SDMMC1_Init 2 */

    /* SDMMC1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SDMMC1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SDMMC1_IRQn);

    /* USER CODE END SDMMC1_Init 2 */
}

/**
  * @brief SDMMC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC2_SD_Init(void)
{
  /* USER CODE BEGIN SDMMC2_Init 0 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // Enable GPIO and SPI Clocks
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = SD2_DET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SD2_DET_GPIO_Port, &GPIO_InitStruct);//ADD exti13 for SD2 -> PC13

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0); //ADD exti13 for SD2
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE END SDMMC2_Init 0 */

  /* USER CODE BEGIN SDMMC2_Init 1 */

  /* USER CODE END SDMMC2_Init 1 */
  hsd2.Instance = SDMMC2;
  hsd2.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd2.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd2.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd2.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd2.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd2.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDMMC2_Init 2 */


  /* SDMMC1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SDMMC2_IRQn, 5, 1);
  HAL_NVIC_EnableIRQ(SDMMC2_IRQn);

  /* USER CODE END SDMMC2_Init 2 */

}


#if (ENABLE_CAMERA_OV5640 == 1)
/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI2 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  hspi5.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi5.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

void MX_SPI5_DeInit(void)
{
    HAL_SPI_DeInit(&hspi5);

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = SPI5_SCK_Pin|SPI5_MISO_Pin|SPI5_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
}
#endif

/*!
*********************************************************************************************
 * @brief SPI2 (Serial Peripheral Interface Module #2) initialisation
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MX_PANEL_SPI_Init(SPI_HandleTypeDef *hspi)
{
    /* SPI2 parameter configuration*/
    hspi->Instance = PANEL_SPI;
    hspi->Init.Mode = SPI_MODE_MASTER;
    hspi->Init.Direction = SPI_DIRECTION_2LINES;
    hspi->Init.DataSize = SPI_DATASIZE_16BIT;
    hspi->Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi->Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi->Init.NSS = SPI_NSS_SOFT;
    hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
    hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi->Init.TIMode = SPI_TIMODE_DISABLE;
    hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi->Init.CRCPolynomial = 7;
    hspi->Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi->Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    if (HAL_SPI_Init(hspi) != HAL_OK)
    {
      Error_Handler();
    }
}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI4_Init(void)
{
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;  
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
}

/*!
*********************************************************************************************
 * @brief TIM5 (Timer Module #5) initialisation
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MX_TIM5_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 0;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = MAGTORQ_PWM_PERIOD;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
    {
      Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
    {
      Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
    {
      Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
      Error_Handler();
    }
    /* USER CODE BEGIN TIM5_Init 2 */

    /* USER CODE END TIM5_Init 2 */
    HAL_TIM_MspPostInit(&htim5);
}

/*!
*********************************************************************************************
 * @brief UART4 (Universal Asynchronous Receiver-Transmitter Module #4) initialisation
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MX_USART4_UART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_GPIO_WritePin(U4_RS485_S2_TE_GPIO_Port, U4_RS485_S2_TE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOF, U4_RS485_S2_DE_Pin|U4_RS485_S2_nRE_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : U4_RS485_S2_TE_Pin */
    GPIO_InitStruct.Pin = U4_RS485_S2_TE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(U4_RS485_S2_TE_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins :  U4_RS485_S2_DE_Pin U4_RS485_S2_nRE_Pin  */
    GPIO_InitStruct.Pin = U4_RS485_S2_DE_Pin|U4_RS485_S2_nRE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    huart4.Instance = UART4;
    huart4.Init.BaudRate = 115200;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart4) != HAL_OK)
    {
      Error_Handler();
    }
}

/*!
*********************************************************************************************
 * @brief UART4 (Universal Asynchronous Receiver-Transmitter Module #4) De-initialisation
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MX_USART4_UART_DeInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_UART_DeInit(&huart4);

    HAL_GPIO_WritePin(GPIOF, U4_RS485_S2_DE_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins :  U4_RS485_S2_DE_Pin U4_RS485_S2_nRE_Pin  */
    GPIO_InitStruct.Pin = U4_RS485_S2_nRE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = U4_RS485_S2_DE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = U4_RS485_S2_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = U4_RS485_S2_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
void MX_UART5_Init(void)
{
  huart5.Instance = UART5;

#if (ENABLE_GNSS_OEM719 == 1)
  huart5.Init.BaudRate = 9600;
#else
  huart5.Init.BaudRate = 115200;
#endif

  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
}

void MX_UART5_DeInit(void)
{
    HAL_UART_DeInit((UART_HandleTypeDef*)&huart5);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = U5_PROTO_USB_SYSTEM_TX_Pin|U5_PROTO_USB_SYSTEM_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(U5_PROTO_USB_SYSTEM_RX_GPIO_Port, &GPIO_InitStruct);
}


void MX_UART4_Init(void)
{
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
}

void MX_UART4_DeInit(void)
{
    HAL_UART_DeInit((UART_HandleTypeDef*)&huart4);

    {
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = U4_RS485_S2_RX_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(U4_RS485_S2_RX_GPIO_Port, &GPIO_InitStruct);
    }
    {
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = U4_RS485_S2_TX_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(U4_RS485_S2_TX_GPIO_Port, &GPIO_InitStruct);
    }
}


#if (ENABLE_CAMERA_OV5640 != 1)
/*!
*********************************************************************************************
 * @brief USART7 (Universal Synchronous and Asynchronous Receiver-Transmitter Module #1) initialisation
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
/* UART7 init function */
void MX_UART7_Init(void)
{
    huart7.Instance = UART7;
    huart7.Init.BaudRate = 115200;
    huart7.Init.WordLength = UART_WORDLENGTH_8B;
    huart7.Init.StopBits = UART_STOPBITS_1;
    huart7.Init.Parity = UART_PARITY_NONE;
    huart7.Init.Mode = UART_MODE_TX_RX;
    huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart7.Init.OverSampling = UART_OVERSAMPLING_16;
    huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart7) != HAL_OK)
    {
      Error_Handler();
    }
}
#endif

void MX_UART7_DeInit(void)
{
    HAL_UART_DeInit((UART_HandleTypeDef*)&huart7);

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = U7_PAY_RTS_SPI5_MISO_Pin|U7_PAY_CTS_SPI5_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = U7_PAY_RX_SPI5_NSS_Pin | U7_PAY_TX_SPI5_SCK_Pin
                          | U7_PAY_RTS_SPI5_MISO_Pin | U7_PAY_CTS_SPI5_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = U7_PAY_TX_SPI5_SCK_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
}

/**
  * @brief UART8 Initialization Function
  * @param None
  * @retval None
  */
void MX_UART8_Init(void)
{
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  //CTS and RTS - not in use
  GPIO_InitStruct.Pin = U8_COMM_CTS_Pin|U8_COMM_RTS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void MX_UART8_DeInit(void)
{
    HAL_UART_DeInit((UART_HandleTypeDef*)&huart8);

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = U8_COMM_TX_Pin|U8_COMM_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = U8_COMM_CTS_Pin|U8_COMM_RTS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART3_UART_Init(uint32_t BaudRate)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(U3_RS485_S1_nRE_GPIO_Port, U3_RS485_S1_nRE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(U3_RS485_S1_TE_GPIO_Port, U3_RS485_S1_TE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(U3_RS485_S1_DE_GPIO_Port, U3_RS485_S1_DE_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : U3_RS485_S1_nRE_Pin*/
    GPIO_InitStruct.Pin = U3_RS485_S1_nRE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(U3_RS485_S1_nRE_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = U3_RS485_S1_TE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(U3_RS485_S1_TE_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : U3_RS485_S1_DE_Pin */
    GPIO_InitStruct.Pin = U3_RS485_S1_DE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    huart3.Instance = USART3;
    huart3.Init.BaudRate = BaudRate;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_8;
    huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
}

void MX_USART3_UART_DeInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_UART_DeInit(&huart3);

    HAL_GPIO_WritePin(GPIOF, U3_RS485_S1_DE_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : U3_RS485_S1_DE_Pin */
    GPIO_InitStruct.Pin = U3_RS485_S1_DE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = U3_RS485_S1_nRE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(U3_RS485_S1_nRE_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = U3_RS485_S1_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = U3_RS485_S1_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(U3_RS485_S1_TX_GPIO_Port, &GPIO_InitStruct);
}

/*!
*********************************************************************************************
 * @brief DMA (Direct Memory Access) initialisation; enabling the DMA clock
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MX_DMA_Init(void)
{
    MX_DMA_USART3_Init();

    MX_DMA_SDMMC1_Init();
    MX_DMA_SDMMC2_Init();
}

/*!
*********************************************************************************************
 * @brief Channels from DMA used by the USART3 controller (Direct Memory Access Module #1) initialisation; enabling the DMA1 clock
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MX_DMA_USART3_Init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Stream1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

/*!
*********************************************************************************************
 * @brief Channels from DMA used by the SDIO1 controller (Direct Memory Access Module #2) initialisation; enabling the DMA2 clock
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MX_DMA_SDMMC1_Init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA2_Stream3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    /* DMA2_Stream6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
}

/*!
*********************************************************************************************
 * @brief Channels from DMA used by the SDIO1 controller (Direct Memory Access Module #2) initialisation; enabling the DMA2 clock
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MX_DMA_SDMMC2_Init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA2_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    /* DMA2_Stream5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 5, 1);
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
}

/*!
*********************************************************************************************
 * @brief RTC (Real-Time Clock Module) initialisation
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MX_RTC_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();

    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 127;
    hrtc.Init.SynchPrediv = 255;
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    if (HAL_RTC_Init(&hrtc) != HAL_OK)
    {
        Error_Handler();
    }
}

/*!
*********************************************************************************************
 * @brief WWDG (Window WatchDoG Module) initialisation
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MX_WWDG_Init(void)
{
#ifdef DEBUG_ENABLED
    __HAL_DBGMCU_FREEZE_WWDG();
#endif /* #ifdef DEBUG_ENABLED */

    //time = APB1 clk / 2096 / Prescaler / 2; Start = hwwdg.Init.Counter ; Beginning of the windows = hwwdg.Init.Window; End of windows = 0x40;
    hwwdg.Instance = WWDG;
    hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
    hwwdg.Init.Counter = 127; /* 127 - 0x40 = 75ms max time */
    hwwdg.Init.Window = hwwdg.Init.Counter - 5; /* 6.01 ms min time => windows is 70ms wide => minimum stable refresh time 7ms (chosen TASK_MONITOR_REFRESH_WDG_PERIOD), maximum 75*/
    hwwdg.Init.EWIMode = WWDG_EWI_ENABLE;
    if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
    {
        Error_Handler();
    }
}


/*!
*********************************************************************************************
 * @brief IWDG (Independent WatchDoG Module) initialisation
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MX_IWDG_Init(void)
{
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
    hiwdg.Init.Window = 4095;
    hiwdg.Init.Reload = 4095; /* that is the very maximum time 30sec */
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
    {
        Error_Handler();
    }
}

/*!
*********************************************************************************************
 * @brief Initialize all periphery needed for the magnitometers
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MCU_MagnetometersInit(void)
{
    //Init SPI2
    I2C2_Used |= SENSORS_I2C_USE_COMPAS;

    if (Magnitometers_LIS3MDL_Init(LIS3MDL_MAG_I2C_ADDRESS_LOW) != E_OK)
    {
        EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_MCU_INIT, eEXEH_MCU_INIT_EXCEPTION_ID_MANITO_INIT, __LINE__);
    }

    if (Magnitometers_LIS3MDL_Init(LIS3MDL_MAG_I2C_ADDRESS_HIGH) != E_OK)
    {
        EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_MCU_INIT, eEXEH_MCU_INIT_EXCEPTION_ID_MANITO_INIT, __LINE__);
    }
}

/*!
*********************************************************************************************
 * @brief DeInitialize all periphery needed for the magnitometers
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MCU_MagnetometersDeInit(void)
{
    //todo VVS: add low-power mode for the magnetometers

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    I2C2_Used &= ~SENSORS_I2C_USE_COMPAS;
    if( I2C2_Used == 0 )
    {
        //DeInit I2C
        HAL_I2C_DeInit(&SENSORS_I2C_HANDLER);

        //Deinit GPIOs
        GPIO_InitStruct.Pin = I2C2_SEN_SCL_Pin|I2C2_SEN_SDA_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(I2C2_SEN_SDA_GPIO_Port, &GPIO_InitStruct);
    }
}

/*!
*********************************************************************************************
 * @brief Initialize all periphery needed for the magnetorquers
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MCU_MagnetorqersInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIO and SPI Clocks
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    // Init GPIOs

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(PAN4_DIR1_GPIO_Port, PAN4_DIR1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PAN5_DIR1_GPIO_Port, PAN5_DIR1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOH, PAN6_DIR1_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(PAN_PS_GPIO_Port, PAN_PS_Pin, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = PAN6_DIR1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PAN5_DIR1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /*Configure GPIO pins : PAN4_DIR1_Pin*/
    GPIO_InitStruct.Pin = PAN4_DIR1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PAN4_DIR1_GPIO_Port, &GPIO_InitStruct);  //PD

    /*Configure GPIO pins : PAN_PS_Pin*/
    GPIO_InitStruct.Pin = PAN_PS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PAN_PS_GPIO_Port, &GPIO_InitStruct);  //PE

    //Init TIM5
    MX_TIM5_Init();

    Stop_Magnetorquers();
}

/*!
*********************************************************************************************
 * @brief DeInitialize all periphery needed for the magnetorquers
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MCU_MagnetorqersDeInit(void)
{
    Stop_Magnetorquers();
	
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    //DeInit TIM5
    HAL_TIM_PWM_DeInit(&htim5);

    //Enalbe power safe mode of the H-Bridge Driver
    HAL_GPIO_WritePin(PAN_PS_GPIO_Port, PAN_PS_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : PAN_PS_Pin*/
    GPIO_InitStruct.Pin = PAN_PS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PAN_PS_GPIO_Port, &GPIO_InitStruct);  //PE

    // DeInit GPIOs
    GPIO_InitStruct.Pin = PAN6_DIR1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PAN5_DIR1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PAN5_DIR1_GPIO_Port, &GPIO_InitStruct);   //PF

    GPIO_InitStruct.Pin = PAN4_DIR1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PAN4_DIR1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PAN4_PWM_Pin|PAN5_PWM_Pin|PAN6_PWM_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
}

/*!
*********************************************************************************************
 * @brief Initialize all periphery needed for the gyroscopes
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MCU_GyrosInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    //Init SPI2
    PANLE_SPI_Used |= PANLE_SPI_USE_GPS;
    if(( PANLE_SPI_Used & PANLE_SPI_USE_TEMPR ) == 0)   // If temerature sensor is not initialised
    {
        Panels_ChipSelect2_Init();

        //Init SPI if not done already
        MX_PANEL_SPI_Init(&PANEL_SPI_HANDLER);
    }

    // Enable GPIO and SPI Clocks
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();


    // Init GPIOs
    HAL_GPIO_WritePin(GPIOI, PAN6_CS1_Pin|PAN3_CS1_Pin|PAN4_CS1_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(PAN5_CS1_GPIO_Port, PAN5_CS1_Pin, GPIO_PIN_SET);  //PE

    HAL_GPIO_WritePin(GPIOG, PAN2_CS1_Pin |PAN1_CS1_Pin , GPIO_PIN_SET);

    GPIO_InitStruct.Pin = PAN6_CS1_Pin|PAN3_CS1_Pin|PAN4_CS1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PAN5_CS1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PAN5_CS1_GPIO_Port, &GPIO_InitStruct);    //PE

    GPIO_InitStruct.Pin = PAN2_CS1_Pin|PAN1_CS1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, PAN3_6_VGY_Pin|PAN1_4_VGY_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(PAN2_5_VGY_GPIO_Port, PAN2_5_VGY_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = PAN3_6_VGY_Pin|PAN1_4_VGY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PAN2_5_VGY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PAN2_5_VGY_GPIO_Port, &GPIO_InitStruct);  //PG

    //Inizialize GYR Sensor X
    if (ADIS16265_Init(PANEL_4) != SEN_SUCCESS)
    {
        EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_MCU_INIT, eEXEH_MCU_INIT_EXCEPTION_ID_GYRO_INIT, __LINE__);
    }

    //Inizialize GYR Sensor Y
    if (ADIS16265_Init(PANEL_5) != SEN_SUCCESS)
    {
        EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_MCU_INIT, eEXEH_MCU_INIT_EXCEPTION_ID_GYRO_INIT, __LINE__);
    }

    //Inizialize GYR Sensor Z
    if (ADIS16265_Init(PANEL_6) != SEN_SUCCESS)
    {
        EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_MCU_INIT, eEXEH_MCU_INIT_EXCEPTION_ID_GYRO_INIT, __LINE__);
    }
}

/*!
*********************************************************************************************
 * @brief DeInitialize all periphery needed for the gyroscopes
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MCU_GyrosDeInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    ADIS16265_DeInit(PANEL_4);
    ADIS16265_DeInit(PANEL_5);
    ADIS16265_DeInit(PANEL_6);

    // Init GPIOs
    HAL_GPIO_WritePin(PAN2_5_VGY_GPIO_Port, PAN2_5_VGY_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOB, PAN1_4_VGY_Pin|PAN3_6_VGY_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = PAN3_6_VGY_Pin|PAN1_4_VGY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;//GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PAN2_5_VGY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;//GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PAN2_5_VGY_GPIO_Port, &GPIO_InitStruct);  //PG

    GPIO_InitStruct.Pin = PAN6_CS1_Pin|PAN3_CS1_Pin|PAN4_CS1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PAN5_CS1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PAN5_CS1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PAN1_CS1_Pin|PAN2_CS1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    PANLE_SPI_Used &= ~PANLE_SPI_USE_GPS;
    if (PANLE_SPI_Used == 0)
    {
        //DeInit SPI2
        HAL_SPI_DeInit(&PANEL_SPI_HANDLER);
        Panels_ChipSelect2_DeInit();

        //DeInit GPIOs
        GPIO_InitStruct.Pin = PAN_MOSI_Pin | PAN_MISO_Pin | PAN_SCK_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
    }
}

/*!
*********************************************************************************************
 * @brief Initialize all periphery needed for the temperature sensors
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MCU_TemprInit(void)
{
    uint8_t txdata = 0x00;

    //Init all "chip select 2" pins (for all 6 panels)
    Panels_ChipSelect2_Init();

    //Init SPI2
    PANLE_SPI_Used |= PANLE_SPI_USE_TEMPR;
    MX_PANEL_SPI_Init(&PANEL_SPI_HANDLER);

    //Do one dummy SPI transfer (Otherwise the first temperature sensor reading is wrong)
    (void)HAL_SPI_Transmit(&PANEL_SPI_HANDLER, (uint8_t *) &txdata, 1, 10);
}

/*!
*********************************************************************************************
 * @brief Initialize all periphery needed for the temperature sensors
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MCU_TemprDeInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    //Disable the power supply of the Temperature sensors
    HAL_GPIO_WritePin(PAN_Temp_PS_GPIO_Port, PAN_Temp_PS_Pin, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = PAN_Temp_PS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PAN5_CS2_GPIO_Port, &GPIO_InitStruct);

    Panels_ChipSelect2_DeInit();

    PANLE_SPI_Used &= ~PANLE_SPI_USE_TEMPR;
    if (PANLE_SPI_Used == 0)
    {
        //DeInit SPI2
        HAL_SPI_DeInit(&PANEL_SPI_HANDLER);

        // DeInit GPIOs
        GPIO_InitStruct.Pin = PAN_SCK_Pin | PAN_MISO_Pin | PAN_MOSI_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
    }
}

/*!
*********************************************************************************************
 * @brief Initialize all periphery needed for the sun sensors
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MCU_SunSensInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIO and SPI Clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOE, OpAmp_PS_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : OpAmp_PS_Pin*/
    GPIO_InitStruct.Pin = OpAmp_PS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    //Init ADC1
    MX_ADC1_Init();
}

/*!
*********************************************************************************************
 * @brief DeInitialize all periphery needed for the sun sensors
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MCU_SunSensDeInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // Enable GPIO and SPI Clocks

    //HAL_ADC_DeInit(&hadc1);   //ADC is used for CPU temperature measurement which does not stop because it is needed for RTC calibration

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOE, OpAmp_PS_Pin, GPIO_PIN_SET);

    /*Configure GPIO pins : OpAmp_PS_Pin*/
    GPIO_InitStruct.Pin = OpAmp_PS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    // DeInit GPIOs
    GPIO_InitStruct.Pin = PAN1_IN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PAN5_IN_Pin|PAN2_IN_Pin|PAN3_IN_Pin|PAN4_IN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PAN6_IN_Pin|PAN6_IN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}


void MCU_SdioReset(uint8_t cards)
{
    if (cards & 0x01)
    {
        RCC_SDIO1_RESET();
        HAL_SD_MspDeInit(&hsd1); //__HAL_RCC_SDIO_CLK_DISABLE();
        memset((uint8_t*)&hsd1, 0, sizeof(SD_HandleTypeDef));
        memset((uint8_t*)&hdma_sdmmc1_rx, 0, sizeof(DMA_HandleTypeDef));
        memset((uint8_t*)&hdma_sdmmc1_tx, 0, sizeof(DMA_HandleTypeDef));

        MX_DMA_SDMMC1_Init();
        MX_SDMMC1_SD_Init();
        HAL_SD_Init(&hsd1);
    }

    if (cards & 0x02)
    {
        RCC_SDIO2_RESET();
        HAL_SD_MspDeInit(&hsd2); //__HAL_RCC_SDIO_CLK_DISABLE();
        memset((uint8_t*)&hsd2, 0, sizeof(SD_HandleTypeDef));
        memset((uint8_t*)&hdma_sdmmc2_rx, 0, sizeof(DMA_HandleTypeDef));
        memset((uint8_t*)&hdma_sdmmc2_tx, 0, sizeof(DMA_HandleTypeDef));

        MX_DMA_SDMMC2_Init();
        MX_SDMMC2_SD_Init();
        HAL_SD_Init(&hsd2);
    }
}

/*!
*********************************************************************************************
 * @brief Initialize all periphery needed for the SD cards
*********************************************************************************************
 * @param[input]      cards:
 *                       + 1 - SD card 1
 *                       + 2 - SD card 2
 *                       + 3 - Both SD cards 1 and 2
 * @param[output]     none
 * @return            uint8_t:
 *                      + 1 - SD card 1 Successful
 *                      + 2 - SD card 2 Successful
 *                      + 3 - SD card 1 and 2 Successful
 *                      + dpFALSE- Fail
 * @note              none
*********************************************************************************************
 */
uint8_t MCU_SD_CardInit(uint8_t cards)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    FRESULT appl_fd_result = FR_INVALID_PARAMETER;
    uint8_t retStat = pdFALSE;

    // Enable GPIO and SPI Clocks
    __HAL_RCC_GPIOD_CLK_ENABLE();

    if (cards & 0x01)
    {
        // Init GPIOs
        HAL_GPIO_WritePin(SD_EN_GPIO_Port, SD_EN_Pin, GPIO_PIN_RESET);

        /*Configure GPIO pins : PDPin PDPin */
        GPIO_InitStruct.Pin = SD_EN_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(SD_EN_GPIO_Port, &GPIO_InitStruct);
    }

    if (cards & 0x02)
    {
        HAL_GPIO_WritePin(SD2_EN_GPIO_Port, SD2_EN_Pin, GPIO_PIN_RESET);

        /*Configure GPIO pins : PDPin PDPin */
        GPIO_InitStruct.Pin = SD2_EN_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(SD2_EN_GPIO_Port, &GPIO_InitStruct);
    }

    osDelay(1); //wait 1ms after enable of the Power supply


    if (cards & 0x01)
    {
        MX_FATFS_Init(0x01);
        MX_DMA_SDMMC1_Init();
        MX_SDMMC1_SD_Init();

        if (FR_OK == (appl_fd_result = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1)))
        {
            fprintf(SYSCON, "SD card mounted successfully.\r");
            fprintf(COMM, "SD card mounted successfully.\r");

            retStat |= 0x01;
        }
        else
        {
            fprintf(SYSCON, "SD card fail err=%u.\r", appl_fd_result);
            fprintf(COMM, "SD card fail err=%u.\r", appl_fd_result);

            //retStat = pdFALSE;  //Already set
        }
    }

    if (cards & 0x02)
    {
        MX_FATFS_Init(0x02);
        MX_DMA_SDMMC2_Init();
        MX_SDMMC2_SD_Init();
        if (FR_OK == (appl_fd_result = f_mount(&SD2FatFS, (TCHAR const*)SD2Path, 1)))
        {
            fprintf(SYSCON, "SD2 card mounted successfully.\r");
            fprintf(COMM, "SD2 card mounted successfully.\r");

            retStat |= 0x02;
        }
        else
        {
            fprintf(SYSCON, "SD2 card fail err=%u.\r", appl_fd_result);
            fprintf(COMM, "SD2 card fail err=%u.\r", appl_fd_result);

            //retStat = pdFALSE;  //Already set
        }
    }

    return retStat;
}

/*!
*********************************************************************************************
 * @brief DeInitialize all periphery needed for the SD cards
*********************************************************************************************
 * @param[input]      Sd_DetectDeInitEnable: 1 - deinitialize the detection pins, 0 - does not deinitialze it (used when the SD cards is just reset, this way some interrupts are skipped)
 * @param[input]      cards:
 *                       + 1 - SD card 1
 *                       + 2 - SD card 2
 *                       + 3 - Both SD cards 1 and 2
 * @param[input]      cardHwReset:
 *                       + 0 - reset only the SDIO and FAT file system on the MCU
 *                       + 1 - do a HW reset of the (power off, delay, power on) SD card 1
 *                       + 2 - do a HW reset of the (power off, delay, power on) SD card 2
 *                       + 3 - do a HW both SD cards 1 and 2
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void MCU_SD_CardDeInit(uint8_t Sd_DetectDeInitEnable, uint8_t cards, uint8_t cardHwReset)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (cards & 0x01)
    {
        // Enable GPIO and SPI Clocks
        if (cardHwReset & 0x01)
        {
            HAL_GPIO_WritePin(SD_EN_GPIO_Port, SD_EN_Pin, GPIO_PIN_SET);
        }
        RCC_SDIO1_RESET();
        HAL_SD_MspDeInit(&hsd1); //__HAL_RCC_SDIO_CLK_DISABLE();
        memset((uint8_t*)&hsd1, 0, sizeof(SD_HandleTypeDef));
        memset((uint8_t*)&hdma_sdmmc1_rx, 0, sizeof(DMA_HandleTypeDef));
        memset((uint8_t*)&hdma_sdmmc1_tx, 0, sizeof(DMA_HandleTypeDef));
        (void)f_mount(0, SDPath, 0);         //unmount SD1
        (void)FATFS_UnLinkDriver(SDPath);
        memset((uint8_t*)&SDFatFS, 0, sizeof(FATFS));
        // Init GPIOs for SD card 1
        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    }

    if (cards & 0x02)
    {
        if (cardHwReset & 0x02)
        {
            HAL_GPIO_WritePin(SD2_EN_GPIO_Port, SD2_EN_Pin, GPIO_PIN_SET);
        }
        RCC_SDIO2_RESET();
        HAL_SD_MspDeInit(&hsd2); //__HAL_RCC_SDIO_CLK_DISABLE();
        memset((uint8_t*)&hsd2, 0, sizeof(SD_HandleTypeDef));
        memset((uint8_t*)&hdma_sdmmc2_rx, 0, sizeof(DMA_HandleTypeDef));
        memset((uint8_t*)&hdma_sdmmc2_tx, 0, sizeof(DMA_HandleTypeDef));
        (void)f_mount(0, SD2Path, 0);       //unmount SD2
        (void)FATFS_UnLinkDriver(SD2Path);
        memset((uint8_t*)&SD2FatFS, 0, sizeof(FATFS));
        // Init GPIOs for SD card 2
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
    }

    if ((cards & 0x03) == 0x03)
    {
        memset((uint8_t*)&disk, 0, sizeof(Disk_drvTypeDef));
        RCC_DMA2_RESET();
    }

    if (Sd_DetectDeInitEnable > 0)
    {
        if (cards & 0x01)
        {
            GPIO_InitStruct.Pin = SD_DET_Pin;
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(SD_DET_GPIO_Port, &GPIO_InitStruct);
        }

        if (cards & 0x02)
        {
            GPIO_InitStruct.Pin = SD2_DET_Pin;
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_Init(SD2_DET_GPIO_Port, &GPIO_InitStruct);
        }
    }
}


/*!
*********************************************************************************************
* @brief Reset SD card and initialise all components needed to use it
*********************************************************************************************
 * @param[input]      cards:
 *                       + 1 - SD card 1
 *                       + 2 - SD card 2
 *                       + 3 - Both SD cards 1 and 2
 * @param[input]      cardHwReset:
 *                       + pdTRUE - do a HW reset of the SD card (power off, delay, power on)
 *                       + pdFALSE - reset only the SDIO and FAT file system on the MCU
* @param[output]     none
 * @return            uint8_t:
 *                      + 1 - SD card 1 Successful
 *                      + 2 - SD card 2 Successful
 *                      + 3 - SD card 1 and 2 Successful
 *                      + dpFALSE- Fail
* @note              none
*********************************************************************************************
*/
uint8_t MCU_InitSdCard(uint8_t cards, uint8_t cardHwReset)
{

/////////////////////// RESET ALL PERIPHERY AND STRUCTURES /////////////////////////
    MCU_SD_CardDeInit(DISABLE, cards, cardHwReset);

/////////////////////// INITIALISE ALL PERIPHERY AND STRUCTURES /////////////////////////
    if(cardHwReset == pdTRUE){
        sysDelay(250);   /* Wait until the capacitors discharge to 0V - Minimum time 80 ms */
    }

    return MCU_SD_CardInit(cards);
}

void MCU_ExtFlashInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_GPIO_WritePin(SPI4_FRAM_CHIP_SELECT_GPIO_Port, SPI4_FRAM_CHIP_SELECT_Pin, GPIO_PIN_SET);

    /*Configure GPIO pins : SPI4_FRAM_CHIP_SELECT_Pin */
    GPIO_InitStruct.Pin = SPI4_FRAM_CHIP_SELECT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SPI4_FRAM_CHIP_SELECT_GPIO_Port, &GPIO_InitStruct);

    MX_SPI4_Init();
}

void MCU_ExtFlashDeInit(void)
{
    HAL_SPI_DeInit(&hspi4);

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_GPIO_WritePin(SPI4_FRAM_CHIP_SELECT_GPIO_Port, SPI4_FRAM_CHIP_SELECT_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : SPI4_FRAM_CHIP_SELECT_Pin */
    GPIO_InitStruct.Pin = SPI4_FRAM_CHIP_SELECT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SPI4_FRAM_CHIP_SELECT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PE12 PE13 PE14 PE15 */
    GPIO_InitStruct.Pin = SPI4_FRAM_SCK_Pin|SPI4_FRAM_MISO_Pin|SPI4_FRAM_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

void MCU_Init_I2cPullUps(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /*Configure GPIO pins : SYS_I2C_4K7_Pin */
    GPIO_InitStruct.Pin = SYS_I2C_4K7_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SYS_I2C_4K7_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : SYS_I2C_10K_Pin PAY_I2C_4K7_Pin PAY_I2C_10K_Pin */
    GPIO_InitStruct.Pin = SYS_I2C_10K_Pin|PAY_I2C_4K7_Pin|PAY_I2C_10K_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /*Configure GPIO pin Output Levels */

    const uint8_t current_system_i2c_pullup_val = EEPROM_emul_DataTemp.I2cPullUpResistors[MCU_INIT_I2C_PULL_UP_SYS];

    // 4.7k ohms
    if (current_system_i2c_pullup_val == 0x01)
    {
        HAL_GPIO_WritePin(SYS_I2C_4K7_GPIO_Port, SYS_I2C_4K7_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOG,                 SYS_I2C_10K_Pin, GPIO_PIN_SET);
    }
    // 10k ohms
    else if (current_system_i2c_pullup_val== 0x02)
    {
        HAL_GPIO_WritePin(SYS_I2C_4K7_GPIO_Port, SYS_I2C_4K7_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOG,                 SYS_I2C_10K_Pin, GPIO_PIN_RESET);
    }
    // ~3.2k ohms
    else if (current_system_i2c_pullup_val == 0x03)
    {
        HAL_GPIO_WritePin(SYS_I2C_4K7_GPIO_Port, SYS_I2C_4K7_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOG,                 SYS_I2C_10K_Pin, GPIO_PIN_RESET);
    }
    // high Z
    else
    {
        HAL_GPIO_WritePin(SYS_I2C_4K7_GPIO_Port, SYS_I2C_4K7_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOG,                 SYS_I2C_10K_Pin, GPIO_PIN_SET);
    }

    const uint8_t current_payload_i2c_pullup_val = EEPROM_emul_DataTemp.I2cPullUpResistors[MCU_INIT_I2C_PULL_UP_PAY];

    // 4.7k ohms
    if (current_payload_i2c_pullup_val == 0x01)
    {
        HAL_GPIO_WritePin(GPIOG, PAY_I2C_4K7_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOG, PAY_I2C_10K_Pin, GPIO_PIN_SET);
    }
    // 10k ohms
    else if (current_payload_i2c_pullup_val == 0x02)
    {
        HAL_GPIO_WritePin(GPIOG, PAY_I2C_4K7_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOG, PAY_I2C_10K_Pin, GPIO_PIN_RESET);
    }
    // ~3.2k ohms
    else if (current_payload_i2c_pullup_val == 0x03)
    {
        HAL_GPIO_WritePin(GPIOG, PAY_I2C_4K7_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOG, PAY_I2C_10K_Pin, GPIO_PIN_RESET);
    }
    // high Z
    else
    {
        HAL_GPIO_WritePin(GPIOG, PAY_I2C_4K7_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOG, PAY_I2C_10K_Pin, GPIO_PIN_SET);
    }
}

void MX_RTC_ReloadShadowRegs(void)
{
    // first read just copy from data to shadow register. The second read, gets the real value from the shadow registers
    __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
    LL_RTC_ClearFlag_RS(RTC);//
    HAL_RTC_WaitForSynchro(&hrtc);
    HAL_RTC_GetTime(&hrtc, &sTime, CALENDAR_FORMAT);
    HAL_RTC_GetDate(&hrtc, &sDate, CALENDAR_FORMAT);
    LL_RTC_ClearFlag_RS(RTC);//
    HAL_RTC_WaitForSynchro(&hrtc);
}

/*
*********************************************************************************************
* INTERNAL (STATIC) ROUTINES DEFINITION
*********************************************************************************************
*/
/*!
*********************************************************************************************
 * @brief Reset the SDIO controller
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
static void RCC_SDIO1_RESET(void)
{
    __HAL_RCC_SDMMC1_FORCE_RESET(); //RCC->APB2RSTR |= RCC_APB2RSTR_SDIORST;
    while ((RCC->APB2RSTR & RCC_APB2RSTR_SDMMC1RST) != RCC_APB2RSTR_SDMMC1RST);
    osDelay(1);
    __HAL_RCC_SDMMC1_RELEASE_RESET(); //RCC->APB2RSTR &= ~(RCC_APB2RSTR_SDIORST);
    while ((RCC->APB2RSTR & RCC_APB2RSTR_SDMMC1RST) == RCC_APB2RSTR_SDMMC1RST);
}

/*!
*********************************************************************************************
 * @brief Reset the SDIO controller
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
static void RCC_SDIO2_RESET(void)
{
    __HAL_RCC_SDMMC2_FORCE_RESET(); //RCC->APB2RSTR |= RCC_APB2RSTR_SDIORST;
    while ((RCC->APB2RSTR & RCC_APB2RSTR_SDMMC2RST) != RCC_APB2RSTR_SDMMC2RST);
    osDelay(1);
    __HAL_RCC_SDMMC2_RELEASE_RESET(); //RCC->APB2RSTR &= ~(RCC_APB2RSTR_SDIORST);
    while ((RCC->APB2RSTR & RCC_APB2RSTR_SDMMC2RST) == RCC_APB2RSTR_SDMMC2RST);
}

/*!
*********************************************************************************************
 * @brief Reset the DMA2 controller
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
static void RCC_DMA2_RESET(void)
{
    HAL_NVIC_DisableIRQ(DMA2_Stream3_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Stream6_IRQn);

    HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Stream5_IRQn);

    __HAL_RCC_DMA2_FORCE_RESET();
    osDelay(1);
    __HAL_RCC_DMA2_RELEASE_RESET();
}

//////////////
// Programmable voltage detector (PVD) stuff
void PVD_BrownoutReset_Init()
{
    __HAL_RCC_PWR_CLK_ENABLE();   // Enable Power Clock
    //Configure the NVIC for PVD
    HAL_NVIC_SetPriority(PVD_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(PVD_IRQn);
    // Configure the PVD Level to 7 (3.03V-3.14V)
    PWR_PVDTypeDef sConfigPVD;
    sConfigPVD.PVDLevel = PWR_PVDLEVEL_7;
    sConfigPVD.Mode = PWR_MODE_IT_RISING_FALLING;
    HAL_PWR_PVDConfig(&sConfigPVD);
    // Enable the PVD Output
    HAL_PWR_EnablePVD();

    // Brownout reset enable
    FLASH_OBProgramInitTypeDef sOBProgramInitTypeDef;
    HAL_FLASHEx_OBGetConfig(&sOBProgramInitTypeDef); // Get BOR Option Bytes
    if ((sOBProgramInitTypeDef.BORLevel & 0x0F) != OB_BOR_LEVEL3)
    {
        HAL_FLASH_OB_Unlock(); // Unlocks the option bytes block access
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|FLASH_FLAG_PGAERR|FLASH_FLAG_PGPERR|FLASH_FLAG_ERSERR|FLASH_FLAG_BSY); // Clears the FLASH pending flags
        memset(&sOBProgramInitTypeDef, 0, sizeof(FLASH_OBProgramInitTypeDef));
        sOBProgramInitTypeDef.OptionType = OPTIONBYTE_BOR;
        sOBProgramInitTypeDef.BORLevel = OB_BOR_LEVEL3; // Select the desired V(BOR) Level
        HAL_FLASHEx_OBProgram(&sOBProgramInitTypeDef);
        HAL_FLASH_OB_Launch(); // Launch the option byte loading
    }
}

//When the voltage rail powering the MCU goes below 2.9V, the PVD interrupt is triggered causing it to call the IRQ handler below:
void HAL_PWR_PVDCallback(void)
{
    //Write_EEPROM();
    if (READ_BIT(PWR->CSR1, PWR_CSR1_PVDO))
    {
        // VDD is lower than the PVD threshold selected with the PLS[2:0] bits
        DBG_SYSCON("\r\n<%u> --- VDD is low ---\r\n", (unsigned int )GetObcUptimeSeconds());
    }
    else
    {
        //  VDD is higher than the PVD threshold selected with the PLS[2:0] bits
        DBG_SYSCON("\r\n<%u> --- VDD is normal ---\r\n", (unsigned int )GetObcUptimeSeconds());
    }
    // Note: The PVD is stopped by Standby mode. For this reason, this bit is equal to 0 after
    // Standby or reset until the PVDE bit is set.
}

bool MCU_Init_IsOBConfigValid(void)
{
    FLASH_OBProgramInitTypeDef sOBProgramInitTypeDef;

    HAL_FLASHEx_OBGetConfig(&sOBProgramInitTypeDef);

    return ((sOBProgramInitTypeDef.USERConfig & OB_STOP_NO_RST) == OB_STOP_NO_RST) &&
           ((sOBProgramInitTypeDef.USERConfig & OB_STDBY_NO_RST) == OB_STDBY_NO_RST) &&
           ((sOBProgramInitTypeDef.USERConfig & OB_IWDG_SW) == OB_IWDG_SW) &&
           ((sOBProgramInitTypeDef.USERConfig & OB_WWDG_SW) == OB_WWDG_SW) &&
           ((sOBProgramInitTypeDef.USERConfig & OB_DUAL_BOOT_DISABLE) == OB_DUAL_BOOT_DISABLE) &&
           ((sOBProgramInitTypeDef.USERConfig & OB_IWDG_STOP_ACTIVE) != OB_IWDG_STOP_ACTIVE) &&
           ((sOBProgramInitTypeDef.USERConfig & OB_IWDG_STDBY_ACTIVE) != OB_IWDG_STDBY_ACTIVE) &&
#if (ENABLE_DUAL_BANK == 1)
           ((sOBProgramInitTypeDef.USERConfig & OB_NDBANK_SINGLE_BANK) != OB_NDBANK_SINGLE_BANK);
#else
           ((sOBProgramInitTypeDef.USERConfig & OB_NDBANK_SINGLE_BANK) == OB_NDBANK_SINGLE_BANK);
#endif
}

bool MCU_Init_SetDefaultOBConfig(void)
{
    FLASH_OBProgramInitTypeDef sOBProgramInitTypeDef;

    // load current configuration...
    HAL_FLASHEx_OBGetConfig(&sOBProgramInitTypeDef);

    //...and modify only what needs to be set, leave the rest intact
    sOBProgramInitTypeDef.OptionType = OPTIONBYTE_USER;
    sOBProgramInitTypeDef.USERConfig |= OB_STOP_NO_RST;
    sOBProgramInitTypeDef.USERConfig |= OB_STDBY_NO_RST;
    sOBProgramInitTypeDef.USERConfig |= OB_IWDG_SW;
    sOBProgramInitTypeDef.USERConfig |= OB_WWDG_SW;
    sOBProgramInitTypeDef.USERConfig |= OB_DUAL_BOOT_DISABLE;
    sOBProgramInitTypeDef.USERConfig &= ~OB_IWDG_STOP_ACTIVE;
    sOBProgramInitTypeDef.USERConfig &= ~OB_IWDG_STDBY_ACTIVE;
    sOBProgramInitTypeDef.USERConfig &= ~OB_NDBANK_SINGLE_BANK;

    HAL_StatusTypeDef flashRes = HAL_FLASH_OB_Unlock();

    if (flashRes == HAL_OK)
    {
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|FLASH_FLAG_PGAERR|FLASH_FLAG_PGPERR|FLASH_FLAG_ERSERR|FLASH_FLAG_BSY); // Clears the FLASH pending flags

        flashRes = HAL_FLASHEx_OBProgram(&sOBProgramInitTypeDef);

        if (flashRes == HAL_OK)
            flashRes = HAL_FLASH_OB_Launch();
    }

    return flashRes;
}

/* **************************************************************************************** */
