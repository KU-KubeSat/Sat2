/*!
********************************************************************************************
* @file MCU_Init.h
* @brief Header of MCU_Init.c
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
#ifndef MCU_INIT_H
#define MCU_INIT_H

/*
*********************************************************************************************
* INCLUDES
*********************************************************************************************
*/
#include "stm32f7xx_hal.h"
#include <GlobalConfig.h>
#include <stdbool.h>

/*
*********************************************************************************************
* EXTERNAL DEFINES
*********************************************************************************************
*/
//UART5 Init definitions -> SYS UART / USB / ADCS
#define U5_PROTO_USB_SYSTEM_RX_Pin GPIO_PIN_5
#define U5_PROTO_USB_SYSTEM_RX_GPIO_Port GPIOB
#define U5_PROTO_USB_SYSTEM_TX_Pin GPIO_PIN_6
#define U5_PROTO_USB_SYSTEM_TX_GPIO_Port GPIOB
//End of UART5 Init definitions
#define En_GNSS_Pin GPIO_PIN_11
#define En_GNSS_GPIO_Port GPIOF

//UART8 Init definitions -> COMM UART / Ext EPS
#define U8_COMM_TX_Pin GPIO_PIN_1
#define U8_COMM_TX_GPIO_Port GPIOE
#define U8_COMM_RX_Pin GPIO_PIN_0
#define U8_COMM_RX_GPIO_Port GPIOE

#define U8_COMM_CTS_Pin GPIO_PIN_14
#define U8_COMM_CTS_GPIO_Port GPIOD
#define U8_COMM_RTS_Pin GPIO_PIN_15
#define U8_COMM_RTS_GPIO_Port GPIOD
//End of UART8 Init definitions

//UART3 Init definitions -> SYS1 ES MAC RS485 / S/X-Band
#define U3_RS485_S1_nRE_Pin GPIO_PIN_6
#define U3_RS485_S1_nRE_GPIO_Port GPIOE
#define U3_RS485_S1_DE_Pin GPIO_PIN_10
#define U3_RS485_S1_DE_GPIO_Port GPIOD
#define U3_RS485_S1_TE_Pin GPIO_PIN_8
#define U3_RS485_S1_TE_GPIO_Port GPIOI

#define U3_RS485_S1_RX_Pin GPIO_PIN_9
#define U3_RS485_S1_RX_GPIO_Port GPIOD
#define U3_RS485_S1_TX_Pin GPIO_PIN_10
#define U3_RS485_S1_TX_GPIO_Port GPIOB
//End of UART3 Init definitions

//UART4 Init definitions -> PAY2 ES MAC RS485 / ESTTC uart
#define U4_RS485_S2_DE_Pin GPIO_PIN_13
#define U4_RS485_S2_DE_GPIO_Port GPIOF
#define U4_RS485_S2_nRE_Pin GPIO_PIN_14
#define U4_RS485_S2_nRE_GPIO_Port GPIOF
#define U4_RS485_S2_RX_Pin GPIO_PIN_9
#define U4_RS485_S2_RX_GPIO_Port GPIOI
#define U4_RS485_S2_TX_Pin GPIO_PIN_13
#define U4_RS485_S2_TX_GPIO_Port GPIOH
#define U4_RS485_S2_TE_Pin GPIO_PIN_3
#define U4_RS485_S2_TE_GPIO_Port GPIOE
//End of UART4 Init definitions

//UART7 Init definitions -> Payload ESTTC uart / GNSS
#define U7_PAY_RX_SPI5_NSS_Pin GPIO_PIN_6
#define U7_PAY_RX_SPI5_NSS_GPIO_Port GPIOF
#define U7_PAY_TX_SPI5_SCK_Pin GPIO_PIN_7
#define U7_PAY_TX_SPI5_SCK_GPIO_Port GPIOF
#define U7_PAY_RTS_SPI5_MISO_Pin GPIO_PIN_8
#define U7_PAY_RTS_SPI5_MISO_GPIO_Port GPIOF
#define U7_PAY_CTS_SPI5_MOSI_Pin GPIO_PIN_9
#define U7_PAY_CTS_SPI5_MOSI_GPIO_Port GPIOF
//End of UART7 Init definitions

//UART1 Init definitions -> PAY1 ES MAC RS485
#define RS422_HS1_DE_UART1_SYS1_Pin             GPIO_PIN_10
#define RS422_HS1_DE_UART1_SYS1_GPIO_Port       GPIOE
#define RS422_HS1_nRE_UART1_SYS1_Pin            GPIO_PIN_5
#define RS422_HS1_nRE_UART1_SYS1_GPIO_Port      GPIOF
//End of UART1 Init definitions

//UART6 Init definitions -> PAY1 ES MAC RS485
#define RS422_HS2_DE_UART6_PAY1_Pin GPIO_PIN_10
#define RS422_HS2_DE_UART6_PAY1_GPIO_Port GPIOF
#define RS422_HS2_nRE_UART6_PAY1_Pin GPIO_PIN_12
#define RS422_HS2_nRE_UART6_PAY1_GPIO_Port GPIOF
//End of UART6 Init definitions

//GPIO Init definitions
#define LED_G_Pin GPIO_PIN_2
#define LED_G_GPIO_Port GPIOB

#define OBC_OUT5_Pin GPIO_PIN_6
#define OBC_OUT5_GPIO_Port GPIOG

#define OBC_OUT4_6_Pin GPIO_PIN_0
#define OBC_OUT4_6_GPIO_Port GPIOD
#define OBC_OUT3_Pin GPIO_PIN_4
#define OBC_OUT3_GPIO_Port GPIOD
#define OBC_OUT2_Pin GPIO_PIN_5
#define OBC_OUT2_GPIO_Port GPIOD
#define OBC_OUT1_Pin GPIO_PIN_15
#define OBC_OUT1_GPIO_Port GPIOB
//End of GPIO Init definitions

//I2C1 Init definitions -> SYS I2C
#define I2C1_SYS_SDA_Pin GPIO_PIN_7
#define I2C1_SYS_SDA_GPIO_Port GPIOB
#define I2C1_SYS_SCL_Pin GPIO_PIN_8
#define I2C1_SYS_SCL_GPIO_Port GPIOB
//End of I2C1 Init definitions

//I2C2 Init definitions -> Sesors I2C
#define I2C2_SEN_SCL_Pin GPIO_PIN_1
#define I2C2_SEN_SCL_GPIO_Port GPIOF
#define I2C2_SEN_SDA_Pin GPIO_PIN_0
#define I2C2_SEN_SDA_GPIO_Port GPIOF
//End of I2C2 Init definitions

//I2C3 Init definitions
#define I2C3_PAY_SCL_Pin GPIO_PIN_7
#define I2C3_PAY_SCL_GPIO_Port GPIOH
#define I2C3_PAY_SDA_Pin GPIO_PIN_8
#define I2C3_PAY_SDA_GPIO_Port GPIOH
//End of I2C3 Init definitions

//SPI5 Init definitions
#define SPI5_SCK_Pin U7_PAY_TX_SPI5_SCK_Pin
#define SPI5_SCK_GPIO_Port U7_PAY_TX_SPI5_SCK_GPIO_Port
#define SPI5_MISO_Pin U7_PAY_RTS_SPI5_MISO_Pin
#define SPI5_MISO_GPIO_Port U7_PAY_RTS_SPI5_MISO_GPIO_Port
#define SPI5_MOSI_Pin U7_PAY_CTS_SPI5_MOSI_Pin
#define SPI5_MOSI_GPIO_Port U7_PAY_CTS_SPI5_MOSI_GPIO_Port
#define SPI5_IRQ_Pin RS422_HS2_DE_UART6_PAY1_Pin
#define SPI5_IRQ_GPIO_Port RS422_HS2_DE_UART6_PAY1_GPIO_Port

#define ARDU_CAM_CS_Pin U7_PAY_RX_SPI5_NSS_Pin
#define ARDU_CAM_CS_GPIO_Port U7_PAY_RX_SPI5_NSS_GPIO_Port
//End of SPI5 Init definitions

//Accelerometers Init definitions
#define ACC_EN_Pin GPIO_PIN_15
#define ACC_EN_GPIO_Port GPIOE
#define SENSORS_I2C_USE_ACCEL   (uint8_t)(1<<0)
#define SENSORS_I2C_USE_COMPAS  (uint8_t)(1<<1)
//End of Accelerometers Init definitions

//Magnetorqers Init definitions
#define PAN_PS_Pin GPIO_PIN_7
#define PAN_PS_GPIO_Port GPIOE

#define PAN4_DIR1_Pin GPIO_PIN_11
#define PAN4_DIR1_GPIO_Port GPIOD
#define PAN5_DIR1_Pin GPIO_PIN_3
#define PAN5_DIR1_GPIO_Port GPIOF
#define PAN6_DIR1_Pin GPIO_PIN_4
#define PAN6_DIR1_GPIO_Port GPIOH

#define OBC_OUT7_Pin GPIO_PIN_5
#define OBC_OUT7_GPIO_Port GPIOH
#define OBC_OUT8_Pin GPIO_PIN_15
#define OBC_OUT8_GPIO_Port GPIOH
#define LED_Y_Pin GPIO_PIN_4
#define LED_Y_GPIO_Port GPIOF

#define PAN4_PWM_Pin GPIO_PIN_10
#define PAN4_PWM_GPIO_Port GPIOH
#define PAN5_PWM_Pin GPIO_PIN_11
#define PAN5_PWM_GPIO_Port GPIOH
#define PAN6_PWM_Pin GPIO_PIN_12
#define PAN6_PWM_GPIO_Port GPIOH

#define PAN4_PWM_Pin GPIO_PIN_10
#define PAN4_PWM_GPIO_Port GPIOH
#define PAN5_PWM_Pin GPIO_PIN_11
#define PAN5_PWM_GPIO_Port GPIOH
#define PAN6_PWM_Pin GPIO_PIN_12
#define PAN6_PWM_GPIO_Port GPIOH

//End of Magnetorqers Init definitions

//Gyroscopes Init definitions
#define PANLES_SPI      SPI2

#define PAN1_CS1_Pin GPIO_PIN_3
#define PAN1_CS1_GPIO_Port GPIOG
#define PAN2_CS1_Pin GPIO_PIN_5
#define PAN2_CS1_GPIO_Port GPIOG
#define PAN3_CS1_Pin GPIO_PIN_5
#define PAN3_CS1_GPIO_Port GPIOI
#define PAN4_CS1_Pin GPIO_PIN_7
#define PAN4_CS1_GPIO_Port GPIOI
#define PAN5_CS1_Pin GPIO_PIN_5
#define PAN5_CS1_GPIO_Port GPIOE
#define PAN6_CS1_Pin GPIO_PIN_11
#define PAN6_CS1_GPIO_Port GPIOI

#define PAN1_4_VGY_Pin GPIO_PIN_14
#define PAN1_4_VGY_GPIO_Port GPIOB
#define PAN2_5_VGY_Pin GPIO_PIN_14
#define PAN2_5_VGY_GPIO_Port GPIOG
#define PAN3_6_VGY_Pin GPIO_PIN_9
#define PAN3_6_VGY_GPIO_Port GPIOB

//SPI2 pins
#define PAN_SCK_Pin GPIO_PIN_1
#define PAN_SCK_GPIO_Port GPIOI
#define PAN_MISO_Pin GPIO_PIN_2
#define PAN_MISO_GPIO_Port GPIOI
#define PAN_MOSI_Pin GPIO_PIN_3
#define PAN_MOSI_GPIO_Port GPIOI

#define PANLE_SPI_USE_GPS   (uint8_t)(1<<0)
#define PANLE_SPI_USE_TEMPR  (uint8_t)(1<<1)
//End of Gyroscopes Init definitions

//Temperature sensors Init definitions
#define PAN_Temp_PS_Pin GPIO_PIN_8
#define PAN_Temp_PS_GPIO_Port GPIOE

#define PAN1_CS2_Pin GPIO_PIN_0
#define PAN1_CS2_GPIO_Port GPIOI
#define PAN2_CS2_Pin GPIO_PIN_4
#define PAN2_CS2_GPIO_Port GPIOG
#define PAN3_CS2_Pin GPIO_PIN_4
#define PAN3_CS2_GPIO_Port GPIOI
#define PAN4_CS2_Pin GPIO_PIN_6
#define PAN4_CS2_GPIO_Port GPIOI
#define PAN5_CS2_Pin GPIO_PIN_4
#define PAN5_CS2_GPIO_Port GPIOE
#define PAN6_CS2_Pin GPIO_PIN_2
#define PAN6_CS2_GPIO_Port GPIOF
//End of Temperature sensors Init definitions

//Sun sensors Init definitions
#define PAN1_IN_Pin GPIO_PIN_0
#define PAN1_IN_GPIO_Port GPIOC
#define PAN2_IN_Pin GPIO_PIN_3
#define PAN2_IN_GPIO_Port GPIOA
#define PAN3_IN_Pin GPIO_PIN_4
#define PAN3_IN_GPIO_Port GPIOA
#define PAN4_IN_Pin GPIO_PIN_5
#define PAN4_IN_GPIO_Port GPIOA
#define PAN5_IN_Pin GPIO_PIN_6
#define PAN5_IN_GPIO_Port GPIOA
#define PAN6_IN_Pin GPIO_PIN_0
#define PAN6_IN_GPIO_Port GPIOB
//End of Sun sensors Init definitions


#define OpAmp_PS_Pin GPIO_PIN_9
#define OpAmp_PS_GPIO_Port GPIOE


//SD card Init definitions
#define SD_EN_Pin GPIO_PIN_3
#define SD_EN_GPIO_Port GPIOD
#define SD2_EN_Pin GPIO_PIN_8
#define SD2_EN_GPIO_Port GPIOD
#define SD_DET_Pin GPIO_PIN_15
#define SD_DET_GPIO_Port GPIOG
#define SD2_DET_Pin GPIO_PIN_13
#define SD2_DET_GPIO_Port GPIOC
//End of SD card Init definitions

#define ETH_RSTn_Pin GPIO_PIN_2
#define ETH_RSTn_GPIO_Port GPIOH
#define ETH_EN_Pin GPIO_PIN_3
#define ETH_EN_GPIO_Port GPIOH

#define SYS_I2C_4K7_Pin GPIO_PIN_15
#define SYS_I2C_4K7_GPIO_Port GPIOF
#define SYS_I2C_10K_Pin GPIO_PIN_0
#define SYS_I2C_10K_GPIO_Port GPIOG
#define PAY_I2C_4K7_Pin GPIO_PIN_1
#define PAY_I2C_4K7_GPIO_Port GPIOG
#define PAY_I2C_10K_Pin GPIO_PIN_2
#define PAY_I2C_10K_GPIO_Port GPIOG

#define SPI4_FRAM_CHIP_SELECT_Pin GPIO_PIN_11
#define SPI4_FRAM_CHIP_SELECT_GPIO_Port GPIOE
#define SPI4_FRAM_SCK_Pin GPIO_PIN_12
#define SPI4_FRAM_SCK_GPIO_Port GPIOE
#define SPI4_FRAM_MISO_Pin GPIO_PIN_13
#define SPI4_FRAM_MISO_GPIO_Port GPIOE
#define SPI4_FRAM_MOSI_Pin GPIO_PIN_14
#define SPI4_FRAM_MOSI_GPIO_Port GPIOE

#define EN_CAM_Pin GPIO_PIN_9
#define EN_CAM_GPIO_Port GPIOH

#define CAN_RS_Pin GPIO_PIN_8
#define CAN_RS_GPIO_Port GPIOA

#define EPS_RST_Pin GPIO_PIN_7          //VMI_TODO -> use that pin
#define EPS_RST_GPIO_Port GPIOG

#define SYS_WAKE1_Pin GPIO_PIN_0
#define SYS_WAKE1_GPIO_Port GPIOA


#define MCU_WWDG_MAX_RESET_TIME     (82)                            /*  112 = 21ms min time ;  127 = 83ms max time */
#define MCU_WWDG_MAX_CYCLING_TIME   (MCU_WWDG_MAX_RESET_TIME/2)     // Maximum time to wait in a while() loop

//////////////////////////////////////////////////////////////////////////////////////////
////  New define /////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
#define SYSTEM_I2C_HANDLER      (hi2c1)
#define SYSTEM_I2C              (I2C1)

#define SENSORS_I2C_HANDLER      (hi2c2)
#define SENSORS_I2C              (I2C2)

#define PAYLOAD_I2C_HANDLER      (hi2c3)
#define PAYLOAD_I2C              (I2C3)

#define PANEL_SPI_HANDLER       (hspi2)
#define PANEL_SPI               (SPI2)

typedef struct
{
    uint8_t OutToChange;    //Allowed outputs to be changed: bit-field
    uint8_t OutState;       //States into which to be changed: bit-field
} MCU_Init_OutStates_struct;

typedef enum
{
    MCU_INIT_OUT1,    // S/X-band
    MCU_INIT_OUT2,    // S/X-band
    MCU_INIT_OUT3,    // VHF
    MCU_INIT_OUT5,    // UHF
    MCU_INIT_OUT4_6,  // OBC
    MCU_INIT_OUT7,    //
    MCU_INIT_OUT8,    //
    MCU_INIT_OUT_MAX  //
} MCU_Init_OutputsOrder_enum;

typedef enum
{
    MCU_INIT_I2C_PULL_UP_SYS,
    MCU_INIT_I2C_PULL_UP_PAY,

    MCU_INIT_I2C_MAX_NUMBER
} MCU_Init_I2C_pullUps_enum;

/*
*********************************************************************************************
* EXTERNAL TYPES DECLARATIONS
*********************************************************************************************
*/
/* No External types declarations */

/*
*********************************************************************************************
* EXTERNAL VARIABLES DECLARATIONS
*********************************************************************************************
*/
extern CAN_HandleTypeDef hcan1;
extern ETH_HandleTypeDef heth;

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;

extern SD_HandleTypeDef  hsd1;
extern SD_HandleTypeDef  hsd2;

extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi5;
extern SPI_HandleTypeDef hspi4;
extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart3;

extern WWDG_HandleTypeDef hwwdg;
extern IWDG_HandleTypeDef hiwdg;

//extern SRAM_HandleTypeDef hsram1;

extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;

extern DMA_HandleTypeDef hdma_sdmmc1_rx;
extern DMA_HandleTypeDef hdma_sdmmc1_tx;
extern DMA_HandleTypeDef hdma_sdmmc2_rx;
extern DMA_HandleTypeDef hdma_sdmmc2_tx;

extern DMA_HandleTypeDef hdma_usart3_rx;

extern uint32_t USART_rx_data_dummy;
/*
*********************************************************************************************
* EXTERNAL ROUTINES DECLARATIONS 
*********************************************************************************************
*/
void SystemClock_MaxConfig(void);
void SystemClock_IdleConfig(void);

void MX_GPIO_Init(void);
void MX_AllPeripheryAndGpio_DeInit(void);
void MX_GPIO_Outputs_Set(MCU_Init_OutStates_struct *states);
void MX_GPIO_Outputs_DeInit(void);
void MX_DMA_Init(void);

//void MX_DMA_USART7_Init(void);
//void MX_DMA_SDIO_Init(void);
void MX_DMA_USART3_Init(void);
void MX_DMA_SDMMC1_Init(void);
void MX_DMA_SDMMC2_Init(void);

void MX_ADC1_Init(void);
//void MX_SPI1_Init(void);
//void MX_SPI1_DeInit(void);
void MX_SPI5_Init(void);
void MX_SPI5_DeInit(void);

void MX_PANEL_SPI_Init(SPI_HandleTypeDef *hspi);
void MX_SPI6_Init(void);
void MX_TIM5_Init(void);

//ESTTC
void MX_UART5_Init(void);
void MX_UART5_DeInit(void);
void MX_UART8_Init(void);
void MX_UART8_DeInit(void);
void MX_UART4_Init(void);
void MX_UART4_DeInit(void);
void MX_USART4_UART_Init(void);
void MX_USART4_UART_DeInit(void);

//ES MAC
void MX_UART7_Init(void);
void MX_UART7_DeInit(void);
void MX_USART3_UART_Init(uint32_t BaudRate);
void MX_USART3_UART_DeInit(void);

void MX_RTC_Init(void);

void MX_WWDG_Init(void);
void MX_IWDG_Init(void);

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c);

void MCU_MagnetometersInit(void);
void MCU_MagnetometersDeInit(void);
void MCU_MagnetorqersInit(void);
void MCU_MagnetorqersDeInit(void);
void MCU_GyrosInit(void);
void MCU_GyrosDeInit(void);
void MCU_TemprInit(void);
void MCU_TemprDeInit(void);
void MCU_SunSensInit(void);
void MCU_SunSensDeInit(void);

void MCU_SdioReset(uint8_t cards);
uint8_t MCU_SD_CardInit(uint8_t cards);
void MCU_SD_CardDeInit(uint8_t Sd_DetectDeInitEnable, uint8_t cards, uint8_t cardHwReset);
uint8_t MCU_InitSdCard(uint8_t cards, uint8_t cardHwReset);

void MCU_ExtFlashInit(void);
void MCU_ExtFlashDeInit(void);
void MCU_Init_I2cPullUps(void);

void MX_RTC_ReloadShadowRegs(void);

void PVD_BrownoutReset_Init();
void HAL_PWR_PVDCallback(void);

bool MCU_Init_IsOBConfigValid(void);
bool MCU_Init_SetDefaultOBConfig(void);

#endif    /* MCU_INIT_H */
/* **************************************************************************************** */
