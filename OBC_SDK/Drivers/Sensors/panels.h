/*!
********************************************************************************************
* @file Panels.h
* @brief Header of Panels.c
********************************************************************************************
* @author            Kolio
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
* @revision{         1.0.0  , 2018.07.04, author Kolio, Initial revision }
* @endhistory
********************************************************************************************
*/
#ifndef PANELS_H
#define PANELS_H

/*
*********************************************************************************************
* INCLUDES
*********************************************************************************************
*/
#include "stm32f7xx_hal.h"
#include "User_types.h"

/*
*********************************************************************************************
* EXTERNAL DEFINES
*********************************************************************************************
*/
#define MTQ_DIP_MOM_TO_PWM_CONV_MULT ((float)833.34) /* Magnetorquer ADCS requested dipole moment conv to PWM percentage multiplier */

typedef enum tag_ePinState_t
{
    ePINSTATE_OFF         = 0,
    ePINSTATE_ON          = 1,
    ePINSTATE_INV_STATE   = 2,
} ePinStates_t;

typedef struct tag_BD_GPIO_Cntrl{
    GPIO_TypeDef * port;
    uint16_t       pin;
    ePinStates_t   state;
} tag_BD_GPIO_Cntrl;

typedef struct tag_PWM_Cntrl {
    uint32_t          TIM_CHANNEL;
    tag_BD_GPIO_Cntrl INA;
} PWM_Cntrl_t;

typedef enum
{
    PANEL_1,    // Panel X Plus
    PANEL_2,    // Panel Y Plus
    PANEL_3,    // Panel Z Plus
    PANEL_4,    // Panel X Minus
    PANEL_5,    // Panel Y Minus
    PANEL_6,    // Panel Z Minus
    MAX_PAN
} PanId_t;

typedef enum
{
    PAN_NEG,
    PAN_POS,
    MAX_DIR
} PanDir_t;

#define TRQ_1           1       /* Magnetorquer 1 */
#define TRQ_2           2       /* Magnetorquer 2 */
#define TRQ_3           3       /* Magnetorquer 3 */

#define GYR_1           1       /* Magnetorquer 1 */
#define GYR_2           2       /* Magnetorquer 2 */
#define GYR_3           3       /* Magnetorquer 3 */


#define GYRO_CS 0x00
#define TEMP_CS 0x80

#define CS_ON   0x00
#define CS_OFF  0x60
   
#define PAN_GYRO_PRESENT        0xFF   
#define PAN_GYRO_NP             0x00
   
#define MAGTORQ_PWM_PERIOD 899

#define TMP122_CONVERSION_TIME  (265)   // Minimum measured time for temperature conversion is 235 ms, according to datasheet is 250ms

/*
*********************************************************************************************
* EXTERNAL TYPES DECLARATIONS
*********************************************************************************************
*/
#define ADIS16260_FLASH_CNT  0x00 /* Flash memory write count */
#define ADIS16260_SUPPLY_OUT 0x02 /* Power supply measurement */
#define ADIS16260_GYRO_OUT   0x04 /* X-axis gyroscope output */
#define ADIS16260_AUX_ADC    0x0A /* analog input channel measurement */
#define ADIS16260_TEMP_OUT   0x0C /* internal temperature measurement */
#define ADIS16260_ANGL_OUT   0x0E /* angle displacement */
#define ADIS16260_GYRO_OFF   0x14 /* Calibration, offset/bias adjustment */
#define ADIS16260_GYRO_SCALE 0x16 /* Calibration, scale adjustment */
#define ADIS16260_ALM_MAG1   0x20 /* Alarm 1 magnitude/polarity setting */
#define ADIS16260_ALM_MAG2   0x22 /* Alarm 2 magnitude/polarity setting */
#define ADIS16260_ALM_SMPL1  0x24 /* Alarm 1 dynamic rate of change setting */
#define ADIS16260_ALM_SMPL2  0x26 /* Alarm 2 dynamic rate of change setting */
#define ADIS16260_ALM_CTRL   0x28 /* Alarm control */
#define ADIS16260_AUX_DAC    0x30 /* Auxiliary DAC data */
#define ADIS16260_GPIO_CTRL  0x32 /* Control, digital I/O line */
#define ADIS16260_MSC_CTRL   0x34 /* Control, data ready, self-test settings */
#define ADIS16260_SMPL_PRD   0x36 /* Control, internal sample rate */
#define ADIS16260_SENS_AVG   0x38 /* Control, dynamic range, filtering */
#define ADIS16260_SLP_CNT    0x3A /* Control, sleep mode initiation */
#define ADIS16260_DIAG_STAT  0x3C /* Diagnostic, error flags */
#define ADIS16260_GLOB_CMD   0x3E /* Control, global commands */
#define ADIS16260_LOT_ID1    0x52 /* Lot Identification Code 1 */
#define ADIS16260_LOT_ID2    0x54 /* Lot Identification Code 2 */
#define ADIS16260_PROD_ID    0x56 /* Product identifier;
                                   * convert to decimal = 16,265/16,260 */
#define ADIS16260_SERIAL_NUM 0x58 /* Serial number */

#define ADIS16265_PROD_NUM   16265/* Product P/N */
#define ANGLE_UNITS_360      9828
//#define RATE_UNITS_320

#define ADIS16260_GYRO_OUT_POS_MAX_VAL  ((int16_t)0x1110)
#define ADIS16260_GYRO_OUT_NEG_MAX_VAL  ((int16_t)0x2EF0)
#define ADIS16260_GYRO_OUT_NEG_MIN_VAL  ((int16_t)0x4000)

#define ADIS16260_ERROR_ACTIVE                  (1<<14)
#define ADIS16260_NEW_DATA                      (1<<15)

/* MSC_CTRL */
#define ADIS16260_MSC_CTRL_MEM_TEST             (1<<11)
/* Internal self-test enable */
#define ADIS16260_MSC_CTRL_INT_SELF_TEST        (1<<10)
#define ADIS16260_MSC_CTRL_NEG_SELF_TEST        (1<<9)
#define ADIS16260_MSC_CTRL_POS_SELF_TEST        (1<<8)
#define ADIS16260_MSC_CTRL_DATA_RDY_EN          (1<<2)
#define ADIS16260_MSC_CTRL_DATA_RDY_POL_HIGH    (1<<1)
#define ADIS16260_MSC_CTRL_DATA_RDY_DIO2        (1<<0)

/* SMPL_PRD */
/* Time base (tB): 0 = 1.953 ms, 1 = 60.54 ms */
#define ADIS16260_SMPL_PRD_TIME_BASE            (1<<7)
#define ADIS16260_SMPL_PRD_DIV_MASK             0x7F

/* SLP_CNT */
#define ADIS16260_SLP_CNT_POWER_OFF             0x80

/* DIAG_STAT */
#define ADIS16260_DIAG_STAT_ALARM2              (1<<9)
#define ADIS16260_DIAG_STAT_ALARM1              (1<<8)
#define ADIS16260_DIAG_STAT_FLASH_CHK_BIT       6
#define ADIS16260_DIAG_STAT_SELF_TEST_BIT       5
#define ADIS16260_DIAG_STAT_OVERFLOW_BIT        4
#define ADIS16260_DIAG_STAT_SPI_FAIL_BIT        3
#define ADIS16260_DIAG_STAT_FLASH_UPT_BIT       2
#define ADIS16260_DIAG_STAT_POWER_HIGH_BIT      1
#define ADIS16260_DIAG_STAT_POWER_LOW_BIT       0

/* GLOB_CMD */
#define ADIS16260_GLOB_CMD_SW_RESET             (1<<7)
#define ADIS16260_GLOB_CMD_FLASH_UPD            (1<<3)
#define ADIS16260_GLOB_CMD_DAC_LATCH            (1<<2)
#define ADIS16260_GLOB_CMD_FAC_CALIB            (1<<1)
#define ADIS16260_GLOB_CMD_AUTO_NULL            (1<<0)

/* Values of all commands */
typedef enum{
    PANLE_GYROS_AXIS_X,
    PANLE_GYROS_AXIS_Y,
    PANLE_GYROS_AXIS_Z,
    PANLE_GYROS_AXIS_ALL,
    PANLE_GYROS_AXIS_NUMBER
}PANLE_GYROS_AXIS;

typedef struct{
    uint8_t power;
    uint8_t dir;
}PANLE_TRQ_str;

/*
*********************************************************************************************
* EXTERNAL VARIABLES DECLARATIONS
*********************************************************************************************
*/
extern uint16_t PanelLight[MAX_PAN];
extern uint16_t PanelTemp[MAX_PAN];
extern volatile uint8_t GyroStat;

/*
*********************************************************************************************
* EXTERNAL ROUTINES DECLARATIONS 
*********************************************************************************************
*/
void Panels_ChipSelect2_Init(void);
void Panels_ChipSelect2_DeInit(void);
char Panels_GetPanelAxisDirectionAsChar(const PanDir_t direction);
status_t Pan_PD_ADC_Measure(uint8_t Panel, uint16_t *val);
void Panel_GetPhotodiodesLum(void);
status_t GetMagnetorque(uint8_t Panel, PANLE_TRQ_str* data);
status_t SetMagnetorque(uint8_t Panel, uint8_t perc, uint8_t dir);
void Magnetorquers_Update (Magnetorquer_Axis_t MT_level);
void Boost_Magnetorquers (uint8_t Arrow);
void Stop_Magnetorquers (void);
bool Panels_IsAttached(const PanId_t panelId);
status_t ADIS16265_WriteReg8(uint8_t Address, uint8_t data, uint8_t Panel);
status_t ADIS16265_WriteReg16(uint8_t Address, uint16_t data, uint8_t Panel);
status_t ADIS16265_ReadReg16(uint8_t Address, uint16_t *data, uint8_t Panel);
status_t ADIS16265_Init(uint8_t Panel);
void ADIS16265_DeInit(uint8_t Panel);
status_t ADIS16265_GetAxesRate(PanId_t panel, int16_t* rate);
status_t ADIS16265_GetAxesAngle(PanId_t panelId, int16_t* angle);
status_t ADIS16265_GetTemperature(Temperature_t* tmp);

status_t TMP122_ReadReg16(uint16_t *data, uint8_t Panel);
status_t TMP122_GetTemperatureP(Temperature_t* tmp);
int16_t TMP122_getTemperatureInDegC(int16_t rawTempValue);
status_t TMP122_GetTemperatureM(Temperature_t* tmp);
status_t TMP122_GetTemperatureFromPanel(PanId_t panelNumber, int16_t * tmp);

#endif    /* PANELS_H */
/* **************************************************************************************** */
