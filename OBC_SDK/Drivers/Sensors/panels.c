/*!
 ********************************************************************************************
 * @file Panels.c
 * @brief Manage the panels with all the sensors ( magnetorquer, gyroscope, temeperature sensor, sun sensor )
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

/*
*********************************************************************************************
 * INCLUDES
*********************************************************************************************
 */
#include "FreeRTOS.h"
#include "task.h"

#include "stm32f7xx_hal.h"
#include "main.h"
#include "MCU_Init.h"
#include "cmsis_os.h"
#include "panels.h"
#include <stdio.h>
#include "math.h"

#include <stdint.h>
#include <string.h>

/*
*********************************************************************************************
 * INTERNAL DEFINES
*********************************************************************************************
 */
#define	__no_operation()		    __asm volatile("nop");
#define PANLE_GYRO_MAX_TRYES        (3)

static uint8_t Panel_Magnetorqer_Power[3] = {0,0,0};
static uint8_t Panel_Magnetorqer_Direction[3] = {0,0,0};

static PWM_Cntrl_t sPWMCntrl[3] = {
        {
                .TIM_CHANNEL = TIM_CHANNEL_1,
                {
                        .port = PAN4_DIR1_GPIO_Port,
                        .pin  = PAN4_DIR1_Pin,
                },
        },
        {
                .TIM_CHANNEL = TIM_CHANNEL_2,
                {
                        .port = PAN5_DIR1_GPIO_Port,
                        .pin  = PAN5_DIR1_Pin,
                },
        },
        {
                .TIM_CHANNEL = TIM_CHANNEL_3,
                {
                        .port = PAN6_DIR1_GPIO_Port,
                        .pin  = PAN6_DIR1_Pin,
                },
        },
};


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
/**
 * @brief Array of MAX_PAN (6) elements, representing panels' photometric sensor data input
 */
uint16_t PanelLight[MAX_PAN] = { 0, 0, 0, 0, 0, 0 };
/**
 * @brief Array of MAX_PAN (6) elements, representing panels' thermoelectric sensor data input
 */
uint16_t PanelTemp[MAX_PAN] = { 0, 0, 0, 0, 0, 0 };
/**
 * @brief Bitmask, representing mounted and active gyroscopes to gather data from (spatial and angular differentials)
 */
volatile uint8_t GyroStat = 0;

/*
*********************************************************************************************
 * INTERNAL (STATIC) VARIABLES DEFINITION/DECLARATION
*********************************************************************************************
 */
/**
 * @brief Bitmask, defining mounted and active panels to gather data from (luminescence and temperature)
 */
static volatile uint8_t PanelStat = 0;

static const uint32_t AdcChannel[MAX_PAN] = { ADC_CHANNEL_10, ADC_CHANNEL_3, ADC_CHANNEL_4,
                                              ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_8 };

/*
*********************************************************************************************
 * INTERNAL (STATIC) ROUTINES DECLARATION
*********************************************************************************************
 */
static void SPI_SlaveSelect(uint8_t Mode);

/*
*********************************************************************************************
 * EXTERNAL (NONE STATIC) ROUTINES DEFINITION
*********************************************************************************************
 */
/*!
*********************************************************************************************
 * @brief Panels initialisation routine
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void Panels_ChipSelect2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIO and SPI Clocks
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(PAN_Temp_PS_GPIO_Port, PAN_Temp_PS_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin  = PAN_Temp_PS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PAN5_CS2_GPIO_Port, &GPIO_InitStruct);
    PanelStat = 0;

    // Configure all panel temperature sensors CS as inputs with pull-down
    GPIO_InitStruct.Pin = PAN5_CS2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PAN5_CS2_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin =  PAN1_CS2_Pin | PAN3_CS2_Pin | PAN4_CS2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PAN6_CS2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PAN6_CS2_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PAN2_CS2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PAN2_CS2_GPIO_Port, &GPIO_InitStruct);

    sysDelay(2);

    if (HAL_GPIO_ReadPin(PAN1_CS2_GPIO_Port, PAN1_CS2_Pin))
        PanelStat |= (1 << PANEL_1);
    if (HAL_GPIO_ReadPin(PAN2_CS2_GPIO_Port, PAN2_CS2_Pin))
        PanelStat |= (1 << PANEL_2);
    if (HAL_GPIO_ReadPin(PAN3_CS2_GPIO_Port, PAN3_CS2_Pin))
        PanelStat |= (1 << PANEL_3);
    if (HAL_GPIO_ReadPin(PAN4_CS2_GPIO_Port, PAN4_CS2_Pin))
        PanelStat |= (1 << PANEL_4);
    if (HAL_GPIO_ReadPin(PAN5_CS2_GPIO_Port, PAN5_CS2_Pin))
        PanelStat |= (1 << PANEL_5);
    if (HAL_GPIO_ReadPin(PAN6_CS2_GPIO_Port, PAN6_CS2_Pin))
        PanelStat |= (1 << PANEL_6);

    // Configure all panel temperature sensors CS as push-pull outputs
    HAL_GPIO_WritePin(PAN5_CS2_GPIO_Port, PAN5_CS2_Pin, GPIO_PIN_SET);                  //Chip selects is OFF
    HAL_GPIO_WritePin(GPIOI, PAN1_CS2_Pin | PAN3_CS2_Pin | PAN4_CS2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PAN6_CS2_GPIO_Port, PAN6_CS2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PAN2_CS2_GPIO_Port, PAN2_CS2_Pin, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = PAN5_CS2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PAN5_CS2_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin =  PAN1_CS2_Pin | PAN3_CS2_Pin | PAN4_CS2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PAN6_CS2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PAN6_CS2_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PAN2_CS2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PAN2_CS2_GPIO_Port, &GPIO_InitStruct);
}


/*!
*********************************************************************************************
 * @brief Panels initialisation routine
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void Panels_ChipSelect2_DeInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitStruct.Pin = PAN5_CS2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PAN5_CS2_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PAN1_CS2_Pin|PAN3_CS2_Pin|PAN4_CS2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PAN6_CS2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PAN6_CS2_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PAN2_CS2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PAN2_CS2_GPIO_Port, &GPIO_InitStruct);
}


char Panels_GetPanelAxisDirectionAsChar(const PanDir_t direction)
{
    static const char dir[MAX_DIR] = { '-', '+' };

    if (direction > MAX_DIR)
        return 0;

    return dir[direction];
}


/*!
*********************************************************************************************
 * @brief Checks if a given panel is attached or not.
*********************************************************************************************
 * @param[input]      panelId - panel identifier;
 * @return            True - panel attached;
 *                    False - panel missing
 * @note              none
*********************************************************************************************
 */
bool Panels_IsAttached(const PanId_t panelId)
{
    return (panelId < MAX_PAN) ?
           (bool) (PanelStat & (1 << panelId)) :
           (false);
}

/*!
*********************************************************************************************
 * @brief Returns last magnetic torque power (in %) and direction (cartesian axis direction) to a specific panel
*********************************************************************************************
 * @param[input]      uint8_t Panel - panel identifier;
 * @param[output]     uint8_t perc - applied power in %; uint8_t dir - cartesian axis direction
 * @return            SEN_SUCCESS or SEN_ERROR
 * @note              none
*********************************************************************************************
 */
status_t GetMagnetorque(uint8_t Panel, PANLE_TRQ_str* data)
{

    if (Panel == PANEL_4) {
            data->power = Panel_Magnetorqer_Power[PANEL_1];
            data->dir  = Panel_Magnetorqer_Direction[PANEL_1];
        } else if (Panel == PANEL_5) {
            data->power = Panel_Magnetorqer_Power[PANEL_2];
            data->dir  = Panel_Magnetorqer_Direction[PANEL_2];
        } else if (Panel == PANEL_6) {
            data->power = Panel_Magnetorqer_Power[PANEL_3];
            data->dir  = Panel_Magnetorqer_Direction[PANEL_3];
        } else
            return SEN_ERROR;

    return SEN_SUCCESS;
}

/*!
*********************************************************************************************
 * @brief Applies or stops magnetic torque power (in %) and direction (cartesian axis direction) to a specific panel
*********************************************************************************************
 * @param[input]      uint8_t Panel - panel identifier; uint8_t perc - applied power in %; uint8_t dir - cartesian axis direction
 * @param[output]     none
 * @return            SEN_SUCCESS or SEN_ERROR
 * @note              none
*********************************************************************************************
 */
status_t SetMagnetorque(uint8_t Panel, uint8_t perc, uint8_t dir) 
{
    uint8_t u8PwrSettingPercentage = (perc > 100U) ? (100U) : (perc);
    uint32_t tmp = (uint32_t) u8PwrSettingPercentage;

    TIM_OC_InitTypeDef sConfigOC;

    if ( (Panel < PANEL_4) || (Panel > PANEL_6) )
        return SEN_ERROR;
    else
        Panel -= (uint8_t)PANEL_4; /* PAN_X_M is the base panel */ //todo VVS: This depends on the enum value ...not good

    if (dir > 1)
        dir = 1;

    HAL_TIM_PWM_Stop(&htim5, sPWMCntrl[Panel].TIM_CHANNEL);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (tmp * (MAGTORQ_PWM_PERIOD + 1) + 50) / 100;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, sPWMCntrl[Panel].TIM_CHANNEL);

    HAL_TIM_PWM_Start(&htim5, sPWMCntrl[Panel].TIM_CHANNEL);

    Panel_Magnetorqer_Power[Panel]     = u8PwrSettingPercentage;
    Panel_Magnetorqer_Direction[Panel] = dir;

    if (dir) {
        HAL_GPIO_WritePin(sPWMCntrl[Panel].INA.port, sPWMCntrl[Panel].INA.pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(sPWMCntrl[Panel].INA.port, sPWMCntrl[Panel].INA.pin, GPIO_PIN_SET);
    }

    return SEN_SUCCESS;
}

/*!
*********************************************************************************************
 * @brief Calculates all panels magnetorquer value, related to the dipole moment parameter (represented hereby Magnetorquer_Axis_t vector); checks direction and updates power.
*********************************************************************************************
 * @param[input]      Magnetorquer_Axis_t MT_level - dipole moment vector -> double {X, Y, Z}
 * @param[output]     none
 * @return            none
 * @note              Connection matrix - Relates mangetometer axis with mangetorquer axis
 *                          magnetometer Y-  <-> magnetorquer X+
 *                          magnetometer X-  <-> magnetorquer Y-
 *                          magnetometer Z-  <-> magnetorquer Z-
*********************************************************************************************
 */
void Magnetorquers_Update(Magnetorquer_Axis_t MT_level) 
{
    static uint8_t LevX = 0, LevY = 0, LevZ = 0; /* PWM value in percentage */
    static uint8_t DirX = 1, DirY = 1, DirZ = 1; /* PWM direction           */

    /* Set direction */
    if (MT_level.AXIS_X >= 0)
        DirX = 1;
    else
        DirX = 0;
    if (MT_level.AXIS_Y >= 0)
        DirY = 1;
    else
        DirY = 0;
    if (MT_level.AXIS_Z >= 0)
        DirZ = 1;
    else
        DirZ = 0;

    LevX = (uint8_t) (fabs(MT_level.AXIS_X * MTQ_DIP_MOM_TO_PWM_CONV_MULT));
    if (LevX > 100)
        LevX = 100;
    LevY = (uint8_t) (fabs(MT_level.AXIS_Y * MTQ_DIP_MOM_TO_PWM_CONV_MULT));
    if (LevY > 100)
        LevY = 100;
    LevZ = (uint8_t) (fabs(MT_level.AXIS_Z * MTQ_DIP_MOM_TO_PWM_CONV_MULT));
    if (LevZ > 100)
        LevZ = 100;

    SetMagnetorque(PANEL_4, LevZ, DirZ); /* PAN4 */
    SetMagnetorque(PANEL_5, LevX, DirX); /* PAN5 */
    SetMagnetorque(PANEL_6, LevY, DirY); /* PAN6 */

}

/*!
*********************************************************************************************
 * @brief Forces all magnetorquers on all panels to maximum power and given input direction
*********************************************************************************************
 * @param[input]      uint8_t Arrow - cartesian axis direction
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void Boost_Magnetorquers(uint8_t Arrow) 
{
    SetMagnetorque(PANEL_4, 100, Arrow);
    SetMagnetorque(PANEL_5, 100, Arrow);
    SetMagnetorque(PANEL_6, 100, Arrow);
}

/*!
*********************************************************************************************
 * @brief Turns off (0% power) all magnetorquers on all panels
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void Stop_Magnetorquers(void) 
{
    SetMagnetorque(PANEL_4, 0, 1);
    SetMagnetorque(PANEL_5, 0, 1);
    SetMagnetorque(PANEL_6, 0, 1);
}

/*!
*********************************************************************************************
 * @brief Makes ADC measurement on the specific channel, attached to each photo sensor
*********************************************************************************************
 * @param[input]      uint8_t Panel - panel identifier
 * @param[output]     uint16_t *val - the ADC output value
 * @return            SEN_SUCCESS or SEN_ERROR
 * @note              none
*********************************************************************************************
 */
status_t Pan_PD_ADC_Measure(uint8_t Panel, uint16_t *val) 
{
    ADC_ChannelConfTypeDef sConfig;

    //Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    sConfig.Channel = AdcChannel[Panel];
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        return SEN_ERROR;
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
        return SEN_ERROR;
    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
        return SEN_ERROR;
    *val = (uint16_t) HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return SEN_SUCCESS;
}

/*!
*********************************************************************************************
 * @brief Gets the ADC measurement of all the photo sensors, attached to the panels, and updates the global photometric data sensor array (that is - PanelLight) accordingly
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void Panel_GetPhotodiodesLum(void) 
{
    for (uint8_t i = 0; i < MAX_PAN; i++) 
	{
        if (Pan_PD_ADC_Measure(i, (uint16_t *) &PanelLight[i]) != SEN_SUCCESS)
            PanelLight[i] = 0xFFFF;
    }
}

/*!
*********************************************************************************************
 * @brief Performs 400 NOP instructions (e.g. sleep simulation for specific delay value, which itself is directly related to the uC operational frequency)
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void StallDelay(void)
{
    //400 cycles => 12.2us at max core speed (216Mhz)
    for (volatile uint32_t i = 0; i < 400; i++)  //two 16 bit sequences must be separated by minimum 12us Stall period
      __no_operation();
}


/*!
*********************************************************************************************
 * @brief Sets ADIS16265 gyroscope SPI slave's register with 8-bit data value for a specific panel
*********************************************************************************************
 * @param[input]      uint8_t Address - register address, uint8_t data - 8-bit input data, uint8_t Panel - panel identifier
 * @param[output]     none
 * @return            SEN_SUCCESS or SEN_ERROR
 * @note              none
*********************************************************************************************
 */
status_t ADIS16265_WriteReg8(uint8_t Address, uint8_t data, uint8_t Panel)
{
    uint16_t txdata;
    HAL_StatusTypeDef res;

    txdata = (0x8000 | ((uint16_t) Address << 8) | (uint16_t) data);

    SPI_SlaveSelect(GYRO_CS | CS_ON | Panel);
    res = HAL_SPI_Transmit(&PANEL_SPI_HANDLER, (uint8_t *) &txdata, 1, 10);
    SPI_SlaveSelect(GYRO_CS | CS_OFF | Panel);

    StallDelay();

    if (res != HAL_OK)
        return SEN_ERROR;

    return SEN_SUCCESS;
}

/*!
*********************************************************************************************
 * @brief Sets ADIS16265 gyroscope SPI slave's register with 16-bit data value for a specific panel
*********************************************************************************************
 * @param[input]      uint8_t Address - register address, uint16_t data - 16-bit input data, uint8_t Panel - panel identifier
 * @param[output]     none
 * @return            SEN_SUCCESS or SEN_ERROR
 * @note              none
*********************************************************************************************
 */
status_t ADIS16265_WriteReg16(uint8_t Address, uint16_t data, uint8_t Panel)
{
    uint16_t txdata;
    HAL_StatusTypeDef res;

    txdata = (0x8000 | ((uint16_t) (Address + 1) << 8) | (data >> 8));

    SPI_SlaveSelect(GYRO_CS | CS_ON | Panel);
    res = HAL_SPI_Transmit(&PANEL_SPI_HANDLER, (uint8_t *) &txdata, 1, 10);
    SPI_SlaveSelect(GYRO_CS | CS_OFF | Panel);

    if (res != HAL_OK)
        return SEN_ERROR;

    StallDelay();

    txdata = (0x8000 | ((uint16_t) Address << 8) | (data & 0xFF));

    SPI_SlaveSelect(GYRO_CS | CS_ON | Panel);
    res = HAL_SPI_Transmit(&PANEL_SPI_HANDLER, (uint8_t *) &txdata, 1, 10);
    SPI_SlaveSelect(GYRO_CS | CS_OFF | Panel);

    if (res != HAL_OK)
        return SEN_ERROR;

    return SEN_SUCCESS;
}

/*!
*********************************************************************************************
 * @brief Reads 16-bit data value for a specific panel's ADIS16265 gyroscope SPI slave's register
*********************************************************************************************
 * @param[input]      uint8_t Address - register address, uint8_t Panel - panel identifier
 * @param[output]     uint16_t *data - 16-bit output data
 * @return            SEN_SUCCESS or SEN_ERROR
 * @note              none
*********************************************************************************************
 */
status_t ADIS16265_ReadReg16(uint8_t Address, uint16_t *data, uint8_t Panel)
{
    uint16_t txdata;
    HAL_StatusTypeDef res = HAL_OK;

    if (data == NULL)
        return SEN_ERROR;

    txdata = (uint16_t) Address << 8;

    SPI_SlaveSelect(GYRO_CS | CS_ON | Panel);
    res = HAL_SPI_Transmit(&PANEL_SPI_HANDLER, (uint8_t *) &txdata, 1, 10);
    SPI_SlaveSelect(GYRO_CS | CS_OFF | Panel);

    if (res != HAL_OK)
        return SEN_ERROR;

    StallDelay();

    SPI_SlaveSelect(GYRO_CS | CS_ON | Panel);
    res = HAL_SPI_TransmitReceive(&PANEL_SPI_HANDLER, (uint8_t *) &txdata, (uint8_t *) data, 1, 10);
    SPI_SlaveSelect(GYRO_CS | CS_OFF | Panel);

    if (res != HAL_OK)
        return SEN_ERROR;

    return SEN_SUCCESS;
}

/*!
*********************************************************************************************
 * @brief Initialises ADIS16265 gyroscope for a specific panel
*********************************************************************************************
 * @param[input]      uint8_t Panel - panel identifier
 * @param[output]     none
 * @return            SEN_SUCCESS or SEN_ERROR
 * @note              none
*********************************************************************************************
 */
status_t ADIS16265_Init(uint8_t Panel)
{
    uint16_t wTmp = 0;

    sysDelay(200);
    if (ADIS16265_ReadReg16(ADIS16260_PROD_ID, &wTmp, Panel) != SEN_SUCCESS)
        return SEN_ERROR;
    if (wTmp != ADIS16265_PROD_NUM)
        return SEN_ERROR;

    GyroStat |= (1 << Panel);

    for (uint8_t i = 0; i < 90; i += 2)
    {
        if ((i < 0x06 || i > 0x09) && (i < 0x10 || i > 0x13)
                && (i < 0x18 || i > 0x1f) && (i < 0x2a || i > 0x2f)
                && (i < 0x40 || i > 0x51))
        {
            if (SEN_SUCCESS != ADIS16265_ReadReg16(i, &wTmp, Panel))
                continue;
        }
    }

    if (SEN_SUCCESS != ADIS16265_WriteReg16(ADIS16260_SENS_AVG, 0x0104, Panel))
        return SEN_ERROR;

    return SEN_SUCCESS;
}

/*!
*********************************************************************************************
 * @brief Deinit ADIS16265 gyroscope for a specific panel
*********************************************************************************************
 * @param[input]      uint8_t Panel - panel identifier
 * @param[output]     none
 * @return            SEN_SUCCESS or SEN_ERROR
 * @note              none
*********************************************************************************************
 */
void ADIS16265_DeInit(uint8_t Panel)
{
    GyroStat &= ~(1 << Panel);
}

/*!
*********************************************************************************************
 * @brief Calculates the current ADIS16265 gyroscope cartesian differential vector raw value
*********************************************************************************************
 * @param[input]      none
 * @param[output]     AxesRaw_t* buff - cartesian differential vector raw (16-bit integer, uncalibrated) value output - int16_t {AXIS_X, AXIS_Y, AXIS_Z)
 * @return            SEN_SUCCESS or SEN_ERROR
 * @note              none
*********************************************************************************************
 */
status_t ADIS16265_GetAxesRate(PanId_t panel, int16_t* rate)
{
    status_t ReadStat;
    uint8_t i;

    /* Invalidate the data */
    *rate = 0x7FFF;

    if (panel >= PANEL_4 && panel <= PANEL_6)
    {
        for (i = 0; i < PANLE_GYRO_MAX_TRYES; i++)
        {
            if (GyroStat & (1 << panel))
            {
                ReadStat = ADIS16265_ReadReg16(ADIS16260_GYRO_OUT, (uint16_t*)rate, panel);

                if (ReadStat == SEN_SUCCESS)
                {
                    /* Invalidate the measurement if read value is not in the expected range */
                    if (((*rate & 0x3FFF) > ADIS16260_GYRO_OUT_POS_MAX_VAL) &&
                        ((*rate & 0x3FFF) < ADIS16260_GYRO_OUT_NEG_MAX_VAL))
                        i = PANLE_GYRO_MAX_TRYES;

                    break;
                }
            }
            else
                (void)ADIS16265_Init(panel);
        }

        if (i < PANLE_GYRO_MAX_TRYES)
            *rate &= 0x3FFF;
        else
            return SEN_ERROR;
    }
    else
        return SEN_ERROR;

    return SEN_SUCCESS;
}

/*!
*********************************************************************************************
 * @brief Calculates the current ADIS16265 gyroscope polar (angular) differential vector raw value
*********************************************************************************************
 * @param[input]      none
 * @param[output]     AxesRaw_t* buff - polar (angular) differential vector raw (16-bit integer, uncalibrated) value output - int16_t {AXIS_X, AXIS_Y, AXIS_Z)
 * @return            SEN_SUCCESS or SEN_ERROR
 * @note              none
*********************************************************************************************
 */
status_t ADIS16265_GetAxesAngle(PanId_t panel, int16_t* angle)
{
    uint32_t uTmp;
    status_t ReadStat;
    uint8_t i;

    if (angle == NULL)
        return SEN_ERROR;

    *angle = 0x7FFF;

    if (panel >= PANEL_4 && panel <= PANEL_6)
    {
        for (i = 0; i < PANLE_GYRO_MAX_TRYES; i++)
        {
            if (GyroStat & (1 << PANEL_4))
            {
                ReadStat = ADIS16265_ReadReg16(ADIS16260_ANGL_OUT, (uint16_t*)&uTmp, panel);
                if (ReadStat == SEN_SUCCESS)
                    break;
            }
            else
                (void)ADIS16265_Init(panel);
        }

        if (i < PANLE_GYRO_MAX_TRYES)
        {
            uTmp &= 0x3FFF;
            uTmp *= 3600;
            uTmp = (uTmp + ANGLE_UNITS_360 / 2) / ANGLE_UNITS_360;
            *angle = (int16_t)uTmp;
        }
        else
            return SEN_ERROR;
    }
    else
        return SEN_ERROR;

    return SEN_SUCCESS;
}

/*!
*********************************************************************************************
 * @brief Calculates the current ADIS16265 gyroscope temperature 3D gradient value
*********************************************************************************************
 * @param[input]      none
 * @param[output]     Temperature_t* tmp - temperature 3D gradient value output - int16_t {Temp_X, Temp_Y, Temp_Z)
 * @return            SEN_SUCCESS or SEN_ERROR
 * @note              none
*********************************************************************************************
 */
status_t ADIS16265_GetTemperature(Temperature_t* tmp)
{
    int32_t iTmp;
    status_t ReadStat;
    uint8_t i;

    if (tmp == NULL)
        return SEN_ERROR;

    tmp->Temp_X = tmp->Temp_Y = tmp->Temp_Z = 0x7FFF;

    if (PanelStat & (1 << PANEL_4))
    {
        for( i = 0; i < PANLE_GYRO_MAX_TRYES ; i ++ )
        {
            uint16_t val = 0x7FFF;
            ReadStat = ADIS16265_ReadReg16(ADIS16260_TEMP_OUT, (uint16_t *) (&val), PANEL_4);
            if( ReadStat == SEN_SUCCESS )
            {
                tmp->Temp_X = val;
                break;
            }else{
                (void)ADIS16265_Init(PANEL_4);
            }
        }
        if( i < PANLE_GYRO_MAX_TRYES )
        {
            *((uint16_t *) (&tmp->Temp_X)) <<= 4;
            tmp->Temp_X /= 16;
            iTmp = tmp->Temp_X;
            iTmp *= 1453;
            if (iTmp > 0)
                iTmp += 500;
            else
                iTmp -= 500;
            iTmp /= 1000;
            iTmp += 250;
            tmp->Temp_X = iTmp;
        }else{
            return SEN_ERROR;
        }
    }

    if (PanelStat & (1 << PANEL_5))
    {
        for( i = 0; i < PANLE_GYRO_MAX_TRYES ; i ++ )
        {
            uint16_t val = 0x7FFF;
            ReadStat = ADIS16265_ReadReg16(ADIS16260_TEMP_OUT, (uint16_t *) (&val), PANEL_5);
            if( ReadStat == SEN_SUCCESS )
            {
                tmp->Temp_Y = val;
                break;
            }else{
                (void)ADIS16265_Init(PANEL_5);
            }
        }
        if( i < PANLE_GYRO_MAX_TRYES )
        {
            *((uint16_t *) (&tmp->Temp_Y)) <<= 4;
            tmp->Temp_Y /= 16;
            iTmp = tmp->Temp_Y;
            iTmp *= 1453;
            if (iTmp > 0)
                iTmp += 500;
            else
                iTmp -= 500;
            iTmp /= 1000;
            iTmp += 250;
            tmp->Temp_Y = iTmp;
        }else{
            return SEN_ERROR;
        }
    }

    if (PanelStat & (1 << PANEL_6))
    {
        for( i = 0; i < PANLE_GYRO_MAX_TRYES ; i ++ )
        {
            uint16_t val = 0x7FFF;
            ReadStat = ADIS16265_ReadReg16(ADIS16260_TEMP_OUT, (uint16_t *) (&val), PANEL_6);
            if( ReadStat == SEN_SUCCESS )
            {
                tmp->Temp_Z = val;
                break;
            }else{
                (void)ADIS16265_Init(PANEL_6);
            }
        }
        if( i < PANLE_GYRO_MAX_TRYES )
        {
            *((uint16_t *) (&tmp->Temp_Z)) <<= 4;
            tmp->Temp_Z /= 16;
            iTmp = tmp->Temp_Z;
            iTmp *= 1453;
            if (iTmp > 0)
                iTmp += 500;
            else
                iTmp -= 500;
            iTmp /= 1000;
            iTmp += 250;
            tmp->Temp_Z = iTmp;
        }else{
            return SEN_ERROR;
        }
    }

    return SEN_SUCCESS;
}


/*!
*********************************************************************************************
 * @brief Calculates the current positive temperature 3D gradient value of the TMP122 temperature sensor (SPI slave)
*********************************************************************************************
 * @param[input]      none
 * @param[output]     Temperature_t* tmp - positive temperature 3D gradient value output - int16_t {Temp_X, Temp_Y, Temp_Z}
 * @return            SEN_SUCCESS or SEN_ERROR
 * @note              none
*********************************************************************************************
 */
status_t TMP122_GetTemperatureP(Temperature_t* tmp) 
{
    uint16_t txdata = 0;
    HAL_StatusTypeDef res = HAL_OK;

    if (tmp == NULL)
        return SEN_ERROR;

    tmp->Temp_X = tmp->Temp_Y = tmp->Temp_Z = (int16_t)0xFFFF;

    SPI_SlaveSelect(TEMP_CS | CS_ON | PANEL_1);
    res = HAL_SPI_TransmitReceive(&PANEL_SPI_HANDLER, (uint8_t *) &txdata, (uint8_t *) (&tmp->Temp_X), 1, 10);
    SPI_SlaveSelect(TEMP_CS | CS_OFF | PANEL_1);
    if (res != HAL_OK)
        return SEN_ERROR;

    SPI_SlaveSelect(TEMP_CS | CS_ON | PANEL_2);
    res = HAL_SPI_TransmitReceive(&PANEL_SPI_HANDLER, (uint8_t *) &txdata, (uint8_t *) (&tmp->Temp_Y), 1, 10);
    SPI_SlaveSelect(TEMP_CS | CS_OFF | PANEL_2);
    if (res != HAL_OK)
        return SEN_ERROR;

    SPI_SlaveSelect(TEMP_CS | CS_ON | PANEL_3);
    res = HAL_SPI_TransmitReceive(&PANEL_SPI_HANDLER, (uint8_t *) &txdata, (uint8_t *) (&tmp->Temp_Z), 1, 10);
    SPI_SlaveSelect(TEMP_CS | CS_OFF | PANEL_3);
    if (res != HAL_OK)
        return SEN_ERROR;

    return SEN_SUCCESS;
}

/*!
*********************************************************************************************
 * @brief Converts the raw temperature result provided by TMP122_GetTemperatureX() operation
 *        to degrees Celsius
*********************************************************************************************
 * @param[input]      rawTempValue
 * @param[output]     n/a
 * @return            convTempValue - rawTempValue converted to degrees Celsius
 * @note              none
*********************************************************************************************
 */
inline int16_t TMP122_getTemperatureInDegC(int16_t rawTempValue)
{
    return (int16_t) ((rawTempValue >> 3) * 10) >> 4;
}

/*!
*********************************************************************************************
 * @brief Calculates the current negative temperature 3D gradient value of the TMP122 temperature sensor (SPI slave)
*********************************************************************************************
 * @param[input]      none
 * @param[output]     Temperature_t* tmp - negative temperature 3D gradient value output - int16_t {Temp_X, Temp_Y, Temp_Z}
 * @return            SEN_SUCCESS or SEN_ERROR
 * @note              none
*********************************************************************************************
 */
status_t TMP122_GetTemperatureM(Temperature_t* tmp) 
{
    uint16_t txdata = 0;
    HAL_StatusTypeDef res = HAL_OK;

    if (tmp == NULL)
        return SEN_ERROR;

    tmp->Temp_X = tmp->Temp_Y = tmp->Temp_Z = (int16_t)0xFFFF;

    SPI_SlaveSelect(TEMP_CS | CS_ON | PANEL_4);
    res = HAL_SPI_TransmitReceive(&PANEL_SPI_HANDLER, (uint8_t *) &txdata,
            (uint8_t *) (&tmp->Temp_X), 1, 10);
    SPI_SlaveSelect(TEMP_CS | CS_OFF | PANEL_4);
    if (res != HAL_OK)
        return SEN_ERROR;

    SPI_SlaveSelect(TEMP_CS | CS_ON | PANEL_5);
    res = HAL_SPI_TransmitReceive(&PANEL_SPI_HANDLER, (uint8_t *) &txdata,
            (uint8_t *) (&tmp->Temp_Y), 1, 10);
    SPI_SlaveSelect(TEMP_CS | CS_OFF | PANEL_5);
    if (res != HAL_OK)
        return SEN_ERROR;

    SPI_SlaveSelect(TEMP_CS | CS_ON | PANEL_6);
    res = HAL_SPI_TransmitReceive(&PANEL_SPI_HANDLER, (uint8_t *) &txdata,
            (uint8_t *) (&tmp->Temp_Z), 1, 10);
    SPI_SlaveSelect(TEMP_CS | CS_OFF | PANEL_6);
    if (res != HAL_OK)
        return SEN_ERROR;

    return SEN_SUCCESS;
}


/*!
*********************************************************************************************
 * @brief Read a temperature sensor
*********************************************************************************************
 * @param[input]      panelNumber - number of panel
 * @param[output]     int16_t * tmp - temperature from a panel
 * @return            SEN_SUCCESS or SEN_ERROR
 * @note              none
*********************************************************************************************
 */
status_t TMP122_GetTemperatureFromPanel(PanId_t panelNumber, int16_t * tmp)
{
    uint16_t txdata = 0;
    HAL_StatusTypeDef res = HAL_OK;

    if( (tmp == NULL)||(panelNumber >= MAX_PAN ) )
        return SEN_ERROR;

    *tmp = (int16_t)0xFFFF;

    SPI_SlaveSelect(TEMP_CS | CS_ON | panelNumber);
    res = HAL_SPI_TransmitReceive(&PANEL_SPI_HANDLER, (uint8_t *) &txdata, (uint8_t *) (tmp), 1, 10);
    SPI_SlaveSelect(TEMP_CS | CS_OFF | panelNumber);

    if (res != HAL_OK)
        return SEN_ERROR;

    return SEN_SUCCESS;
}

/*
*********************************************************************************************
 * INTERNAL (STATIC) ROUTINES DEFINITION
*********************************************************************************************
 */

/*!
*********************************************************************************************
 * @brief Selects SPI slave (e.g. CS/SS pin for the proper SPI slave)
*********************************************************************************************
 * @param[input]      uint8_t Mode - identification bitmask, consisting of the device selector, operational mode (ON/OFF) and panel identifier
 * @param[output]     none
 * @return            none
 * @note              !!! Partially optimized by Georgi Georgiev on 06-06-19 !!!
*********************************************************************************************
 */
static void SPI_SlaveSelect(uint8_t Mode)
{
    GPIO_PinState GPIO_pstate;

    if (Mode & CS_OFF) {
        GPIO_pstate = GPIO_PIN_SET;
    } else {
        GPIO_pstate = GPIO_PIN_RESET;
    }

    switch (Mode & (MAX_PAN + 1)) {
    case 0:
        if (Mode & TEMP_CS) {
            HAL_GPIO_WritePin(PAN1_CS2_GPIO_Port, PAN1_CS2_Pin, GPIO_pstate);
        } else {
            HAL_GPIO_WritePin(PAN1_CS1_GPIO_Port, PAN1_CS1_Pin, GPIO_pstate);
        }
        break;
    case 1:
        if (Mode & TEMP_CS) {
            HAL_GPIO_WritePin(PAN2_CS2_GPIO_Port, PAN2_CS2_Pin, GPIO_pstate);
        } else {
            HAL_GPIO_WritePin(PAN2_CS1_GPIO_Port, PAN2_CS1_Pin, GPIO_pstate);
        }
        break;
    case 2:
        if (Mode & TEMP_CS) {
            HAL_GPIO_WritePin(PAN3_CS2_GPIO_Port, PAN3_CS2_Pin, GPIO_pstate);
        } else {
            HAL_GPIO_WritePin(PAN3_CS1_GPIO_Port, PAN3_CS1_Pin, GPIO_pstate);
        }
        break;
    case 3:
        if (Mode & TEMP_CS) {
            HAL_GPIO_WritePin(PAN4_CS2_GPIO_Port, PAN4_CS2_Pin, GPIO_pstate);
        } else {
            HAL_GPIO_WritePin(PAN4_CS1_GPIO_Port, PAN4_CS1_Pin, GPIO_pstate);
        }
        break;
    case 4:
        if (Mode & TEMP_CS) {
            HAL_GPIO_WritePin(PAN5_CS2_GPIO_Port, PAN5_CS2_Pin, GPIO_pstate);
        } else {
            HAL_GPIO_WritePin(PAN5_CS1_GPIO_Port, PAN5_CS1_Pin, GPIO_pstate);
        }
        break;
    case 5:
        if (Mode & TEMP_CS) {
            HAL_GPIO_WritePin(PAN6_CS2_GPIO_Port, PAN6_CS2_Pin, GPIO_pstate);
        } else {
            HAL_GPIO_WritePin(PAN6_CS1_GPIO_Port, PAN6_CS1_Pin, GPIO_pstate);
        }
        break;
    default:
        break;
    }

    sysDelay(2);
}


/* **************************************************************************************** */
