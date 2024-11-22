/*!
*********************************************************************************************
* @file MX_I2C.c
* @brief MX_I2C
*********************************************************************************************
* @author            Vassil Milev
* @version           1.0.0
* @date              2020.11.14
*
* @copyright         (C) Copyright Endurosat
*
*                    Contents and presentations are protected world-wide.
*                    Any kind of using, copying etc. is prohibited without prior permission.
*                    All rights - incl. industrial property rights - are reserved.
*
* @history
* @revision{         1.0.0  , 2020.11.14, author Vassil Milev, Initial revision }
* @endhistory
*********************************************************************************************
*/

/*
*********************************************************************************************
* INCLUDES
*********************************************************************************************
*/
#include "MX_I2C.h"        /* Include own header file */
#include "es_exeh.h"                /* Include DTC error handling */
#include "semphr.h"
#include "stm32f7xx_ll_i2c.h"
#include "../Infrastructure/debug/inc/debug.h"

/*
*********************************************************************************************
* INTERNAL DEFINES
*********************************************************************************************
*/
#define MX_I2C_MAX_RETRYS           (5) //max retry to reset the I2C interface

/*
*********************************************************************************************
* INTERNAL TYPES DEFINITION
*********************************************************************************************
*/
typedef struct
{
    SemaphoreHandle_t xSemaphore;
    StaticSemaphore_t xMutex;
    TaskHandle_t xTaskInUse;
} MX_I2C_handlers_str;

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
static MX_I2C_handlers_str i2cHandlers[MX_I2C_BUS_COUNT];

/*
*********************************************************************************************
* INTERNAL (STATIC) ROUTINES DECLARATION
*********************************************************************************************
*/
static I2C_HandleTypeDef * MX_I2C_GetI2cHandler(MX_I2Cbus_enum bus);
static void MX_SYSTEM_I2C_Init(void);
static void MX_SYSTEM_I2C_DeInit(void);
static void MX_SENSORS_I2C_Init(void);
static void MX_SENSORS_I2C_DeInit(void);
static void MX_PAYLOAD_I2C_Init(void);
static void MX_PAYLOAD_I2C_DeInit(void);

/*
*********************************************************************************************
* EXTERNAL (NONE STATIC) ROUTINES DEFINITION
*********************************************************************************************
*/
/*!
*********************************************************************************************
* @brief Init routine for the MX_I2C component
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void MX_I2C_Init(void)
{
    //create the three semaphores for all I2C peripheries
    for (uint8_t i = 0; i < MX_I2C_BUS_COUNT; i++)
    {
        i2cHandlers[i].xSemaphore = xSemaphoreCreateMutexStatic(&i2cHandlers[i].xMutex);
        configASSERT(i2cHandlers[i].xSemaphore);
    }
}

/*!
*********************************************************************************************
* @brief Deinit routine for the MX_I2C component
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void MX_I2C_DeInit(void)
{
    // deinit all I2Cs
    for (uint8_t i = 0; i < MX_I2C_BUS_COUNT; i++)
    {
        TaskHandle_t tempTaskHandler = xSemaphoreGetMutexHolder(i2cHandlers[i].xSemaphore);
        if (tempTaskHandler != NULL)
        {
            MX_I2C_Release(i);
        }
    }
}


HAL_StatusTypeDef MX_I2C_Take(MX_I2Cbus_enum bus, TickType_t maxTicksToWait)
{
    BaseType_t getSemVal = pdFALSE;
    HAL_StatusTypeDef retVal = HAL_ERROR;

    if (bus < MX_I2C_BUS_COUNT)
    {
        getSemVal = xSemaphoreTake(i2cHandlers[bus].xSemaphore, maxTicksToWait);
    }

    if (getSemVal == pdTRUE)
    {
        MX_I2C_BusInit(bus);
        i2cHandlers[bus].xTaskInUse = xSemaphoreGetMutexHolder(i2cHandlers[bus].xSemaphore);
        retVal = HAL_OK;
    }

    return retVal;
}


void MX_I2C_Release(MX_I2Cbus_enum bus)
{
    BaseType_t getSemVal = pdFALSE;

    if (bus < MX_I2C_BUS_COUNT)
    {
        getSemVal = xSemaphoreGive(i2cHandlers[bus].xSemaphore);
    }

    if (getSemVal == pdTRUE)
    {
        MX_I2C_BusDeInit(bus);
    }
    else
    {
        Error_Handler();
    }
}

//func MX_I2C_BlockingTransmit(MX_I2Cbus_enum bus)
HAL_StatusTypeDef MX_I2C_BlockingTransmit(MX_I2Cbus_enum bus, uint16_t devAddress, uint8_t *pData, uint16_t size)
{
    uint8_t timeout = 0;
    HAL_StatusTypeDef I2C_retStat = HAL_ERROR;

    TaskHandle_t tempTaskHandler = xSemaphoreGetMutexHolder(i2cHandlers[bus].xSemaphore);
    if (i2cHandlers[bus].xTaskInUse == tempTaskHandler)
    {
        do
        {
            if (timeout >= 1)
            {
                if ((I2C_retStat == HAL_ERROR) || (I2C_retStat == HAL_TIMEOUT) || ((I2C_retStat == HAL_BUSY) && (hi2c1.State == HAL_I2C_STATE_READY)))
                {
                    MX_I2C_BusReset(bus);
                }
                osDelay(5);

                if (timeout >= MX_I2C_MAX_RETRYS)
                {
                    break;
                }
            }
            I2C_retStat = HAL_I2C_Master_Transmit(MX_I2C_GetI2cHandler(bus), devAddress << 1, (uint8_t*)pData, size, 30); // timeout 30ms ~= max 750 Bytes (200kbps => 25Bk/s => 25B/ms)

            timeout++;
        }
        while (I2C_retStat != HAL_OK);
    }

    return I2C_retStat;
}


HAL_StatusTypeDef MX_I2C_BlockingReceive(MX_I2Cbus_enum bus, uint16_t devAddress, uint8_t *pData, uint16_t size)
{
    uint8_t timeout = 0;
    HAL_StatusTypeDef I2C_retStat = HAL_ERROR;

    TaskHandle_t tempTaskHandler = xSemaphoreGetMutexHolder(i2cHandlers[bus].xSemaphore);
    if (i2cHandlers[bus].xTaskInUse == tempTaskHandler)
    {
        do
        {
            if (timeout >= 1)
            {
                // if there is any error reset the I2C interface
                if ((I2C_retStat == HAL_ERROR) || (I2C_retStat == HAL_TIMEOUT) || ((I2C_retStat == HAL_BUSY) && (hi2c1.State == HAL_I2C_STATE_READY)))
                {
                    MX_I2C_BusReset(bus);
                }
                osDelay(5);

                if (timeout >= MX_I2C_MAX_RETRYS)
                {
                    // Stop trying after certain times
                    EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_EPS_COMM, eEXEH_EPS_EXCEPTION_ID_COMM_ERROR, __LINE__);
                    break;
                }
            }

            //Read the register from the EPS
            I2C_retStat =
                HAL_I2C_Master_Receive(MX_I2C_GetI2cHandler(bus), devAddress << 1, pData, size, 30);

            timeout++; //count one more try
        }
        while (I2C_retStat != HAL_OK);
    }

    return I2C_retStat;
}


HAL_StatusTypeDef MX_I2C_BlockingMemWrite(MX_I2Cbus_enum bus, uint16_t devAddress, uint16_t writeAddr, uint16_t wAddrSize, uint8_t *pData, uint16_t size)
{
    uint8_t timeout = 0;
    HAL_StatusTypeDef I2C_retStat = HAL_ERROR;

    TaskHandle_t tempTaskHandler = xSemaphoreGetMutexHolder(i2cHandlers[bus].xSemaphore);
    if (i2cHandlers[bus].xTaskInUse == tempTaskHandler)
    {
        do
        {
            if (timeout >= 1)
            {
                if ((I2C_retStat == HAL_ERROR) || (I2C_retStat == HAL_TIMEOUT) || ((I2C_retStat == HAL_BUSY) && (hi2c1.State == HAL_I2C_STATE_READY)))
                {
                    MX_I2C_BusReset(bus);
                }
                osDelay(5);

                if (timeout >= MX_I2C_MAX_RETRYS)
                {
                    break;
                }
            }

            I2C_retStat = HAL_I2C_Mem_Write(MX_I2C_GetI2cHandler(bus), devAddress << 1, writeAddr, wAddrSize, (uint8_t*)pData, size, 30); // timeout 30ms ~= max 750 Bytes (200kbps => 25Bk/s => 25B/ms)

            timeout++;
        }
        while (I2C_retStat != HAL_OK);
    }

    return I2C_retStat;
}


HAL_StatusTypeDef MX_I2C_BlockingMemRead(MX_I2Cbus_enum bus, uint16_t devAddress, uint16_t writeAddr, uint16_t wAddrSize, uint8_t *pData, uint16_t size)
{
    uint8_t timeout = 0;
    HAL_StatusTypeDef I2C_retStat = HAL_ERROR;

    TaskHandle_t tempTaskHandler = xSemaphoreGetMutexHolder(i2cHandlers[bus].xSemaphore);
    if (i2cHandlers[bus].xTaskInUse == tempTaskHandler)
    {
        do
        {
            if (timeout >= 1)
            {
                // if there is any error reset the I2C interface
                if ((I2C_retStat == HAL_ERROR) || (I2C_retStat == HAL_TIMEOUT) || ((I2C_retStat == HAL_BUSY) && (hi2c1.State == HAL_I2C_STATE_READY)))
                {
                    MX_I2C_BusReset(bus);
                }
                osDelay(5);

                if (timeout >= MX_I2C_MAX_RETRYS)
                {
                    // Stop trying after certain times
                    EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_EPS_COMM, eEXEH_EPS_EXCEPTION_ID_COMM_ERROR, __LINE__);
                    break;
                }
            }

            //Read the register from the EPS
            I2C_retStat = HAL_I2C_Mem_Read(MX_I2C_GetI2cHandler(bus), devAddress << 1, writeAddr, wAddrSize, (uint8_t*)pData, size, 30); // timeout 30ms ~= max 750 Bytes (200kbps => 25Bk/s => 25B/ms)

            timeout++; //count one more try
        }
        while (I2C_retStat != HAL_OK);
    }

    return I2C_retStat;
}


void MX_I2C_BusInit(MX_I2Cbus_enum bus)
{
    switch (bus)
    {
        case MX_I2C_BUS_SYSTEM:
        {
            MX_SYSTEM_I2C_Init();
        } break;

        case MX_I2C_BUS_SENSORS:
        {
            MX_SENSORS_I2C_Init();
        } break;

        case MX_I2C_BUS_PAYLOAD:
        {
            MX_PAYLOAD_I2C_Init();
        } break;

        default:
        {
        } break;
    }
}


void MX_I2C_BusDeInit(MX_I2Cbus_enum bus)
{
    switch(bus)
    {
        case MX_I2C_BUS_SYSTEM:
        {
            MX_SYSTEM_I2C_DeInit();
        } break;

        case MX_I2C_BUS_SENSORS:
        {
            MX_SENSORS_I2C_DeInit();
        } break;

        case MX_I2C_BUS_PAYLOAD:
        {
            MX_PAYLOAD_I2C_DeInit();
        } break;

        default:
        {
        } break;
    }
}


void MX_I2C_BusReset(MX_I2Cbus_enum bus)
{
    switch(bus)
    {
        case MX_I2C_BUS_SYSTEM:
        {
            SYSTEM_I2C_Reset();
        } break;

        case MX_I2C_BUS_SENSORS:
        {
            SENSORS_I2C_Reset();
        } break;

        case MX_I2C_BUS_PAYLOAD:
        {
            PAYLOAD_I2C_Reset();
        } break;

        default:
        {
        } break;
    }
}


/*!
*********************************************************************************************
 * @brief SENSORS I2C (Integrated Circuit Interface Module #2) reset
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void SYSTEM_I2C_Reset(void)
{
    EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_MCU_INIT, eEXEH_MCU_INIT_EXCEPTION_ID_I2C_RST, __LINE__);

    LL_I2C_Disable(SYSTEM_I2C);//SYSTEM_I2C_HANDLER.Instance->CR1 &= ~I2C_CR1_PE;
    osDelay(1);
    LL_I2C_Enable(SYSTEM_I2C); //SYSTEM_I2C_HANDLER.Instance->CR1 |= I2C_CR1_PE;

    MX_SYSTEM_I2C_DeInit();


    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /*Configure GPIO pins : PCPin PCPin */
    GPIO_InitStruct.Pin = I2C1_SYS_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    //move PB8 -> SYS I2C Clock
    HAL_GPIO_WritePin(GPIOB, I2C1_SYS_SCL_Pin, GPIO_PIN_RESET);
    /*Configure GPIO pins : PCPin PCPin */
    GPIO_InitStruct.Pin = I2C1_SYS_SCL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    for (char i = 0; i < 16; i++)
    {
        if (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOB, I2C1_SYS_SDA_Pin))
        {
            break;
        }
        else // if I2C SDA pin is reset, the I2C SCL must do some clocks to make the data pit let go to high
        {
            HAL_GPIO_WritePin(GPIOB, I2C1_SYS_SCL_Pin, GPIO_PIN_SET);  //p1
            HAL_Delay(0);
            HAL_GPIO_WritePin(GPIOB, I2C1_SYS_SCL_Pin, GPIO_PIN_RESET);
            HAL_Delay(0);
        }
    }

    SYSTEM_I2C_HANDLER.State = HAL_I2C_STATE_RESET;

    MX_SYSTEM_I2C_Init();
}


/*!
*********************************************************************************************
 * @brief SENSORS I2C (Integrated Circuit Interface Module #2) reset
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void SENSORS_I2C_Reset(void)
{
    EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_MCU_INIT, eEXEH_MCU_INIT_EXCEPTION_ID_I2C_RST, __LINE__);

    LL_I2C_Disable(SENSORS_I2C);//SENSORS_I2C_HANDLER.Instance->CR1 &= ~I2C_CR1_PE;
    osDelay(1);
    LL_I2C_Enable(SENSORS_I2C); //SENSORS_I2C_HANDLER.Instance->CR1 |= I2C_CR1_PE;

    MX_SENSORS_I2C_DeInit();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /*Configure GPIO pins : PCPin PCPin */
    GPIO_InitStruct.Pin = I2C2_SEN_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(I2C2_SEN_SDA_GPIO_Port, &GPIO_InitStruct);

    //move PB8 -> SYS I2C Clock
    HAL_GPIO_WritePin(GPIOF, I2C2_SEN_SCL_Pin, GPIO_PIN_RESET);
    /*Configure GPIO pins : PCPin PCPin */
    GPIO_InitStruct.Pin = I2C2_SEN_SCL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    for (char i = 0; i < 16; i++)
    {
        if (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOF, I2C2_SEN_SDA_Pin))
        {
            break;
        }
        else // if I2C SDA pin is reset, the I2C SCL must do some clocks to make the data pit let go to high
        {
            HAL_GPIO_WritePin(GPIOF, I2C2_SEN_SCL_Pin, GPIO_PIN_SET);  //p1
            HAL_Delay(0);
            HAL_GPIO_WritePin(GPIOF, I2C2_SEN_SCL_Pin, GPIO_PIN_RESET);
            HAL_Delay(0);
        }
    }

    SENSORS_I2C_HANDLER.State = HAL_I2C_STATE_RESET;

    MX_SENSORS_I2C_Init();
}


/*!
*********************************************************************************************
 * @brief SENSORS I2C (Integrated Circuit Interface Module #2) reset
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
void PAYLOAD_I2C_Reset(void)
{
    EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_MCU_INIT, eEXEH_MCU_INIT_EXCEPTION_ID_I2C_RST, __LINE__);

    LL_I2C_Disable(PAYLOAD_I2C);//PAYLOAD_I2C_HANDLER.Instance->CR1 &= ~I2C_CR1_PE;
    osDelay(1);
    LL_I2C_Enable(PAYLOAD_I2C); //PAYLOAD_I2C_HANDLER.Instance->CR1 |= I2C_CR1_PE;

    MX_PAYLOAD_I2C_DeInit();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /*Configure GPIO pins : PCPin PCPin */
    GPIO_InitStruct.Pin = I2C3_PAY_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    //move PB8 -> SYS I2C Clock
    HAL_GPIO_WritePin(GPIOH, I2C3_PAY_SCL_Pin, GPIO_PIN_RESET);
    /*Configure GPIO pins : PCPin PCPin */
    GPIO_InitStruct.Pin = I2C3_PAY_SCL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    for (char i = 0; i < 16; i++)
    {
        if (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOH, I2C3_PAY_SDA_Pin))
        {
            break;
        }
        else // if I2C SDA pin is reset, the I2C SCL must do some clocks to make the data pit let go to high
        {
            HAL_GPIO_WritePin(GPIOH, I2C3_PAY_SCL_Pin, GPIO_PIN_SET);  //p1
            HAL_Delay(0);
            HAL_GPIO_WritePin(GPIOH, I2C3_PAY_SCL_Pin, GPIO_PIN_RESET);
            HAL_Delay(0);
        }
    }

    PAYLOAD_I2C_HANDLER.State = HAL_I2C_STATE_RESET;

    MX_PAYLOAD_I2C_Init();
}


/*
*********************************************************************************************
* INTERNAL (STATIC) ROUTINES DEFINITION
*********************************************************************************************
*/
/* No Internal routines definition */

static I2C_HandleTypeDef *MX_I2C_GetI2cHandler(MX_I2Cbus_enum bus)
{
    I2C_HandleTypeDef *handler = NULL;

    switch (bus)
    {
        case MX_I2C_BUS_SYSTEM:
        {
            handler = &SYSTEM_I2C_HANDLER;
        } break;

        case MX_I2C_BUS_SENSORS:
        {
            handler = &SENSORS_I2C_HANDLER;
        } break;

        case MX_I2C_BUS_PAYLOAD:
        {
            handler = &PAYLOAD_I2C_HANDLER;
        } break;

        default:
        {
        } break;
    }

    return handler;
}

/*!
*********************************************************************************************
 * @brief I2C1 (Integrated Circuit Interface Module #1) initialisation
*********************************************************************************************
 * @param[input]      &SYSTEM_I2C_HANDLER
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
static void MX_SYSTEM_I2C_Init(void)
{
    // take semaphore if not taken
    SYSTEM_I2C_HANDLER.Instance = SYSTEM_I2C;
    SYSTEM_I2C_HANDLER.Init.Timing = 0x6000030D;
    SYSTEM_I2C_HANDLER.Init.OwnAddress1 = 0;
    SYSTEM_I2C_HANDLER.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    SYSTEM_I2C_HANDLER.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    SYSTEM_I2C_HANDLER.Init.OwnAddress2 = 0;
    SYSTEM_I2C_HANDLER.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    SYSTEM_I2C_HANDLER.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    SYSTEM_I2C_HANDLER.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&SYSTEM_I2C_HANDLER) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Analogue filter */
    if (HAL_I2CEx_ConfigAnalogFilter(&SYSTEM_I2C_HANDLER, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Digital filter */
    if (HAL_I2CEx_ConfigDigitalFilter(&SYSTEM_I2C_HANDLER, 0) != HAL_OK)
    {
        Error_Handler();
    }
}


/*!
*********************************************************************************************
 * @brief I2C1 (Integrated Circuit Interface Module #1) DeInitialisation
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
static void MX_SYSTEM_I2C_DeInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = I2C1_SYS_SCL_Pin|I2C1_SYS_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_I2C_DeInit(&SYSTEM_I2C_HANDLER);
}


/*!
*********************************************************************************************
 * @brief I2C2 (Integrated Circuit Interface Module #2) initialisation
*********************************************************************************************
 * @param[input]      &SENSORS_I2C_HANDLER
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
static void MX_SENSORS_I2C_Init(void)
{
    SENSORS_I2C_HANDLER.Instance = SENSORS_I2C;
    SENSORS_I2C_HANDLER.Init.Timing = 0x6000030D;
    SENSORS_I2C_HANDLER.Init.OwnAddress1 = 0;
    SENSORS_I2C_HANDLER.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    SENSORS_I2C_HANDLER.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    SENSORS_I2C_HANDLER.Init.OwnAddress2 = 0;
    SENSORS_I2C_HANDLER.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    SENSORS_I2C_HANDLER.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    SENSORS_I2C_HANDLER.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&SENSORS_I2C_HANDLER) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Analogue filter */
    if (HAL_I2CEx_ConfigAnalogFilter(&SENSORS_I2C_HANDLER, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Digital filter */
    if (HAL_I2CEx_ConfigDigitalFilter(&SENSORS_I2C_HANDLER, 0) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_SENSORS_I2C_DeInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = I2C2_SEN_SCL_Pin|I2C2_SEN_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(I2C2_SEN_SCL_GPIO_Port, &GPIO_InitStruct);

    HAL_I2C_DeInit(&SENSORS_I2C_HANDLER);
}


/*!
*********************************************************************************************
 * @brief I2C3 (Integrated Circuit Interface Module #3) initialisation
*********************************************************************************************
 * @param[input]      &PAYLOAD_I2C_HANDLER
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
static void MX_PAYLOAD_I2C_Init(void)
{
    PAYLOAD_I2C_HANDLER.Instance = PAYLOAD_I2C;
    PAYLOAD_I2C_HANDLER.Init.Timing = 0x6000030D;
    PAYLOAD_I2C_HANDLER.Init.OwnAddress1 = 0;
    PAYLOAD_I2C_HANDLER.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    PAYLOAD_I2C_HANDLER.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    PAYLOAD_I2C_HANDLER.Init.OwnAddress2 = 0;
    PAYLOAD_I2C_HANDLER.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    PAYLOAD_I2C_HANDLER.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    PAYLOAD_I2C_HANDLER.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&PAYLOAD_I2C_HANDLER) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Analogue filter */
    if (HAL_I2CEx_ConfigAnalogFilter(&PAYLOAD_I2C_HANDLER, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Digital filter */
    if (HAL_I2CEx_ConfigDigitalFilter(&PAYLOAD_I2C_HANDLER, 0) != HAL_OK)
    {
        Error_Handler();
    }
}


/*!
*********************************************************************************************
 * @brief I2C3 (Integrated Circuit Interface Module #3) DeInitialisation
*********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              none
*********************************************************************************************
 */
static void MX_PAYLOAD_I2C_DeInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = I2C3_PAY_SCL_Pin|I2C3_PAY_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    HAL_I2C_DeInit(&PAYLOAD_I2C_HANDLER);
}

/* ******************************************************************************************* */
