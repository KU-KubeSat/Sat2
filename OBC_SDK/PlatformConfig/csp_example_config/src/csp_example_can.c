/*!
 ********************************************************************************************
 * @file csp_example_can.c
 *********************************************************************************************
 * @author            Andrian Dimitrov
 * @date              28.09.2021
 *
 * @copyright         (C) Copyright Endurosat
 *
 *                    Contents and presentations are protected world-wide.
 *                    Any kind of using, copying etc. is prohibited without prior permission.
 *                    All rights - incl. industrial property rights - are reserved.
 *********************************************************************************************
 */
#include "csp_example_config.h"
#include "csp/csp_debug.h"
#include "csp/arch/csp_thread.h"
#include "csp/arch/csp_semaphore.h"
#include "stm32f7xx_hal.h"
#include <string.h>


typedef struct
{
    bool full;
    uint32_t head;
    uint32_t tail;
    cspex_rx_msg_can_t msgs[CSPEX_DRIVER_BUFFERS_CAN];
} _csp_example_can_fifo_t;


static volatile _csp_example_can_fifo_t _cspex_can_rx_fifo =
{
    .full = false,
    .head = 0,
    .tail = 0,
    .msgs = {{0}}
};


static CAN_HandleTypeDef* _cspex_driver_data;


bool cspex_can_init(void* driver_data)
{
    if (driver_data == NULL)
        return false;

    _cspex_driver_data = (CAN_HandleTypeDef*)driver_data;

    _cspex_driver_data->Instance                  = CSPEX_INSTANCE_CAN;
    _cspex_driver_data->Init.Prescaler            = CPSEX_BAUD_CAN;
    _cspex_driver_data->Init.Mode                 = CAN_MODE_NORMAL;
    _cspex_driver_data->Init.SyncJumpWidth        = CAN_SJW_1TQ;
    _cspex_driver_data->Init.TimeSeg1             = CAN_BS1_6TQ;
    _cspex_driver_data->Init.TimeSeg2             = CAN_BS2_2TQ;
    _cspex_driver_data->Init.TimeTriggeredMode    = DISABLE;
    _cspex_driver_data->Init.AutoBusOff           = DISABLE;
    _cspex_driver_data->Init.AutoWakeUp           = DISABLE;
    _cspex_driver_data->Init.AutoRetransmission   = DISABLE;
    _cspex_driver_data->Init.ReceiveFifoLocked    = DISABLE;
    _cspex_driver_data->Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(_cspex_driver_data) != HAL_OK)
        return false;

    CAN_FilterTypeDef filterConfig    = { 0 };
    filterConfig.FilterBank           = 0;
    filterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
    filterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    filterConfig.FilterIdHigh         = 0;
    filterConfig.FilterIdLow          = 0;
    filterConfig.FilterMaskIdHigh     = 0;
    filterConfig.FilterMaskIdLow      = 0;
    filterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    filterConfig.FilterActivation     = ENABLE;
    filterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(_cspex_driver_data, &filterConfig) != HAL_OK)
        return false;

    if (HAL_CAN_ActivateNotification(_cspex_driver_data, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        return false;

    if (HAL_CAN_Start(_cspex_driver_data) != HAL_OK)
        return false;

    return true;
}


bool cspex_can_deinit(void* driver_data)
{
    if (_cspex_driver_data == NULL)
        return false;

    if (driver_data != _cspex_driver_data)
        return false;

    if (HAL_CAN_DeInit(_cspex_driver_data) != HAL_OK)
        return false;

    return true;
}


int cspex_can_transmit(void* driver_data, uint32_t id, const uint8_t* data, uint8_t data_sz)
{
    if (data == NULL || !IS_CAN_DLC(data_sz))
        return CSP_ERR_DRIVER;

    if (_cspex_driver_data == NULL)
        return CSP_ERR_DRIVER;

    if (driver_data != _cspex_driver_data)
        return CSP_ERR_DRIVER;

    while (HAL_CAN_GetTxMailboxesFreeLevel(_cspex_driver_data) < 3)
        csp_sleep_ms(10);

    CAN_TxHeaderTypeDef header =
    {
        .ExtId = id,
        .IDE   = CAN_ID_EXT,
        .RTR   = CAN_RTR_DATA,
        .DLC   = data_sz,
        .TransmitGlobalTime = DISABLE,
    };

    uint32_t mailbox;
    uint16_t elapsed_ms = 0;

    while (HAL_CAN_AddTxMessage(_cspex_driver_data, &header, (uint8_t*)data, &mailbox) != HAL_OK)
    {
        if (elapsed_ms >= 10)
        {
            csp_log_warn("HAL_CAN_AddTxMessage()");
            return CSP_ERR_DRIVER;
        }

        csp_sleep_ms(10);
        elapsed_ms += 1;
    }

    return CSP_ERR_NONE;
}


bool cspex_can_receive(cspex_rx_msg_can_t* msg)
{
    bool res = false;

    CSP_ENTER_CRITICAL(_cspex_can_lock);

    if (msg == NULL)
    {
        CSP_EXIT_CRITICAL(_cspex_can_lock);
        return res;
    }

    if ((_cspex_can_rx_fifo.tail != _cspex_can_rx_fifo.head) || _cspex_can_rx_fifo.full)
    {
        memcpy((void*)msg, (void*)&_cspex_can_rx_fifo.msgs[_cspex_can_rx_fifo.tail], sizeof(cspex_rx_msg_can_t));

        _cspex_can_rx_fifo.tail++;
        if (_cspex_can_rx_fifo.tail >= CSPEX_DRIVER_BUFFERS_CAN)
            _cspex_can_rx_fifo.tail = 0;
        _cspex_can_rx_fifo.full = false;

        res = true;
    }

    CSP_EXIT_CRITICAL(_cspex_can_lock);

    return res;
}


void cspex_can_rx_clbk(void)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[CSPEX_DRIVER_BUFFER_DATA_SIZE_CAN];

    if (_cspex_driver_data == NULL)
        return;

    if (HAL_CAN_GetRxMessage(_cspex_driver_data, CAN_RX_FIFO0, &header, data) != HAL_OK)
        return;

    if (_cspex_can_rx_fifo.full)
        return;

    _cspex_can_rx_fifo.msgs[_cspex_can_rx_fifo.head].id      = header.ExtId;
    _cspex_can_rx_fifo.msgs[_cspex_can_rx_fifo.head].data_sz = header.DLC;
    memcpy((void*)(_cspex_can_rx_fifo.msgs[_cspex_can_rx_fifo.head].data), (void*)data, CSPEX_DRIVER_BUFFER_DATA_SIZE_CAN);

    _cspex_can_rx_fifo.head++;
    if (_cspex_can_rx_fifo.head >= CSPEX_DRIVER_BUFFERS_CAN)
        _cspex_can_rx_fifo.head = 0;

    if (_cspex_can_rx_fifo.head == _cspex_can_rx_fifo.tail)
        _cspex_can_rx_fifo.full = true;

}


void cspex_can_err_clbk(void)
{
    if (!cspex_can_deinit(_cspex_driver_data))
        csp_log_error("cspex_can_deinit()");

    if (!cspex_can_init(_cspex_driver_data))
        csp_log_error("cspex_can_init()");
}




