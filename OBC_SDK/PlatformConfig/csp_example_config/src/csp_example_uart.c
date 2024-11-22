/*!
 *********************************************************************************************
 * @file csp_example_uart.c
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
#include "csp_example/inc/csp_example.h"
#include "csp/csp_debug.h"
#include "csp/arch/csp_semaphore.h"
#include "stm32f7xx_hal.h"
#include <string.h>


static volatile uint8_t _cspex_uart_rx_byte;


typedef struct
{
    bool full;
    uint32_t head;
    uint32_t tail;
    cspex_rx_msg_uart_t msgs[CSPEX_DRIVER_BUFFERS_UART];
} _cspex_uart_fifo_t;


static volatile _cspex_uart_fifo_t _cspex_uart_rx_fifo =
{
    .full = false,
    .head = 0,
    .tail = 0,
    .msgs = {{0}}
};


static UART_HandleTypeDef* _cspex_driver_data;


bool cspex_uart_init(void* driver_data)
{
    if (driver_data == NULL)
        return false;

    _cspex_driver_data = (UART_HandleTypeDef*)driver_data;

    _cspex_driver_data->Instance                    = CSPEX_INSTANCE_UART;
    _cspex_driver_data->Init.BaudRate               = CSPEX_BAUD_UART;
    _cspex_driver_data->Init.WordLength             = UART_WORDLENGTH_8B;
    _cspex_driver_data->Init.StopBits               = UART_STOPBITS_1;
    _cspex_driver_data->Init.Parity                 = UART_PARITY_NONE;
    _cspex_driver_data->Init.Mode                   = UART_MODE_TX_RX;
    _cspex_driver_data->Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    _cspex_driver_data->Init.OverSampling           = UART_OVERSAMPLING_16;
    _cspex_driver_data->Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    _cspex_driver_data->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(_cspex_driver_data) != HAL_OK)
        return false;

    if (HAL_UART_Receive_IT(_cspex_driver_data, (uint8_t*)&_cspex_uart_rx_byte, 1) != HAL_OK)
        return false;

    return true;
}


bool cspex_uart_deinit(void* driver_data)
{
    if (_cspex_driver_data == NULL)
        return false;

    if (driver_data != _cspex_driver_data)
        return false;

    if (HAL_UART_DeInit(_cspex_driver_data) != HAL_OK)
        return false;

    return true;
}


int cspex_uart_transmit(void* driver_data, const uint8_t* data, size_t data_sz)
{
    if (data == NULL || data_sz == 0)
        return CSP_ERR_DRIVER;

    if (_cspex_driver_data == NULL)
        return CSP_ERR_DRIVER;

    if (driver_data != _cspex_driver_data)
        return CSP_ERR_DRIVER;

    if (HAL_UART_Transmit(_cspex_driver_data, (uint8_t*)data, data_sz, 100) != HAL_OK)
        return CSP_ERR_DRIVER;

    return CSP_ERR_NONE;
}


bool cspex_uart_receive(cspex_rx_msg_uart_t* msg)
{
    bool res = false;

    if (msg == NULL)
        return res;

    CSP_ENTER_CRITICAL(_cspex_uart_lock);

    if ((_cspex_uart_rx_fifo.tail != _cspex_uart_rx_fifo.head) || _cspex_uart_rx_fifo.full)
    {
        memcpy((void*)msg, (void*)&_cspex_uart_rx_fifo.msgs[_cspex_uart_rx_fifo.tail], sizeof(cspex_rx_msg_uart_t));

        _cspex_uart_rx_fifo.tail++;
        if (_cspex_uart_rx_fifo.tail >= CSPEX_DRIVER_BUFFERS_UART)
            _cspex_uart_rx_fifo.tail = 0;
        _cspex_uart_rx_fifo.full = false;

        res = true;
    }

    CSP_EXIT_CRITICAL(_cspex_uart_lock);

    return res;
}


void cspex_uart_rx_clbk(void)
{
    static cspex_rx_msg_uart_t rx_msg = {0};

    if (_cspex_driver_data == NULL)
        return;

    rx_msg.data[rx_msg.data_sz] = _cspex_uart_rx_byte;
    rx_msg.data_sz++;

    if (rx_msg.data_sz >= CSPEX_DRIVER_BUFFER_DATA_SIZE_UART || _cspex_uart_rx_byte == 0xC0)
    {
        if (!_cspex_uart_rx_fifo.full)
        {
            _cspex_uart_rx_fifo.msgs[_cspex_uart_rx_fifo.head].data_sz = rx_msg.data_sz;

            memcpy((void*)(_cspex_uart_rx_fifo.msgs[_cspex_uart_rx_fifo.head].data),
                   (void*)rx_msg.data,
                   CSPEX_DRIVER_BUFFER_DATA_SIZE_UART);

            _cspex_uart_rx_fifo.head++;
            if (_cspex_uart_rx_fifo.head >= CSPEX_DRIVER_BUFFERS_UART)
                _cspex_uart_rx_fifo.head = 0;

            if (_cspex_uart_rx_fifo.head == _cspex_uart_rx_fifo.tail)
                _cspex_uart_rx_fifo.full = true;
        }

        rx_msg.data_sz = 0;
        memset(&rx_msg, 0, sizeof(rx_msg));
    }

    if (HAL_UART_Receive_IT(_cspex_driver_data, (uint8_t*)&_cspex_uart_rx_byte, 1) != HAL_OK)
        cspex_uart_err_clbk();
}


void cspex_uart_err_clbk(void)
{
    if (!cspex_uart_deinit(_cspex_driver_data))
        return;

    if (!cspex_uart_init(_cspex_driver_data))
        return;
}


