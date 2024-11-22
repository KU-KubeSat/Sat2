/*!
 *********************************************************************************************
 * @file csp_example.c
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
#include "../inc/csp_example.h"
#include <stdio.h>

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif

#include "csp/csp.h"
#include "csp/arch/csp_thread.h"
#include "csp/interfaces/csp_if_can.h"
#include "csp/interfaces/csp_if_kiss.h"

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif


typedef enum
{
    _CSPEX_IFACE_CAN = 0,
    _CSPEX_IFACE_KISS,
    _CSPEX_IFACE_COUNT
} _cspex_iface_t;


typedef struct
{
    uint8_t comm_type;
    uint8_t local_port_uart;
    uint8_t local_port_can;
    uint8_t target_address_uart;
    uint8_t target_port_uart;
    uint8_t target_address_can;
    uint8_t target_port_can;
    uint8_t connection_options;
    csp_conf_t                config;
    csp_can_interface_data_t  iface_data_can;
    csp_kiss_interface_data_t iface_data_kiss;
    csp_iface_t               iface[_CSPEX_IFACE_COUNT];
    cspex_iface_can_t*        iface_can;
    cspex_iface_uart_t*       iface_uart;
} _cspex_t;


static _cspex_t _cspex;


CSP_DEFINE_TASK(_cspex_task_server)
{
    csp_socket_t* sock = csp_socket(_cspex.connection_options);
    csp_bind(sock, CSP_ANY);
    csp_listen(sock, 10);

    while (1)
    {
        csp_conn_t* conn = csp_accept(sock, 10000);
        if (conn == NULL)
            continue;

        csp_packet_t* packet;

        while ((packet = csp_read(conn, 1000)) != NULL)
        {
            int dport = csp_conn_dport(conn);

            if (dport == _cspex.local_port_uart)
            {
                csp_log_info("TASK_SERVER received UART packet: %.*s", packet->length, packet->data);
            }
            else if (dport == _cspex.local_port_can)
            {
                csp_log_info("TASK_SERVER received CAN packet: %.*s", packet->length, packet->data);
            }
            else
            {
                csp_log_info("TASK_SERVER received other request");
                csp_service_handler(conn, packet);
            }

            csp_buffer_free(packet);
        }

        while (csp_close(conn) != CSP_ERR_NONE);
    }

    return CSP_TASK_RETURN;
}


CSP_DEFINE_TASK(_cspex_task_client_uart)
{
    const char msg[] = "endurosat.CSP.UART";
    uint8_t count = 0;

    while (1)
    {
        csp_sleep_ms(10);

        csp_conn_t* conn = csp_connect(
                CSP_PRIO_NORM,
                _cspex.target_address_uart,
                _cspex.target_port_uart,
                2000,
                _cspex.connection_options);

        if (conn == NULL)
            continue;

        csp_packet_t* packet = csp_buffer_get(sizeof(csp_packet_t));
        if (packet == NULL)
        {
            csp_close(conn);
            continue;
        }

        int offset = sprintf((char*)packet->data, msg);
        offset = sprintf((char*)&packet->data[offset], " %u", (unsigned int)count);
        packet->length = strlen(msg) + offset;

        if (csp_send(conn, packet, 1000) == CSP_ERR_NONE)
            count++;
        csp_close(conn);
    }

    return CSP_TASK_RETURN;
}


CSP_DEFINE_TASK(_cspex_task_client_can)
{
    const char msg[] = "endurosat.CSP.CAN";
    uint8_t count = 0;

    while (1)
    {
        csp_sleep_ms(10);

        csp_conn_t* conn = csp_connect(
            CSP_PRIO_NORM,
            _cspex.target_address_can,
            _cspex.target_port_can,
            2000,
            _cspex.connection_options);

        if (conn == NULL)
            continue;

        if (conn == NULL)
            continue;

        csp_packet_t* packet = csp_buffer_get(sizeof(csp_packet_t));
        if (packet == NULL)
        {
            csp_close(conn);
            continue;
        }

        int offset = sprintf((char*)packet->data, msg);
        offset = sprintf((char*)&packet->data[offset], " %u", (unsigned int)count);
        packet->length = strlen(msg) + offset;

        if (csp_send(conn, packet, 1000) == CSP_ERR_NONE)
            count++;
        csp_close(conn);
    }

    return CSP_TASK_RETURN;
}


CSP_DEFINE_TASK(_cspex_task_rx_poll)
{
    while (1)
    {
        csp_sleep_ms(1);

        if (_cspex.comm_type & CSPEX_COMM_UART)
        {
            if (_cspex.iface_uart == NULL)
                continue;

            if (_cspex.iface_uart->rx == NULL)
                continue;

            cspex_rx_msg_uart_t rx_msg;
            if (_cspex.iface_uart->rx(&rx_msg))
                csp_kiss_rx(&_cspex.iface[_CSPEX_IFACE_KISS], (const uint8_t*)rx_msg.data, rx_msg.data_sz, NULL);
        }

        if (_cspex.comm_type & CSPEX_COMM_CAN)
        {
            if (_cspex.iface_can == NULL)
                continue;

            if (_cspex.iface_can->rx == NULL)
                continue;

            cspex_rx_msg_can_t rx_msg;
            if (_cspex.iface_can->rx(&rx_msg))
                csp_can_rx(&_cspex.iface[_CSPEX_IFACE_CAN], rx_msg.id, (const uint8_t*)rx_msg.data, rx_msg.data_sz, NULL);
        }
    }
}


bool cspex_init(cspex_iface_can_t*  iface_can,
                cspex_iface_uart_t* iface_uart,
                uint8_t comm_type,
                uint8_t mode,
                uint8_t local_address,
                uint8_t local_port_uart,
                uint8_t local_port_can,
                uint8_t target_address_uart,
                uint8_t target_port_uart,
                uint8_t target_address_can,
                uint8_t target_port_can,
                uint8_t connection_options,
                uint8_t tasks_priority)
{
    csp_conf_get_defaults(&_cspex.config);

    _cspex.iface_can               = iface_can;
    _cspex.iface_uart              = iface_uart;
    _cspex.comm_type               = comm_type;
    _cspex.local_port_uart         = local_port_uart;
    _cspex.local_port_can          = local_port_can;
    _cspex.target_address_uart     = target_address_uart;
    _cspex.target_port_uart        = target_port_uart;
    _cspex.target_address_can      = target_address_can;
    _cspex.target_port_can         = target_port_can;
    _cspex.connection_options      = connection_options;
    _cspex.config.buffers          = CSPEX_BUFFERS;
    _cspex.config.buffer_data_size = CSPEX_BUFFER_DATA_SIZE;
    _cspex.config.address          = local_address;

    if (csp_init(&_cspex.config) != CSP_ERR_NONE)
        return false;

    if (comm_type & CSPEX_COMM_UART)
    {
        if (_cspex.iface_uart == NULL)
            return false;

        if (_cspex.iface_uart->init == NULL)
            return false;

        if (_cspex.iface_uart->deinit == NULL)
            return false;

        if (_cspex.iface_uart->tx == NULL)
            return false;

        if (_cspex.iface_uart->rx == NULL)
            return false;

        _cspex.iface_data_kiss.tx_func                 = _cspex.iface_uart->tx;
        _cspex.iface[_CSPEX_IFACE_KISS].driver_data    = (void*)_cspex.iface_uart->driver_data;
        _cspex.iface[_CSPEX_IFACE_KISS].name           = "CSP_OVER_UART";
        _cspex.iface[_CSPEX_IFACE_KISS].interface_data = (void*)(&_cspex.iface_data_kiss);

        if (csp_kiss_add_interface(&_cspex.iface[_CSPEX_IFACE_KISS]) != CSP_ERR_NONE)
            return false;

        if (csp_rtable_set(_cspex.target_address_uart, CSP_ID_HOST_SIZE, &_cspex.iface[_CSPEX_IFACE_KISS], CSP_NO_VIA_ADDRESS) != CSP_ERR_NONE)
            return false;

        if (mode & CSPEX_MODE_CLIENT)
        {
            if (csp_thread_create(_cspex_task_client_uart, "CSP_CLIENT_UART", 500, NULL, tasks_priority, NULL) != CSP_ERR_NONE)
                return false;
        }

        if (!_cspex.iface_uart->init(_cspex.iface_uart->driver_data))
            return false;
    }

    if (comm_type & CSPEX_COMM_CAN)
    {
        if (_cspex.iface_can == NULL)
            return false;

        if (_cspex.iface_can->init == NULL)
            return false;

        if (_cspex.iface_can->deinit == NULL)
            return false;

        if (_cspex.iface_can->tx == NULL)
            return false;

        if (_cspex.iface_can->rx == NULL)
            return false;

        _cspex.iface_data_can.tx_func                 = _cspex.iface_can->tx;
        _cspex.iface[_CSPEX_IFACE_CAN].driver_data    = (void*)_cspex.iface_can->driver_data;
        _cspex.iface[_CSPEX_IFACE_CAN].name           = "CSP_OVER_CAN";
        _cspex.iface[_CSPEX_IFACE_CAN].interface_data = (void*)(&_cspex.iface_data_can);

        if (csp_can_add_interface(&_cspex.iface[_CSPEX_IFACE_CAN]) != CSP_ERR_NONE)
            return false;

        if (csp_rtable_set(_cspex.target_address_can, CSP_ID_HOST_SIZE, &_cspex.iface[_CSPEX_IFACE_CAN], CSP_NO_VIA_ADDRESS) != CSP_ERR_NONE)
            return false;

        if (mode & CSPEX_MODE_CLIENT)
        {
            if (csp_thread_create(_cspex_task_client_can, "CSP_CLIENT_CAN", 500, NULL, tasks_priority, NULL) != CSP_ERR_NONE)
                return false;
        }

        if (!_cspex.iface_can->init(_cspex.iface_can->driver_data))
            return false;
    }

    if (csp_route_start_task(500, tasks_priority) != CSP_ERR_NONE)
        return false;

    if (mode & CSPEX_MODE_SERVER)
    {
        if (csp_thread_create(_cspex_task_server, "CSP_SERVER", 500, NULL, tasks_priority, NULL) != CSP_ERR_NONE)
            return false;
    }

    if (csp_thread_create(_cspex_task_rx_poll, "CSP_RX_POLL", 500, NULL, tasks_priority, NULL) != CSP_ERR_NONE)
        return false;

    return true;
}


bool cspex_deinit(void)
{
    csp_free_resources();

    if (_cspex.iface_uart == NULL)
        return false;

    if (_cspex.iface_uart->deinit == NULL)
        return false;

    if (!_cspex.iface_uart->deinit(_cspex.iface_uart->driver_data))
        return false;

    if (_cspex.iface_can == NULL)
        return false;

    if (_cspex.iface_can->deinit == NULL)
        return false;

    if (!_cspex.iface_can->deinit(_cspex.iface_can->driver_data))
        return false;

    return true;
}

