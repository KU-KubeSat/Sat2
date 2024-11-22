/*!
 ********************************************************************************************
 * @file csp_example.h
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
#ifndef CSP_EXAMPLE_H_
#define CSP_EXAMPLE_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>


/** CSP example communication types (use with cspex_init()) */
#define CSPEX_COMM_UART                    (0x01)  /* mode UART */
#define CSPEX_COMM_CAN                     (0x02)  /* mode CAN  */


/** CSP example modes (use with cspex_init()) */
#define CSPEX_MODE_SERVER                  (0x01)  /* mode UART */
#define CSPEX_MODE_CLIENT                  (0x02)  /* mode CAN  */


/** used in csp_example.c */
#define CSPEX_BUFFERS                      ( 20)   /* user configurable */
#define CSPEX_BUFFER_DATA_SIZE             (128)   /* user configurable */


/** maximum user data in single message coming from driver CAN/UART */
#define CSPEX_DRIVER_BUFFER_DATA_SIZE_UART (128)   /* user configurable     */
#define CSPEX_DRIVER_BUFFER_DATA_SIZE_CAN  (  8)   /* NOT user configurable */


/**
 * CSP example uses the following data structure to retrieve
 * Incoming messages from CAN driver
 */
typedef struct
{
    uint32_t id;
    uint8_t  data_sz;
    uint8_t  data[CSPEX_DRIVER_BUFFER_DATA_SIZE_CAN];
} cspex_rx_msg_can_t;


typedef bool (*cspex_func_can_init_t)  (void* driver_data);
typedef bool (*cspex_func_can_deinit_t)(void* driver_data);
typedef int  (*cspex_func_can_tx_t)    (void* driver_data, uint32_t id, const uint8_t* data, uint8_t data_sz);
typedef bool (*cspex_func_can_rx_t)    (cspex_rx_msg_can_t* msg);


/**
 * IMPORTANT:
 *
 * if you don't find implementation of the following type
 * inside this project you must implement them yourself
 */
typedef struct
{
    void*                   driver_data; /** can be used by driver                                              */
    cspex_func_can_init_t   init;        /** start CAN driver (called from within CSP example)                  */
    cspex_func_can_deinit_t deinit;      /** stop CAN driver (called from within CSP example)                   */
    cspex_func_can_tx_t     tx;          /** transmit data via CAN (function declaration is dictated by libCSP) */
    cspex_func_can_rx_t     rx;          /** receive data via CAN (called from within CSP example)              */
} cspex_iface_can_t;


/**
 * CSP example uses the following data structure to retrieve
 * Incoming messages from UART driver
 */
typedef struct
{
    size_t  data_sz;
    uint8_t data[CSPEX_DRIVER_BUFFER_DATA_SIZE_UART];
} cspex_rx_msg_uart_t;


typedef bool (*cspex_func_uart_init_t)  (void* driver_data);
typedef bool (*cspex_func_uart_deinit_t)(void* driver_data);
typedef int  (*cspex_func_uart_tx_t)    (void* driver_data, const uint8_t* data, size_t data_sz);
typedef bool (*cspex_func_uart_rx_t)    (cspex_rx_msg_uart_t* msg);


/**
 * IMPORTANT:
 *
 * if you don't find implementation of the following type
 * inside this project you must implement them yourself
 */
typedef struct
{
    void*                    driver_data; /** can be used by driver                                               */
    cspex_func_uart_init_t   init;        /** start UART driver (called from within CSP example)                  */
    cspex_func_uart_deinit_t deinit;      /** stop UART driver (called from within CSP example)                   */
    cspex_func_uart_tx_t     tx;          /** transmit data via UART (function declaration is dictated by libCSP) */
    cspex_func_uart_rx_t     rx;          /** receive data via UART (called from within CSP example)              */
} cspex_iface_uart_t;


bool cspex_init(cspex_iface_can_t*  iface_can,  /* CAN driver interface    */
                cspex_iface_uart_t* iface_uart, /* UART driver interface   */
                uint8_t comm_type,              /* communication type      */
                uint8_t mode,                   /* mode (client or server) */
                uint8_t local_address,          /* local address           */
                uint8_t local_port_uart,        /* local port uart         */
                uint8_t local_port_can,         /* local port can          */
                uint8_t target_address_uart,    /* target uart address     */
                uint8_t target_port_uart,       /* target uart port        */
                uint8_t target_address_can,     /* target can address      */
                uint8_t target_port_can,        /* target can port         */
                uint8_t connection_options,     /* connection options      */
                uint8_t tasks_priority);        /* OS threads priority     */


bool cspex_deinit(void);


#endif /* CSP_EXAMPLE_H_ */
