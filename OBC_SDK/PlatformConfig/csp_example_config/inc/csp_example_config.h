/*!
 ********************************************************************************************
 * @file csp_example_config.h
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
#ifndef CSP_EXAMPLE_CONFIG_H_
#define CSP_EXAMPLE_CONFIG_H_

#include "stm32f777xx.h"
#include "MCU_Init.h"
#include "csp_example/inc/csp_example.h"
#include <stdarg.h>

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include "csp/csp_debug.h"
#pragma GCC diagnostic pop

/**
 * NOTE:
 *
 * the given values are valid only when used with default configuration
 * provided by EnduroSat in csp_example_can.c
 */
#define CSPEX_BAUD_CAN_1000_KBPS  (3)
#define CSPEX_BAUD_CAN_500_KBPS   (6)
#define CSPEX_HANDLE_CAN          (&hcan1)
#define CSPEX_INSTANCE_CAN        (CAN1)
#define CSPEX_HANDLE_UART         (&huart4)
#define CSPEX_INSTANCE_UART       (UART4)


/** used by driver in csp_example_uart.c */
#define CSPEX_BAUD_UART           (115200)                   /* user configurable */


/** used by driver in csp_example_can.c */
#define CPSEX_BAUD_CAN            (CSPEX_BAUD_CAN_1000_KBPS) /* user configurable */


/** driver FIFO length */
#define CSPEX_DRIVER_BUFFERS_UART (3)                        /* user configurable */
#define CSPEX_DRIVER_BUFFERS_CAN  (48)                       /* user configurable */


/** implementation required by CSP example */
bool cspex_uart_init(void* driver_data);


/** implementation required by CSP example */
bool cspex_uart_deinit(void* driver_data);


/** implementation required by CSP example */
int cspex_uart_transmit(void* driver_data, const uint8_t* data, size_t data_sz);


/** implementation required by CSP example */
bool cspex_uart_receive(cspex_rx_msg_uart_t* msg);


/** called from interrupt context when UART has received data */
void cspex_uart_rx_clbk(void);


/** called from interrupt context when UART driver error occurs */
void cspex_uart_err_clbk(void);


/** implementation required by CSP example */
bool cspex_can_init(void* driver_data);


/** implementation required by CSP example */
bool cspex_can_deinit(void* driver_data);


/** implementation required by CSP example */
int cspex_can_transmit(void* driver_data, uint32_t id, const uint8_t* data, uint8_t data_sz);


/** implementation required by CSP example */
bool cspex_can_receive(cspex_rx_msg_can_t* msg);


/** called from interrupt context when CAN has received data */
void cspex_can_rx_clbk(void);


/** called from interrupt context when CAN driver error occurs */
void cspex_can_err_clbk(void);


/** implementation required by CSP example */
void cspex_debug_hook(csp_debug_level_t level, const char *format, va_list args);


#endif /* CSP_EXAMPLE_CONFIG_H_ */
