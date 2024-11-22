/*!
 ********************************************************************************************
 * @file csp_example_debug.c
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
#include "../inc/csp_example_config.h"
#include "main.h"


#define _CSPEX_PRINT(fmt, ...)   fprintf(SYSCON, fmt ##__VA_ARGS__)
#define _CSPEX_VPRINT(fmt, args) vfprintf(SYSCON, fmt, args)


void cspex_debug_hook(csp_debug_level_t level, const char *format, va_list args)
{
    switch (level)
    {
        case CSP_INFO:
            _CSPEX_PRINT("[CSP_INFO]: ");
            break;
        case CSP_ERROR:
            _CSPEX_PRINT("[CSP_ERROR]: ");
            break;
        case CSP_WARN:
            _CSPEX_PRINT("[CSP_WARN]: ");
            break;
        case CSP_BUFFER:
            _CSPEX_PRINT("[CSP_BUFFER]: ");
            break;
        case CSP_PACKET:
            _CSPEX_PRINT("[CSP_PACKET]: ");
            break;
        case CSP_PROTOCOL:
            _CSPEX_PRINT("[CSP_PROTOCOL]: ");
            break;
        case CSP_LOCK:
            _CSPEX_PRINT("[CSP_LOCK]: ");
            break;
        default:
            return;
    }

    _CSPEX_VPRINT(format, args);
    _CSPEX_PRINT("\r\n");
}

