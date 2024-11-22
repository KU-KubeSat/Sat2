/**
 ******************************************************************************
 * @file    ex_exeh_cfg.c
 * @brief   Project specific EXEH definitions
 ******************************************************************************
 *
 * COPYRIGHT(c) 2020 Enduro Sat. All rights reserved.
 *
 ******************************************************************************
 */
#include "es_cdef.h"
#include "es_exeh_cfg.h"
#include "main.h"


void EXEH_vHandleWarning(void)
{

}


void EXEH_vHandleError(void)
{

}


void EXEH_vHandleFatal(void)
{
    Error_Handler();
}
