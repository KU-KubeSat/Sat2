/**
 ******************************************************************************
 * @file    cdef.h
 * @brief   Global common type and macro definitions.
 *
 * TODOs:
 * 1. Disable CDEF_IO_WEAK qualifier when building in release.
 ******************************************************************************
 *
 * COPYRIGHT(c) 2020 Enduro Sat. All rights reserved.
 *
 ******************************************************************************
 */
#ifndef OBC_ES_CDEF_H
#define OBC_ES_CDEF_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>


typedef int32_t    S32;
typedef uint32_t   U32;

typedef int16_t    S16;
typedef uint16_t   U16;

typedef int8_t     S8;
typedef uint8_t    U8;

typedef uint64_t   U64;
typedef int64_t    S64;

typedef float      F32;
typedef double     F64;

typedef bool       BOOL;

#ifndef WEAK_CBK
  #define WEAK_CBK __attribute__((weak))
#endif

#ifndef PACKED
  #define PACKED   __attribute__((__packed__))
#endif

#endif

