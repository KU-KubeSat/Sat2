/*!
********************************************************************************************
* @file version.h
* @brief Keeps all version of the device
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
#ifndef VERSION_H
#define VERSION_H

/*
*********************************************************************************************
* INCLUDES
*********************************************************************************************
*/
#include <stdint.h>

/*
*********************************************************************************************
* EXTERNAL DEFINES
*********************************************************************************************
*/
#define BUILD_DATE_SIZE     ((uint8_t) 11)
#define BUILD_TIME_SIZE     ((uint8_t)  8)
#define BUILD_CFG_DESC_SIZE ((uint8_t) 10)
#define APPMODE_SIZE        ((uint8_t)  5)

/*
*********************************************************************************************
* EXTERNAL TYPES DECLARATIONS
*********************************************************************************************
*/
typedef struct
{
    uint8_t fwMajor;
    uint8_t fwMinor;
    char buildDate[BUILD_DATE_SIZE];
    char buildDate_nullZ;
    char buildTime[BUILD_TIME_SIZE];
    char buildTime_nullZ;
    char buildConfigDesc[BUILD_CFG_DESC_SIZE];
    char appMode[APPMODE_SIZE];
} __attribute__((__packed__)) VersionInfo_t;

/*
*********************************************************************************************
* EXTERNAL VARIABLES DECLARATIONS
*********************************************************************************************
*/
/* No External variables declarations */

/*
*********************************************************************************************
* EXTERNAL ROUTINES DECLARATIONS
*********************************************************************************************
*/
const VersionInfo_t * Version_getInfo(void);
const char * Version_getDevSerial(void);

#endif    /* VERSION_H */
/* **************************************************************************************** */
