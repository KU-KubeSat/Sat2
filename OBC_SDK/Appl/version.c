/*
 * version.c
 *
 *  Created on: 23 Oct 2020
 *      Author: Ivan Petrov
 */

#include <stdio.h>
#include "GlobalConfig.h"
#include "version.h"
#include "EEPROM_emul.h"

const char caDefaultDevSerialPrefix[] = "SN_OBC_";
const VersionInfo_t versionInfo =
{
    .fwMajor = FW_MAJOR_REV,
    .fwMinor = FW_MINOR_REV,
    .buildDate = __DATE__,
    .buildDate_nullZ = '\0',
    .buildTime = __TIME__,
    .buildTime_nullZ = '\0',
    .buildConfigDesc = "STDPF\0",
    .appMode =
#if defined(BOOTLOADER)
        "BOOT\0",
#else
        "APPL\0",
#endif
};

const VersionInfo_t * Version_getInfo(void)
{
    return &versionInfo;
}

const char * Version_getDevSerial(void)
{
    return "0\0";
}
