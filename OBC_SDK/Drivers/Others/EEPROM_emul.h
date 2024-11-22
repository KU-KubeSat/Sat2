/*!
********************************************************************************************
* @file EEPROM_Emul.h
* @brief Header of EEPROM_Emul.c
********************************************************************************************
* @author            Vassil Milev
* @version           1.0.0
* @date              2019.06.24
*
* @copyright         (C) Copyright Endurosat
*
*                    Contents and presentations are protected world-wide.
*                    Any kind of using, copying etc. is prohibited without prior permission.
*                    All rights - incl. industrial property rights - are reserved.
*
* @history
* @revision{         1.0.0  , 2019.06.24, author Vassil Milev, Initial revision }
* @endhistory
********************************************************************************************
*/
#ifndef EEPROM_EMUL_H
#define EEPROM_EMUL_H

/*
*********************************************************************************************
* INCLUDES
*********************************************************************************************
*/
#include "Svc_RTC.h"
#include "AntUHF.h"
#include <PwrMng.h>


/*
*********************************************************************************************
* EXTERNAL DEFINES
*********************************************************************************************
*/
#define EEPROM_EMUL_USED_SECTOR             (FLASH_SECTORS_COUNT - 2U)
#define EEPROM_EMUL_SELECT_SECOND_BANK      (0x10)
#define EEPROM_SECTOR_SIZE                  (1024)                            /* One sector of the EEPROM is considered one piece of memory that is flashed at once. Keep the size of "EEPROM_INFO_Struct" with the size of one EEPROM sector */

/*
*********************************************************************************************
* EXTERNAL TYPES DECLARATIONS
*********************************************************************************************
*/

#define EEP_USER_DATA_LIST \
    uint32_t                       Pattern;                                          /*Magic number that is always the same and can be used for searching of valid data in the Flash memory. Keep this in the beginning of the structure */                               \
    uint16_t                       nvmVersion;                                       /*NVM "Major" version (EEPROM_EMUL_CODE_VERSION) - have to matched to the Code to be valid. Change only if the new NVM structure is not going to be compatible with the code !!! */  \
    boot_struct                    ResetFaults;                                      /*Keeps counters for various reset reasons */                                                                                                                                        \
    AntUHFSettings_Struct          AntUHFSettings;                                   /*Settings for the UHF antenna */                                                                                                                                                    \
    uint8_t                        I2cPullUpResistors[MCU_INIT_I2C_MAX_NUMBER];      /*Configure the pull up resistors of the I2C to be enabled or disabled */

// This structure is a temporary workaround to enable automatic calculation of the
// EEPROM_INFO_Struct::ReservedBytes[] array size.
typedef struct
{
    EEP_USER_DATA_LIST
}__attribute__((__packed__)) sEepUserData_t;

/* Keeps data about the Errors, faults, CRC of the Flash memory - overall size = 512 bytes which results in 256 writes in one block before being deleted to start over from the beginning */
typedef struct {
    EEP_USER_DATA_LIST
    uint8_t            ReservedBytes[EEPROM_SECTOR_SIZE - sizeof(sEepUserData_t) - sizeof(uint32_t)];                                /* The size of the structure must remain the same as in the bootloader. If some variables are needed reduce that reserved bytes and add variables with the same number of bytes */
    uint32_t           DataCRC;		                                     /* CRC of all the data into the structure. Keep this variable at the end of the structure */
}__attribute__((__packed__)) EEPROM_INFO_Struct;

/*
*********************************************************************************************
* EXTERNAL VARIABLES DECLARATIONS
*********************************************************************************************
*/
extern EEPROM_INFO_Struct * EEPROM_emul_pDataInfo;
extern EEPROM_INFO_Struct   EEPROM_emul_DataTemp;

/*
*********************************************************************************************
* EXTERNAL ROUTINES DECLARATIONS
*********************************************************************************************
*/
void EEPROM_Emul_Init(void);
void EEPROM_Emul_SyncInfo(void);

#endif    /* EEPROM_EMUL_H */
/* **************************************************************************************** */
