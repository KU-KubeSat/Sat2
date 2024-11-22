/*!
********************************************************************************************
* @file SdMngr.h
* @brief Header of SdMngr.c
********************************************************************************************
* @author            Vassil Milev
* @version           1.0.0
* @date              2021.01.29
*
* @copyright         (C) Copyright Endurosat
*
*                    Contents and presentations are protected world-wide.
*                    Any kind of using, copying etc. is prohibited without prior permission.
*                    All rights - incl. industrial property rights - are reserved.
*
* @history
* @revision{         1.0.0  , 2021.01.28, author Vassil Milev, Initial revision }
* @endhistory
********************************************************************************************
*/
#ifndef SDMNGR_H
#define SDMNGR_H

/*
********************************************************************************************
* INCLUDES
********************************************************************************************
*/
#include "fatfs.h"                 /*  */

/*
********************************************************************************************
* EXTERNAL DEFINES
********************************************************************************************
*/
#define SdMngr_f_size(a)       f_size(a)

#define SD_MNGR_CARD_0         (0x01U)
#define SD_MNGR_CARD_1         (0x02U)
#define SD_MNGR_CARD_BOTH      (SD_MNGR_CARD_0 | SD_MNGR_CARD_1)

/*
********************************************************************************************
* EXTERNAL TYPES DECLARATIONS
********************************************************************************************
*/
typedef enum{
    SDMNGR_CARD_STATE_READY,
    SDMNGR_CARD_STATE_NOT_INIT,
    SDMNGR_CARD_STATE_NOT_FORMATTED,
    SDMNGR_CARD_STATE_EJECTED,
    SDMNGR_CARD_STATE_ERROR,

    SDMNGR_CARD_STATE_NUMBER
}SdMngr_cardStatus_enum;

typedef struct
{
    int16_t ff_call_use_count;
    int16_t ff_call_use_max_value;
} sRamStats_t;

/*
********************************************************************************************
* EXTERNAL VARIABLES DECLARATIONS
********************************************************************************************
*/
/* No External variables declarations */

/*
********************************************************************************************
* EXTERNAL ROUTINES DECLARATIONS 
********************************************************************************************
*/
void SdMngr_InitTask(void);
SdMngr_cardStatus_enum SdMngr_GetCardState(uint8_t card);

uint8_t SdMngr_RequestSdCardInit(uint8_t cards, uint8_t blocking);
FRESULT SdMngr_RequestSdCardFormat(uint8_t cards, uint8_t blocking);

void SD_Card_Eject_Detection_Service(void);
void SD_Card_Eject_Detected(uint8_t cards);

void SdMngr_NotifyTaskFromISR(void);
void SdMngr_NotifyTask(void);

uint16_t SdMngr_GetStatisticCounterById(FRESULT id);
void SdMngr_GetRamStatisticCounters(sRamStats_t * const pStats);
void SdMngr_ClearStatisticCounters(void);

FRESULT SdMngr_f_open (FIL* fp, const TCHAR* path, BYTE mode);                                 /* Open or create a file */
FRESULT SdMngr_f_close (FIL* fp);                                                              /* Close an open file object */
FRESULT SdMngr_f_read (FIL* fp, void* buff, UINT btr, UINT* br);                               /* Read data from the file */
FRESULT SdMngr_f_write (FIL* fp, const void* buff, UINT btw, UINT* bw);                        /* Write data to the file */
FRESULT SdMngr_f_lseek (FIL* fp, FSIZE_t ofs);                                                 /* Move file pointer of the file object */
FRESULT SdMngr_f_truncate (FIL* fp);                                                           /* Truncate the file */
FRESULT SdMngr_f_sync (FIL* fp);                                                               /* Flush cached data of the writing file */
FRESULT SdMngr_f_opendir (DIR* dp, const TCHAR* path);                                         /* Open a directory */
FRESULT SdMngr_f_closedir (DIR* dp);                                                           /* Close an open directory */
FRESULT SdMngr_f_readdir (DIR* dp, FILINFO* fno);                                              /* Read a directory item */
FRESULT SdMngr_f_findfirst (DIR* dp, FILINFO* fno, const TCHAR* path, const TCHAR* pattern);   /* Find first file */
FRESULT SdMngr_f_findnext (DIR* dp, FILINFO* fno);                                             /* Find next file */
FRESULT SdMngr_f_mkdir (const TCHAR* path);                                                    /* Create a sub directory */
FRESULT SdMngr_f_unlink (const TCHAR* path);                                                   /* Delete an existing file or directory */
FRESULT SdMngr_f_rename (const TCHAR* path_old, const TCHAR* path_new);                        /* Rename/Move a file or directory */
FRESULT SdMngr_f_stat (const TCHAR* path, FILINFO* fno);                                       /* Get file status */
FRESULT SdMngr_f_chmod (const TCHAR* path, BYTE attr, BYTE mask);                              /* Change attribute of a file/dir */
FRESULT SdMngr_f_utime (const TCHAR* path, const FILINFO* fno);                                /* Change timestamp of a file/dir */
FRESULT SdMngr_f_chdir (const TCHAR* path);                                                    /* Change current directory */
FRESULT SdMngr_f_chdrive (const TCHAR* path);                                                  /* Change current drive */
FRESULT SdMngr_f_getcwd (TCHAR* buff, UINT len);                                               /* Get current directory */
FRESULT SdMngr_f_getfree (const TCHAR* path, DWORD* nclst, FATFS** fatfs);                     /* Get number of free clusters on the drive */
FRESULT SdMngr_f_getlabel (const TCHAR* path, TCHAR* label, DWORD* vsn);                       /* Get volume label */
FRESULT SdMngr_f_setlabel (const TCHAR* label);                                                /* Set volume label */
//not supported: FRESULT SdMngr_f_forward (FIL* fp, UINT(*func)(const BYTE*,UINT), UINT btf, UINT* bf);         /* Forward data to the stream */
FRESULT SdMngr_f_expand (FIL* fp, FSIZE_t szf, BYTE opt);                                      /* Allocate a contiguous block to the file */
FRESULT SdMngr_f_mount (FATFS* fs, const TCHAR* path, BYTE opt);                               /* Mount/Unmount a logical drive */
FRESULT SdMngr_f_mkfs (const TCHAR* path, BYTE opt, DWORD au, void* work, UINT len);           /* Create a FAT volume */
FRESULT SdMngr_f_fdisk (BYTE pdrv, const DWORD* szt, void* work);                              /* Divide a physical drive into some partitions */
int SdMngr_f_putc (TCHAR c, FIL* fp);                                                          /* Put a character to the file */
int SdMngr_f_puts (const TCHAR* str, FIL* cp);                                                 /* Put a string to the file */
int SdMngr_f_printf (FIL* fp, const TCHAR* str, ...);                                          /* Put a formatted string to the file */
TCHAR* SdMngr_f_gets (TCHAR* buff, int len, FIL* fp);                                          /* Get a string from the file */

#endif    /* SDMNGR_H */
/* **************************************************************************************** */
