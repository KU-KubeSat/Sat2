/*!
********************************************************************************************
* @file SdMngr.c
* @brief Manages common SD card functionalities and do some service operations
********************************************************************************************
* @author            Biser Kazakov / Ivan Petrov
* @version           1.0.0
* @date              2021.01.29
*
* @copyright         (C) Copyright Endurosat
*
*                    Contents and presentations are protected world-wide.
*                    Any kind of using, copying etc. is prohibited without prior permission.
*                    All rights - incl. industrial property rights - are reserved.
********************************************************************************************
*/

/*
********************************************************************************************
* INCLUDES
********************************************************************************************
*/
#include <SdMngr.h>             /* Standard types */
#include <cmsis_os2.h>          /*  */
#include <TaskMonitor.h>        /*  */
#include "es_cdef.h"
#include "es_exeh.h"
#include "string.h"
#include "ff_gen_drv.h"
#include "limits.h"
#include "ExtFram.h"
#include "ExtFram_AddrMap_Cfg.h"
#include "crc16-ccitt.h"
#include "debug.h"
#include <stdarg.h>

/*
********************************************************************************************
* INTERNAL DEFINES
********************************************************************************************
*/
//#define SDMNGR_STOP_ON_ERROR

#define SDMNGR_IN_USE_COUNTER_PERIOD (10)    // in ms
#define SDMNGR_MAX_IN_USE_TIMEOUT_MS (SD_TIMEOUT)                                        // should be a bit longer then max SD card timeout -> SD_TIMEOUT)
#define SDMNGR_MAX_IN_USE_TIMEOUT    (SD_TIMEOUT/SDMNGR_IN_USE_COUNTER_PERIOD)           // as counter not in ms
#define SDMNGR_MAX_REINIT_TIMEOUT_MS (SDMNGR_MAX_IN_USE_TIMEOUT_MS * SDMNGR_MAX_RETRY)    // should be a bit longer then max SD card timeout  -> SD_TIMEOUT)
#define SDMNGR_MAX_FORMAT_TIMEOUT_MS (SDMMC_MAXERASETIMEOUT * SDMNGR_MAX_RETRY)           // time = value * SDMNGR_IN_USE_COUNTER_PERIOD (should be a bit longer then max SD card timeout -> SD_TIMEOUT)

#define SDMNGR_MAX_RETRY             (3)     // number of retries for any failed operation

#define SDMNGT_TASK_TIME_CYCLE       (2000)

#define SDMNGR_BLOCKING_REINIT_PULL_INTERVAL    (10) //ms to check for a sd card init complete
#define SDMNGR_BLOCKING_FORMAT_PULL_INTERVAL    (100)  //ms to check for a sd card format complete

#define SDMNGR_PATH_MAX_LENTH       (50)

#define SDMNGR_DIRECT_F_CALL        (1)

#define INC_FF_CALL { ff_call_use_count++; if (ff_call_use_max_value < ff_call_use_count) ff_call_use_max_value = ff_call_use_count; }
#define DEC_FF_CALL { ff_call_use_count--; }

// APIs using FIL handles
#define ROUTE_FF_FILE_API(hnd, apiStatement)                \
        {                                                   \
            FRESULT fr = FR_INVALID_PARAMETER;              \
            if ((hnd) != NULL)                              \
            {                                               \
                if (fs_bIsFileHandleAccessAllowed(hnd))     \
                {                                           \
                    INC_FF_CALL;                            \
                    fr = (apiStatement);                    \
                    if (fr != FR_OK)                        \
                        fs_vHandleFFError(fr, (hnd));       \
                    DEC_FF_CALL;                            \
                }                                           \
            }                                               \
            return fr;                                      \
        }

// APIs using DIR handles
#define ROUTE_DIR_FILE_API(hnd, apiStatement)               \
        {                                                   \
            FRESULT fr = FR_INVALID_PARAMETER;              \
            if ((hnd) != NULL)                              \
            {                                               \
                if (fs_bIsDirHandleAccessAllowed((hnd)))    \
                {                                           \
                    INC_FF_CALL;                            \
                    fr = (apiStatement);                    \
                    if (fr != FR_OK)                        \
                        fs_vHandleDirError(fr, (hnd));      \
                    DEC_FF_CALL;                            \
                }                                           \
            }                                               \
            return fr;                                      \
        }

// APIs using TCHAR path
#define ROUTE_PATH_API(path, apiStatement)                  \
        {                                                   \
            FRESULT fr = FR_INVALID_PARAMETER;              \
            if ((path) != NULL)                             \
            {                                               \
                if (fs_bIsPathAccessAllowed((const TCHAR*)(path)))        \
                {                                           \
                    INC_FF_CALL;                            \
                    fr = (apiStatement);                    \
                    if (fr != FR_OK)                        \
                        fs_vHandlePathError(fr, (volatile TCHAR*) (path));    \
                    DEC_FF_CALL;                            \
                }                                           \
            }                                               \
            return fr;                                      \
        }

// Note: ensure this parameter is aligned to the maximum value in FRESULT enum
#define FFAPIERR_MAX_CNT          (FR_INVALID_PARAMETER + 1)
/*
********************************************************************************************
* INTERNAL TYPES DEFINITION
********************************************************************************************
*/

typedef enum
{
    eFileManagerVolumeId_SDCard0,
    eFileManagerVolumeId_SDCard1,
    eFileManagerVolumeId_MAX
} eFileManagerVolumeId_t;

typedef struct
{
    uint16_t u16Crc;
    uint16_t au16FFApiErrCounters[FFAPIERR_MAX_CNT];
} PACKED sPersistentStatisticCounters_t;

typedef struct
{
    // file or directory handle provided by FATFS driver
    void* handle;
    // ID of the card on which the handle is opened
    eFileManagerVolumeId_t cardId;
} sOpenHandleEntry;

/*
********************************************************************************************
* EXTERNAL VARIABLES DEFINITION
********************************************************************************************
*/
static uint8_t  SdMngr_sdCardReinit;
static uint8_t  SdMngr_sdCardReinitStat;
static uint8_t  SdMngr_sdCardFormat;
static FRESULT  SdMngr_sdCardFormatStat;

static uint8_t  SdMngr_Card_Eject_detected_Flag;

static uint8_t  SdMngr_formatBuffer[_MAX_SS];

static SdMngr_cardStatus_enum SdMngr_cardState[2];

static int16_t ff_call_use_count;
static int16_t ff_call_use_max_value;
static sPersistentStatisticCounters_t statCounters;
static bool bApiErrCountersChanged = false;

/*
********************************************************************************************
* INTERNAL (STATIC) VARIABLES DEFINITION/DECLARATION 
********************************************************************************************
*/
osThreadId_t SdMngr_TaskHandle;
static const osThreadAttr_t SdMngr_attributes = {
  .name = "SdMngr_Task",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 10
};

/*
********************************************************************************************
* INTERNAL (STATIC) ROUTINES DECLARATION
********************************************************************************************
*/
static void SdMngr_Task(void * argument);

static void SdMngr_UnhandledErrors(FIL* fp, DIR* dp);
static uint8_t HandleFFError(FRESULT error);

static void SdMngr_BlockingWaitReinit(uint32_t maxMsToWait, uint8_t card);
static void SdMngr_BlockingWaitFormat(uint32_t maxMsToWait, uint8_t card);

static inline eFileManagerVolumeId_t fs_eGetVolIdForPath(const TCHAR* path);

static void fs_vHandleFFError(FRESULT error, volatile FIL* fp);
static void fs_vHandleDirError(FRESULT error, volatile DIR* dp);
static void fs_vHandlePathError(FRESULT error, volatile TCHAR* path);

static void fs_UpdateStatsCrc(sPersistentStatisticCounters_t * const pStats);
static bool fs_IsStatsCrcValid(const sPersistentStatisticCounters_t * const pStats);

/*
********************************************************************************************
* EXTERNAL (NONE STATIC) ROUTINES DEFINITION
********************************************************************************************
*/
/*!
********************************************************************************************
* @brief Init routine for the SdMngr component
********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
********************************************************************************************
*/
void SdMngr_InitTask(void)
{
    SdMngr_sdCardReinit = 0;
    SdMngr_sdCardReinitStat = pdFALSE;

    SdMngr_sdCardFormat = 0;
    SdMngr_sdCardFormatStat = FR_OK;

    SdMngr_Card_Eject_detected_Flag = 0;

    SdMngr_cardState[0] = SDMNGR_CARD_STATE_NOT_INIT;
    SdMngr_cardState[1] = SDMNGR_CARD_STATE_NOT_INIT;

    // attempt to initialize counter values from FRAM
    DBG_ASSERT(sizeof(statCounters) < UCHAR_MAX);

    FRAM_ReadData(ADDR_SD_STATS, sizeof(statCounters), (uint8_t *) &statCounters);

    if (!fs_IsStatsCrcValid(&statCounters))
        (void) memset(&statCounters, 0U, sizeof(statCounters));

    SdMngr_TaskHandle = osThreadNew(SdMngr_Task, NULL, &SdMngr_attributes);
    configASSERT( SdMngr_TaskHandle );
}

/*!
*********************************************************************************************
* @brief Get the current state of a card
*********************************************************************************************
* @param[input]      card:
*                       + 1 - SD card 1
*                       + 2 - SD card 2
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
SdMngr_cardStatus_enum SdMngr_GetCardState(uint8_t card)
{
    if ((card == 1) || (card == 2))
        return SdMngr_cardState[card - 1];
    else
        return SDMNGR_CARD_STATE_NUMBER;
}

/*!
*********************************************************************************************
* @brief Checking if any SD card is ejected and inserted again.
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void SD_Card_Eject_Detection_Service(void)
{
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
    SD_TypeDef *Instance;
    uint8_t selectedSdCard;

    uint8_t filtered_pin_state = 0x55;
    if( SdMngr_Card_Eject_detected_Flag & 0x03)    // Is the card ejected or inserted in the slot
    {
        if(SdMngr_Card_Eject_detected_Flag & 0x01)
        {
            GPIOx = SD_DET_GPIO_Port;
            GPIO_Pin = SD_DET_Pin;
            selectedSdCard = 1;
            Instance = sd_driver_card_handler.Instance;
        }else{
            GPIOx = SD2_DET_GPIO_Port;
            GPIO_Pin = SD2_DET_Pin;
            selectedSdCard = 2;
            Instance = sd2_driver_card_handler.Instance;
        }

        do
        {
            filtered_pin_state <<= 1;
            filtered_pin_state |= HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
            osDelay(5);
        }while(( filtered_pin_state != 0 ) && (filtered_pin_state != 0xFF));

        if( 0xFF == filtered_pin_state )
        {
            if(FR_OK != SDMMC_CmdReadSingleBlock(Instance, 0))
            {
                SdMngr_sdCardReinit |= selectedSdCard;
            }
        }

        SdMngr_Card_Eject_detected_Flag &= ~selectedSdCard; // Clear the flag
    }
}

/*!
*********************************************************************************************
* @brief Set flag for detected ejecting or insertion of the card 1 in/out of the slot
*********************************************************************************************
* @param[input]      cards:
*                       + 1 - SD card 1
*                       + 2 - SD card 2
*                       + 3 - Both SD cards 1 and 2
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void SD_Card_Eject_Detected(uint8_t cards)
{
    SdMngr_Card_Eject_detected_Flag = cards & 0x03;

    if(cards & 1){
        SdMngr_cardState[0] = SDMNGR_CARD_STATE_EJECTED;
    }
    if(cards & 2){
        SdMngr_cardState[1] = SDMNGR_CARD_STATE_EJECTED;
    }

    SdMngr_NotifyTaskFromISR();
}

/*!
*********************************************************************************************
* @brief Set flag to rquest the SD card to be reinitialise
*********************************************************************************************
* @param[input]      cards:
*                       + 1 - SD card 1
*                       + 2 - SD card 2
*                       + 3 - Both SD cards 1 and 2
* @param[input]      blocking:
*                       + pdTRUE  - block the task until the Initialisation is complete
*                       + pdFALSE - set a request that will be done later ( during next execution of "SdMngr_Task()" )
* @param[output]     none
* @return            uint8_t:
*                       + pdTRUE - Successful
*                       + dpFALSE- Fail
* @note              Do not call from ISR
*********************************************************************************************
*/
uint8_t SdMngr_RequestSdCardInit(uint8_t cards, uint8_t blocking)
{
    uint8_t retVal = pdTRUE;

    SdMngr_sdCardReinit |= (cards & 0x03);

    if(cards & SD_MNGR_CARD_0){
        SdMngr_cardState[0] = SDMNGR_CARD_STATE_NOT_INIT;
    }

    if(cards & SD_MNGR_CARD_1){
        SdMngr_cardState[1] = SDMNGR_CARD_STATE_NOT_INIT;
    }

    SdMngr_NotifyTask();

    if(blocking)
    {
        SdMngr_BlockingWaitReinit( SDMNGR_MAX_IN_USE_TIMEOUT_MS + SDMNGR_MAX_REINIT_TIMEOUT_MS + 2000, cards);
        retVal = SdMngr_sdCardReinitStat;
    }

    return retVal;
}

/*!
*********************************************************************************************
* @brief Set flag to request the SD card to be formatted
*********************************************************************************************
* @param[input]      cards:
*                       + 1 - SD card 1
*                       + 2 - SD card 2
*                       + 3 - Both SD cards 1 and 2
* @param[input]      blocking:
*                       + pdTRUE  - block the task until the Initialisation is complete
*                       + pdFALSE - set a request that will be done later ( during next execution of "SdMngr_Task()" )
* @param[output]     none
* @return            none
* @note              Do not call from ISR
*********************************************************************************************
*/
FRESULT SdMngr_RequestSdCardFormat(uint8_t cards, uint8_t blocking)
{
    FRESULT retVal = FR_OK;

    SdMngr_sdCardFormat |= (cards & 0x03);

    if(cards & 1){
            SdMngr_cardState[0] = SDMNGR_CARD_STATE_NOT_FORMATTED;
        }
        if(cards & 2){
            SdMngr_cardState[1] = SDMNGR_CARD_STATE_NOT_FORMATTED;
        }

    SdMngr_NotifyTask();

    if(blocking)
    {
        SdMngr_BlockingWaitFormat( SDMNGR_MAX_IN_USE_TIMEOUT_MS + SDMNGR_MAX_FORMAT_TIMEOUT_MS + 2000, cards);
        retVal = SdMngr_sdCardFormatStat;
    }

    return retVal;
}

/*!
********************************************************************************************
* @brief Request immediate wake up of the task (only call from ISR)
********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
********************************************************************************************
*/
void SdMngr_NotifyTaskFromISR(void)
{
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    vTaskNotifyGiveFromISR( SdMngr_TaskHandle, &xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/*!
********************************************************************************************
* @brief Request immediate wake up of the task (Do not call from ISR)
********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
********************************************************************************************
*/
void SdMngr_NotifyTask(void)
{
    xTaskNotifyGive( SdMngr_TaskHandle );
}

uint16_t SdMngr_GetStatisticCounterById(FRESULT id)
{
    return (id < FFAPIERR_MAX_CNT) ? (statCounters.au16FFApiErrCounters[(uint8_t) id]) : (0U);
}

void SdMngr_GetRamStatisticCounters(sRamStats_t * const pStats)
{
    if (pStats != NULL)
    {
        taskENTER_CRITICAL();
        pStats->ff_call_use_count = ff_call_use_count;
        pStats->ff_call_use_max_value = ff_call_use_max_value;
        taskEXIT_CRITICAL();
    }
}

void SdMngr_ClearStatisticCounters(void)
{
    ff_call_use_count = 0U;
    ff_call_use_max_value = 0U;

    (void) memset(&statCounters, 0U, sizeof(statCounters));
    fs_UpdateStatsCrc(&statCounters);

    FRAM_WriteData(ADDR_SD_STATS, sizeof(statCounters), (uint8_t *) &statCounters);
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

static inline eFileManagerVolumeId_t fs_eGetVolForPhysDrv(BYTE physDriveNumber)
{
    switch (physDriveNumber)
    {
        case 0:
            return eFileManagerVolumeId_SDCard0;

        case 1:
            return eFileManagerVolumeId_SDCard1;

        default:
            return eFileManagerVolumeId_MAX;
    }
}

static bool fs_bIsFileHandleAccessAllowed(const FIL* fp)
{
    bool bRes = false;

    if (fp != NULL)
    {
        eFileManagerVolumeId_t volId = fs_eGetVolForPhysDrv((fp->obj.fs)->drv);

        if (volId != eFileManagerVolumeId_MAX)
            bRes = (SdMngr_cardState[volId] == SDMNGR_CARD_STATE_READY);
    }

    return bRes;
}

static bool fs_bIsDirHandleAccessAllowed(const DIR* dp)
{
    bool bRes = false;

    if (dp != NULL)
    {
        eFileManagerVolumeId_t volId = fs_eGetVolForPhysDrv((dp->obj.fs)->drv);

        if (volId != eFileManagerVolumeId_MAX)
            bRes = (SdMngr_cardState[volId] == SDMNGR_CARD_STATE_READY);
    }

    return bRes;
}

static bool fs_bIsPathAccessAllowed(const TCHAR* path)
{
    bool bRes = false;

    if (path != NULL)
    {
        eFileManagerVolumeId_t volId = fs_eGetVolIdForPath(path);

        if (volId != eFileManagerVolumeId_MAX)
            bRes = (SdMngr_cardState[volId] == SDMNGR_CARD_STATE_READY);
    }

    return bRes;
}



FRESULT SdMngr_f_open (FIL* fp, const TCHAR* path, BYTE mode)
{
    ROUTE_PATH_API(path, f_open(fp, path, mode));
}


FRESULT SdMngr_f_close (FIL* fp)
{
    ROUTE_FF_FILE_API(fp, f_close(fp));
}

FRESULT SdMngr_f_read (FIL* fp, void* buff, UINT btr, UINT* br)
{
    UINT fullSectorCount = btr / _MAX_SS;
    UINT partialCount = btr % _MAX_SS;
    UINT br1 = 0U, br2 = 0U;
    FRESULT fr = FR_INVALID_PARAMETER;

    if (br != NULL)
        *br = 0U;

    if (fp != NULL)
    {
        if (fs_bIsFileHandleAccessAllowed(fp))
        {
            INC_FF_CALL

            // split reads in sectors...
            if (fullSectorCount > 0)
            {
                fr = f_read(fp, buff, fullSectorCount * _MAX_SS, &br1);

                if (br != NULL)
                    *br = br1;
            }
            else
                fr = FR_OK;

            if (fr == FR_OK)
            {
                // ...read remaining partial sector separately
                if (partialCount > 0)
                {
                    fr = f_read(fp,
                                (void *) ((uint8_t *) buff + (fullSectorCount * _MAX_SS)),
                                partialCount,
                                &br2);

                    if ((fr == FR_OK) && (br != NULL))
                        *br += br2;
                }
            }

            DEC_FF_CALL
        }
    }

    if (fr != FR_OK)
        fs_vHandleFFError(fr, fp);

    return fr;
}

FRESULT SdMngr_f_write (FIL* fp, const void* buff, UINT btw, UINT* bw)
{
    UINT fullSectorCount = btw / _MAX_SS;
    UINT partialCount = btw % _MAX_SS;
    UINT bw1 = 0U, bw2 = 0U;
    FRESULT fr = FR_INVALID_PARAMETER;

    if (bw != NULL)
        *bw = 0U;

    if (fp != NULL)
    {
        if (fs_bIsFileHandleAccessAllowed(fp))
        {
            INC_FF_CALL

            // split reads in sectors...
            if (fullSectorCount > 0)
            {
                fr = f_write(fp, buff, fullSectorCount * _MAX_SS, &bw1);

                if (bw != NULL)
                    *bw = bw1;
            }
            else
                fr = FR_OK;

            if (fr == FR_OK)
            {
                // ...read remaining partial sector separately
                if (partialCount > 0)
                {
                    fr = f_write(fp,
                                 (void *) ((uint8_t *) buff + (fullSectorCount * _MAX_SS)),
                                 partialCount,
                                 &bw2);

                    if ((fr == FR_OK) && (bw != NULL))
                        *bw += bw2;
                }
            }

            DEC_FF_CALL
        }
    }

    if (fr != FR_OK)
        fs_vHandleFFError(fr, fp);

    return fr;
}

FRESULT SdMngr_f_lseek (FIL* fp, FSIZE_t ofs)
{
    ROUTE_FF_FILE_API(fp, f_lseek(fp, ofs));
}

FRESULT SdMngr_f_truncate (FIL* fp)
{
    ROUTE_FF_FILE_API(fp, f_truncate(fp));
}

FRESULT SdMngr_f_sync (FIL* fp)
{
    ROUTE_FF_FILE_API(fp, f_sync(fp));
}

FRESULT SdMngr_f_opendir (DIR* dp, const TCHAR* path)
{
    ROUTE_PATH_API(path, f_opendir(dp, path));
}

FRESULT SdMngr_f_closedir (DIR* dp)
{
    ROUTE_DIR_FILE_API(dp, f_closedir(dp));
}

FRESULT SdMngr_f_readdir (DIR* dp, FILINFO* fno)
{
    ROUTE_DIR_FILE_API(dp, f_readdir(dp, fno));
}

FRESULT SdMngr_f_findfirst (DIR* dp, FILINFO* fno, const TCHAR* path, const TCHAR* pattern)
{
    ROUTE_PATH_API(path, f_findfirst(dp, fno, path, pattern));
}

FRESULT SdMngr_f_findnext (DIR* dp, FILINFO* fno)
{
    ROUTE_DIR_FILE_API(dp, f_findnext(dp, fno));
}

FRESULT SdMngr_f_mkdir (const TCHAR* path)
{
    ROUTE_PATH_API(path, f_mkdir(path));
}

FRESULT SdMngr_f_unlink (const TCHAR* path)
{
    ROUTE_PATH_API(path, f_unlink(path));
}

FRESULT SdMngr_f_rename (const TCHAR* path_old, const TCHAR* path_new)
{
    ROUTE_PATH_API(path_old, f_rename(path_old, path_new));
}

FRESULT SdMngr_f_stat (const TCHAR* path, FILINFO* fno)
{
    ROUTE_PATH_API(path, f_stat(path, fno));
}

FRESULT SdMngr_f_chmod (const TCHAR* path, BYTE attr, BYTE mask)
{
    ROUTE_PATH_API(path, f_chmod(path, attr, mask));
}

FRESULT SdMngr_f_utime (const TCHAR* path, const FILINFO* fno)
{
    ROUTE_PATH_API(path, f_utime(path, fno));
}

FRESULT SdMngr_f_chdir (const TCHAR* path)
{
    ROUTE_PATH_API(path, f_chdir(path));
}

FRESULT SdMngr_f_chdrive (const TCHAR* path)
{
    ROUTE_PATH_API(path, f_chdrive(path));
}

FRESULT SdMngr_f_getfree (const TCHAR* path, DWORD* nclst, FATFS** fatfs)
{
    ROUTE_PATH_API(path, f_getfree(path, nclst, fatfs));
}

FRESULT SdMngr_f_mkfs (const TCHAR* path, BYTE opt, DWORD au, void* work, UINT len)
{
    ROUTE_PATH_API(path, f_mkfs(path, opt, au, work,  len));
}

int SdMngr_f_putc (TCHAR c, FIL* fp)
{
    INC_FF_CALL
    int ret = f_putc(c, fp);
    DEC_FF_CALL

    return ret;

}

int SdMngr_f_puts (const TCHAR* str, FIL* cp)
{
    INC_FF_CALL
    int ret = f_puts(str, cp);
    DEC_FF_CALL

    return ret;
}

TCHAR* SdMngr_f_gets (TCHAR* buff, int len, FIL* fp)
{
    INC_FF_CALL
    TCHAR* ret = f_gets(buff, len, fp);
    DEC_FF_CALL

    return ret;
}


int SdMngr_f_printf(FIL* f, const TCHAR* format, ...)
{
    va_list arg;
    va_start(arg, format);

    uint8_t* traverse;
    uint8_t TxBuffer[20];

    uint8_t argStr[20];
    uint8_t* pArgStr;

    unsigned int i;
    double temp_double;
    uint8_t* s;
    uint16_t size;

    FRESULT res;
    int written = 0;
    UINT bytes = 0;

    if (f == (FIL*)NULL)
    {
        return written;
    }

    if (format == NULL)
    {
        return written;
    }

    for (traverse = (uint8_t*)format; *traverse != '\0'; traverse++)
    {
        while ((*traverse != '%') && (*traverse != '\0'))
        {
            bytes = 0;
            res = SdMngr_f_write(f, traverse, 1, &bytes);
            if (res != FR_OK)
                goto write_error;
            traverse++;
            written += bytes;
        }

        if (*traverse == '\0')
        {
            break;
        }

        pArgStr = argStr;

        /* copy % and numbers */
        do
        {
            *pArgStr = *traverse;
            pArgStr++;
            traverse++;
        }
        while (((*traverse >= '0') && (*traverse <= '9')) || (*traverse == '.') || (*traverse == 'l'));
        /* Copy data format */
        *pArgStr = *traverse;
        pArgStr++;
        *pArgStr = '\0'; /* end of string */

        //Module 2: Fetching and executing arguments
        switch (*traverse)
        {
            case 'F':
            case 'f':
            {
                temp_double = va_arg(arg, double);     //Fetch Decimal/Integer argument
                size = sprintf((char*)TxBuffer, (char*)argStr, temp_double);
                bytes = 0;
                res = SdMngr_f_write(f, TxBuffer, size, &bytes);
                if (res != FR_OK)
                    goto write_error;
                written += bytes;
            }
            break;

            case 'c':
            case 'u':
            case 'i':
            case 'd':
            case 'o':
            case 'X':
            case 'x':
            {
                i = va_arg(arg, int);     //Fetch Decimal/Integer argument
                size = sprintf((char*)TxBuffer, (char*)argStr, i);
                bytes = 0;
                res = SdMngr_f_write(f, TxBuffer, size, &bytes);
                if (res != FR_OK)
                    goto write_error;
            }
            break;

            case 's':
            {
                s = (uint8_t*)va_arg(arg, char*);       //Fetch string
                size = strlen((char*)s);
                bytes = 0;
                res = SdMngr_f_write(f, s, size, &bytes);
                if (res != FR_OK)
                    goto write_error;

                written += bytes;
            }
            break;

            case '%':
            {
                i = '%';
                bytes = 0;
                res = SdMngr_f_write(f, (uint8_t*)&i, 1, &bytes);
                if (res != FR_OK)
                    goto write_error;

                written += bytes;
            }
            break;

            default:
            {
                //traverse++;
            }
            break;
        }
    }

    va_end(arg);

write_error:
    return written;
}

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

/*
********************************************************************************************
* INTERNAL (STATIC) ROUTINES DEFINITION
********************************************************************************************
*/
/*!
*********************************************************************************************
* @brief Task routing that serves requests for Reinit, Format, Eject/Inserted card
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
static void SdMngr_Task(void * argument)
{
    TaskMonitor_TaskInitialized(TASK_MONITOR_SD_MANAGER);

    for(;;)
    {
        SD_Card_Eject_Detection_Service();

        if (bApiErrCountersChanged)
        {
            taskENTER_CRITICAL();
            fs_UpdateStatsCrc(&statCounters);

            sPersistentStatisticCounters_t localTempBuf;
            (void) memcpy(&localTempBuf, &statCounters, sizeof(localTempBuf));
            taskEXIT_CRITICAL();

            FRAM_WriteData(ADDR_SD_STATS, sizeof(localTempBuf), (uint8_t *) &localTempBuf);

            bApiErrCountersChanged = false;
        }

        if (SdMngr_cardState[0] != SDMNGR_CARD_STATE_READY || SdMngr_cardState[1] != SDMNGR_CARD_STATE_READY)
        {
            //SdMngr_sdCardReinit &= 0x01; // Removes SD2 card because of RTC calibration
            if( SdMngr_sdCardReinit )
            {
                for(uint8_t i = 0; i < (SDMNGR_MAX_RETRY*2); i++)
                {
                    SdMngr_sdCardReinitStat = MCU_InitSdCard(SdMngr_sdCardReinit, pdTRUE);

                    if(SdMngr_sdCardReinitStat & SD_MNGR_CARD_0){
                        SdMngr_cardState[0] = SDMNGR_CARD_STATE_READY;
                    }

                    if(SdMngr_sdCardReinitStat & SD_MNGR_CARD_1){
                        SdMngr_cardState[1] = SDMNGR_CARD_STATE_READY;
                    }

                    if( SdMngr_sdCardReinitStat )
                    {
                        SdMngr_Card_Eject_detected_Flag &= ~SdMngr_sdCardReinitStat; // Clear the flags
                        SdMngr_sdCardReinit &= ~SdMngr_sdCardReinitStat;

                        if(SdMngr_sdCardReinit == 0)
                        {
                            break;
                        }
                    }else{
                        if(i == ((SDMNGR_MAX_RETRY*2)-1) )
                        {
                            EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_SDMNGR, eEXEH_SDMNGR_EXCEPTION_INIT_FAILED, __LINE__);
                            SdMngr_Card_Eject_detected_Flag = 0; // Clear the flags
                            SdMngr_sdCardReinit = 0;
                        }else{
                            osDelay(100);
                        }
                    }

                    TaskMonitor_IamAlive(TASK_MONITOR_SD_MANAGER);
                }
            }
            else if( SdMngr_sdCardFormat )
            {
                uint8_t card;
                char    volume[4];

                if(SdMngr_sdCardFormat & 0x01)
                {
                    card = SD_MNGR_CARD_0;
                    volume[0] = '0';
                }else{
                    card = SD_MNGR_CARD_1;
                    volume[0] = '1';
                }
                volume[1] = ':';
                volume[2] = '/';
                volume[3] = '\0';  //termination symbol

                for(uint8_t i = 0; i < SDMNGR_MAX_RETRY; i++)
                {
                    SdMngr_sdCardFormatStat = f_mkfs(volume, FM_FAT32, 0, SdMngr_formatBuffer, _MAX_SS);
                    if( FR_OK == SdMngr_sdCardFormatStat )
                    {
                        SdMngr_cardState[card-1] = SDMNGR_CARD_STATE_READY;
                        break;
                    }else{
                        if(i == (SDMNGR_MAX_RETRY-1))
                        {
                            EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_SDMNGR, eEXEH_SDMNGR_EXCEPTION_FORMAT_FAILED, __LINE__);
                        }else{
                            SdMngr_sdCardReinitStat = MCU_InitSdCard(SdMngr_sdCardFormat, pdFALSE);

                            if( pdFALSE == SdMngr_sdCardReinitStat )
                                SdMngr_sdCardReinitStat = MCU_InitSdCard(SdMngr_sdCardFormat, pdTRUE);
                        }
                    }

                    TaskMonitor_IamAlive(TASK_MONITOR_SD_MANAGER);
                }

                SdMngr_sdCardFormat &= ~card;
            }
        }

        (void)ulTaskNotifyTake( pdTRUE, SDMNGT_TASK_TIME_CYCLE );

        TaskMonitor_IamAlive(TASK_MONITOR_SD_MANAGER);
        vApplicationLowStackCheck(TASK_MONITOR_SD_MANAGER);
    }
}

/*!
*********************************************************************************************
* @brief Error that is not handled - so interrupt all request to the card.
*********************************************************************************************
* @param[input]      fp - pointer to a file structure (NULL accepted)
* @param[input]      dp - pointer to a directory structure (NULL accepted)
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
static void SdMngr_UnhandledErrors(FIL* fp, DIR* dp)
{
    EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_SDMNGR, eEXEH_SDMNGR_EXCEPTION_RECTED_OPPERATION, __LINE__);
    if(fp != NULL){
        SdMngr_f_close(fp);
    }

    if(dp != NULL){
        SdMngr_f_closedir(dp);
    }
#ifdef DEBUG_ENABLED
    #ifdef SDMNGR_STOP_ON_ERROR
        Error_Handler();
    #endif
#endif
}


/*!
*********************************************************************************************
* @brief Handle errors from the SD card
*********************************************************************************************
* @param[input]      error - the error to be handled
* @param[input]      path - path to the file/directory used for last operation
* @param[output]     none
* @return            uint8_t:
*                       + pdTRUE - the card is available
*                       + pdFALSE - the card is not available
* @note              none
*********************************************************************************************
*/

static void fs_vHandleFFError(volatile FRESULT error, volatile FIL* fp)
{
    (void) fp;

    HandleFFError(error);
}

static void fs_vHandleDirError(volatile FRESULT error, volatile DIR* dp)
{
    (void) dp;

    HandleFFError(error);
}

static void fs_vHandlePathError(volatile FRESULT error, volatile TCHAR* path)
{
    (void) path;

    HandleFFError(error);
}


static uint8_t HandleFFError(volatile FRESULT error)
{
    uint8_t retVal = pdFALSE;

    if (error < FFAPIERR_MAX_CNT)
    {
        if (statCounters.au16FFApiErrCounters[error] < USHRT_MAX)
        {
            statCounters.au16FFApiErrCounters[error]++;
            bApiErrCountersChanged = true;
        }
    }

    switch(error)
    {
        case FR_DISK_ERR:   /* (1) A hard error occurred in the low level disk I/O layer */
        {
            EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_SDMNGR, eEXEH_SDMNGR_EXCEPTION_FR_DISK_ERR, __LINE__);
        }break;

        case FR_INT_ERR:    /* (2) Assertion failed */
        {
            EXEH_vException(eEXEHSeverity_Fatal, eEXEHModuleID_SDMNGR, eEXEH_SDMNGR_EXCEPTION_FR_INT_ERR, __LINE__);
            //In this case look for stack overflow or pointer writing out of range
        }break;

        case FR_NOT_READY:  /* (3) The physical drive cannot work */
        {
            //“The disk drive cannot work due to incorrect medium removal or disk initialize function failed.”
            EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_SDMNGR, eEXEH_SDMNGR_EXCEPTION_FR_NOT_READY, __LINE__);
        }break;

        case FR_NO_FILE:    /* (4) Could not find the file */
        {
            //wrong file name
            //EXEH_vException(eEXEHSeverity_Warning, eEXEHModuleID_SDMNGR, eEXEH_SDMNGR_EXCEPTION_FR_NO_FILE, __LINE__);    //Normal to happen when testing if a file is existing
        }break;

        case FR_NO_PATH:    /* (5) Could not find the path */
        {
            //wrong path
            EXEH_vException(eEXEHSeverity_Warning, eEXEHModuleID_SDMNGR, eEXEH_SDMNGR_EXCEPTION_FR_NO_PATH, __LINE__);
        }break;

        case FR_INVALID_NAME:   /* (6) The path name format is invalid */
        {
            //wrong characters/length for file name
            EXEH_vException(eEXEHSeverity_Warning, eEXEHModuleID_SDMNGR, eEXEH_SDMNGR_EXCEPTION_FR_INVALID_NAME, __LINE__);
        }break;

        case FR_DENIED:/* (7) Access denied due to prohibited access or directory full */
        {
            // request for operation with a read only file
            EXEH_vException(eEXEHSeverity_Warning, eEXEHModuleID_SDMNGR, eEXEH_SDMNGR_EXCEPTION_FR_DENIED, __LINE__);
        }break;

        case FR_EXIST:/* (8) Access denied due to prohibited access */
        {
            //Name collision. An object with the same name is already existing in the directory.
            EXEH_vException(eEXEHSeverity_Warning, eEXEHModuleID_SDMNGR, eEXEH_SDMNGR_EXCEPTION_FR_EXIST, __LINE__);
        }break;

        case FR_INVALID_OBJECT: /* (9) The file/directory object is invalid */
        {
            //Access to already closed file or never open one
            EXEH_vException(eEXEHSeverity_Warning, eEXEHModuleID_SDMNGR, eEXEH_SDMNGR_EXCEPTION_FR_INVALID_OBJECT, __LINE__);
        }break;

        case FR_WRITE_PROTECTED:    /* (10) The physical drive is write protected */
        {
            //A write mode operation against the write-protected media.
            EXEH_vException(eEXEHSeverity_Warning, eEXEHModuleID_SDMNGR, eEXEH_SDMNGR_EXCEPTION_FR_WRITE_PROTECTED, __LINE__);
        }break;

        case FR_INVALID_DRIVE:  /* (11) The logical drive number is invalid */
        {
            //Invalid drive number is specified in the path name or a null pointer is given as the path name
            EXEH_vException(eEXEHSeverity_Warning, eEXEHModuleID_SDMNGR, eEXEH_SDMNGR_EXCEPTION_FR_INVALID_DRIVE, __LINE__);
        }break;

        case FR_NOT_ENABLED:    /* (12) The volume has no work area */
        {
            //Work area for the logical drive has not been registered by f_mount function
            EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_SDMNGR, eEXEH_SDMNGR_EXCEPTION_FR_NOT_ENABLED, __LINE__);
        }break;

        case FR_NO_FILESYSTEM:  /* (13) There is no valid FAT volume */
        {
            EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_SDMNGR, eEXEH_SDMNGR_EXCEPTION_FR_NO_FILESYSTEM, __LINE__);
        }break;

        case FR_MKFS_ABORTED:   /* (14) The f_mkfs() aborted due to any problem */
        {
            EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_SDMNGR, eEXEH_SDMNGR_EXCEPTION_FR_NOT_ENABLED, __LINE__);
        }break;

        case FR_TIMEOUT:    /* (15) Could not get a grant to access the volume within defined period */
        {
            EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_SDMNGR, eEXEH_SDMNGR_EXCEPTION_FR_TIMEOUT, __LINE__);
        }break;

        case FR_LOCKED: /* (16) The operation is rejected according to the file sharing policy */
        {
            EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_SDMNGR, eEXEH_SDMNGR_EXCEPTION_FR_LOCKED, __LINE__);
        }break;

        case FR_NOT_ENOUGH_CORE:    /* (17) LFN working buffer could not be allocated */
        {
            //if long file names are used, enter here for to short buffer to fit the name
            EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_SDMNGR, eEXEH_SDMNGR_EXCEPTION_FR_NOT_ENOUGH_CORE, __LINE__);
        }break;

        case FR_TOO_MANY_OPEN_FILES:    /* (18) Number of open files > _FS_LOCK */
        {
            EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_SDMNGR, eEXEH_SDMNGR_EXCEPTION_FR_TOO_MANY_OPEN_FILES, __LINE__);
        }break;

        case FR_INVALID_PARAMETER:  /* (19) Given parameter is invalid */
        {
            EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_SDMNGR, eEXEH_SDMNGR_EXCEPTION_FR_INVALID_PARAMETER, __LINE__);
        }break;

        default:
        {
            Error_Handler();
        }
    }

    if(retVal == pdFALSE)   //During development stop here to see the problem
    {
        if((error!= FR_NO_FILE)&&(error!= FR_NO_PATH))  //Expected to happen errors
        {
            SdMngr_UnhandledErrors(NULL, NULL);
        }
    }

    return retVal;
}

/*!
*********************************************************************************************
* @brief Error that is not handled - so interrupt all request to the card.
*********************************************************************************************
* @param[input]      path - path to the file/directory used for last operation
* @param[output]     none
* @return            uint8_t: number of the logical drive for that SD card and path
* @note              none
*********************************************************************************************
*/
static inline eFileManagerVolumeId_t fs_eGetVolIdForPath(const TCHAR* path)
{
    eFileManagerVolumeId_t res = eFileManagerVolumeId_MAX;

    if (path != NULL)
    {
        switch (path[0])
        {
            case '0':
                res = eFileManagerVolumeId_SDCard0;
            break;

            case '1':
                res = eFileManagerVolumeId_SDCard1;
            break;

            default:
                res = eFileManagerVolumeId_SDCard0;
            break;
        }
    }

    return res;
}

/*!
*********************************************************************************************
* @brief Waits to finish the already requested ReInit of a SD card
*********************************************************************************************
* @param[input]      fp - pointer to a file structure (NULL accepted)
* @param[input]      pathNumber - index of the record with that file pointer (from SdMngr_FindPath() )
* @param[input]      ofs - offset from the beginning of the file
* @param[output]     none
* @return            uint8_t - index of the record with that file pointer
* @note              none
*********************************************************************************************
*/
static void SdMngr_BlockingWaitReinit(uint32_t maxMsToWait, uint8_t card)
{
    uint16_t waitedTime = 0;

    while(waitedTime < maxMsToWait)
    {
        if( (SdMngr_sdCardReinit & card) == 0)
        {
            break;
        }
        waitedTime += SDMNGR_BLOCKING_REINIT_PULL_INTERVAL;
        osDelay(SDMNGR_BLOCKING_REINIT_PULL_INTERVAL);
    }
}


/*!
*********************************************************************************************
* @brief Waits to finish the already requested Format of a SD card
*********************************************************************************************
* @param[input]      fp - pointer to a file structure (NULL accepted)
* @param[input]      pathNumber - index of the record with that file pointer (from SdMngr_FindPath() )
* @param[input]      ofs - offset from the beginning of the file
* @param[output]     none
* @return            uint8_t - index of the record with that file pointer
* @note              none
*********************************************************************************************
*/
static void SdMngr_BlockingWaitFormat(uint32_t maxMsToWait, uint8_t card)
{
    uint16_t waitedTime = 0;

    while(waitedTime < maxMsToWait)
    {
        if( (SdMngr_sdCardFormat & card) == 0)
        {
            break;
        }
        waitedTime += SDMNGR_BLOCKING_FORMAT_PULL_INTERVAL;
        osDelay(SDMNGR_BLOCKING_FORMAT_PULL_INTERVAL);
    }
}

static void fs_UpdateStatsCrc(sPersistentStatisticCounters_t * const pStats)
{
    DBG_ASSERT(pStats != NULL);

    pStats->u16Crc = crc16_ccitt_table((const uint8_t *) &pStats->au16FFApiErrCounters,
                                       sizeof(pStats->au16FFApiErrCounters));
}

static bool fs_IsStatsCrcValid(const sPersistentStatisticCounters_t * const pStats)
{
    uint16_t u16Crc;

    DBG_ASSERT(pStats != NULL);

    u16Crc = crc16_ccitt_table((const uint8_t *) &pStats->au16FFApiErrCounters,
                               sizeof(pStats->au16FFApiErrCounters));

    return (u16Crc == pStats->u16Crc);
}

/* **************************************************************************************** */
