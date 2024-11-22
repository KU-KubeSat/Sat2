/**
 ******************************************************************************
 * @file    es_exeh.c
 * @brief   Exception handling module interface funtions definition.
 ******************************************************************************
 *
 * COPYRIGHT(c) 2020 Enduro Sat. All rights reserved.
 *
 ******************************************************************************
 */
#include "es_cdef.h"
#include "es_exeh.h"
#include "es_exeh_cfg.h"
#include "debug.h"
#include "string.h"
#include "cmsis_os.h"
#include "string.h"
#include "SdMngr.h"
#include "TaskMonitor.h"
#include "version.h"
#include "GlobalConfig.h"

#define EXEH_CURRENT_MODULE_ID      (eEXEHModuleID_ES_EXEH)

// Even EXEH will store its own exception in case of invalid parameters.
typedef enum
{
    eEXEHException_InvalidArgument,
} eEXEHModuleExceptions_t;

typedef struct
{
    uint16_t u16WarnLastFreeIndex;
    uint16_t u16ErrorLastFreeIndex;
    uint16_t u16FatalLastFreeIndex;
} sEXEHExceptionsModuleControlBlock;

typedef struct
{
    uint32_t u32WarnCount;
    uint32_t u32ErrorCount;
    uint32_t u32FatalCount;
} sEXEHExceptionsStats;

static const char cEXEHDFileSignature[] = "OBCEXEHD";

typedef struct
{
    char signature[sizeof(cEXEHDFileSignature)];
    VersionInfo_t verInfo;
    char szModuleSerial[50];
} PACKED sEXEHFileHeader;

typedef struct
{
    sEXEHFileHeader hdr;
    sEXEHModuleExceptions_t moduleExceptionStorage[eEXEHModuleID_COUNT];
} PACKED sEXEHDumpFileFormat;

static sEXEHModuleExceptions_t moduleExceptionStorage[eEXEHModuleID_COUNT];
static sEXEHDumpFileFormat exehFileTempBuf = { 0U };
static sEXEHExceptionsModuleControlBlock modCB[eEXEHModuleID_COUNT];
static sEXEHExceptionsStats modStats[eEXEHModuleID_COUNT];
static sEXEHExceptionsStats modStatsTotal;
static osThreadId_t EXEHPersistor_TaskHandle = NULL;
static sEXEHExceptionsStats lastStats;

/**
 * @brief Store an exception. Function expects valid parameters.
 * @param fp_eLevel - exception level
 * @param fp_eModuleID - id of the module reporting the exception
 * @param fp_s32ExceptionID - an user given exception ID.
 */
static void fs_vStoreException(eEXEHSeverityLevel_t fp_eLevel,
                               eEXEHModuleID_t      fp_eModuleID,
                               int32_t              fp_s32ExceptionID,
                               uint32_t             fp_u32FileLineNo);

static bool fs_bPersistExceptions(const TCHAR * const aEXEHFileName);

static void fs_vEXEHPersistor_Task();

// Store exception
void EXEH_vException(eEXEHSeverityLevel_t fp_eLevel,
                     eEXEHModuleID_t      fp_eModuleID,
                     int32_t              fp_s32ExceptionID,
                     uint32_t             fp_u32FileLineNo)
{
    bool bExceptionValid = true;
    
    DBG_TRACE("EXEH_vException: lev: %d, modid: %d, id: %d, line: %d\n", fp_eLevel, fp_eModuleID, fp_s32ExceptionID, fp_u32FileLineNo);

    // The severity level is known.
    fs_vStoreException(fp_eLevel,
                       fp_eModuleID,
                       fp_s32ExceptionID,
                       fp_u32FileLineNo);

    switch(fp_eLevel)
    {
        case eEXEHSeverity_Warning:
            EXEH_vHandleWarning();
            break;
        
        case eEXEHSeverity_Error:
            EXEH_vHandleError();
            break;
        
        case eEXEHSeverity_Fatal:
            EXEH_vHandleFatal();
            break;
        
        default:
        case eEXEHSeverity_Count:
            bExceptionValid = false;
            break;
    }
    
    if (!bExceptionValid)
    {
        EXEH_HANDLE(eEXEHSeverity_Error,
                    (int32_t)eEXEHException_InvalidArgument);
    }
}

// Get count of exceptions for a level including wrap-arounds (which were actually discarded)
uint32_t EXEH_u32GetExceptionCount(eEXEHSeverityLevel_t fp_eLevel)
{
    uint32_t u32Count = 0U;

    if ((fp_eLevel >= 0U) &&
        (fp_eLevel < eEXEHSeverity_Count))
    {
        switch (fp_eLevel)
        {
        case eEXEHSeverity_Warning:
            u32Count = modStatsTotal.u32WarnCount;
            break;

        case eEXEHSeverity_Error:
            u32Count = modStatsTotal.u32ErrorCount;
            break;

        case eEXEHSeverity_Fatal:
            u32Count = modStatsTotal.u32FatalCount;
            break;

        default:
        case eEXEHSeverity_Count:
            u32Count = 0U;
            break;
        }
    }
    else
        EXEH_HANDLE(eEXEHSeverity_Warning,
                    eEXEHException_InvalidArgument);

    return u32Count;
}

uint32_t EXEH_u32GetExceptions(const eEXEHModuleID_t fp_eModuleID,
                               uint8_t * const fp_psOutArray,
                               const uint32_t fp_u32OutArraySize)
{
    uint32_t u32ResultSize = 0U;
    UBaseType_t uxSavedInterruptStatus;

    if ((fp_eModuleID >= 0U) && (fp_eModuleID < eEXEHModuleID_COUNT) && (fp_psOutArray != NULL) && (fp_u32OutArraySize >= sizeof(sEXEHModuleExceptions_t)))
    {
        uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

        (void) memcpy(fp_psOutArray, &moduleExceptionStorage[fp_eModuleID], sizeof(sEXEHModuleExceptions_t));

        u32ResultSize = sizeof(sEXEHModuleExceptions_t);

        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    }

    return u32ResultSize;
}

// (Re)Initialize EXEH.
bool EXEH_bInit(void)
{
    uint16_t u16ModIndex;
    uint16_t u16SlotIdx;

    for (u16ModIndex = 0U; u16ModIndex < eEXEHModuleID_COUNT; u16ModIndex++)
    {
#if EXEH_CONFIG_STORE_FILENAME_ENABLED == 1
        (void)memset(moduleExceptionStorage[u16ModIndex].fileName, '\0', sizeof(moduleExceptionStorage[u16ModIndex].fileName));
#endif

        for (u16SlotIdx = 0U; u16SlotIdx < WARN_SLOTS_COUNT; u16SlotIdx++)
            memset(&moduleExceptionStorage[u16ModIndex].warnings[u16SlotIdx], 0U, sizeof(sEXEHException_t));

        for (u16SlotIdx = 0U; u16SlotIdx < ERROR_SLOTS_COUNT; u16SlotIdx++)
            memset(&moduleExceptionStorage[u16ModIndex].errors[u16SlotIdx], 0U, sizeof(sEXEHException_t));

        for (u16SlotIdx = 0U; u16SlotIdx < FATAL_SLOTS_COUNT; u16SlotIdx++)
            memset(&moduleExceptionStorage[u16ModIndex].fatals[u16SlotIdx], 0U, sizeof(sEXEHException_t));

        modCB[u16ModIndex].u16WarnLastFreeIndex = 0U;
        modCB[u16ModIndex].u16ErrorLastFreeIndex = 0U;
        modCB[u16ModIndex].u16FatalLastFreeIndex = 0U;

        modStats[u16ModIndex].u32WarnCount = 0U;
        modStats[u16ModIndex].u32ErrorCount = 0U;
        modStats[u16ModIndex].u32FatalCount = 0U;
    }

    modStatsTotal.u32WarnCount = 0U;
    modStatsTotal.u32ErrorCount = 0U;
    modStatsTotal.u32FatalCount = 0U;

    return true;
}

void EXEH_bStartPersistorTask(void)
{
#if (ENABLE_EXEH_PERSIST == 1)
    static const osThreadAttr_t EXEHPersistor_Task_attributes =
    {
        .name = "EXEHPersistorTask",
        .priority = (osPriority_t) osPriorityNormal,
        .stack_size = 2500
    };

    // initialize file buffer headers once before starting the task;
    // later, only exception data is updated
    (void) memcpy(&exehFileTempBuf.hdr.signature, &cEXEHDFileSignature, sizeof(exehFileTempBuf.hdr.signature));
    (void) memcpy(&exehFileTempBuf.hdr.verInfo, Version_getInfo(), sizeof(VersionInfo_t));
    (void) memset(&exehFileTempBuf.hdr.szModuleSerial, 0U, sizeof(exehFileTempBuf.hdr.szModuleSerial));
    (void) memcpy(&lastStats, &modStatsTotal, sizeof(sEXEHExceptionsStats));

    EXEHPersistor_TaskHandle = osThreadNew(fs_vEXEHPersistor_Task, NULL, &EXEHPersistor_Task_attributes);
    configASSERT(EXEHPersistor_TaskHandle);

    TaskMonitor_TaskInitialized(TASK_MONITOR_EXEH_PERSISTOR);
#endif
}

void EXEH_InitModuleFileName(eEXEHModuleID_t fp_eModuleID, const char * const fp_pcFileName)
{
#if EXEH_CONFIG_STORE_FILENAME_ENABLED == 1
    const char* pcLastPathSlash = NULL;

    if ((fp_eModuleID >= 0U) && (fp_eModuleID < eEXEHModuleID_COUNT))
    {
        // perform a quick check whether fileName has already been initialized
        if (moduleExceptionStorage[fp_eModuleID].fileName[0] == '\0')
        {
            // usually compilers put a full path to the file name if __FILE__ macro is used, hence only the file name should be extracted and
            // the position of the last forward slash character needs to be found
            pcLastPathSlash = strrchr(fp_pcFileName, '/');

            if (pcLastPathSlash == NULL)
            {
                // try backslash... most probably a Windows build...
                pcLastPathSlash = strrchr(fp_pcFileName, '\\');

                if (pcLastPathSlash == NULL)
                    pcLastPathSlash = fp_pcFileName;
            }
            else
                // skip backslash if found...
                pcLastPathSlash = (char*) (pcLastPathSlash + 1);

            (void)strncpy(moduleExceptionStorage[fp_eModuleID].fileName, pcLastPathSlash, sizeof(moduleExceptionStorage[fp_eModuleID].fileName));

            // in case pFileName points to a longer string which can fit in the internal storage, an explicit null termination is needed
            moduleExceptionStorage[fp_eModuleID].fileName[sizeof(moduleExceptionStorage[fp_eModuleID].fileName) - 1U] = '\0';
        }
    }
#endif
}

// Store an exception.
static void fs_vStoreException(eEXEHSeverityLevel_t fp_eLevel,
                               eEXEHModuleID_t      fp_eModuleID,
                               int32_t              fp_s32ExceptionID,
                               uint32_t             fp_u32FileLineNo)
{
#define INC_STATS_CNTR(cntr)    if ((cntr) < 0xFFFFFFFFU) (cntr)++;
    sEXEHException_t* pNewException = NULL;
    uint16_t* pu16LastFreeIndex = NULL;
    uint16_t u16SlotsCount = 0U;
    uint32_t* pu32StatsCounter = NULL;
    UBaseType_t uxSavedInterruptStatus;

    if ((fp_eLevel < eEXEHSeverity_Count) && (fp_eModuleID >= 0U) && (fp_eModuleID < eEXEHModuleID_COUNT))
    {
        uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

        switch (fp_eLevel)
        {
            case eEXEHSeverity_Warning:
                pNewException = &moduleExceptionStorage[fp_eModuleID].warnings[modCB[fp_eModuleID].u16WarnLastFreeIndex];
                pu16LastFreeIndex = &modCB[fp_eModuleID].u16WarnLastFreeIndex;
                u16SlotsCount = WARN_SLOTS_COUNT;
                pu32StatsCounter = &modStats[fp_eModuleID].u32WarnCount;
                INC_STATS_CNTR(modStatsTotal.u32WarnCount);

                break;

            case eEXEHSeverity_Error:
                pNewException = &moduleExceptionStorage[fp_eModuleID].errors[modCB[fp_eModuleID].u16ErrorLastFreeIndex];
                pu16LastFreeIndex = &modCB[fp_eModuleID].u16ErrorLastFreeIndex;
                u16SlotsCount = ERROR_SLOTS_COUNT;
                pu32StatsCounter = &modStats[fp_eModuleID].u32ErrorCount;
                INC_STATS_CNTR(modStatsTotal.u32ErrorCount);

                break;

            case eEXEHSeverity_Fatal:
                pNewException = &moduleExceptionStorage[fp_eModuleID].fatals[modCB[fp_eModuleID].u16FatalLastFreeIndex];
                pu16LastFreeIndex = &modCB[fp_eModuleID].u16FatalLastFreeIndex;
                u16SlotsCount = FATAL_SLOTS_COUNT;
                pu32StatsCounter = &modStats[fp_eModuleID].u32FatalCount;
                INC_STATS_CNTR(modStatsTotal.u32FatalCount);

                break;

            default:
            case eEXEHSeverity_Count:
                break;
        }

        if ((pNewException != NULL) && (pu16LastFreeIndex != NULL) && (u16SlotsCount > 0U))
        {
#if EXEH_CONFIG_EVENT_TIMESTAMP_ENABLED == 1
            pNewException->u32Timestamp = getSystemTickCount();
#endif
            pNewException->s32Exception = fp_s32ExceptionID;
            pNewException->u32LineNumber = fp_u32FileLineNo;

            (*pu16LastFreeIndex) = ((*pu16LastFreeIndex) + 1U) % u16SlotsCount;
            
            if (pu32StatsCounter != NULL)
                INC_STATS_CNTR(*pu32StatsCounter);
        }

        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    }
}

#if (ENABLE_EXEH_PERSIST == 1)
void EXEH_ForcePersistExceptions(void)
{
    static TCHAR aEXEHFileName[] = EXEH_DUMP_FILE_NAME;
    static bool bFileNameInitDone = false;

    if (((aEXEHFileName[EXEH_DUMP_FN_VOL_POS] == '0') && (SdMngr_GetCardState(SD_MNGR_CARD_0) == SDMNGR_CARD_STATE_READY)) ||
       ((aEXEHFileName[EXEH_DUMP_FN_VOL_POS] == '1') && (SdMngr_GetCardState(SD_MNGR_CARD_1) == SDMNGR_CARD_STATE_READY)))
    {
        if (memcmp(&lastStats, &modStatsTotal, sizeof(sEXEHExceptionsStats)) != 0)
        {
            if (!bFileNameInitDone)
            {
                FILINFO fInfo;

                // try to determine the next file name to use for dumping exceptions...
                // names can range from 0exeh_*.d to 9exeh_*.d;
                // the currently active dump ends with an EXEH_DUMP_FN_ACTIVE_MARKER, while older dumps end with an EXEH_DUMP_FN_INACTIVE_MARKER
                bool bFreeSlotFound = false;
                aEXEHFileName[EXEH_DUMP_FN_MARKER_POS] = EXEH_DUMP_FN_ACTIVE_MARKER;

                for (uint8_t fileId = 0U; fileId < EXEH_DUMP_FN_SLOTS; fileId++)
                {
                    // last active file ends with EXEH_DUMP_FN_ACTIVE_MARKER and here we are trying to find it
                    aEXEHFileName[EXEH_DUMP_FN_INDEX_POS] = (uint8_t) ('0' + fileId);

                    if (SdMngr_f_stat(aEXEHFileName, &fInfo) == FR_OK)
                    {
                        TCHAR aEXEHFileNameTemp[sizeof(EXEH_DUMP_FILE_NAME)];

                        (void) memcpy(aEXEHFileNameTemp, aEXEHFileName, sizeof(aEXEHFileNameTemp));

                        // preserve the existing file under a different name
                        aEXEHFileNameTemp[EXEH_DUMP_FN_MARKER_POS] = EXEH_DUMP_FN_INACTIVE_MARKER;
                        // delete any old dump with the same name...
                        (void) SdMngr_f_unlink(aEXEHFileNameTemp);
                        // rename current dump
                        (void) SdMngr_f_rename(aEXEHFileName, aEXEHFileNameTemp);

                        // move index position for new file and mark it as active
                        aEXEHFileName[EXEH_DUMP_FN_INDEX_POS] = '0' + ((fileId + 1U) % EXEH_DUMP_FN_SLOTS);

                        bFreeSlotFound = true;

                        break;
                    }
                }

                // if no active slot found, use zero slot...
                if (!bFreeSlotFound)
                    aEXEHFileName[EXEH_DUMP_FN_INDEX_POS] = (uint8_t) '0';

                bFileNameInitDone = true;
            }

            bool bPersistSuccessful = fs_bPersistExceptions(aEXEHFileName);

            // in case of failure, swap volumes and give it another try
            if (!bPersistSuccessful)
            {
                aEXEHFileName[EXEH_DUMP_FN_VOL_POS] = (aEXEHFileName[EXEH_DUMP_FN_VOL_POS] == '0') ? '1' : '0';

                (void) fs_bPersistExceptions(aEXEHFileName);
            }

            (void) memcpy(&lastStats, &modStatsTotal, sizeof(sEXEHExceptionsStats));
        }
    }
}

static bool fs_bPersistExceptions(const TCHAR * const aEXEHFileName)
{
    FRESULT fres = FR_INT_ERR;
    FIL fHnd;

    if (aEXEHFileName != NULL)
    {
        // attempt to delete any remnants of a previous dump
        (void) SdMngr_f_unlink(aEXEHFileName);

        fres = SdMngr_f_open(&fHnd, aEXEHFileName, FA_WRITE | FA_CREATE_ALWAYS);

        if (fres == FR_OK)
        {
            UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
            (void) memcpy(&exehFileTempBuf.moduleExceptionStorage, moduleExceptionStorage, sizeof(moduleExceptionStorage));
            taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);

            UINT bytesWritten = 0U;
            fres = SdMngr_f_write(&fHnd, &exehFileTempBuf, sizeof(sEXEHDumpFileFormat), &bytesWritten);

            if ((fres == FR_OK) && (bytesWritten != sizeof(sEXEHDumpFileFormat)))
                fres = FR_INT_ERR;

            (void) SdMngr_f_close(&fHnd);
        }
    }

    return (fres == FR_OK);
}

static void fs_vEXEHPersistor_Task()
{
    static uint8_t u8TaskCounter = 0U;

    TaskMonitor_TaskInitialized(TASK_MONITOR_EXEH_PERSISTOR); /* The task is initialized and is ready */

    while (true)
    {
        if (((u8TaskCounter++) % TASK_PERSISTOR_MAX_TASK_CNT) == 0)
        {
            EXEH_ForcePersistExceptions();
        }

        osDelay(TASK_PERSISTOR_PERIOD_MS);

        TaskMonitor_IamAlive(TASK_MONITOR_EXEH_PERSISTOR);
        vApplicationLowStackCheck(TASK_MONITOR_EXEH_PERSISTOR);
    }
}
#endif // #if (ENABLE_EXEH_PERSIST == 1)
