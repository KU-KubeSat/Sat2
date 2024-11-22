/**
 ******************************************************************************
 * @file    es_exeh.h
 * @brief   Exception storage and handling. Used for storing and reporting
 *    exceptions.
 ******************************************************************************
 *
 * COPYRIGHT(c) 2020 EnduroSat. All rights reserved.
 *
 ******************************************************************************
 */
#ifndef OBC_ES_EXEH_H
#define OBC_ES_EXEH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "es_cdef.h"
#include "es_exeh_cfg.h"


/**
 * @brief this macro allows specifying the logical module (M parameter) to which the exception relates
 */
#define EXEH_HANDLE_EX(L, M, E)             {                                            \
                                                EXEH_InitModuleFileName(M, __FILE__);    \
                                                EXEH_vException(L, M, E, __LINE__);      \
                                            }

/**
 * @brief This is a simpler-to-use version of the EXEH_HANDLE_EX(...) macro where the module ID is not explicitly
 *        specified and is assumed to be defined in the same module under the name of EXEH_CURRENT_MODULE_ID
 */
#define EXEH_HANDLE(L, E)                  EXEH_vException(L, EXEH_CURRENT_MODULE_ID, E, __LINE__);

#define EXEH_INIT_MODULE_FILENAME()        EXEH_InitModuleFileName(EXEH_CURRENT_MODULE_ID, __FILE__) 
#define EXEH_INIT_MODULE_FILENAME_EX(M)    EXEH_InitModuleFileName(M, __FILE__)

/**
 * @brief Exception severity level enumeration.
 */
typedef enum
{
    // Warning - just logged
    eEXEHSeverity_Warning,
    // Error - just logged
    eEXEHSeverity_Error,
    // Fatal - logged and reset
    eEXEHSeverity_Fatal,
    // Count of severities
    eEXEHSeverity_Count,
}eEXEHSeverityLevel_t;

typedef struct
{
#if EXEH_CONFIG_EVENT_TIMESTAMP_ENABLED == 1
    // tick count from system start
    uint32_t u32Timestamp;
#endif
    // The user supplied exception ID.
    int32_t  s32Exception;
    // The line number where the exception occurred.
    uint32_t u32LineNumber;
} PACKED sEXEHException_t;

typedef struct
{
#if EXEH_CONFIG_STORE_FILENAME_ENABLED == 1
    char fileName[FNAME_SIZE];
#endif
    sEXEHException_t warnings[WARN_SLOTS_COUNT];
    sEXEHException_t errors[ERROR_SLOTS_COUNT];
    sEXEHException_t fatals[FATAL_SLOTS_COUNT];
} PACKED sEXEHModuleExceptions_t;

/**
 * @brief Request storing of exception with level and a user supplied ID.
 * @param fp_eLevel         - level
 * @param fp_eModuleID      - the ID of the module reporting the exception.
 * @param fp_s32ExceptionID - the user defined exception ID.
 * @param fp_u32FileLineNo  - line in the file where the exception occurred.
 */
void EXEH_vException(eEXEHSeverityLevel_t fp_eLevel,
                     eEXEHModuleID_t      fp_eModuleID,
                     int32_t              fp_s32ExceptionID,
                     uint32_t             fp_u32FileLineNo);

/**
 * @brief Get the count of exceptions.
 * @param fp_eLevel - the level to get the count for.
 * @return uint32_t - the count of exceptions for fp_eLevel.
 */
uint32_t EXEH_u32GetExceptionCount(eEXEHSeverityLevel_t fp_eLevel);

/**
 * Get all exceptions stored.
 * @param fp_psOutArray - the output
 * @param fp_pu32ArraySize - the size of the output array.
 * @return uint32_t - number of bytes returned
 */
uint32_t EXEH_u32GetExceptions(const eEXEHModuleID_t fp_eModuleID,
                               uint8_t* const fp_psOutArray,
                               const uint32_t fp_u32OutArraySize);

/**
 * @brief (Re)Initialize EXEH. Initializes storage buffers.
 * @return true on success.
 */
bool EXEH_bInit(void);

/**
 * @brief Start the task which persists the EXEH information to SD card.
 * @return none
 */
void EXEH_bStartPersistorTask(void);

/**
 * @brief Attempt to immediately persist the currently stored exceptions.
 * @return none
 */
void EXEH_ForcePersistExceptions(void);

/**
 * @brief Associate a file name with a module ID. Can be called only once as subsequent calls do
 *        not have any effect.
 * @param fp_eModuleID - module identifier
 * @param fp_pcFileName - File name to associate to fp_eModuleID
 * @return none
 */
void EXEH_InitModuleFileName(eEXEHModuleID_t fp_eModuleID, const char* const fp_pcFileName);

#ifdef __cplusplus
}
#endif

#endif //OBC_ES_EXEH_H
