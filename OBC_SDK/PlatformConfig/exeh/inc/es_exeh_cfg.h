/**
 ******************************************************************************
 * @file    es_exeh_cfg.h
 * @brief   TODO
 ******************************************************************************
 *
 * COPYRIGHT(c) 2020 Enduro Sat. All rights reserved.
 *
 ******************************************************************************
 */

#ifndef ES_EXEH_CFG_H
#define ES_EXEH_CFG_H

#include "stm32f7xx_hal.h"


/**
 * @brief
 * Config options
 */
#define EXEH_CONFIG_STORE_FILENAME_ENABLED  (0)
#define EXEH_CONFIG_EVENT_TIMESTAMP_ENABLED (1)


#define TASK_PERSISTOR_PERIOD_MS            (1000U)


/**
 * @brief
 * number of task periods before attempting to persist the exception trace buffers to storage
 */
#define TASK_PERSISTOR_MAX_TASK_CNT         (10U)


/**
 * @brief
 * position of the volume character "0:/exeh_btl.d__"
 *                                   ^
 *                                   |
 *                                   0
 */
#define EXEH_DUMP_FN_VOL_POS                (0U)


/**
 * @brief
 * position of the index character "0:/exeh_btl.d__"
 *                                               ^
 *                                               |
 *                                               13
 */
#define EXEH_DUMP_FN_INDEX_POS              (13U)


/**
 * @brief
 * position of the index character "0:/exeh_btl.d__"
 *                                                 ^
 *                                                 |
 *                                                 14
 */
#define EXEH_DUMP_FN_MARKER_POS             (14U)


/**
 * @brief
 * the file ending with this character is the currently active file
 */
#define EXEH_DUMP_FN_ACTIVE_MARKER          'o'


/**
 * @brief
 * file names ending with this character are old dumps
 */
#define EXEH_DUMP_FN_INACTIVE_MARKER        'x'


#define EXEH_DUMP_FN_SLOTS                  10U


#if EXEH_DUMP_FN_SLOTS > 10
  #error EXEH_DUMP_FN_SLOTS shall be set a value <= 10!
#endif


#if defined(BOOTLOADER)
  #define EXEH_DUMP_FILE_NAME               "0:/exeh_btl.d__"
#else
  #define EXEH_DUMP_FILE_NAME               "0:/exeh_app.d__"
#endif


/**
 * @brief
 * Maximum number of exceptions stored per severity level
 */
#define EXEH_EXCEPTIONS_MAX                 (20U)
#define FNAME_SIZE                          (30U)
#define WARN_SLOTS_COUNT                    (2U)
#define ERROR_SLOTS_COUNT                   (2U)
#define FATAL_SLOTS_COUNT                   (2U)


#define getSystemTickCount()                HAL_GetTick()


/**
 * @brief
 * enumeration with all modules that could store exceptions
 */
typedef enum
{
    // Exceptions from the operating system.
    eEXEHModuleID_OS,                           // not in uses so far !!!
    // Exceptions from MCU init module.
    eEXEHModuleID_MCU_INIT,
    // Exceptions from the EXEH module itself.
    eEXEHModuleID_ES_EXEH,
    // Exceptions from RCC driver.
    eEXEHModuleID_RCC,
    // Exceptions from the UART module.
    eEXEHModuleID_UART,
    // Exceptions from EPS communication.
    eEXEHModuleID_EPS_COMM,
    // Exceptions from EPS communication.
    eEXEHModuleID_UHF_COMM,
    // Exceptions from UHF_ANT communication.
    eEXEHModuleID_UHF_ANT_COMM,
    // Exceptions from ARDU_CAM communication.
    eEXEHModuleID_ARDU_CAM_COMM,
    // Exceptions from power manager.
    eEXEHModuleID_PWR_MNG,
    // Exceptions from system status and telemetry.
    eEXEHModuleID_SYS_STAT_AND_TLMTRY,
    // Exceptions from Firmware Update
    eEXEHModuleID_FWUPD_HANDLER,
    eEXEHModuleID_FWUPD_HANDLER_CFG,
    // Exceptions from the Sd card manager (SdMngr) module
    eEXEHModuleID_SDMNGR,
    // Exceptions from file logging module
    eEXEHModuleID_FILE_LOGS,
    eEXEHModuleID_TASK_MONITOR,
    eEXEHModuleID_COUNT
} eEXEHModuleID_t;


/**
 * @brief
 * List of all Exception ID according the Module ID
 * Keep these arranged systematically for easy documentation
 * can be used from external applications (GUI, Ground Station, etc.)
 */

typedef enum
{
    eEXEH_MCU_INIT_EXCEPTION_ID_I2C_RST,
    eEXEH_MCU_INIT_EXCEPTION_ID_MANITO_INIT,
    eEXEH_MCU_INIT_EXCEPTION_ID_GYRO_INIT,
    eEXEH_MCU_INIT_EXCEPTION_ID_COUNT
} eEXEH_MCU_INIT_ExceptionID_t;


typedef enum
{
    eEXEH_EPS_EXCEPTION_ID_COMM_ERROR,
    eEXEH_EPS_EXCEPTION_ID_COMM_INVALID_PARAMS_ERROR,
    eEXEH_EPS_EXCEPTION_ID_COUNT
} eEXEH_EPS_ExceptionID_t;


typedef enum
{
    eEXEH_UHF_EXCEPTION_ID_COMM_I2C_ERROR,
    eEXEH_UHF_EXCEPTION_ID_COMM_INVALID_PARAMS_ERROR,
    eEXEH_UHF_EXCEPTION_ID_COUNT
} eEXEH_UHF_ExceptionID_t;


typedef enum
{
    eEXEH_UHF_ANT_EXCEPTION_ID_COMM_ERROR,
    eEXEH_UHF_ANT_EXCEPTION_ID_VBAT_RX_ERROR,
    eEXEH_UHF_ANT_EXCEPTION_ID_COUNT
} eEXEH_UHF_ANT_ExceptionID_t;


typedef enum
{
    eEXEH_ARDU_CAM_EXCEPTION_ID_COMM_ERROR,
    eEXEH_ARDU_CAM_EXCEPTION_ID_COUNT
} eEXEH_ARDU_CAM_ExceptionID_t;


typedef enum
{
    eEXEH_PWR_MNG_EXCEPTION_SET_WAKEUP_TIME_FAIL,
    eEXEH_PWR_MNG_EXCEPTION_ID_COUNT
} eEXEH_PWR_MNG_ExceptionID_t;


typedef enum
{
    eEXEH_SDMNGR_EXCEPTION_FR_DISK_ERR,
    eEXEH_SDMNGR_EXCEPTION_FR_INT_ERR,
    eEXEH_SDMNGR_EXCEPTION_FR_NOT_READY,
    eEXEH_SDMNGR_EXCEPTION_FR_NO_FILE,
    eEXEH_SDMNGR_EXCEPTION_FR_NO_PATH,
    eEXEH_SDMNGR_EXCEPTION_FR_INVALID_NAME,
    eEXEH_SDMNGR_EXCEPTION_FR_DENIED,
    eEXEH_SDMNGR_EXCEPTION_FR_EXIST,
    eEXEH_SDMNGR_EXCEPTION_FR_INVALID_OBJECT,
    eEXEH_SDMNGR_EXCEPTION_FR_WRITE_PROTECTED,
    eEXEH_SDMNGR_EXCEPTION_FR_INVALID_DRIVE,
    eEXEH_SDMNGR_EXCEPTION_FR_NOT_ENABLED,
    eEXEH_SDMNGR_EXCEPTION_FR_NO_FILESYSTEM,
    eEXEH_SDMNGR_EXCEPTION_FR_MKFS_ABORTED,
    eEXEH_SDMNGR_EXCEPTION_FR_TIMEOUT,
    eEXEH_SDMNGR_EXCEPTION_FR_LOCKED,
    eEXEH_SDMNGR_EXCEPTION_FR_NOT_ENOUGH_CORE,
    eEXEH_SDMNGR_EXCEPTION_FR_TOO_MANY_OPEN_FILES,
    eEXEH_SDMNGR_EXCEPTION_FR_INVALID_PARAMETER,
    eEXEH_SDMNGR_EXCEPTION_INIT_FAILED,
    eEXEH_SDMNGR_EXCEPTION_FORMAT_FAILED,
    eEXEH_SDMNGR_EXCEPTION_RECTED_OPPERATION,
    eEXEH_SDMNGR_EXCEPTION_ID_COUNT
} eEXEH_SDMNGR_ExceptionID_t;


typedef enum
{
    eEXEH_FILE_LOGS_EXCEPTION_WRITE_FAIL,
    eEXEH_FILE_LOGS_EXCEPTION_CREATE_FILE,
    eEXEH_FILE_LOGS_EXCEPTION_PREPARE_FILE,
    eEXEH_FILE_LOGS_EXCEPTION_ID_COUNT
} eEXEH_FILE_LOGS_ExceptionID_t;


typedef enum
{
    eEXEH_TASK_MONITOR_TASK_TIMEOUT,
    eEXEH_TASK_MONITOR_TASK_STACK_WARN,
    eEXEH_TASK_MONITOR_TASK_COUNT
} eEXEH_TASK_MON_ExceptionID_t;


/**
 * @brief
 * Project specific EXEH warning handler.
 * Called when handling a warning.
 */
void EXEH_vHandleWarning(void);


/**
 * @brief
 * Project specific EXEH error handler.
 * Called when handling errors.
 */
void EXEH_vHandleError(void);


/**
 * @brief
 * Project specific EXEH fatal handler.
 * Called when handling fatal exception.
 */
void EXEH_vHandleFatal(void);


#endif // ES_EXEH_CFG_H
