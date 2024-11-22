/*!
 ********************************************************************************************
 * @file GlobalConfig.h
 * @brief Configuration settings that influences the whole project or more then one module
 ********************************************************************************************
 * @author            Vassil Milev
 * @version           1.0.0
 * @date              2020.12.01
 *
 * @copyright         (C) Copyright Endurosat
 *
 *                    Contents and presentations are protected world-wide.
 *                    Any kind of using, copying etc. is prohibited without prior permission.
 *                    All rights - incl. industrial property rights - are reserved.
 *
 * @history
 * @revision{         1.0.0  , 2020.12.01, author Vassil Milev, Initial revision }
 * @endhistory
 ********************************************************************************************
 */
#ifndef GLOABAL_CONFIG_H
#define GLOABAL_CONFIG_H

#ifdef DEBUG_ENABLED
  #define ENABLE_DEBUG_LIB                (1)  // 0 - Disable; 1 - Enable
#endif


#define FW_MAJOR_REV                      ( 3)
#define FW_MINOR_REV                      (87)


#define ENABLE_DUAL_BANK                  (1)  // 0 - SINGLE; 1 - DUAL; BANK for FLASH
#define ENABLE_ESTTC_FAULT_TESTS          (1)  // 0 - Disable; 1 - Enable
#define ENABLE_WATCHDOG_AND_TASK_HANDLER  (1)  // 0 - Disable; 1 - Enable
#define ENABLE_POWER_MANAGER              (1)  // 0 - Disable; 1 - Enable
#define ENABLE_EXEH_PERSIST	              (1)  // 0 - Disable; 1 - Enable
#define ENABLE_CSP_EXAMPLE                (1)  // 0 - Disable; 1 - Enable

// external devices
#define ENABLE_UHF_ANT_SUPPORT            (1)  // 0 - Disable; 1 - Enable
#define ENABLE_UHF_II_SUPPORT             (1)  // 0 - Disable; 1 - Enable
#define ENABLE_EPS_I_SUPPORT              (1)  // 0 - Disable; 1 - Enable
#define ENABLE_GNSS_OEM719                (1)  // 0 - Disable; 1 - Enable
#define ENABLE_CAMERA_OV5640              (0)  // 0 - Disable; 1 - Enable


#endif /* GLOABAL_CONFIG_H */

