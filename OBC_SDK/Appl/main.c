/*!
 ********************************************************************************************
 * @file main.c
 * @brief Initialisation of some periphery and start of the operation system
 ********************************************************************************************
 * @author            Vassil Milev
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
 * @revision{         1.0.0  , 2018.07.04, author Vassil Milev, Initial revision }
 * @endhistory
 ********************************************************************************************
 */
#include "main.h"
#include "MCU_Init.h"
#include "ESTTC.h"
#include "AppTasks.h"
#include "TaskMonitor.h"
#include <errno.h>
#include <sys/unistd.h>
#include <stdarg.h>
#include <EEPROM_emul.h>
#include "PwrMng.h"
#include <es_crc32.h>
#include <MX_I2C.h>
#include <es_exeh.h>
#include <UHF.h>
#include <EPS_I.h>
#include <BootLdr.h>
#include <SdMngr.h>
#include "ExtFram.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "csp_example_config.h"
#include "csp/csp_types.h"
#pragma GCC diagnostic pop

#include "GlobalConfig.h"
#include "OEM719.h"

#include "../KUbeSat/KUbeSatTasks.h"
#include "../KUbeSat/SatStates.h"

static bool cspex_initialize(void);

bool test = 0;

int main(void)
{
    // PVD and Brownout reset init
    PVD_BrownoutReset_Init();

#ifdef DEBUG_ENABLED
    vRamTest_StackFill();

    // Enable debug mode in sleep and stop modes
    HAL_DBGMCU_EnableDBGSleepMode();
    HAL_DBGMCU_EnableDBGStopMode();
#endif
    // Enable All fault handlers
    SCB->SHCSR |= (SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk);

    // basic initialization of the exception storage facility...
    EXEH_bInit();

    // Initialize EEPROM emulation
    EEPROM_Emul_Init();

#if defined(BOOTLOADER)
  BootLdr_BootingProcess();
#else
    __disable_interrupt();
  #if  defined(NO_BOOTLOADER_ENABLED)
    // Redirect the Vector table to these of the Application, not to stay at these of the bootloader
    SCB->VTOR = BOOT_ADDRESS;
  #else
    // Redirect the Vector table to these of the Application, not to stay at these of the bootloader
    SCB->VTOR = APPL_ADDRESS;
  #endif
    __enable_interrupt();

    // set boot data at the beginning of the RTC backup registers RTC_BKPxR
    BootData = (boot_struct*)RTC_INIT_ADDRESS;
#endif

#ifdef DEBUG_ENABLED
  #if (ENABLE_WATCHDOG_AND_TASK_HANDLER == 1)
    MX_IWDG_Init();
  #endif
#else
    MX_IWDG_Init();
#endif

    // Reset of all peripherals, Initializes the Flash interface and the Systick.
    HAL_Init();

    // Configure the system clock
    SystemClock_IdleConfig();

    // Initialize all configured peripherals
    MX_AllPeripheryAndGpio_DeInit();
    MX_GPIO_Init();
    MX_USART4_UART_DeInit();

    // make sure to initialize after EEPROM_Emul_Init()
    EXEH_bStartPersistorTask();

    ESTTC_Init();

#if (ENABLE_POWER_MANAGER == 1)
    PwrMng_Init();
#endif

    MX_ADC1_Init();
    MX_RTC_Init();
    MX_I2C_Init();

    ExtFram_Init();

    SdMngr_InitTask();

    AMBER_LED_OFF();
    GREEN_LED_OFF();

    // Init scheduler
    osKernelInitialize();

    TaskMonitor_Init();

    AppTask_Init();
    ESTTC_InitTask();
    ServicesTask_Init();

#if (ENABLE_UHF_ANT_SUPPORT == 1)
    AntUHF_Init();
#endif

#if (ENABLE_EPS_I_SUPPORT == 1)
    EPS_I_Init();
#endif

#if (ENABLE_UHF_II_SUPPORT == 1)
    UHF_Init();
#endif

#if (ENABLE_GNSS_OEM719 == 1)
    OEM719_Init();
#endif

    cspex_initialize();

#ifdef DEBUG_ENABLED
    vRamTest_StackUsageCheck();
#endif

    // Start CODE
    //xTaskCreate(I2CTest, "I2C Test", 1024, NULL, 1, NULL);
    //xTaskCreate(EPSTest, "EPS Test", 1024, NULL, 1, NULL);
    //xTaskCreate(PiTest, "Pi Test", 1024, NULL, 1, NULL);
    //xTaskCreate(radioTest, "Boot", 1024, NULL, 1, NULL);
    //xTaskCreate(I2CTest2, "I2C", 1024, NULL, 1, NULL);
    //xTaskCreate(radioTest, "Radio", 1024, NULL, 1, NULL);
    //TaskCreate(bootCheck, "Boot", 1024, NULL, 2, NULL);
    xTaskCreate(checkClydStatus, "Power Status", 1024, NULL, 1, NULL);
    //xTaskCreate(pdmTest, "PDM Test", 1024, NULL, 1, NULL);
    //xTaskCreate(heaterTest, "Heater Test", 1024, NULL, 1, NULL);

    //EPS ping to stop watchdog reset
    xTaskCreate(EPSPing, "EPSPing", 1024, NULL, 1, NULL);



    DBG_SYSCON("Startup Complete\n");
    DBG_SYSCON("SatHawk v0.8\n");
    // Start scheduler
    osKernelStart();

    while (1)
    {
    }
}


void assert_failed(uint8_t *file, uint32_t line)
{
#ifdef DEBUG_ENABLED
    while (1)
    {
        GREEN_LED_ON();
        AMBER_LED_ON();
        __BKPT();
    }
#else
    TaskMonitor_ImmediatReset(TASK_MONITOR_FAULT_RESET);
#endif
}


void Error_Handler(void)
{
#ifdef DEBUG_ENABLED
    while (1)
    {
        GREEN_LED_ON();
        AMBER_LED_ON();
        __BKPT();
    }
#else
    TaskMonitor_ImmediatReset(TASK_MONITOR_FAULT_RESET);
#endif
}


void vApplicationMallocFailedHook(void)
{
    Error_Handler();
}


void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    static volatile char* taskName;
    taskName = pcTaskName;
    (void)taskName;
    Error_Handler();
}


static bool cspex_initialize(void)
{
#if (ENABLE_CSP_EXAMPLE == 1)
    static cspex_iface_can_t iface_can =
        {
            .driver_data = CSPEX_HANDLE_CAN,
            .init   = cspex_can_init,
            .deinit = cspex_can_deinit,
            .tx     = cspex_can_transmit,
            .rx     = cspex_can_receive
        };

    static cspex_iface_uart_t iface_uart =
        {
            .driver_data = CSPEX_HANDLE_UART,
            .init   = cspex_uart_init,
            .deinit = cspex_uart_deinit,
            .tx     = cspex_uart_transmit,
            .rx     = cspex_uart_receive
        };

    csp_debug_hook_set(cspex_debug_hook);
    csp_debug_set_level(CSP_ERROR, false);
    csp_debug_set_level(CSP_WARN, false);
    csp_debug_set_level(CSP_INFO, true);

    if (cspex_init(&iface_can,
                   &iface_uart,
                   CSPEX_COMM_UART | CSPEX_COMM_CAN,
                   CSPEX_MODE_SERVER | CSPEX_MODE_CLIENT,
                   1,
                   1,
                   2,
                   2,
                   1,
                   3,
                   2,
                   CSP_O_RDP,
                   osPriorityNormal))
    {
        return false;
    }

    return true;
#else
	return false;
#endif
}
