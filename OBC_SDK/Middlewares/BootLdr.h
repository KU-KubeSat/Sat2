/*!
 ********************************************************************************************
 * @file BootLdr.h
 * @brief Header of BootLdr.c
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
#ifndef XXX_H
#define XXX_H

/*
 ********************************************************************************************
 * INCLUDES
 ********************************************************************************************
 */
#include "TaskMonitor.h"
#include "ff.h"
#include <stdbool.h>

/*
 ********************************************************************************************
 * EXTERNAL DEFINES
 ********************************************************************************************
 */
#define RST_NUMBER_MAX_COUNT_LEVEL1 (10)    /* maximum number of reset that have to occur to consider the application invalid */
#define RST_NUMBER_MAX_COUNT_LEVEL2 (20)    /* maximum number of reset that have to occur to consider the bootloader invalid */

//
//  FLASH-related definitions (RM0410.pdf / page 88, Table 3/4)
//
#if (ENABLE_DUAL_BANK == 1)  // dual-bank FLASH configuration: Option byte -> nDBANK = 0
  // OBC FLASH Memory Map
  //
  // | Bank  | Sector | AXIM base address         | Sector size (KB) | Comment                      |
  // |-------|--------|---------------------------|------------------|------------------------------|
  // |   1   |    0   | 0x0800 0000 - 0x0800 3FFF |        16        | Bootloader region            |
  // |   1   |    1   | 0x0800 4000 - 0x0800 7FFF |        16        | Bootloader region            |
  // |   1   |    2   | 0x0800 8000 - 0x0800 BFFF |        16        | Bootloader region            |
  // |   1   |    3   | 0x0800 C000 - 0x0800 FFFF |        16        | Bootloader region            |
  // |   1   |    4   | 0x0801 0000 - 0x0800 FFFF |        64        | Bootloader region            |
  // |   1   |    5   | 0x0802 0000 - 0x0800 FFFF |       128        | Bootloader region            |
  // |   1   |    6   | 0x0804 0000 - 0x0800 FFFF |       128        | Bootloader region            |
  // |   1   |    7   | 0x0806 0000 - 0x0800 FFFF |       128        | Bootloader region            |
  // |   1   |    8   | 0x0808 0000 - 0x0800 FFFF |       128        | Application region           |
  // |   1   |    9   | 0x080A 0000 - 0x0800 FFFF |       128        | Application region           |
  // |   1   |   10   | 0x080C 0000 - 0x0800 FFFF |       128        | Application region           |
  // |   1   |   11   | 0x080E 0000 - 0x0800 FFFF |       128        | Application region           |
  // |   2   |   12   | 0x0810 0000 - 0x0810 3FFF |        16        | Application region           |
  // |   2   |   13   | 0x0810 4000 - 0x0810 7FFF |        16        | Application region           |
  // |   2   |   14   | 0x0810 8000 - 0x0810 BFFF |        16        | Application region           |
  // |   2   |   15   | 0x0810 C000 - 0x0810 FFFF |        16        | Application region           |
  // |   2   |   16   | 0x0811 0000 - 0x0811 FFFF |        64        | Application region           |
  // |   2   |   17   | 0x0812 0000 - 0x0813 FFFF |       128        | Application region           |
  // |   2   |   18   | 0x0814 0000 - 0x0815 FFFF |       128        | Application region           |
  // |   2   |   19   | 0x0816 0000 - 0x0817 FFFF |       128        | Application region           |
  // |   2   |   20   | 0x0818 0000 - 0x0819 FFFF |       128        | Application region           |
  // |   2   |   21   | 0x081A 0000 - 0x081B FFFF |       128        | Application region           |
  // |   2   |   22   | 0x081C 0000 - 0x081D FFFF |       128        | EEPROM emulation region      |
  // |   2   |   23   | 0x081E 0000 - 0x081F FFFF |       128        | Bootloader Flash Info region |

  #define FLASH_SECTOR_SIZE         0x20000               // 128 KB
  #define FLASH_MIN_SECTOR          12                    // dual-bank: application start address: 0x08080000 <-- change if FLASH_MIN_ADDR is modified
  #define FLASH_SECTORS_COUNT       24
  #define FLASH_BLOCK_SIZE          (128 * 1024)          // Size of one physical block from the flash in application area for dual bank mode
#else   // single-bank FLASH configuration: Option byte -> nDBANK = 1
  // OBC FLASH Memory Map
  //
  // | Bank  | Sector | AXIM base address         | Sector size (KB) | Comment                      |
  // |-------|--------|---------------------------|------------------|------------------------------|
  // |   1   |    0   | 0x0800 0000 - 0x0800 7FFF |        32        | Bootloader region            |
  // |   1   |    1   | 0x0800 8000 - 0x0800 FFFF |        32        | Bootloader region            |
  // |   1   |    2   | 0x0801 0000 - 0x0801 7FFF |        32        | Bootloader region            |
  // |   1   |    3   | 0x0801 8000 - 0x0801 FFFF |        32        | Bootloader region            |
  // |   1   |    4   | 0x0802 0000 - 0x0803 FFFF |       128        | Bootloader region            |
  // |   1   |    5   | 0x0804 0000 - 0x0807 FFFF |       256        | Bootloader region            |
  // |   1   |    6   | 0x0808 0000 - 0x080B FFFF |       256        | Bootloader region            |
  // |   1   |    7   | 0x080C 0000 - 0x080F FFFF |       256        | Bootloader region            |
  // |   1   |    8   | 0x0810 0000 - 0x0813 FFFF |       256        | Application region           |
  // |   1   |    9   | 0x0814 0000 - 0x0817 FFFF |       256        | Application region           |
  // |   1   |   10   | 0x0818 0000 - 0x081B FFFF |       256        | Application region           |
  // |   1   |   11   | 0x081C 0000 - 0x081F FFFF |       256        | Application region           |

  #define FLASH_SECTOR_SIZE         0x40000               // 256 KB
  #define FLASH_MIN_SECTOR          8                     // single-bank: application start address: 0x08080000 <-- change if FLASH_MIN_ADDR is modified
  #define FLASH_SECTORS_COUNT       12
  #define FLASH_BLOCK_SIZE          (256 * 1024)          // Size of one physical block from the flash in application area for single bank mode
#endif

#define BANK_SIZE                   (1024 * 1024)         // 1MB
#define FLASH_BLANK                 0xFF
#define FLASH_MIN_ADDR              APPL_ADDRESS
#define FLASH_MAX_ADDR              0x08200000            // Last physical flash address

// This symbol defines the maximum allowed application size in bytes (the last two flash sectors are reserved for
// flashed image information and EEPROM emulation
#define MAX_APP_SIZE                (BANK_SIZE - 2 * FLASH_BLOCK_SIZE)

// Information about the flashed application image is persisted by Bootloader in the last FLASH sector
#define FLASH_INFO_ADDRESS          (FLASH_MAX_ADDR - FLASH_BLOCK_SIZE)

//
//  RAM-related definitions
//
#define RAM_MIN_ADDRESS             (0x20000000)
#define RAM_SIZE                    (512*1024)
#define RAM_MAX_ADDRESS             (RAM_MIN_ADDRESS + RAM_SIZE-1)

#define BOOTLDR_BIN_CRC_SIZE        (4)

#define BOOTLDR_MAX_FILE_NAME_LEN   (15)
#define BOOTLDR_AUTOFLASH_FILE_PATH "0:/OBC_img.bin"

/*
 ********************************************************************************************
 * EXTERNAL TYPES DECLARATIONS
 ********************************************************************************************
 */
typedef struct
{
    uint32_t CodeSize;
    uint32_t CodeCRC;
} FlashInfoStruct;

// type used by WriteHandler_FaultTest
typedef void (*pFunction)(void);

typedef enum
{
    BOOTLDR_FW_OP_RESUS_OK,
    BOOTLDR_FW_OP_ERR_READ,
    BOOTLDR_FW_OP_ERR_WRITE,
    BOOTLDR_FW_OP_ERR_COMPARE,
    BOOTLDR_FW_OP_ERR_WRITE_INCOMPLETE,
    BOOTLDR_FW_OP_ERR_CRC,
    BOOTLDR_FW_OP_ERR_SIZE,
    BOOTLDR_FW_OP_ERR_FILENAME,
    BOOTLDR_FW_OP_ERR_FILE_FORMAT,
} BOOTLDR_FwOpRes_enum;

typedef enum
{
    BOOTLDR_RST_COUNT_OK,
    BOOTLDR_RST_COUNT_ERROR_LEVEL1,
    BOOTLDR_RST_COUNT_ERROR_LEVEL2
} BOOTLDR_RstCountErr_enum;

/*
 ********************************************************************************************
 * EXTERNAL VARIABLES DECLARATIONS
 ********************************************************************************************
 */

/*
 ********************************************************************************************
 * EXTERNAL ROUTINES DECLARATIONS
 ********************************************************************************************
 */
void BootLdr_Init(void);
BOOTLDR_RstCountErr_enum BootLdr_GetResetCounterslevel(void);
void BootLdr_BootingProcess(void);
uint8_t BootLdr_IsFlashingStarted(void);
void BootLdr_SetFlashingFlag(uint8_t state);
uint8_t BootLdr_IsSleepStarted(void);

void BootLdr_FwUpdateRequest(const uint8_t *fileName);
void BootLdr_CheckAutoFlashCmd(void);
BOOTLDR_FwOpRes_enum BootLdr_VerifyCrcAndSizeOfBinFile(TCHAR *inputBinPath, uint32_t *file_CRC, uint32_t *codeSize);
BOOTLDR_FwOpRes_enum BootLdr_VerifyCrcOfScmFile(FIL *df, const char *filePath, uint32_t *file_CRC, uint32_t *codeSize, TASK_MONITOR_TasksEnum task);
BOOTLDR_FwOpRes_enum BootLdr_FlashingFromFile(TCHAR *inputBinPath, uint8_t startPosition, uint32_t fileCRC);
HAL_StatusTypeDef BootLdr_EraseFlash(uint32_t codeSize, uint8_t *errasedSectors);
uint32_t BootLdr_FlashEmptyPatternCheck(uint32_t addr, uint32_t len);
uint8_t BootLdr_FlashVerifyCRC(void);
void BootLdr_ImmediatAppJump(void);
uint32_t BootLdr_FlashCalcBootCRC(void);
bool BootLdr_IsBootloaderImageActive(void);

#endif    /* XXX_H */
/* **************************************************************************************** */
