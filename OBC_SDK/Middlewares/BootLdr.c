/*!
********************************************************************************************
* @file BootLdr.c
* @brief Bootloader functionality
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

/*
********************************************************************************************
* INCLUDES
********************************************************************************************
*/
#include <BootLdr.h>          /* own header file */
#include <stm32f7xx_ll_pwr.h> /* Power registers */
#include <EEPROM_emul.h>      /* EEPROM emulation */
#include <string.h>           /* string functions */
#include <Svc_RTC.h>          /* BootData structure */
#include <TaskMonitor.h>      /*  */
#include "SdMngr.h"
#include <es_crc32.h>

/*
********************************************************************************************
* INTERNAL DEFINES
********************************************************************************************
*/
#define BOOTLDR_BUFF_SIZE            (256)
#define BOOTLDR_AUTOREPEAT_TIMES     (3)

#define BOOTLDRSCM_TO_BIN_FILE_NAME_LEN   (14)
#define BOOTLDR_SCM_TO_BIN_FILE_PATH      "0:/fw_out.bin"

/*
********************************************************************************************
* INTERNAL TYPES DEFINITION
********************************************************************************************
*/
/* No Internal types definition */

/*
********************************************************************************************
* EXTERNAL VARIABLES DEFINITION
********************************************************************************************
*/
/* No External variables definition */

/**
* @brief Variable description
*/


/*
********************************************************************************************
* INTERNAL (STATIC) VARIABLES DEFINITION/DECLARATION 
********************************************************************************************
*/
static uint8_t fileTempBuffSize[BOOTLDR_BUFF_SIZE];
FIL dest_file;

struct
{
    uint8_t flashRequested;
    uint8_t len;
    TCHAR   fileName[BOOTLDR_MAX_FILE_NAME_LEN + 1];  //+ string termination symbol
} static bootLdrManualFlash =
  {
      .flashRequested = 0
  };

static uint8_t BootLdr_autoFlashRepeats = 0;

static uint8_t BootLdr_flashingInProggress = pdFALSE;
static uint8_t BootLdr_sleepInProggress = pdFALSE;

/*
********************************************************************************************
* INTERNAL (STATIC) ROUTINES DECLARATION
********************************************************************************************
*/
static HAL_StatusTypeDef FLASH_Write(uint8_t *src, uint32_t addr, uint32_t len);
static uint8_t HexToBin(uint8_t hb, uint8_t lb);
static void BootLdr_AutoFlashFromBin(TCHAR *inputBinPath, uint8_t strLength);
static uint8_t getLastSectorForSize(const uint8_t u8StartSector, const uint32_t u32DataSizeBytes);

/*
********************************************************************************************
* EXTERNAL (NONE STATIC) ROUTINES DEFINITION
********************************************************************************************
*/
/*!
********************************************************************************************
* @brief Reads the error counters and returns corruption level
*        depending of the maximum number of SW resets
********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            BOOTLDR_RstCountErr_enum - level according to the maximum resets number
* @note              none
********************************************************************************************
*/
BOOTLDR_RstCountErr_enum BootLdr_GetResetCounterslevel(void)
{
    BOOTLDR_RstCountErr_enum retVal;

    // valid RTC backup memory
    if ((BootData->RST_WWD        > RST_NUMBER_MAX_COUNT_LEVEL2) || /* Event WINDOW_WATCHDOG_RESET */
        (BootData->RST_IWD        > RST_NUMBER_MAX_COUNT_LEVEL2) || /* Event INDEPENDENT_WATCHDOG_RESET */
        (BootData->RST_HardFault  > RST_NUMBER_MAX_COUNT_LEVEL2) || /* Event HardFault_Handler() */
        (BootData->RST_MemFault   > RST_NUMBER_MAX_COUNT_LEVEL2) || /* Event MemManage_Handler() */
        (BootData->RST_BusFault   > RST_NUMBER_MAX_COUNT_LEVEL2) || /* Event BusFault_Handler() */
        (BootData->RST_UsageFault > RST_NUMBER_MAX_COUNT_LEVEL2))   /* Event UsageFault_Handler() */
    {
        // in case of WD reset more then several times remain in bootloader
        retVal = BOOTLDR_RST_COUNT_ERROR_LEVEL2;
    }
    else
    {
        if ((BootData->RST_WWD        > RST_NUMBER_MAX_COUNT_LEVEL1) || /* Event WINDOW_WATCHDOG_RESET */
            (BootData->RST_IWD        > RST_NUMBER_MAX_COUNT_LEVEL1) || /* Event INDEPENDENT_WATCHDOG_RESET */
            (BootData->RST_HardFault  > RST_NUMBER_MAX_COUNT_LEVEL1) || /* Event HardFault_Handler() */
            (BootData->RST_MemFault   > RST_NUMBER_MAX_COUNT_LEVEL1) || /* Event MemManage_Handler() */
            (BootData->RST_BusFault   > RST_NUMBER_MAX_COUNT_LEVEL1) || /* Event BusFault_Handler() */
            (BootData->RST_UsageFault > RST_NUMBER_MAX_COUNT_LEVEL1))   /* Event UsageFault_Handler() */
        {
            // in case of WD reset more then several times remain in bootloader
            retVal = BOOTLDR_RST_COUNT_ERROR_LEVEL1;
        }
        else
        {
            retVal = BOOTLDR_RST_COUNT_OK;
        }
    }

    return retVal;
}

/*!
********************************************************************************************
* @brief Function that decides to stay in Bootloader or jump to Application
********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
********************************************************************************************
*/
void BootLdr_BootingProcess(void)
{

    /* Enable mailbox write access */
    {
        __HAL_RCC_PWR_CLK_ENABLE();   /* Enable Power Clock*/
        LL_PWR_EnableBkUpAccess();    /* Enable write access to Backup domain */
    }

    BootData = (boot_struct*)RTC_INIT_ADDRESS; /* set boot data at the beginning of the RTC backup registers RTC_BKPxR */

    uint8_t RTC_BKP_valid;
    uint8_t FaultCountersErr = 0;

    RTC_BKP_valid = RTC_ValidateRTC_BKP() | RTC_ValidateRTC_Counters();

    if (0 == RTC_BKP_valid) /* If the RTC backup memory is not valid */
    {
        //Restore the counters from the EEPROM emulated memory
        BootData->Reserve1       = EEPROM_emul_DataTemp.ResetFaults.Reserve1;
        BootData->Mailbox        = EEPROM_emul_DataTemp.ResetFaults.Mailbox;
        BootData->RST_WWD        = EEPROM_emul_DataTemp.ResetFaults.RST_WWD;
        BootData->RST_IWD        = EEPROM_emul_DataTemp.ResetFaults.RST_IWD;
        BootData->RST_LPR        = EEPROM_emul_DataTemp.ResetFaults.RST_LPR;
        BootData->RST_POR        = EEPROM_emul_DataTemp.ResetFaults.RST_POR;
        BootData->RST_RstPin     = EEPROM_emul_DataTemp.ResetFaults.RST_RstPin;
        BootData->RST_BOR        = EEPROM_emul_DataTemp.ResetFaults.RST_BOR;
        BootData->RST_HardFault  = EEPROM_emul_DataTemp.ResetFaults.RST_HardFault;
        BootData->RST_MemFault   = EEPROM_emul_DataTemp.ResetFaults.RST_MemFault;
        BootData->RST_BusFault   = EEPROM_emul_DataTemp.ResetFaults.RST_BusFault;
        BootData->RST_UsageFault = EEPROM_emul_DataTemp.ResetFaults.RST_UsageFault;
        BootData->RST_ErrHandler = EEPROM_emul_DataTemp.ResetFaults.RST_ErrHandler;
        BootData->RebootRequest  = EEPROM_emul_DataTemp.ResetFaults.RebootRequest;
        BootData->Reserve3       = EEPROM_emul_DataTemp.ResetFaults.Reserve3;
        BootData->BootDataCRC    = EEPROM_emul_DataTemp.ResetFaults.BootDataCRC;
    }

    RTC_CountRstType();

    // in case of WD reset more then several times remain in bootloader
    FaultCountersErr = BootLdr_GetResetCounterslevel();

    /* Clear the RCT Flags for the last reset reason */
    RTC_ClearRSTReason();

    if (BootData->Mailbox == MAILBOX_VAL_AUTO_FLASH)
    {
        //Prevent from new try if a unexpected reset happen
        BootLdr_autoFlashRepeats = 3;
        BootData->Mailbox = MAILBOX_VAL_BOOT;
    }

    /* Calculate the CRC for the information in the RTC Backup registers */
    RTC_SetRTC_BKP_CRC();

    /* If the RTC data has changed write it to the memory */
    if( 0 != memcmp(&EEPROM_emul_DataTemp.ResetFaults, (void *)BootData, sizeof(boot_struct)) )
    {
        /* Sync the RTC backup registers data with the buffer of the emulated EEPROM memory */
        memcpy(&EEPROM_emul_DataTemp.ResetFaults, (void *)BootData, sizeof(boot_struct));

        /* Sync EEPROM emulated memory with the buffer "EEPROM_emul_DataTemp" */
        EEPROM_Emul_SyncInfo();
    }

#ifndef DEBUG_ENABLED
    /* Check CRC over the Flash memory */
    uint8_t FlashCRC_valid = BootLdr_FlashVerifyCRC();
#endif

    if (((BootData->Mailbox) == MAILBOX_VAL_APPL) && // != MAILBOX_VAL_BOOT)&&(BootData->Mailbox != MAILBOX_VAL_HARD))&&  //== MAILBOX_VAL_APPL)
#ifndef DEBUG_ENABLED     /* Working with debugger at the Application part will lead to missing CRC at the end of the Flash. So skip that check, so it is possible to jump back from Application to Application, Not staing suck in the bootloader */
        ( 1 == FlashCRC_valid )&&   // CRC of the flash is right
#endif
        (*(uint32_t*)APPL_ADDRESS != 0xFFFFFFFF) && /* Check if Appl ISR table is empty or invalid */
        (BOOTLDR_RST_COUNT_OK == FaultCountersErr))  // error counters are not detected/exceeded
    {
        pFunction appEntry;
        uint32_t appStack;

        MX_IWDG_Init();

        __disable_interrupt();
        // Disable all interrupts
        NVIC->ICER[0] = 0xFFFFFFFF;
        NVIC->ICER[1] = 0xFFFFFFFF;
        NVIC->ICER[2] = 0xFFFFFFFF;
        // Clear all pending interrupts
        NVIC->ICPR[0] = 0xFFFFFFFF;
        NVIC->ICPR[1] = 0xFFFFFFFF;
        NVIC->ICPR[2] = 0xFFFFFFFF;
        // Stop sys tick
        appStack = SysTick->CTRL;
        appStack &= ~0x02;
        SysTick->CTRL = appStack;

        /* Reconfigure vector table offset register to match the application location */
        SCB->VTOR = APPL_ADDRESS;

        /* Get the application stack pointer (First entry in the application vector table) */
        appStack = (uint32_t)*((__IO uint32_t*)APPL_ADDRESS);
        /* Get the application entry point (Second entry in the application vector table) */
        appEntry = (pFunction)*(__IO uint32_t*)(APPL_ADDRESS + 4);
        /* Set the application stack pointer */
        __set_MSP(appStack);
        /* Start the application */

        appEntry();
    }
    else
    {
        __disable_interrupt();
        SCB->VTOR = BOOT_ADDRESS;
        __enable_interrupt();
    }
}


/*!
********************************************************************************************
* @brief Routine that checks if FW update is needed to be done and perform it
********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            uint8_t:
*                       + pdTRUE - update has started
*                       + pdFALSE - updated is not on going
* @note              none
********************************************************************************************
*/
uint8_t BootLdr_IsFlashingStarted(void)
{
    return BootLdr_flashingInProggress;
}

/*!
********************************************************************************************
* @brief Routine that checks if FW update is needed to be done and perform it
********************************************************************************************
* @param[input]      state - path and name of the file to be used to update from it
*                       + pdTRUE - update has started
*                       + pdFALSE - updated is not on going
* @param[output]     none
* @return            none
* @note              none
********************************************************************************************
*/
void BootLdr_SetFlashingFlag(uint8_t state)
{
    BootLdr_flashingInProggress = state;
}

/*!
********************************************************************************************
* @brief Routine that checks if FW update is needed to be done and perform it
********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            uint8_t:
*                       + pdTRUE - update has started
*                       + pdFALSE - updated is not on going
* @note              none
********************************************************************************************
*/
uint8_t BootLdr_IsSleepStarted(void)
{
    return BootLdr_sleepInProggress;
}

/*!
********************************************************************************************
* @brief Routine that checks if FW update is needed to be done and perform it
********************************************************************************************
* @param[input]      fileName - path and name of the file to be used to update from it
* @param[output]     none
* @return            none
* @note              none
********************************************************************************************
*/
void BootLdr_FwUpdateRequest(const uint8_t * fileName)
{
    memcpy(bootLdrManualFlash.fileName, fileName, BOOTLDR_MAX_FILE_NAME_LEN);
    bootLdrManualFlash.fileName[BOOTLDR_MAX_FILE_NAME_LEN] = 0; // terminate the string just in case of invalid input
    bootLdrManualFlash.len = (uint8_t)strlen(bootLdrManualFlash.fileName);
    bootLdrManualFlash.flashRequested = 1;
}

/*!
********************************************************************************************
* @brief Routine that checks if FW update is needed to be done and perform it
********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
********************************************************************************************
*/
void BootLdr_CheckAutoFlashCmd(void)
{
    static uint8_t initTimeout = 0;

    if ((TaskMonitor_CheckTasksInit()) || (initTimeout > 10))
    {
        if (BootLdr_autoFlashRepeats)
        {
            BootLdr_AutoFlashFromBin(NULL, 0);
            BootLdr_autoFlashRepeats--;
        }

        BootLdr_sleepInProggress = pdFALSE;

        if ((bootLdrManualFlash.flashRequested) && (BootLdr_sleepInProggress == pdFALSE))
        {
            BootLdr_AutoFlashFromBin(bootLdrManualFlash.fileName, bootLdrManualFlash.len);
        }
    }
    else
    {
        initTimeout++;
    }
}



/*!
********************************************************************************************
* @brief Verifies a file for a firmware update
********************************************************************************************
* @param[input]      inputBinPath - path to the file located in a SD card
* @param[output]     file_CRC - calculated CRC of the .bin file
* @param[output]     codeSize - size of the code that is going to be written with that .bin file
* @return            BOOTLDR_FwOpRes_enum - final status of the process
* @note              none
********************************************************************************************
*/
BOOTLDR_FwOpRes_enum BootLdr_VerifyCrcAndSizeOfBinFile(TCHAR *inputBinPath, uint32_t *file_CRC, uint32_t *codeSize)
{
    BOOTLDR_FwOpRes_enum fwOperationsResult;
    FRESULT readStatus;
    uint32_t CRC_data;
    uint16_t readWriteSize;
    uint32_t filePossition;
    UINT br;

    SdMngr_f_close(&dest_file);    //Just in case

    for (uint8_t i = 0; i < BOOTLDR_AUTOREPEAT_TIMES; i++)
    {
        fwOperationsResult = BOOTLDR_FW_OP_RESUS_OK;
        readWriteSize = BOOTLDR_BUFF_SIZE;
        filePossition = BOOTLDR_BIN_CRC_SIZE;

        readStatus = SdMngr_f_open(&dest_file, inputBinPath, FA_READ | FA_OPEN_EXISTING);
        if (FR_OK != readStatus)
        {
            fwOperationsResult = BOOTLDR_FW_OP_ERR_FILENAME;
        }
        else
        {

            *codeSize = SdMngr_f_size(&dest_file) - BOOTLDR_BIN_CRC_SIZE;

            /* extract the CRC from the file */
            readStatus = SdMngr_f_read(&dest_file, fileTempBuffSize, BOOTLDR_BIN_CRC_SIZE, (UINT*)&br);
            if (FR_OK != readStatus)
            {
                fwOperationsResult = BOOTLDR_FW_OP_ERR_READ;
            }
            else
            {

                *file_CRC = (fileTempBuffSize[3] << 24) | (fileTempBuffSize[2] << 16) | (fileTempBuffSize[1] << 8) | fileTempBuffSize[0];

                /* Calculated the file CRC */
                CRC_data = 0; /* Initial value = 0 */

                /* Start reading the file to calculate the CRC */
                for (filePossition = BOOTLDR_BIN_CRC_SIZE; filePossition < SdMngr_f_size(&dest_file); filePossition += readWriteSize)
                {
                    if (SdMngr_f_size(&dest_file) - filePossition < readWriteSize) /* check if the last portion of the file is smaller than the buffer size */
                    {
                        readWriteSize = SdMngr_f_size(&dest_file) - filePossition; // Last packet alignment
                    }

                    /* fill the buffer */
                    readStatus = SdMngr_f_read(&dest_file, fileTempBuffSize, readWriteSize, (UINT*)&br);
                    if ((FR_OK != readStatus) || (readWriteSize != br))
                    {
                        fwOperationsResult = BOOTLDR_FW_OP_ERR_READ;
                        break;
                    }

                    /* Calculate the CRC */
                    CRC_data = crc32(CRC_data, (BYTE*)fileTempBuffSize, readWriteSize);
                }

                if (fwOperationsResult == BOOTLDR_FW_OP_RESUS_OK)
                {
                    /* check if the available flash memory is enough to fit the bin file */
                    if ((*codeSize) > MAX_APP_SIZE)
                    {
                        fwOperationsResult = BOOTLDR_FW_OP_ERR_SIZE;
                    }

                    if (CRC_data != *file_CRC)
                    {
                        fwOperationsResult = BOOTLDR_FW_OP_ERR_CRC;
                    }
                }
            }

        }
        SdMngr_f_close(&dest_file);

        if (fwOperationsResult == BOOTLDR_FW_OP_RESUS_OK)
        {
            break;  //no need to repeat any more
        }
    }

    return fwOperationsResult;
}

/*!
********************************************************************************************
* @brief Verifies a file for a firmware update
********************************************************************************************
* @param[input]      inputBinPath - path to the file located in a SD card
* @param[output]     file_CRC - calculated CRC of the .scm file
* @param[output]     codeSize - size of the code that is going to be written with that .scm file
* @return            BOOTLDR_FwOpRes_enum - final status of the process
* @note              none
********************************************************************************************
*/
BOOTLDR_FwOpRes_enum BootLdr_VerifyCrcOfScmFile(FIL *df, const char *filePath, uint32_t *file_CRC, uint32_t *codeSize, TASK_MONITOR_TasksEnum task)
{
    BOOTLDR_FwOpRes_enum fwOperationsResult;
    FRESULT fileResult;
    UINT br;
    uint16_t pack_counter;                  /* calculates number of read operations has been done to indicate progress */
    uint32_t source_file_position;          /* number of bytes processed from the source file */
    uint32_t data_compare_position;         /* position the file before the last write */
    uint8_t  read_back_data_buffer[16*5];   /* 5 rows, 16 bytes per row = 80 */
    uint8_t  row_counter;                   /* counter for number of processed rows */
    uint8_t  number_rows_in_packet;         /* number of rows with data in one buffer read at once */
    uint16_t packet_displacement;           /* number of bytes processed from the buffer filled with one read */
    uint16_t data_displacement;             /* counter for processed de-scrambled bites */
    uint16_t len;                           /* counter for processed de-scrambled bites */
    uint8_t  curr_row_size;                 /* Number of bytes in one row */
    uint8_t  curr_row_checksum;             /* Sum of all data bytes in one row - needed to be compared with the checksum at the end of every row in the Motorola S-record file */

    #define ESTTC_MANAGE_ROWS_AT_A_TIME          (BOOTLDR_BUFF_SIZE/ESTTC_SYMBOLS_PER_ROW_IN_BIN_FILE)
    #define ESTTC_SYMBOLS_PER_ROW_IN_BIN_FILE    (48)
    #define ESTTC_ROWS_AT_A_TIME                 (10)
    #define ESTTC_RECORD_SIZE                    (2)
    #define ESTTC_COUNT_SIZE                     (2)
    #define ESTTC_ADDRESS_SIZE                   (8)
    #define ESTTC_DATA_START_ADDRESS             (( ESTTC_RECORD_SIZE + ESTTC_COUNT_SIZE + ESTTC_ADDRESS_SIZE ) / 2)
    #define ESTTC_CHECKSUM_PROGRESS_SPEED        (50)  /* The smaller number, the faster the progress will go */
    #define ESTTC_MAX_PROGRESS_FOR_FLASH_SIZE    (MAX_APP_SIZE/ESTTC_CHECKSUM_PROGRESS_SPEED + 1)


    SdMngr_f_close(&dest_file);    //Just in case
    SdMngr_f_unlink (BOOTLDR_SCM_TO_BIN_FILE_PATH); /* Clean up if that file exist */

    for (uint8_t i = 0; i < BOOTLDR_AUTOREPEAT_TIMES; i++)
    {
        fwOperationsResult = BOOTLDR_FW_OP_RESUS_OK;
        source_file_position = 18;
        pack_counter = 0;

        fileResult = SdMngr_f_open(&dest_file, BOOTLDR_SCM_TO_BIN_FILE_PATH, FA_WRITE | FA_CREATE_ALWAYS);
        if (FR_OK != fileResult) /* Try to create the file if possible */
        {
            fwOperationsResult = BOOTLDR_FW_OP_ERR_FILENAME;
        }
        else
        {
            SdMngr_f_close(&dest_file);

            br = ESTTC_SYMBOLS_PER_ROW_IN_BIN_FILE; /* Set init value */

            /* while( read data != 0 ) */
            while (br)
            {
                /* Open the scrambled file */
                fileResult = SdMngr_f_open(&dest_file, filePath, FA_READ | FA_OPEN_EXISTING);
                if (FR_OK != fileResult)
                {
                    fwOperationsResult = BOOTLDR_FW_OP_ERR_FILENAME;
                    break;
                }
                else
                {
                    fileResult = SdMngr_f_lseek(&dest_file, source_file_position); /* set position after the first row. First time that is just a header */

                    /* Read ESTTC_SYMBOLS_PER_ROW_IN_BIN_FILE symbols from the file - 5 rows max for 256 byte buffer */
                    fileResult = SdMngr_f_read(&dest_file, fileTempBuffSize, ESTTC_MANAGE_ROWS_AT_A_TIME * ESTTC_SYMBOLS_PER_ROW_IN_BIN_FILE, (UINT*)&br);
                    if (FR_OK != fileResult)
                    {
                        fwOperationsResult = BOOTLDR_FW_OP_ERR_READ;
                        break;
                    }

                    /* close the file */
                    SdMngr_f_close(&dest_file);

                    if (br < 4) /* defence for the next check "(br-2-1) / ESTTC_SYMBOLS_PER_ROW_IN_BIN_FILE)" */
                    {
                        br = ESTTC_MANAGE_ROWS_AT_A_TIME * ESTTC_SYMBOLS_PER_ROW_IN_BIN_FILE;

                        /* if all bytes are processed  then extract the CRC from the end of the file */
                        *file_CRC = HexToBin(fileTempBuffSize[br - 8 - 4], fileTempBuffSize[br - 7 - 4]) << 24 | HexToBin(fileTempBuffSize[br - 6 - 4], fileTempBuffSize[br - 5 - 4]) << 16 | HexToBin(fileTempBuffSize[br - 4 - 4], fileTempBuffSize[br - 3 - 4]) << 8 | HexToBin(fileTempBuffSize[br - 2 - 4], fileTempBuffSize[br - 1 - 4]);

                        /* if none bytes are read exit the while cycle */
                        break;
                    }

                    row_counter = 0; /* set current row in the buffer number to be the first */
                    data_displacement = 0; /* counter for processed de-scrambled bites */
                    number_rows_in_packet = br / ESTTC_SYMBOLS_PER_ROW_IN_BIN_FILE;
                    if ((br % ESTTC_SYMBOLS_PER_ROW_IN_BIN_FILE) > 16) /* check if remaining symbols are more then the last S7 row with the checksum */
                    {
                        /* if yes, there is a not full row with S3 record data */
                        number_rows_in_packet++;
                    }

                    for (row_counter = 0; row_counter < number_rows_in_packet; row_counter++)
                    {
                        source_file_position += ESTTC_SYMBOLS_PER_ROW_IN_BIN_FILE; /* Set the position to the next row in the file */
                        packet_displacement = row_counter * ESTTC_SYMBOLS_PER_ROW_IN_BIN_FILE; /* Set position to the next row in the packet */

                        if (fileTempBuffSize[packet_displacement + 1] != '3') /* check if the file format is right */
                        {
                            fwOperationsResult = BOOTLDR_FW_OP_ERR_FILE_FORMAT;
                            break;
                        }

                        /* Calculate the checksum of the row and the number of data bytes in the row */
                        curr_row_checksum = curr_row_size = HexToBin(fileTempBuffSize[packet_displacement + 2], fileTempBuffSize[packet_displacement + 2 + 1]);
                        curr_row_size = curr_row_size + 1 - ESTTC_DATA_START_ADDRESS;

                        for (BYTE i = ESTTC_RECORD_SIZE + ESTTC_COUNT_SIZE; i < ESTTC_RECORD_SIZE + ESTTC_COUNT_SIZE + ESTTC_ADDRESS_SIZE; i += 2)
                        {
                            curr_row_checksum += HexToBin(fileTempBuffSize[packet_displacement + i], fileTempBuffSize[packet_displacement + i + 1]);
                        }

                        /* S-record de-scrambling */

                        /* Extract data from the packet - transform from two ascii symbols to hex */
                        for (BYTE i = ESTTC_RECORD_SIZE + ESTTC_COUNT_SIZE + ESTTC_ADDRESS_SIZE, j = 0; j <= curr_row_size; i += 2, j++)
                        {
                            // S-record HEX->BIN conversion
                            fileTempBuffSize[data_displacement + j] = HexToBin(fileTempBuffSize[packet_displacement + i], fileTempBuffSize[packet_displacement + i + 1]);
                            if (j < curr_row_size)
                            {
                                curr_row_checksum += fileTempBuffSize[data_displacement + j];
                            }
                        }
                        /* de-scrambling */
                        for (BYTE i = 0, j = 7; i < curr_row_size; i++, j++)
                        {
                            fileTempBuffSize[data_displacement + i] ^= (j * 4 + 1);
                        }
                        /*  End of S-record de-scrambling */

                        /* calculate checksum */
                        curr_row_checksum = 0xFF - (curr_row_checksum & 0xFF);
                        if (curr_row_checksum != fileTempBuffSize[data_displacement + curr_row_size])
                        {
                            fwOperationsResult = BOOTLDR_FW_OP_ERR_CRC;
                            break;
                        }

                        data_displacement += curr_row_size;
                    }

                    if (fwOperationsResult == BOOTLDR_FW_OP_RESUS_OK)
                    {
                        /* Open the destination file */
                        fileResult = SdMngr_f_open(&dest_file, BOOTLDR_SCM_TO_BIN_FILE_PATH, FA_WRITE | FA_READ);
                        if (FR_OK != fileResult)
                        {
                            fwOperationsResult = BOOTLDR_FW_OP_ERR_FILENAME;
                            break;
                        }

                        /* go to the end of the file */
                        fileResult = SdMngr_f_lseek(&dest_file, SdMngr_f_size(&dest_file));
                        data_compare_position = SdMngr_f_size(&dest_file);

                        /* write the de-scrambled data to the output file */
                        if (fileResult == FR_OK)
                        {
                            fileResult = SdMngr_f_write(&dest_file, &fileTempBuffSize[0], data_displacement, (UINT*)&len);

                            SdMngr_f_close(&dest_file);

                            if (fileResult != FR_OK)
                            {
                                fwOperationsResult = BOOTLDR_FW_OP_ERR_WRITE;
                                break;
                            }
                            else
                            {

                                /* read back the just written data  and compare it with the buffer */
                                fileResult = SdMngr_f_open(&dest_file, BOOTLDR_SCM_TO_BIN_FILE_PATH, FA_WRITE | FA_READ);
                                if (fileResult == FR_OK)
                                {
                                    fileResult = SdMngr_f_lseek(&dest_file, data_compare_position);
                                    if (fileResult == FR_OK)
                                    {
                                        fileResult = SdMngr_f_read(&dest_file, read_back_data_buffer, data_displacement, (UINT*)&len);
                                        if (fileResult == FR_OK)
                                        {
                                            if (0 != memcmp(&fileTempBuffSize[0], read_back_data_buffer, data_displacement))
                                            {
                                                fwOperationsResult = BOOTLDR_FW_OP_ERR_COMPARE;
                                                break;
                                            }
                                            else
                                            {
                                                /* close the destination file */
                                                SdMngr_f_close(&dest_file);

                                                /* check if all bytes are read */
                                                if (br < ESTTC_MANAGE_ROWS_AT_A_TIME * ESTTC_SYMBOLS_PER_ROW_IN_BIN_FILE)
                                                {
                                                    /* if all bytes are processed  then extract the CRC from the end of the file */
                                                    *file_CRC = HexToBin(fileTempBuffSize[br - 8 - 4], fileTempBuffSize[br - 7 - 4]) << 24 | HexToBin(fileTempBuffSize[br - 6 - 4], fileTempBuffSize[br - 5 - 4]) << 16 | HexToBin(fileTempBuffSize[br - 4 - 4], fileTempBuffSize[br - 3 - 4]) << 8 | HexToBin(fileTempBuffSize[br - 2 - 4], fileTempBuffSize[br - 1 - 4]);

                                                    /* if some bytes are read, finish them and then exit the while cycle */
                                                    br = 0;
                                                }

                                                pack_counter++;

                                                /* just for the user to see that something is working */
                                                if (pack_counter > ESTTC_CHECKSUM_PROGRESS_SPEED)
                                                {
                                                    //fprintf(ComInterface, "*");
                                                    pack_counter = 0;

                                                    /* if the size of the flash is not exceeded keep that process running (prevent from watchdog reset) */
                                                    if (ESTTC_MAX_PROGRESS_FOR_FLASH_SIZE > pack_counter)
                                                    {
                                                        TaskMonitor_IamAlive(TASK_MONITOR_ESTTC); /* Prevent from WatchDog reset */
                                                    }
                                                }
                                            }
                                        }
                                        else
                                        {
                                            fwOperationsResult = BOOTLDR_FW_OP_ERR_READ;
                                            break;
                                        }
                                    }
                                    else
                                    {
                                        fwOperationsResult = BOOTLDR_FW_OP_ERR_READ;
                                        break;
                                    }
                                }
                                else
                                {
                                    fwOperationsResult = BOOTLDR_FW_OP_ERR_FILENAME;
                                    break;
                                }
                            }
                        }
                        else
                        {
                            fwOperationsResult = BOOTLDR_FW_OP_ERR_READ;
                            break;
                        }
                    }
                    else
                    {
                        break;
                    }
                }
            }    //End of while(br)

            /* Report to task monitor the that this task is still alive and is going */
            TaskMonitor_IamAlive(task); /* Prevent from WatchDog reset */

            if (fwOperationsResult == BOOTLDR_FW_OP_RESUS_OK)
            {
                /* Open the de-scrambled file from the beginning in read mode to have file size so we know how much flash needs to be erased */
                fileResult = SdMngr_f_open(&dest_file, BOOTLDR_SCM_TO_BIN_FILE_PATH, FA_READ | FA_OPEN_EXISTING);
                if (FR_OK != fileResult)
                {
                    fwOperationsResult = BOOTLDR_FW_OP_ERR_FILENAME;
                }
                else
                {

                    *codeSize = SdMngr_f_size(&dest_file);

                    /* check if the available flash memory is enough to fit the bin file */
                    if ((*codeSize) > MAX_APP_SIZE)
                    {
                        fwOperationsResult = BOOTLDR_FW_OP_ERR_SIZE;
                    }
                }
            }
        }

        SdMngr_f_close(&dest_file);

        if (fwOperationsResult == BOOTLDR_FW_OP_RESUS_OK)
        {
            break;  //no need to repeat any more
        }
        else
        {
            /* erase the bin file */
            SdMngr_f_unlink(BOOTLDR_SCM_TO_BIN_FILE_PATH);
        }
    }

    return BOOTLDR_FW_OP_RESUS_OK;
}

/*!
********************************************************************************************
* @brief Perform a firmware update from a file
********************************************************************************************
* @param[input]      inputBinPath - path to the file located in a SD card
* @param[output]     file_CRC - calculated CRC of the .scm file
* @param[output]     codeSize - size of the code that is going to be written with that .scm file
* @return            BOOTLDR_FwOpRes_enum - final status of the process
* @note              none
********************************************************************************************
*/
BOOTLDR_FwOpRes_enum BootLdr_FlashingFromFile(TCHAR *inputBinPath, uint8_t startPosition, uint32_t fileCRC)
{
    BOOTLDR_FwOpRes_enum fwOperationsResult = BOOTLDR_FW_OP_RESUS_OK;
    uint16_t readWriteSize = BOOTLDR_BUFF_SIZE;
    uint32_t filePossition;
    uint32_t codeSize;
    FRESULT readStatus;
    UINT br;

    TCHAR binPath[BOOTLDRSCM_TO_BIN_FILE_NAME_LEN];
    uint8_t unlinkOnExit = 0;
    if (inputBinPath == NULL)    //Temp file created in BootLdr_VerifyCrcOfScmFile()
    {
        memcpy(binPath, BOOTLDR_SCM_TO_BIN_FILE_PATH, BOOTLDRSCM_TO_BIN_FILE_NAME_LEN);

        inputBinPath = binPath;
        unlinkOnExit = 1;
    }

    SdMngr_f_close(&dest_file);    //Just in case

    for (uint8_t i = 0; i < BOOTLDR_AUTOREPEAT_TIMES; i++)
    {
        readStatus = SdMngr_f_open(&dest_file, inputBinPath, FA_READ | FA_OPEN_EXISTING);
        if (FR_OK != readStatus)
        {
            fwOperationsResult = BOOTLDR_FW_OP_ERR_FILENAME;
        }
        else
        {
            readStatus = SdMngr_f_lseek(&dest_file, startPosition);
            if (readStatus != FR_OK)
            {
                fwOperationsResult = BOOTLDR_FW_OP_ERR_READ;
            }
            else
            {
                uint8_t writeStatus;
                HAL_StatusTypeDef unlockStatus = HAL_FLASH_Unlock();

                if (HAL_OK == unlockStatus)
                {
                    /* Start writing the flash memory */
                    readWriteSize = BOOTLDR_BUFF_SIZE;
                    for (filePossition = startPosition; filePossition < SdMngr_f_size(&dest_file); filePossition += readWriteSize)
                    {
                        if (SdMngr_f_size(&dest_file) - filePossition < readWriteSize) /* check if the last portion of the file is smaller than the buffer size */
                        {
                            readWriteSize = SdMngr_f_size(&dest_file) - filePossition; // Last packet alignment
                        }

                        /* fill the buffer */
                        readStatus = SdMngr_f_read(&dest_file, fileTempBuffSize, readWriteSize, (UINT*)&br);

                        if ((FR_OK != readStatus) || (readWriteSize != br))
                        {
                            fwOperationsResult = BOOTLDR_FW_OP_ERR_READ;
                            break;
                        }

                        writeStatus = FLASH_Write((uint8_t*)fileTempBuffSize, FLASH_MIN_ADDR + filePossition - startPosition, readWriteSize);

                        /* Write the data from the bin file to the flash memory */
                        if (HAL_OK != writeStatus)
                        {
                            fwOperationsResult = BOOTLDR_FW_OP_ERR_WRITE;
                            break;
                        }
                    }

                    codeSize = SdMngr_f_size(&dest_file) - startPosition;
                    SdMngr_f_close(&dest_file);

                    if (filePossition - startPosition != codeSize)
                    {
                        fwOperationsResult = BOOTLDR_FW_OP_ERR_WRITE_INCOMPLETE;
                    }

                    FlashInfoStruct FlashData;
                    FlashData.CodeSize = codeSize;
                    FlashData.CodeCRC = fileCRC;

                    //Flash the info about the bin file, so it can be checked later at boot-up
                    writeStatus = FLASH_Write((uint8_t*)&FlashData, FLASH_INFO_ADDRESS, sizeof(FlashData));
                    if (HAL_OK != writeStatus)
                    {
                        fwOperationsResult = BOOTLDR_FW_OP_ERR_WRITE;
                    }
                }
                else
                {
                    fwOperationsResult = BOOTLDR_FW_OP_ERR_WRITE;
                }
            }
        }

        SdMngr_f_close(&dest_file);

        if (fwOperationsResult == BOOTLDR_FW_OP_RESUS_OK)
        {
            break;  //no need to repeat any more
        }
    }

    if (unlinkOnExit)
    {
        /* erase the bin file */
        SdMngr_f_unlink(BOOTLDR_SCM_TO_BIN_FILE_PATH);
    }

    return fwOperationsResult;
}

/*!
********************************************************************************************
* @brief Erase part of the flash memory according the a given size
********************************************************************************************
* @param[input]      codeSize - needed minimum size
* @param[output]     errasedSectors - number of erased sectors (each 256 KB)
* @return            BOOTLDR_FwOpRes_enum - final status of the process
* @note              none
********************************************************************************************
*/
HAL_StatusTypeDef BootLdr_EraseFlash(uint32_t codeSize, uint8_t *errasedSectors)
{
    uint8_t numberSectors;
    uint32_t i;
    HAL_StatusTypeDef eraseState;
    uint32_t sectorsError;
    FLASH_EraseInitTypeDef fes;
    FLASH_OBProgramInitTypeDef    OBInit;

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    /* calculate number of sector to erase */
    numberSectors = getLastSectorForSize(FLASH_MIN_SECTOR, codeSize) - FLASH_MIN_SECTOR + 1;

    /* Unlock the Flash to enable the flash control register access *************/
        eraseState = HAL_FLASH_Unlock();
    /* Allow Access to option bytes sector */
        eraseState = HAL_FLASH_OB_Unlock();
    /* Get the Dual bank configuration status */
    HAL_FLASHEx_OBGetConfig(&OBInit);


  #if (ENABLE_DUAL_BANK == 1)/* Check for wrong bank configuration */
    if((OBInit.USERConfig & OB_NDBANK_SINGLE_BANK) == OB_NDBANK_SINGLE_BANK)
  #else
    if((OBInit.USERConfig & OB_NDBANK_SINGLE_BANK) == OB_NDBANK_DUAL_BANK)
  #endif
    {
        Error_Handler();
    }

    for (i = FLASH_MIN_SECTOR; i < (FLASH_MIN_SECTOR + numberSectors); i++)
    {
        fes.TypeErase = FLASH_TYPEERASE_SECTORS;
        fes.NbSectors = 1;

        fes.Sector = i;

        fes.VoltageRange = FLASH_VOLTAGE_RANGE_3;

        __disable_interrupt();//taskENTER_CRITICAL();

        /* Erase the number of sectors needed to fit the current bin file */
        /* erase the last sector to be able to write the CRC at the and of the flash */
        eraseState = HAL_FLASHEx_Erase(&fes, &sectorsError);
        if (HAL_OK != eraseState)
        {
            *errasedSectors = i;
            return eraseState;
        }
    }

    *errasedSectors = numberSectors;

    if (i < (FLASH_SECTORS_COUNT - 1))/* if the last sector is not erased */
    {
        fes.Sector = FLASH_SECTORS_COUNT - 1;
        /* erase the last sector to be able to write the CRC at the and of the flash */
        eraseState = HAL_FLASHEx_Erase(&fes, &sectorsError);
        if (HAL_OK != eraseState)
        {
            *errasedSectors = i;
            return eraseState;
        }
        else
        {
            *errasedSectors += 1;
        }
    }

    __enable_interrupt();//taskEXIT_CRITICAL();

    return HAL_OK;
}

/*!
********************************************************************************************
* @brief Checks if a part of the flash is really erased
********************************************************************************************
* @param[input]      addr - starting address
* @param[input]      len - size of the flash
* @param[output]     none
* @return            HAL_OK - if OK, a number > 0 - if any flash cell not erased
* @note              none
********************************************************************************************
*/
uint32_t BootLdr_FlashEmptyPatternCheck(uint32_t addr, uint32_t len)
{
    uint32_t i;

    if (addr < FLASH_MIN_ADDR)
        return HAL_ERROR;

    if ((addr + len) > (FLASH_MIN_ADDR + MAX_APP_SIZE))
        return HAL_ERROR;

    for (i = addr; i < addr + len; i++)
    {
        if (FLASH_BLANK != *((uint8_t*)i))
        {
            return i;
        }
    }

    return HAL_OK;
}


/*!
********************************************************************************************
* @brief Checks if a part of the flash is really erased
********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            pdFALSE - The CRC of the flash memory is NOT valid, pdTRUE - The CRC of the flash memory is valid
* @note              none
********************************************************************************************
*/
uint8_t BootLdr_FlashVerifyCRC(void)
{
    uint8_t retVal = 0;
    FlashInfoStruct* FlashData = (FlashInfoStruct*)FLASH_INFO_ADDRESS;

    if (FlashData->CodeSize <= MAX_APP_SIZE)
    {
        uint32_t flashCRC = crc32(0, (uint8_t*)FLASH_MIN_ADDR, FlashData->CodeSize);
        if (FlashData->CodeCRC == flashCRC)
        {
            retVal = 1;
        }
    }
    return retVal;
}

/*!
********************************************************************************************
* @brief Jump to Application with a reset
********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
********************************************************************************************
*/
void BootLdr_ImmediatAppJump(void)
{
    /* jump from bootloader to application by reset */
    BootData->Mailbox = MAILBOX_VAL_APPL; //*((__IO uint32_t *)MAILBOX_ADDRESS) = MAILBOX_VAL_APPL;
    BootData->RebootRequest = pdTRUE;

    Svc_Rtc_ClearResetCounter(RSTCOUNTERID_ALL);
    RTC_SetRTC_BKP_CRC();      /* set valid checksum for the RTC backup registers after changing the mailbox */

    __disable_interrupt();
    SCB->VTOR = APPL_ADDRESS;
    __enable_interrupt();

    TaskMonitor_ImmediatReset(TASK_MONITOR_SW_RESET);
}


uint32_t BootLdr_FlashCalcBootCRC(void)
{
    uint32_t flashCRC = crc32( 0, (uint8_t*)BOOT_ADDRESS, APPL_ADDRESS-BOOT_ADDRESS);

    return flashCRC;
}

/*
********************************************************************************************
* INTERNAL (STATIC) ROUTINES DEFINITION
********************************************************************************************
*/
/*!
********************************************************************************************
* @brief Writes a peace of flash memory
********************************************************************************************
* @param[input]      src - pointer to the data to write
* @param[input]      addr - starting address
* @param[input]      len - size of the data
* @param[output]     none
* @return            none
* @note              none
********************************************************************************************
*/
static HAL_StatusTypeDef FLASH_Write(uint8_t *src, uint32_t addr, uint32_t len)
{
    if ((src == NULL) || (addr < FLASH_MIN_ADDR) || ((addr + len) > (FLASH_MAX_ADDR)))
        return HAL_ERROR;

    HAL_StatusTypeDef res = HAL_OK;
    uint32_t prog_size = FLASH_TYPEPROGRAM_DOUBLEWORD;
    uint32_t addr_inc = sizeof(uint64_t);

    {
        prog_size = FLASH_TYPEPROGRAM_BYTE;
        addr_inc = sizeof(uint8_t);
    }

    for (uint32_t i = 0; i < len; i += addr_inc)
    {
        res = HAL_FLASH_Program(prog_size, addr + i, src[i]);
        if (res != HAL_OK)
            break;
    }

    return res;
}

/*!
*********************************************************************************************
* @brief Hexadecimal ASCII to binary number converted
*********************************************************************************************
* @param[input]      hb - high significant part of the byte
*                    lb - low significant part of the byte
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
static uint8_t HexToBin(uint8_t hb, uint8_t lb)
{
    uint8_t thb = hb, tlb = lb;

    if (thb > '9')
        thb += 9;
    if (tlb > '9')
        tlb += 9;

    return (thb << 4) + (tlb & 0x0f);
}

/*!
********************************************************************************************
* @brief Perfore a firmware update from a file
********************************************************************************************
* @param[input]      inputBinPath - path to the file located in a SD card
* @param[input]      strLength - length of the path
* @param[output]     none
* @return            none
* @note              none
********************************************************************************************
*/
static void BootLdr_AutoFlashFromBin(TCHAR *inputBinPath, uint8_t strLength)
{
    FRESULT fileOperationsResult;
    TCHAR binPath[BOOTLDR_MAX_FILE_NAME_LEN];

    const char message0[] = "Open file error\r\n";
    const char message1[] = "CRC error\r\n";
    const char message2[] = "Erase error\r\n";
    const char message3[] = "Erase check error\r\n";
    const char message4[] = "Flash from file error\r\n";
    const char message5[] = "Flash CRC error\r\n";
    const char message6[] = "FW update already started\r\n";

    const char *messages[] = {message0, message1, message2, message3, message4, message5, message6};

    if (BootLdr_flashingInProggress == pdFALSE)
    {
        BootLdr_flashingInProggress = pdTRUE;   //Set flag for started Flashing
    }
    else
    {
        fprintf(COMM, messages[6]);
        fprintf(SYSCON, messages[6]);   //Print: "FW update already started"
        return;
    }

    if (inputBinPath == NULL)
    {
        memcpy(binPath, BOOTLDR_AUTOFLASH_FILE_PATH, BOOTLDR_MAX_FILE_NAME_LEN);
    }
    else
    {
        if ((strLength > BOOTLDR_MAX_FILE_NAME_LEN) || (strLength == 0))
        {
            return;
        }
        memcpy(binPath, inputBinPath, strLength);
    }

    fileOperationsResult = SdMngr_f_open(&dest_file, binPath, FA_READ | FA_OPEN_EXISTING);

    if (FR_OK == fileOperationsResult)
    {
        uint8_t fileCrcStatus;
        uint32_t file_CRC;
        uint32_t codeSize;

        fileCrcStatus = BootLdr_VerifyCrcAndSizeOfBinFile(binPath, &file_CRC, &codeSize);

        if (fileCrcStatus == 0)
        {
            uint8_t errasedSectors;
            HAL_StatusTypeDef erraseStatus = BootLdr_EraseFlash(codeSize, &errasedSectors);

            if (erraseStatus == HAL_OK)
            {
                uint32_t checkedSectorsStatus;

                checkedSectorsStatus = BootLdr_FlashEmptyPatternCheck(FLASH_MIN_ADDR, codeSize);

                if (HAL_OK == checkedSectorsStatus)
                {
                    uint8_t writeStatus;

                    writeStatus = BootLdr_FlashingFromFile(binPath, BOOTLDR_BIN_CRC_SIZE, file_CRC);
                    if (writeStatus == 0)
                    {
                        uint32_t CRC_data;

                        CRC_data = crc32(0, (uint8_t*)FLASH_MIN_ADDR, codeSize);

                        if (CRC_data == file_CRC)
                        {
                            BootLdr_ImmediatAppJump();
                        }
                        else
                        {
                            fprintf(COMM, messages[5]);
                            fprintf(SYSCON, messages[5]);   //Print: "Flash CRC error"
                        }
                    }
                    else
                    {
                        fprintf(COMM, messages[4]);
                        fprintf(SYSCON, messages[4]);   //Print: "Flash from file Error"
                    }
                }
                else
                {
                    fprintf(COMM, messages[3]);
                    fprintf(SYSCON, messages[3]);   //Print: "Erase Check Error"
                }
            }
            else
            {
                fprintf(COMM, messages[2]);
                fprintf(SYSCON, messages[2]);   //Print: "Erase Error"
            }
        }
        else
        {
            fprintf(COMM, messages[1]);
            fprintf(SYSCON, messages[1]);   //Print: "CRC Error"
        }
    }
    else
    {
        fprintf(COMM, messages[0]);
        fprintf(SYSCON, messages[0]);   //Print: "Verifying CRC"
    }

    BootLdr_flashingInProggress = pdFALSE;
}

/*!
********************************************************************************************
* @brief The function calculates the ID of the last flash sector starting from an initial
*        sector id and a data block size (it is needed because the FLASH sectors are not of
*        equal size and can be different depending on DUAL or SINGLE bank configuration.
********************************************************************************************
* @param[input]      u8StartSector - zero-based sector ID to start from
* @param[input]      u32DataSizeBytes - size of data block to accommodate in FLASH (in bytes)
* @return            zero-based index of the end sector which will be occupied by the data
*                    block if written starting from u32StartSector
* @note              none
********************************************************************************************
*/
static uint8_t getLastSectorForSize(const uint8_t u8StartSector, const uint32_t u32DataSizeBytes)
{
    static const uint8_t au8SectorSizesKb[FLASH_SECTORS_COUNT] =
    {
#if (ENABLE_DUAL_BANK == 1)  // Option byte -> nDBANK = 0
        /* Sector 0 */  16U,
        /* Sector 1 */  16U,
        /* Sector 2 */  16U,
        /* Sector 3 */  16U,
        /* Sector 4 */  64U,
        /* Sector 5 */  128U,
        /* Sector 6 */  128U,
        /* Sector 7 */  128U,
        /* Sector 8 */  128U,
        /* Sector 9 */  128U,
        /* Sector 10 */ 128U,
        /* Sector 11 */ 128U,
        /* Sector 12 */ 16U,
        /* Sector 13 */ 16U,
        /* Sector 14 */ 16U,
        /* Sector 15 */ 16U,
        /* Sector 16 */ 64U,
        /* Sector 17 */ 128U,
        /* Sector 18 */ 128U,
        /* Sector 19 */ 128U,
        /* Sector 20 */ 128U,
        /* Sector 21 */ 128U,
        /* Sector 22 */ 128U,
        /* Sector 23 */ 128U
#else
        /* Sector 0 */  32U,
        /* Sector 1 */  32U,
        /* Sector 2 */  32U,
        /* Sector 3 */  32U,
        /* Sector 4 */  128U,
        /* Sector 5 */  256U,
        /* Sector 6 */  256U,
        /* Sector 7 */  256U,
        /* Sector 8 */  256U,
        /* Sector 9 */  256U,
        /* Sector 10 */ 256U,
        /* Sector 11 */ 256U
#endif
};

    uint32_t u32TempSize = 0U;
    uint8_t u8CurrentSector;

    for (u8CurrentSector = u8StartSector;
         u8CurrentSector < sizeof(au8SectorSizesKb) / sizeof(au8SectorSizesKb[0U]);
         u8CurrentSector++)
    {
        u32TempSize += au8SectorSizesKb[u8CurrentSector];

        if ((u32TempSize * 1024U) >= u32DataSizeBytes)
            break;
    }

    return u8CurrentSector;
}

bool BootLdr_IsBootloaderImageActive(void)
{
#if defined(BOOTLOADER)
    return true;
#else
    return false;
#endif
}

/* **************************************************************************************** */
