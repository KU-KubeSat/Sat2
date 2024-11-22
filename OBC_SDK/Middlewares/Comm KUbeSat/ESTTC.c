/*!
********************************************************************************************
* @file ESTTC.c
* @brief EnduroSat telemetry and telecommand communication protocol
********************************************************************************************
* @author            Kolio
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
* @revision{         1.0.0  , 2018.07.04, author Kolio, Initial revision }
* @endhistory
********************************************************************************************
*/

/*
*********************************************************************************************
* INCLUDES
*********************************************************************************************
*/
#include <stdbool.h>
#include "AppTasks.h"
#include "ESTTC.h"
#include "MCU_Init.h"
#include "panels.h"
#include "version.h"
#include "DAT_Inputs.h"
#include "TaskMonitor.h"
#include "es_crc32.h"
#include "Svc_RTC.h"
#include "EEPROM_emul.h"
#include "AntUHF.h"
#include "stm32f7xx_it.h"
#include <uuid.h>
#include <ctype.h>
#include "PwrMng.h"
#include "MX_I2C.h"
#include "es_exeh.h"
#include "BootLdr.h"
#include <string.h>
#include "SdMngr.h"
#include "OEM719.h"
#include "Camera.h"


/*
 *********************************************************************************************
 * INTERNAL DEFINES
 *********************************************************************************************
 */
#define ESTTS_TASK_TIME_CYCLE (5000)                                      /* Time in ms, after which the task will be called again */
#define ESTTS_MAX_PIPE_TIMEOUT ((1000 / ESTTS_TASK_TIME_CYCLE) * 15 * 60) /* 15 min is the maximum possible time for PIPE mode of the UHF transmitter */
#define ESTTS_SYNC_FILE_PACKETS (200)                                     /* Max number of packets without synchronisation of the file with the buffers */
#define ESTTC_CYMBOLS_IN_CRC (9)                                          /* 8 symbols for 4 bytes hex + 1 symbol "space" to separate CRC from the command */
#define ESTTC_NUMBER_READ_CMDS (39)                                       /* Number of read commands */
#define ESTTC_NUMBER_WRITE_CMDS (19)                                      /* Number of write commands */

/*
*********************************************************************************************
* INTERNAL TYPES DEFINITION
*********************************************************************************************
*/
/* No Internal types definition */

/*
*********************************************************************************************
* EXTERNAL VARIABLES DEFINITION
*********************************************************************************************
*/
/**
* @brief Arbitrary UART (COM1, COM4, COM6) ISR reception buffer
*/
static volatile char RxBuffer[ESTTC_INTERFACE_NUMBER][UART_BUFFER_SIZE];
/**
* @brief Arbitrary UART (COM1, COM4, COM6) ISR reception buffer head and tail positions and length
*/
static volatile uint32_t RxBuffHead[ESTTC_INTERFACE_NUMBER], RxBuffTail[ESTTC_INTERFACE_NUMBER], RxBuffLen[ESTTC_INTERFACE_NUMBER];

/*
*********************************************************************************************
* INTERNAL (STATIC) VARIABLES DEFINITION/DECLARATION 
*********************************************************************************************
*/
static char txline[ESTTC_INTERFACE_NUMBER][LINE_BUFFER_SIZE];
static char rxline[ESTTC_INTERFACE_NUMBER][LINE_BUFFER_SIZE];
static FILE* const Esttc_usart_interfaces[ESTTC_INTERFACE_NUMBER] = {COMM, PAYLOAD, SYSCON};
static uint8_t Esttc_interface_data[ESTTC_INTERFACE_NUMBER] = {0};
static uint32_t fpos[ESTTC_INTERFACE_NUMBER], bsize[ESTTC_INTERFACE_NUMBER];
static FILINFO fno[ESTTC_INTERFACE_NUMBER];
static uint16_t pack_data_position[ESTTC_INTERFACE_NUMBER] = {0, 0, 0};
static uint16_t ESTTC_sync_file_timeout[ESTTC_INTERFACE_NUMBER];     /* After starting to write a file, if the communication is stopped but the file is not closed or synchronised, it will happen automatically */
static uint16_t ESTTC_sync_file_numbPackets[ESTTC_INTERFACE_NUMBER]; /* After starting to write a file, sync the file after a certain number of packets */
static FIL df[ESTTC_INTERFACE_NUMBER];
static DIR dd[ESTTC_INTERFACE_NUMBER];
static char print_buff[ESTTC_INTERFACE_NUMBER][LINE_BUFFER_SIZE + 1 + ESTTC_CYMBOLS_IN_CRC + 1 + 1]; /* buffer just to print responses = data + Checksum + CRC + \r + \0 */
static uint8_t ESTTC_UhfPipeMode = 0;
static uint16_t ESTTC_UhfPipeModeTimeout = 0;

osThreadId_t ESTTC_UART_TASK_TaskHandle;
static const osThreadAttr_t ESTTC_UART_TASK_attributes =
{
    .name = "ESTTC_Task",
    .priority = (osPriority_t)osPriorityNormal,
    .stack_size = 128 * 18
};

UhfMasterObcSlaveCmds_str uhfCmdStrings[3] =
{
    {
		.uhfCmdStrings = {'+', 'P', 'I', 'P', 'E', 0},
     	.PacketLength = 5,
	    .CmdNumber = 0
	},
    {
		.uhfCmdStrings = {'+', 'E', 'S', 'T', 'T', 'C', 0},
     	.PacketLength = 6,
     	.CmdNumber = 0
	},
    {
		.uhfCmdStrings = {'+', 'P', 'H', 'O', 'E', 'N', 'I', 'X', 0},
     	.PacketLength = 8,
     	.CmdNumber = 1
	}
};

const char cVersionFormatString[] = "OK+%s/Dev %d: v.%d.%0d / <%s %s> [%s] / SN#: %s%s";

/*
*********************************************************************************************
* INTERNAL (STATIC) ROUTINES DECLARATION
*********************************************************************************************
*/
static uint32_t GetPhrase(char *dst, uint32_t len, char term, ESTTC_InterfacesEnum Interface);
static int ESTTC_getchar(ESTTC_InterfacesEnum Interface);
static uint8_t ESTTC_AvailableCommands(void);
static int getbyte(uint32_t tmt_ms, ESTTC_InterfacesEnum Interface);
static uint8_t HexToBin(uint8_t hb, uint8_t lb, uint8_t *hexVal);
static uint8_t ESTTC_ProcessData(ESTTC_InterfacesEnum Interface);
static void ESTTC_UART_TASK(void *argument);
static uint16_t ESTTC_GetCmdLength(uint8_t cmd_type, uint8_t CMD);
static uint8_t ESTTC_ExtractCRC(char *crc_buffer, uint32_t *CalcCRC);

//ESTTC converter from API
#include "ESTTC_CmdHandlers.c"

/*
*********************************************************************************************
* EXTERNAL (NONE STATIC) ROUTINES DEFINITION
*********************************************************************************************
*/
/*!
*********************************************************************************************
* @brief Initialises the ESTTC (EnduroSat Telemetry and TeleCommunications) FreeRTOS task
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              On HAL error, invokes Error_Handler() method
*********************************************************************************************
*/
void ESTTC_InitTask(void)
{
    ESTTC_UART_TASK_TaskHandle = osThreadNew(ESTTC_UART_TASK, NULL, &ESTTC_UART_TASK_attributes);
    configASSERT(ESTTC_UART_TASK_TaskHandle);
}

/*!
*********************************************************************************************
* @brief Return the handler of the ESTTC task
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
osThreadId_t ESTTC_GetTaskHandler(void)
{
    return ESTTC_UART_TASK_TaskHandle;
}

/*!
*********************************************************************************************
* @brief Initialises the ESTTC (EnduroSat Telemetry and TeleCommunications) component
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              On HAL error, invokes Error_Handler() method
*********************************************************************************************
*/
void ESTTC_Init(void)
{
    uint8_t u8_index;
    HAL_StatusTypeDef retVal;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();

    for (u8_index = 0; u8_index < ESTTC_INTERFACE_NUMBER; u8_index++)
    {
        df[u8_index].obj.fs = 0;

        RxBuffHead[u8_index] = 0;
        RxBuffTail[u8_index] = 0;
        RxBuffLen[u8_index] = 0;

        fpos[u8_index] = 0;
        bsize[u8_index] = 0;
        ESTTC_sync_file_timeout[u8_index] = 0;
        ESTTC_sync_file_numbPackets[u8_index] = 0;
    }

    if (Esttc_usart_interfaces[ESTTC_COMM_INTEFACE] != INVALID_INTERFACE)
    {
        MX_UART5_Init();
        HAL_UART_Abort_IT((UART_HandleTypeDef*)Esttc_usart_interfaces[ESTTC_COMM_INTEFACE]);
        retVal = HAL_UART_Receive_IT((UART_HandleTypeDef*)Esttc_usart_interfaces[ESTTC_COMM_INTEFACE], (uint8_t*)&Esttc_interface_data[ESTTC_COMM_INTEFACE], 1);
        if (HAL_OK != retVal)
        {
            Error_Handler();
        }
    }

    if (Esttc_usart_interfaces[ESTTC_PAYLOAD_INTEFACE] != INVALID_INTERFACE)
    {
        MX_UART7_Init();
        HAL_UART_Abort_IT((UART_HandleTypeDef*)Esttc_usart_interfaces[ESTTC_PAYLOAD_INTEFACE]);
        retVal = HAL_UART_Receive_IT((UART_HandleTypeDef*)Esttc_usart_interfaces[ESTTC_PAYLOAD_INTEFACE], (uint8_t*)&Esttc_interface_data[ESTTC_PAYLOAD_INTEFACE], 1);
        if (HAL_OK != retVal)
        {
            Error_Handler();
        }
    }

    if (Esttc_usart_interfaces[ESTTC_SYSCOMM_INTEFACE] != INVALID_INTERFACE)
    {
        MX_UART8_Init();
        HAL_UART_Abort_IT((UART_HandleTypeDef*)Esttc_usart_interfaces[ESTTC_SYSCOMM_INTEFACE]);
        retVal = HAL_UART_Receive_IT((UART_HandleTypeDef*)Esttc_usart_interfaces[ESTTC_SYSCOMM_INTEFACE], (uint8_t*)&Esttc_interface_data[ESTTC_SYSCOMM_INTEFACE], 1);
        if (HAL_OK != retVal)
        {
            Error_Handler();
        }
    }
}

/*!
*********************************************************************************************
* @brief Deinitialises the ESTTC (EnduroSat Telemetry and TeleCommunications) component
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              On HAL error, invokes Error_Handler() method
*********************************************************************************************
*/
void ESTTC_DeInit(void)
{
    if (Esttc_usart_interfaces[ESTTC_COMM_INTEFACE] != INVALID_INTERFACE)
    {
        MX_UART5_DeInit();
    }

    if (Esttc_usart_interfaces[ESTTC_PAYLOAD_INTEFACE] != INVALID_INTERFACE)
    {
        MX_UART7_DeInit();
    }

    if (Esttc_usart_interfaces[ESTTC_SYSCOMM_INTEFACE] != INVALID_INTERFACE)
    {
        MX_UART8_DeInit();
    }
}

/*!
*********************************************************************************************
* @brief Weak function to be implemented elsewhere to be called on event from the ESTTC
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              On HAL error, invokes Error_Handler() method
*********************************************************************************************
*/
WEAK_CBK void ESTTC_Notif_OnEvent(const ESTTC_EventsEnum event, const ESTTC_InterfacesEnum Interface)
{
    (void)event;
    (void)Interface;
}

/*!
*********************************************************************************************
* @brief Notify the ESTTC task to unblock it
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void ESTTC_NotifyTaskFromISR(void)
{
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    vTaskNotifyGiveFromISR(ESTTC_UART_TASK_TaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*!
*********************************************************************************************
* @brief Task serving ESTTC protocol through 3 USARTs
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              On HAL error, invokes Error_Handler() method
*********************************************************************************************
*/
static void ESTTC_UART_TASK(void *argument)
{
    uint8_t ProcessedPacket = 0;

    TaskMonitor_TaskInitialized(TASK_MONITOR_ESTTC); /* The task is initialised and is ready */

    for (;;)
    {
        if (ESTTC_UhfPipeMode)
        {
            if (ESTTC_UhfPipeModeTimeout)
            {
                ESTTC_UhfPipeModeTimeout--;
                if (ESTTC_UhfPipeModeTimeout == 0)
                {
                    ESTTC_UhfPipeMode = 0;
                }
            }
        }

        TaskMonitor_IamAlive(TASK_MONITOR_ESTTC); /* Prevent from WatchDog reset */

        ProcessedPacket = ProcessedPacket | ESTTC_ProcessData(ESTTC_COMM_INTEFACE);

        ProcessedPacket = ProcessedPacket | ESTTC_ProcessData(ESTTC_PAYLOAD_INTEFACE);

        ProcessedPacket = ProcessedPacket | ESTTC_ProcessData(ESTTC_SYSCOMM_INTEFACE);

        if (ProcessedPacket)
        {
            ESTTC_Notif_OnEvent(ESTTC_EVENT_PACKET_SERVED_OK, ESTTC_INTERFACE_NUMBER);
            ProcessedPacket = 0;
        }

        if (ESTTC_AvailableCommands())
        {
            osDelay(1);
        }
        else
        {
            (void)ulTaskNotifyTake(pdTRUE, ESTTS_TASK_TIME_CYCLE);
        }

        vApplicationLowStackCheck(TASK_MONITOR_ESTTC);
    }
}

/*!
*********************************************************************************************
* @brief Prints a string on COM serial port
*********************************************************************************************
* @param[input]      FILE * ComInterface - output COM serial interface, char * print_buff - null-terminated string to print out
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void ESTTC_CMD_CRC_Print_string(FILE *ComInterface, char *print_buff)
{
    uint16_t print_len;

    if (print_buff != NULL)
    {
        print_len = strlen(print_buff); /* get string lengh */

        ESTTC_CMD_CRC_Print_raw_data(ComInterface, print_buff, print_len);
    }
    else
    {
        Error_Handler();
    }
}

/*!
*********************************************************************************************
* @brief Prints arbitrary binary buffer as a hex string on COM serial port
*********************************************************************************************
* @param[input]      FILE * ComInterface - output COM serial interface, char * print_buff - output binary buffer, uint16_t size - output binary buffer size
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void ESTTC_CMD_CRC_Print_raw_data(FILE *ComInterface, char *print_buff, uint16_t size)
{
    uint16_t print_len = size;
    uint32_t CRC_value_calc;

    if (print_buff != NULL)
    {
        CRC_value_calc = crc32(0, (BYTE *)print_buff, print_len);                 /* Calculate the CRC */
        sprintf(&print_buff[print_len], " %08X\r", (unsigned int)CRC_value_calc); /* Attach the calcuated CRC at the end of the string */
        print_len += 10;                                                          /* add to the length the new added 10 symbols, 8 for CRC, 1 interval before it and \r after it */
        (void)fwrite(print_buff, 0, print_len, ComInterface);                     /* print the symbols without any formating */
    }
    else
    {
        Error_Handler();
    }
}

/*!
*********************************************************************************************
* @brief Prints the ESTTC version on the UART/USART COM interface
*********************************************************************************************
* @param[input]      FILE *f - UART/USART COM interface
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void ESTTC_PrintVersion(char *buff)
{
    const VersionInfo_t *pVersionInfo = Version_getInfo();

    uint8_t devNumber;

    devNumber = 0;

    sprintf(buff, cVersionFormatString,
            pVersionInfo->appMode,
            devNumber,
            pVersionInfo->fwMajor,
            pVersionInfo->fwMinor,
            pVersionInfo->buildDate,
            pVersionInfo->buildTime,
            pVersionInfo->buildConfigDesc,
            Version_getDevSerial(),
            "\0");
}

/*!
*********************************************************************************************
* @brief Prints the ESTTC version on the UART/USART COM interface
*********************************************************************************************
* @param[input]      FILE *f - UART/USART COM interface
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void ESTTC_PrintPowerUpLog(FILE *f)
{
    const VersionInfo_t *pVersionInfo = Version_getInfo();

    uint8_t devNumber;

    devNumber = 0;

    fprintf(f, cVersionFormatString,
            pVersionInfo->appMode,
            devNumber,
            pVersionInfo->fwMajor,
            pVersionInfo->fwMinor,
            pVersionInfo->buildDate,
            pVersionInfo->buildTime,
            pVersionInfo->buildConfigDesc,
            Version_getDevSerial(),
            "\r\n\0");

    fprintf(f, "System UUID: %08X-%08X-%08X\r\n",
            (unsigned int)(STM32_UUID[0]), (unsigned int)(STM32_UUID[1]), (unsigned int)(STM32_UUID[2]));

    if (f == SYSCON)
        ESTTC_RxHandler_RST_COUNTS(ESTTC_SYSCOMM_INTEFACE);
}

/*!
*********************************************************************************************
* @brief Returns if the UHF is in PIPE mode
*********************************************************************************************
* @param[input]      FILE *f - UART/USART COM interface
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
uint8_t ESTTC_GetUhfPipeMode(void)
{
    return ESTTC_UhfPipeMode;
}


void ESTTC_ReceiveData(ESTTC_InterfacesEnum interface)
{
    if (interface >= ESTTC_INTERFACE_NUMBER)
        return;

    if (Esttc_usart_interfaces[interface] != INVALID_INTERFACE)
    {
        if (((RxBuffHead[interface] + 1) % UART_BUFFER_SIZE) != RxBuffTail[interface])
        {
            RxBuffer[interface][RxBuffHead[interface]] = Esttc_interface_data[interface];
            RxBuffHead[interface] = (RxBuffHead[interface] + 1) % UART_BUFFER_SIZE;
            RxBuffLen[interface]++;
        }

        HAL_StatusTypeDef retVal = HAL_UART_Receive_IT((UART_HandleTypeDef*)Esttc_usart_interfaces[interface], (uint8_t*)&Esttc_interface_data[interface], 1);
        if (HAL_OK != retVal)
        {
            Error_Handler();
        }
    }
}


/*
*********************************************************************************************
* INTERNAL (STATIC) ROUTINES DEFINITION
*********************************************************************************************
*/
/*!
*********************************************************************************************
* @brief Function processing all ESTTC commands
*********************************************************************************************
* @param[input]      Interface - UART/USART COM interface
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
static uint8_t ESTTC_ProcessData(ESTTC_InterfacesEnum Interface)
{
    uint32_t len;
    uint32_t i, j;
    uint32_t br;
    char s8_tmp;
    FRESULT fr;
    FILE *ComInterface;
    uint8_t ProcessedPacket = 0;
    uint32_t CRC_value_calc;
    uint32_t CRC_value_rx;
    uint16_t cmd_length = 0;

    if (Interface >= ESTTC_INTERFACE_NUMBER)
    {
        Error_Handler();
    }

    if (GetPhrase(rxline[Interface], LINE_BUFFER_SIZE - 1, '\r', Interface))
    {
        char *begin;

        ComInterface = Esttc_usart_interfaces[Interface];

        begin = strstr(rxline[Interface], "ES+");
        if (begin != NULL)
        {
            ProcessedPacket = 1;
            ESTTC_Notif_OnEvent(ESTTC_EVENT_RX_PACKET_ON_INTERFACE, Interface);

            len = strlen(begin);

            if (len <= LINE_BUFFER_SIZE)
            {
                strcpy(txline[Interface], begin);

#if (ENABLE_GNSS_OEM719 == 1)
                int res = strncmp(&txline[Interface][3], "GNSS", 4);
                if (res == 0)
                {
                    OEM719_SendRawCmd(ComInterface, (uint8_t*)&txline[Interface][7], strlen(&txline[Interface][7]));
                    return ProcessedPacket;
                }
#endif

                //read the destination address and the command number with verification
                {
                    uint8_t inputsValid;
                    inputsValid = HexToBin(txline[Interface][4], txline[Interface][4 + 1], (uint8_t *)&txline[Interface][4]);

                    if (inputsValid != pdFALSE)
                    {
                        if ((txline[Interface][3] == 'R') || (txline[Interface][3] == 'W'))
                        {
                            inputsValid = HexToBin(txline[Interface][6], txline[Interface][6 + 1], (uint8_t *)&txline[Interface][5]);
                        }
                    }

                    if (inputsValid == pdFALSE)
                    {
                        sprintf(print_buff[Interface], "Err - Wrong parameters");
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                        return ProcessedPacket;
                    }
                }

                if ((txline[Interface][4] == OBC_I2C_ADDRESS) ||
                    (txline[Interface][4] == EPS_I_I2C_ADDRESS) ||
                    (txline[Interface][4] == ANT_I2C_ADDRESS))
                {
                    if (txline[Interface][4] == OBC_I2C_ADDRESS)
                    {
                        if (txline[Interface][3] == 'R')
                        {
                            cmd_length = ESTTC_GetCmdLength(0, txline[Interface][5]);
                        }
                        else if (txline[Interface][3] == 'W')
                        {
                            cmd_length = ESTTC_GetCmdLength(1, txline[Interface][5]);
                        }
                        else if (txline[Interface][3] == 'D')
                        {
                            cmd_length = len;
                        }
                        else
                        {
                            sprintf(print_buff[Interface], "ERR cmd");
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            return ProcessedPacket;
                        }
                    }
                    else if (txline[Interface][4] == EPS_I_I2C_ADDRESS)
                    {
                        if (txline[Interface][3] == 'R')
                        {
                            cmd_length = ESTTC_GetCmdLength(6, txline[Interface][5]);
                        }
                        else if (txline[Interface][3] == 'W')
                        {
                            cmd_length = ESTTC_GetCmdLength(7, txline[Interface][5]);
                        }
                        else
                        {
                            sprintf(print_buff[Interface], "ERR cmd");
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            return ProcessedPacket;
                        }
                    }
                    else if (txline[Interface][4] == ANT_I2C_ADDRESS)
                    {
                        if (txline[Interface][3] == 'R')
                        {
                            cmd_length = ESTTC_GetCmdLength(8, txline[Interface][5]);
                        }
                        else if (txline[Interface][3] == 'W')
                        {
                            cmd_length = ESTTC_GetCmdLength(9, txline[Interface][5]);
                        }
                        else
                        {
                            sprintf(print_buff[Interface], "ERR cmd");
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            return ProcessedPacket;
                        }
                    }

                    uint32_t actual_cmd_length = len;

                    if (actual_cmd_length != cmd_length + ESTTC_CYMBOLS_IN_CRC &&
                        actual_cmd_length != cmd_length)
                    {
                        sprintf(print_buff[Interface], "ERR - Invalid length of the CMD");
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                        return ProcessedPacket;
                    }
                    else if (actual_cmd_length == cmd_length + ESTTC_CYMBOLS_IN_CRC)
                    {
                        actual_cmd_length -= ESTTC_CYMBOLS_IN_CRC;

                        uint32_t crc_calculated_val = crc32(0, (BYTE *)begin, actual_cmd_length);
                        uint32_t crc_received_val;
                        uint8_t valid_input = ESTTC_ExtractCRC(&begin[cmd_length + 1], &crc_received_val);

                        if ((crc_calculated_val != crc_received_val) || (valid_input == pdFALSE))
                        {
                            sprintf(print_buff[Interface], "ERR - Wrong CRC %04X", (unsigned int)crc_calculated_val);
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            return ProcessedPacket;
                        }
                    }

                    const uint32_t start_byte = 6;

                    for (i = 8, j = start_byte; i < actual_cmd_length; i += 2, j++)
                    {
                        uint8_t inputsValid;

                        inputsValid = HexToBin(txline[Interface][i],
                                               txline[Interface][i + 1],
                                               (uint8_t *)&txline[Interface][j]);

                        if ((txline[Interface][3] == 'R') || (txline[Interface][3] == 'W'))
                        {
                            if (inputsValid == pdFALSE)
                            {
                                sprintf(print_buff[Interface],
                                        "ERR - illegal parameter (%c%c)",
                                        txline[Interface][i],
                                        txline[Interface][i + 1]);
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                return ProcessedPacket;
                            }
                        }
                    }

                    if (txline[Interface][3] == 'W')
                    {
                        const uint16_t expected_data_len = txline[Interface][6];
                        const uint16_t actual_data_len = (j - 1) - start_byte;
                        const int32_t mismatch_len = actual_data_len - expected_data_len;

                        if (mismatch_len != 0)
                        {
                            sprintf(print_buff[Interface], "ERR - Invalid length of the CMD");
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            return ProcessedPacket;
                        }
                    }
                }

                if (txline[Interface][3] == 'R')
                {
                    if (txline[Interface][4] == EPS_I_I2C_ADDRESS)
                    {
                        HAL_StatusTypeDef I2C_retStat;
                        uint8_t temp_reg_buff[2];
                        uint16_t eps_reg;

                        cmd_length = 10;
                        if (len == cmd_length + ESTTC_CYMBOLS_IN_CRC)
                        {
                            CRC_value_calc = crc32(0, (BYTE *)begin, len - ESTTC_CYMBOLS_IN_CRC);

                            uint8_t validInput;
                            validInput = ESTTC_ExtractCRC(&begin[cmd_length + 1], &CRC_value_rx);

                            if ((CRC_value_calc != CRC_value_rx) || (validInput == pdFALSE))
                            {
                                sprintf(print_buff[Interface], "ERR - Wrong CRC %04X", (unsigned int)CRC_value_calc);
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                return ProcessedPacket;
                            }
                        }
                        else if (len != cmd_length)
                        {
                            sprintf(print_buff[Interface], "ERR - length");
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            return ProcessedPacket;
                        }

                        //Validate the requested register number
                        if ((txline[Interface][6] < EPS_MAX_PAR_NUM) && (txline[Interface][6] > 0) && (txline[Interface][5] == 0))
                        {
                            I2C_retStat = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);
                            if (I2C_retStat == HAL_OK)
                            {
                                I2C_retStat = MX_I2C_BlockingMemRead(MX_I2C_BUS_SYSTEM, EPS_I_I2C_ADDRESS, txline[Interface][6], sizeof(uint8_t), temp_reg_buff, sizeof(uint16_t));
                                MX_I2C_Release(MX_I2C_BUS_SYSTEM);
                            }

                            if (HAL_OK == I2C_retStat)
                            {
                                //the register is read successfully
                                eps_reg = (temp_reg_buff[0] << 8) + temp_reg_buff[1];

                                //Print out the register value
                                sprintf(print_buff[Interface], "OK+%04X", eps_reg);
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            }
                            else
                            {
                                //Reading has failed. Possibly the EPS is missing
                                sprintf(print_buff[Interface], "ERR - executing");
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            }
                        }
                        else
                        {
                            // Wrong register number
                            sprintf(print_buff[Interface], "ERR - parameter");
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                        }
                    }
#if (ENABLE_UHF_ANT_SUPPORT == 1)
                    else if (txline[Interface][4] == ANT_I2C_ADDRESS)
                    {
                        HAL_StatusTypeDef AntStat;

                        cmd_length = 8;
                        if (len == cmd_length + ESTTC_CYMBOLS_IN_CRC)
                        {
                            CRC_value_calc = crc32(0, (BYTE *)begin, len - ESTTC_CYMBOLS_IN_CRC);

                            uint8_t validInput;
                            validInput = ESTTC_ExtractCRC(&begin[cmd_length + 1], &CRC_value_rx);

                            if ((CRC_value_calc != CRC_value_rx) || (validInput == pdFALSE))
                            {
                                sprintf(print_buff[Interface], "ERR - Wrong CRC %04X", (unsigned int)CRC_value_calc);
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                return ProcessedPacket;
                            }
                        }
                        else if (len != cmd_length)
                        {
                            sprintf(print_buff[Interface], "ERR - length");
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            return ProcessedPacket;
                        }

                        AntUHFRegs_Struct regsUhfAnt;

                        AntStat = AntUHF_Read_Data(ANTUHFREADTYPE_ACTUAL_VALUE, &regsUhfAnt);

                        if (AntStat != HAL_OK)
                        {
                            //Reading has failed. Possibly the UHF antenna is missing
                            sprintf(print_buff[Interface], "ERR The UHF Antenna not connected");
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                        }
                        else
                        {
                            //the registers are read successfully

                            /* Print the four registers with the status of the antenna */
                            sprintf(print_buff[Interface], "OK+%02X%02X%02X%02X", regsUhfAnt.ant_io[0], regsUhfAnt.ant_io[1], regsUhfAnt.ant_io[2], regsUhfAnt.ant_io[3]);
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                        }
                    }
#endif
                    else if (txline[Interface][4] == OBC_I2C_ADDRESS)
                    {
                        for (int cmdNumb = 0; cmdNumb < (sizeof(CmdHandlerList) / sizeof(CmdHandlerList[0])); cmdNumb++)
                        {
                            if (txline[Interface][5] == CmdHandlerList[cmdNumb].cmd)
                            {
                                if (CmdHandlerList[cmdNumb].RxHandler != NULL)
                                    CmdHandlerList[cmdNumb].RxHandler(Interface);
                            }
                        }
                    }
                    else
                    {
                        sprintf(print_buff[Interface], "ERR addr");
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    }
                }
                else if (txline[Interface][3] == 'W')
                {
                    if (txline[Interface][4] == EPS_I_I2C_ADDRESS)
                    {
                        HAL_StatusTypeDef I2C_retStat;

                        cmd_length = 14;
                        if (len == cmd_length + ESTTC_CYMBOLS_IN_CRC)
                        {
                            CRC_value_calc = crc32(0, (BYTE *)begin, len - ESTTC_CYMBOLS_IN_CRC);

                            uint8_t validInput;
                            validInput = ESTTC_ExtractCRC(&begin[cmd_length + 1], &CRC_value_rx);

                            if ((CRC_value_calc != CRC_value_rx) || (validInput == pdFALSE))
                            {
                                sprintf(print_buff[Interface], "ERR - Wrong CRC %04X", (unsigned int)CRC_value_calc);
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                return ProcessedPacket;
                            }
                        }
                        else if (len != cmd_length)
                        {
                            sprintf(print_buff[Interface], "ERR - length");
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            return ProcessedPacket;
                        }

                        if (txline[Interface][5] == 0)
                        {
                            I2C_retStat = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);
                            if (I2C_retStat == HAL_OK)
                            {
                                I2C_retStat = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, EPS_I_I2C_ADDRESS, (uint8_t *)&txline[Interface][7], sizeof(uint16_t));
                                MX_I2C_Release(MX_I2C_BUS_SYSTEM);
                            }

                            if (HAL_OK == I2C_retStat)
                            {
                                sprintf(print_buff[Interface], "OK EPS CMD %X, Reg %x", txline[Interface][5], txline[Interface][7]);
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            }
                            else
                            {
                                sprintf(print_buff[Interface], "ERR - executing");
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            }
                        }
                        else
                        {
                            sprintf(print_buff[Interface], "ERR - unknown command");
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                        }
                    }
#if (ENABLE_UHF_ANT_SUPPORT == 1)
                    else if (txline[Interface][4] == ANT_I2C_ADDRESS)
                    {
                        HAL_StatusTypeDef I2C_retStat;

                        cmd_length = 12;
                        if (len == cmd_length + ESTTC_CYMBOLS_IN_CRC)
                        {
                            CRC_value_calc = crc32(0, (BYTE *)begin, len - ESTTC_CYMBOLS_IN_CRC);

                            uint8_t validInput;
                            validInput = ESTTC_ExtractCRC(&begin[cmd_length + 1], &CRC_value_rx);

                            if ((CRC_value_calc != CRC_value_rx) || (validInput == pdFALSE))
                            {
                                sprintf(print_buff[Interface], "ERR - Wrong CRC %04X", (unsigned int)CRC_value_calc);
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                return ProcessedPacket;
                            }
                        }
                        else if (len != cmd_length)
                        {
                            sprintf(print_buff[Interface], "ERR - length");
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            return ProcessedPacket;
                        }

                        I2C_retStat = AntUHF_Write_Data(txline[Interface][7]);

                        if (HAL_OK == I2C_retStat)
                        {
                            sprintf(print_buff[Interface], "OK ANT CMD %X, Val 0x%x", txline[Interface][5], txline[Interface][7]);
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                        }
                        else
                        {
                            sprintf(print_buff[Interface], "ERR - executing");
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                        }
                    }
#endif
                    else if (txline[Interface][4] == OBC_I2C_ADDRESS)
                    {
                        if (txline[Interface][5] <= ESTTC_CMD_PAR_NUM)
                        {
                            for (int cmdNumb = 0; cmdNumb < (sizeof(CmdHandlerList) / sizeof(CmdHandlerList[0])); cmdNumb++)
                            {
                                if (txline[Interface][5] == CmdHandlerList[cmdNumb].cmd)
                                {
                                    if (CmdHandlerList[cmdNumb].TxHandler != NULL)
                                        CmdHandlerList[cmdNumb].TxHandler(Interface);
                                }
                            }
                        } /* if (txline[Interface][5] <= ESTTC_CMD_PAR_NUM) */
                        else
                        {
                            sprintf(print_buff[Interface], "ERR - unknown command");
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                        }
                    } /* if (txline[Interface][4] == OBC_I2C_ADDRESS) */
                    else if (txline[Interface][4] == UHF_I2C_ADDRESS)
                    {
                        uint8_t TxSize = txline[Interface][5];

                        if ((TxSize + 2) == (len - 8))
                        {
                            HAL_StatusTypeDef I2C_retStat;
                            uint8_t RxSize;
                            uint8_t DevAddr;

                            if (pdFALSE == HexToBin(txline[Interface][8],  txline[Interface][9],  &RxSize) ||
                                pdFALSE == HexToBin(txline[Interface][14], txline[Interface][15], &DevAddr))
                            {
                                sprintf(print_buff[Interface], "ERR - Not valid parameters!");
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            }
                            else
                            {
                                CRC_value_calc = crc32(0, (BYTE *)&rxline[Interface][10], TxSize);                 /* Calculate the CRC */
                                sprintf(&rxline[Interface][10 + TxSize], " %08X\r", (unsigned int)CRC_value_calc); /* Attach the calculated CRC at the end of the string */
                                TxSize += ESTTC_CYMBOLS_IN_CRC + 1;

                                I2C_retStat = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);
                                if (I2C_retStat == HAL_OK)
                                {
                                    I2C_retStat = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, DevAddr, (uint8_t *)&rxline[Interface][10], TxSize);
                                    if (I2C_retStat == HAL_OK)
                                    {
                                        sprintf(print_buff[Interface], "OK UHF CMD %X", txline[Interface][10]);
                                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                                        //Just in case the read length is less then the full string add new line and zero termination
                                        rxline[Interface][RxSize] = '\r';  // add new line
                                        rxline[Interface][RxSize + 1] = 0; // add zero termination

                                        I2C_retStat = MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, DevAddr, (uint8_t *)&rxline[Interface][0], RxSize);

                                        if (rxline[Interface][RxSize - 1] == '\r')
                                        {
                                            rxline[Interface][RxSize] = 0; // add zero termination
                                        }

                                        if (HAL_OK == I2C_retStat)
                                        {
                                            fprintf(ComInterface, "UHF: %s", &rxline[Interface][0]);
                                        }
                                        else
                                        {
                                            sprintf(print_buff[Interface], "ERR - Rx");
                                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                        }
                                    }
                                    MX_I2C_Release(MX_I2C_BUS_SYSTEM);
                                }

                                if (I2C_retStat != HAL_OK)
                                {
                                    EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_UHF_COMM, eEXEH_UHF_EXCEPTION_ID_COMM_I2C_ERROR, __LINE__);
                                }
                            }
                        }
                        else
                        {
                            sprintf(print_buff[Interface], "ERR - length");
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                        }
                    }
                    else
                    {
                        sprintf(print_buff[Interface], "ERR addr");
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    }
                }
                else if (txline[Interface][3] == 'D')
                {
                    if ((txline[Interface][4] == OBC_I2C_ADDRESS) && (begin[6] == 'F') /* All OBC file commands */
                        && (begin[7] != 'W'))                                          /* the write command will receive more bytes so the CRC will be checked after that */
                    {
                        /* get CRC */
                        int ch = -1;
                        char numbCR = 0;

                        for (i = 0; i < 9; i++)
                        {
                            if ((ch = getbyte(20, Interface)) == -1)
                            {
                                break;
                            }

                            begin[len + 1 + i] = (BYTE)ch; /* Place the expected CRC after the the Carriage Return (0x0D) */

                            if (ch == 0x0D)
                            {
                                numbCR++; /* Count the number of the Carriage return */
                            }
                        }

                        if (i == 9)
                        {
                            CRC_value_calc = crc32(0, (BYTE *)begin, len);

                            uint8_t validInput;
                            validInput = ESTTC_ExtractCRC(&begin[len + 2], &CRC_value_rx);

                            if ((CRC_value_calc != CRC_value_rx) || (validInput == pdFALSE))
                            {
                                sprintf(print_buff[Interface], "ERR - Wrong CRC %04X", (unsigned int)CRC_value_calc);
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                return ProcessedPacket;
                            }
                        }
                        else
                        {
                            i -= numbCR;

                            if (i != 0) /* there should be none data but Carriage return symbols */
                            {
                                sprintf(print_buff[Interface], "ERR - length");
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                                return ProcessedPacket;
                            }
                        }
                    }

                    if (txline[Interface][4] == OBC_I2C_ADDRESS)
                    {
                        if (begin[6] == 'F')
                        {
                            switch (begin[7])
                            {
                            case 'B':
                            case 'A':
                            {
#ifndef BOOTLOADER
                                sprintf(print_buff[Interface], "ERR+INAPP");
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
#else
                                uint8_t startPosition;
                                uint32_t file_CRC = 0;
                                uint32_t CRC_data;

                                if (BootLdr_IsFlashingStarted() == pdTRUE)
                                {
                                    sprintf(print_buff[Interface], "ERR: FW update already started");
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    return ProcessedPacket;
                                }
                                else if (BootLdr_IsSleepStarted() == pdTRUE)
                                {
                                    sprintf(print_buff[Interface], "ERR: Sleeping started");
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    return ProcessedPacket;
                                }
                                else
                                {
                                    BootLdr_SetFlashingFlag(pdTRUE);
                                }

                                if (begin[7] == 'B')
                                {
                                    startPosition = BOOTLDR_BIN_CRC_SIZE;
                                }
                                else
                                {
                                    startPosition = 0;
                                }

                                if (df[Interface].obj.fs)
                                    SdMngr_f_close(&df[Interface]);

                                /* extract the path to the file from the command */
                                sprintf(txline[Interface], "0:/%s", &begin[8]);

                                /* open the file pointed in the command */
                                if (FR_OK != (fr = SdMngr_f_open(&df[Interface], txline[Interface], FA_READ | FA_OPEN_EXISTING)))
                                {
                                    sprintf(print_buff[Interface], "ERR+FNF(%u)=%s", fr, txline[Interface]);
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    return ProcessedPacket;
                                }
                                else
                                {
                                    FSIZE_t fileSize = SdMngr_f_size(&df[Interface]);
                                    if (fileSize < 0xFFFF)
                                    {
                                        sprintf(print_buff[Interface], "OK+%08X B", (uint16_t)(fileSize));
                                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    }
                                    else if (fileSize < 0x3FFFFF)
                                    {
                                        sprintf(print_buff[Interface], "OK+%08X kB", (uint16_t)(fileSize >> 10));
                                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    }
                                    else
                                    {
                                        sprintf(print_buff[Interface], "OK+%08X MB", (uint16_t)(fileSize >> 20));
                                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    }
                                }
                                /* =================================================================================================== */
                                /*                                        File CRC Verification                                        */
                                /* =================================================================================================== */

                                /* Report to task monitor the that this task is still alive and is going */
                                TaskMonitor_IamAlive(TASK_MONITOR_ESTTC); /* Prevent from WatchDog reset */

                                sprintf(print_buff[Interface], "Calculating checksum ... ");
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                                uint32_t codeSize;

                                if (begin[7] == 'B')
                                {
                                    BOOTLDR_FwOpRes_enum fileCrcStatus = BootLdr_VerifyCrcAndSizeOfBinFile((TCHAR *)&txline[Interface], &file_CRC, &codeSize);

                                    if (fileCrcStatus == BOOTLDR_FW_OP_RESUS_OK)
                                    {
                                        sprintf(print_buff[Interface], "OK+Valid File CRC");
                                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    }
                                    else
                                    {
                                        if (fileCrcStatus == BOOTLDR_FW_OP_ERR_CRC)
                                        {
                                            sprintf(print_buff[Interface], "ERR+CRC %X", fileCrcStatus);
                                        }
                                        else if (fileCrcStatus == BOOTLDR_FW_OP_ERR_SIZE)
                                        {
                                            sprintf(print_buff[Interface], "ERR+Bin file exceeds flash memory: (%u)>%u", (unsigned int)SdMngr_f_size(&df[Interface]), (unsigned int)BANK_SIZE);
                                        }
                                        else // if( fileCrcStatus == BOOTLDR_FW_OP_ERR_READ )
                                        {
                                            sprintf(print_buff[Interface], "ERR+Read Error");
                                        }
                                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                        return ProcessedPacket;
                                    }
                                }
                                else
                                {
                                    BOOTLDR_FwOpRes_enum verifyHexResult = BootLdr_VerifyCrcOfScmFile(&df[Interface], txline[Interface], &file_CRC, &codeSize, TASK_MONITOR_ESTTC);

                                    if (verifyHexResult != BOOTLDR_FW_OP_RESUS_OK)
                                    {
                                        if ((verifyHexResult == BOOTLDR_FW_OP_ERR_FILENAME) || (verifyHexResult == BOOTLDR_FW_OP_ERR_READ) || (verifyHexResult == BOOTLDR_FW_OP_ERR_WRITE))
                                        {
                                            sprintf(print_buff[Interface], "ERR+FNF(%u)", verifyHexResult);
                                        }
                                        else if (verifyHexResult == BOOTLDR_FW_OP_ERR_FILE_FORMAT)
                                        {
                                            sprintf(print_buff[Interface], "ERR+Wrong File Format");
                                        }
                                        else if (verifyHexResult == BOOTLDR_FW_OP_ERR_CRC)
                                        {
                                            sprintf(print_buff[Interface], "ERR+CheckSum");
                                        }
                                        else if (verifyHexResult == BOOTLDR_FW_OP_ERR_COMPARE)
                                        {
                                            sprintf(print_buff[Interface], "Data comparison failed");
                                        }
                                        else if (verifyHexResult == BOOTLDR_FW_OP_ERR_SIZE)
                                        {
                                            sprintf(print_buff[Interface], "ERR+Bin file exceeds flash memory: (%u)>%u", (unsigned int)SdMngr_f_size(&df[Interface]), (unsigned int)BANK_SIZE);
                                        }

                                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                        return ProcessedPacket;
                                    }
                                }

                                /* =================================================================================================== */
                                /*                                              Erasing                                                */
                                /* =================================================================================================== */

                                /* Report to task monitor the that this task is still alive and is going */
                                TaskMonitor_IamAlive(TASK_MONITOR_ESTTC); /* Prevent from WatchDog reset */

                                sprintf(print_buff[Interface], "Erasing FLASH...\n");
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                                uint8_t errasedSectors;
                                HAL_StatusTypeDef erraseStatus = BootLdr_EraseFlash(codeSize, &errasedSectors);

                                if (erraseStatus == HAL_OK)
                                {
                                    sprintf(print_buff[Interface], "Sectors erased: %u", errasedSectors);
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                }
                                else
                                {
                                    SdMngr_f_close(&df[Interface]);
                                    sprintf(print_buff[Interface], "ERR+FBSE%u@%u", (uint16_t)erraseStatus, errasedSectors);
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    return ProcessedPacket;
                                }

                                /* Report to task monitor the that this task is still alive and is going */
                                TaskMonitor_IamAlive(TASK_MONITOR_ESTTC); /* Prevent from WatchDog reset */
                                osDelay(1);

                                /* =================================================================================================== */
                                /*                                             Blank check                                             */
                                /* =================================================================================================== */

                                /* check if the memory has been erased successful */
                                sprintf(print_buff[Interface], "BLANK CHECK...");
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                                uint32_t checkedSectorsStatus = BootLdr_FlashEmptyPatternCheck(FLASH_MIN_ADDR, codeSize);
                                if (0 != checkedSectorsStatus)
                                {
                                    SdMngr_f_close(&df[Interface]);

                                    sprintf(print_buff[Interface], "ERR+FB%08X addr=%08X size=%08X", (unsigned int)checkedSectorsStatus, (uint16_t)FLASH_MIN_ADDR, (uint16_t)SdMngr_f_size(&df[Interface]));
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                                    return ProcessedPacket;
                                }
                                sprintf(print_buff[Interface], "BLANK");
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                                /* =================================================================================================== */
                                /*                                             Flashing                                                */
                                /* =================================================================================================== */
                                sprintf(print_buff[Interface], "Writing flash memory...");
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                                /* Report to task monitor the that this task is still alive and is going */
                                TaskMonitor_IamAlive(TASK_MONITOR_ESTTC); /* Prevent from WatchDog reset */

                                uint8_t writeStatus;

                                if ((begin[7] == 'A'))
                                {
                                    writeStatus = BootLdr_FlashingFromFile(NULL, startPosition, file_CRC);
                                }
                                else
                                {
                                    writeStatus = BootLdr_FlashingFromFile((TCHAR *)&txline[Interface], startPosition, file_CRC);
                                }

                                if (writeStatus != 0)
                                {
                                    if (writeStatus == BOOTLDR_FW_OP_ERR_WRITE_INCOMPLETE)
                                    {
                                        sprintf(print_buff[Interface], "ERR+Writing not completed!");
                                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    }
                                    else //if((writeStatus == BOOTLDR_FW_OP_ERR_READ)||(writeStatus == BOOTLDR_FW_OP_ERR_WRITE))
                                    {
                                        sprintf(print_buff[Interface], "ERR+FIR(%u)", writeStatus);
                                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    }

                                    return ProcessedPacket;
                                }
                                /* =================================================================================================== */
                                /*                                          Verification                                               */
                                /* =================================================================================================== */

                                /* Report to task monitor the that this task is still alive and is going */
                                TaskMonitor_IamAlive(TASK_MONITOR_ESTTC); /* Prevent from WatchDog reset */

                                /* calculate the CRC of the just written flash memory and the provided CRC at the end of the file */

                                CRC_data = crc32(0, (uint8_t *)FLASH_MIN_ADDR, codeSize);

                                if (CRC_data == file_CRC)
                                {
                                    sprintf(print_buff[Interface], "OK+Valid Flash CRC");
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                                    sprintf(print_buff[Interface], "OK+Flashing completed successful");
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                }
                                else
                                {
                                    sprintf(print_buff[Interface], "ERR+CRC Calculated %X", (unsigned int)CRC_data);
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    return ProcessedPacket;
                                }

                                /* Report to task monitor the that this task is still alive and is going */
                                TaskMonitor_IamAlive(TASK_MONITOR_ESTTC); /* Prevent from WatchDog reset */
                                osDelay(1);

                                /* =================================================================================================== */
                                /*                                       Reboot and jump to App                                        */
                                /* =================================================================================================== */

                                sprintf(print_buff[Interface], "Rebooting into Application ...");
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                                BootLdr_ImmediatAppJump();

                                BootLdr_SetFlashingFlag(pdFALSE);
#endif
                            }
                            break;

                            case 'F': /* Format the SD Cards */
                            {
                                if (0 == memcmp("SDCard", &begin[8], 6))
                                {
                                    fr = SdMngr_RequestSdCardFormat(SD_MNGR_CARD_0, pdTRUE);

                                    if (FR_OK == fr)
                                    {
                                        sprintf(print_buff[Interface], "OK+Format OK. SD card is empty.");
                                    }
                                    else
                                    {
                                        sprintf(print_buff[Interface], "ERR+Format failed(%u)", fr);
                                    }
                                }
                                else
                                {
                                    sprintf(print_buff[Interface], "ERR+Wrong PSW");
                                }
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            }
                            break;

                            case 'C': /* Close a file */
                                if (df[Interface].obj.fs)
                                {
                                    fr = SdMngr_f_close(&df[Interface]);
                                    if (FR_OK == fr)
                                    {
                                        sprintf(print_buff[Interface], "OK Closed File");
                                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    }
                                    else
                                    {
                                        sprintf(print_buff[Interface], "ERR+FNF(%u)", fr);
                                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    }
                                }
                                else
                                {
                                    sprintf(print_buff[Interface], "OK Nothing to Close");
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                }
                                ESTTC_sync_file_timeout[Interface] = 0; /* sync of the file is done here, no need to do it afterwards at timeout */
                                break;

                            case 'S': // Calculate file checksum
                                if (df[Interface].obj.fs)
                                    SdMngr_f_close(&df[Interface]);
                                sprintf(txline[Interface], "0:/%s", &begin[8]);
                                if (FR_OK != (fr = SdMngr_f_open(&df[Interface], txline[Interface], FA_READ | FA_OPEN_EXISTING)))
                                {
                                    sprintf(print_buff[Interface], "ERR+FNF(%u)=%s", fr, txline[Interface]);
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    break;
                                }

                                FSIZE_t fileSize = SdMngr_f_size(&df[Interface]);

                                for (j = 0, i = 0; i < fileSize; i++)
                                {
                                    if (FR_OK != (fr = SdMngr_f_read(&df[Interface], &s8_tmp, 1, (UINT *)&br)))
                                    {
                                        SdMngr_f_close(&df[Interface]);
                                        sprintf(print_buff[Interface], "ERR+FIR(%u)=%u", (uint16_t)fr, (uint16_t)br);
                                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                        break;
                                    }
                                    if (1 != br)
                                    {
                                        SdMngr_f_close(&df[Interface]);
                                        sprintf(print_buff[Interface], "ERR+FRS=%u", (uint16_t)br);
                                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                        break;
                                    }
                                    j += s8_tmp;
                                }

                                sprintf(&print_buff[Interface][0], "OK+%04X", (uint16_t)(j >> 16));
                                sprintf(&print_buff[Interface][3 + 4], "%04X", (uint16_t)j);
                                sprintf(&print_buff[Interface][3 + 4 + 4], " %04X", (uint16_t)(fileSize >> 16));
                                sprintf(&print_buff[Interface][3 + 4 + 4 + 5], "%04X", (uint16_t)fileSize);

                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                                SdMngr_f_close(&df[Interface]);
                                break;

                            case 'E': // Open existing file for writing   //already in use by the "UHF and OBC configurator"
                            case 'I': // Find a existing file for read or write
                                if (df[Interface].obj.fs)
                                    SdMngr_f_close(&df[Interface]);
                                sprintf(txline[Interface], "0:/%s", &begin[8]);
                                if (FR_OK == (fr = SdMngr_f_open(&df[Interface], txline[Interface], FA_READ | FA_WRITE | FA_OPEN_EXISTING)))
                                {
                                    FSIZE_t fileSize = SdMngr_f_size(&df[Interface]);
                                    sprintf(&print_buff[Interface][0], "OK+%04X", (uint16_t)(fileSize >> 16));
                                    sprintf(&print_buff[Interface][7], "%04X Found+Opened", (uint16_t)fileSize);
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                }
                                else
                                {
                                    sprintf(print_buff[Interface], "ERR+FNF(%u)=%s", fr, txline[Interface]);
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                }
                                break;

                            case 'R': // Read from file @ position
                                if (df[Interface].obj.fs == 0)
                                {
                                    sprintf(print_buff[Interface], "ERR+FIH");
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    break;
                                }
                                sscanf(&begin[8], "%08X", (unsigned int *)&fpos[Interface]);
                                sscanf(&begin[8 + 8], "%04X", (unsigned int *)&bsize[Interface]);
                                if ((fpos[Interface] > SdMngr_f_size(&df[Interface])) || (bsize[Interface] > sizeof(txline[Interface])))
                                {
                                    sprintf(&print_buff[Interface][0], "ERR+FIP=");
                                    sprintf(&print_buff[Interface][8], "%04X", (uint16_t)(fpos[Interface] >> 16));
                                    sprintf(&print_buff[Interface][8 + 4], "%04X", (uint16_t)fpos[Interface]);

                                    sprintf(&print_buff[Interface][8 + 4 + 4], "-%04X", (uint16_t)(fno[Interface].fsize >> 16));
                                    sprintf(&print_buff[Interface][8 + 4 + 4 + 5], "%04X,", (uint16_t)fno[Interface].fsize);

                                    sprintf(&print_buff[Interface][8 + 4 + 4 + 5 + 5], "%04X", (uint16_t)(bsize[Interface] >> 16));
                                    sprintf(&print_buff[Interface][8 + 4 + 4 + 5 + 5 + 4], "%04X", (uint16_t)bsize[Interface]);

                                    sprintf(&print_buff[Interface][8 + 4 + 4 + 5 + 5 + 4 + 4], "-%04X", (uint16_t)(sizeof(txline[Interface]) >> 16));
                                    sprintf(&print_buff[Interface][8 + 4 + 4 + 5 + 5 + 4 + 4 + 5], "%04X", (uint16_t)sizeof(txline[Interface]));

                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                                    break;
                                }
                                if (FR_OK != (fr = SdMngr_f_lseek(&df[Interface], fpos[Interface])))
                                {
                                    sprintf(print_buff[Interface], "ERR+FIS(%u)=%u", (uint16_t)fr, (uint16_t)fpos[Interface]);
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    break;
                                }
                                if (FR_OK != (fr = SdMngr_f_read(&df[Interface], txline[Interface], bsize[Interface], (UINT *)&br)))
                                {
                                    sprintf(print_buff[Interface], "ERR+FIR(%u)=%u", fr, (uint16_t)br);
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    break;
                                }
                                if (bsize[Interface] != br)
                                {
                                    sprintf(print_buff[Interface], "ERR+FRS(%u)=%u", (uint16_t)bsize[Interface], (uint16_t)br);
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    break;
                                }
                                br = 0;
                                for (i = 0; i < bsize[Interface]; i++)
                                {
                                    br += txline[Interface][i];
                                    sprintf(&print_buff[Interface][i], "%c", txline[Interface][i]);
                                }

                                sprintf(&print_buff[Interface][bsize[Interface]], "%c", (BYTE)br);

                                ESTTC_CMD_CRC_Print_raw_data(ComInterface, print_buff[Interface], bsize[Interface] + 1);

                                break;

                            case 'O': // Create file for writing
                                if (df[Interface].obj.fs)
                                    SdMngr_f_close(&df[Interface]);
                                sprintf(txline[Interface], "0:/%s", &begin[8]);
                                if (FR_OK == (fr = SdMngr_f_open(&df[Interface], txline[Interface], FA_WRITE | FA_CREATE_ALWAYS)))
                                {
                                    sprintf(print_buff[Interface], "OK+Created+Opened %s", txline[Interface]);
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                }
                                else
                                {
                                    sprintf(print_buff[Interface], "ERR+FNC(%u)=%s", fr, txline[Interface]);
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                }

                                ESTTC_sync_file_numbPackets[Interface] = 0; /* Start over to count number of written packets in one file */
                                break;

                            case 'W': // Write to file @ position
                            {
                                if (df[Interface].obj.fs == NULL)
                                {
                                    sprintf(print_buff[Interface], "ERR+FIH");
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    break;
                                }
                                sscanf(&begin[8], "%08X", (unsigned int *)&fpos[Interface]);
                                sscanf(&begin[8 + 8], "%04X", (unsigned int *)&bsize[Interface]);
                                if ((fpos[Interface] > SdMngr_f_size(&df[Interface])) || (bsize[Interface] > sizeof(txline[Interface])))
                                {
                                    sprintf(&print_buff[Interface][0], "ERR+FIP=");

                                    sprintf(&print_buff[Interface][8], "%04X", (uint16_t)(fpos[Interface] >> 16));
                                    sprintf(&print_buff[Interface][8 + 4], "%04X", (uint16_t)fpos[Interface]);

                                    sprintf(&print_buff[Interface][8 + 4 + 4], "-%04X", (uint16_t)(fno[Interface].fsize >> 16));
                                    sprintf(&print_buff[Interface][8 + 4 + 4 + 5], "%04X,", (uint16_t)fno[Interface].fsize);

                                    sprintf(&print_buff[Interface][8 + 4 + 4 + 5 + 5], "%s", &begin[8]);
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    break;
                                }
                                if (FR_OK != (fr = SdMngr_f_lseek(&df[Interface], fpos[Interface])))
                                {
                                    sprintf(print_buff[Interface], "ERR+FIS(%u)=%u", (uint16_t)fr, (uint16_t)fpos[Interface]);
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    break;
                                }

                                begin[len] = '\r';                                 /* restore the '\r' symbol */
                                CRC_value_calc = crc32(0, (BYTE *)begin, len + 1); /* calculate the CRC over the packet without the data that are going to be written */

                                int ch = -1;
                                br = 0;
                                for (i = 0; i < bsize[Interface]; i++)
                                {
                                    if ((ch = getbyte(1200, Interface)) == -1)
                                        break; /* receive the data that are going to be written */
                                    rxline[Interface][i] = (BYTE)ch;
                                    br += (BYTE)ch;
                                }

                                if (ch == -1)
                                {
                                    sprintf(print_buff[Interface], "ERR+FTM");
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    break;
                                }

                                if ((ch = getbyte(40, Interface)) == -1) /* receive the checksum */
                                {
                                    sprintf(print_buff[Interface], "ERR+FTM");
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    break;
                                }
                                rxline[Interface][i] = ch; /* add the checksum to the buffer */

                                if (((BYTE)br) != ((BYTE)ch)) /* compare the calculated checksum and the received checksum */
                                {
                                    sprintf(print_buff[Interface], "ERR+FEC=%02X(%02X)", (BYTE)br, (BYTE)ch);
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    break;
                                }

                                char numbCR = 0;

                                for (i = 0; i < 9; i++) /* read space + 8 symbols for CRC value */
                                {
                                    if ((ch = getbyte(80, Interface)) == -1)
                                    { /* read a byte from the USART buffer */
                                        break;
                                    }

                                    txline[Interface][i] = (BYTE)ch; /* store the byte in the buffer */

                                    if (ch == 0x0D)
                                    {
                                        numbCR++; /* Count the number of the Carriage return */
                                    }
                                }

                                if (i == 9)
                                {
                                    CRC_value_calc = crc32((DWORD)CRC_value_calc, (BYTE *)&rxline[Interface][0], bsize[Interface] + 1); /* calculate the CRC over the rest of the packet */

                                    uint8_t validInput;
                                    validInput = ESTTC_ExtractCRC(&txline[Interface][1], &CRC_value_rx);

                                    if ((CRC_value_calc != CRC_value_rx) || (validInput == pdFALSE)) /* compare the calculated value over the packet with the receive value at the end of the packet */
                                    {
                                        sprintf(print_buff[Interface], "ERR - Wrong CRC %04X", (unsigned int)CRC_value_calc);
                                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                        return ProcessedPacket;
                                    }
                                }
                                else
                                {
                                    i -= numbCR;

                                    if (i != 0)
                                    {
                                        sprintf(print_buff[Interface], "ERR - length");
                                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

                                        return ProcessedPacket;
                                    }
                                }

                                if (FR_OK != (fr = SdMngr_f_write(&df[Interface], rxline[Interface], bsize[Interface], (UINT *)&br)))
                                {
                                    sprintf(print_buff[Interface], "ERR+FWE=%u", fr);
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    break;
                                }
                                if (bsize[Interface] != br)
                                {
                                    sprintf(print_buff[Interface], "ERR+FWC=%u(%u)", (uint16_t)br, (uint16_t)bsize[Interface]);
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    break;
                                }

                                ESTTC_sync_file_timeout[Interface] = ESTTS_MAX_PIPE_TIMEOUT; /* timeout to sync the file if there is no writing request for long time */
                                ESTTC_sync_file_numbPackets[Interface]++;                    /* count number of written packets */

                                sprintf(print_buff[Interface], "OK Written");
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            }
                            break;

                            case 'D': // Delete file
                                if (df[Interface].obj.fs)
                                    SdMngr_f_close(&df[Interface]);
                                sprintf(txline[Interface], "0:/%s", &begin[8]);
                                if (FR_OK != SdMngr_f_unlink(txline[Interface]))
                                {
                                    sprintf(print_buff[Interface], "ERR+FDL%s", txline[Interface]);
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                }
                                else
                                {
                                    sprintf(print_buff[Interface], "OK Deleted %s", txline[Interface]);
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                }
                                break;

                            case 'L': // Write to file "DirList" a list of existing files
                                if (df[Interface].obj.fs)
                                    SdMngr_f_close(&df[Interface]);
                                if (FR_OK != (fr = SdMngr_f_open(&df[Interface], "0:/DirList.txt", FA_WRITE | FA_CREATE_ALWAYS)))
                                {
                                    sprintf(print_buff[Interface], "ERR+LCF(%u)", fr);
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    break;
                                }
                                len = strlen(&begin[8]);
                                if ((len == 0) || (len > 12))
                                {
                                    begin[8] = '*';
                                    begin[9] = 0;
                                }

                                SdMngr_f_printf(&df[Interface], "----- FATFS RevID.%05u -----\r\n", _FATFS);
                                SdMngr_f_printf(&df[Interface], "--- Name ---    --- size ---\r\n");
                                strcpy(txline[Interface], &begin[8]);
                                j = 0;
                                if (FR_OK != (fr = SdMngr_f_findfirst(&dd[Interface], &fno[Interface], "0:", txline[Interface])))
                                {
                                    sprintf(print_buff[Interface], "ERR+LFF(%u)", fr);
                                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                    break;
                                }
                                do
                                {
                                    if (strlen(fno[Interface].fname) == 0)
                                        break;
                                    j++;

                                    if (fno[Interface].fsize < 0xFFFF)
                                    {
                                        sprintf(rxline[Interface], "%13s   %u B\n", fno[Interface].fname, (uint16_t)fno[Interface].fsize);
                                    }
                                    else if (fno[Interface].fsize < 0x3FFFFF)
                                    {
                                        sprintf(rxline[Interface], "%13s   ~%u kB\n", fno[Interface].fname, (uint16_t)(fno[Interface].fsize >> 10));
                                    }
                                    else
                                    {
                                        sprintf(rxline[Interface], "%13s   ~%u MB\n", fno[Interface].fname, (uint16_t)(fno[Interface].fsize >> 20));
                                    }

                                    SdMngr_f_puts(rxline[Interface], &df[Interface]);
                                } while (FR_OK == SdMngr_f_findnext(&dd[Interface], &fno[Interface]));

                                SdMngr_f_printf(&df[Interface], "-----------------------------\n         TOTAL FILES %u\n", (uint16_t)j);
                                SdMngr_f_close(&df[Interface]);
                                sprintf(print_buff[Interface], "OK DirList %s", txline[Interface]);
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                break;

                            default: // Unknown command
                                sprintf(print_buff[Interface], "ERR+UNC:%s", &begin[7]);
                                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                                break;
                            }
                        }
                        else
                        {
                            sprintf(print_buff[Interface], "ERR Wrong Data type");
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                        }
                    }
                    else
                    {
                        sprintf(print_buff[Interface], "ERR Address");
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    }
                }
                else
                {
                    sprintf(print_buff[Interface], "ERR cmd");
                    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                }
            }
            else
            {
                sprintf(print_buff[Interface], "ERR - Wrong Length!");
                ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
            }
        }
        else
        {
            int ch = -1;

            UhfMasterObcSlaveCmds_str *uhfCmdStrings_temp = NULL;

            for (int k = 0; k < 3; k++)
            {
                begin = strstr(rxline[Interface], &uhfCmdStrings[k].uhfCmdStrings[0]);
                if (begin != NULL)
                {
                    uhfCmdStrings_temp = &uhfCmdStrings[k];
                    break;
                }
            }

            if (uhfCmdStrings_temp != NULL)
            {
                len = strlen(begin);

                if ((len == uhfCmdStrings_temp->PacketLength) || (len == (uhfCmdStrings_temp->PacketLength + ESTTC_CYMBOLS_IN_CRC)))
                {
                    if (uhfCmdStrings_temp->CmdNumber == 0)
                    {
                        /* get SCW */
                        for (i = 0; i < 7 + ESTTC_CYMBOLS_IN_CRC + 1; i++)
                        {
                            if ((ch = getbyte(1200, Interface)) == -1)
                                break; /* receive the data that are going to be written */
                            begin[len + i + 1] = (BYTE)ch;
                            if (ch == 0x0D)
                            {
                                break;
                            }
                        }

                        //check the SCW of the UHF
                        uint8_t SCW;
                        uint8_t validInput;
                        validInput = HexToBin(begin[len + 6], begin[len + 7], &SCW);

                        if (validInput == pdFALSE)
                        {
                            sprintf(print_buff[Interface], "ERR - Wrong parameters");
                            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                            return ProcessedPacket;
                        }

                        if (SCW & 1 << 5) //Pipe mode is the sixth LSB bit (bit No 5)
                        {
                            //enable
                            ESTTC_UhfPipeMode = 1;
                            ESTTC_UhfPipeModeTimeout = ESTTS_MAX_PIPE_TIMEOUT;
                        }
                        else
                        {
                            //disable
                            ESTTC_UhfPipeMode = 0;
                        }
                    }
                    else if (uhfCmdStrings_temp->CmdNumber == 1)
                    {
                        sprintf(print_buff[Interface], "OK+PHOENIX");
                        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
                    }
                }
            }
        }
    }
    else
    {
        /* Check if a file has been written but has not been closed for a long time */
        if (ESTTC_sync_file_timeout[Interface])
        {
            ESTTC_sync_file_timeout[Interface]--;

            if ((ESTTC_sync_file_timeout[Interface] == 1) || (ESTTC_sync_file_numbPackets[Interface] > ESTTS_SYNC_FILE_PACKETS))
            {
                ESTTC_sync_file_numbPackets[Interface] = 0;

                if (df[Interface].obj.fs) /* if there is a open file */
                {
                    /* sync the data */
                    fr = SdMngr_f_sync(&df[Interface]);
                }
            }
        }
    }

    return ProcessedPacket;
}

/*!
*********************************************************************************************
* @brief Gets a single character from UART/USART serial buffer
*********************************************************************************************
* @param[input]      Interface - UART/USART COM interface
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
static int ESTTC_getchar(ESTTC_InterfacesEnum Interface)
{
    int ch = -1;

    taskENTER_CRITICAL();

    if (RxBuffTail[Interface] != RxBuffHead[Interface])
    {
        ch = (BYTE)RxBuffer[Interface][RxBuffTail[Interface]];
        RxBuffTail[Interface] = (RxBuffTail[Interface] + 1) % UART_BUFFER_SIZE;
        RxBuffLen[Interface]--;
    }
    taskEXIT_CRITICAL();

    return ch;
}

/*!
*********************************************************************************************
* @brief Check for available data in the buffer
*********************************************************************************************
* @param[input]      Interface - UART/USART COM interface
* @param[output]     Status:
*                       - pdTRUE - There are some more data in the buffers
*                       - pdFALSE - No more data is received
* @return            none
* @note              none
*********************************************************************************************
*/
static uint8_t ESTTC_AvailableCommands(void)
{
    uint8_t retuVal = pdFALSE;

    taskENTER_CRITICAL();
    for (uint8_t i = 0; i < (uint8_t)ESTTC_INTERFACE_NUMBER; i++)
    {
        if (RxBuffTail[i] != RxBuffHead[i])
        {
            retuVal = pdTRUE;
        }
    }
    taskEXIT_CRITICAL();

    return retuVal;
}

/*!
*********************************************************************************************
* @brief Gets a single byte from UART/USART serial buffer in blocking manner
*********************************************************************************************
* @param[input]      Interface - UART/USART COM interface
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
static int getbyte(uint32_t tmt_ms, ESTTC_InterfacesEnum Interface)
{
    int ch;

    if ((ch = ESTTC_getchar(Interface)) != -1)
        return ch;

    for (uint32_t i = 0; i < tmt_ms; i++)
    {
        osDelay(1);
        if ((ch = ESTTC_getchar(Interface)) != -1)
            break;
    }
    return ch;
}

/*!
*********************************************************************************************
* @brief Gets a whole phrase from UART/USART serial buffer in blocking manner
*********************************************************************************************
* @param[input]      Interface - UART/USART COM interface
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
static uint32_t GetPhrase(char *dst, uint32_t len, char term, ESTTC_InterfacesEnum Interface)
{
    int ch;

    if ((dst == NULL) || (len == 0))
        return 0;

    while ((ch = ESTTC_getchar(Interface)) != -1)
    {
        dst[pack_data_position[Interface]] = (BYTE)ch;

        pack_data_position[Interface] = (pack_data_position[Interface] + 1) % LINE_BUFFER_SIZE;

        if (term == (BYTE)ch)
        {
            dst[pack_data_position[Interface] - 1] = 0;
            pack_data_position[Interface] = 0;
            return 1;
        }
    }

    return 0;
}

/*!
*********************************************************************************************
* @brief Hexadecimal ASCII to binary number converted
*********************************************************************************************
* @param[input]      hb - high significant part of the byte
*                    lb - low significant part of the byte
* @param[output]     hexVal - Hexadecimal value from both digits
* @return            validity of the input parameters:
*                       - pdTRUE - valid
*                       - pdFALSE - invalid
* @note              none
*********************************************************************************************
*/
static uint8_t HexToBin(uint8_t hb, uint8_t lb, uint8_t *hexVal)
{
    if (!isxdigit(hb) || !isxdigit(lb) || hexVal == NULL)
        return pdFALSE;

    if (hb > '9')
        hb += 9;

    if (lb > '9')
        lb += 9;

    *hexVal = (hb << 4) + (lb & 0x0f);

    return pdTRUE;
}

/*!
*********************************************************************************************
* @brief Gets CMD interface command length
*********************************************************************************************
* @param[input]      cmd_type - type of the command:
                              0 - Read command
                              1 - Write command
* @param[input]      CMD - Command
* @param[output]     none
* @return            uint32_t - CRC value
* @note              none
*********************************************************************************************
*/
static uint16_t ESTTC_GetCmdLength(uint8_t cmd_type, uint8_t CMD)
{
    uint16_t cmd_length = 0;
    uint8_t u8_index;
    uint8_t cmd_number = 0;
    esttc_cmd_params_type *temp_cmd_params = NULL;

    if (cmd_type == 0) /* if the command is type Reading */
    {
        temp_cmd_params = (esttc_cmd_params_type *)ESTTC_ReadCmdLenght;
        cmd_number = ESTTC_NUMBER_READ_CMDS + 1;
    }
    else if (cmd_type == 1) /* if the command is type Writing */
    {
        temp_cmd_params = (esttc_cmd_params_type *)ESTTC_WriteCmdLenght;
        cmd_number = ESTTC_NUMBER_WRITE_CMDS;
    }
    else if (cmd_type == 6) /* if the command is type Reading to EPS I  */
    {
        return 10;
    }
    else if (cmd_type == 7) /* if the command is type Writing to EPS I  */
    {
        return 14;
    }
    else if (cmd_type == 8) /* if the command is type Reading to UHF Antenna I2C  */
    {
        return 8;
    }
    else if (cmd_type == 9) /* if the command is type Writing to UHF Antenna I2C  */
    {
        return 12;
    }
    else
    {
        return 0;
    }

    for (u8_index = 0; u8_index < cmd_number; u8_index++)
    {
        if (temp_cmd_params[u8_index].cmd == CMD)
        {
            cmd_length = temp_cmd_params[u8_index].length;
            break;
        }
    }

    return cmd_length;
}

/*!
*********************************************************************************************
* @brief Converts ASCII string of a CRC value to a hex value
*********************************************************************************************
* @param[input]      crc_buffer - Array with a ASCII string with CRC
* @param[output]     uint32_t - CRC value
* @return            validity of the input parameters:
*                       - pdTRUE - valid
*                       - pdFALSE - invalid
* @note              none
*********************************************************************************************
*/
static uint8_t ESTTC_ExtractCRC(char *crc_buffer, uint32_t *CalcCRC)
{
    uint8_t i, j;
    uint8_t CRC_value_rx_buf[4];
    uint8_t validInput;

    /* convert the CRC from ASCII to hex values */
    for (i = 0, j = 0; i < 1 + 6; i += 2, j++)
    {
        validInput = HexToBin(crc_buffer[i], crc_buffer[i + 1], &CRC_value_rx_buf[j]);
    }

    /* combine the four bytes to one word */
    *CalcCRC = CRC_value_rx_buf[0] << 24 |
               CRC_value_rx_buf[1] << 16 |
               CRC_value_rx_buf[2] << 8 |
               CRC_value_rx_buf[3];

    return validInput;
}

/* **************************************************************************************** */
