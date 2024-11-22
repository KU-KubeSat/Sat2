/*!
********************************************************************************************
* @file ESTTC.h
* @brief Header of ESTTC.c.
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
#ifndef ESTTC_H
#define ESTTC_H

/*
*********************************************************************************************
* INCLUDES
*********************************************************************************************
*/
#include <stdio.h>
#include "cmsis_os.h"

/*
*********************************************************************************************
* EXTERNAL DEFINES
*********************************************************************************************
*/
#define UART_BUFFER_SIZE 256
#define LINE_BUFFER_SIZE UART_BUFFER_SIZE

/*
*********************************************************************************************
* EXTERNAL TYPES DECLARATIONS
*********************************************************************************************
*/
typedef enum
{
	ESTTC_CMD_MAG1_DATA			      = 0x04,
	ESTTC_CMD_MAG1_ACESS		      = 0x05,
	ESTTC_CMD_MAG2_DATA			      = 0x06,
	ESTTC_CMD_MAG2_ACESS		      = 0x07,

	ESTTC_CMD_GYRO_RADIO_DATA_PANEL_4 = 0x08,
	ESTTC_CMD_GYRO_ANGLE_DATA_PANEL_4 = 0x09,
	ESTTC_CMD_GYRO_REG_PANEL_4        = 0x0A,
	ESTTC_CMD_GYRO_RADIO_DATA_PANEL_5 = 0x0B,
	ESTTC_CMD_GYRO_ANGLE_DATA_PANEL_5 = 0x0C,
	ESTTC_CMD_GYRO_REG_PANEL_5        = 0x0D,
	ESTTC_CMD_GYRO_RADIO_DATA_PANEL_6 = 0x0E,
	ESTTC_CMD_GYRO_ANGLE_DATA_PANEL_6 = 0x0F,
	ESTTC_CMD_GYRO_REG_PANEL_6        = 0x10,
	ESTTC_CMD_MTORQ_POWER_PANEL_4     = 0x11,
	ESTTC_CMD_MTORQ_POWER_PANEL_5	  = 0x12,
	ESTTC_CMD_MTORQ_POWER_PANEL_6     = 0x13,
	ESTTC_CMD_TEMP_PANEL_1	          = 0x14,
	ESTTC_CMD_TEMP_PANEL_2  	      = 0x15,
	ESTTC_CMD_TEMP_PANEL_3  	      = 0x16,
	ESTTC_CMD_TEMP_PANEL_4  	      = 0x17,
	ESTTC_CMD_TEMP_PANEL_5  	      = 0x18,
	ESTTC_CMD_TEMP_PANEL_6  	      = 0x19,
	ESTTC_CMD_PHOTO_PANEL_1		      = 0x1A,
	ESTTC_CMD_PHOTO_PANEL_2		      = 0x1B,
	ESTTC_CMD_PHOTO_PANEL_3		      = 0x1C,
	ESTTC_CMD_PHOTO_PANEL_4		      = 0x1D,
	ESTTC_CMD_PHOTO_PANEL_5		      = 0x1E,
	ESTTC_CMD_PHOTO_PANEL_6		      = 0x1F,

	ESTTC_CMD_OUTPUT_CONTROL          = 0x20,
	ESTTC_CMD_SENSORS_CONTROL         = 0x21,
	ESTTC_CMD_I2C_PULLUPS_CONTROL     = 0x22,

    ESTTC_CMD_GET_TIME                = 0x31,
    ESTTC_CMD_SET_TIME                = 0x32,
    ESTTC_CMD_GET_DATA                = 0x33,
    ESTTC_CMD_SET_DATA                = 0x34,
    ESTTC_CMD_UPTIME                  = 0x35,

    ESTTC_CMD_ANT_SETTINGS            = 0x47,

    ESTTC_CMD_CAPTURE_PAR             = 0x51,

	ESTTC_CMD_RST_COUNTS              = 0x61,
	ESTTC_CMD_FAULTS_TST              = 0x62,
    ESTTC_CMD_POWER_MODE              = 0x71,

	ESTTC_CMD_RESET                   = 0x7F,
	ESTTC_CMD_PAR_NUM                 = 0x80,

	ESTTC_CMD_KUbeSatTest             = 0x63
} ESTTC_CommandEnum;

typedef enum
{
    ESTTC_COMM_INTEFACE = 0,
    ESTTC_PAYLOAD_INTEFACE,
    ESTTC_SYSCOMM_INTEFACE,
    ESTTC_INTERFACE_NUMBER
} ESTTC_InterfacesEnum;

typedef struct
{
    ESTTC_InterfacesEnum cmd;
    uint8_t length;
} esttc_cmd_params_type;


typedef uint8_t (*input_func)(ESTTC_InterfacesEnum Interface);

typedef struct
{
    ESTTC_InterfacesEnum cmd;
    input_func RxHandler;
    input_func TxHandler;
} esttc_cmd_handlers_type;

typedef struct
{
    char uhfCmdStrings[10];
    char PacketLength;
    char CmdNumber;
} UhfMasterObcSlaveCmds_str;

typedef enum
{
    ESTTC_EVENT_PACKET_SERVED_OK,
    ESTTC_EVENT_RX_PACKET_ON_INTERFACE,
    ESTTC_EVENT_COUNT
} ESTTC_EventsEnum;

/*
*********************************************************************************************
* EXTERNAL VARIABLES DECLARATIONS
*********************************************************************************************
*/

/*
*********************************************************************************************
* EXTERNAL ROUTINES DECLARATIONS 
*********************************************************************************************
*/
void ESTTC_InitTask(void);
osThreadId_t ESTTC_GetTaskHandler(void);
void ESTTC_Init(void);
void ESTTC_DeInit(void);
void ESTTC_Notif_OnEvent(const ESTTC_EventsEnum event, const ESTTC_InterfacesEnum Interface);

void ESTTC_CMD_CRC_Print_string(FILE * ComInterface, char * print_buff);
void ESTTC_CMD_CRC_Print_raw_data(FILE * ComInterface, char * print_buff, uint16_t size);
void ESTTC_PrintVersion(char * buff);
void ESTTC_PrintPowerUpLog(FILE *f);
uint8_t ESTTC_GetUhfPipeMode(void);
void ESTTC_ReceiveData(ESTTC_InterfacesEnum interface);
uint8_t RxHandler_Accsel1(void);
uint8_t TxHandlerAccsel1(uint8_t * input);
void ESTTC_NotifyTaskFromISR(void);
void ESTTC_RxHandler_RST_COUNTS(ESTTC_InterfacesEnum Interface);

void Test_Send_Serial();

#endif    /* ESTTC_H */
/* **************************************************************************************** */
