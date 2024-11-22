#ifndef __ESTTC_CMDHANDLERC_IMPL__

/*
 * This file is to be included in ESTTC.c directly. It shall not be compiled alone because it relies on
 * dependencies from ESTTC.c. The only reason for moving this code here, is to provide better readability for
 * the code and to improve maintainability.
 */

#include "cmsis_os2.h"
#include "CommandsApi.h"
#include "EPS_I.h"
#include <math.h>
#include "../Infrastructure/debug/inc/debug.h"


#define GET_COMPASS_PRINT_ID(compassId) ((uint8_t) (((uint8_t) (compassId)) + (uint8_t) 1))
#define GET_PANEL_PRINT_ID(panelId)     ((uint8_t) (((uint8_t) (panelId)) + (uint8_t) 1))

#ifndef ADCS_GYRO_ANG_RATE_LSB
  #define ADCS_GYRO_ANG_RATE_LSB        ((double) 0.07326)     /* ADIS16260/16265 rate in deg/sec per 1 LSB */
#endif


static const char *sensorNames[OBCSENSORID_MAX] =
{
    "Accelerometers",
    "Magnetometers",
    "Gyroscopes",
    "Magnetorquers",
    "Tempr. sensors",
    "Sun sensors"
};


/* list of all commands and their lengths without the CRC */
static const esttc_cmd_params_type ESTTC_ReadCmdLenght[ESTTC_NUMBER_READ_CMDS + 1] =
{
    /* CMD number */                            /* Length */
    {/* 0x04 */ ESTTC_CMD_MAG1_DATA               ,   8  }, // 1
    {/* 0x05 */ ESTTC_CMD_MAG1_ACESS              ,  10  }, // 2
    {/* 0x06 */ ESTTC_CMD_MAG2_DATA               ,   8  }, // 3
    {/* 0x07 */ ESTTC_CMD_MAG2_ACESS              ,  10  }, // 4
    {/* 0x08 */ ESTTC_CMD_GYRO_RADIO_DATA_PANEL_4 ,   8  }, // 5
    {/* 0x09 */ ESTTC_CMD_GYRO_ANGLE_DATA_PANEL_4 ,   8  }, // 6
    {/* 0x0A */ ESTTC_CMD_GYRO_REG_PANEL_4        ,  10  }, // 7
    {/* 0x0B */ ESTTC_CMD_GYRO_RADIO_DATA_PANEL_5 ,   8  }, // 8
    {/* 0x0C */ ESTTC_CMD_GYRO_ANGLE_DATA_PANEL_5 ,   8  }, // 9
    {/* 0x0D */ ESTTC_CMD_GYRO_REG_PANEL_5        ,  10  }, // 10
    {/* 0x0E */ ESTTC_CMD_GYRO_RADIO_DATA_PANEL_6 ,   8  }, // 11
    {/* 0x0F */ ESTTC_CMD_GYRO_ANGLE_DATA_PANEL_6 ,   8  }, // 12
    {/* 0x10 */ ESTTC_CMD_GYRO_REG_PANEL_6        ,  10  }, // 13
    {/* 0x11 */ ESTTC_CMD_MTORQ_POWER_PANEL_4     ,   8  }, // 14
    {/* 0x12 */ ESTTC_CMD_MTORQ_POWER_PANEL_5     ,   8  }, // 15
    {/* 0x13 */ ESTTC_CMD_MTORQ_POWER_PANEL_6     ,   8  }, // 16
    {/* 0x14 */ ESTTC_CMD_TEMP_PANEL_1            ,   8  }, // 17
    {/* 0x15 */ ESTTC_CMD_TEMP_PANEL_2            ,   8  }, // 18
    {/* 0x16 */ ESTTC_CMD_TEMP_PANEL_3            ,   8  }, // 19
    {/* 0x17 */ ESTTC_CMD_TEMP_PANEL_4            ,   8  }, // 20
    {/* 0x18 */ ESTTC_CMD_TEMP_PANEL_5            ,   8  }, // 21
    {/* 0x19 */ ESTTC_CMD_TEMP_PANEL_6            ,   8  }, // 22
    {/* 0x1A */ ESTTC_CMD_PHOTO_PANEL_1           ,   8  }, // 23
    {/* 0x1B */ ESTTC_CMD_PHOTO_PANEL_2           ,   8  }, // 24
    {/* 0x1C */ ESTTC_CMD_PHOTO_PANEL_3           ,   8  }, // 25
    {/* 0x1D */ ESTTC_CMD_PHOTO_PANEL_4           ,   8  }, // 26
    {/* 0x1E */ ESTTC_CMD_PHOTO_PANEL_5           ,   8  }, // 27
    {/* 0x1F */ ESTTC_CMD_PHOTO_PANEL_6           ,   8  }, // 28
    {/* 0x20 */ ESTTC_CMD_OUTPUT_CONTROL          ,   8  }, // 29
    {/* 0x21 */ ESTTC_CMD_SENSORS_CONTROL         ,  10  }, // 30
    {/* 0x22 */ ESTTC_CMD_I2C_PULLUPS_CONTROL     ,   8  }, // 31
    {/* 0x31 */ ESTTC_CMD_GET_TIME                ,   8  }, // 32
    {/* 0x33 */ ESTTC_CMD_GET_DATA                ,   8  }, // 33
    {/* 0x35 */ ESTTC_CMD_UPTIME                  ,   8  }, // 34
    {/* 0x47 */ ESTTC_CMD_ANT_SETTINGS            ,   8  }, // 35
    {/* 0x51 */ ESTTC_CMD_CAPTURE_PAR             ,  12  }, // 36
    {/* 0x61 */ ESTTC_CMD_RST_COUNTS              ,   8  }, // 37
    {/* 0x71 */ ESTTC_CMD_POWER_MODE              ,   8  }, // 38
    {/* 0x7F */ ESTTC_CMD_RESET                   ,   8  }, // 39
    {/* 0x63 */ KUbeSatTest                       ,   8  }  // KUbeSat Test
};

static const esttc_cmd_params_type ESTTC_WriteCmdLenght[ESTTC_NUMBER_WRITE_CMDS] =
{
    {/* 0x05 */  ESTTC_CMD_MAG1_ACESS             ,  14  }, // 1
    {/* 0x07 */  ESTTC_CMD_MAG2_ACESS             ,  14  }, // 2
    {/* 0x0A */  ESTTC_CMD_GYRO_REG_PANEL_4       ,  16  }, // 3
    {/* 0x0D */  ESTTC_CMD_GYRO_REG_PANEL_5       ,  16  }, // 4
    {/* 0x10 */  ESTTC_CMD_GYRO_REG_PANEL_6       ,  16  }, // 5
    {/* 0x11 */  ESTTC_CMD_MTORQ_POWER_PANEL_4    ,  14  }, // 6
    {/* 0x12 */  ESTTC_CMD_MTORQ_POWER_PANEL_5    ,  14  }, // 7
    {/* 0x13 */  ESTTC_CMD_MTORQ_POWER_PANEL_6    ,  14  }, // 8
    {/* 0x20 */  ESTTC_CMD_OUTPUT_CONTROL         ,  14  }, // 9
    {/* 0x21 */  ESTTC_CMD_SENSORS_CONTROL        ,  14  }, // 10
    {/* 0x26 */  ESTTC_CMD_I2C_PULLUPS_CONTROL    ,  14  }, // 11
    {/* 0x32 */  ESTTC_CMD_SET_TIME               ,  22  }, // 12
    {/* 0x34 */  ESTTC_CMD_SET_DATA               ,  22  }, // 13
    {/* 0x35 */  ESTTC_CMD_UPTIME                 ,  26  }, // 14
    {/* 0x47 */  ESTTC_CMD_ANT_SETTINGS           ,  18  }, // 15
    {/* 0x61 */  ESTTC_CMD_RST_COUNTS             ,  12  }, // 16
    {/* 0x62 */  ESTTC_CMD_FAULTS_TST             ,  12  }, // 17
    {/* 0x71 */  ESTTC_CMD_POWER_MODE             ,  18  }, // 18
    {/* 0x7F */  ESTTC_CMD_RESET                  ,  12  }  // 19
};


static void HelperRxHandler_Compass_XYZ(ESTTC_InterfacesEnum Interface, CompassId_t id)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];
    CommApi_Compass_XYZ_Cmd_str RxData = ReadHandler_Compass_XYZ(COMPASSID_LOW);
    uint8_t compassPrintId = GET_COMPASS_PRINT_ID(id);

    if (RxData.status != SEN_SUCCESS)
    {
        sprintf(print_buff[Interface], "ERR - executing");
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
    }
    else
    {
        sprintf(print_buff[Interface],
            "OK+M%d:%5.0f/%5.0f/%5.0f",
            compassPrintId, RxData.data.AXIS_X, RxData.data.AXIS_Y, RxData.data.AXIS_Z);

        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

        sprintf(print_buff[Interface],
            "Magnetometer %d - Magnetic field in specific range X=%1.0f Y=%1.0f Z=%1.0f",
            compassPrintId, RxData.data.AXIS_X, RxData.data.AXIS_Y, RxData.data.AXIS_Z);

        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
    }
}

static void ESTTC_RxHandler_Compass_1_XYZ(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_Compass_XYZ(Interface, COMPASSID_LOW);
}

static void ESTTC_RxHandler_Compass_2_XYZ(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_Compass_XYZ(Interface, COMPASSID_HIGH);
}

static void HelperRxHandler_Compass_Reg(ESTTC_InterfacesEnum Interface, CompassId_t id)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];
    CommApi_Compass_reg_Cmd_str RxData = ReadHandler_Compass_Reg(id, txline[Interface][6]);
    uint8_t compassPrintId = GET_COMPASS_PRINT_ID(id);

    if (RxData.status == SEN_SUCCESS)
    {
        sprintf(print_buff[Interface],
            "OK+M%d:%02X/%02X",
            compassPrintId, txline[Interface][6], RxData.data);

        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

        sprintf(print_buff[Interface],
            "Magnetometer %d reg No %d has value %d",
            compassPrintId, txline[Interface][6], RxData.data);

        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
    }
    else
    {
        sprintf(print_buff[Interface], "ERR - executing");
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
    }
}

static void ESTTC_RxHandler_Compass_1_Reg(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_Compass_Reg(Interface, COMPASSID_LOW);
}

static void ESTTC_RxHandler_Compass_2_Reg(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_Compass_Reg(Interface, COMPASSID_HIGH);
}

static void HelperTxHandler_Compass_Reg(ESTTC_InterfacesEnum Interface, CompassId_t id)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];
    status_t RxData = WriteHandler_Compass_Reg(id, txline[Interface][7],txline[Interface][8]);
    uint8_t compassPrintId = GET_COMPASS_PRINT_ID(id);

    if (RxData == SEN_DISABLED)
    {
        sprintf(print_buff[Interface], "ERR - Magnetometers are disabled!");
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
    }
    else
    {
        if (RxData == SEN_SUCCESS)
        {
            sprintf(print_buff[Interface],
                    "OK+M%d:%02X/%02X",
                    compassPrintId, txline[Interface][7], txline[Interface][8]);

            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

            sprintf(print_buff[Interface],
                    "Magnetometer %d set reg No %d with value %d",
                    compassPrintId, txline[Interface][7], txline[Interface][8]);

            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
        }
        else
        {
            sprintf(print_buff[Interface], "ERR - executing");
            ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
        }
    }
}

static void ESTTC_TxHandler_Compass_1_Reg(ESTTC_InterfacesEnum Interface)
{
    HelperTxHandler_Compass_Reg(Interface, COMPASSID_LOW);
}

static void ESTTC_TxHandler_Compass_2_Reg(ESTTC_InterfacesEnum Interface)
{
    HelperTxHandler_Compass_Reg(Interface, COMPASSID_HIGH);
}

static void HelperRxHandler_GYRO_METRIC(ESTTC_InterfacesEnum Interface, PanId_t panel)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];
    uint8_t printablePanId = GET_PANEL_PRINT_ID(panel);

    if (panel >= MAX_PAN)
    {
        sprintf(print_buff[Interface], "ERR - Invalid Panel Id %d", printablePanId);
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
        return;
    }

    CommApi_Gyro_axis_Cmd_str RxData = ReadHandler_Gyro_Metric(panel);

    if (RxData.status != SEN_SUCCESS)
    {
        sprintf(print_buff[Interface], "ERR - executing");
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
        return;
    }

    float f16Data;
    if (RxData.data >= ADIS16260_GYRO_OUT_NEG_MAX_VAL)
        f16Data = ((ADIS16260_GYRO_OUT_NEG_MIN_VAL - RxData.data) * -ADCS_GYRO_ANG_RATE_LSB);
    else
        f16Data = (RxData.data * ADCS_GYRO_ANG_RATE_LSB);

    sprintf(print_buff[Interface], "OK+G%d:%6d", printablePanId, RxData.data);
    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

    sprintf(print_buff[Interface], "Gyroscope %d Data=%.6f", printablePanId, f16Data);
    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
}

static void ESTTC_RxHandler_GYRO_METRIC_PANEL_4(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_GYRO_METRIC(Interface, PANEL_4);
}

static void ESTTC_RxHandler_GYRO_METRIC_PANEL_5(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_GYRO_METRIC(Interface, PANEL_5);
}

static void ESTTC_RxHandler_GYRO_METRIC_PANEL_6(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_GYRO_METRIC(Interface, PANEL_6);
}

static void HelperRxHandler_GYRO_ANGLE(ESTTC_InterfacesEnum Interface, PanId_t panel)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];
    uint8_t printablePanId = GET_PANEL_PRINT_ID(panel);

    if (panel >= MAX_PAN)
    {
        sprintf(print_buff[Interface], "ERR - Invalid Panel Id %d", printablePanId);
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
        return;
    }

    CommApi_Gyro_axis_Cmd_str RxData = ReadHandler_Gyro_Angle(panel);

    if (RxData.status != SEN_SUCCESS)
    {
        sprintf(print_buff[Interface], "ERR - executing");
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
        return;
    }

    sprintf(print_buff[Interface], "OK+G%d:%6d", printablePanId, RxData.data);
    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

    sprintf(print_buff[Interface], "Gyroscope %d Angle = %6d", printablePanId, RxData.data);
    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
}

static void ESTTC_RxHandler_GYRO_ANGLE_PANEL_4(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_GYRO_ANGLE(Interface, PANEL_4);
}

static void ESTTC_RxHandler_GYRO_ANGLE_PANEL_5(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_GYRO_ANGLE(Interface, PANEL_5);
}

static void ESTTC_RxHandler_GYRO_ANGLE_PANEL_6(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_GYRO_ANGLE(Interface, PANEL_6);
}

static void HelperRxHandler_GYRO_REG(ESTTC_InterfacesEnum Interface, PanId_t panId)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];
    uint8_t printablePanId = GET_PANEL_PRINT_ID(panId);

    if (panId >= MAX_PAN)
    {
        sprintf(print_buff[Interface], "ERR - Invalid Panel Id %d", printablePanId);
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
        return;
    }

    uint8_t registerNum = txline[Interface][6];
    CommApi_Compass_reg_Cmd_str RxData = ReadHandler_Gyro_Reg(panId, registerNum);

    if (RxData.status != SEN_SUCCESS)
    {
        sprintf(print_buff[Interface], "ERR - executing");
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
        return;
    }

    sprintf(print_buff[Interface],
        "OK+G%d:%02X/%02X",
        printablePanId, registerNum, RxData.data);

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

    sprintf(print_buff[Interface],
        "Gyroscope %d reg. No %02d has value %3d",
        printablePanId, registerNum, RxData.data);

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
}

static void ESTTC_RxHandler_GYRO_REG_PANEL_4(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_GYRO_REG(Interface, PANEL_4);
}

static void ESTTC_RxHandler_GYRO_REG_PANEL_5(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_GYRO_REG(Interface, PANEL_5);
}

static void ESTTC_RxHandler_GYRO_REG_PANEL_6(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_GYRO_REG(Interface, PANEL_6);
}

static void HelperTxHandler_GYRO_REG(ESTTC_InterfacesEnum Interface, PanId_t panId)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];
    uint8_t printablePanId = GET_PANEL_PRINT_ID(panId);

    if (panId >= MAX_PAN)
    {
        sprintf(print_buff[Interface], "ERR - Invalid Panel Id %d", printablePanId);
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
        return;
    }

    uint16_t writeData = ((uint16_t)txline[Interface][8] << 8) + (uint16_t) txline[Interface][9];
    uint8_t registerNum = txline[Interface][7];
    status_t RxData = WriteHandler_Gyro_Reg(panId, registerNum, writeData);

    if (RxData == SEN_DISABLED)
    {
        sprintf(print_buff[Interface], "ERR - Gyros are disabled!");
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
        return;
    }

    if (RxData != SEN_SUCCESS)
    {
        sprintf(print_buff[Interface], "ERR - executing");
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
        return;
    }

    sprintf(print_buff[Interface],
        "OK+G%d:%02X/%02X",
        printablePanId, registerNum, writeData);

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

    sprintf(print_buff[Interface],
        "Gyroscope %d set reg. No %02d with value %3d",
        printablePanId, registerNum, writeData);

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
}

static void ESTTC_TxHandler_GYRO_REG_PANEL_4(ESTTC_InterfacesEnum Interface)
{
    HelperTxHandler_GYRO_REG(Interface, PANEL_4);
}

static void ESTTC_TxHandler_GYRO_REG_PANEL_5(ESTTC_InterfacesEnum Interface)
{
    HelperTxHandler_GYRO_REG(Interface, PANEL_5);
}

static void ESTTC_TxHandler_GYRO_REG_PANEL_6(ESTTC_InterfacesEnum Interface)
{
    HelperTxHandler_GYRO_REG(Interface, PANEL_6);
}

static void HelperRxHandler_MTORQ_POWER(ESTTC_InterfacesEnum Interface, PanId_t panel)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];
    uint8_t printablePanId = GET_PANEL_PRINT_ID(panel);

    if (panel >= MAX_PAN)
    {
        sprintf(print_buff[Interface], "ERR - Invalid Panel Id %d", printablePanId);
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
        return;
    }

    CommApi_Magnetorq_Pwr_Cmd_str RxData = ReadHandler_MagnetorquerData(panel);

    if (RxData.status != SEN_SUCCESS)
    {
        sprintf(print_buff[Interface], "ERR - Not valid parameters!");
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
        return;
    }

    const char axisDirAsChar = Panels_GetPanelAxisDirectionAsChar(RxData.direction);

    if (!axisDirAsChar)
    {
        sprintf(print_buff[Interface], "ERR - Not valid parameters!");
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
        return;
    }

    sprintf(print_buff[Interface], "OK+M%d:%02X/DIR%c", printablePanId, RxData.power, axisDirAsChar);
    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

    sprintf(print_buff[Interface],
            "Magnetorquer %d (TRQ%d) on PAN %d with power %d and direction %c",
            printablePanId, printablePanId, printablePanId, RxData.power, axisDirAsChar);

    if (RxData.state == 0)
        strcat(print_buff[Interface], " (Disabled)");

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
}

static void ESTTC_RxHandler_MTORQ_POWER_PANEL_4(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_MTORQ_POWER(Interface, PANEL_4);
}

static void ESTTC_RxHandler_MTORQ_POWER_PANEL_5(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_MTORQ_POWER(Interface, PANEL_5);
}

static void ESTTC_RxHandler_MTORQ_POWER_PANEL_6(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_MTORQ_POWER(Interface, PANEL_6);
}

static void HelperTxHandler_MTORQ_POWER(ESTTC_InterfacesEnum Interface, PanId_t panel)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];
    uint8_t printablePanId = GET_PANEL_PRINT_ID(panel);

    if (MagnTrq_ReadState() == 0)
    {
        sprintf(print_buff[Interface], "ERR - Magnetorquers are disabled!");
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
        return;
    }

    if (panel >= MAX_PAN)
    {
        sprintf(print_buff[Interface], "ERR - Invalid Panel Id %d", printablePanId);
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
        return;
    }

    const uint8_t power      = txline[Interface][7] > 100 ? 100 : txline[Interface][7];
    const uint8_t direction  = txline[Interface][8];
    const char axisDirAsChar = Panels_GetPanelAxisDirectionAsChar(direction);

    if (!axisDirAsChar)
    {
        sprintf(print_buff[Interface], "ERR - Not valid parameters!");
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
        return;
    }

    if (SetMagnetorque(panel, power, direction) != SEN_SUCCESS)
    {
        sprintf(print_buff[Interface], "ERR - Not valid parameters!");
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
        return;
    }

    sprintf(print_buff[Interface], "OK+M%d:%02X/DIR%c", printablePanId, power, axisDirAsChar);

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

    sprintf(print_buff[Interface],
            "Magnetorquer %d (TRQ%d) on PAN %d with power %d and direction %c",
            printablePanId, printablePanId, printablePanId, power, axisDirAsChar);

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
}

static void ESTTC_TxHandler_MTORQ_POWER_PANEL_4(ESTTC_InterfacesEnum Interface)
{
    HelperTxHandler_MTORQ_POWER(Interface, PANEL_4);
}

static void ESTTC_TxHandler_MTORQ_POWER_PANEL_5(ESTTC_InterfacesEnum Interface)
{
    HelperTxHandler_MTORQ_POWER(Interface, PANEL_5);
}

static void ESTTC_TxHandler_MTORQ_POWER_PANEL_6(ESTTC_InterfacesEnum Interface)
{
    HelperTxHandler_MTORQ_POWER(Interface, PANEL_6);
}

static void HelperRxHandler_TEMP(ESTTC_InterfacesEnum Interface, PanId_t panId)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];
    CommApi_Temp_Cmd_str tempData;
    uint8_t printablePanId = GET_PANEL_PRINT_ID(panId);
    bool bPanelAttached = false;
    char temp_as_str[9];

    if (panId >= MAX_PAN)
    {
        sprintf(print_buff[Interface], "ERR - Invalid Panel Id %d", printablePanId);
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
        return;
    }

    tempData = ReadHandler_Temp();
    bPanelAttached = (tempData.statusVec[panId] == SEN_SUCCESS);

    if (bPanelAttached)
    {
        sprintf(print_buff[Interface],
            "OK+T%d:%04X",
            printablePanId, (uint16_t)tempData.rawDataVec[panId]);

        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

        snprintf(temp_as_str, sizeof(temp_as_str),
            "%*.3f",
            (int)strlen(temp_as_str), (tempData.rawDataVec[panId] >> 3) * 0.0625);

        sprintf(print_buff[Interface],
            "Temperature Panel %d (PAN%d) = %s",
            printablePanId, printablePanId, temp_as_str);
    }
    else
        sprintf(print_buff[Interface], "ERR - Panel %d is not attached!", printablePanId);

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
}

static void ESTTC_RxHandler_TEMP_PANEL_1(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_TEMP(Interface, PANEL_1);
}

static void ESTTC_RxHandler_TEMP_PANEL_2(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_TEMP(Interface, PANEL_2);
}

static void ESTTC_RxHandler_TEMP_PANEL_3(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_TEMP(Interface, PANEL_3);
}

static void ESTTC_RxHandler_TEMP_PANEL_4(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_TEMP(Interface, PANEL_4);
}

static void ESTTC_RxHandler_TEMP_PANEL_5(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_TEMP(Interface, PANEL_5);
}

static void ESTTC_RxHandler_TEMP_PANEL_6(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_TEMP(Interface, PANEL_6);
}

static void HelperRxHandler_PHOTO(ESTTC_InterfacesEnum Interface, PanId_t panId)
{
    FILE * ComInterface = (FILE *) Esttc_usart_interfaces[Interface];
    PanPhotometricInfo_t sensData = ReadHandler_PhotometricInfo();
    uint8_t printablePanId = GET_PANEL_PRINT_ID(panId);

    if (panId >= MAX_PAN)
    {
        sprintf(print_buff[Interface], "ERR - Invalid Panel Id %d", printablePanId);
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
        return;
    }

    sprintf(print_buff[Interface],
        "OK+P%d:%02X",
        printablePanId, sensData.sensorReadings[panId]);

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

    sprintf(print_buff[Interface],
        "Panel Light %d (PAN%d) =%4u",
        printablePanId, printablePanId, sensData.sensorReadings[panId]);

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
}

static void ESTTC_RxHandler_PHOTO_PANEL_1(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_PHOTO(Interface, PANEL_1);
}

static void ESTTC_RxHandler_PHOTO_PANEL_2(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_PHOTO(Interface, PANEL_2);
}

static void ESTTC_RxHandler_PHOTO_PANEL_3(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_PHOTO(Interface, PANEL_3);
}

static void ESTTC_RxHandler_PHOTO_PANEL_4(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_PHOTO(Interface, PANEL_4);
}

static void ESTTC_RxHandler_PHOTO_PANEL_5(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_PHOTO(Interface, PANEL_5);
}

static void ESTTC_RxHandler_PHOTO_PANEL_6(ESTTC_InterfacesEnum Interface)
{
    HelperRxHandler_PHOTO(Interface, PANEL_6);
}

static void ESTTC_RxHandler_OUTPUT_CONTROL(ESTTC_InterfacesEnum Interface)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];
    GpioStat_t outputStates = ReadHandler_GpOutputState();

    sprintf(print_buff[Interface], "Output States 0x%X", outputStates.gpioStatusBitField);
    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
}

static void ESTTC_TxHandler_OUTPUT_CONTROL(ESTTC_InterfacesEnum Interface)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];
    HwRes_t res;
    GpioStat_t outputStatus;

    if (txline[Interface][8] < 2)
    {
        res = WriteHandler_GpOutputState(txline[Interface][7]-1, txline[Interface][8]);

        if (res != HWRES_SUCCESS)
        {
            sprintf(print_buff[Interface], "ERR - Invalid output!");
        }
        else
        {
            outputStatus = ReadHandler_GpOutputState();

            sprintf(print_buff[Interface], "Output States 0x%X", outputStatus.gpioStatusBitField);
        }
    }
    else
    {
        sprintf(print_buff[Interface], "ERR - Invalid state!");
    }

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
}

static void ESTTC_RxHandler_SENSORS_CONTROL(ESTTC_InterfacesEnum Interface)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];
    uint8_t sensorId = (uint8_t) txline[Interface][6];
    SensInUseData_t data = ReadHandler_SensorsInUse((ObcSensorId_t) sensorId);

    if (data.isSensorValid)
    {
        sprintf(print_buff[Interface],
            "%s in use by %03d user(s)",
            sensorNames[sensorId], data.u8UsersCount);
    }
    else
    {
        sprintf(print_buff[Interface], "ERR - Invalid sensor!");
    }

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
}

static void ESTTC_TxHandler_SENSORS_CONTROL(ESTTC_InterfacesEnum Interface)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];
    uint8_t sensorId = (uint8_t) txline[Interface][7];
    uint8_t cmdId = (uint8_t) txline[Interface][8];

    SensInUseData_t sensData = WriteHandler_SensorsInUse((ObcSensorId_t) sensorId, cmdId);

    if (sensData.isSensorValid)
    {
        switch (cmdId)
        {
            case SENCTRL_RESERVE_FOR_ONE_USER:
                sprintf(print_buff[Interface],
                    "%s in use by %03d user(s)",
                    sensorNames[sensorId], sensData.u8UsersCount);
            break;

            case SENCTRL_RELEASE_FOR_ONE_USER:
                if (sensData.u8UsersCount == 0)
                    sprintf(print_buff[Interface], "%s are disabled", sensorNames[sensorId]);
            break;

            case SENCTRL_RELEASE_FOR_ALL_AND_OFF:
                sprintf(print_buff[Interface], "Force disable of the %s", sensorNames[sensorId]);
            break;

            default:
            break;
        }
    }
    else
    {
        sprintf(print_buff[Interface], "ERR - executing");
    }

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
}

static void ESTTC_RxHandler_I2C_PULLUPS_CONTROL(ESTTC_InterfacesEnum Interface)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];
    I2CPUState_t i2cPuState = ReadHandler_I2CPullUpsState();

    sprintf(print_buff[Interface],
        "OK I2C Pull Ups: SYS 0x%d, PAY 0x%d",
        i2cPuState.SystemBus, i2cPuState.PayloadBus);

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
}

static void ESTTC_TxHandler_I2C_PULLUPS_CONTROL(ESTTC_InterfacesEnum Interface)
{
    I2CPUState_t reqState =
    {
        .SystemBus = txline[Interface][7],
        .PayloadBus = txline[Interface][8]
    };

    WriteHandler_I2CPullUpsState(reqState);

    // read back the I2C status after applying the requested changes
    ESTTC_RxHandler_I2C_PULLUPS_CONTROL(Interface);
}

static void ESTTC_RxHandler_GET_TIME(ESTTC_InterfacesEnum Interface)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];

    RtcTm_t timeVal = ReadHandler_RTCTime();

    if (timeVal.isTimeValid)
    {
        sprintf(print_buff[Interface],
            "OK TIME %02d:%02d:%02d",
            timeVal.hours, timeVal.minutes, timeVal.seconds);
    }
    else
    {
        sprintf(print_buff[Interface], "ERR - executing");
    }

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
}

static inline bool u8FromAscii(char digit1, char digit2, uint8_t* digit)
{
    if (digit1 < '0' || digit1 > '9')
        return false;

    if (digit2 < '0' || digit2 > '9')
        return false;

    *digit = (digit1 - (uint8_t) '0') * 10 + (digit2 - (uint8_t) '0');

    return true;
}

static void ESTTC_TxHandler_SET_TIME(ESTTC_InterfacesEnum Interface)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];
    RtcTm_t newTime = { 0 };
    StdResult_t res;

    if (!u8FromAscii(txline[Interface][ 7], txline[Interface][ 8], (uint8_t*)&newTime.hours) ||
        !u8FromAscii(txline[Interface][ 9], txline[Interface][10], &newTime.minutes) ||
        !u8FromAscii(txline[Interface][11], txline[Interface][12], &newTime.seconds))
    {
        sprintf(print_buff[Interface], "ERR - executing");
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
        return;
    }

    res = WriteHandler_RTCTime(newTime);

    switch (res)
    {
        case STDRESULT_SUCCESS:
            sprintf(print_buff[Interface],
                "OK TIME %02d:%02d:%02d",
                newTime.hours, newTime.minutes, newTime.seconds);
        break;

        case STDRESULT_INVALID_ARGS:
            sprintf(print_buff[Interface], "ERR - Wrong parameters");
        break;

        case STDRESULT_ERROR:
        default:
            sprintf(print_buff[Interface], "ERR - executing");
        break;
    }

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
}

static void ESTTC_RxHandler_GET_DATA(ESTTC_InterfacesEnum Interface)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];

    RtcDt_t dateVal = ReadHandler_RTCDate();

    if (dateVal.isDateValid)
        sprintf(print_buff[Interface],
            "OK DATE YY/MM/DD %02d / %02d / %02d",
            dateVal.year, dateVal.month, dateVal.day);
    else
        sprintf(print_buff[Interface], "ERR - executing");

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
}

static void ESTTC_TxHandler_SET_DATA(ESTTC_InterfacesEnum Interface)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];
    RtcDt_t newDate = { 0 };
    StdResult_t res;

    if (!u8FromAscii(txline[Interface][ 7], txline[Interface][ 8], (uint8_t*)&newDate.year) ||
        !u8FromAscii(txline[Interface][ 9], txline[Interface][10], &newDate.month) ||
        !u8FromAscii(txline[Interface][11], txline[Interface][12], &newDate.day))
    {
        sprintf(print_buff[Interface], "ERR - executing");
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
        return;
    }

    // TODO: FIX THIS - Now it is aligned with the current ESTTC documentation / GUI. Must be fixed in future versions
    newDate.dayOfWeek = DATA_VALID_DAY_OF_WEEK;

    res = WriteHandler_RTCDate(newDate);

    switch (res)
    {
        case STDRESULT_SUCCESS:
            sprintf(print_buff[Interface],
                "OK DATE YY/MM/DD %02d / %02d / %02d",
                newDate.year, newDate.month, newDate.day);
        break;

        case STDRESULT_INVALID_ARGS:
            sprintf(print_buff[Interface], "ERR - Wrong parameters");
        break;

        default:
        case STDRESULT_ERROR:
            sprintf(print_buff[Interface], "ERR - executing");
        break;
    }

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
}

static void ESTTC_RxHandler_UPTIME(ESTTC_InterfacesEnum Interface)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];
    UpTimeInfo_t upTime = ReadHandler_Uptime();

    sprintf(print_buff[Interface],
        "OK UPTIME DDDDD:HH:MM:SS %05u:%02d:%02d:%02d",
        (unsigned int) upTime.days, upTime.hours, upTime.minutes, upTime.seconds);

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
}

static void ESTTC_TxHandler_UPTIME(ESTTC_InterfacesEnum Interface)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];

#if (ENABLE_ESTTC_FAULT_TESTS == 1)
    UpTimeInfo_t reqUptime = { 0 };
    StdResult_t res;

    if (!u8FromAscii(txline[Interface][ 7], txline[Interface][ 8], (uint8_t*)&reqUptime.days) ||
        !u8FromAscii(txline[Interface][ 9], txline[Interface][10], &reqUptime.hours)   ||
        !u8FromAscii(txline[Interface][11], txline[Interface][12], &reqUptime.minutes) ||
        !u8FromAscii(txline[Interface][13], txline[Interface][14], &reqUptime.seconds))
    {
        sprintf(print_buff[Interface], "ERR - executing");
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
        return;
    }

    res = WriteHandler_Uptime(reqUptime);

    if (res == STDRESULT_SUCCESS)
    {
        ESTTC_RxHandler_UPTIME(Interface);  // read status after applying new up time
    }
    else if (res == STDRESULT_INVALID_ARGS)
    {
        sprintf(print_buff[Interface], "ERR UPTIME - Wrong params");
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
    }
#else
    sprintf(print_buff[Interface], "The command is disabled");
    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
#endif
}

static void ESTTC_RxHandler_ANT_SETTINGS(ESTTC_InterfacesEnum Interface)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];

#if (ENABLE_UHF_ANT_SUPPORT == 1)
    AntUHFSettings_Struct settings = AntUHF_ReadSettings();
    static const char* const hardcodedResponses[2U] =
    {
        "Enabled\0",
        "Disabled\0"
    };

    // Print out the status of bit 1 - Enabled/Disable of the Antenna service
    sprintf(print_buff[Interface],
        "UHF Antenna service: %s",
        (settings.Byte.AntUHFEnable) ? (hardcodedResponses[0U]) : (hardcodedResponses[1U]));

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

    // Print out the status of First deployment algorithm
    sprintf(print_buff[Interface],
        "Special algorithm: %s",
        (settings.Byte.AntUHFFrstDepl) ? (hardcodedResponses[0U]) : (hardcodedResponses[1U]));

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

    sprintf(print_buff[Interface],
        "Use Out2 for deployment: %s",
        (settings.Byte.Out2Deployment) ? (hardcodedResponses[0U]) : (hardcodedResponses[1U]));

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

    sprintf(print_buff[Interface],
        "Low battery check status: %s",
        (settings.Byte.LowBatCheckEn) ? (hardcodedResponses[0U]) : (hardcodedResponses[1U]));

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

    // Print out the delay after power up until the service of the antenna is started
    sprintf(print_buff[Interface], "Power up delay: %d Minutes",
            settings.Byte.PowerUpDelay);
    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

#if (ENABLE_EPS_I_SUPPORT == 1)
    sprintf(print_buff[Interface],
        "Low battery voltage threshold: %d [ADC]/ %d [mv]",
        settings.Byte.LowBatThreshold, (uint16_t)(settings.Byte.LowBatThreshold * EPS_I_ROW_TO_MV_CONST));
#else
    sprintf(print_buff[Interface],
        "Low battery voltage threshold: %d [ADC]",
        settings.Byte.LowBatThreshold);
#endif
#else
    sprintf(print_buff[Interface], "ERR - executing");
#endif

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
}

static void ESTTC_TxHandler_ANT_SETTINGS(ESTTC_InterfacesEnum Interface)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];

#if (ENABLE_UHF_ANT_SUPPORT == 1)
    UHFAntResult res;
    AntUHFSettings_Struct newSettings;

    newSettings.u32Settings = (uint32_t)((uint32_t)txline[Interface][7]  << 24) |
                                        ((uint32_t)txline[Interface][8] << 16) |
                                        ((uint32_t)txline[Interface][9] <<  8) |
                                        ((uint32_t)txline[Interface][10]);

    res = AntUHF_UpdateSettings(&newSettings);

    if (res == UHFANTRESULT_SUCCESS)
    {
        ESTTC_RxHandler_ANT_SETTINGS(Interface);
    }
    else
    {
        sprintf(print_buff[Interface], "Wrong power-up delay");
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
    }
#else
    sprintf(print_buff[Interface], "ERR - executing");
    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
#endif
}

static void ESTTC_RxHandler_CAPTURE_PAR(ESTTC_InterfacesEnum Interface)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];

#if (ENABLE_CAMERA_OV5640 == 1)
    CamCtrl = txline[Interface][7];
    if (Camm_IsCammReady())
    {
        if (Camm_GetStatus() == 1)
        {
            sprintf(print_buff[Interface], "ERR+CAMBUSY");
        }
        else
        {
            Camm_MakePicture();
            sprintf(print_buff[Interface], "OK+Request a Picture");
        }
    }
    else
    {
        sprintf(print_buff[Interface], "ERR+NOCAM");
    }
#else
    sprintf(print_buff[Interface], "ERR+NO CAM CMD");
#endif
    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
}

void ESTTC_RxHandler_RST_COUNTS(ESTTC_InterfacesEnum Interface)
{
    FILE * ComInterface = (FILE *) Esttc_usart_interfaces[Interface];

    sprintf(print_buff[Interface],
            "RST Counters: "
            "0.WWD = %03d / "
            "1.IWD = %03d / "
            "2.LPR = %03d / "
            "3.POR = %03d / "
            "4.RstPin = %03d / "
            "5.BOR = %03d / "
            "6.HardFault = %03d / "
            "7.MemFault = %03d / "
            "8.BusFault = %03d / "
            "9.UsageFault = %03d",
            (int) BootData->RST_WWD,
            (int) BootData->RST_IWD,
            (int) BootData->RST_LPR,
            (int) BootData->RST_POR,
            (int) BootData->RST_RstPin,
            (int) BootData->RST_BOR,
            (int) BootData->RST_HardFault,
            (int) BootData->RST_MemFault,
            (int) BootData->RST_BusFault,
            (int) BootData->RST_UsageFault);

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
}

static void ESTTC_TxHandler_RST_COUNTS(ESTTC_InterfacesEnum Interface)
{
    static const char* cntrNames[RSTCOUNTERID_ALL] =
    {
       "WWD",
       "IWD",
       "LPR",
       "POR",
       "RstPin",
       "BOR",
       "HardFault",
       "MemFault",
       "BusFault",
       "UsageFault"
    };

    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];
    uint8_t cntrId = txline[Interface][7];

    bool res = Svc_Rtc_ClearResetCounter((RstCounterId_t) cntrId);

    if (res)
        sprintf(print_buff[Interface], "Cleared RST counter: %s", cntrNames[cntrId]);
    else
        sprintf(print_buff[Interface], "Wrong reset counter number: %0d", cntrId);

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
}

static void ESTTC_TxHandler_FAULTS_TST(ESTTC_InterfacesEnum Interface)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];
    uint8_t faultTestId = txline[Interface][7];
    StdResult_t res = WriteHandler_FaultTest(faultTestId);

    switch (res)
    {
        case STDRESULT_INVALID_ARGS:
            sprintf(print_buff[Interface], "Wrong fault number: %0d", faultTestId);
        break;

        case STDRESULT_ERROR:
            sprintf(print_buff[Interface], "The command is disabled");
        break;

        case STDRESULT_SUCCESS:
        default:
        break;
    }

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
}

static void ESTTC_RxHandler_POWER_MODE(ESTTC_InterfacesEnum Interface)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];
#if (ENABLE_POWER_MANAGER == 1)
    PowerModeInfo_t res = ReadHandler_PowerModeInfo();

    switch (res.pwMode)
    {
        case PWRMNG_DYNAMIC_RUN_MODE:
            sprintf(print_buff[Interface], "00 Dynamic run mode");
        break;

        case PWRMNG_STOP_MODE:
            sprintf(print_buff[Interface], "01 Stop mode");
        break;

        default:
            sprintf(print_buff[Interface], "Default Err");
        break;
    }

    sprintf(&print_buff[Interface][strlen(print_buff[Interface])],
            ", Sleep Period: %5d seconds, Rx Awake Period: %3d seconds",
            res.settings.sleepPeriod, res.settings.stayAwakePeriod);
#else
    sprintf(print_buff[Interface], "ERR - executing");
#endif

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
}

static void ESTTC_TxHandler_POWER_MODE(ESTTC_InterfacesEnum Interface)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];

#if (ENABLE_POWER_MANAGER == 1)
    PwrModeSettingsResult res;
    PwrModeSettings_t newSettings;

    if (txline[Interface][6] == 4)   // Validate the requested size
    {
        newSettings.stayAwakePeriod = (uint16_t)txline[Interface][8];
        newSettings.sleepPeriod = (uint16_t)((((uint16_t)txline[Interface][9]) << 8) | (uint16_t)txline[Interface][10]);

        res = WriteHandler_PowerModeSettings(&newSettings);

        switch (res)
        {
            case PWRMODESETTINGSRESULT_SUCCESS:
                sprintf(print_buff[Interface],
                    "Sleep Period: %5d seconds, Rx Awake Period: %3d seconds",
                    newSettings.sleepPeriod, newSettings.stayAwakePeriod);
            break;

            case PWRMODESETTINGSRESULT_INVALID_SLEEP_TIME:
                sprintf(print_buff[Interface], "ERR - wrong sleep period %d", newSettings.sleepPeriod);
            break;

            case PWRMODESETTINGSRESULT_INVALID_STAYAWAKE_TIME:
                sprintf(print_buff[Interface], "ERR - wrong awake period %d", newSettings.stayAwakePeriod);
            break;

            default:
            break;
        }
    }
    else
    {
        sprintf(print_buff[Interface], "ERR - size");
    }
#else
    sprintf(print_buff[Interface], "ERR - Power Manager Disabled");
#endif

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
}

static void ESTTC_RxHandler_RESET(ESTTC_InterfacesEnum Interface)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];

    ESTTC_PrintVersion(print_buff[Interface]);
    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
}

static void ESTTC_TxHandler_RESET(ESTTC_InterfacesEnum Interface)
{
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];
    AppMode_t newAppMode = APPMODE_MAX;

    if ((txline[Interface][6] == 1) && (txline[Interface][7] == 0xA))
    {
        sprintf(print_buff[Interface], "OK+Requested. APP");

        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

        newAppMode = APPMODE_APPLICATION;
    }
    else if ((txline[Interface][6] == 1) && (txline[Interface][7] == 0xB))
    {
        sprintf(print_buff[Interface], "OK+Requested. BOOT");

        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

        newAppMode = APPMODE_BOOTLOADER;
    }
    else if ((txline[Interface][6] == 1) && (txline[Interface][7] == 0xC))
    {
        sprintf(print_buff[Interface], "OK+Reboot+AutoFwUpdate");
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

        newAppMode = APPMODE_AUTO_FW_UPDATE;
    }
    else
    {
        sprintf(print_buff[Interface], "ERR");
        ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);
    }

    // wait some time until the USART buffers are empty
    osDelay(50);

    if (newAppMode != APPMODE_MAX)
        (void) WriteHandler_ResetInBootOrAppMode(newAppMode);
}

static void KUbeSatCommandTest(ESTTC_InterfacesEnum Interface) {
    FILE* ComInterface = (FILE*)Esttc_usart_interfaces[Interface];

    sprintf(print_buff[Interface], "Command Worked Print Buffer\n");

    ESTTC_CMD_CRC_Print_string(ComInterface, print_buff[Interface]);

    DBG_SYSCON("Command Worked Syscon\n");

}

esttc_cmd_handlers_type CmdHandlerList[] =
{
    { ESTTC_CMD_MAG1_DATA,               (input_func)ESTTC_RxHandler_Compass_1_XYZ,       (input_func)NULL },
    { ESTTC_CMD_MAG1_ACESS,              (input_func)ESTTC_RxHandler_Compass_1_Reg,       (input_func)ESTTC_TxHandler_Compass_1_Reg },
    { ESTTC_CMD_MAG2_DATA,               (input_func)ESTTC_RxHandler_Compass_2_XYZ,       (input_func)NULL },
    { ESTTC_CMD_MAG2_ACESS,              (input_func)ESTTC_RxHandler_Compass_2_Reg,       (input_func)ESTTC_TxHandler_Compass_2_Reg },

    { ESTTC_CMD_GYRO_RADIO_DATA_PANEL_4, (input_func)ESTTC_RxHandler_GYRO_METRIC_PANEL_4, (input_func)NULL },
    { ESTTC_CMD_GYRO_ANGLE_DATA_PANEL_4, (input_func)ESTTC_RxHandler_GYRO_ANGLE_PANEL_4,  (input_func)NULL },
    { ESTTC_CMD_GYRO_REG_PANEL_4,        (input_func)ESTTC_RxHandler_GYRO_REG_PANEL_4,    (input_func)ESTTC_TxHandler_GYRO_REG_PANEL_4 },
    { ESTTC_CMD_GYRO_RADIO_DATA_PANEL_5, (input_func)ESTTC_RxHandler_GYRO_METRIC_PANEL_5, (input_func)NULL },
    { ESTTC_CMD_GYRO_ANGLE_DATA_PANEL_5, (input_func)ESTTC_RxHandler_GYRO_ANGLE_PANEL_5,  (input_func)NULL },
    { ESTTC_CMD_GYRO_REG_PANEL_5,        (input_func)ESTTC_RxHandler_GYRO_REG_PANEL_5,    (input_func)ESTTC_TxHandler_GYRO_REG_PANEL_5 },
    { ESTTC_CMD_GYRO_RADIO_DATA_PANEL_6, (input_func)ESTTC_RxHandler_GYRO_METRIC_PANEL_6, (input_func)NULL },
    { ESTTC_CMD_GYRO_ANGLE_DATA_PANEL_6, (input_func)ESTTC_RxHandler_GYRO_ANGLE_PANEL_6,  (input_func)NULL },
    { ESTTC_CMD_GYRO_REG_PANEL_6,        (input_func)ESTTC_RxHandler_GYRO_REG_PANEL_6,    (input_func)ESTTC_TxHandler_GYRO_REG_PANEL_6 },
    { ESTTC_CMD_MTORQ_POWER_PANEL_4,     (input_func)ESTTC_RxHandler_MTORQ_POWER_PANEL_4, (input_func)ESTTC_TxHandler_MTORQ_POWER_PANEL_4 },
    { ESTTC_CMD_MTORQ_POWER_PANEL_5,     (input_func)ESTTC_RxHandler_MTORQ_POWER_PANEL_5, (input_func)ESTTC_TxHandler_MTORQ_POWER_PANEL_5 },
    { ESTTC_CMD_MTORQ_POWER_PANEL_6,     (input_func)ESTTC_RxHandler_MTORQ_POWER_PANEL_6, (input_func)ESTTC_TxHandler_MTORQ_POWER_PANEL_6 },
    { ESTTC_CMD_TEMP_PANEL_1,            (input_func)ESTTC_RxHandler_TEMP_PANEL_1,        (input_func)NULL },
    { ESTTC_CMD_TEMP_PANEL_2,            (input_func)ESTTC_RxHandler_TEMP_PANEL_2,        (input_func)NULL },
    { ESTTC_CMD_TEMP_PANEL_3,            (input_func)ESTTC_RxHandler_TEMP_PANEL_3,        (input_func)NULL },
    { ESTTC_CMD_TEMP_PANEL_4,            (input_func)ESTTC_RxHandler_TEMP_PANEL_4,        (input_func)NULL },
    { ESTTC_CMD_TEMP_PANEL_5,            (input_func)ESTTC_RxHandler_TEMP_PANEL_5,        (input_func)NULL },
    { ESTTC_CMD_TEMP_PANEL_6,            (input_func)ESTTC_RxHandler_TEMP_PANEL_6,        (input_func)NULL },
    { ESTTC_CMD_PHOTO_PANEL_1,           (input_func)ESTTC_RxHandler_PHOTO_PANEL_1,       (input_func)NULL },
    { ESTTC_CMD_PHOTO_PANEL_2,           (input_func)ESTTC_RxHandler_PHOTO_PANEL_2,       (input_func)NULL },
    { ESTTC_CMD_PHOTO_PANEL_3,           (input_func)ESTTC_RxHandler_PHOTO_PANEL_3,       (input_func)NULL },
    { ESTTC_CMD_PHOTO_PANEL_4,           (input_func)ESTTC_RxHandler_PHOTO_PANEL_4,       (input_func)NULL },
    { ESTTC_CMD_PHOTO_PANEL_5,           (input_func)ESTTC_RxHandler_PHOTO_PANEL_5,       (input_func)NULL },
    { ESTTC_CMD_PHOTO_PANEL_6,           (input_func)ESTTC_RxHandler_PHOTO_PANEL_6,       (input_func)NULL },

    { ESTTC_CMD_OUTPUT_CONTROL,          (input_func)ESTTC_RxHandler_OUTPUT_CONTROL,      (input_func)ESTTC_TxHandler_OUTPUT_CONTROL },
    { ESTTC_CMD_SENSORS_CONTROL,         (input_func)ESTTC_RxHandler_SENSORS_CONTROL,     (input_func)ESTTC_TxHandler_SENSORS_CONTROL },
    { ESTTC_CMD_I2C_PULLUPS_CONTROL,     (input_func)ESTTC_RxHandler_I2C_PULLUPS_CONTROL, (input_func)ESTTC_TxHandler_I2C_PULLUPS_CONTROL },

    { ESTTC_CMD_GET_TIME,                (input_func)ESTTC_RxHandler_GET_TIME,            (input_func)NULL },
    { ESTTC_CMD_SET_TIME,                (input_func)NULL,                                (input_func)ESTTC_TxHandler_SET_TIME },
    { ESTTC_CMD_GET_DATA,                (input_func)ESTTC_RxHandler_GET_DATA,            (input_func)NULL },
    { ESTTC_CMD_SET_DATA,                (input_func)NULL,                                (input_func)ESTTC_TxHandler_SET_DATA },

    { ESTTC_CMD_UPTIME,                  (input_func)ESTTC_RxHandler_UPTIME,              (input_func)ESTTC_TxHandler_UPTIME },

    { ESTTC_CMD_ANT_SETTINGS,            (input_func)ESTTC_RxHandler_ANT_SETTINGS,        (input_func)ESTTC_TxHandler_ANT_SETTINGS },

    { ESTTC_CMD_CAPTURE_PAR,             (input_func)ESTTC_RxHandler_CAPTURE_PAR,         (input_func)NULL },
    { ESTTC_CMD_RST_COUNTS,              (input_func)ESTTC_RxHandler_RST_COUNTS,          (input_func)ESTTC_TxHandler_RST_COUNTS },
    { ESTTC_CMD_FAULTS_TST,              (input_func)NULL,                                (input_func)ESTTC_TxHandler_FAULTS_TST },

    { ESTTC_CMD_POWER_MODE,              (input_func)ESTTC_RxHandler_POWER_MODE,          (input_func)ESTTC_TxHandler_POWER_MODE },

    { ESTTC_CMD_RESET,                   (input_func)ESTTC_RxHandler_RESET,               (input_func)ESTTC_TxHandler_RESET },
    { ESTTC_CMD_PAR_NUM,                 (input_func)NULL,                                (input_func)NULL },
    { KUbeSatTest,                       (input_func)KUbeSatCommandTest,                  (input_func)NULL },
};

#endif  // #ifndef __ESTTC_CMDHANDLERC_IMPL__
