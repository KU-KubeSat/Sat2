/*
 * KUbeSatTasks.c
 *
 *  Created on: Mar 2, 2022
 *      Author: daniel
 */


#include "KUbeSatTasks.h"

#define TIME_BETWEEN_PACS   (3)

typedef struct
{
    uint8_t initalStr[3];
    uint8_t opperation;
    uint8_t address[2];
    uint8_t command[2];
    uint8_t AddCRCString[98];
} __attribute__((__packed__)) Test_Command_Struct;


//Test Task
void lightTask(void *parameter) {
    while (1) {
        AMBER_LED_ON();
        vTaskDelay(500/portTICK_PERIOD_MS); // 500 ms
        AMBER_LED_OFF();
        vTaskDelay(500/portTICK_PERIOD_MS);
    }
}

void lightTask2(void *parameter) {
    while (1) {
        AMBER_LED_ON();
        vTaskDelay(100/portTICK_PERIOD_MS); // 500 ms
        AMBER_LED_OFF();
        vTaskDelay(250/portTICK_PERIOD_MS);
    }
}

// Playing w/ global variables
/*void Test1(void *parameter) {
    while (1) {
        DBG_SYSCON("A: %d", a);
        vTaskDelay(1000/portTICK_PERIOD_MS); // 500 ms
    }
}

void Test2(void *parameter) {
    while (1) {
        a++;
        vTaskDelay(200/portTICK_PERIOD_MS); // 500 ms
    }
}*/



//New Test
void SDCardTask2(void *parameter) {
    DBG_SYSCON("Entered SD Card Task2\n");
    FIL testFile;
    testFile.obj.fs = 0;
    FRESULT fr;           //FatFs function common result code
    //fr = SdMngr_f_open(&testFile, "Test.txt", FA_CREATE_ALWAYS);
    SdMngr_RequestSdCardInit(3, 1);
    fr = SdMngr_f_open(&testFile, "0 /DirList.txt", FA_WRITE | FA_CREATE_ALWAYS);

    DBG_SYSCON("File Result: %d\n", (int)fr);
    DBG_SYSCON("Exited SD Card Task2\n");
}

void SDCardTaskWrite(void *parameter);

void TestTask(void *parameter) {
    DBG_SYSCON("Entered Test Task\n");
    DBG_SYSCON("Exited Test Task\n");
    vTaskDelete(NULL);
}


// Working
void SDCardTaskCreate(void *parameter) {
    DBG_SYSCON("Entered SD Card Task\n");
    testFile.obj.fs = 0;

    FRESULT fr;           //FatFs function common result code
    SdMngr_RequestSdCardInit(3, 1);
    fr = SdMngr_f_open(&testFile, "0:/Test.txt", FA_WRITE | FA_CREATE_ALWAYS);

    DBG_SYSCON("File Result: %d\n", fr);
    DBG_SYSCON("Exited SD Card Task\n");
    xTaskCreate(SDCardTaskWrite, "Write to File", 1024, NULL, 2, NULL);
    vTaskDelete(NULL);
}

int writeCount;
void SDCardTaskWrite(void *parameter) {
    DBG_SYSCON("Entered SD Card Task Write\n");

    FRESULT fr2;           //FatFs function common result code

    while (1) {
    fr2 = SdMngr_f_printf(&testFile, "Write: %d\n", writeCount);
    writeCount++;
    vTaskDelay(1000/portTICK_PERIOD_MS);

    if (writeCount == 100)
        break;
    }

    DBG_SYSCON("Write Result: %d\n", fr2);
    DBG_SYSCON("Exited SD Card Task Write\n");

    SdMngr_f_close(&testFile);
    vTaskDelete(NULL);
}

void UHFTestOld(void *parameter) {
    DBG_SYSCON("Entered UHF Test Old\n");

    uint32_t init = 0;
    UHFcmdStatus_Enum UHFResult;
    uint32_t *upTime = 1;
    //upTime = init;

    UHFResult = UHF_Get_Uptime(1, upTime);

    int data = *upTime;
    DBG_SYSCON("UHF up time: %d\n", data);
    DBG_SYSCON("UHF Result: %d\n", UHFResult);
    DBG_SYSCON("Exiting UHF Test");

    vTaskDelete(NULL);
}

void UHFTest(void *parameter) {
    DBG_SYSCON("Entered UHF Test\n");

    Test_Command_Struct txMessage = {
        .initalStr =
        {
            'E',
            'S',
            '+'
        },
        .opperation = 'R',
        .address =
        {
            2,
            2
        }
    };

    txMessage.command[0] = 0;
    txMessage.command[1] = 2;

    //uint32_t CRC_value_calc1 = crc32(0, (BYTE*)&txMessage, sizeof(Test_Command_Struct) - 11);

    //sprintf((char*)&txMessage.AddCRCString, " %08X\r", (unsigned int)CRC_value_calc1);
     txMessage.AddCRCString[0] = '\r';


    uint8_t temp_reg_buff[26];

    HAL_StatusTypeDef I2C_stat;
    I2C_stat = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);
    DBG_SYSCON("Status 1 %d\n", I2C_stat);


    for (int i = 0; i < 5; i++) {
    I2C_stat = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, UHF_I2C_ADDRESS, (uint8_t*)&txMessage, 18);
    DBG_SYSCON("Status 2 %d\n", I2C_stat);

    osDelay(TIME_BETWEEN_PACS);

    if (I2C_stat == HAL_OK)
    {
        I2C_stat =
            MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, UHF_I2C_ADDRESS, &temp_reg_buff[0], 106);
        DBG_SYSCON("Status 3 %d\n", I2C_stat);

        osDelay(TIME_BETWEEN_PACS); //wait after receiving to prevent to frequent request
    }

    if (I2C_stat == HAL_OK)
        break;

    DBG_SYSCON("Try: %d\n", i);
    }

    MX_I2C_Release(MX_I2C_BUS_SYSTEM);

    for (int i = 0; i < 26; i++) {
        DBG_SYSCON("%d ", temp_reg_buff[i]);
    }
    DBG_SYSCON("\n");

    DBG_SYSCON("Exiting UHF Test");

    vTaskDelete(NULL);
}

void I2CTest(void *parameter) {
    DBG_SYSCON("Entered I2C Test\n");

    HAL_StatusTypeDef I2C_stat;
    I2C_stat = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);
    DBG_SYSCON("Status 1: %d\n", I2C_stat);

    uint8_t dataPtr = 1;

    for (int i = 0; i < 5; i++) {
        I2C_stat = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, OBC_I2C_ADDRESS, &dataPtr, 1);
        DBG_SYSCON("Status 2: %d    Try: %d\n", I2C_stat, i);

        if (I2C_stat == HAL_OK)
            break;

        osDelay(3);
    }

    MX_I2C_Release(MX_I2C_BUS_SYSTEM);

    DBG_SYSCON("Exiting UHF Test\n");
    vTaskDelete(NULL);
}

void I2CTest2(void *parameter) {
    DBG_SYSCON("Entered I2C Test\n");

    osDelay(10*1000/portTICK_PERIOD_MS);

    DBG_SYSCON("End Delay\n");

    HAL_StatusTypeDef I2C_stat;
    uint16_t devAddress = 0x57;
    uint8_t *pData = (uint8_t) malloc(1*sizeof(uint8_t));
    uint8_t temp_reg_buff[8];

    pData[0] = 0x00;

    I2C_stat = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);
    DBG_SYSCON("Status Take: %d\n", I2C_stat);

    int count = 0;
    while(1) {
        //pData[0] = count;
        I2C_stat = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, devAddress, pData, 1);
        DBG_SYSCON("Status Transmit: %d\n", I2C_stat);
        //osDelay(50);
        DBG_SYSCON("Count: %d\n", count);
        //I2C_stat = MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, devAddress, &temp_reg_buff[0], 8); //Size 2 for EPS
        //DBG_SYSCON("Status Recieve: %d\n", I2C_stat);

        //for (int i = 0; i < 8; i++) {
        //    DBG_SYSCON("%d ", temp_reg_buff[i]);
        //}
        //DBG_SYSCON("\n");

        osDelay(1000/portTICK_PERIOD_MS);
        count++;
    }

    DBG_SYSCON("Exiting I2C Test\n");
    vTaskDelete(NULL);
}

void PiTest(void *parameter) {
    DBG_SYSCON("Entered Pi Test\n");

    Test_Send_Serial();

    DBG_SYSCON("Exiting PI Test\n");
    vTaskDelete(NULL);
}

void EPSTest(void *parameter) {
    DBG_SYSCON("Entered EPS Test\n");

    HAL_StatusTypeDef EPSResult;
    uint16_t devAddress = CLYDE_EPS_ADDRESS;
    uint8_t *pData = (uint8_t) malloc(2*sizeof(uint8_t)); //Get Watchdog
    uint8_t *pData2 = (uint8_t) malloc(2*sizeof(uint8_t)); //Set Watchdog
    uint8_t temp_reg_buff[2];
    //uint8_t temp_reg_buff2[1];

    // Set PMD On
    pData[0] = 0x40;
    pData[1] = 0x00; //0xE3;
    //pData[2] = 0x08;
    uint16_t size = 2;

    // Request PDM Status
    pData2[0] = 0x42;
    pData2[1] = 0x00;

    //for (int i = 0; i < 100000; i++) {

    for (int i = 0; i < 10; i++) {
        //I2C_HandleTypeDef *test = MX_I2C_GetI2cHandler(MX_I2C_BUS_SYSTEM);
        EPSResult = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);
        DBG_SYSCON("Status Take: %d\n", EPSResult);

        EPSResult = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, devAddress, pData, size);

        DBG_SYSCON("Status Transmit: %d    Try: %d\n", EPSResult, i);

        osDelay(1/portTICK_PERIOD_MS);
        //osDelay(TIME_BETWEEN_PACS);

        if (EPSResult == HAL_OK)
        {
            EPSResult =
                MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, devAddress, &temp_reg_buff[0], 2); //Size 2 for EPS
            DBG_SYSCON("Status 3 %d\n", EPSResult);

            osDelay(4/portTICK_PERIOD_MS); //wait after receiving to prevent to frequent request
        }


        if (EPSResult == HAL_OK)
            break;


        osDelay(10);
    }

    DBG_SYSCON("Get Watchdog 1 %d %d\n", temp_reg_buff[0], temp_reg_buff[1]);

    osDelay(5/portTICK_PERIOD_MS);

    // Set Watchdog
    EPSResult = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, devAddress, pData2, size);
    DBG_SYSCON("Status 4 %d\n", EPSResult);

    EPSResult =
             MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, devAddress, &temp_reg_buff[0], 2);

    osDelay(5/portTICK_PERIOD_MS);

    // Get Watchdog 2
    EPSResult = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, devAddress, pData, size);

    osDelay(2/portTICK_PERIOD_MS);

    EPSResult =
         MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, devAddress, &temp_reg_buff[0], 2);

    DBG_SYSCON("Status 5 %d\n", EPSResult);

    DBG_SYSCON("Get Watchdog 2 %d %d\n", temp_reg_buff[0], temp_reg_buff[1]);

//    for (int i = 0; i < 2; i++) {
//        DBG_SYSCON("%d ", temp_reg_buff[i]);
//    }

    //DBG_SYSCON("\n");

    MX_I2C_Release(MX_I2C_BUS_SYSTEM);

    //osDelay(2000/portTICK_PERIOD_MS);
    //}



    //double result = (temp_reg_buff[0] << 8 | temp_reg_buff[1])*0.372434 - 273.15;

    //DBG_SYSCON("Result: %f\n", result);

    DBG_SYSCON("Exiting EPS Test\n");

    vTaskDelete(NULL);
}

void heaterTest(void *paramater) {
    DBG_SYSCON("Entered Heater Test\n");

    HAL_StatusTypeDef EPSResult;
    uint16_t devAddress = CLYDE_BAT_ADDRESS;
    uint8_t *pData = (uint8_t) malloc(2*sizeof(uint8_t));
    uint8_t *pData2 = (uint8_t) malloc(2*sizeof(uint8_t));
    uint8_t *pData3 = (uint8_t) malloc(2*sizeof(uint8_t));
    uint8_t *pData4 = (uint8_t) malloc(3*sizeof(uint8_t));
    uint8_t temp_reg_buff[2];
    uint8_t temp_reg_buff2[2];
    uint8_t temp_reg_buff3[2];

    double boardTemp = -1;

    // Set Heater On
    pData[0] = 0x91;
    pData[1] = 0x01;
    uint16_t size = 2;

    // Request Heater Status
    pData2[0] = 0x90;
    pData2[1] = 0x00;

    // Turn Heater off
    pData3[0] = 0x91;
    pData3[1] = 0x00;

    // Get board temp
    pData4[0] = 0x10;
    pData4[1] = 0xE3;
    pData4[2] = 0x9F;

    // Take
    EPSResult = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);

    // Request Status 1
    EPSResult = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, devAddress, pData2, size);
    osDelay(30/portTICK_PERIOD_MS);
    EPSResult =
             MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, devAddress, &temp_reg_buff[0], 2);

    // Switch on
    osDelay(50/portTICK_PERIOD_MS);
    EPSResult = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, devAddress, pData, size);

    // Request again
    osDelay(1000/portTICK_PERIOD_MS);
    EPSResult = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, devAddress, pData2, size);
    osDelay(30/portTICK_PERIOD_MS);
    EPSResult =
             MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, devAddress, &temp_reg_buff2[0], 2);

    // Get board Temp
    EPSResult = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, devAddress, pData4, 3);
    osDelay(30/portTICK_PERIOD_MS);
    EPSResult =
             MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, devAddress, &temp_reg_buff3[0], 2);
    boardTemp = (temp_reg_buff3[0] << 8 | temp_reg_buff3[1])*0.372434 - 273.15;

    // Release
    MX_I2C_Release(MX_I2C_BUS_SYSTEM);

    // Print 1
    DBG_SYSCON("Status 1: ");
    for (int i = 0; i < 2; i++) {
       DBG_SYSCON("%d ", temp_reg_buff[i]);
    }
    DBG_SYSCON("\n");

    // Print 2
    DBG_SYSCON("Status 2: ");
    for (int i = 0; i < 2; i++) {
        DBG_SYSCON("%d ", temp_reg_buff2[i]);
    }
    DBG_SYSCON("\n");
    DBG_SYSCON("Board Temp: %f\n", boardTemp);


    // Take
   EPSResult = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);

   // Turn off
   osDelay(1000*30/portTICK_PERIOD_MS);
   EPSResult = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, devAddress, pData3, size);

   // Request again again
   osDelay(1000/portTICK_PERIOD_MS);
   EPSResult = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, devAddress, pData2, size);
   osDelay(30/portTICK_PERIOD_MS);
   EPSResult =
            MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, devAddress, &temp_reg_buff[0], 2);

   // Get board Temp
   EPSResult = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, devAddress, pData4, 3);
   osDelay(30/portTICK_PERIOD_MS);
   EPSResult =
            MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, devAddress, &temp_reg_buff3[0], 2);
   boardTemp = (temp_reg_buff3[0] << 8 | temp_reg_buff3[1]); //*0.372434 - 273.15;

   // Release
   MX_I2C_Release(MX_I2C_BUS_SYSTEM);
   // Print 3
   DBG_SYSCON("Status 3: ");
   for (int i = 0; i < 2; i++) {
      DBG_SYSCON("%d ", temp_reg_buff[i]);
   }
   DBG_SYSCON("\n");
   DBG_SYSCON("Board Temp: %f\n", boardTemp);

   DBG_SYSCON("Exiting Heater Test\n");

   vTaskDelete(NULL);
}

void pdmTest(void *paramater) {
    DBG_SYSCON("Entered PDM Test\n");

    HAL_StatusTypeDef EPSResult;
    uint16_t devAddress = CLYDE_EPS_ADDRESS;
    uint8_t *pData = (uint8_t) malloc(2*sizeof(uint8_t));
    uint8_t *pData2 = (uint8_t) malloc(2*sizeof(uint8_t));
    uint8_t *pData3 = (uint8_t) malloc(2*sizeof(uint8_t));
    uint8_t temp_reg_buff[4];
    uint8_t temp_reg_buff2[4];

    // Set PMD On
    pData[0] = 0x50;
    pData[1] = 0x05; //0xE3;
    //pData[2] = 0x08;
    uint16_t size = 2;

    // Request PDM Status
    pData2[0] = 0x42;
    pData2[1] = 0x00;

    // Turn PDM off
    pData3[0] = 0x51;
    pData3[1] = 0x05;

    // Take
    EPSResult = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);

    // Request Status 1
    EPSResult = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, devAddress, pData2, size);
    osDelay(30/portTICK_PERIOD_MS);
    EPSResult =
             MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, devAddress, &temp_reg_buff[0], 4);

    // Switch on
    osDelay(50/portTICK_PERIOD_MS);
    EPSResult = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, devAddress, pData, size);

    // Request again
    osDelay(1000/portTICK_PERIOD_MS);
    EPSResult = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, devAddress, pData2, size);
    osDelay(30/portTICK_PERIOD_MS);
    EPSResult =
             MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, devAddress, &temp_reg_buff2[0], 4);

    // Release
    MX_I2C_Release(MX_I2C_BUS_SYSTEM);

    // Print 1
    DBG_SYSCON("Status 1: ");
    for (int i = 0; i < 4; i++) {
       DBG_SYSCON("%d ", temp_reg_buff[i]);
    }
    DBG_SYSCON("\n ");

    // Print 2
    DBG_SYSCON("Status 2: ");
    for (int i = 0; i < 4; i++) {
        DBG_SYSCON("%d ", temp_reg_buff2[i]);
    }
    DBG_SYSCON("\n ");

    // Take
    EPSResult = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);

    // Turn off
    osDelay(1000*30/portTICK_PERIOD_MS);
    EPSResult = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, devAddress, pData3, size);

    // Request again again
    osDelay(1000/portTICK_PERIOD_MS);
    EPSResult = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, devAddress, pData2, size);
    osDelay(30/portTICK_PERIOD_MS);
    EPSResult =
             MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, devAddress, &temp_reg_buff[0], 4);

    // Release
    MX_I2C_Release(MX_I2C_BUS_SYSTEM);
    // Print 3
    DBG_SYSCON("Status 3: ");
    for (int i = 0; i < 4; i++) {
       DBG_SYSCON("%d ", temp_reg_buff[i]);
    }
    DBG_SYSCON("\n ");

    DBG_SYSCON("Exiting PDM Test\n");

    vTaskDelete(NULL);
}

void checkClydStatus(void *parameter) {
    osDelay(100/portTICK_PERIOD_MS);
    for (int i = 0; i < 10; i++) {
    uint8_t epsStat = getEPSStatus();
    osDelay(100/portTICK_PERIOD_MS);
    uint8_t batStat = getBatStatus();

    DBG_SYSCON("EPS Status: %d\n", epsStat);
    DBG_SYSCON("Bat Status: %d\n", batStat);
    osDelay(1000/portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void EPSPing(void *parameter) {
    HAL_StatusTypeDef I2CResult;
    uint16_t devAddressEPS = CLYDE_EPS_ADDRESS;

    uint8_t *statusCMD = (uint8_t) malloc(2*sizeof(uint8_t));
    statusCMD[0] = 0x01;
    statusCMD[1] = 0x00;
    uint16_t statusSize = 2;

    while(1) {
        I2CResult = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);
        I2CResult = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, devAddressEPS, statusCMD, statusSize);

        if (I2CResult == HAL_OK) {
            DBG_SYSCON("EPS Transmit Successful\n");
        } else {
            DBG_SYSCON("EPS Transmit Unsuccessful\n");
        }

        MX_I2C_Release(MX_I2C_BUS_SYSTEM);

        double rate = 1000*60; // Rate in miliseconds
        osDelay(rate/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void bootCheck(void *parameter) {
    HAL_StatusTypeDef I2CResult;
    uint16_t devAddressEPS = CLYDE_EPS_ADDRESS;
    uint16_t devAddressBAT = CLYDE_BAT_ADDRESS;

    uint8_t temp_reg_buff[2];
    uint8_t *statusCMD = (uint8_t) malloc(2*sizeof(uint8_t));
    uint8_t *batVoltCMD = (uint8_t) malloc(3*sizeof(uint8_t));
    uint8_t *batVoltCMD2 = (uint8_t) malloc(3*sizeof(uint8_t));

    statusCMD[0] = 0x01;
    statusCMD[1] = 0x00;
    uint16_t statusSize = 2;

    batVoltCMD[0] = 0x10;
    batVoltCMD[1] = 0xE2;
    batVoltCMD[2] = 0x20;
    uint16_t batVoltCMDSize = 3;

    batVoltCMD2[0] = 0x10;
    batVoltCMD2[1] = 0xE2;
    batVoltCMD2[2] = 0x80;

    for (int i = 0; i < 1; i++) {
        I2CResult = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);

        //EPS Status
        I2CResult = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, devAddressEPS, statusCMD, statusSize);


        osDelay(1/portTICK_PERIOD_MS);

        if (I2CResult == HAL_OK)
        {
            I2CResult =
                MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, devAddressEPS, &temp_reg_buff[0], 2);

            osDelay(1/portTICK_PERIOD_MS); //wait after receiving to prevent to frequent request
        }

        if (temp_reg_buff[0] == 0 && temp_reg_buff[1] == 0) {
            DBG_SYSCON("EPS Good Boot\n");
        } else {
            DBG_SYSCON("EPS Bad Boot %d %d\n", temp_reg_buff[0], temp_reg_buff[1]);
        }

        if (I2CResult != HAL_OK)
            DBG_SYSCON("EPS I2C Error: Status Check\n");

        osDelay(10/portTICK_PERIOD_MS);

        MX_I2C_Release(MX_I2C_BUS_SYSTEM);
        I2CResult = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);

        //BAT Status
        I2CResult = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, devAddressBAT, statusCMD, statusSize);


        osDelay(8/portTICK_PERIOD_MS);

        if (I2CResult == HAL_OK)
        {
            I2CResult =
                MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, devAddressBAT, &temp_reg_buff[0], 2);

            osDelay(1/portTICK_PERIOD_MS); //wait after receiving to prevent to frequent request
        }

        osDelay(10);

        // 128 is good boot meaning heating circitry is on (bits are backwards in returned value)
        if (temp_reg_buff[0] == 0 && temp_reg_buff[1] == 128) {
            DBG_SYSCON("BAT Good Boot\n");
        } else {
            DBG_SYSCON("BAT Bad Boot %d %d\n", temp_reg_buff[0], temp_reg_buff[1]);
        }

        if (I2CResult != HAL_OK)
            DBG_SYSCON("Bat I2C Error: Status Check\n");

        //Bat voltage
        I2CResult = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, devAddressEPS, batVoltCMD, batVoltCMDSize);


        osDelay(8/portTICK_PERIOD_MS);

        if (I2CResult == HAL_OK)
        {
            I2CResult =
                MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, devAddressEPS, &temp_reg_buff[0], 2);

            osDelay(1/portTICK_PERIOD_MS); //wait after receiving to prevent to frequent request
        }

        osDelay(10);

        double batVolt = (temp_reg_buff[0] << 8 | temp_reg_buff[1])*0.008978;
        //DBG_SYSCON("Battery Voltage Results: %d %d %d\n", temp_reg_buff[0], temp_reg_buff[1], temp_reg_buff[0] << 8 | temp_reg_buff[1]);

        if (batVolt > 7.46) {
            DBG_SYSCON("EPS Battery High %f\n", batVolt);
        } else {
            DBG_SYSCON("EPS Battery Low %f\n", batVolt);
        }

        I2CResult = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, devAddressBAT, batVoltCMD2, batVoltCMDSize);

        osDelay(8/portTICK_PERIOD_MS);

        I2CResult =
              MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, devAddressBAT, &temp_reg_buff[0], 2);

        double batVolt2 = (temp_reg_buff[0] << 8 | temp_reg_buff[1])*0.008993;
        DBG_SYSCON("BAT Battery %f\n", batVolt2);

        if (I2CResult != HAL_OK)
            DBG_SYSCON("EPS I2C Error: Bat Voltage\n");

        MX_I2C_Release(MX_I2C_BUS_SYSTEM);

        }

   double testBat = getEPSBat();
   DBG_SYSCON("Test Voltage %f\n", testBat);

   xTaskCreate(EPSPing, "EPSPing", 1024, NULL, 1, NULL);
   vTaskDelete(NULL);
}

uint8_t getEPSStatus() {
    HAL_StatusTypeDef I2CResult;
    uint16_t devAddressEPS = CLYDE_EPS_ADDRESS;
    uint8_t temp_reg_buff[2];

    uint8_t *statusCMD = (uint8_t) malloc(2*sizeof(uint8_t));
    statusCMD[0] = 0x01;
    statusCMD[1] = 0x00;
    uint16_t cmdSize = 2;


    // Transmit
    I2CResult = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);

    //Bat voltage
    I2CResult = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, devAddressEPS, statusCMD, cmdSize);


    osDelay(8/portTICK_PERIOD_MS);

    if (I2CResult == HAL_OK)
    {
        I2CResult =
            MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, devAddressEPS, &temp_reg_buff[0], 2);

        osDelay(1/portTICK_PERIOD_MS); //wait after receiving to prevent to frequent request
    }

    osDelay(10);
    MX_I2C_Release(MX_I2C_BUS_SYSTEM);

    return temp_reg_buff[1];
}

uint8_t getBatStatus() {
    HAL_StatusTypeDef I2CResult;
    uint16_t devAddressEPS = CLYDE_BAT_ADDRESS;
    uint8_t temp_reg_buff[2];

    uint8_t *statusCMD = (uint8_t) malloc(2*sizeof(uint8_t));
    statusCMD[0] = 0x01;
    statusCMD[1] = 0x00;
    uint16_t cmdSize = 2;


    // Transmit
    I2CResult = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);

    //Bat voltage
    I2CResult = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, devAddressEPS, statusCMD, cmdSize);


    osDelay(8/portTICK_PERIOD_MS);

    if (I2CResult == HAL_OK)
    {
        I2CResult =
            MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, devAddressEPS, &temp_reg_buff[0], 2);

        osDelay(1/portTICK_PERIOD_MS); //wait after receiving to prevent to frequent request
    }

    osDelay(10);
    MX_I2C_Release(MX_I2C_BUS_SYSTEM);

    return temp_reg_buff[1];
}

double getEPSBat() {
    HAL_StatusTypeDef I2CResult;
    uint16_t devAddressEPS = CLYDE_EPS_ADDRESS;
    uint8_t temp_reg_buff[2];

    uint8_t *batVoltCMD = (uint8_t) malloc(3*sizeof(uint8_t));
    batVoltCMD[0] = 0x10;
    batVoltCMD[1] = 0xE2;
    batVoltCMD[2] = 0x20;
    uint16_t batVoltCMDSize = 3;


    // Transmit
    I2CResult = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);

    //Bat voltage
    I2CResult = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, devAddressEPS, batVoltCMD, batVoltCMDSize);


    osDelay(8/portTICK_PERIOD_MS);

    if (I2CResult == HAL_OK)
    {
        I2CResult =
            MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, devAddressEPS, &temp_reg_buff[0], 2);

        osDelay(1/portTICK_PERIOD_MS); //wait after receiving to prevent to frequent request
    }

    osDelay(10);
    MX_I2C_Release(MX_I2C_BUS_SYSTEM);

    double batVolt = (temp_reg_buff[0] << 8 | temp_reg_buff[1])*0.008978;
    return batVolt;
}

void radioTest(void *parameter) {
    DBG_SYSCON("Entering Radio Test\n");
    vTaskDelay(10*1000/portTICK_PERIOD_MS);

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 100*portTICK_PERIOD_MS;

    HAL_StatusTypeDef I2CResult[8];
    uint8_t temp_reg_buff[26];
    uint8_t temp_reg_buff2[26];
    uint8_t temp_reg_buff3[26];

    Test_Command_Struct txMessage = {
            .initalStr =
            {
                'E',
                'S',
                '+'
            },
            .opperation = 'W',
            .address =
            {
                2,
                2
            }
        };

    Test_Command_Struct txMessage2 = {
        .initalStr =
        {
            'E',
            'S',
            '+'
        },
        .opperation = 'W',
        .address =
        {
            2,
            2
        }
    };

    Test_Command_Struct txMessage3 = {
        .initalStr =
        {
            'E',
            'S',
            '+'
        },
        .opperation = 'R',
        .address =
        {
            2,
            2
        }
    };



        txMessage.command[0] = 'E';
        txMessage.command[1] = 'E';

        txMessage2.command[0] = 'F';
        txMessage2.command[1] = 'B';

        txMessage3.command[0] = 'F';
        txMessage3.command[1] = 'B';

        //uint32_t CRC_value_calc1 = crc32(0, (BYTE*)&txMessage, sizeof(Test_Command_Struct) - 11);

        //sprintf((char*)&txMessage.AddCRCString, " %08X\r", (unsigned int)CRC_value_calc1);
         txMessage.AddCRCString[0] = '\r';
         //sprintf((char*)&txMessage2.AddCRCString, "0ATest, Test\r");
         txMessage3.AddCRCString[0] = '\r';


    I2CResult[0] = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);

    //sprintf((char*)&txMessage2.AddCRCString, "0BHello Earth\r");
    sprintf((char*)&txMessage2.AddCRCString, "09Count: 0%d\r", 0);// Changing TxMessage
    txMessage.initalStr[0] = 'E';

    int errorCount = -1;

    int counter = 0;

    DBG_SYSCON("Message: ");
    for (int i = 0; i < 3; i++) {
        DBG_SYSCON("%c", txMessage.initalStr[i]);
    }

    DBG_SYSCON("%c", txMessage.opperation);
    DBG_SYSCON("%d", txMessage.address[0]);
    DBG_SYSCON("%d", txMessage.address[1]);
    DBG_SYSCON("%c", txMessage.command[0]);
    DBG_SYSCON("%c", txMessage.command[1]);
    DBG_SYSCON("\n");

    // Start Becon
    for (int i = 0; i < 41; i++) {
    I2CResult[1] = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, UHF_I2C_ADDRESS, (uint8_t*)&txMessage, 20);
    //vTaskDelay(5*1000/portTICK_PERIOD_MS);
    xLastWakeTime = xTaskGetTickCount() - TIME_BETWEEN_PACS;
    TickType_t timeStart = xTaskGetTickCount()/portTICK_PERIOD_MS;



    if (i < 10) {
        sprintf((char*)&txMessage2.AddCRCString, "09Count: 0%d\r", i);
        txMessage.initalStr[0] = 'E';
    } else {
        sprintf((char*)&txMessage2.AddCRCString, "09Count: %d\r", i);
        txMessage.initalStr[0] = 'E';
    }

    DBG_SYSCON("Take: %d\n", i);
    vTaskDelayUntil( &xLastWakeTime, xFrequency);

    // Test Read
    I2CResult[2] = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, UHF_I2C_ADDRESS, (uint8_t*)&txMessage2, 22);
    //I2CResult[3] = MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, UHF_I2C_ADDRESS, &temp_reg_buff[0], 23);
    TickType_t timeRead1 = xTaskGetTickCount()/portTICK_PERIOD_MS;

    osDelay(TIME_BETWEEN_PACS);
    if ((i-errorCount)%7 == 0) {
        DBG_SYSCON("Error HERE\n");
        //MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, UHF_I2C_ADDRESS, NULL, 18);
        osDelay(TIME_BETWEEN_PACS);
    }
    I2CResult[4] = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, UHF_I2C_ADDRESS, (uint8_t*)&txMessage3, 18);
    I2CResult[5] = MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, UHF_I2C_ADDRESS, &temp_reg_buff2[0], 23);
    TickType_t timeRead2 = xTaskGetTickCount()/portTICK_PERIOD_MS;

    if (temp_reg_buff2[0] == 0 && errorCount == -1) {
        errorCount = i;
        DBG_SYSCON("Error: %d\n", errorCount);
    }

    //I2CResult[6] = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, UHF_I2C_ADDRESS, (uint8_t*)&txMessage2, 18);
    //I2CResult[7] = MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, UHF_I2C_ADDRESS, &temp_reg_buff3[0], 23);
    TickType_t timeRead3 = xTaskGetTickCount()/portTICK_PERIOD_MS;

    DBG_SYSCON("I2C Status: ");
    for (int i = 0; i < 8; i++) {
        DBG_SYSCON("%d ", I2CResult[i]);
    }
    DBG_SYSCON("\n");

    /*DBG_SYSCON("Transmit 1: ");
    for (int i = 0; i < 26; i++) {
        DBG_SYSCON("%d ", temp_reg_buff[i]);
    }
    DBG_SYSCON("\n");*/

    DBG_SYSCON("Read Beacon Back: ");
    for (int i = 0; i < 26; i++) {
        DBG_SYSCON("%d ", temp_reg_buff2[i]);
    }
    DBG_SYSCON("\n");

    /*DBG_SYSCON("Transmit 3: ");
    for (int i = 0; i < 26; i++) {
        DBG_SYSCON("%d ", temp_reg_buff3[i]);
    }
    DBG_SYSCON("\n");*/

    DBG_SYSCON("Start Time: %d\n", (int)timeStart);
    DBG_SYSCON("Time1: %d\n", (int)timeRead1);
    DBG_SYSCON("Time2: %d\n", (int)timeRead2);
    DBG_SYSCON("Time3: %d\n", (int)timeRead3);

    }

    DBG_SYSCON("Error: %d\n", errorCount);
    MX_I2C_Release(MX_I2C_BUS_SYSTEM);

    DBG_SYSCON("Exiting Radio Test");
    vTaskDelete(NULL);
}

