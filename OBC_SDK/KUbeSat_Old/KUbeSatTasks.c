/*
 * KUbeSatTasks.c
 *
 *  Created on: Mar 2, 2022
 *      Author: daniel
 */


#include "KUbeSatTasks.h"

#define TIME_BETWEEN_PACS   (3)


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
    DBG_SYSCON("Entered UHF Test Old");

    uint32_t init = 0;
    UHFcmdStatus_Enum UHFResult;
    uint32_t *upTime;
    upTime = &init;

    UHFResult = UHF_Get_Uptime(1, upTime);

    DBG_SYSCON("UHF up time: %d\n", (int) upTime);
    DBG_SYSCON("UHF Result: %d\n", UHFResult);
    DBG_SYSCON("Exiting UHF Test");

    vTaskDelete(NULL);
}

void UHFTest(void *parameter) {
    DBG_SYSCON("Entered UHF Test\n");
    uint8_t temp_reg_buff[26];

    HAL_StatusTypeDef I2C_stat;
    I2C_stat = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 100);
    DBG_SYSCON("Status 1 %d\n", I2C_stat);
                                                                                    //ES+W UFHAdd 070000 value \r
    I2C_stat = MX_I2C_BlockingTransmit(MX_I2C_BUS_SYSTEM, UHF_I2C_ADDRESS, (uint8_t*)&"ES+W22070000000A \r", 8);
    DBG_SYSCON("Status 2 %d\n", I2C_stat);

    osDelay(TIME_BETWEEN_PACS);

    if (I2C_stat == HAL_OK)
    {
        I2C_stat =
            MX_I2C_BlockingReceive(MX_I2C_BUS_SYSTEM, UHF_I2C_ADDRESS, &temp_reg_buff[0], 23);
        DBG_SYSCON("Status 3 %d\n", I2C_stat);

        osDelay(TIME_BETWEEN_PACS); //wait after receiving to prevent to frequent request
    }

    MX_I2C_Release(MX_I2C_BUS_SYSTEM);

    for (int i = 0; i < 26; i++) {
        DBG_SYSCON("%d ", temp_reg_buff[i]);
    }
    DBG_SYSCON("\n");

    DBG_SYSCON("Exiting UHF Test");

    vTaskDelete(NULL);
}

/*void EPSTest(void *parameter) {
    DBG_SYSCON("Entered EPS Test");

    HAL_StatusTypeDef EPSResult;
    MX_I2Cbus_enum bus = 0;
    uint16_t devAddress = CLYDE_EPS_ADDRESS;
    uint8_t *pData = "S2BW01";
    uint16_t size;
    I2C_stat = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);

    EPSResult = MX_I2C_BlockingTransmit(bus, devAddress, pData, size);

    sDelay(TIME_BETWEEN_PACS);



    DBG_SYSCON("Exiting EPS Test");

    vTaskDelete(NULL);
}*/

