#include "MCU_Init.h"
#include <stdio.h>
#include "../Infrastructure/debug/inc/debug.h"
#include "main.h"


FILE* testUart = (FILE*)&huart4;
char buff[100];
uint8_t data;

void startUpUartTest() {
    MX_UART4_Init();
}

void transmitData() {
    //fprintf(testUart, "Testing Serial Comms!\n");
    HAL_UART_Transmit((UART_HandleTypeDef*)testUart, (uint8_t*)"Test Serial\n", 12, 5);
}

void recieveData1() {
    fscanf(testUart, "%s", buff);
}

void recieveData2() {
    HAL_UART_Receive((UART_HandleTypeDef*)testUart, (uint8_t*)&data, 10, 5);

    DBG_SYSCON("Data: %c\n", data);
    transmitData();
}
