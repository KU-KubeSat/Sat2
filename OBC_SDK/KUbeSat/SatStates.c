/*
 * SatStates.c
 *
 *  Created on: Mar 9, 2022
 *      Author: daniel
 */


#include "SatStates.h"

int counterState = 0;

/*
 *
 *
 * Modes:
 *  Deploy
 *  Tumble/Recovery
 *  Normal Ops
 *  Standby
 *  Low Power
 *  Error
 *  GS Overpass
 *
 *
 */

void Deploy(void *parameter) {
    DBG_SYSCON("Entering Deploy\n");

    // Do state stuff
    vTaskDelay(2000/portTICK_PERIOD_MS);

    // State Manager
    xTaskCreate(TumbleRec, "Tumble State", 1024, NULL, 1, NULL);

    vTaskDelete(NULL);
}

void TumbleRec(void *parameter) {
    DBG_SYSCON("Entering Detumble Recovery\n");

    // Do state stuff
    vTaskDelay(2000/portTICK_PERIOD_MS);

    // State Manager
    xTaskCreate(NormOps, "Normal Operations State", 1024, NULL, 1, NULL);

    vTaskDelete(NULL);
}

void NormOps(void *parameter) {
    DBG_SYSCON("Entering Normal Operations\n");

    // Do state stuff
    vTaskDelay(2000/portTICK_PERIOD_MS);

    // State Manager
    int nextState = 0; // Int for now, move to enum
    nextState = counterState;
    counterState++;

    switch (nextState) {
        case 0:
            xTaskCreate(Standby, "Standby State", 1024, NULL, 1, NULL);
            break;
        case 1:
            xTaskCreate(GSOverpass, "Ground State Overpass State", 1024, NULL, 1, NULL);
            break;
        case 2:
            xTaskCreate(LowPower, "Low Power State", 1024, NULL, 1, NULL);
            break;
        case 3:
            xTaskCreate(ErrorState, "Error State", 1024, NULL, 1, NULL);
            break;
        case 4:
            xTaskCreate(TumbleRec, "Detumble Recovery State", 1024, NULL, 1, NULL);
            break;
        case 5:
            DBG_SYSCON("Ending Test\n");
            break;
    }

    vTaskDelete(NULL);
}

void Standby(void *parameter) {
    DBG_SYSCON("Entering Standby\n");

    // Do state stuff
    vTaskDelay(2000/portTICK_PERIOD_MS);

    // State Manager
    xTaskCreate(NormOps, "Normal Operations State", 1024, NULL, 1, NULL);

    vTaskDelete(NULL);
}

void LowPower(void *parameter) {
    DBG_SYSCON("Entering Low Power\n");

    // DO state stuff
    vTaskDelay(2000/portTICK_PERIOD_MS);

    // State Manager
    xTaskCreate(TumbleRec, "Tumble State", 1024, NULL, 1, NULL);

    vTaskDelete(NULL);
}

void ErrorState(void *parameter) {
    DBG_SYSCON("Entering Error\n");

    // Do state stuff
    vTaskDelay(2000/portTICK_PERIOD_MS);

    // State Manager
    xTaskCreate(TumbleRec, "Tumble State", 1024, NULL, 1, NULL);

    vTaskDelete(NULL);
}

void GSOverpass(void *parameter) {
    DBG_SYSCON("Entering GSOverpass\n");

    // Do state stuff
    vTaskDelay(2000/portTICK_PERIOD_MS);

    // State Manager
    xTaskCreate(NormOps, "Normal Operations State", 1024, NULL, 1, NULL);

    vTaskDelete(NULL);
}
