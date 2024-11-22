/*
 * KUbeSatTasks.h
 *
 *  Created on: Mar 2, 2022
 *      Author: daniel
 */

#ifndef KUBESATTASKS_H_
#define KUBESATTASKS_H_

#include "TaskMonitor.h"
#include "SdMngr.h"
#include "../Infrastructure/debug/inc/debug.h"
#include "UHF.h"
#include "MX_I2C.h"
#include "es_crc32.h"
#include "ESTTC.h"
#include "task.h"

FIL testFile;
//int a = 0;

void SDCardTaskCreate(void *parameter);
void SDCardTaskWrite(void *parameter);

void UHFTest(void *parameter);
void UHFTestOld(void *parameter);
void EPSTest(void *parameter);

void I2CTest(void *parameter);
void I2CTest2(void *parameter);

void PiTest(void *parameter);

void pdmTest(void *paramater);
void heaterTest(void *paramater);

void bootCheck(void *parameter);
double getEPSBat();
uint8_t getEPSStatus();
uint8_t getBatStatus();

void checkClydStatus(void *parameter);

void radioTest(void *parameter);
void radioTest2(void *parameter);

// Ping to Avoid Watchdog reset
void EPSPing(void *parameter);


#endif /* KUBESATTASKS_H_ */
