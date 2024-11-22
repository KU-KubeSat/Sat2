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

FIL testFile;
//int a = 0;

void SDCardTaskCreate(void *parameter);
void SDCardTaskWrite(void *parameter);

void UHFTest(void *parameter);
void UHFTestOld(void *parameter);
void EPSTest(void *parameter);


#endif /* KUBESATTASKS_H_ */
