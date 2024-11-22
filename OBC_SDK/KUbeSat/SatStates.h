/*
 * SatStates.h
 *
 *  Created on: Mar 9, 2022
 *      Author: daniel
 */

#ifndef SATSTATES_H_
#define SATSTATES_H_

#include "TaskMonitor.h"
#include "SdMngr.h"
#include "../Infrastructure/debug/inc/debug.h"
#include "UHF.h"
#include "MX_I2C.h"

void Deploy(void *parameter);
void TumbleRec(void *parameter);
void NormOps(void *parameter);
void Standby(void *parameter);
void LowPower(void *parameter);
void ErrorState(void *parameter);
void GSOverpass(void *parameter);



#endif /* SATSTATES_H_ */
