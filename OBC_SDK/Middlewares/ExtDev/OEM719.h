/**
 *********************************************************************************************
 * @file OEM719.h
 * @brief Header of OEM719.
 *********************************************************************************************
 * @author            OEM719
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
 * @revision{         1.0.0  , 2018.07.04, author OEM719, Initial revision }
 * @endhistory
 *********************************************************************************************
 */
#ifndef OEM719_H
#define OEM719_H

#include <stdio.h>


void OEM719_Init       (void);
void OEM719_Task       (void* args);
void OEM719_SendRawCmd (FILE* com, uint8_t* rawCmd, uint32_t rawCmdSz);
void OEM719_ReceiveChar(void);


#endif // OEM719_H

