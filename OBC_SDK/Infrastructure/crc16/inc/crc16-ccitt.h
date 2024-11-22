/*
 * crc16-ccitt.h
 *
 *  Created on: Sep 2, 2020
 *      Author: Anton Tkachenko
 */

#ifndef CRC16_CCITT_H_
#define CRC16_CCITT_H_

#include <stdint.h>

uint16_t crc16_ccitt_table(uint8_t const *pDataP, unsigned int iLenP);
uint16_t crc16_ccitt_calc(uint8_t const *pDataP, unsigned int iLenP);
uint16_t crc16_8(uint8_t *fp_pu8Data, int fp_u16Count);


#endif /* CRC16_CCITT_H_ */
