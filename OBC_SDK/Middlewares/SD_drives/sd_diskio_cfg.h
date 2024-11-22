/*
 * sd_diskio_cfg.h
 *
 *  Created on: 15 May 2021
 *      Author: Ivan Petrov
 */

#ifndef SD_DRIVES_SD_DISKIO_CFG_H_
#define SD_DRIVES_SD_DISKIO_CFG_H_

// maximum size of the SD diskio driver scratch buffer (you should ensure that applications do not
// write larger chunks than this size because the driver cannot properly handle it for the moment)
#define SCRATCH_BUF_SIZE                    (8192U)

#endif /* SD_DRIVES_SD_DISKIO_CFG_H_ */
