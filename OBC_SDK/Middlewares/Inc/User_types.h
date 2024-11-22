/*!
********************************************************************************************
* @file User_types.h
* @brief Header of common types
********************************************************************************************
* @author            Kolio
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
* @revision{         1.0.0  , 2018.07.04, author Kolio, Initial revision }
* @endhistory
********************************************************************************************
*/
#ifndef USER_TYPES_H
#define USER_TYPES_H

/*
*********************************************************************************************
* INCLUDES
*********************************************************************************************
*/
#include <stdbool.h>
#include <stdint.h>

/*
*********************************************************************************************
* EXTERNAL DEFINES
*********************************************************************************************
*/

/*
*********************************************************************************************
* EXTERNAL TYPES DECLARATIONS
*********************************************************************************************
*/
typedef enum MagnetometersType {
  Magnetometer_0,
  Magnetometer_1,
  Magnetometer_2,
  NO_OF_MMETER_IDS
} MagnetometersType;

typedef enum ES_ReturnType {
  E_OK,
  E_PENDING,
  E_UNKNOWN_FAILURE,
  E_NOT_INIT,
  E_INV_ARG,
  E_INV_CFG,
  E_ERROR,
  E_MMETER_READING,
  E_FILTERING,
  E_DATA_NORMALIZE,
  E_BDOT_CALCULATING,
  E_MTORQ_UPDATE,
  E_NOT_OK
} ES_ReturnType;

typedef struct Magnetorquer_Axis_t {
  double AXIS_X;
  double AXIS_Y;
  double AXIS_Z;  
}__attribute__((__packed__)) Magnetorquer_Axis_t;

typedef struct Compass_Axis_t{
  double AXIS_X;
  double AXIS_Y;
  double AXIS_Z;
}__attribute__((__packed__)) Compass_Axis_t;

typedef enum
{
  SEN_ERROR = 0x00,
  SEN_SUCCESS = 0x01,
  SEN_DISABLED = 0xFF
} status_t;

typedef struct {
  int16_t AXIS_X;
  int16_t AXIS_Y;
  int16_t AXIS_Z;
}__attribute__((__packed__)) AxesRaw_t;

typedef struct {
  int16_t Temp_X;
  int16_t Temp_Y;
  int16_t Temp_Z;
}__attribute__((__packed__)) Temperature_t;


/*
*********************************************************************************************
* EXTERNAL VARIABLES DECLARATIONS
*********************************************************************************************
*/
/* No External variables declarations */

/*
*********************************************************************************************
* EXTERNAL ROUTINES DECLARATIONS 
*********************************************************************************************
*/
/* No External routine declarations */

#endif    /* USER_TYPES_H */
/* **************************************************************************************** */
