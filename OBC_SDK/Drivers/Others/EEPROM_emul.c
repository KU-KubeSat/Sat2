/*!
********************************************************************************************
* @file EEPROM_Emul.c
* @brief Emulates EEPROM in the Flash memory
********************************************************************************************
* @author            Vassil Milev
* @version           1.0.0
* @date              2019.06.24
*
* @copyright         (C) Copyright Endurosat
*
*                    Contents and presentations are protected world-wide.
*                    Any kind of using, copying etc. is prohibited without prior permission.
*                    All rights - incl. industrial property rights - are reserved.
*
* @history
* @revision{         1.0.0  , 2019.06.24, author Vassil Milev, Initial revision }
* @endhistory
********************************************************************************************
*/

/*
*********************************************************************************************
* INCLUDES
*********************************************************************************************
*/
#include "EEPROM_Emul.h"
#include "es_crc32.h"
#include "stm32f7xx_hal_conf.h"
#include "string.h"
#include "BootLdr.h"

/*
*********************************************************************************************
* INTERNAL DEFINES
*********************************************************************************************
*/
#define EEPROM_SIZE	                            (1*(FLASH_BLOCK_SIZE))            /* One block of Flash memory = 128kB */
#define EEPROM_EMUL_FLASH_START_ADDRESS         (FLASH_MAX_ADDR - 2 * FLASH_BLOCK_SIZE)    /* Use the sector before last from the flash as a EEPROM */
#define EEPROM_EMUL_FLASH_END_ADDRESS           (EEPROM_EMUL_FLASH_START_ADDRESS + EEPROM_SIZE - 1U)
#define EEPROM_EMUL_PATTERN_VALUE				(0x55AA4321)                      /* A random magic number */

#define EEPROM_EMUL_CODE_VERSION				(1)                               /* The version EEPROM emulation structure have to match with that in the EEPROM data to validate the data */
/*
*********************************************************************************************
* INTERNAL TYPES DEFINITION
*********************************************************************************************
*/
/* No Internal types definition */

/*
*********************************************************************************************
* EXTERNAL VARIABLES DEFINITION
*********************************************************************************************
*/
EEPROM_INFO_Struct * EEPROM_emul_pDataInfo;    /* Pointer to the Emulated EEPROM where the last data has been saved */
EEPROM_INFO_Struct   EEPROM_emul_DataTemp;     /* Keeps data temporally. It can be changed quick but it will need to be written into the flash memory */
/*
*********************************************************************************************
* INTERNAL (STATIC) VARIABLES DEFINITION/DECLARATION
*********************************************************************************************
*/
/* No Internal variables definition/declaration */

/*
*********************************************************************************************
* INTERNAL (STATIC) ROUTINES DECLARATION
*********************************************************************************************
*/
static void EEPROM_emul_calc_CRC(EEPROM_INFO_Struct * FlashInfo,uint32_t * CRC_val);
static EEPROM_INFO_Struct * EEPROM_Emul_Search_NextFreePosition(void);
static EEPROM_INFO_Struct * EEPROM_Emul_Search_LastData(void);
static void EEPROM_Emul_EraseSection(void);

/*
*********************************************************************************************
* EXTERNAL (NONE STATIC) ROUTINES DEFINITION
*********************************************************************************************
*/
/*!
*********************************************************************************************
* @brief Init routine for the EEPROM Emulation component
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void EEPROM_Emul_Init(void)
{
    if (sizeof(EEPROM_INFO_Struct) != EEPROM_SECTOR_SIZE)
    {
        //Please fix the size of the "EEPROM_INFO_Struct" changing the nubmer of the reserved bytes
        Error_Handler();
    }
	/* Find last written data */
    EEPROM_emul_pDataInfo = EEPROM_Emul_Search_LastData();
	if( EEPROM_emul_pDataInfo == NULL )
	{
		/* clean up all counters and set proper values */
		memset(&EEPROM_emul_DataTemp, 0, sizeof(EEPROM_emul_DataTemp));
		EEPROM_emul_DataTemp.Pattern = EEPROM_EMUL_PATTERN_VALUE;
		EEPROM_emul_DataTemp.nvmVersion = EEPROM_EMUL_CODE_VERSION;
		
		EEPROM_emul_pDataInfo = (EEPROM_INFO_Struct *)EEPROM_EMUL_FLASH_START_ADDRESS;
	}else{
		/* copy the data */
        memcpy(&EEPROM_emul_DataTemp, EEPROM_emul_pDataInfo, sizeof(EEPROM_emul_DataTemp));
	}
}

/*!
*********************************************************************************************
* @brief Copy the data buffer "EEPROM_emul_DataTemp" to the flash memory
*********************************************************************************************
* @param[input]      none
* @param[output]     none
* @return            none
* @note              none
*********************************************************************************************
*/
void EEPROM_Emul_SyncInfo(void)
{
	EEPROM_INFO_Struct * DataPointer;
	uint32_t CRC_calc = 0;

    HAL_StatusTypeDef res = HAL_OK;
    uint32_t TypeProgram;
    uint64_t u64Data;

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

	DataPointer = EEPROM_Emul_Search_NextFreePosition();

	// no more space in the allocated EEPROM section
	if (DataPointer == NULL)
	{
	    EEPROM_Emul_EraseSection();

		DataPointer = (EEPROM_INFO_Struct *) EEPROM_EMUL_FLASH_START_ADDRESS;
	}
	
	// Set service data
	EEPROM_emul_DataTemp.Pattern = EEPROM_EMUL_PATTERN_VALUE;

	// Set a new CRC calculated over the data
	EEPROM_emul_calc_CRC(&EEPROM_emul_DataTemp,&CRC_calc);
	EEPROM_emul_DataTemp.DataCRC = CRC_calc;
	
	TypeProgram = FLASH_TYPEPROGRAM_BYTE;

	//Clear all error bits
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
	res = HAL_FLASH_Unlock();

	for (uint32_t i = 0; i < sizeof(EEPROM_INFO_Struct)/(TypeProgram+1); i += TypeProgram+1)
	{
	    u64Data = *((uint8_t *)(((uint32_t)&EEPROM_emul_DataTemp)+i));

	    // Write the flash memory
	    res = HAL_FLASH_Program(TypeProgram, ((uint32_t)DataPointer) + i, u64Data);
	    if (res != HAL_OK)
	    {
#ifdef DEBUG_ENABLED
	        uint32_t u32Error = HAL_FLASH_GetError();
	        (void) u32Error;

            Error_Handler();
#endif
	        break;
	    }
	}

	// Disable the write access of the Flash memory
	HAL_FLASH_Lock();

	// Copy the position of the last written block of memory
	EEPROM_emul_pDataInfo = DataPointer;
}




/*
*********************************************************************************************
* INTERNAL (STATIC) ROUTINES DEFINITION
*********************************************************************************************
*/

static void EEPROM_Emul_EraseSection(void)
{
    uint32_t se;
    FLASH_EraseInitTypeDef fes;
    HAL_StatusTypeDef eraseState;

    eraseState = HAL_FLASH_Unlock();

    if (eraseState == HAL_OK)
    {
        //Erase the last block Flash memory
        fes.TypeErase = FLASH_TYPEERASE_SECTORS;
        fes.NbSectors = 1;
        fes.Sector =  EEPROM_EMUL_USED_SECTOR;
        fes.VoltageRange = FLASH_VOLTAGE_RANGE_3;

        if (HAL_OK != HAL_FLASHEx_Erase(&fes, &se))
        {
    #ifdef DEBUG_ENABLED
            uint32_t u32Error = HAL_FLASH_GetError();
            (void) u32Error;

            Error_Handler();
    #endif
        }
    }

    HAL_FLASH_Lock();
}

/*!
*********************************************************************************************
* @brief Calculated the CRC over a buffer from type EEPROM_INFO_Struct
*********************************************************************************************
* @param[input]      FlashInfo  - data block to calculate CRC over it
* @param[output]     CRC_val    - calculated CRC
* @return            none
* @note              none
*********************************************************************************************
*/
static void EEPROM_emul_calc_CRC(EEPROM_INFO_Struct * FlashInfo,uint32_t * CRC_val)
{
    // Check if the pointer points to valid memory
    if(
        (( (uint32_t)FlashInfo >= EEPROM_EMUL_FLASH_START_ADDRESS )&&((uint32_t)FlashInfo <= (EEPROM_EMUL_FLASH_END_ADDRESS - sizeof(EEPROM_INFO_Struct)))) ||   //Check for the flash region
        (( (uint32_t)FlashInfo >= RAM_MIN_ADDRESS )&&(((uint32_t)FlashInfo + sizeof(EEPROM_INFO_Struct)) < (RAM_MAX_ADDRESS)))                                                //Check for the RAM region
      )
    {
        if( CRC_val != NULL )
        {
            // Write the calculated CRC to the pointed memory
            *CRC_val = crc32(0, (BYTE*)FlashInfo, sizeof(EEPROM_INFO_Struct)-sizeof(FlashInfo->DataCRC));
        }else{
#ifdef DEBUG_ENABLED
            Error_Handler();
#endif
        }
    }else{
#ifdef DEBUG_ENABLED
            Error_Handler();
#endif
    }
}

/*!
*********************************************************************************************
* @brief Search in the flash memory (EEPROM emulated memory) to find free space for the next writing
*********************************************************************************************
* @param[input]      none
* @param[output]     EEPROM_INFO_Struct    - pointer to the last valid data. If NULL is return, means the data is not data at all or it is not consistent.
* @return            none
* @note              none
*********************************************************************************************
*/
static EEPROM_INFO_Struct * EEPROM_Emul_Search_NextFreePosition(void)
{
    uint16_t i;
    int16_t index = 0;
    EEPROM_INFO_Struct * DataPointer = EEPROM_emul_pDataInfo;    // Set the pointer to the beginning of the emulated EEPROM

    do{
        for( i = 0; i < sizeof(EEPROM_INFO_Struct); i ++ )
        {
            //Check if all bytes are erased
            if( (*(uint8_t*)((uint32_t)&DataPointer[index]+i)) != 0xFF)
            {
                break;
            }
        }

        if( i < sizeof(EEPROM_INFO_Struct) )
        {
            // Some bytes are already written
            index ++;
        }else{
            // All bytes with the size of the structure are erased
            return &DataPointer[index];
        }

     //until the end of the Flash memory
    }while( ( (uint32_t)&DataPointer[index] + sizeof(EEPROM_INFO_Struct)) <= EEPROM_EMUL_FLASH_END_ADDRESS );

    // Insufficient free memory
    return NULL;
}

/*!
*********************************************************************************************
* @brief Search in the flash memory (EEPROM emulated memory) to find where is the last written data
*********************************************************************************************
* @param[input]      none
* @param[output]     EEPROM_INFO_Struct    - pointer to the beginning of the position the found data. If NULL is return, means the data is not consistent or there is not data.
* @return            none
* @note              none
*********************************************************************************************
*/
static EEPROM_INFO_Struct * EEPROM_Emul_Search_LastData(void)
{
    // Variable for the calculated CRC
    uint32_t CRC_calc = 0;
    // Set index to the last possible
    int16_t index = (EEPROM_SIZE / sizeof(EEPROM_INFO_Struct)) - 1U;
    // Set the pointer to the beginning of the emulated EEPROM
    EEPROM_INFO_Struct * DataPointer = (EEPROM_INFO_Struct *) EEPROM_EMUL_FLASH_START_ADDRESS;

    do{
        if( DataPointer[index].Pattern == EEPROM_EMUL_PATTERN_VALUE )   // Validate a known cell of memory for a quick search
        {
            if(DataPointer[index].nvmVersion == EEPROM_EMUL_CODE_VERSION)
            {
                // Calculate the CRC of the structure
                EEPROM_emul_calc_CRC(&DataPointer[index],&CRC_calc);

                // Validate the whole structure
                if(CRC_calc == DataPointer[index].DataCRC)
                {
                    return &DataPointer[index];
                }
            }
        }

        index --; // Switch to previous section
    }while( index >= 0 );

    // A valid data structure is not found
    return NULL;
}


/* **************************************************************************************** */
