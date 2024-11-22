/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#include "fatfs.h"

uint8_t retSD;    /* Return value for SD */
char SDPath[4];   /* SD logical drive path */
FATFS SDFatFS;    /* File system object for SD logical drive */
FIL SDFile;       /* File object for SD */

uint8_t retSD2;    /* Return value for SD */
char SD2Path[4];   /* SD logical drive path */
FATFS SD2FatFS;    /* File system object for SD logical drive */
FIL SD2File;       /* File object for SD */

//uint8_t retUSER;    /* Return value for USER */
//char USERPath[4];   /* USER logical drive path */
//FATFS USERFatFS;    /* File system object for USER logical drive */
//FIL USERFile;       /* File object for USER */

/* USER CODE BEGIN Variables */
#include "MCU_Init.h"
/* USER CODE END Variables */    

void MX_FATFS_Init(uint8_t cards)
{
    if( cards & 0x01 )
    {
      /*## FatFS: Link the SD driver ###########################*/
      retSD = FATFS_LinkDriver(0, &SD_Driver, SDPath);
    }

    if( cards & 0x02 )
    {
      /*## FatFS: Link the USER driver ###########################*/
      retSD2 = FATFS_LinkDriver(1, &SD2_Driver, SD2Path);
    }
  /* USER CODE BEGIN Init */
  /* additional user code for init */     
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC 
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
    DWORD FatTime = 0;
    WORD  Date = 0;
    WORD  Time = 0;

    MX_RTC_ReloadShadowRegs();

    if (HAL_OK == HAL_RTC_GetDate(&hrtc, &sDate, CALENDAR_FORMAT))
    {
        Date = ((sDate.Year + 20) << 9) |
                (sDate.Month << 5) |
                sDate.Date;
    }

    if (HAL_OK == HAL_RTC_GetTime(&hrtc, &sTime, CALENDAR_FORMAT))
    {
       Time =  (sTime.Hours << 11) |
               (sTime.Minutes << 5) |
               (sTime.Seconds >> 1);
    }

  return FatTime = Date << 16 | Time;
  /* USER CODE END get_fattime */  
}

/* USER CODE BEGIN BeforeCallBacksSection */
/* can be used to modify previous code / undefine following code / add code */
/* USER CODE END BeforeCallBacksSection */
/**
  * @brief SD Abort callbacks
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd)
{
  if(hsd == &sd_driver_card_handler)
  {
      BSP_SD_AbortCallback();
  }else if(hsd == &sd2_driver_card_handler){
      BSP_SD2_AbortCallback();
  }
}

/**
  * @brief Tx Transfer completed callback
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
    if(hsd == &sd_driver_card_handler)
    {
        BSP_SD_WriteCpltCallback();
    }else if(hsd == &sd2_driver_card_handler){
        BSP_SD2_WriteCpltCallback();
    }
}

/**
  * @brief Rx Transfer completed callback
  * @param hsd: SD handle
  * @retval None
  */
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
    if(hsd == &sd_driver_card_handler)
    {
        BSP_SD_ReadCpltCallback();
    }else if(hsd == &sd2_driver_card_handler){
        BSP_SD2_ReadCpltCallback();
    }
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
