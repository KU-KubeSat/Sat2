/*!
*********************************************************************************************
* @file Camera.c
* @brief Manage camera OV5640
*********************************************************************************************
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
*********************************************************************************************
*/

/*
*********************************************************************************************
* INCLUDES
*********************************************************************************************
*/
#include "stm32f7xx_hal.h"
#include "main.h"
#include "MCU_Init.h"
#include "panels.h"
#include "Camera.h"
#include "ov5640_func.h"
#include "cmsis_os.h"
#include "SdMngr.h"
#include "MX_I2C.h"
#include "es_exeh.h"

/*
*********************************************************************************************
* INTERNAL DEFINES
*********************************************************************************************
*/
#define pgm_read_word(x)        ( ((*((unsigned char *)x + 1)) << 8) + (*((unsigned char *)x)))
#define ARDUCAM_SPI_HANDLER     (hspi5)
#define PIC_BUFF_LEN            512

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
status_t comstat = SEN_SUCCESS;
uint8_t  CamCtrl = 0x05;

/*
*********************************************************************************************
* INTERNAL (STATIC) VARIABLES DEFINITION/DECLARATION 
*********************************************************************************************
*/
static FIL df;
static uint8_t MakePicture;
static char fline[PIC_BUFF_LEN];
static FRESULT fd_result;
static uint8_t CameraReady;

/*
*********************************************************************************************
* INTERNAL (STATIC) ROUTINES DECLARATION
*********************************************************************************************
*/
/* No Internal routines declaration */

/*
*********************************************************************************************
* EXTERNAL (NONE STATIC) ROUTINES DEFINITION
*********************************************************************************************
*/
status_t CamSetup(void)
{
  status_t res = SEN_SUCCESS;
  CamWriteSPI(ARDUCHIP_TEST1, 0x55);
  if (CamReadSPI(ARDUCHIP_TEST1) != 0x55)
  {
    fprintf(COMM, "Camera over SPI initialisation failed \r");
    res = SEN_ERROR;
  }
  CamWriteSPI(ARDUCHIP_MODE, MCU2LCD_MODE);

  HAL_StatusTypeDef I2C_retStat = HAL_ERROR;
  I2C_retStat = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);
  if(I2C_retStat == HAL_OK)
  {

      if (CamReadI2C(OV5640_CHIPID_HIGH)!=0x56 || CamReadI2C(OV5640_CHIPID_LOW)!=0x42)  //VMI_TODO_6UPLATFORM -> changed from 0x40 to 0x42 (the camera is a bit different, so some registers are changed)
      {
        fprintf(COMM, "Camera over I2C initialisation failed \r");
        res = SEN_ERROR;
      }

      MX_I2C_Release(MX_I2C_BUS_SYSTEM);
  }else{
      EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_ARDU_CAM_COMM, eEXEH_ARDU_CAM_EXCEPTION_ID_COMM_ERROR, __LINE__);
  }

  return res;
}

void CamInit(uint8_t Ctrl)
{
  osDelay(100);

  HAL_StatusTypeDef I2C_retStat = HAL_ERROR;
  I2C_retStat = MX_I2C_Take(MX_I2C_BUS_SYSTEM, 50);
  if(I2C_retStat == HAL_OK)
  {

      if ((Ctrl>>7) == JPG)
      {
        CamWriteI2C(0x3008, 0x82);
        osDelay(500);
        CamWriteRegsI2C(OV5640YUV_Sensor_Dvp_Init);
        CamWriteRegsI2C(OV5640_JPEG_QSXGA);
        OV5640_set_JPEG_size((Ctrl>>4)&0x07);
        CamWriteI2C(0x4407, 0x04);
      }
      else
      {
        CamWriteI2C(0x3008, 0x82);
        osDelay(500);
        CamWriteRegsI2C(OV5640YUV_Sensor_Dvp_Init);
        CamWriteRegsI2C(OV5640_RGB_QVGA);
      }

      MX_I2C_Release(MX_I2C_BUS_SYSTEM);
  }else{
      EXEH_vException(eEXEHSeverity_Error, eEXEHModuleID_ARDU_CAM_COMM, eEXEH_ARDU_CAM_EXCEPTION_ID_COMM_ERROR, __LINE__);
  }

  osDelay(100);
  CamWriteSPI(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
}

uint32_t CamReadFifoLen(void)
{
  uint32_t len1, len2, len3, length = 0;

  len1 = CamReadSPI(FIFO_SIZE1);
  osDelay(10);   //VMI_TODO_6UPLATFORM -> what time ?
  len2 = CamReadSPI(FIFO_SIZE2);
  osDelay(10);
  len3 = CamReadSPI(FIFO_SIZE3);
  length = ((len3 << 16) | (len2 << 8) | len1) & FIFO_LEN_MASK;

  return length;
}

uint8_t CamWriteRegsI2C(const struct sensor_reg reglist[])
{
  uint16_t reg_addr = 0;
  uint16_t reg_val = 0;
  const struct sensor_reg *next = reglist;

  while ((reg_addr != 0xffff) || (reg_val != 0xff))
  {
    reg_addr = pgm_read_word(&next->reg);
    reg_val = pgm_read_word(&next->val);
    if (!CamWriteI2C(reg_addr, reg_val))
      return 0;
    next++;
  }

  return 1;
}

uint8_t CamReadI2C(uint16_t Addr)
{
  uint8_t data;

  if (HAL_OK != HAL_I2C_Mem_Read(&SYSTEM_I2C_HANDLER, CAM_ADDR, Addr, 2, &data, 1, 10))
  	comstat = SEN_ERROR;

  return data;
}

uint8_t CamWriteI2C(uint16_t Addr, uint8_t Val)
{
  uint8_t i = 0, data = Val;

  while (HAL_OK != HAL_I2C_Mem_Write(&SYSTEM_I2C_HANDLER, CAM_ADDR, Addr, 2, &data, 1, 10))
  {
    i++;
    if (i > 250)
    {
      comstat = SEN_ERROR;
      return 0;
    }
  }
  comstat = SEN_SUCCESS;

  return 1;
}

void CamBurstReadSPI(uint8_t *buff, uint16_t size)
{
  uint8_t txdata = 0x3C & 0x7F; // Provide CMD[7]=0 for read operation

  HAL_GPIO_WritePin(ARDU_CAM_CS_GPIO_Port, ARDU_CAM_CS_Pin, GPIO_PIN_RESET);
  if (HAL_OK != HAL_SPI_TransmitReceive(&ARDUCAM_SPI_HANDLER, &txdata, buff, 1, 10))
  {
  	comstat = SEN_ERROR;
  }
  if (HAL_OK != HAL_SPI_Receive(&ARDUCAM_SPI_HANDLER, buff, size, 10))
  {
  	comstat = SEN_ERROR;
  }
  HAL_GPIO_WritePin(ARDU_CAM_CS_GPIO_Port, ARDU_CAM_CS_Pin, GPIO_PIN_SET);
}

uint8_t CamReadSPI(uint8_t Addr)
{
  uint8_t data = Addr & 0x7F; // Provide CMD[7]=0 for read operation

  HAL_GPIO_WritePin(ARDU_CAM_CS_GPIO_Port, ARDU_CAM_CS_Pin, GPIO_PIN_RESET);
  if (HAL_OK != HAL_SPI_Transmit(&ARDUCAM_SPI_HANDLER, &data, 1, 10))
  {
   comstat = SEN_ERROR;
  }
  if (HAL_OK != HAL_SPI_Receive(&ARDUCAM_SPI_HANDLER, &data, 1, 10))
  {
   comstat = SEN_ERROR;
  }
  HAL_GPIO_WritePin(ARDU_CAM_CS_GPIO_Port, ARDU_CAM_CS_Pin, GPIO_PIN_SET);

  return data;
}

void CamWriteSPI(uint8_t Addr, uint8_t Data)
{
  uint8_t txdata[2];

  txdata[0] = Addr | 0x80; // Provide CMD[7]=1 for write operation
  txdata[1] = Data; // Register content to be written


  HAL_GPIO_WritePin(ARDU_CAM_CS_GPIO_Port, ARDU_CAM_CS_Pin, GPIO_PIN_RESET);

  if (HAL_OK != HAL_SPI_Transmit(&ARDUCAM_SPI_HANDLER, txdata, 2, 10))
  {
   comstat = SEN_ERROR;
  }
  HAL_GPIO_WritePin(ARDU_CAM_CS_GPIO_Port, ARDU_CAM_CS_Pin, GPIO_PIN_SET);
}


int CamCapture(uint8_t Ctrl)
{
  CamInit(Ctrl);
  if (comstat != SEN_SUCCESS) return -1;
  osDelay(1000);
  // Flush the FIFO
  CamWriteSPI(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
  osDelay(100);//VMI_TODO_6UPLATFORM - What time
  // Clear the capture done flag
  CamWriteSPI(ARDUCHIP_FIFO, FIFO_CLEAR_MASK|FIFO_RDPTR_RST_MASK|FIFO_WRPTR_RST_MASK);
  osDelay(100);//VMI_TODO_6UPLATFORM - What time
  // Start capture
  CamWriteSPI(ARDUCHIP_FIFO, FIFO_START_MASK);
  osDelay(100);//VMI_TODO_6UPLATFORM - What time

  for (int i = 0; i < (Ctrl & 0x0F)*10; i++)
  {
    if (!(CamReadSPI(ARDUCHIP_TRIG) & CAP_DONE_MASK))
      osDelay(100);
    else
      return CamReadFifoLen();
  }
  return -1;
}





void InitCameraModule(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_GPIO_WritePin(ARDU_CAM_CS_GPIO_Port, ARDU_CAM_CS_Pin, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = ARDU_CAM_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ARDU_CAM_CS_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(EN_CAM_GPIO_Port, EN_CAM_Pin, GPIO_PIN_SET);  //HAL_GPIO_WritePin(OBC_OUT1_GPIO_Port, OBC_OUT1_Pin, GPIO_PIN_SET);

    MX_SPI5_Init();

    uint8_t txdata;

    (void)HAL_SPI_Transmit(&hspi5, &txdata, 1, 10);


  osDelay(200);
  if (CamSetup() == SEN_SUCCESS)
    CameraReady = 1;
  else
    CameraReady = 0;

  HAL_GPIO_WritePin(EN_CAM_GPIO_Port, EN_CAM_Pin, GPIO_PIN_RESET);  //HAL_GPIO_WritePin(OBC_OUT1_GPIO_Port, OBC_OUT1_Pin, GPIO_PIN_RESET);

}

uint8_t Camm_IsCammReady(void)
{
    return  CameraReady;
}

//VMI_TODO_6UPLATFORM add Deinit

void Camm_MakePicture(void)
{
    MakePicture = 1;
}

uint8_t Camm_GetStatus(void)
{
    return MakePicture;
}

void AppProcessCameraData(void)
{
    if (MakePicture)
    {
      comstat = SEN_SUCCESS;

      HAL_GPIO_WritePin(EN_CAM_GPIO_Port, EN_CAM_Pin, GPIO_PIN_SET);//HAL_GPIO_WritePin(GPIOG, OBC_OUT1_Pin, GPIO_PIN_SET);

      osDelay(200);

      if (CamSetup() != SEN_SUCCESS) fprintf(COMM, "SetupERR=%X\r", comstat);
      int len, i;

      len = CamCapture(CamCtrl);
      if (len > 0)
      {
        HAL_RTC_GetTime(&hrtc, &sTime, CALENDAR_FORMAT);
        HAL_RTC_GetDate(&hrtc, &sDate, CALENDAR_FORMAT);
        sprintf(fline, "0://%02d%02d%02d%02d.jpg", sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds);
        if (df.obj.fs) SdMngr_f_close(&df);
        fd_result = SdMngr_f_open(&df, fline, FA_WRITE | FA_CREATE_ALWAYS);
        if (fd_result == FR_OK)
        {
          fprintf(COMM, "OK+%02X %s %u", CamCtrl, fline, len);
          UINT bw;
          for (i = 0; i < len; i += PIC_BUFF_LEN)
          {
            if (i+PIC_BUFF_LEN < len)
              bw = PIC_BUFF_LEN;
            else
              bw = len - i;
            CamBurstReadSPI((uint8_t*)fline, bw);
            if (FR_OK != (fd_result = SdMngr_f_write(&df, fline, bw, &bw))) break;
          }
          if (comstat != SEN_SUCCESS) fprintf(COMM, "*%X*", comstat);
          osDelay(100);
          SdMngr_f_close(&df);
        }
        if (fd_result == FR_OK)
          fprintf(COMM, " DONE\r");
        else
          fprintf(COMM, "ERR+FILE\r");
      }
      else
        fprintf(COMM, "ERR+CAPTURE %02X\r", CamCtrl);
      // Clear the capture done flag
      comstat = SEN_SUCCESS;
      CamWriteSPI(ARDUCHIP_FIFO, FIFO_CLEAR_MASK|FIFO_RDPTR_RST_MASK|FIFO_WRPTR_RST_MASK);
      CamInit(BMP);
      MakePicture = 0;

      HAL_GPIO_WritePin(EN_CAM_GPIO_Port, EN_CAM_Pin, GPIO_PIN_RESET); //HAL_GPIO_WritePin(GPIOG, OBC_OUT1_Pin, GPIO_PIN_RESET);

    }
}

/*
*********************************************************************************************
* INTERNAL (STATIC) ROUTINES DEFINITION
*********************************************************************************************
*/
/* No Internal routines definition */

/* ******************************************************************************************* */
