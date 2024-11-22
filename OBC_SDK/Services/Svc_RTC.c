/*!
 ********************************************************************************************
 * @file Svc_RTC.c
 * @brief Service for RTC temperature calibration
 ********************************************************************************************
 * @author            Vassil Milev
 * @version           1.0.0
 * @date              2019.05.15
 *
 * @copyright         (C) Copyright Endurosat
 *
 *                    Contents and presentations are protected world-wide.
 *                    Any kind of using, copying etc. is prohibited without prior permission.
 *                    All rights - incl. industrial property rights - are reserved.
 *
 * @history
 * @revision{         1.0.0  , 2019.05.15, author Vassil Milev, Initial revision }
 * @endhistory
 ********************************************************************************************
 */

/*
 *********************************************************************************************
 * INCLUDES
 *********************************************************************************************
 */
#include "Svc_RTC.h"
#include "main.h"
#include "cmsis_os.h"
#include "es_crc32.h"
#include "EEPROM_Emul.h"
#include <string.h>
#include "AppTasks.h"

/*
 *********************************************************************************************
 * INTERNAL DEFINES
 *********************************************************************************************
 */
#define AVG_SLOPE 	(float)2.5		/* Value that express the slope from the characteristic of the ADC valur from the sensor depending of the temperature */
#define AVRG_PERIOD (100)			/* time between two measurement */

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
/**
 * @brief Keeps counters for various reset reasons
 */
volatile boot_struct* BootData = (boot_struct*)RTC_INIT_ADDRESS;

/*
 *********************************************************************************************
 * INTERNAL (STATIC) VARIABLES DEFINITION/DECLARATION
 *********************************************************************************************
 */
static uint32_t Svc_RTC_LastCalibTime;

#define CRC_CALCULATION_EN

#ifdef  CRC_CALCULATION_EN

#else
/**
* @brief That is a look up table that express the calibration constant of the RTC depending of the MCU temperature
*/
static int16_t RTC_LUP_TemperatureCompensation[] =
{/* temperature */ /* Compensation value */
        /* -55 */ -115 ,
        /* -54 */ -110 ,
        /* -53 */ -105 ,
        /* -52 */ -101 ,
        /* -51 */ -96 ,
        /* -50 */ -91 ,
        /* -49 */ -87 ,
        /* -48 */ -83 ,
        /* -47 */ -78 ,
        /* -46 */ -74 ,
        /* -45 */ -70 ,
        /* -44 */ -65 ,
        /* -43 */ -61 ,
        /* -42 */ -57 ,
        /* -41 */ -53 ,
        /* -40 */ -49 ,
        /* -39 */ -45 ,
        /* -38 */ -41 ,
        /* -37 */ -37 ,
        /* -36 */ -33 ,
        /* -35 */ -30 ,
        /* -34 */ -26 ,
        /* -33 */ -22 ,
        /* -32 */ -19 ,
        /* -31 */ -15 ,
        /* -30 */ -12 ,
        /* -29 */ -8 ,
        /* -28 */ -5 ,
        /* -27 */ -2 ,
        /* -26 */ 2 ,
        /* -25 */ 5 ,
        /* -24 */ 8 ,
        /* -23 */ 11 ,
        /* -22 */ 14 ,
        /* -21 */ 17 ,
        /* -20 */ 20 ,
        /* -19 */ 23 ,
        /* -18 */ 26 ,
        /* -17 */ 29 ,
        /* -16 */ 31 ,
        /* -15 */ 34 ,
        /* -14 */ 37 ,
        /* -13 */ 39 ,
        /* -12 */ 42 ,
        /* -11 */ 44 ,
        /* -10 */ 46 ,
        /* -9  */ 49 ,
        /* -8  */ 51 ,
        /* -7  */ 53 ,
        /* -6  */ 56 ,
        /* -5  */ 58 ,
        /* -4  */ 60 ,
        /* -3  */ 62 ,
        /* -2  */ 64 ,
        /* -1  */ 66 ,
        /* 0   */ 68 ,
        /* 1   */ 69 ,
        /* 2   */ 71 ,
        /* 3   */ 73 ,
        /* 4   */ 74 ,
        /* 5   */ 76 ,
        /* 6   */ 78 ,
        /* 7   */ 79 ,
        /* 8   */ 81 ,
        /* 9   */ 82 ,
        /* 10  */ 83 ,
        /* 11  */ 85 ,
        /* 12  */ 86 ,
        /* 13  */ 87 ,
        /* 14  */ 88 ,
        /* 15  */ 89 ,
        /* 16  */ 90 ,
        /* 17  */ 91 ,
        /* 18  */ 92 ,
        /* 19  */ 93 ,
        /* 20  */ 94 ,
        /* 21  */ 94 ,
        /* 22  */ 95 ,
        /* 23  */ 96 ,
        /* 24  */ 96 ,
        /* 25  */ 97 ,
        /* 26  */ 97 ,
        /* 27  */ 98 ,
        /* 28  */ 98 ,
        /* 29  */ 98 ,
        /* 30  */ 99 ,
        /* 31  */ 99 ,
        /* 32  */ 99 ,
        /* 33  */ 99 ,
        /* 34  */ 99 ,
        /* 35  */ 99 ,
        /* 36  */ 99 ,
        /* 37  */ 99 ,
        /* 38  */ 99 ,
        /* 39  */ 98 ,
        /* 40  */ 98 ,
        /* 41  */ 98 ,
        /* 42  */ 97 ,
        /* 43  */ 97 ,
        /* 44  */ 96 ,
        /* 45  */ 96 ,
        /* 46  */ 95 ,
        /* 47  */ 95 ,
        /* 48  */ 94 ,
        /* 49  */ 93 ,
        /* 50  */ 92 ,
        /* 51  */ 92 ,
        /* 52  */ 91 ,
        /* 53  */ 90 ,
        /* 54  */ 89 ,
        /* 55  */ 87 ,
        /* 56  */ 86 ,
        /* 57  */ 85 ,
        /* 58  */ 84 ,
        /* 59  */ 83 ,
        /* 60  */ 81 ,
        /* 61  */ 80 ,
        /* 62  */ 78 ,
        /* 63  */ 77 ,
        /* 64  */ 75 ,
        /* 65  */ 74 ,
        /* 66  */ 72 ,
        /* 67  */ 70 ,
        /* 68  */ 68 ,
        /* 69  */ 67 ,
        /* 70  */ 65 ,
        /* 71  */ 63 ,
        /* 72  */ 61 ,
        /* 73  */ 59 ,
        /* 74  */ 57 ,
        /* 75  */ 54 ,
        /* 76  */ 52 ,
        /* 77  */ 50 ,
        /* 78  */ 48 ,
        /* 79  */ 45 ,
        /* 80  */ 43 ,
        /* 81  */ 40 ,
        /* 82  */ 38 ,
        /* 83  */ 35 ,
        /* 84  */ 33 ,
        /* 85  */ 30 ,
        /* 86  */ 27 ,
        /* 87  */ 24 ,
        /* 88  */ 22 ,
        /* 89  */ 19 ,
        /* 90  */ 16 ,
        /* 91  */ 13 ,
        /* 92  */ 10 ,
        /* 93  */ 6 ,
        /* 94  */ 3 ,
        /* 95  */ 0 ,
        /* 96  */ -3 ,
        /* 97  */ -7 ,
        /* 98  */ -10 ,
        /* 99  */ -13 ,
        /* 100 */ -17 ,
        /* 101 */ -21 ,
        /* 102 */ -24 ,
        /* 103 */ -28 ,
        /* 104 */ -32 ,
        /* 105 */ -35 ,
        /* 106 */ -39 ,
        /* 107 */ -43 ,
        /* 108 */ -47 ,
        /* 109 */ -51 ,
        /* 110 */ -55 ,
        /* 111 */ -59 ,
        /* 112 */ -63 ,
        /* 113 */ -67 ,
        /* 114 */ -72 ,
        /* 115 */ -76 ,
        /* 116 */ -80 ,
        /* 117 */ -85 ,
        /* 118 */ -89 ,
        /* 119 */ -94 ,
        /* 120 */ -98 ,
        /* 121 */ -103 ,
        /* 122 */ -108 ,
        /* 123 */ -112 ,
        /* 124 */ -117 ,
        /* 125 */ -122 ,
        /* 126 */ -127 ,
        /* 127 */ -132 ,
        /* 128 */ -137 ,
        /* 129 */ -142 ,
        /* 130 */ -147 ,
        /* 131 */ -152 ,
        /* 132 */ -157 ,
        /* 133 */ -163 ,
        /* 134 */ -168 ,
        /* 135 */ -173
};
#endif  // CRC_CALCULATION_EN

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
/*!
 *********************************************************************************************
 * @brief Calibrates periodicaly the RTC according to the MCU temperature
 *********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              The expected typical error is under 10 seconds/month (2 minutes/year).
 *                    Maximum expected error is about 40 seconds/month     (8 minutes/year).
 *********************************************************************************************
 */
void Svc_RtcTask(void)
{
    float CPU_temperature;
    uint32_t PlusPulses;
    int16_t Compensation;

    uint32_t upSeconds = ServicesTask_UptimeSeconds();

    if ((upSeconds == 0) || (upSeconds - Svc_RTC_LastCalibTime) >= RTC_CALIBRATION_PERIOD)
    {
        Svc_RTC_LastCalibTime = upSeconds;

#ifdef  CRC_CALCULATION_EN
        float Crystal_temperature;
        int16_t Accuracy;
#else
        int16_t     temperature;
#endif

        CPU_temperature = GetCpuAverageTemperature();

#ifdef  CRC_CALCULATION_EN
        /* Empirically found formula for the Endurosat OBC layout */
        Crystal_temperature = (CPU_temperature - 3) / 1.25;
        /* The calculations are done according to the datasheet "Application note AN2604" - "STM32F101xx and STM32F103xx RTC calibration" */
        Accuracy = 94 + (-0.04 * (Crystal_temperature - 25) * (Crystal_temperature - 25)); /* Acc = AccT0 + k * (T - To)^2 */
        Compensation = Accuracy / 0.954 + 0.5; /* +0.5 to round up *//* Compensation = Accuracy * k */
#else
        temperature = (int16_t)(CPU_temperature + 0.5);     /* round up and convert to integer number */
        if( temperature < -55 ) temperature = -55;          /* do not exceed the minimum temperature */
        if( temperature > 135 ) temperature = 135;          /* do not exceed the maximum temperature */

        Compensation = RTC_LUP_TemperatureCompensation[temperature+55];
#endif

        if (Compensation < 0)
        {
            PlusPulses = RTC_SMOOTHCALIB_PLUSPULSES_SET; /* that flag adds 512ppm */
            Compensation = 512 + Compensation; /* remove the rest to achieve the minus value i.e. faster clock */
        }
        else
        {
            PlusPulses = RTC_SMOOTHCALIB_PLUSPULSES_RESET;
        }

        HAL_RTCEx_SetSmoothCalib(&hrtc, RTC_SMOOTHCALIB_PERIOD_32SEC, PlusPulses, Compensation);
    }
    else
    {
        /*
         float CPU_temperature;
         GetCpuTemperature(&CPU_temperature);
         fprintf(SYSCON, "Temperature: %.1f === Up seconds: %lu, %lu, %lu\r", CPU_temperature, upSeconds, GetObcUptimeSeconds(), (int)xTaskGetTickCount()/1000);
         */
    }
}

/*!
 *********************************************************************************************
 * @brief Reads the ADC value from the temperature sensor of the MCU and converts the value to temperature
 *********************************************************************************************
 * @param[input]      none
 * @param[output]     temperature - overwritten by the measured temperature
 * @return            SEN_ERROR   - Temperature could not be read
 SEN_SUCCESS - Temperature has been read successfully
 * @note              none
 *********************************************************************************************
 */
status_t GetCpuTemperature(float *temperature)
{
    uint16_t Vsense;
    uint16_t V25_DEGREE;

    ADC_ChannelConfTypeDef sConfig;

    /* Initiate reading the ADC value from the temperature sensor */
    sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        return SEN_ERROR;
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
        return SEN_ERROR;

    /* wait until the value has been read */
    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
        return SEN_ERROR;

    /* get the value */
    Vsense = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    /* Read the calibration value for the current MCU */
    //V30_DEGREE = *(uint16_t*)(0x1FFF7A2C);
    V25_DEGREE = (int16_t)(0x3AA);

    /* Convert the ADC value to degrees Celsius */
    *temperature = ((float)Vsense - V25_DEGREE) / (float)AVG_SLOPE + (float)25.0;

    /* Restore sun sensor channel */
    sConfig.Channel = ADC_CHANNEL_3;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    return SEN_SUCCESS;
}

/*!
 *********************************************************************************************
 * @brief Reads the internal temperature sensor
 *********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            temperature - measured temperature
 * @note              none
 *********************************************************************************************
 */
float GetCpuAverageTemperature(void)
{
    float CPU_temperature[4] =
    {
        0,
        0,
        0,
        0
    };
    status_t temp_status;

    /* read four time the temperature */
    temp_status = GetCpuTemperature(&CPU_temperature[0]);
    if (temp_status != SEN_SUCCESS)
    {
        Error_Handler();
    }

    osDelay(100);

    temp_status = GetCpuTemperature(&CPU_temperature[1]);
    if (temp_status != SEN_SUCCESS)
    {
        Error_Handler();
    }

    osDelay(100);

    temp_status = GetCpuTemperature(&CPU_temperature[2]);
    if (temp_status != SEN_SUCCESS)
    {
        Error_Handler();
    }

    osDelay(100);

    temp_status = GetCpuTemperature(&CPU_temperature[3]);
    if (temp_status != SEN_SUCCESS)
    {
        Error_Handler();
    }

    /* calculate the average temperature from the four values */
    CPU_temperature[0] = (CPU_temperature[0] + CPU_temperature[1] + CPU_temperature[2] + CPU_temperature[3]) / 4; /* average of four values */

    return CPU_temperature[0];
}

/*!
 *********************************************************************************************
 * @brief Real-Time Clock - setting boot data CRC32
 *********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            none
 * @note              Detects a reset reason
 *********************************************************************************************
 */
void RTC_SetRTC_BKP_CRC(void)
{
    BootData->BootDataCRC = crc32(0, (BYTE*)BootData, sizeof(boot_struct) - sizeof(BootData->BootDataCRC));
}

/*!
 *********************************************************************************************
 * @brief Initialize the internal temperature channel of the ADC
 *********************************************************************************************
 * @param[input]      none
 * @param[output]     none
 * @return            temperature - measured temperature
 * @note              none
 *********************************************************************************************
 */
void CpuTemprInit(void)
{
    MX_ADC1_Init();
}

bool Svc_Rtc_ClearResetCounter(RstCounterId_t cntrId)
{
    bool res = true;

    switch (cntrId)
    {
        case RSTCOUNTERID_WWD:
        {
            BootData->RST_WWD = 0;
        } break;

        case RSTCOUNTERID_IWD:
        {
            BootData->RST_IWD = 0;
        } break;

        case RSTCOUNTERID_LPR:
        {
            BootData->RST_LPR = 0;
        } break;

        case RSTCOUNTERID_POR:
        {
            BootData->RST_POR = 0;
        } break;

        case RSTCOUNTERID_RSTPIN:
        {
            BootData->RST_RstPin = 0;
        } break;

        case RSTCOUNTERID_BOR:
        {
            BootData->RST_BOR = 0;
        } break;

        case RSTCOUNTERID_HARDFAULT:
        {
            BootData->RST_HardFault = 0;
        } break;

        case RSTCOUNTERID_MEMFAULT:
        {
            BootData->RST_MemFault = 0;
        } break;

        case RSTCOUNTERID_BUSFAULT:
        {
            BootData->RST_BusFault = 0;
        } break;

        case RSTCOUNTERID_USAGEFAULT:
        {
            BootData->RST_UsageFault = 0;
        } break;

        case RSTCOUNTERID_ALL:
        {
            BootData->RST_WWD = 0;
            BootData->RST_IWD = 0;
            BootData->RST_LPR = 0;
            BootData->RST_POR = 0;
            BootData->RST_RstPin = 0;
            BootData->RST_BOR = 0;

            BootData->RST_HardFault = 0; /* HardFault_Handler()  */
            BootData->RST_MemFault = 0; /* MemManage_Handler()  */
            BootData->RST_BusFault = 0; /* BusFault_Handler()   */
            BootData->RST_UsageFault = 0; /* UsageFault_Handler() */
        } break;

        default:
            res = false;
        break;
    }

    if (res)
    {
        RTC_SetRTC_BKP_CRC();

        // Sync the RTC backup registers data with the buffer of the emulated EEPROM memory
        memcpy(&EEPROM_emul_DataTemp.ResetFaults, (void*)BootData, sizeof(boot_struct));

        // Sync EEPROM emulated memory with the buffer "EEPROM_emul_DataTemp"
        EEPROM_Emul_SyncInfo();
    }

    return res;
}

/* Clear all reset flags about the last reset reason */
void RTC_ClearRSTReason(void)
{
    // Clear all the reset flags or else they will remain set during future resets until system power is fully removed.
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

uint8_t RTC_ValidateRTC_Counters(void)
{
    uint8_t IncrementedCounters = 0;
    uint8_t SameCounters = 0;
    char i;

    uint32_t* pBootData = (uint32_t*)&BootData->Mailbox;
    uint32_t* pEEPROM_Emul_Data = &EEPROM_emul_DataTemp.ResetFaults.Mailbox;

    for (i = 0; i < 12; i++)
    {
        if (*pBootData == *pEEPROM_Emul_Data)
        {
            SameCounters++;
        }
        else if (((*pEEPROM_Emul_Data) + 1) == *pBootData)
        {
            IncrementedCounters++;
        }

        pBootData++;
        pEEPROM_Emul_Data++;
    }

    if ((SameCounters + IncrementedCounters) == 12)
    {
        /* Valid */
        return 1;
    }
    else
    {
        /* Invalid */
        return 0;
    }
}

/* validate the RTC backup memory */
uint8_t RTC_ValidateRTC_BKP(void)
{
    DWORD RTC_backup_crc;

    RTC_backup_crc = crc32(0, (BYTE*)BootData, sizeof(boot_struct) - sizeof(BootData->BootDataCRC));

    if (RTC_backup_crc == BootData->BootDataCRC)
    {
        /* Valid */
        return 1;
    }
    else
    {
        /* Invalid */
        return 0;
    }
}

/* count the reset reason */
void RTC_CountRstType(void)
{
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) /* WINDOW_WATCHDOG_RESET */
    {
        BootData->RST_WWD++;
    }

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) /* INDEPENDENT_WATCHDOG_RESET */
    {
        BootData->RST_IWD++;
    }

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) /* LOW_POWER_RESET */
    {
        BootData->RST_LPR++;
    }

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) /* POWER-ON_RESET (POR) / POWER-DOWN_RESET (PDR) */
    {
        BootData->RST_POR++;
    }

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) /* EXTERNAL_RESET_PIN_RESET */
    {
        BootData->RST_RstPin++;
    }

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST)) /* BROWNOUT_RESET (BOR) */
    {
        BootData->RST_BOR++;
    }
}

/*
 *********************************************************************************************
 * INTERNAL (STATIC) ROUTINES DEFINITION
 *********************************************************************************************
 */
/* No Internal routines definition */

/* **************************************************************************************** */
