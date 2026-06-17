#include "sensor.h"
#include "motor.h"
#include <math.h>
#include "llrlpf.h"


#define ADC_MAX        4095.0f
#define VREF           3.3f
#define CSA_GAIN       40.0f
#define SHUNT_RES      0.010f   // <-- change if different
#define ADC2_CURRENT_A_INDEX   0   // IN3
#define ADC2_CURRENT_B_INDEX   1   // IN4
#define INA180A2_GAIN		  50

// Bus voltage divider
#define R_UP           34800.0f
#define R_DOWN         5100.0f
#define VOLTAGE_GAIN_CORRECTION  1.0f

// NTC
#define NTC_R_FIXED    10000.0f
#define NTC_BETA       3950.0f
#define NTC_T0         298.15f    // 25°C in Kelvin
#define NTC_R0         10000.0f


// ---------- LPF INDEX ----------
#define LPF_VOLTAGE_A      0
#define LPF_VOLTAGE_B      1
#define LPF_VOLTAGE_C      2
#define LPF_BUS_VOLTAGE    3
#define LPF_CURRENT_A      4
#define LPF_CURRENT_B      5
#define LPF_CURRENT_C      6
#define LPF_TOTAL_CURRENT  7
#define LPF_TEMPERATURE    8

// ---------- ADC1 LPF CHANNELS ----------
#define LPF_ADC1_0    0   // ADC1 Channel IN4
#define LPF_ADC1_1    1   // ADC1 Channel IN11
#define LPF_ADC1_2    2   // ADC1 Channel IN12
#define LPF_ADC1_3    3   // ADC1 Channel IN15

// ---------- ADC2 LPF CHANNELS ----------
#define LPF_ADC2_0    4   // ADC2 Channel IN3
#define LPF_ADC2_1    5   // ADC2 Channel IN4
#define LPF_ADC2_2    6   // ADC2 Channel IN12 (Virtual N)
#define LPF_ADC2_3    7   // ADC2 Channel IN13
#define LPF_ADC2_4    8   // ADC2 Channel IN17
#define LPF_OFFSET_A  9
#define LPF_OFFSET_B  10


#define VOLTAGE_ALPHA	30
#define CURRENT_ALPHA	30
#define TEMP_ALPHA		10



/* ─── Calibrated Constants  Gain 20 ─────────────────────── */
#define PHASE_A_INTERCEPT   2054.0f
#define PHASE_B_INTERCEPT   2038.0f
#define PHASE_A_INV_SLOPE   4.0039f
#define PHASE_B_INV_SLOPE   4.0039f
#define PHASE_A_CAL_FACTOR	0.91f
#define PHASE_B_CAL_FACTOR	0.91f

/* ─── Calibrated Constants  Gain 40 ─────────────────────── */
//#define PHASE_A_INTERCEPT   2039.4f
//#define PHASE_B_INTERCEPT   2019.0f   // ← 2021.7 থেকে change
//#define PHASE_A_INV_SLOPE   2.0161f   // 1 / 0.4960
//#define PHASE_B_INV_SLOPE   2.0000f   // 1 / 0.5003




void Sensor_Current_Amp_Offset_Measure(void)
{

}




void Sensor_ADC2_DMA_Start(void)
{
    /* 1. Disable DMA first */
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;

    /* 2. Clear DMA flags (important for G4 stability) */
    DMA1->IFCR |= DMA_IFCR_CTCIF2 | DMA_IFCR_CHTIF2 | DMA_IFCR_CGIF2 | DMA_IFCR_CTEIF2;

    /* 3. Setup DMA */
    DMA1_Channel2->CPAR  = (uint32_t)&ADC2->DR;
    DMA1_Channel2->CMAR  = (uint32_t)adc2_buffer;
    DMA1_Channel2->CNDTR = 4;

    DMA1_Channel2->CCR =
          DMA_CCR_MINC
        | DMA_CCR_CIRC
        | DMA_CCR_PSIZE_0   // 16-bit
        | DMA_CCR_MSIZE_0;  // 16-bit

    DMA1_Channel2->CCR |= DMA_CCR_EN;

    /* 4. Enable ADC DMA mode */
    ADC2->CFGR |= (ADC_CFGR_DMAEN | ADC_CFGR_DMACFG);

    /* 5. Proper ADC disable before enable (IMPORTANT for G4) */
    if (ADC2->CR & ADC_CR_ADEN)
    {
        ADC2->CR |= ADC_CR_ADDIS;
        while (ADC2->CR & ADC_CR_ADEN);
    }

    /* 6. Small delay for ADC state settle */
    for (volatile int i = 0; i < 1000; i++);

    /* 7. Enable ADC */
    ADC2->CR |= ADC_CR_ADEN;
    while (!(ADC2->ISR & ADC_ISR_ADRDY));

    /* 8. Start conversion */
    ADC2->CR |= ADC_CR_ADSTART;
}


// ADC debug print function
void Sensor_ADC_Debug_Print(void)
{
    Debug_Add_Log("=== ADC DEBUG ===\r\n");

    // Current sensing (INA240)
    Debug_Add_Log("Curr_A ADC : %ld\r\n", (long)c_asense_adc);
    Debug_Add_Log("Curr_B ADC : %ld\r\n", (long)c_bsense_adc);
    Debug_Add_Log("Curr_C ADC : %ld\r\n", (long)c_csense_adc);

    // Phase voltages
    Debug_Add_Log("Vol_C  ADC : %u\r\n", (uint16_t)adc1_buffer[0]);
    Debug_Add_Log("Vol_B  ADC : %u\r\n", (uint16_t)adc1_buffer[1]);
    Debug_Add_Log("Vol_A  ADC : %u\r\n", (uint16_t)adc2_buffer[2]);

    // System
    Debug_Add_Log("Temp   ADC : %u\r\n", (uint16_t)adc2_buffer[0]);
    Debug_Add_Log("Bus V  ADC : %u\r\n", (uint16_t)adc2_buffer[1]);
    Debug_Add_Log("Tot_I  ADC : %u\r\n", (uint16_t)adc2_buffer[3]);

    Debug_Add_Log("=================\r\n");
}



// ---------- SENSOR LOOP ----------
void Sensor_Main_Loop_Executable(void)
{
//
    // -------- Temperature (scaled ×10) --------
//    float r_ntc = (v_adc1_1 * NTC_R_FIXED) / (VREF - v_adc1_1);
//    float temp_kelvin = 1.0f / ((1.0f/NTC_T0) + (1.0f/NTC_BETA) * logf(r_ntc / NTC_R0));
//    temperature = 10*(temp_kelvin - 273.15f);
}


void Sensor_ADC_Init(void){
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    HAL_Delay(10);
    //Sensor_ADC2_DMA_Start();
//    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buffer, 4);
    /* Regular channel DMA */
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buffer, 2) != HAL_OK)
        Error_Handler();

    Sensor_ADC2_DMA_Start();
    /* Injected channel interrupt — দুটোই start করো */
    HAL_ADCEx_InjectedStart_IT(&hadc1);  // ✅ A, B phase current
    //HAL_ADCEx_InjectedStart_IT(&hadc2);  // ✅ আগে থেকে ছিল

    LPF_Init();
    // ... বাকি LPF config
}





