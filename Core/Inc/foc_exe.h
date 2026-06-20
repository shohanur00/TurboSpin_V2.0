/*
 * foc_isr_example.c
 *
 * TIM1 Update ISR integration for TurboSpin V2.0
 * Copy the relevant parts into your main.c / stm32g4xx_it.c
 *
 * ADC injected sequence:  JDR1 = Phase A,  JDR2 = Phase B
 * PWM channels:           TIM1_CH1 = PhA,  CH2 = PhB,  CH3 = PhC
 */

#include "foc_math.h"
#include "stm32g4xx_hal.h"   /* adjust to your HAL path */

/* ------------------------------------------------------------------ */
/*  Globals                                                             */
/* ------------------------------------------------------------------ */
static foc_state_t g_foc;

/* Set these from your throttle / speed loop */
volatile float g_iq_ref  = 0.0f;   /* A  – torque demand   */
volatile float g_id_ref  = 0.0f;   /* A  – keep 0 for MTPA */
volatile float g_vbus    = 16.8f;  /* V  – 4S LiPo nominal */

/* PWM period = TIM1->ARR, set during TIM1 init for 20 kHz center-aligned */
/* ARR = (168 MHz / (2 × 20 kHz)) – 1  if sysclk = 168 MHz              */
/* STM32G431 runs up to 170 MHz; adjust for your clock config             */
#define PWM_PERIOD  4249U   /* example: 170 MHz / (2×20 kHz) - 1         */

/* ------------------------------------------------------------------ */
/*  Call once from main() after peripheral init                         */
/* ------------------------------------------------------------------ */
void FOC_Init(void)
{
    foc_init(&g_foc, OBS_GAMMA);   /* OBS_GAMMA defined in foc_math.c  */

    /* Start TIM1 in center-aligned PWM mode – enable update interrupt */
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
//    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
//
//    /* Trigger injected ADC conversion on TIM1 TRGO */
//    HAL_ADCEx_InjectedStart(&hadc1);
}

/* ------------------------------------------------------------------ */
/*  TIM1 Update ISR  (20 kHz)                                           */
/* ------------------------------------------------------------------ */
void ADC_INJECTED_ISR_EXE(void)
{
    uint32_t raw_a = ADC1->JDR1;
    uint32_t raw_b = ADC1->JDR2;

    float ia = ADC_TO_AMPS(raw_a);
    float ib = ADC_TO_AMPS(raw_b);

    uint32_t tA, tB, tC;

    foc_run(&g_foc,
            ia, ib,
            g_iq_ref,
            g_id_ref,
            g_vbus,
            PWM_PERIOD,
            &tA, &tB, &tC);

    TIM1->CCR1 = tA;
    TIM1->CCR2 = tB;
    TIM1->CCR3 = tC;
}

/* ------------------------------------------------------------------ */
/*  Optional: slow speed/torque loop  (1 kHz task / RTOS task)         */
/* ------------------------------------------------------------------ */
void FOC_SpeedLoop(float rpm_setpoint)
{
    /* Example: simple P controller on electrical speed               */
    float rpm_now = (g_foc.omega / (2.0f * (float)M_PI)) * 60.0f;
    float err     = rpm_setpoint - rpm_now;

    /* Kp = 0.01 A/RPM – tune this */
    g_iq_ref = utils_truncate_number(err * 0.01f, -20.0f, 20.0f);
}
