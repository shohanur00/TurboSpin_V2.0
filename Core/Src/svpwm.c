#include "svpwm.h"
#include "main.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;

static inline float clamp(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

void SVPWM_Update(float v_alpha,
                  float v_beta,
                  float vbus,
                  uint32_t pwm_period)
{
    /* Normalize voltage */
    float inv_vbus = 1.0f / (vbus + 0.001f);

    float alpha = v_alpha * inv_vbus;
    float beta  = v_beta  * inv_vbus;

    /* Inverse Clarke */
    float Va = alpha;
    float Vb = -0.5f * alpha + 0.8660254f * beta;
    float Vc = -0.5f * alpha - 0.8660254f * beta;

    /* Zero-sequence injection */
    float vmax = Va;
    float vmin = Va;

    if(Vb > vmax) vmax = Vb;
    if(Vc > vmax) vmax = Vc;

    if(Vb < vmin) vmin = Vb;
    if(Vc < vmin) vmin = Vc;

    float Voffset = 0.5f * (vmax + vmin);

    Va -= Voffset;
    Vb -= Voffset;
    Vc -= Voffset;

    /* Duty */
    float dutyA = clamp(0.5f + 0.5f * Va, 0.0f, 1.0f);
    float dutyB = clamp(0.5f + 0.5f * Vb, 0.0f, 1.0f);
    float dutyC = clamp(0.5f + 0.5f * Vc, 0.0f, 1.0f);

    __HAL_TIM_SET_COMPARE(&htim1,
                          TIM_CHANNEL_1,
                          (uint32_t)(dutyA * pwm_period));

    __HAL_TIM_SET_COMPARE(&htim1,
                          TIM_CHANNEL_2,
                          (uint32_t)(dutyB * pwm_period));

    __HAL_TIM_SET_COMPARE(&htim1,
                          TIM_CHANNEL_3,
                          (uint32_t)(dutyC * pwm_period));
}
