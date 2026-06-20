#ifndef FOC_MATH_H
#define FOC_MATH_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ------------------------------------------------------------------ */
/*  Motor parameters  (TurboSpin V2.0)                                 */
/* ------------------------------------------------------------------ */
#define FOC_MOTOR_R             0.09f       /* Phase resistance  [Ω]  */
#define FOC_MOTOR_L             80e-6f      /* Phase inductance  [H]  */
#define FOC_MOTOR_FLUX_LINKAGE  0.00175f    /* λ  [Wb]  – tune this!  */
#define FOC_PWM_FREQ_HZ         20000.0f
#define FOC_DT                  (1.0f / FOC_PWM_FREQ_HZ)
#define OBS_GAMMA 				10.0f
/* ------------------------------------------------------------------ */
/*  ADC / current sensing  (INA240A1, 8× oversample)                  */
/* ------------------------------------------------------------------ */
#define ADC_VREF                3.3f
#define ADC_RESOLUTION          4096.0f
#define INA240_GAIN             20.0f       /* A1 variant             */
#define SHUNT_RESISTANCE        0.001f      /* 1 mΩ shunt             */
#define ADC_OFFSET              2042        /* mid-rail counts        */
#define ADC_TO_AMPS(raw)  (((float)((raw) - ADC_OFFSET)) * \
                           (ADC_VREF / (ADC_RESOLUTION * INA240_GAIN * SHUNT_RESISTANCE)))

/* ------------------------------------------------------------------ */
/*  Math helpers                                                        */
/* ------------------------------------------------------------------ */
#define SQ(x)               ((x)*(x))
#define ONE_BY_SQRT3        0.57735026919f
#define TWO_BY_SQRT3        1.15470053838f
#define SQRT3_BY_2          0.86602540378f

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define UTILS_NAN_ZERO(x)   do { if (!isfinite(x)) (x) = 0.0f; } while(0)

#define UTILS_LP_FAST(val, sample, factor) \
    (val) = (val) + ((sample) - (val)) * (factor)

static inline float utils_fast_atan2(float y, float x)
{
    float abs_y = fabsf(y) + 1e-10f;
    float r, angle;
    if (x >= 0.0f) {
        r = (x - abs_y) / (x + abs_y);
        angle = 0.1963f * SQ(r) * r - 0.9817f * r + (float)(M_PI / 4.0);
    } else {
        r = (x + abs_y) / (abs_y - x);
        angle = 0.1963f * SQ(r) * r - 0.9817f * r + (float)(3.0 * M_PI / 4.0);
    }
    return (y < 0.0f) ? -angle : angle;
}

static inline float utils_norm_angle_rad(float a)
{
    while (a >  (float)M_PI)  a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI)  a += 2.0f * (float)M_PI;
    return a;
}

static inline float utils_truncate_number(float n, float lo, float hi)
{
    if (n < lo) return lo;
    if (n > hi) return hi;
    return n;
}

static inline float utils_truncate_number_abs(float n, float mag)
{
    if (n >  mag) return  mag;
    if (n < -mag) return -mag;
    return n;
}

static inline float norm2f(float a, float b)
{
    return sqrtf(SQ(a) + SQ(b));
}

/* ------------------------------------------------------------------ */
/*  Observer state  (Ortega Original)                                  */
/* ------------------------------------------------------------------ */
typedef struct {
    float x1;              /* flux-linkage α estimate                 */
    float x2;              /* flux-linkage β estimate                 */
    float lambda_est;      /* estimated |λ| (for lambda-comp variant) */
    float i_alpha_last;
    float i_beta_last;
} observer_state_t;

/* ------------------------------------------------------------------ */
/*  FOC run-time state                                                  */
/* ------------------------------------------------------------------ */
typedef struct {
    /* Measured currents (A) */
    float ia, ib, ic;

    /* Clarke */
    float i_alpha, i_beta;

    /* Park */
    float id, iq;

    /* Voltage commands (duty-normalised, ±1) */
    float vd, vq;
    float v_alpha, v_beta;

    /* Angle & speed */
    float theta;            /* electrical angle  [rad]                */
    float omega;            /* electrical speed  [rad/s]  (PLL)       */

    /* PI integrators */
    float id_int, iq_int;

    /* Observer */
    observer_state_t obs;

    /* PLL */
    float pll_phase;
    float pll_speed;

    /* Gamma (observer gain) */
    float gamma;
} foc_state_t;

/* ------------------------------------------------------------------ */
/*  PI controller config                                                */
/* ------------------------------------------------------------------ */
typedef struct {
    float kp;
    float ki;
    float i_max;   /* integrator clamp */
    float out_max; /* output clamp     */
} pi_config_t;

/* ------------------------------------------------------------------ */
/*  Function prototypes                                                 */
/* ------------------------------------------------------------------ */

/* Clarke & Park */
void foc_clarke(float ia, float ib, float *alpha, float *beta);
void foc_park(float alpha, float beta, float theta, float *d, float *q);
void foc_inv_park(float d, float q, float theta, float *alpha, float *beta);

/* Ortega observer (VESC-style) */
void foc_observer_update(float v_alpha, float v_beta,
                         float i_alpha, float i_beta,
                         float dt, observer_state_t *state,
                         float *phase);

/* PLL */
void foc_pll_run(float phase, float dt,
                 float *phase_var, float *speed_var,
                 float pll_kp, float pll_ki);

/* PI */
float foc_pi_run(float error, float dt,
                 float *integrator, const pi_config_t *cfg);

/* SVPWM */
void foc_svm(float alpha, float beta, float max_mod,
             uint32_t period,
             uint32_t *tA, uint32_t *tB, uint32_t *tC,
             uint32_t *sector);

/* Top-level FOC step – call from PWM ISR */
void foc_run(foc_state_t *s,
             float ia, float ib,
             float iq_ref, float id_ref,
             float vbus,
             uint32_t pwm_period,
             uint32_t *tA, uint32_t *tB, uint32_t *tC);

/* Init */
void foc_init(foc_state_t *s, float gamma);

#endif /* FOC_MATH_H */
