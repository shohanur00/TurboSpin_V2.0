/*
 * foc_math.c  –  VESC-style sensorless FOC for TurboSpin V2.0
 *
 * MCU   : STM32G431CBT6
 * Driver: DRV8301
 * Sense : INA240A1  (ADC injected, 8× oversample)
 *
 * Observer: Ortega Original
 * Reference:
 *   http://cas.ensmp.fr/~praly/Telechargement/Journaux/
 *   2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf
 */

#include "foc_math.h"
#include <string.h>

/* ================================================================== */
/*  Init                                                                */
/* ================================================================== */
void foc_init(foc_state_t *s, float gamma)
{

}

/* ================================================================== */
/*  Clarke transform   (ia + ib + ic = 0  →  ic = -(ia+ib))           */
/* ================================================================== */
void foc_clarke(float ia, float ib, float *alpha, float *beta)
{
    *alpha = ia;
    *beta  = ONE_BY_SQRT3 * ia + TWO_BY_SQRT3 * ib;
}

/* ================================================================== */
/*  Park transform                                                      */
/* ================================================================== */
void foc_park(float alpha, float beta, float theta, float *d, float *q)
{
    float c = cosf(theta);
    float s = sinf(theta);
    *d =  alpha * c + beta * s;
    *q = -alpha * s + beta * c;
}

/* ================================================================== */
/*  Inverse Park transform                                              */
/* ================================================================== */
void foc_inv_park(float d, float q, float theta, float *alpha, float *beta)
{
    float c = cosf(theta);
    float s = sinf(theta);
    *alpha = d * c - q * s;
    *beta  = d * s + q * c;
}

/* ================================================================== */
/*  Ortega Original flux observer  (VESC foc_observer_update)          */
/*                                                                      */
/*  state->x1, x2  are  ψα, ψβ  (flux linkage estimates)              */
/*  phase output   is   atan2(x2 - L·iβ, x1 - L·iα)                  */
/* ================================================================== */
// observer_ortega.c

#define OBS_GAMMA       500.0f
#define MOTOR_RS        0.05f      // phase resistance (Ω) — measure করতে হবে
#define MOTOR_PSI_NOM   0.003f     // nominal flux (Wb) — datasheet বা measure

typedef struct {
    float psi_alpha;   // flux estimate α
    float psi_beta;    // flux estimate β
    float theta;       // estimated angle (rad)
    float omega;       // estimated angular velocity (rad/s)
    float theta_prev;  // previous angle for omega calculation
} OrtegaObs_t;

//void Ortega_Update(OrtegaObs_t *obs,
//                   float v_alpha, float v_beta,
//                   float i_alpha, float i_beta,
//                   float Ts)
//{
//    // |ψ|² - ψ_nom²
//    float psi_sq = obs->psi_alpha * obs->psi_alpha
//                 + obs->psi_beta  * obs->psi_beta;
//    float psi_nom_sq = MOTOR_PSI_NOM * MOTOR_PSI_NOM;
//    float err = psi_sq - psi_nom_sq;
//
//    // dψα/dt = Vα - Rs·Iα - γ·err·ψα
//    float dpsi_alpha = v_alpha
//                     - MOTOR_RS * i_alpha
//                     - OBS_GAMMA * err * obs->psi_alpha;
//
//    // dψβ/dt = Vβ - Rs·Iβ - γ·err·ψβ
//    float dpsi_beta  = v_beta
//                     - MOTOR_RS * i_beta
//                     - OBS_GAMMA * err * obs->psi_beta;
//
//    // Forward Euler integration
//    obs->psi_alpha += dpsi_alpha * Ts;
//    obs->psi_beta  += dpsi_beta  * Ts;
//
//    // Angle extraction
//    obs->theta_prev = obs->theta;
//    obs->theta = atan2f(obs->psi_beta, obs->psi_alpha);
//
//    // Angular velocity (numerical derivative, unwrapped)
//    float d_theta = obs->theta - obs->theta_prev;
//    // Unwrap
//    if (d_theta >  M_PI) d_theta -= 2.0f * M_PI;
//    if (d_theta < -M_PI) d_theta += 2.0f * M_PI;
//    obs->omega = d_theta / Ts;
//}

/* ================================================================== */
/*  PLL  (tracks observer phase, gives smoothed speed)                  */
/* ================================================================== */
void foc_pll_run(float phase, float dt,
                 float *phase_var, float *speed_var,
                 float pll_kp, float pll_ki)
{
    UTILS_NAN_ZERO(*phase_var);
    UTILS_NAN_ZERO(*speed_var);

    float delta_theta = phase - *phase_var;

    /* Normalise to [-π, π] */
    delta_theta = utils_norm_angle_rad(delta_theta);

    *phase_var += (*speed_var + pll_kp * delta_theta) * dt;
    *phase_var  = utils_norm_angle_rad(*phase_var);

    *speed_var += pll_ki * delta_theta * dt;
}

/* ================================================================== */
/*  PI controller with anti-windup                                      */
/* ================================================================== */
float foc_pi_run(float error, float dt,
                 float *integrator, const pi_config_t *cfg)
{
    *integrator += error * cfg->ki * dt;
    *integrator  = utils_truncate_number_abs(*integrator, cfg->i_max);

    float output = error * cfg->kp + *integrator;
    return utils_truncate_number_abs(output, cfg->out_max);
}

/* ================================================================== */
/*  SVPWM  (direct port from VESC foc_svm)                             */
/*                                                                      */
/*  alpha, beta : voltage vector (normalised: |v| ≤ √3/2 ≈ 0.866)     */
/*  max_mod     : modulation limit  (e.g. 0.95)                        */
/*  period      : TIMx->ARR value                                       */
/* ================================================================== */
void foc_svm(float alpha, float beta, float max_mod,
             uint32_t period,
             uint32_t *tA, uint32_t *tB, uint32_t *tC,
             uint32_t *sector)
{
    uint32_t sec;

    if (beta >= 0.0f) {
        if (alpha >= 0.0f)
            sec = (ONE_BY_SQRT3 * beta > alpha) ? 2 : 1;
        else
            sec = (-ONE_BY_SQRT3 * beta > alpha) ? 3 : 2;
    } else {
        if (alpha >= 0.0f)
            sec = (-ONE_BY_SQRT3 * beta > alpha) ? 5 : 6;
        else
            sec = (ONE_BY_SQRT3 * beta > alpha) ? 4 : 5;
    }

    int tA_i, tB_i, tC_i;
    int P = (int)period;

    switch (sec) {
    case 1: {
        int t1 = (int)((alpha - ONE_BY_SQRT3 * beta) * (float)P);
        int t2 = (int)((TWO_BY_SQRT3 * beta)          * (float)P);
        tA_i = (P + t1 + t2) / 2;
        tB_i = tA_i - t1;
        tC_i = tB_i - t2;
        break;
    }
    case 2: {
        int t2 = (int)(( alpha + ONE_BY_SQRT3 * beta) * (float)P);
        int t3 = (int)((-alpha + ONE_BY_SQRT3 * beta) * (float)P);
        tB_i = (P + t2 + t3) / 2;
        tA_i = tB_i - t3;
        tC_i = tA_i - t2;
        break;
    }
    case 3: {
        int t3 = (int)((TWO_BY_SQRT3 * beta)           * (float)P);
        int t4 = (int)((-alpha - ONE_BY_SQRT3 * beta)  * (float)P);
        tB_i = (P + t3 + t4) / 2;
        tC_i = tB_i - t3;
        tA_i = tC_i - t4;
        break;
    }
    case 4: {
        int t4 = (int)((-alpha + ONE_BY_SQRT3 * beta)  * (float)P);
        int t5 = (int)((-TWO_BY_SQRT3 * beta)          * (float)P);
        tC_i = (P + t4 + t5) / 2;
        tB_i = tC_i - t5;
        tA_i = tB_i - t4;
        break;
    }
    case 5: {
        int t5 = (int)((-alpha - ONE_BY_SQRT3 * beta)  * (float)P);
        int t6 = (int)(( alpha - ONE_BY_SQRT3 * beta)  * (float)P);
        tC_i = (P + t5 + t6) / 2;
        tA_i = tC_i - t5;
        tB_i = tA_i - t6;
        break;
    }
    default: /* case 6 */ {
        int t6 = (int)((-TWO_BY_SQRT3 * beta)          * (float)P);
        int t1 = (int)(( alpha + ONE_BY_SQRT3 * beta)  * (float)P);
        tA_i = (P + t6 + t1) / 2;
        tC_i = tA_i - t1;
        tB_i = tC_i - t6;
        break;
    }
    }

    /* Clamp */
    int t_max = (int)((float)P * (1.0f - (1.0f - max_mod) * 0.5f));
    if (tA_i < 0) tA_i = 0;  if (tA_i > t_max) tA_i = t_max;
    if (tB_i < 0) tB_i = 0;  if (tB_i > t_max) tB_i = t_max;
    if (tC_i < 0) tC_i = 0;  if (tC_i > t_max) tC_i = t_max;

    *tA     = (uint32_t)tA_i;
    *tB     = (uint32_t)tB_i;
    *tC     = (uint32_t)tC_i;
    *sector = sec;
}

/* ================================================================== */
/*  Top-level FOC step  –  call every PWM period from TIM1 ISR         */
/*                                                                      */
/*  ia, ib   : phase currents in Amperes (already converted)           */
/*  iq_ref   : torque current setpoint  [A]                            */
/*  id_ref   : flux current setpoint    [A]  (0 for MTPA)             */
/*  vbus     : DC-link voltage          [V]                            */
/*  pwm_period: TIM1->ARR                                               */
/* ================================================================== */

/* Tune these for your motor / bus */
#define PLL_KP      1000.0f
#define PLL_KI      20000.0f

/* Current PI gains  – start conservative, increase until noise        */
#define ID_KP       0.5f
#define ID_KI       200.0f
#define IQ_KP       0.5f
#define IQ_KI       200.0f

#define I_OUT_MAX   0.95f   /* normalised voltage limit */

/* Observer gain – increase for faster tracking, decrease if unstable  */
#define OBS_GAMMA   500.0f

void foc_run(foc_state_t *s,
             float ia, float ib,
             float iq_ref, float id_ref,
             float vbus,
             uint32_t pwm_period,
             uint32_t *tA, uint32_t *tB, uint32_t *tC)
{
    const float dt = FOC_DT;

    /* ---- 1. Store raw currents ------------------------------------ */
    s->ia = ia;
    s->ib = ib;
    s->ic = -(ia + ib);

    /* ---- 2. Clarke ------------------------------------------------ */
    foc_clarke(ia, ib, &s->i_alpha, &s->i_beta);

    /* ---- 3. Observer  (needs v_alpha, v_beta from previous cycle) - */
    /*  On first call v_alpha/beta are 0 → observer starts from rest   */
    float obs_phase;
    s->obs.lambda_est = OBS_GAMMA;    /* reuse field as gain carrier   */
    foc_observer_update(s->v_alpha, s->v_beta,
                        s->i_alpha, s->i_beta,
                        dt, &s->obs, &obs_phase);

    /* ---- 4. PLL --------------------------------------------------- */
    foc_pll_run(obs_phase, dt,
                &s->pll_phase, &s->pll_speed,
                PLL_KP, PLL_KI);

    s->theta = s->pll_phase;
    s->omega  = s->pll_speed;

    /* ---- 5. Park -------------------------------------------------- */
    foc_park(s->i_alpha, s->i_beta, s->theta, &s->id, &s->iq);

    /* ---- 6. Current PI controllers -------------------------------- */
    static const pi_config_t pi_d = { ID_KP, ID_KI, 0.9f, I_OUT_MAX };
    static const pi_config_t pi_q = { IQ_KP, IQ_KI, 0.9f, I_OUT_MAX };

    float vd_ff = -s->omega * FOC_MOTOR_L * s->iq;   /* decoupling   */
    float vq_ff =  s->omega * FOC_MOTOR_L * s->id
                 + s->omega * FOC_MOTOR_FLUX_LINKAGE;

    s->vd = foc_pi_run(id_ref - s->id, dt, &s->id_int, &pi_d) + vd_ff;
    s->vq = foc_pi_run(iq_ref - s->iq, dt, &s->iq_int, &pi_q) + vq_ff;

    /* ---- 7. Inverse Park ----------------------------------------- */
    foc_inv_park(s->vd, s->vq, s->theta, &s->v_alpha, &s->v_beta);

    /* ---- 8. Normalise to ±1 using vbus ---------------------------- */
    float inv_vbus = (vbus > 1.0f) ? (1.0f / vbus) : 1.0f;
    float va_norm  = s->v_alpha * inv_vbus;
    float vb_norm  = s->v_beta  * inv_vbus;

    /* ---- 9. SVPWM ------------------------------------------------- */
    uint32_t sec;
    foc_svm(va_norm, vb_norm, 0.95f, pwm_period, tA, tB, tC, &sec);
}
