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
    memset(s, 0, sizeof(foc_state_t));
    s->gamma               = gamma;
    s->obs.lambda_est      = FOC_MOTOR_FLUX_LINKAGE;
    /* Seed x1 so we don't start at zero magnitude */
    s->obs.x1              = FOC_MOTOR_FLUX_LINKAGE;
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
void foc_observer_update(float v_alpha, float v_beta,
                         float i_alpha, float i_beta,
                         float dt, observer_state_t *state,
                         float *phase)
{
    const float R      = FOC_MOTOR_R;
    const float L      = FOC_MOTOR_L;
    const float lambda = FOC_MOTOR_FLUX_LINKAGE;

    float L_ia = L * i_alpha;
    float L_ib = L * i_beta;

    float R_ia = R * i_alpha;
    float R_ib = R * i_beta;

    float gamma_half = state->lambda_est * state->lambda_est;   /* reused below */

    /* ---- Ortega Original ----------------------------------------- */
    /*
     * err = λ² - |ψ - L·i|²
     * Clamp err ≤ 0  (forces convergence, see paper)
     */
    float err = SQ(lambda) - (SQ(state->x1 - L_ia) + SQ(state->x2 - L_ib));

    if (err > 0.0f) {
        err = 0.0f;
    }

    /*
     * gamma_half is ½ · γ  in the paper.
     * We store plain γ in foc_state_t and pass lambda_est here as a
     * stand-in initialised value; the actual gain comes from the caller
     * via the state struct field used below.
     *
     * Practical default: γ ≈ 1000  (tune for your motor speed range)
     */
    gamma_half = state->lambda_est;   /* re-purpose field as gain carrier */
    /* NOTE: caller sets state->lambda_est = gamma on init so the field
       doubles as the gain.  If you want proper lambda adaptation, switch
       to FOC_OBSERVER_ORTEGA_LAMBDA_COMP variant below.             */

    float x1_dot = v_alpha - R_ia + gamma_half * (state->x1 - L_ia) * err;
    float x2_dot = v_beta  - R_ib + gamma_half * (state->x2 - L_ib) * err;

    state->x1 += x1_dot * dt;
    state->x2 += x2_dot * dt;

    state->i_alpha_last = i_alpha;
    state->i_beta_last  = i_beta;

    /* Protect against NaN */
    UTILS_NAN_ZERO(state->x1);
    UTILS_NAN_ZERO(state->x2);

    /* If magnitude collapses, rescale to avoid unstable angle */
    float mag = norm2f(state->x1, state->x2);
    if (mag < (lambda * 0.5f)) {
        state->x1 *= 1.1f;
        state->x2 *= 1.1f;
    }

    if (phase) {
        *phase = utils_fast_atan2(state->x2 - L_ib, state->x1 - L_ia);
    }
}

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
