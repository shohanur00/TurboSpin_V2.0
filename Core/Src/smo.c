#include "smo.h"
#include <math.h>






void SMO_Init(SMO_t *s,
              float R,
              float L,
              float dt,
              float Ksmo,
              float Kf)
{
    s->R = R;
    s->L = L;
    s->dt = dt;

    s->invL = 1.0f / L;
    s->R_div_L = R / L;

    s->Ksmo = Ksmo;
    s->Kf = Kf;

    s->sat_eps = 0.05f;

    s->i_alpha_hat = 0.0f;
    s->i_beta_hat  = 0.0f;

    s->z_alpha = 0.0f;
    s->z_beta  = 0.0f;

    s->e_alpha = 0.0f;
    s->e_beta  = 0.0f;

    s->theta = 0.0f;
    s->omega = 0.0f;
    s->I_max = 100.0f;
}



// smo.c এ sign() replace করো
static inline float sigmoid(float x, float k)
{
    return x / (fabsf(x) + k);
}

#define PI_F      3.14159265359f
#define TWO_PI_F  6.28318530718f

void SMO_Update(SMO_t *s,
                float i_alpha,
                float i_beta,
                float v_alpha,
                float v_beta)
{
    /* =========================
       1. Current error
    ========================= */
    float err_alpha = s->i_alpha_hat - i_alpha;
    float err_beta  = s->i_beta_hat  - i_beta;

    /* =========================
       2. Sliding Mode function
    ========================= */
    s->z_alpha = s->Ksmo * sigmoid(err_alpha, s->sat_eps);
    s->z_beta  = s->Ksmo * sigmoid(err_beta,  s->sat_eps);

    /* =========================
       3. Current observer
    ========================= */
    s->i_alpha_hat += s->dt *
    (
        v_alpha * s->invL
        - s->R_div_L * s->i_alpha_hat
        - s->z_alpha * s->invL
    );

    s->i_beta_hat += s->dt *
    (
        v_beta * s->invL
        - s->R_div_L * s->i_beta_hat
        - s->z_beta * s->invL
    );

    /* =========================
       4. Clamp current estimate
    ========================= */
    if(s->i_alpha_hat > s->I_max)  s->i_alpha_hat = s->I_max;
    if(s->i_alpha_hat < -s->I_max) s->i_alpha_hat = -s->I_max;

    if(s->i_beta_hat > s->I_max)  s->i_beta_hat = s->I_max;
    if(s->i_beta_hat < -s->I_max) s->i_beta_hat = -s->I_max;

    /* =========================
       5. Back-EMF LPF
    ========================= */
    s->e_alpha += s->Kf * (s->z_alpha - s->e_alpha);
    s->e_beta  += s->Kf * (s->z_beta  - s->e_beta);

    /* =========================
       6. EMF magnitude check
    ========================= */
    float emf_mag =
        s->e_alpha * s->e_alpha +
        s->e_beta  * s->e_beta;

    float theta_new = s->theta;

    if(emf_mag > 1e-3f)
    {
        theta_new = atan2f(-s->e_alpha,
                            s->e_beta);
    }

    /* =========================
       7. Angle difference
    ========================= */
    float d_theta = theta_new - s->theta;

    if(d_theta > PI_F)
        d_theta -= TWO_PI_F;

    if(d_theta < -PI_F)
        d_theta += TWO_PI_F;

    /* =========================
       8. Speed estimation
    ========================= */
    float omega_new = d_theta / s->dt;

    s->omega += 0.05f * (omega_new - s->omega);

    /* =========================
       9. Update angle
    ========================= */
    s->theta = theta_new;
}



void PLL_Init(PLL_t *p)
{
    p->kp = 80.0f;
    p->ki = 400.0f;

    p->integrator = 0.0f;

    p->theta = 0.0f;
    p->omega = 0.0f;

    p->sin_t = 0.0f;
    p->cos_t = 1.0f;
}


void PLL_Update(PLL_t *p,
                float e_alpha,
                float e_beta,
                float dt)
{
    /* Phase error (cross product) */
    float error = (-e_alpha * p->cos_t) +
                  ( e_beta  * p->sin_t);

    /* PI controller */
    p->integrator += p->ki * error * dt;

    float omega = p->kp * error + p->integrator;

    /* integrate angle */
    p->theta += omega * dt;

    /* wrap */
    if(p->theta > 3.1415926f)
        p->theta -= 6.2831852f;

    if(p->theta < -3.1415926f)
        p->theta += 6.2831852f;

    /* update sin/cos */
    p->sin_t = sinf(p->theta);
    p->cos_t = cosf(p->theta);

    p->omega = omega;
}
