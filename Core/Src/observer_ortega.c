// observer_ortega.c
#include "observer_ortega.h"
#include <math.h>

#define MOTOR_PSI_NOM 0.025f  // actual observed magnitude
#define OBS_GAMMA     100.0f   // আরো কমাও
#define MOTOR_RS      0.09f


#define PSI_MIN (0.2f * MOTOR_PSI_NOM)
#define PSI_MAX (2.0f * MOTOR_PSI_NOM)

void Ortega_Init(OrtegaObs_t *obs)
{
    obs->psi_alpha  = MOTOR_PSI_NOM;  // zero থেকে শুরু না করে
    obs->psi_beta   = 0.0f;           // convergence দ্রুত হয়
    obs->theta      = 0.0f;
    obs->omega      = 0.0f;
    obs->theta_prev = 0.0f;
}

void Ortega_Update(OrtegaObs_t *obs,
                   float v_alpha, float v_beta,
                   float i_alpha, float i_beta,
                   float Ts)
{
    // |ψ|² - ψ_nom²
    float psi_sq     = obs->psi_alpha * obs->psi_alpha
                     + obs->psi_beta  * obs->psi_beta;
    float psi_nom_sq = MOTOR_PSI_NOM  * MOTOR_PSI_NOM;
    float err        = psi_sq - psi_nom_sq;

    // dψ/dt
    float dpsi_alpha = v_alpha - MOTOR_RS * i_alpha
                     - OBS_GAMMA * err * obs->psi_alpha;
    float dpsi_beta  = v_beta  - MOTOR_RS * i_beta
                     - OBS_GAMMA * err * obs->psi_beta;

    // Euler integration
    obs->psi_alpha += dpsi_alpha * Ts;
    obs->psi_beta  += dpsi_beta  * Ts;

    float mag = sqrtf(obs->psi_alpha * obs->psi_alpha +
                      obs->psi_beta  * obs->psi_beta);

    if (mag > PSI_MAX)
    {
        float k = PSI_MAX / mag;
        obs->psi_alpha *= k;
        obs->psi_beta  *= k;
    }
    else if (mag < PSI_MIN && mag > 1e-6f)
    {
        float k = PSI_MIN / mag;
        obs->psi_alpha *= k;
        obs->psi_beta  *= k;
    }

    // Angle
    obs->theta_prev = obs->theta;
    // observer_ortega.c এ theta calculation এ:
    obs->theta = atan2f(obs->psi_beta, obs->psi_alpha);

    // Wrap করো -π to +π এ:
    if(obs->theta >  3.14159f) obs->theta -= 6.28318f;
    if(obs->theta < -3.14159f) obs->theta += 6.28318f;

    // Omega (unwrapped)
    float dtheta = obs->theta - obs->theta_prev;
    if (dtheta >  3.14159f) dtheta -= 6.28318f;
    if (dtheta < -3.14159f) dtheta += 6.28318f;
    float omega_new = dtheta / Ts;

    obs->omega += 0.05f * (omega_new - obs->omega);
}
