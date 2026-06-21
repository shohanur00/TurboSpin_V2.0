// observer_ortega.c
#include "observer_ortega.h"
#include <math.h>

#define OBS_GAMMA     500.0f
#define MOTOR_RS      0.09f       // তোমার MOTOR_R এর সাথে match
#define MOTOR_PSI_NOM 0.008f      // শুরুতে এটা দিয়ে try করো

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

    // Angle
    obs->theta_prev = obs->theta;
    obs->theta      = atan2f(obs->psi_beta, obs->psi_alpha);

    // Omega (unwrapped)
    float dtheta = obs->theta - obs->theta_prev;
    if (dtheta >  3.14159f) dtheta -= 6.28318f;
    if (dtheta < -3.14159f) dtheta += 6.28318f;
    obs->omega = dtheta / Ts;
}
