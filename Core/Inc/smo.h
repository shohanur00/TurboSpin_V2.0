#ifndef SMO_H
#define SMO_H

typedef struct
{
    /* Motor Parameters */
    float R;
    float L;
    float dt;

    /* Precomputed */
    float invL;
    float R_div_L;

    /* Gains */
    float Ksmo;
    float Kf;

    /* Sliding saturation */
    float sat_eps;

    /* Observer states */
    float i_alpha_hat;
    float i_beta_hat;

    float z_alpha;
    float z_beta;

    float e_alpha;
    float e_beta;

    /* Output */
    float theta;
    float omega;

    float I_max;

} SMO_t;


typedef struct
{
    float kp;
    float ki;

    float integrator;

    float theta;
    float omega;

    float sin_t;
    float cos_t;

} PLL_t;


void SMO_Init(SMO_t *s, float R, float L, float dt, float Ksmo, float Kf);
void SMO_Update(SMO_t *s, float i_alpha, float i_beta, float v_alpha, float v_beta);
void PLL_Init(PLL_t *p);
void PLL_Update(PLL_t *p,
                float e_alpha,
                float e_beta,
                float dt);

#endif
