// observer_ortega.h
#ifndef OBSERVER_ORTEGA_H
#define OBSERVER_ORTEGA_H

#include <stdint.h>

typedef struct {
    float psi_alpha;
    float psi_beta;
    float theta;
    float omega;
    float theta_prev;
} OrtegaObs_t;

void Ortega_Init(OrtegaObs_t *obs);
void Ortega_Update(OrtegaObs_t *obs,
                   float v_alpha, float v_beta,
                   float i_alpha, float i_beta,
                   float Ts);

#endif
