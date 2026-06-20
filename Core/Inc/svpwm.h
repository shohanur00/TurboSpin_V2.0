#ifndef SVPWM_H
#define SVPWM_H

#include<stdint.h>

void SVPWM_Update(float v_alpha, float v_beta, float vbus, uint32_t tim_period);

#endif
