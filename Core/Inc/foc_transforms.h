#ifndef FOC_TRANSFORMS_H
#define FOC_TRANSFORMS_H

#include <stdint.h>

// foc_transforms.h এ add করো

typedef struct {
    float alpha;
    float beta;
} Clarke_t;

typedef struct {
    float d;
    float q;
} Park_t;

// Clarke

Clarke_t Clarke(float ia, float ib);
Park_t Park(Clarke_t c, float sin_theta, float cos_theta);

#endif
