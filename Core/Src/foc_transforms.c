#include "foc_transforms.h"
#include "main.h"
#include <math.h>

#define INV_SQRT3 0.577350269f   // 1/sqrt(3)


Clarke_t Clarke(float ia, float ib)
{
    Clarke_t out;

    out.alpha = ia;
    out.beta  = (ia + 2.0f * ib) * 0.577350269f;

    return out;
}

// Park
Park_t Park(Clarke_t c, float sin_theta, float cos_theta)
{
    Park_t out;
    out.d =  c.alpha * cos_theta + c.beta * sin_theta;
    out.q = -c.alpha * sin_theta + c.beta * cos_theta;
    return out;
}
