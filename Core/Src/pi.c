#include "pi.h"

static inline float clamp(float x, float min, float max)
{
    if(x > max) return max;
    if(x < min) return min;
    return x;
}

void PI_Init(PI_t *pi,
             float kp,
             float ki,
             float Ts,
             float out_min,
             float out_max)
{
    pi->Kp = kp;
    pi->Ki = ki;

    pi->Ts = Ts;

    pi->integral = 0.0f;

    pi->out_min = out_min;
    pi->out_max = out_max;
}

void PI_Reset(PI_t *pi)
{
    pi->integral = 0.0f;
}

float PI_Update(PI_t *pi,
                float ref,
                float feedback)
{
    float error = ref - feedback;

    /* PI output before integration */
    float p = pi->Kp * error;

    /* Integral update */
    pi->integral += pi->Ki * error * pi->Ts;

    /* Anti-windup */
    pi->integral = clamp(pi->integral,
                         pi->out_min,
                         pi->out_max);

    /* Total output */
    float output = p + pi->integral;

    output = clamp(output,
                   pi->out_min,
                   pi->out_max);

    return output;
}

