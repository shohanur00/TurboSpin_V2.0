#ifndef PI_H
#define PI_H

typedef struct
{
    float Kp;
    float Ki;

    float Ts;

    float integral;

    float out_min;
    float out_max;

}PI_t;

void PI_Init(PI_t *pi,
             float kp,
             float ki,
             float Ts,
             float out_min,
             float out_max);

float PI_Update(PI_t *pi,
                float ref,
                float feedback);

void PI_Reset(PI_t *pi);

#endif
