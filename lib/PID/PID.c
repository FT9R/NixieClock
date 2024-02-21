#include "PID.h"

float32_t arm_pid_f32(arm_pid_instance_f32 *S, float32_t in)
{
    float32_t out;

    /* y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2] */
    out = (S->A0 * in) + (S->A1 * S->state[0]) + (S->A2 * S->state[1]) + (S->state[2]);

    /* Update state */
    S->state[1] = S->state[0];
    S->state[0] = in;
    S->state[2] = out;

    /* return to application */
    return (out);
}

void arm_pid_init_f32(arm_pid_instance_f32 *S, int32_t resetStateFlag)
{
    /* Derived coefficient A0 */
    S->A0 = S->Kp + S->Ki + S->Kd;

    /* Derived coefficient A1 */
    S->A1 = (-S->Kp) - ((float32_t) 2.0f * S->Kd);

    /* Derived coefficient A2 */
    S->A2 = S->Kd;

    /* Check whether state needs reset or not */
    if (resetStateFlag)
        arm_pid_reset_f32(S);
}

void arm_pid_reset_f32(arm_pid_instance_f32 *S)
{
    /* Reset state to zero, The size will be always 3 samples */
    memset(S->state, 0, 3U * sizeof(float32_t));
}