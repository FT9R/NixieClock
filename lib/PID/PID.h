#pragma once

#include <stdint.h>
#include <string.h>

typedef float float32_t;

/**
 * @ingroup PID
 * @brief Instance structure for the floating-point PID Control.
 */
typedef struct
{
    float32_t A0; /**< The derived gain, A0 = Kp + Ki + Kd . */
    float32_t A1; /**< The derived gain, A1 = -Kp - 2Kd. */
    float32_t A2; /**< The derived gain, A2 = Kd . */
    float32_t state[3]; /**< The state array of length 3. */
    float32_t Kp; /**< The proportional gain. */
    float32_t Ki; /**< The integral gain. */
    float32_t Kd; /**< The derivative gain. */
} arm_pid_instance_f32;

/**
 * @brief Process function for the floating-point PID Control
 * @param S: is an instance of the floating-point PID Control structure
 * @param in: input sample to process
 * @return processed output sample
 */
float32_t arm_pid_f32(arm_pid_instance_f32 *S, float32_t in);

/**
 * @brief Initialization function for the floating-point PID Control
 * @param S: points to an instance of the PID structure
 * @param resetStateFlag: 0: no change in state, 1: reset state
 */
void arm_pid_init_f32(arm_pid_instance_f32 *S, int32_t resetStateFlag);

/**
 * @brief Reset function for the floating-point PID Control
 * @param S: points to an instance of the floating-point PID structure
 */
void arm_pid_reset_f32(arm_pid_instance_f32 *S);