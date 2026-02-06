//
// Created by local_user on 2026/2/5.
//
/**
 * @file LQR.c
 * @brief LQR controller implementation
 * @author Alex Wang
 * @date 2026/02/05
 * @version v1.0
 */
/* INCLUDE FILES */
#include "LQR.h"
#include "arm_math.h"
#include "Control_Task.h"
#include "INS_Task.h"
#include "Motor.h"

void LQR_ref_state_init(vec_state *ref_state)
{
    ref_state->theta_ll = 0.f;
    ref_state->dtheta_ll = 0.f;
    ref_state->theta_lr = 0.f;
    ref_state->dtheta_lr = 0.f;
    ref_state->theta_wl = 0.f;
    ref_state->dtheta_wl = 0.f;
    ref_state->theta_wr = 0.f;
    ref_state->dtheta_wr = 0.f;
    ref_state->theta_b = 0.f;
    ref_state->dtheta_b = 0.f;//all the reference state set to zero at the very beginning.
}

void LQR_ref_state_update(vec_state *ref_state)
{
    ref_state->theta_b = 0.f;
    ref_state->dtheta_b = 0.f;
    ref_state->theta_wl = 0.f;
    ref_state->dtheta_wl = 0.f;//Control_Info.Target.Chassis_Velocity;
    ref_state->theta_wr = 0.f;
    ref_state->dtheta_wr = 0.f;//Control_Info.Target.Chassis_Velocity;
    ref_state->theta_ll = 0.f;
    ref_state->dtheta_ll = 0.f;
    ref_state->theta_lr = 0.f;
    ref_state->dtheta_lr = 0.f;//waiting for rifle's remote control.
}//update the target state.

void LQR_state_update(vec_state *state, vec_state *last_state, vec_state *ref_state ,vec_state *error)
{
    state->theta_b = INS_Info.Pitch_Angle;
    state->dtheta_b = INS_Info.Pitch_Gyro;
    state->theta_wl = DJI_Left_Motor.Data.Angle;
    state->dtheta_wl = DJI_Left_Motor.Data.Velocity;
    state->theta_wr = DJI_Right_Motor.Data.Angle;
    state->dtheta_wr = DJI_Right_Motor.Data.Velocity;
    state->theta_ll = 0.f;
    state->dtheta_ll = 0.f - last_state->dtheta_ll;//waiting for leg motors' feedback.
    state->theta_lr = 0.f;
    state->dtheta_lr = 0.f - last_state->dtheta_lr;//waiting for leg motors' feedback.
    last_state = state;//update the last state.

    error->theta_b = ref_state->theta_b - state->theta_b;
    error->dtheta_b = ref_state->dtheta_b - state->dtheta_b;
    error->theta_wl = ref_state->theta_wl - state->theta_wl;
    error->dtheta_wl = ref_state->dtheta_wl - state->dtheta_wl;
    error->theta_wr = ref_state->theta_wr - state->theta_wr;
    error->dtheta_wr = ref_state->dtheta_wr - state->dtheta_wr;
    error->theta_ll = ref_state->theta_ll - state->theta_ll;
    error->dtheta_ll = ref_state->dtheta_ll - state->dtheta_ll;
    error->theta_lr = ref_state->theta_lr - state->theta_lr;
    error->dtheta_lr = ref_state->dtheta_lr - state->dtheta_lr;//calculate the error between target state and current state.
}

void LQR_calc(vec_state *vec_error, vec_input *vec_input)
{
    vec_input->T_bll = matK[0][0] * vec_error->theta_ll + matK[0][1] * vec_error->dtheta_ll + matK[0][2] * vec_error->theta_lr + matK[0][3] * vec_error->dtheta_lr
                 + matK[0][4] * vec_error->theta_wl + matK[0][5] * vec_error->dtheta_wl + matK[0][6] * vec_error->theta_wr
                 + matK[0][7] * vec_error->dtheta_wr + matK[0][8] * vec_error->theta_b + matK[0][9] * vec_error->dtheta_b;
    vec_input->T_blr = matK[1][0] * vec_error->theta_ll + matK[1][1] * vec_error->dtheta_ll + matK[1][2] * vec_error->theta_lr + matK[1][3] * vec_error->dtheta_lr
                 + matK[1][4] * vec_error->theta_wl + matK[1][5] * vec_error->dtheta_wl + matK[1][6] * vec_error->theta_wr
                 + matK[1][7] * vec_error->dtheta_wr + matK[1][8] * vec_error->theta_b + matK[1][9] * vec_error->dtheta_b;
    vec_input->T_lwl = matK[2][0] * vec_error->theta_ll + matK[2][1] * vec_error->dtheta_ll + matK[2][2] * vec_error->theta_lr + matK[2][3] * vec_error->dtheta_lr
                 + matK[2][4] * vec_error->theta_wl + matK[2][5] * vec_error->dtheta_wl + matK[2][6] * vec_error->theta_wr
                 + matK[2][7] * vec_error->dtheta_wr + matK[2][8] * vec_error->theta_b + matK[2][9] * vec_error->dtheta_b;
    vec_input->T_lwr = matK[3][0] * vec_error->theta_ll + matK[3][1] * vec_error->dtheta_ll + matK[3][2] * vec_error->theta_lr + matK[3][3] * vec_error->dtheta_lr
                 + matK[3][4] * vec_error->theta_wl + matK[3][5] * vec_error->dtheta_wl + matK[3][6] * vec_error->theta_wr
                 + matK[3][7] * vec_error->dtheta_wr + matK[3][8] * vec_error->theta_b + matK[3][9] * vec_error->dtheta_b;
}//calculate the input based on the error and gain matrix. u=-K(x-x_ref)
