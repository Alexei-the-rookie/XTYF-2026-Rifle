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
    ref_state->dtheta_wl = Control_Info.Target.Chassis_Velocity;
    ref_state->theta_wr = 0.f;
    ref_state->dtheta_wr = Control_Info.Target.Chassis_Velocity;
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

void LQR_calc(vec_state *error, vec_input *input)
{
    input->T_bll = matK[0][0] * error->theta_ll + matK[1][0] * error->dtheta_ll + matK[2][0] * error->theta_lr + matK[3][0] * error->dtheta_lr
                 + matK[4][0] * error->theta_wl + matK[5][0] * error->dtheta_wl + matK[6][0] * error->theta_wr
                 + matK[7][0] * error->dtheta_wr + matK[8][0] * error->theta_b + matK[9][0] * error->dtheta_b;
    input->T_blr = matK[0][1] * error->theta_ll + matK[1][1] * error->dtheta_ll + matK[2][1] * error->theta_lr + matK[3][1] * error->dtheta_lr
                 + matK[4][1] * error->theta_wl + matK[5][1] * error->dtheta_wl + matK[6][1] * error->theta_wr
                 + matK[7][1] * error->dtheta_wr + matK[8][1] * error->theta_b + matK[9][1] * error->dtheta_b;
    input->T_lwl = matK[0][2] * error->theta_ll + matK[1][2] * error->dtheta_ll + matK[2][2] * error->theta_lr + matK[3][2] * error->dtheta_lr
                 + matK[4][2] * error->theta_wl + matK[5][2] * error->dtheta_wl + matK[6][2] * error->theta_wr
                 + matK[7][2] * error->dtheta_wr + matK[8][2] * error->theta_b + matK[9][2] * error->dtheta_b;
    input->T_lwr = matK[0][3] * error->theta_ll + matK[1][3] * error->dtheta_ll + matK[2][3] * error->theta_lr + matK[3][3] * error->dtheta_lr
                 + matK[4][3] * error->theta_wl + matK[5][3] * error->dtheta_wl + matK[6][3] * error->theta_wr
                 + matK[7][3] * error->dtheta_wr + matK[8][3] * error->theta_b + matK[9][3] * error->dtheta_b;
}//calculate the input based on the error and gain matrix. u=-K(x-x_ref)
