//
// Created by local_user on 2026/2/5.
/* USER CODE BEGIN Header */
/**
 * @file LQR.h
 * @brief LQR controller header file
 * @author Alex Wang
 * @date 2026/02/05
 * @version v1.0
 */
/* USER CODE END Header */
#ifndef COD_H7_TEMPLATE_LQR_H
#define COD_H7_TEMPLATE_LQR_H

#include "Control_Task.h"
#include "INS_Task.h"

/* Typedefs ***************************************************************/
typedef struct
{
    float theta_ll;
    float dtheta_ll;
    float theta_lr;
    float dtheta_lr;
    float theta_wl;
    float dtheta_wl;
    float theta_wr;
    float dtheta_wr;
    float theta_b;
    float dtheta_b;
}vec_state;

typedef struct
{
    float T_bll;
    float T_blr;
    float T_lwl;
    float T_lwr;
}vec_input;

/* Externs ***************************************************************/
extern vec_state state, last_state, d_state, ref_state, error;
extern vec_input input;

/* Function Prototypes ***********************************************/
void LQR_ref_state_init(vec_state *ref_state);
void LQR_ref_state_update(vec_state *ref_state);
void LQR_state_update(vec_state *state, vec_state *last_state, vec_state *ref_state ,vec_state *error);
#endif //COD_H7_TEMPLATE_LQR_H