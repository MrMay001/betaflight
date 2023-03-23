#pragma once

#include "common/time.h"
#include "flight/alt_ctrl.h"


typedef struct attitude_ctrl
{
    float r_x;
    float r_y;
    float r_z;

    float r_x_lowpassfilter;
    float r_y_lowpassfilter;

    float r_x_lowpassfilter_last;
    float r_y_lowpassfilter_last;

    float Error_x;
    float Error_y;
    float Error_z;

    float Error_x_filter;
    float Error_y_filter;
    float Error_z_filter;

    float r_Roll;
    float r_Pitch;
    float r_Yaw;

    float roll;  //rad/s
    float pitch;  //rad/s
    float yaw;   //rad/s    

    float error_angle;    
    float error_angle_output;                                                                                              

    float altitude_thrust;  //0-1

    uint16_t sum;
    uint16_t sum1;
    uint16_t sum2;

    float usec;
    float dt;
    float dtHz;
}attitude_ctrl_t;

typedef struct attitude_send
{
    float ROLL;
    float PITCH;
    float YAW;

    float ROLL_rate;
    float PITCH_rate;
    float YAW_rate;

    float test_yaw;
}attitude_send_t;

extern attitude_send_t attitude_send;
extern attitude_ctrl_t attitude_controller;


void attitude_init(attitude_ctrl_t * controller);
void x_controller_init(controller_t * controller);
void y_controller_init(controller_t * controller);
void z_controller_init(controller_t * controller);
void yaw_controller_init(controller_t * controller);
void Attitude_Send_Init(attitude_send_t * attitude_send);
void Position_init(void);

void Position_x_ctrl(attitude_ctrl_t * controller);
void Position_y_ctrl(attitude_ctrl_t * controller);

void Update_Lowpass_Filter(timeUs_t currentTimeUs); //100Hz

void Lowpass_Filter(attitude_ctrl_t * ctrl, float alphax, float alphay, int n);

float Get_vrpn_x(void);
float Get_vrpn_y(void);
float Get_vrpn_z(void);
float Get_z_measure(void);
float Get_vrpn_Roll(void);
float Get_vrpn_Pitch(void);
float Get_vrpn_Yaw(void);

