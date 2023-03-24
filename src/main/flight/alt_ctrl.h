#pragma once

#include "common/time.h"

#include "sensors/rangefinder.h"
#include "kalman_filter.h"

typedef struct pid
{
    float P;
    float I;
    float D;

    float Error1;   //Error[n-1]
    float Error2;   //Error[n-2]
    float iError;   //Error[n]
    float IiError;
    float DiError;

}pid_t;
typedef struct controller
{
    pid_t pid;

    float setpoint;
    float setpoint_input;
    float output;
    float output_min;
    float output_max;
    float throttle;

    float input_error_range;

    float dt;
    float process_dt;

}controller_t;


typedef struct attitude_ctrl
{
    float r_x;
    float r_y;
    float r_z;

    float r_x_last;
    float r_y_last;
    float r_z_last;

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

    float Error_x_filter_last;
    float Error_y_filter_last;
    float Error_z_filter_last;

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
    float pidupdate_dt;
    float filter_dt;
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

extern controller_t attitude_x_controller;
extern controller_t attitude_y_controller;
extern controller_t attitude_z_controller;
extern controller_t vel_x_controller;
extern controller_t vel_y_controller;
extern controller_t vel_z_controller;
extern controller_t attitude_yaw_controller;
extern controller_t height_controller; 
extern controller_t vel_controller; 

void attitude_controller_init(attitude_ctrl_t * ctrl);
void position_controller_init(controller_t * controller, int axis);
void vel_controller_init(controller_t * controller, int axis);
void Controller_Init(void);

float pid_controller(float process_value, controller_t *controller, float I_limit);
void adjust_position(kalman_filter_t *filter);
void adjust_velocity(kalman_filter_t *filter);

void Update_PID_Position(timeUs_t currentTimeUs);
void Update_PID_Velocity(timeUs_t currentTimeUs);
void Update_Lowpass_Filter(timeUs_t currentTimeUs);

float Get_Height_PID_Output(int n);
float Get_Velocity_PID_Output(int n);
float Get_Velocity_throttle(int n);
float Get_Position_LpFiter(int n);
float Get_Velocity_LpFiter(int n);
