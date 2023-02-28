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
    float output;
    float output_min;
    float output_max;
    float throttle;

    float input_error_range;

}controller_t;

typedef struct attitude_ctrl
{
    float roll;  //rad/s
    float pitch;  //rad/s
    float yaw;   //rad/s

    float altitude_thrust;  //0-1
}attitude_ctrl_t;


extern attitude_ctrl_t attitude_controller;
extern controller_t height_controller; 
extern controller_t vel_controller; 

void height_controller_init(controller_t * controller);
void vel_controller_init(controller_t * controller);
void attitude_init(attitude_ctrl_t * controller);
void Controller_Init(void);

float pid_controller(float process_value, controller_t *controller);
void adjust_height(float height_error_range, kalman_filter_t *filter);
void adjust_velocity(float vel_error_range, kalman_filter_t *filter);

void Update_PID_Height(timeUs_t currentTimeUs);
void Update_PID_Velocity(timeUs_t currentTimeUs);

float Get_Height_PID_Output(void);
float Get_Velocity_PID_Output(void);
float Get_Velocity_throttle(void);
