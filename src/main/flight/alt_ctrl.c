#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "common/maths.h"

#include "drivers/time.h"

#include "flight/alt_ctrl.h"
#include "flight/position_ctrl.h"
#include "sensors/rangefinder.h"

#include "kalman_filter.h"

controller_t attitude_x_controller;
controller_t attitude_y_controller;
controller_t attitude_z_controller;
controller_t vel_x_controller;
controller_t vel_y_controller;
controller_t vel_z_controller;
controller_t attitude_yaw_controller;
controller_t vel_controller; 
controller_t height_controller; 

static float throttle_init = 0.3;
// float height_setpoint = 0.50;
// float vel_setpoint = 0.20;
// float thrust_range = 0.005;
float height_error_range = 0.02;
float vel_error_range = 0.01;
// float limit = 1;

void height_controller_init(controller_t * controller, int axis)
{
    memset(controller, 0, sizeof(controller_t));
    if(axis == 0)
    {
        controller->pid.P = 1;
        controller->pid.I = 0.0;
        controller->pid.D = 0;

        controller->pid.Error1 = 0.0;
        controller->pid.Error2 = 0.0;
        controller->pid.iError = 0.0;

        controller->setpoint = 0.7;
        controller->output_min = -9999;
        controller->output_max = 9999;

        controller->input_error_range = height_error_range;

    }
    if(axis == 1)
    {
        controller->pid.P = 1;
        controller->pid.I = 0.0;
        controller->pid.D = 0;

        controller->pid.Error1 = 0.0;
        controller->pid.Error2 = 0.0;
        controller->pid.iError = 0.0;

        controller->setpoint = 0;
        controller->output_min = -9999;
        controller->output_max = 9999;

        controller->input_error_range = height_error_range;
    }
    if(axis == 2)
    {
        controller->pid.P = 1;
        controller->pid.I = 0.0;
        controller->pid.D = 0;

        controller->pid.Error1 = 0.0;
        controller->pid.Error2 = 0.0;
        controller->pid.iError = 0.0;

        controller->setpoint = 1;
        controller->output_min = -99999;
        controller->output_max = 99999;

        controller->input_error_range = height_error_range;

    }
}

void vel_controller_init(controller_t * controller, int axis)
{
    memset(controller, 0, sizeof(controller_t));
    if(axis == 0)
    {
        controller->pid.P = 10;
        controller->pid.I = 0.0;
        controller->pid.D = 0;

        controller->pid.Error1 = 0.0;
        controller->pid.Error2 = 0.0;
        controller->pid.iError = 0.0;

        controller->setpoint = 0;
        controller->throttle = 0;

        controller->output_min = -70;
        controller->output_max = 70;

        controller->input_error_range = vel_error_range;
    }
    if(axis == 1)
    {
        controller->pid.P = -10;
        controller->pid.I = 0.0;
        controller->pid.D = 0;

        controller->pid.Error1 = 0.0;
        controller->pid.Error2 = 0.0;
        controller->pid.iError = 0.0;

        controller->setpoint = 0;
        controller->throttle = 0;

        controller->output_min = -70;
        controller->output_max = 70;

        controller->input_error_range = vel_error_range;

    }
    if(axis == 2)
    {
        controller->pid.P = 0.4;
        controller->pid.I = 0.0;
        controller->pid.D = 0;

        controller->pid.Error1 = 0.0;
        controller->pid.Error2 = 0.0;
        controller->pid.iError = 0.0;

        controller->setpoint = 0;
        controller->throttle = 0;

        controller->output_min = -0.3;
        controller->output_max = 0.65;

        controller->input_error_range = vel_error_range;

    }
}


void Controller_Init(void)
{
    height_controller_init(&attitude_x_controller, 0);
    height_controller_init(&attitude_y_controller, 1);
    height_controller_init(&attitude_z_controller, 2);
    vel_controller_init(&vel_x_controller, 0);
    vel_controller_init(&vel_y_controller, 1);
    vel_controller_init(&vel_z_controller, 2);
    // Position_init();
    // attitude_init(&attitude_controller);
}

float pid_controller(float process_value, controller_t *controller, float I_limit) //增量式pid计算
{
    controller->pid.iError = controller->setpoint - process_value;  // 当前误差
    if(controller->pid.iError < controller->input_error_range && controller->pid.iError > -controller->input_error_range)
    {
        controller->pid.iError = 0;
    }
    controller->pid.DiError = controller->pid.iError - controller->pid.Error1;
    controller->pid.IiError += controller->pid.iError;
    controller->pid.IiError = constrainf(controller->pid.IiError, -I_limit, I_limit);

    controller->output = controller->pid.P * controller->pid.iError 
                        + controller->pid.I * controller->pid.IiError 
                        + controller->pid.D * controller->pid.DiError;

    // controller->output = controller->pid.P * (controller->pid.iError - controller->pid.Error1)
    //                      + controller->pid.I * controller->pid.iError
    //                      + controller->pid.D * (controller->pid.iError - 2 * controller->pid.Error1 + controller->pid.Error2);

    controller->pid.Error2 = controller->pid.Error1;
    controller->pid.Error1 = controller->pid.iError;

    controller->output = constrainf(controller->output, controller->output_min, controller->output_max);
    
    return controller->output;
}

void adjust_velocity(kalman_filter_t *filter)
{
    float voutputx = pid_controller(attitude_controller.Error_x_filter, &vel_x_controller, 0.5);
    float voutputy = pid_controller(attitude_controller.Error_y_filter, &vel_y_controller, 0.5);
    float voutputz = pid_controller(filter->X_Hat_current->element[1], &vel_z_controller, 0.5);
    
    vel_x_controller.throttle = voutputx;
    vel_y_controller.throttle = voutputy;
    vel_z_controller.throttle = voutputz + throttle_init;
}

void adjust_height(kalman_filter_t *filter)
{
    float outputx = pid_controller(attitude_controller.r_x_lowpassfilter, &attitude_x_controller, 0.5);
    float outputy = pid_controller(attitude_controller.r_y_lowpassfilter, &attitude_y_controller, 0.5);
    float outputz = pid_controller(filter->X_Hat_current->element[0], &attitude_z_controller, 0.5);

    vel_x_controller.setpoint = outputx;
    vel_y_controller.setpoint = outputy;
    vel_z_controller.setpoint = outputz;
    adjust_velocity(&kalman_filter1); 
}

// void Position_yaw_ctrl(attitude_ctrl_t * controller)
// {
//     float output_yaw =  pid_controller(controller->r_Yaw, &attitude_yaw_controller, 50);
// }

void Update_PID_Velocity(timeUs_t currentTimeUs) //500Hz
{
    UNUSED(currentTimeUs);
    adjust_velocity(&kalman_filter1);
}
void Update_PID_Height(timeUs_t currentTimeUs) //200Hz
{
    // UNUSED(currentTimeUs);
    static timeUs_t lastTimeUs = 0;
    // uint32_t ProcessTimeUs = micros();
    float dTime = (currentTimeUs - lastTimeUs)*1e-6f;
    // Lowpass_Filter(&attitude_controller, 0.45, 0.45, 0);
    attitude_controller.dt = dTime;
    adjust_height(&kalman_filter1);
    // attitude_controller.dtHz = (micros() - ProcessTimeUs) * 1e-3f;
    lastTimeUs = currentTimeUs;

}

float Get_Height_PID_Output(int n)
{
    switch(n){
        case 0:
            return attitude_x_controller.output;
        case 1:
            return attitude_y_controller.output;
        case 2:
            return attitude_z_controller.output;
        default:
            return 0;
    }
}
float Get_Velocity_PID_Output(int n)
{
    switch(n){
    case 0:
        return vel_x_controller.output;
    case 1:
        return vel_y_controller.output;
    case 2:
        return vel_z_controller.output;
    default:
        return 0;
    }
}

float Get_Velocity_throttle(int n)
{
    switch(n){
    case 0:
        return vel_x_controller.throttle;
    case 1:
        return vel_y_controller.throttle;
    case 2:
        return vel_z_controller.throttle;
    default:
        return 0;
    }
    // return vel_controller.throttle;
}

float Get_Height_PID_Error(void)
{
    return height_controller.pid.iError;
}
