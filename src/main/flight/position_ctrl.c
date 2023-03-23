#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include"flight/position_ctrl.h"

#include "common/maths.h"
#include "alt_ctrl.h"
#include "kalman_filter.h"

attitude_ctrl_t attitude_controller;
attitude_send_t attitude_send;

float z_throttle = 0.23;

void attitude_init(attitude_ctrl_t * controller)
{
    controller->r_x = 0;
    controller->r_y = 0;
    controller->r_z = 0;

    controller->r_x_lowpassfilter = 0;
    controller->r_y_lowpassfilter = 0;

    controller->Error_x = 0;
    controller->Error_y = 0;
    controller->Error_z = 0;

    controller->Error_x_filter = 0;
    controller->Error_y_filter = 0;
    controller->Error_z_filter = 0;

    controller->roll = 0;
    controller->pitch = 0;
    controller->yaw = 0;

    controller->sum = 0;
    controller->sum1 = 0;
    controller->sum2 = 0;
    controller->altitude_thrust = 0;
}

void x_controller_init(controller_t * controller)
{
    memset(controller, 0, sizeof(controller_t));
    controller->pid.P = 30;
    controller->pid.I = 0.0;
    controller->pid.D = 0.01;

    controller->pid.Error1 = 0.0;
    controller->pid.Error2 = 0.0;
    controller->pid.iError = 0.0;

    controller->setpoint = 0.0;
    controller->throttle = 0;

    controller->output_min = -150;
    controller->output_max = 150;

    controller->input_error_range = 0;
}
void y_controller_init(controller_t * controller)
{
    memset(controller, 0, sizeof(controller_t));
    controller->pid.P = 30;
    controller->pid.I = 0.0;
    controller->pid.D = 0.01;

    controller->pid.Error1 = 0.0;
    controller->pid.Error2 = 0.0;
    controller->pid.iError = 0.0;

    controller->setpoint = 0.0;
    controller->throttle = 0;

    controller->output_min = -150;
    controller->output_max = 150;

    controller->input_error_range = 0;
}
void z_controller_init(controller_t * controller)
{
    memset(controller, 0, sizeof(controller_t));
    controller->pid.P = 5;
    controller->pid.I = 0.0;
    controller->pid.D = 0.0;

    controller->pid.Error1 = 0.0;
    controller->pid.Error2 = 0.0;
    controller->pid.iError = 0.0;

    controller->setpoint = 0.6;
    controller->throttle = 0;
                                                                 
    controller->output_min = -0.1;
    controller->output_max = 0.15;

    controller->input_error_range = 0;
}

void yaw_controller_init(controller_t * controller)
{
    memset(controller, 0, sizeof(controller_t));
    controller->pid.P = 5;
    controller->pid.I = 0.0;
    controller->pid.D = 0.0;

    controller->pid.Error1 = 0.0;
    controller->pid.Error2 = 0.0;
    controller->pid.iError = 0.0;

    controller->setpoint = 1.8;
    controller->throttle = 0;

    controller->output_min = -100;
    controller->output_max = 100;

    controller->input_error_range = 0;
}

void Attitude_Send_Init(attitude_send_t * attitude_send)
{
    attitude_send->ROLL = 0;
    attitude_send->PITCH = 0;
    attitude_send->YAW = 0;

    attitude_send->ROLL_rate = 0;
    attitude_send->PITCH_rate = 0;
    attitude_send->YAW_rate = 0;

    attitude_send->test_yaw = 0;
}


void Position_init(void)
{
    x_controller_init(&attitude_x_controller);
    y_controller_init(&attitude_y_controller);
    z_controller_init(&attitude_z_controller);
    // yaw_controller_init(&attitude_yaw_controller);
    Attitude_Send_Init(&attitude_send);
}

void Position_x_ctrl(attitude_ctrl_t * controller)
{
    float output_x =  pid_controller(controller->r_x, &attitude_x_controller, 10);

    

    //如果当前速度误差为正，则增加推力
    if(output_x > 1 || output_x < -1)
    {
        attitude_x_controller.throttle = output_x;
    }
    // 如果当前速度误差为负，则减小推力
    else{
        attitude_x_controller.throttle = 0;
    }
}

void Position_y_ctrl(attitude_ctrl_t * controller)
{
    float output_y =  pid_controller(controller->r_y, &attitude_y_controller, 100);

    //如果当前速度误差为正，则增加推力
    if(output_y > 1 || output_y < -1)
    {
        attitude_y_controller.throttle = output_y;
    }
    // 如果当前速度误差为负，则减小推力
    else{
        attitude_y_controller.throttle = 0;
    }
    
}
void Position_z_ctrl(attitude_ctrl_t * controller)
{
    float output_z =  pid_controller(controller->r_z, &attitude_z_controller, 1);

    //如果当前速度误差为正，则增加推力
    if(output_z > 0.003 || output_z < -0.003)
    {
        attitude_z_controller.throttle = output_z + z_throttle;
    }
    // 如果当前速度误差为负，则减小推力
    else{
        attitude_z_controller.throttle = z_throttle;
    }
    
}

// void Position_z_ctrl_vel(attitude_ctrl_t * controller)
// {
//     float output = pid_controller(controller->X_Hat_current->element[1], &vel_controller, 0.5);
    
// }

// void Position_yaw_ctrl(attitude_ctrl_t * controller)
// {
//     float output_yaw =  pid_controller(controller->r_Yaw, &attitude_yaw_controller, 50);

//     //如果当前速度误差为正，则增加推力
//     if(output_yaw > 0.01 || output_yaw < -0.01)
//     {
//         attitude_yaw_controller.throttle = output_yaw;
//     }
//     // // 如果当前速度误差为负，则减小推力
//     else{
//         attitude_yaw_controller.throttle = 0;
//     }
    
// }

// RC = 1.0 / (2.0 * PI * cutoff_freq)
// alpha = 1.0 / (1.0 + RC * sample_rate)
void Lowpass_Filter(attitude_ctrl_t * ctrl, float alphax, float alphay, int n)
{
    UNUSED(n);
    //ctrl->r_x_filter = alphax * ctrl->r_x + (1 - alphax) * ctrl->r_x_filter
    ctrl->r_x_lowpassfilter_last = ctrl->r_x_lowpassfilter;
    ctrl->r_y_lowpassfilter_last = ctrl->r_y_lowpassfilter;
    ctrl->r_x_lowpassfilter = ctrl->r_x_lowpassfilter + alphax * (ctrl->r_x - ctrl->r_x_lowpassfilter);
    ctrl->r_y_lowpassfilter = ctrl->r_y_lowpassfilter + alphay * (ctrl->r_y - ctrl->r_y_lowpassfilter);

 //   ctrl->Error_z_filter = ctrl->Error_z_filter + alphay * (ctrl->Error_z - ctrl->Error_z_filter);
}

void Update_Lowpass_Filter(timeUs_t currentTimeUs)
{
    static uint64_t lasttime = 0;
    float dt = (currentTimeUs - lasttime) * 1e-6f;
    // UNUSED(currentTimeUs);

    Lowpass_Filter(&attitude_controller, 0.45, 0.45, 0);//lowpass_filter
    attitude_controller.Error_x_filter = (attitude_controller.r_x_lowpassfilter - attitude_controller.r_x_lowpassfilter_last) / dt;
    attitude_controller.Error_y_filter = (attitude_controller.r_y_lowpassfilter - attitude_controller.r_y_lowpassfilter_last) / dt;

    lasttime = currentTimeUs;

}

float Get_vrpn_x(void)
{
    return attitude_x_controller.throttle;
}

float Get_vrpn_y(void)
{
    return attitude_y_controller.throttle;
}

float Get_vrpn_z(void)
{
    return attitude_z_controller.throttle;
}

float Get_z_measure(void)
{
    return attitude_controller.r_z;
}

float Get_vrpn_Roll(void)
{
    return attitude_controller.r_Roll;
}

float Get_vrpn_Pitch(void)
{
    return attitude_controller.r_Pitch;
}

float Get_vrpn_Yaw(void)
{
    return attitude_controller.r_Yaw/3.1415926*180;
}