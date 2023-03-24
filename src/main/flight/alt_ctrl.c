#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "common/maths.h"

#include "drivers/time.h"

#include "flight/alt_ctrl.h"
#include "fc/runtime_config.h"
#include "sensors/rangefinder.h"

#include "kalman_filter.h"

attitude_send_t attitude_send;
attitude_ctrl_t attitude_controller;

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
float height_error_range = 0.02;
float vel_error_range = 0.01;

void attitude_controller_init(attitude_ctrl_t * ctrl)
{
    ctrl->altitude_thrust = 0;
    ctrl->error_angle = 0;
    ctrl->error_angle_output = 0;
    ctrl->Error_x = 0;
    ctrl->Error_x_filter = 0;
    ctrl->Error_x_filter_last = 0;
    ctrl->Error_y = 0;
    ctrl->Error_y_filter = 0;
    ctrl->Error_y_filter_last = 0;

    ctrl->filter_dt = 0;
    ctrl->pidupdate_dt = 0;
    ctrl->pitch = 0;
    ctrl->r_Pitch = 0;
    ctrl->r_Roll = 0;
    ctrl->r_x = 0;
    ctrl->r_x_last = 0;
    ctrl->r_x_lowpassfilter = 0;
    ctrl->r_x_lowpassfilter_last = 0;

    ctrl->r_y = 0;
    ctrl->r_y_last = 0;
    ctrl->r_y_lowpassfilter = 0;
    ctrl->r_y_lowpassfilter_last = 0;
    ctrl->r_z = 0;
    ctrl->r_z_last = 0;
    ctrl->sum = 0;
    ctrl->sum1 = 0;
    ctrl->yaw = 0;
}
void position_controller_init(controller_t * controller, int axis)
{
    memset(controller, 0, sizeof(controller_t));
    if(axis == 0)
    {
        controller->pid.P = 0.85;
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
        controller->pid.P = 1.1;
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
        controller->pid.P = -8.5;
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
        controller->pid.P = 0.04;
        controller->pid.I = 0.01;
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
    attitude_controller_init(&attitude_controller);
    position_controller_init(&attitude_x_controller, 0);
    position_controller_init(&attitude_y_controller, 1);
    position_controller_init(&attitude_z_controller, 2);
    vel_controller_init(&vel_x_controller, 0);
    vel_controller_init(&vel_y_controller, 1);
    vel_controller_init(&vel_z_controller, 2);
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

    controller->pid.Error2 = controller->pid.Error1;
    controller->pid.Error1 = controller->pid.iError;

    controller->output = constrainf(controller->output, controller->output_min, controller->output_max);
    
    return controller->output;
}

// RC = 1.0 / (2.0 * PI * cutoff_freq) 
// alpha = 1.0 / (1.0 + RC * sample_rate)
void Lowpass_Filter(attitude_ctrl_t * ctrl, float alphax, float alphay, int n)  //filter
{
    UNUSED(n);
    //ctrl->r_x_filter = alphax * ctrl->r_x + (1 - alphax) * ctrl->r_x_filter
    ctrl->Error_x = (ctrl->r_x - ctrl->r_x_last) / ctrl->filter_dt;
    ctrl->Error_y = (ctrl->r_y - ctrl->r_y_last) / ctrl->filter_dt;

    ctrl->Error_x_filter = ctrl->Error_x_filter_last + alphax * (ctrl->Error_x - ctrl->Error_x_filter_last);
    ctrl->Error_y_filter = ctrl->Error_y_filter_last + alphay * (ctrl->Error_y - ctrl->Error_y_filter_last);

    ctrl->Error_x_filter_last = ctrl->Error_x_filter;
    ctrl->Error_y_filter_last = ctrl->Error_y_filter;
    ctrl->r_x_last = ctrl->r_x;
    ctrl->r_y_last = ctrl->r_y;

 //   ctrl->Error_z_filter = ctrl->Error_z_filter + alphay * (ctrl->Error_z - ctrl->Error_z_filter);
}

//Lowpass_Filter
void Update_Lowpass_Filter(timeUs_t currentTimeUs)
{
    static uint64_t lasttime = 0;
    float dt = (currentTimeUs - lasttime) * 1e-6f;
    attitude_controller.filter_dt = dt;
    
    Lowpass_Filter(&attitude_controller, 0.3, 0., 0);//lowpass_filter
 

    lasttime = currentTimeUs;

}

//
void adjust_position(kalman_filter_t *filter)
{
    float outputx = pid_controller(attitude_controller.r_x, &attitude_x_controller, 0.5);
    float outputy = pid_controller(attitude_controller.r_y, &attitude_y_controller, 0.5);
    float outputz = pid_controller(filter->X_Hat_current->element[0], &attitude_z_controller, 0.5);

    // vel_x_controller.setpoint = outputx;
    // vel_y_controller.setpoint = outputy;
    vel_x_controller.setpoint = outputx;
    vel_y_controller.setpoint = outputy; 
    vel_z_controller.setpoint = 1.0;
    adjust_velocity(&kalman_filter1); 
}

void adjust_velocity(kalman_filter_t *filter)
{
    float voutputx = pid_controller(attitude_controller.Error_x_filter, &vel_x_controller, 0.5);
    float voutputy = pid_controller(attitude_controller.Error_y_filter, &vel_y_controller, 0.5);
    float voutputz = pid_controller(filter->X_Hat_current->element[1], &vel_z_controller, 0.1);
    
    vel_x_controller.throttle = voutputx;
    vel_y_controller.throttle = voutputy;
    vel_z_controller.throttle = voutputz + throttle_init;
}


void Update_PID_Velocity(timeUs_t currentTimeUs) //500Hz
{
    UNUSED(currentTimeUs);
    adjust_velocity(&kalman_filter1);
}
void Update_PID_Position(timeUs_t currentTimeUs) //200Hz
{
    // UNUSED(currentTimeUs);
    static timeUs_t lastTimeUs = 0;
    float dTime = (currentTimeUs - lastTimeUs)*1e-6f;
    attitude_controller.pidupdate_dt = dTime;
    if(FLIGHT_MODE(POSITION_HOLD_MODE))
    {
        attitude_x_controller.setpoint = attitude_x_controller.setpoint_input;
        attitude_y_controller.setpoint = attitude_y_controller.setpoint_input;
    }
    else{
        attitude_x_controller.setpoint = 0;
        attitude_y_controller.setpoint = 0;
    }
    adjust_position(&kalman_filter1);

    lastTimeUs = currentTimeUs;

}

float Get_Height_PID_Output(int n)  //Vx,Vy,Vz_Setpoint
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
float Get_Velocity_PID_Output(int n) //Roll,Pitch,throttle_Setpoint
{
    switch(n){
    case 0:
        return vel_x_controller.output/180*3.1415926;
    case 1:
        return vel_y_controller.output/180*3.1415926;
    case 2:
        return vel_z_controller.output;
    default:
        return 0;
    }
}

float Get_Velocity_throttle(int n)  //Roll,Pitch,throttle_Setpoint
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
}

float Get_Position_LpFiter(int n) //x,y,z_true
{
    switch(n){
    case 0:
        return attitude_controller.r_x_lowpassfilter;
    case 1:
        return attitude_controller.r_y_lowpassfilter;
    case 2:
        return kalman_filter1.X_Hat_current->element[0];
    default:
        return 0;
    }
}

float Get_Velocity_LpFiter(int n) //Vx,Vy,Vz_true
{
    switch(n){
    case 0:
        return attitude_controller.Error_x_filter;
    case 1:
        return attitude_controller.Error_y_filter;
    case 2:
        return kalman_filter1.X_Hat_current->element[1];
    default:
        return 0;
    }
}