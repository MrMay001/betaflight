#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include"flight/position_ctrl.h"

#include "common/maths.h"
#include "alt_ctrl.h"
#include "kalman_filter.h"

attitude_ctrl_t attitude_controller;

void attitude_init(attitude_ctrl_t * controller)
{
    controller->roll = 0;
    controller->pitch = 0;
    controller->yaw = 0;

    controller->sum = 0;
    controller->altitude_thrust = 0;
}










float Get_vrpn_x(void)
{
    return attitude_controller.r_x;
}

float Get_vrpn_y(void)
{
    return attitude_controller.r_y;
}

float Get_vrpn_z(void)
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
    return attitude_controller.r_Yaw;
}