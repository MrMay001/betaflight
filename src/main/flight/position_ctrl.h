#pragma once

#include "common/time.h"


typedef struct attitude_ctrl
{
    float r_x;
    float r_y;
    float r_z;

    float r_Roll;
    float r_Pitch;
    float r_Yaw;

    float roll;  //rad/s
    float pitch;  //rad/s
    float yaw;   //rad/s

    float altitude_thrust;  //0-1

    uint16_t sum;

    uint32_t dt;
}attitude_ctrl_t;


extern attitude_ctrl_t attitude_controller;


void attitude_init(attitude_ctrl_t * controller);
float Get_vrpn_x(void);
float Get_vrpn_y(void);
float Get_vrpn_z(void);
float Get_vrpn_Roll(void);
float Get_vrpn_Pitch(void);
float Get_vrpn_Yaw(void);