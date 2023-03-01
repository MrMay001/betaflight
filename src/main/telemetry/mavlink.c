/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * telemetry_mavlink.c
 *
 * Author: Konstantin Sharlaimov
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#if defined(USE_TELEMETRY_MAVLINK)

#include "build/debug.h"
#include "build/build_config.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/color.h"
#include "common/utils.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/sensor.h"
#include "drivers/time.h"
#include "drivers/light_led.h"

#include "config/config.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/position.h"
#include "flight/alt_ctrl.h"
#include "flight/kalman_filter.h"
#include "flight/position_ctrl.h"

#include "io/serial.h"
#include "io/gimbal.h"
#include "io/gps.h"
#include "io/ledstrip.h"

#include "rx/rx.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/barometer.h"
#include "sensors/boardalignment.h"
#include "sensors/battery.h"
#include "sensors/rangefinder.h"

#include "telemetry/telemetry.h"
#include "telemetry/mavlink.h"

// mavlink library uses unnames unions that's causes GCC to complain if -Wpedantic is used
// until this is resolved in mavlink library - ignore -Wpedantic for mavlink code
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "common/mavlink.h"
#pragma GCC diagnostic pop

#define TELEMETRY_MAVLINK_INITIAL_PORT_MODE MODE_RXTX
#define TELEMETRY_MAVLINK_MAXRATE 100
#define TELEMETRY_MAVLINK_DELAY ((100 * 10) / TELEMETRY_MAVLINK_MAXRATE) //1000/100us=0.01ms

extern uint16_t rssi; // FIXME dependency on mw.c

static serialPort_t *mavlinkPort = NULL;
static const serialPortConfig_t *portConfig;

static bool mavlinkTelemetryEnabled =  false;
static portSharing_e mavlinkPortSharing;

/* MAVLink datastream rates in Hz */
static const uint8_t mavRates[] = {
    [MAV_DATA_STREAM_EXTENDED_STATUS] = 2, //2Hz
    [MAV_DATA_STREAM_RC_CHANNELS] = 5, //5Hz
    [MAV_DATA_STREAM_POSITION] = 10, //100Hz
    [MAV_DATA_STREAM_EXTRA1] = 100, //10Hz
    [MAV_DATA_STREAM_EXTRA2] = 1, //100Hz
    [MAV_DATA_STREAM_EXTRA3] = 5
};

#define MAXSTREAMS ARRAYLEN(mavRates)

static uint8_t mavTicks[MAXSTREAMS];
static mavlink_message_t mavMsg;
static uint8_t mavBuffer[MAVLINK_MAX_PACKET_LEN];
//static uint32_t lastMavlinkMessage = 0;
static uint32_t mavlinkstate_position = 0;
//static uint8_t wifi_uart_baud = 1;

//串口接收触发函数
static void mavlinkReceive(uint16_t c, void* data) {

    UNUSED(data);
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t mav_type;
    uint8_t mav_autopilot;
    uint8_t mav_basemode;
    uint32_t mav_custommode;
    uint8_t mav_systemstatus;
    uint8_t mav_version;

    static bool state1 = 0;
    static bool state = 0;
    

    if (mavlink_parse_char(MAVLINK_COMM_0, (uint8_t)c, &msg, &status)) {

        switch(msg.msgid) {
            // receive heartbeat
            case 0: {
                mavlink_heartbeat_t command;
                mavlink_msg_heartbeat_decode(&msg,&command);
                mav_custommode = command.custom_mode;
                mav_type = command.type;
                mav_autopilot = command.autopilot;
                mav_basemode = command.base_mode;
                mav_systemstatus = command.custom_mode;
                mav_version = command.mavlink_version;
                ledSet(0, state); 
                state = !state;
                break;
            }
            // setpoint command
            case 81: {
                mavlink_manual_setpoint_t command;
                mavlink_msg_manual_setpoint_decode(&msg,&command);
                attitude_controller.altitude_thrust = -command.thrust * 100;
                attitude_controller.roll = command.roll;   //maybe need normalization but this should be done in the JeVois
                attitude_controller.pitch = -command.pitch;
                attitude_controller.yaw = command.yaw;

                // DEBUG_SET(DEBUG_UART,1,command.time_boot_ms);	
                // DEBUG_SET(DEBUG_UART,3,uart_altitude);
                // DEBUG_SET(DEBUG_COMMAND,0,uart_altitude);
                // DEBUG_SET(DEBUG_COMMAND,1,uart_roll / 3.14 * 180);
                // DEBUG_SET(DEBUG_COMMAND,2,uart_pitch / 3.14 * 180);
                // DEBUG_SET(DEBUG_COMMAND,3,uart_yaw / 3.14 * 180);
                break;
            }
            case 102:{
                mavlink_vision_position_estimate_t command;
                mavlink_msg_vision_position_estimate_decode(&msg,&command);
                attitude_controller.dt = micros();
                attitude_controller.r_x = command.x;
                attitude_controller.r_y = command.y;
                attitude_controller.r_z = command.z;
                attitude_controller.r_Roll = command.roll;
                attitude_controller.r_Pitch = command.pitch;
                attitude_controller.r_Yaw = command.yaw;
                attitude_controller.sum++;
                ledSet(1, state1); 
                state1 = !state1;
                break;
            }
            // case 111:{
            //     ledSet(0, state); 
            //     state = !state;
            //     break;
            // }
            default:
                break;
        }
    }
}


static int mavlinkStreamTrigger(enum MAV_DATA_STREAM streamNum)
{
    uint8_t rate = (uint8_t) mavRates[streamNum];
    if (rate == 0) {
        return 0;
    }

    if (mavTicks[streamNum] == 0) {
        // we're triggering now, setup the next trigger point
        if (rate > TELEMETRY_MAVLINK_MAXRATE) {
            rate = TELEMETRY_MAVLINK_MAXRATE;
        }

        mavTicks[streamNum] = (TELEMETRY_MAVLINK_MAXRATE / rate);
        return 1;
    }

    // count down at TASK_RATE_HZ
    mavTicks[streamNum]--;
    return 0;
}


static void mavlinkSerialWrite(uint8_t * buf, uint16_t length)
{
    for (int i = 0; i < length; i++)
        serialWrite(mavlinkPort, buf[i]);
}

static int16_t headingOrScaledMilliAmpereHoursDrawn(void)
{
    if (isAmperageConfigured() && telemetryConfig()->mavlink_mah_as_heading_divisor > 0) {
        // In the Connex Prosight OSD, this goes between 0 and 999, so it will need to be scaled in that range.
        return getMAhDrawn() / telemetryConfig()->mavlink_mah_as_heading_divisor;
    }
    // heading Current heading in degrees, in compass units (0..360, 0=north)
    return DECIDEGREES_TO_DEGREES(attitude.values.yaw);
}


void freeMAVLinkTelemetryPort(void)
{
    closeSerialPort(mavlinkPort);
    mavlinkPort = NULL;
    mavlinkTelemetryEnabled = false;
}

void initMAVLinkTelemetry(void)
{
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_MAVLINK);
    mavlinkPortSharing = determinePortSharing(portConfig, FUNCTION_TELEMETRY_MAVLINK);
}

void configureMAVLinkTelemetryPort(void)
{
    if (!portConfig) {
        return;
    }

    baudRate_e baudRateIndex = portConfig->telemetry_baudrateIndex;
    if (baudRateIndex == BAUD_AUTO) {
        // default rate for minimOSD
        baudRateIndex = BAUD_230400;
    }
    else
    {
        baudRateIndex = BAUD_230400;
    }

    mavlinkPort = openSerialPort(portConfig->identifier, FUNCTION_TELEMETRY_MAVLINK, mavlinkReceive, NULL, baudRates[baudRateIndex], TELEMETRY_MAVLINK_INITIAL_PORT_MODE, SERIAL_STOPBITS_1);

    if (!mavlinkPort) {
        return;
    }

    mavlinkTelemetryEnabled = true;
    if(mavlinkstate_position < 1)
    {
        WifiInitHardware_Esp8266();
        mavlinkstate_position++;
    }
}

void checkMAVLinkTelemetryState(void)
{
    if (portConfig && telemetryCheckRxPortShared(portConfig, rxRuntimeState.serialrxProvider)) {
        if (!mavlinkTelemetryEnabled && telemetrySharedPort != NULL) {
            mavlinkPort = telemetrySharedPort;
            mavlinkTelemetryEnabled = true;
        }
    } else {
        bool newTelemetryEnabledValue = telemetryDetermineEnabledState(mavlinkPortSharing);

        if (newTelemetryEnabledValue == mavlinkTelemetryEnabled) {
            return;
        }

        if (newTelemetryEnabledValue)
            configureMAVLinkTelemetryPort();
        else
            freeMAVLinkTelemetryPort();
    }
}


void mavlinkSendAttitude(void) //ID 30
{
    uint16_t msgLength;
    mavlink_msg_attitude_pack(0, 200, &mavMsg,
        // time_boot_ms Timestamp (milliseconds since system boot)
        millis(),
        // roll Roll angle (rad)
        DECIDEGREES_TO_RADIANS(attitude.values.roll),
        // pitch Pitch angle (rad)
        DECIDEGREES_TO_RADIANS(-attitude.values.pitch),
        // yaw Yaw angle (rad)
        DECIDEGREES_TO_RADIANS(attitude.values.yaw),
        // rollspeed Roll angular speed (rad/s)
        gyro.gyroADCf[0],
        // pitchspeed Pitch angular speed (rad/s)
        gyro.gyroADCf[1],
        // yawspeed Yaw angular speed (rad/s)
        gyro.gyroADCf[2]);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

void mavlinksendAltitude(void) //ID 141
{
    uint16_t msgLength;
    float mavAltitude_Measure = 0;
    float mavVel_Hat_current = 0;
    float mavVel_Measure = 0;
    float mavAltitude_Hat_current = 0;
    float mavPID_vel_output = 0;
    float mavPID_height_output = 0;
    // float mav_vel_throttle = 0;

    
    mavVel_Measure = Get_Acc_bias_kalman(); //速度测量值  (airspeed)
    mavVel_Hat_current = Get_Vel_Kalman(); //速度最优估计值 (groundspeed)
    mavAltitude_Measure = rangefinderGetLatestAltitude(); //高度测量值 (altitude)
    mavAltitude_Hat_current = Get_Alt_Kalman(); //高度最优估计值 (climb)
    mavPID_vel_output = Get_Velocity_PID_Output(); //获取内环pid结果
    mavPID_height_output = Get_Height_PID_Output(); //获取外环pid结果
    // mav_vel_throttle = Get_Velocity_throttle();

    mavlink_msg_altitude_pack(0, 200, &mavMsg,
    millis(),
    mavVel_Measure,
    mavVel_Hat_current,
    mavAltitude_Measure,
    mavAltitude_Hat_current,
    mavPID_vel_output,
    mavPID_height_output
    );
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

void mavlinkSendHUDAndHeartbeat(void) //ID 74
{
   uint16_t msgLength;
    float mavAltitude = 0;
    float mavGroundSpeed = 0;
    float mavAirSpeed = 0;
    float mavClimbRate = 0;

#if defined(USE_GPS)
    // use ground speed if source available
    if (sensors(SENSOR_GPS)) {
        mavGroundSpeed = gpsSol.groundSpeed / 100.0f;
    }
#endif

    mavAltitude = getEstimatedAltitudeCm() / 100.0;

    mavlink_msg_vfr_hud_pack(0, 200, &mavMsg,
        // airspeed Current airspeed in m/s
        mavAirSpeed,
        // groundspeed Current ground speed in m/s
        mavGroundSpeed,
        // heading Current heading in degrees, in compass units (0..360, 0=north)
        headingOrScaledMilliAmpereHoursDrawn(),
        // throttle Current throttle setting in integer percent, 0 to 100
        scaleRange(constrain(rcData[THROTTLE], PWM_RANGE_MIN, PWM_RANGE_MAX), PWM_RANGE_MIN, PWM_RANGE_MAX, 0, 100),
        // alt Current altitude (MSL), in meters, if we have sonar or baro use them, otherwise use GPS (less accurate)
        mavAltitude,
        // climb Current climb rate in meters/second
        mavClimbRate);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);


    uint8_t mavModes = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    if (ARMING_FLAG(ARMED))
        mavModes |= MAV_MODE_FLAG_SAFETY_ARMED;

    uint8_t mavSystemType;
    switch (mixerConfig()->mixerMode)
    {
        case MIXER_TRI:
            mavSystemType = MAV_TYPE_TRICOPTER;
            break;
        case MIXER_QUADP:
        case MIXER_QUADX:
        case MIXER_Y4:
        case MIXER_VTAIL4:
            mavSystemType = MAV_TYPE_QUADROTOR;
            break;
        case MIXER_Y6:
        case MIXER_HEX6:
        case MIXER_HEX6X:
            mavSystemType = MAV_TYPE_HEXAROTOR;
            break;
        case MIXER_OCTOX8:
        case MIXER_OCTOFLATP:
        case MIXER_OCTOFLATX:
            mavSystemType = MAV_TYPE_OCTOROTOR;
            break;
        case MIXER_FLYING_WING:
        case MIXER_AIRPLANE:
        case MIXER_CUSTOM_AIRPLANE:
            mavSystemType = MAV_TYPE_FIXED_WING;
            break;
        case MIXER_HELI_120_CCPM:
        case MIXER_HELI_90_DEG:
            mavSystemType = MAV_TYPE_HELICOPTER;
            break;
        default:
            mavSystemType = MAV_TYPE_GENERIC;
            break;
    }

    // Custom mode for compatibility with APM OSDs
    uint8_t mavCustomMode = 1;  // Acro by default

    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
        mavCustomMode = 0;      //Stabilize
        mavModes |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    }

    uint8_t mavSystemState = 0;
    if (ARMING_FLAG(ARMED)) {
        if (failsafeIsActive()) {
            mavSystemState = MAV_STATE_CRITICAL;
        }
        else {
            mavSystemState = MAV_STATE_ACTIVE;
        }
    }
    else {
        mavSystemState = MAV_STATE_STANDBY;
    }

    mavlink_msg_heartbeat_pack(0, 200, &mavMsg,
        // type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
        mavSystemType,
        // autopilot Autopilot type / class. defined in MAV_AUTOPILOT ENUM
        MAV_AUTOPILOT_GENERIC,
        // base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
        mavModes,
        // custom_mode A bitfield for use for autopilot-specific flags.
        mavCustomMode,
        // system_status System status flag, see MAV_STATE ENUM
        mavSystemState);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

void mavlinkSendHeartbeat(void)  //ID 0
{
    uint16_t msgLength;
    uint8_t mavModes = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    if (ARMING_FLAG(ARMED))
        mavModes |= MAV_MODE_FLAG_SAFETY_ARMED;

    uint8_t mavSystemType;
    switch (mixerConfig()->mixerMode)
    {
        case MIXER_TRI:
            mavSystemType = MAV_TYPE_TRICOPTER;
            break;
        case MIXER_QUADP:
        case MIXER_QUADX:
        case MIXER_Y4:
        case MIXER_VTAIL4:
            mavSystemType = MAV_TYPE_QUADROTOR;
            break;
        case MIXER_Y6:
        case MIXER_HEX6:
        case MIXER_HEX6X:
            mavSystemType = MAV_TYPE_HEXAROTOR;
            break;
        case MIXER_OCTOX8:
        case MIXER_OCTOFLATP:
        case MIXER_OCTOFLATX:
            mavSystemType = MAV_TYPE_OCTOROTOR;
            break;
        case MIXER_FLYING_WING:
        case MIXER_AIRPLANE:
        case MIXER_CUSTOM_AIRPLANE:
            mavSystemType = MAV_TYPE_FIXED_WING;
            break;
        case MIXER_HELI_120_CCPM:
        case MIXER_HELI_90_DEG:
            mavSystemType = MAV_TYPE_HELICOPTER;
            break;
        default:
            mavSystemType = MAV_TYPE_GENERIC;
            break;
    }

    // Custom mode for compatibility with APM OSDs
    uint8_t mavCustomMode = 1;  // Acro by default

    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) {
        mavCustomMode = 0;      //Stabilize
        mavModes |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    }
    if ( FLIGHT_MODE(RANGEFINDER_MODE))
        mavCustomMode = 2; // Alt Hold

    uint8_t mavSystemState = 0;
    if (ARMING_FLAG(ARMED)) {
        if (failsafeIsActive()) {
            mavSystemState = MAV_STATE_CRITICAL;
        }
        else {
            mavSystemState = MAV_STATE_ACTIVE;
        }
    }
    else {
        mavSystemState = MAV_STATE_STANDBY;
    }

    mavlink_msg_heartbeat_pack(0, 200, &mavMsg,
        // type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
        mavSystemType,
        // autopilot Autopilot type / class. defined in MAV_AUTOPILOT ENUM
        MAV_AUTOPILOT_GENERIC,
        // base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
        mavModes,
        // custom_mode A bitfield for use for autopilot-specific flags.
        mavCustomMode,
        // system_status System status flag, see MAV_STATE ENUM
        mavSystemState);
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

void mavlinkLocalPositionNed(void) //ID 32
{
    uint16_t msgLength;
    float r_x = Get_vrpn_x();
    float r_y = Get_vrpn_y();
    float r_z = Get_vrpn_z();

    float r_Roll = Get_vrpn_Roll();
    float r_Pitch = Get_vrpn_Pitch();
    float r_Yaw = Get_vrpn_Yaw();
    mavlink_msg_local_position_ned_pack(0, 200, &mavMsg,
    micros(),
    r_x,
    r_y,
    r_z,
    r_Roll,
    r_Pitch,
    r_Yaw
    );
    msgLength = mavlink_msg_to_send_buffer(mavBuffer, &mavMsg);
    mavlinkSerialWrite(mavBuffer, msgLength);
}

void processMAVLinkTelemetry(void)
{
    // is executed @ TELEMETRY_MAVLINK_MAXRATE rate
    // if (mavlinkStreamTrigger(MAV_DATA_STREAM_EXTENDED_STATUS)) {
    //     mavlinkSendSystemStatus();
    // }

    if (mavlinkStreamTrigger(MAV_DATA_STREAM_EXTRA1)) {
        mavlinkSendAttitude();
        mavlinksendAltitude();
        mavlinkLocalPositionNed();
    }

    if (mavlinkStreamTrigger(MAV_DATA_STREAM_EXTRA2)) {
        mavlinkSendHeartbeat();
    }

}

void handleMAVLinkTelemetry(void)
{
    if (!mavlinkTelemetryEnabled) {
        return;
    }

    if (!mavlinkPort) {
        return;
    }

    processMAVLinkTelemetry();
}

void WifiInitHardware_Esp8266(void)
{
    uint32_t nowtime;
    uint8_t c;
    #ifdef USE_WIFI_ESP8266
        nowtime = millis();
        serialPrint(mavlinkPort, "AT\r\n");
        delay(10);
        nowtime = millis();
        c = serialRead(mavlinkPort);
        while(c != 'O')
        {
            if(millis()-nowtime > 200){
                serialPrint(mavlinkPort, "wait1\r\n");
                break;
            }
        };
        c = 0;

        serialPrint(mavlinkPort, "AT+CWMODE=1\r\n");
        nowtime = millis();
        c = serialRead(mavlinkPort);
        while(c != 'O')
        { 
            if(millis()-nowtime > 500){
                serialPrint(mavlinkPort, "wait2\r\n");     
                break;               
            }  
        };
        c = 0;

//         if(wifi_uart_baud == 0){
//             serialPrint(mavlinkPort, "AT+UART_DEF=230400,8,1,0,0\r\n");
//             nowtime = millis();
//             c = serialRead(mavlinkPort);
//             while(c != 'O')
//             { 
//                 if(millis()-nowtime > 500){
//                     serialPrint(mavlinkPort, "wait0\r\n");     
//                     break;               
//                 }  
//             };
//             delay(1000);
//             delay(1000);
//             delay(1000);
//             delay(1000);
//             delay(1000);
//             delay(1000);
//             serialPrint(mavlinkPort, "AT\r\n");
//             delay(1000);
//             delay(1000);
//             serialPrint(mavlinkPort, "AT\r\n");
//             delay(1000);
//             delay(1000);
//             serialPrint(mavlinkPort, "AT\r\n");
//             delay(1000);
//             delay(1000);
//             serialPrint(mavlinkPort, "AT\r\n");
//             delay(1000);
//             delay(1000);
//             c = 0;
//             wifi_uart_baud++;
//             return;
//         }

        serialPrint(mavlinkPort, "AT+RST\r\n");
        nowtime = millis();
        c = serialRead(mavlinkPort);
        while(c != 'O')
        {
            if(millis()-nowtime > 200){
                serialPrint(mavlinkPort, "wait3\r\n");
                break;
            }
        };
        delay(1000);
        delay(1000);
        delay(1000);
        delay(1000);
        c = 0;

        //serialPrint(mavlinkPort, "AT+CWJAP=\"FAST_0530\",\"13525755559\"\r\n");
        //serialPrint(mavlinkPort, "AT+CWJAP=\"mi12\",\"11111111\"\r\n");
        //serialPrint(mavlinkPort, "AT+CWJAP=\"NeSC\",\"nesc2022\"\r\n");
        serialPrint(mavlinkPort, "AT+CWJAP=\"Redmi_0C5C\",\"12345678\"\r\n");
        nowtime = millis();
        c = serialRead(mavlinkPort);
        while(c != 'W')
        {
            if(millis()-nowtime > 1000){
                serialPrint(mavlinkPort, "wait4\r\n");
                break;
            }      
        };
        c = 0;
        delay(1000);
        delay(1000);
        delay(1000);
        delay(1000);
        delay(1000);
        delay(1000);
        delay(1000);
        delay(1000);

        serialPrint(mavlinkPort,"AT+CIPMUX=0\r\n");
        nowtime = millis();
        c = serialRead(mavlinkPort);
        while(c != 'O')
        {
            if(millis()-nowtime > 2000){
                serialPrint(mavlinkPort, "wait5\r\n");
                break;
            }
        };
        delay(200);
        c = 0;


//        serialPrint(mavlinkPort,"AT+CIPSTART=\"UDP\",\"192.168.254.53\",14555\r\n");
        serialPrint(mavlinkPort,"AT+CIPSTART=\"UDP\",\"192.168.31.142\",14555,9000,0\r\n");
        delay(1000);
        nowtime = millis();
        c = serialRead(mavlinkPort);
        while(c != 'O')
        {
            if(millis()-nowtime > 1000){
                serialPrint(mavlinkPort, "wait6\r\n");
                break;
            }
        };
        delay(1000);
        c = 0;

        serialPrint(mavlinkPort,"AT+CIPMODE=1\r\n");
        nowtime = millis();
        c = serialRead(mavlinkPort);
        while(c != 'O')
        {
            if(millis()-nowtime > 2000){
                serialPrint(mavlinkPort, "wait7\r\n");
                break;
            }
        };
        delay(200);
        c = 0;

        serialPrint(mavlinkPort,"AT+CIPSEND\r\n");
        delay(200);

 //       serialPrint(mavlinkPort,"Connect Success!\r\n");
        nowtime = millis();
#endif
}
#endif
