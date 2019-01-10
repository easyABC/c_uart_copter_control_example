#ifndef MSG_TYPE_H
#define MSG_TYPE_H

#include <stdint.h>

#define MSG_TYPE_TRANSMIT           0
#define MSG_TYPE_RECEIVING          1

#define MAX_LEN 100
#define UAV_NUM 3

#define MAX_PAYLOAD_LEN    255

#define TCP_PORT 8888

extern char recv_buf[MAX_PAYLOAD_LEN + 4];
extern char server_ip[20];

enum MessageType
{
    Vehicle_UAV_msg=1, square_trajectory_msg, square_puase_msg, square_continue_msg,
    explore_setpoint_msg, explore_adjust_msg, explore_return_msg,
    platoon_takeoff_msg, platoon_start_msg, platoon_pause_msg, platoon_land_msg,
    UAV_state_msg,
    CAV_state_msg
};

enum CommandType
{
    None_cmd, Platoon_cmd, Explore_cmd, SquareTrack_cmd, Vehicle_to_UAV_cmd
};

typedef struct __DSRC_message
{
    uint8_t message_ID;
    uint8_t dst_ID;
    uint8_t src_ID;
    uint8_t len;   // length of payload

    uint8_t payload[MAX_PAYLOAD_LEN];

} DSRC_message_t;

/*
 * UAV state feedback used for UI display
 *
 */
typedef struct __multicopter_state_t
{

    int system_id;
    int autopilot_id;
    int companion_id;

    uint8_t base_mode;    // indicate arm or not
    uint8_t landed_state; // indicate land or not

    int32_t lat;   // Latitude
    int32_t lon;   // Longitude
    int32_t relative_alt;  // Altitude above ground

    // fused GPS and accelerometers
    int16_t vx;  // Ground X Speed (Latitude, positive north) cm/s
    int16_t vy;  // Ground Y Speed (Longitude, positive east) cm/s
    int16_t vz;  // Ground Z Speed (Altitude, positive down)  cm/s

    int16_t heading; // Current heading in degrees, in compass units (0..360, 0=north)

    // aeronautical frame, NED / north-east-down convention
    int16_t ax;    // X Acceleration    cm/s^2
    int16_t ay;    // Y Acceleration    cm/s^2
    int16_t az;    // Z Acceleration    cm/s^2

} multicopter_state_t;

/*
 *
 *
 */
typedef struct __explore_setpoint_message_t
{

    int32_t lat;   // Latitude
    int32_t lon;   // Longitude
    int32_t relative_alt;  // Altitude above ground

} explore_setpoint_message_t;

/*
 *
 *
 */
typedef struct __explore_adjust_message_t
{
    int32_t offset_x;   // offset x m
    int32_t offset_y;   // offset y m
    int32_t offset_z;   // offset z m

    int32_t offset_yaw;

} explore_adjust_message_t;


/*
 *
 *
 */
typedef struct __platoon_message_t
{
    char platoon_command_type;
} platoon_message_t;


/*
 *
 *
 */
typedef struct Message_vehicle_Basic
{
    int32_t lon;   // Longitude
    int32_t lat;   // Latitude
    int32_t relative_alt;  // Altitude above ground
    int32_t velocity;
    int32_t acceleration;
    int32_t heading;
    uint8_t command1;  // 起飞:1, 定点:2, 降落:3
    uint8_t command2;
    uint8_t command3;
    uint8_t command4;
    uint8_t command5;
}Message_vehicle_Basic_t;

/*
 *
 *
 */
typedef struct BasicSaftyMessage
{
    uint8_t message_ID;
    uint8_t dst_ID;
    uint8_t src_ID;
    uint8_t len;   // length of payload

    Message_vehicle_Basic_t Message_Veh;

    struct BasicSaftyMessage *pBsm_pre;
    struct BasicSaftyMessage *pBsm_next;
}BasicSaftyMessage_st;

int get_system_id(char * msg);

int get_autopilot_id(char * msg);

int get_companion_id(char * msg);

// Latitude
int32_t get_lat(char * msg);

// Longitude
int32_t get_lon(char * msg);

// Altitude above ground
int32_t get_relative_alt(char * msg);

// fused GPS and accelerometers
// Ground X Speed (Latitude, positive north) mm/s
int16_t get_vx(char * msg);

// Ground Y Speed (Longitude, positive east) mm/s
int16_t get_vy(char * msg);

// Ground Z Speed (Altitude, positive down)  mm/s
int16_t get_vz(char * msg);

// Current heading in degrees, in compass units (0..360, 0=north)
int16_t get_heading(char * msg);

// aeronautical frame, NED / north-east-down convention
// X Acceleration    mm/s^2
int16_t get_ax(char * msg);

// Y Acceleration    mm/s^2
int16_t get_ay(char * msg);

int16_t get_az(char * msg);


#endif // MSG_TYPE

