#include <cstring>
#include "msg_type.h"

char recv_buf[MAX_PAYLOAD_LEN+4];
// char server_ip[20] = "192.168.1.100";
char server_ip[20] = "127.0.0.1";

int8_t return_int8_t(char * msg, int offset);

int16_t return_int16_t(char * msg, int offset);

int32_t return_int32_t(char * msg, int offset);

int return_int(char * msg, int offset);


int get_system_id(char * msg)
{
   return return_int(msg,0);
}

int get_autopilot_id(char * msg)
{
    return return_int(msg,sizeof(int));
}

int get_companion_id(char * msg)
{
    return return_int(msg,sizeof(int)*2);
}

int8_t get_base_mode(char * msg)
{
    return return_int8_t(msg,sizeof(int)*3);
}

int8_t get_landed_state(char * msg)
{
    return return_int8_t(msg,sizeof(int)*3 + 1);
}

// Latitude
int32_t get_lat(char * msg)
{
    return return_int32_t(msg,sizeof(int)*3 + 2);
}

// Longitude
int32_t get_lon(char * msg)
{
    return return_int32_t(msg,sizeof(int)*3 + 6);
}

// Altitude above ground
int32_t get_relative_alt(char * msg)
{
    return return_int32_t(msg,sizeof(int)*3 + 10);
}

// fused GPS and accelerometers
// Ground X Speed (Latitude, positive north) mm/s
int16_t get_vx(char * msg)
{
    return return_int16_t(msg,sizeof(int)*3 + 14);
}

// Ground Y Speed (Longitude, positive east) mm/s
int16_t get_vy(char * msg)
{
    return return_int16_t(msg,sizeof(int)*3 + 16);
}

// Ground Z Speed (Altitude, positive down)  mm/s
int16_t get_vz(char * msg)
{
    return return_int16_t(msg,sizeof(int)*3 + 18);
}

// Current heading in degrees, in compass units (0..360, 0=north)
int16_t get_heading(char * msg)
{
    return return_int16_t(msg,sizeof(int)*3 + 20);
}

// aeronautical frame, NED / north-east-down convention
// X Acceleration    mm/s^2
int16_t get_ax(char * msg)
{
    return return_int16_t(msg,sizeof(int)*3 + 22);
}

// Y Acceleration    mm/s^2
int16_t get_ay(char * msg)
{
    return return_int16_t(msg,sizeof(int)*3 + 24);
}

int16_t get_az(char * msg)
{
    return return_int16_t(msg,sizeof(int)*3 + 26);
}



//
int8_t return_int8_t(char * msg, int offset)
{
    int8_t store;
    memcpy(&store, msg+offset, sizeof(int8_t)); //保存数据信息
    return store;
}

int16_t return_int16_t(char * msg, int offset)
{
    int16_t store;
    memcpy(&store, msg+offset, sizeof(int16_t)); //保存数据信息
    return store;
}

int32_t return_int32_t(char * msg, int offset)
{
    int32_t store;
    memcpy(&store, msg+offset, sizeof(int32_t)); //保存数据信息
    return store;
}

int return_int(char * msg, int offset)
{
    int store;
    memcpy(&store, msg+offset, sizeof(int)); //保存数据信息
    return store;
}
