#ifndef BEACON_H
#define BEACON_H

#include <stdint.h>

#define GREEN 0,255,0
#define BLUE 0,0,255
#define RED 255,0,0
#define YELLOW 255,40,0
#define PINK 255, 20, 20
#define PURPLE 180, 0, 80
#define WHITE 255, 255, 255
struct distance_measurement {
    uint8_t id;
    uint32_t distance;
    int32_t position_x; // in mm
    int32_t position_y; // in mm
    int32_t position_z; // in mm
    uint8_t flags;
    uint16_t error; // in mm
};

void set_led(uint8_t r, uint8_t g, uint8_t b);

//persistent
extern uint8_t node_id;
extern float pos_x;
extern float pos_y;
extern float pos_z;
extern uint8_t purpose;
extern uint8_t positioning_system_status;
extern uint8_t wifi_enabled;
#endif