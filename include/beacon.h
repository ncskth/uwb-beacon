#ifndef BEACON_H
#define BEACON_H
#include <stdint.h>
struct distance_measurement {
    uint8_t id;
    uint32_t distance;
    int32_t position_x; // in mm
    int32_t position_y; // in mm
    int32_t position_z; // in mm
    uint8_t flags;
    uint16_t error; // in mm
};

extern uint8_t node_id;

extern int32_t pos_x;
extern int32_t pos_y;
extern int32_t pos_z;
extern uint8_t purpose;
extern uint8_t positioning_system_status;
#endif