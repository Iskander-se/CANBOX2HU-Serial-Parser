#ifndef VEHICLE_STATE_H
#define VEHICLE_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct {
    uint8_t current_speed_kmh;   //  Current speed in km/h
    bool    speed_over;          // Flag for speed limit violation
    size_t  pdc_payload_len;     // Length of payload (LEN) for frame 0x26
    uint8_t current_speed_limit; // Current speed limit sign (0 = no sign)
} vehicle_state_t;

// Global variable to hold the vehicle state
extern vehicle_state_t g_vehicle;

#endif // VEHICLE_STATE_H