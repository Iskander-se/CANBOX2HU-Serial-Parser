#include "vehicle_state.h"

vehicle_state_t g_vehicle = {
    .current_speed_kmh   = 0,
    .speed_over          = false,
    .pdc_payload_len     = 0,
    .current_speed_limit = 30,
};