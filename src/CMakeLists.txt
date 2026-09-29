# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

idf_component_register(
    SRCS "main.c" "board_hw.c" "vehicle_state.c" "canbox_proto.c"
    INCLUDE_DIRS "."
    REQUIRES driver freertos esp_ringbuf esp_system
)