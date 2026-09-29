cmake_minimum_required(VERSION 3.16.0)

# Disable ESP-IDF Git version extraction to fix grabRef.cmake errors
set(GIT_EXECUTABLE "" CACHE FILEPATH "Path to a program." FORCE)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(CANBOX2HU-Serial-Parser)

# Global C17 standard configuration
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)