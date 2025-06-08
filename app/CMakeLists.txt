cmake_minimum_required(VERSION 3.5)

# Set ESP-IDF path and target before including project.cmake
if(NOT DEFINED ENV{IDF_PATH})
    message(FATAL_ERROR "IDF_PATH is not set in environment")
endif()

set(SUPPORTED_TARGETS esp32)
set(IDF_TARGET esp32)

# Include ESP-IDF's project.cmake first
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Define project after including project.cmake
project(hexapod_esp32 C CXX)

# Set component dirs 
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/components)

# Configure build flags for both C and C++
idf_build_set_property(COMPILE_OPTIONS 
    "-Wno-error=format;-Wall;-Wextra" APPEND)

# C++ specific flags
idf_build_set_property(COMPILE_OPTIONS 
    "$<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>;$<$<COMPILE_LANGUAGE:CXX>:-fexceptions>;$<$<COMPILE_LANGUAGE:CXX>:-frtti>" 
    APPEND)

# Component configuration
idf_build_set_property(COMPILE_DEFINITIONS 
    "-DCONFIG_FREERTOS_HZ=1000;-DLOG_LOCAL_LEVEL=ESP_LOG_INFO" 
    APPEND)
