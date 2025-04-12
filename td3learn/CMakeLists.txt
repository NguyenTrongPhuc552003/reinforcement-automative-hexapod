cmake_minimum_required(VERSION 3.10)
project(td3learn VERSION 0.1.0 LANGUAGES CXX)

# Options
option(ENABLE_TIDL "Enable TIDL acceleration" ON)
option(ENABLE_OPENCL "Enable OpenCL acceleration" OFF)
option(BUILD_TESTS "Build test suite" OFF)
option(BUILD_EXAMPLES "Build example programs" ON)

# C++ standard - update to C++17 for std::filesystem
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Include directories - make sure all needed paths are included
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../app/inc # Add path to app/inc for user-space driver headers
)

# Find dependencies
find_package(Threads REQUIRED)

# TIDL support
if(ENABLE_TIDL)
    # Find TIDL package or set include paths manually based on SDK location
    add_definitions(-DENABLE_TIDL)

    # Use the TIDL API in the local environment
    include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/../tidl/api/tidl_api/inc
    )
endif()

# OpenCL support
if(ENABLE_OPENCL)
    # Add compile definition
    add_definitions(-DENABLE_OPENCL)

    # Use the OpenCL API in the local environment
    include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/../opencl/api/builtins/include
    )
endif()

# Source files
file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS "inc/td3learn/*.hpp")

# Main library
add_library(td3learn STATIC ${SOURCES} ${HEADERS})

# Link with required libraries
target_link_libraries(td3learn PRIVATE
    Threads::Threads
)

if(ENABLE_TIDL)
    target_link_libraries(td3learn PRIVATE tidl_api)
endif()

if(ENABLE_OPENCL)
    target_link_libraries(td3learn PRIVATE ${OpenCL_LIBRARIES})
endif()

# Build executables
add_executable(td3learn_train src/main.cpp)
target_link_libraries(td3learn_train PRIVATE td3learn)

add_executable(td3learn_deploy src/deploy.cpp)
target_link_libraries(td3learn_deploy PRIVATE td3learn)

add_executable(td3learn_run src/run.cpp)
target_link_libraries(td3learn_run PRIVATE td3learn)

# Install targets
install(TARGETS td3learn td3learn_train td3learn_deploy td3learn_run
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY inc/td3learn
    DESTINATION inc
)

install(DIRECTORY configs
    DESTINATION share/td3learn
)

install(DIRECTORY tools
    DESTINATION share/td3learn
    FILES_MATCHING PATTERN "*.sh"
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)

# Build examples
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Build tests
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()
