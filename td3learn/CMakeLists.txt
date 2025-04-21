cmake_minimum_required(VERSION 3.10)
project(td3learn VERSION 0.2.0 LANGUAGES CXX)

# Options
option(ENABLE_TIDL "Enable TIDL acceleration" ON)
option(ENABLE_OPENCL "Enable OpenCL acceleration" ON)
option(BUILD_TESTS "Build test suite" OFF)
option(BUILD_EXAMPLES "Build example programs" ON)

# C++ standard requirements
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../app/inc
)

# Find required packages
find_package(Threads REQUIRED)

# Ensure filesystem library is linked
link_libraries(stdc++fs)

# TIDL support with proper paths and fallback
if(ENABLE_TIDL)
    # Allow specifying custom TIDL API paths
    if(NOT DEFINED TIDL_API_DIR)
        message(STATUS "TIDL_API_DIR not specified, searching in common locations...")
        # Try various common installation paths including system paths
        set(TIDL_API_PATHS
            "${CMAKE_CURRENT_SOURCE_DIR}/../tidl/api/tidl_api"
        )
        
        if(TIDL_HEADER_DIR)
            message(STATUS "Found TIDL API header at: ${TIDL_HEADER_DIR}")
            set(TIDL_API_DIR ${TIDL_HEADER_DIR})
        else()
            message(STATUS "TIDL API header not found in standard locations.")
        endif()
    endif()
endif()

# OpenCL support with proper paths and fallback
if(ENABLE_OPENCL)
    # Try to find OpenCL in multiple paths
    if(NOT DEFINED OPENCL_INCLUDE_DIR)
        find_path(OPENCL_INCLUDE_DIR
            ${CMAKE_CURRENT_SOURCE_DIR}/../opencl/api/builtins/include
            ${CMAKE_CURRENT_SOURCE_DIR}/../opencl/api/host/include
            ${CMAKE_CURRENT_SOURCE_DIR}/../opencl/api/packages/ti/opencl
        )
    endif()
endif()

# Source files
file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS "inc/td3learn/*.hpp")

# Main library
add_library(td3learn STATIC ${SOURCES} ${HEADERS})

# Link with required libraries
target_link_libraries(td3learn PRIVATE
    Threads::Threads
    stdc++fs  # Explicitly link with C++ filesystem library
)

# Only link with TIDL if enabled and found
if(ENABLE_TIDL)
    find_library(TIDL_LIBRARY
        NAMES tidl_api tidl
        PATHS 
        ${TIDL_API_DIR}
        ${TIDL_API_DIR}/lib
        /usr/lib
        /usr/local/lib
        /usr/lib64
        /usr/local/lib64
    )
    
    if(TIDL_LIBRARY)
        target_link_libraries(td3learn PRIVATE ${TIDL_LIBRARY})
        message(STATUS "Linking with TIDL library: ${TIDL_LIBRARY}")
    else
        # Try to find another TIDL library name if tidl_api wasn't found
        find_library(TIDL_ALT_LIBRARY
            NAMES libtidl tidl
            PATHS 
            ${TIDL_API_DIR}
            ${TIDL_API_DIR}/lib
            /usr/lib
            /usr/local/lib 
            /usr/lib64
            /usr/local/lib64
        )
        
        if(TIDL_ALT_LIBRARY)
            target_link_libraries(td3learn PRIVATE ${TIDL_ALT_LIBRARY})
            message(STATUS "Linking with alternative TIDL library: ${TIDL_ALT_LIBRARY}")
        else
            message(WARNING "TIDL library not found. Will continue with minimal TIDL support.")
            # If we can't find the library but have the headers, we'll proceed with minimal functionality
            add_definitions(-DTIDL_MINIMAL_SUPPORT)
        endif()
    endif()
endif()

# Only link with OpenCL if enabled and found
if(ENABLE_OPENCL AND DEFINED OPENCL_LIBRARY)
    target_link_libraries(td3learn PRIVATE ${OPENCL_LIBRARY})
    message(STATUS "Linking with OpenCL library: ${OPENCL_LIBRARY}")
endif()

# Build executables
add_executable(td3learn_train src/main.cpp)
target_link_libraries(td3learn_train PRIVATE td3learn stdc++fs)

add_executable(td3learn_deploy src/deploy.cpp)
target_link_libraries(td3learn_deploy PRIVATE td3learn stdc++fs)

add_executable(td3learn_run src/run.cpp)
target_link_libraries(td3learn_run PRIVATE td3learn stdc++fs)

# Installation rules
install(TARGETS td3learn td3learn_train td3learn_deploy td3learn_run
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY inc/td3learn
    DESTINATION include
)

install(DIRECTORY configs
    DESTINATION share/td3learn
)

# Install scripts
install(DIRECTORY tools
    DESTINATION bin
    FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                     GROUP_READ GROUP_EXECUTE
                     WORLD_READ WORLD_EXECUTE
)

# Build examples if requested
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Build tests if requested
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()
