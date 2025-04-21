#!/bin/bash

# Exit on any error
set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Function to print colored messages
log() {
    local color=$1
    shift
    echo -e "${color}$@${NC}"
}

# Verify API directories
verify_api_dirs() {
    log "${YELLOW}" "Verifying API directories..."
    
    # Check TIDL API
    if [ ! -d "${TIDL_API_DIR}/inc" ] || [ ! -f "${TIDL_API_DIR}/inc/tidl/tidl_api.h" ]; then
        log "${RED}" "TIDL API headers not found at ${TIDL_API_DIR}/inc/tidl/tidl_api.h"
        
        # Try to copy from project structure
        if [ -d "/app/tidl/api/tidl_api/inc" ]; then
            log "${YELLOW}" "Copying TIDL API headers from project..."
            mkdir -p ${TIDL_API_DIR}/inc
            cp -r /app/tidl/api/tidl_api/inc/* ${TIDL_API_DIR}/inc/
        else
            log "${RED}" "TIDL API headers not available in project structure"
            return 1
        fi
    fi
    
    # Check OpenCL API
    if [ ! -d "${OPENCL_INCLUDE_DIR}/CL" ] || [ ! -f "${OPENCL_INCLUDE_DIR}/CL/cl.h" ]; then
        log "${RED}" "OpenCL headers not found at ${OPENCL_INCLUDE_DIR}/CL/cl.h"
        
        # Try to copy from project structure
        if [ -d "/app/opencl/api/host/include/CL" ]; then
            log "${YELLOW}" "Copying OpenCL headers from project..."
            mkdir -p ${OPENCL_INCLUDE_DIR}/CL
            cp -r /app/opencl/api/host/include/CL/* ${OPENCL_INCLUDE_DIR}/CL/
        else
            log "${RED}" "OpenCL headers not available in project structure"
            return 1
        fi
    fi
    
    # Check OpenCL builtins
    if [ ! -d "${OPENCL_BUILTINS_DIR}" ]; then
        log "${RED}" "OpenCL builtins not found at ${OPENCL_BUILTINS_DIR}"
        
        # Try to copy from project structure
        if [ -d "/app/opencl/api/builtins/include" ]; then
            log "${YELLOW}" "Copying OpenCL builtins from project..."
            mkdir -p ${OPENCL_BUILTINS_DIR}
            cp -r /app/opencl/api/builtins/include/* ${OPENCL_BUILTINS_DIR}/
        else
            log "${RED}" "OpenCL builtins not available in project structure"
            return 1
        fi
    fi
    
    # Check OpenCL packages
    if [ ! -d "${OPENCL_PACKAGES_DIR}/ti/opencl" ]; then
        log "${RED}" "OpenCL TI packages not found at ${OPENCL_PACKAGES_DIR}/ti/opencl"
        
        # Try to copy from project structure
        if [ -d "/app/opencl/api/packages/ti/opencl" ]; then
            log "${YELLOW}" "Copying OpenCL TI packages from project..."
            mkdir -p ${OPENCL_PACKAGES_DIR}/ti/opencl
            cp -r /app/opencl/api/packages/ti/opencl/* ${OPENCL_PACKAGES_DIR}/ti/opencl/
        else
            log "${RED}" "OpenCL TI packages not available in project structure"
            return 1
        fi
    fi
    
    log "${GREEN}" "API directories verified successfully"
    return 0
}

# Build TD3Learn library and executables
build_td3learn() {
    log "${YELLOW}" "Building TD3Learn library and executables..."
    
    # Create build directory
    mkdir -p /app/td3learn/build
    cd /app/td3learn/build
    
    # Configure with CMake
    cmake .. \
        -DTIDL_API_DIR=${TIDL_API_DIR} \
        -DOPENCL_INCLUDE_DIR=${OPENCL_INCLUDE_DIR} \
        -DOPENCL_BUILTINS_DIR=${OPENCL_BUILTINS_DIR} \
        -DOPENCL_PACKAGES_DIR=${OPENCL_PACKAGES_DIR} || {
        log "${RED}" "CMake configuration failed!"
        return 1
    }
    
    # Build
    make -j$(nproc) || {
        log "${RED}" "Build failed!"
        return 1
    }
    
    log "${GREEN}" "TD3Learn built successfully!"
    return 0
}

# Main execution based on command
case "$1" in
    "build")
        verify_api_dirs || exit 1
        build_td3learn || exit 1
        ;;
    "run")
        verify_api_dirs || exit 1
        if [ ! -f "/app/td3learn/build/td3learn_run" ]; then
            log "${YELLOW}" "TD3Learn executable not found, building first..."
            build_td3learn || exit 1
        fi
        shift
        /app/td3learn/build/td3learn_run "$@"
        ;;
    "test")
        verify_api_dirs || exit 1
        if [ ! -f "/app/td3learn/build/td3learn_run" ]; then
            log "${YELLOW}" "TD3Learn executable not found, building first..."
            build_td3learn || exit 1
        fi
        shift
        /app/td3learn/build/td3learn_run --test "$@"
        ;;
    *)
        # If no specific command, just run the provided command
        verify_api_dirs || exit 1
        exec "$@"
        ;;
esac
