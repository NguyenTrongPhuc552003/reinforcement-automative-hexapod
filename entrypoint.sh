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

# Common make parameters for kernel module
MAKE_PARAMS="ARCH=${ARCH} CROSS_COMPILE=${CROSS_COMPILE} KERNEL_DIR=${KERNEL_DIR}"

case "$1" in
    "module")
        cd /build/module
        log "${GREEN}" "Building kernel module..."
        
        # Configure build based on build type
        if [ "$BUILD_TYPE" = "debug" ]; then
            EXTRA_CFLAGS="-DDEBUG -I/build/module/inc"
            log "${GREEN}" "Debug build enabled"
        else
            EXTRA_CFLAGS="-DNDEBUG -O2 -I/build/module/inc"
            log "${GREEN}" "Release build enabled"
        fi
        
        # Build the module
        make ${MAKE_PARAMS} EXTRA_CFLAGS="$EXTRA_CFLAGS" || {
            log "${RED}" "Build failed!"
            exit 1
        }
        
        log "${GREEN}" "Copying built modules to deploy directory..."
        mkdir -p /build/deploy
        cp /build/module/*.ko /build/deploy/ 2>/dev/null || true
        log "${GREEN}" "Build successful!"
        ;;
        
    "user")
        cd /build/user
        log "${GREEN}" "Building user space programs..."
        
        # Set debug flag if specified
        if [ "$DEBUG" = "1" ]; then
            make DEBUG=1 || {
                log "${RED}" "Build failed!"
                exit 1
            }
        else
            make || {
                log "${RED}" "Build failed!"
                exit 1
            }
        fi
        
        log "${GREEN}" "Copying programs to deploy directory..."
        mkdir -p /build/deploy
        cp /build/user/bin/* /build/deploy/ 2>/dev/null || true
        log "${GREEN}" "Build successful!"
        ;;
        
    "clean")
        # Clean kernel modules
        cd /build/module
        log "${GREEN}" "Cleaning kernel modules..."
        make ${MAKE_PARAMS} clean || true
        
        # Clean user space programs
        cd /build/user
        log "${GREEN}" "Cleaning user space programs..."
        make clean || true
        
        log "${GREEN}" "Clean completed!"
        ;;
        
    *)
        exec "$@"
        ;;
esac