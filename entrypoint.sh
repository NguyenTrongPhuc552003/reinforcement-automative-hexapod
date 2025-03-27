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

# Function to organize build artifacts
organize_build_artifacts() {
    log "${YELLOW}" "Organizing build artifacts..."
    
    # Create directories for organized artifacts
    mkdir -p /build/module/obj
    mkdir -p /build/module/cmd
    mkdir -p /build/module/deps
    
    # Move .o files
    find /build/module -maxdepth 1 -name "*.o" -exec mv {} /build/module/obj/ \;
    find /build/module/src -name "*.o" -exec mv {} /build/module/obj/ \;
    
    # Move .cmd files
    find /build/module -maxdepth 1 -name "*.cmd" -exec mv {} /build/module/cmd/ \;
    find /build/module/src -name "*.cmd" -exec mv {} /build/module/cmd/ \;
    
    # Move .d files (dependency files)
    find /build/module -maxdepth 1 -name "*.d" -exec mv {} /build/module/deps/ \;
    find /build/module/src -name "*.d" -exec mv {} /build/module/deps/ \;
    
    log "${GREEN}" "Build artifacts organized successfully!"
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
        
        # Organize artifacts after successful build
        organize_build_artifacts
        
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
        
        # Remove organized build artifacts directories first
        log "${YELLOW}" "Removing organized build artifacts..."
        rm -rf /build/module/obj/
        rm -rf /build/module/cmd/
        rm -rf /build/module/deps/
        
        # Run the normal make clean
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