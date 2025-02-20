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
    "kernel")
        cd /build/module
        log "${GREEN}" "Building kernel module..."
        make ${MAKE_PARAMS} || {
            log "${RED}" "Build failed!"
            exit 1
        }
        log "${GREEN}" "Copying built modules to deploy directory..."
        mkdir -p /build/deploy
        cp /build/module/*.ko /build/deploy/ 2>/dev/null || true
        log "${GREEN}" "Build successful!"
        ;;
        
    "user_space")
        cd /build/user_space
        log "${GREEN}" "Building user space programs..."
        make || {
            log "${RED}" "Build failed!"
            exit 1
        }
        log "${GREEN}" "Copying programs to deploy directory..."
        mkdir -p /build/deploy
        cp /build/user_space/bin/* /build/deploy/ 2>/dev/null || true
        log "${GREEN}" "Build successful!"
        ;;
        
    "clean")
        # Clean kernel module
        cd /build/module
        log "${GREEN}" "Cleaning kernel module..."
        make ${MAKE_PARAMS} clean || true
        
        # Clean user space
        cd /build/user_space
        log "${GREEN}" "Cleaning user space..."
        rm -rf bin/* 2>/dev/null || true
        
        log "${GREEN}" "Clean completed!"
        ;;
        
    *)
        exec "$@"
        ;;
esac
