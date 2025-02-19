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

# Change to module directory
cd /build/module

# Common make parameters
MAKE_PARAMS="ARCH=${ARCH} CROSS_COMPILE=${CROSS_COMPILE} KERNEL_DIR=${KERNEL_DIR}"

case "$1" in
    "clean")
        log "${GREEN}" "Cleaning kernel module..."
        make ${MAKE_PARAMS} clean
        log "${GREEN}" "Clean successful!"
        ;;
    "")
        log "${GREEN}" "Building kernel module..."
        make ${MAKE_PARAMS} || {
            log "${RED}" "Build failed!"
            exit 1
        }
        log "${GREEN}" "Copying built modules to deploy directory..."
        mkdir -p /build/deploy
        cp *.ko /build/deploy/ || {
            log "${RED}" "Failed to copy modules!"
            exit 1
        }
        log "${GREEN}" "Build successful!"
        ;;
    *)
        log "${RED}" "Invalid argument: $1"
        exit 1
        ;;
esac
