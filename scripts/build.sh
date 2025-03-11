#!/bin/bash

# Exit on any error
set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Get the project root directory
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KERNEL_MODULE_DIR="${PROJECT_ROOT}/driver"
USER_SPACE_DIR="${PROJECT_ROOT}/app"
DEPLOY_DIR="${PROJECT_ROOT}/deploy"

# Create deploy directory with "whoami" user permission if it doesn't exist
if [ ! -d "${DEPLOY_DIR}" ]; then
    mkdir -p "${DEPLOY_DIR}"
    chown -R "$(whoami)":"$(whoami)" "${DEPLOY_DIR}"
fi

# Function to show usage
usage() {
    echo "Usage: $0 [OPTIONS]"
    echo "Options:"
    echo "  (no argument)     Build all components"
    echo "  -m, module        Build kernel modules"
    echo "  -u, user          Build user space programs"
    echo "  -c, clean         Clean build artifacts"
    echo "  -i, install       Create installation script"
    echo "  -n, --no-cache    Build all without cache"
    echo "  -h, --help        Show this help message"
    exit 1
}

# Function to print colored messages
log() {
    local color=$1
    shift
    echo -e "${color}$@${NC}"
}

# Function to build Docker image
build_docker_image() {
    local cache_flag=$1
    log "${YELLOW}" "Preparing Docker image..."
    
    # Check if Docker is installed and running
    if ! command -v docker &> /dev/null; then
        log "${RED}" "Docker is not installed or not in PATH"
        exit 1
    fi
    
    # Verify docker-entrypoint.sh exists
    if [ ! -f "${PROJECT_ROOT}/docker-entrypoint.sh" ]; then
        log "${RED}" "docker-entrypoint.sh not found in project root!"
        exit 1
    fi
    
    # Build the Docker image
    docker build ${cache_flag} -t hexapod-builder "${PROJECT_ROOT}" || {
        log "${RED}" "Docker build failed!"
        exit 1
    }
    
    # Verify image was created
    if ! docker images | grep -q hexapod-builder; then
        log "${RED}" "Failed to create Docker image 'hexapod-builder'"
        exit 1
    fi
    
    log "${GREEN}" "Docker image 'hexapod-builder' created successfully"
}

# Function to build kernel module
build_kernel_module() {
    log "${YELLOW}" "Preparing kernel modules..."
    
    # Release build by default, add DEBUG=1 for debug build
    local build_type="release"
    if [ "$DEBUG" = "1" ]; then
        build_type="debug"
        log "${YELLOW}" "Building DEBUG version of kernel modules..."
    else
        log "${YELLOW}" "Building RELEASE version of kernel modules..."
    fi
    
    docker run --rm \
        -v "${KERNEL_MODULE_DIR}:/build/module" \
        -v "${DEPLOY_DIR}:/build/deploy" \
        -e BUILD_TYPE=${build_type} \
        hexapod-builder module || {
        log "${RED}" "Kernel modules build failed!"
        exit 1
    }
}

# Function to build user space program
build_user_space() {
    log "${YELLOW}" "Preparing user space programs..."
    
    # Release build by default, add DEBUG=1 for debug build
    local env_params=""
    if [ "$DEBUG" = "1" ]; then
        env_params="-e DEBUG=1"
        log "${YELLOW}" "Building DEBUG version of user space programs..."
    else
        log "${YELLOW}" "Building RELEASE version of user space programs..."
    fi
    
    # Run Docker with proper command format
    docker run --rm \
        -v "${USER_SPACE_DIR}:/build/user" \
        -v "${DEPLOY_DIR}:/build/deploy" \
        ${env_params} \
        hexapod-builder user || {
        log "${RED}" "User space programs build failed!"
        exit 1
    }
}

# Function to create install script
build_install_script() {
    log "${YELLOW}" "Copying installation script..."
    local script_dir="$(dirname "${BASH_SOURCE[0]}")"
    cp "${script_dir}/install.sh" "${DEPLOY_DIR}/"
    chmod +x "${DEPLOY_DIR}/install.sh"
    log "${GREEN}" "Installation script copied successfully"
}

# Parse command line arguments
case "$1" in
    -c | clean)
        log "${YELLOW}" "Cleaning build artifacts..."

        # Check if Docker image exists before cleaning
        if ! docker images | grep -q hexapod-builder; then
            log "${YELLOW}" "Docker image doesn't exist, build first!"
            exit 0
        fi
        
        # Clean kernel driver and user space using Docker
        docker run --rm \
            -v "${KERNEL_MODULE_DIR}:/build/module" \
            -v "${USER_SPACE_DIR}:/build/user" \
            hexapod-builder clean || {
            # If Docker image doesn't exist, that's fine - continue cleaning
            if [ $? -ne 125 ]; then
                log "${RED}" "Docker clean failed!"
                exit 1
            fi
        }
        
        # Clean deploy directory with sudo if needed
        if [ -d "${DEPLOY_DIR}" ]; then
            log "${YELLOW}" "Cleaning deploy directory..."
            if ! rm -rf "${DEPLOY_DIR}"/* 2>/dev/null; then
                log "${YELLOW}" "Using sudo to clean deploy directory..."
                sudo rm -rf "${DEPLOY_DIR}"/*
            fi
        fi
        
        # Clean user space binaries with sudo if needed
        if [ -d "${USER_SPACE_DIR}/bin" ]; then
            log "${GREEN}" "Cleaning user space binaries..."
            if ! rm -rf "${USER_SPACE_DIR}/bin" 2>/dev/null; then
                log "${YELLOW}" "Using sudo to clean user space binaries..."
                sudo rm -rf "${USER_SPACE_DIR}/bin"
            fi
        fi
        
        log "${GREEN}" "Clean completed successfully!"
        exit 0
        ;;

    -n | --no-cache)
        # Remove Docker image if it exists
        if docker images | grep -q hexapod-builder; then
            log "${GREEN}" "Removing Docker image..."
            docker rmi hexapod-builder || true
        fi

        build_docker_image "--no-cache"
        build_kernel_module
        build_user_space
        build_install_script
        log "${GREEN}" "Build completed successfully!"
        log "${GREEN}" "Deployment package created in: ${DEPLOY_DIR}"
        ;;

    -m | module)
        build_docker_image ""
        build_kernel_module
        log "${GREEN}" "Kernel modules build completed successfully!"
        ;;

    -u | user)
        build_docker_image ""
        build_user_space
        log "${GREEN}" "User space programs build completed successfully!"
        ;;

    -i | install)
        build_install_script
        log "${GREEN}" "Installation script created successfully!"
        ;;

    "")
        build_docker_image ""
        build_kernel_module
        build_user_space
        build_install_script
        log "${GREEN}" "Build completed successfully!"
        log "${GREEN}" "Deployment package created in: ${DEPLOY_DIR}"
        ;;
        
    *)
        usage
        ;;
esac