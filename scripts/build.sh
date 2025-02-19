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
KERNEL_DRIVER_DIR="${PROJECT_ROOT}/kernel_driver"
USER_SPACE_DIR="${PROJECT_ROOT}/user_space"
DEPLOY_DIR="${PROJECT_ROOT}/deploy"

# Function to show usage
usage() {
    echo "Usage: $0 [OPTIONS]"
    echo "Options:"
    echo "  clean         Clean build artifacts"
    echo "  --no-cache   Build Docker image without cache"
    echo "  -h, --help   Show this help message"
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
    log "${GREEN}" "Building Docker image..."
    docker build ${cache_flag} -t hexapod-builder "${PROJECT_ROOT}" || {
        log "${RED}" "Docker build failed!"
        exit 1
    }
}

# Function to build kernel module
build_kernel_module() {
    log "${YELLOW}" "Building kernel module..."
    docker run --rm \
        -v "${KERNEL_DRIVER_DIR}:/build/module" \
        -v "${DEPLOY_DIR}:/build/deploy" \
        hexapod-builder || {
        log "${RED}" "Kernel module build failed!"
        exit 1
    }
}

# Function to build user space program
build_user_space() {
    log "${YELLOW}" "Building user space program..."
    cd "${USER_SPACE_DIR}"
    make clean && make || {
        log "${RED}" "User space program build failed!"
        exit 1
    }
    cp servo_test "${DEPLOY_DIR}/"
}

# Function to create install script
create_install_script() {
    cat > "${DEPLOY_DIR}/install.sh" << 'EOF'
#!/bin/bash

# Stop any existing module
sudo rmmod hexapod_main 2>/dev/null || true

# Install new module
sudo insmod hexapod_main.ko

# Create device node if needed
if [ ! -e /dev/hexapod ]; then
    sudo mknod /dev/hexapod c $(grep hexapod /proc/devices | cut -d' ' -f1) 0
    sudo chmod 666 /dev/hexapod
fi

# Load module at boot
if ! grep -q hexapod_main /etc/modules; then
    echo "hexapod_main" | sudo tee -a /etc/modules
fi

# Copy user space program
sudo cp servo_test /usr/local/bin/
sudo chmod +x /usr/local/bin/servo_test

echo "Installation completed successfully!"
EOF

    chmod +x "${DEPLOY_DIR}/install.sh"
}

# Parse command line arguments
case "$1" in
    clean)
        log "${GREEN}" "Cleaning build artifacts..."
        docker run --rm \
            -v "${KERNEL_DRIVER_DIR}:/build/module" \
            -v "${DEPLOY_DIR}:/build/deploy" \
            hexapod-builder clean
        rm -rf "${DEPLOY_DIR}"/*
        cd "${USER_SPACE_DIR}" && make clean
        log "${GREEN}" "Clean completed successfully!"
        exit 0
        ;;
    --no-cache)
        build_docker_image "--no-cache"
        ;;
    -h|--help)
        usage
        ;;
    "")
        # Check if Docker image exists
        if [[ "$(docker images -q hexapod-builder 2> /dev/null)" == "" ]]; then
            build_docker_image
        fi
        ;;
    *)
        usage
        ;;
esac

# Create deploy directory
mkdir -p "${DEPLOY_DIR}"

# Build everything
build_kernel_module
build_user_space
create_install_script

log "${GREEN}" "Build completed successfully!"
log "${GREEN}" "Deployment package created in: ${DEPLOY_DIR}"
