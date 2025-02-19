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

echo "Installing Hexapod driver..."

# Stop if any command fails
set -e

# Remove old module if loaded
if lsmod | grep -q "hexapod_driver"; then
    echo "Removing old module..."
    sudo rmmod hexapod_driver
fi

# Load kernel module
echo "Loading kernel module..."
sudo insmod hexapod_driver.ko

# Create device node if not created automatically
if [ ! -e /dev/hexapod ]; then
    echo "Creating device node..."
    major=$(grep hexapod /proc/devices | cut -d' ' -f1)
    if [ -z "$major" ]; then
        echo "Error: Failed to get device major number!"
        exit 1
    fi
    sudo mknod /dev/hexapod c $major 0
    sudo chmod 666 /dev/hexapod
fi

# Load module at boot
if ! grep -q hexapod_driver /etc/modules; then
    echo "Adding module to /etc/modules..."
    echo "hexapod_driver" | sudo tee -a /etc/modules
fi

# Copy user space program
echo "Installing servo test program..."
sudo cp servo_test /usr/local/bin/
sudo chmod +x /usr/local/bin/servo_test

echo -e "\nInstallation completed successfully!"

# Show system status
echo -e "\nI2C devices on i2c-0:"
i2cdetect -y -r 0

echo -e "\nLoaded kernel modules:"
lsmod | grep hexapod

echo -e "\nDevice node:"
ls -l /dev/hexapod
EOF

    chmod +x "${DEPLOY_DIR}/install.sh"
}

# Parse command line arguments
case "$1" in
    clean)
        log "${GREEN}" "Cleaning build artifacts..."
        # Check if Docker image exists, build if it doesn't
        if [[ "$(docker images -q hexapod-builder 2> /dev/null)" == "" ]]; then
            build_docker_image
        fi
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
