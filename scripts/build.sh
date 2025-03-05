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
COMMON_DIR="${PROJECT_ROOT}/common"
DEPLOY_DIR="${PROJECT_ROOT}/deploy"

# Function to show usage
usage() {
    echo "Usage: $0 [OPTIONS]"
    echo "Options:"
    echo "  (no argument) Build all components"
    echo "  common        Build common library"
    echo "  kernel        Build kernel module"
    echo "  user          Build user space programs"
    echo "  clean         Clean build artifacts"
    echo "  --no-cache    Build Docker image without cache"
    echo "  -h, --help    Show this help message"
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
    docker build ${cache_flag} -t hexapod-builder "${PROJECT_ROOT}" || {
        log "${RED}" "Docker build failed!"
        exit 1
    }
}

# Function to build common library
build_common_library() {
    log "${YELLOW}" "Preparing common library..."
    docker run --rm \
        -v "${COMMON_DIR}:/build/common" \
        -v "${DEPLOY_DIR}:/build/deploy" \
        hexapod-builder common || {
        log "${RED}" "Common library build failed!"
        exit 1
    }
}

# Function to build kernel module
build_kernel_module() {
    log "${YELLOW}" "Preparing kernel module..."
    docker run --rm \
        -v "${KERNEL_DRIVER_DIR}:/build/module" \
        -v "${COMMON_DIR}:/build/common" \
        -v "${DEPLOY_DIR}:/build/deploy" \
        hexapod-builder kernel || {
        log "${RED}" "Kernel module build failed!"
        exit 1
    }
}

# Function to build user space program
build_user_space() {
    log "${YELLOW}" "Preparing user space programs..."
    docker run --rm \
        -v "${USER_SPACE_DIR}:/build/user_space" \
        -v "${COMMON_DIR}:/build/common" \
        -v "${DEPLOY_DIR}:/build/deploy" \
        hexapod-builder user_space || {
        log "${RED}" "User space program build failed!"
        exit 1
    }
}

# Function to create install script
create_install_script() {
    cat > "${DEPLOY_DIR}/install.sh" << 'EOF'
#!/bin/bash

echo "Installing Hexapod driver..."

# Stop if any command fails
set -e

# Check for i2c-tools
if ! command -v i2cdetect &> /dev/null; then
    echo "Installing i2c-tools..."
    sudo apt-get update
    sudo apt-get install -y i2c-tools
fi

# Check I2C bus 3
echo "Checking I2C bus 3..."
if ! i2cdetect -l | grep -q "i2c-3"; then
    echo "Error: I2C bus 3 not found!"
    echo "Please enable I2C3 in your device tree overlay."
    exit 1
fi

# Scan I2C bus 3 for devices
echo "Scanning I2C bus 3 for devices..."
i2cdetect -y -r 3

# Check for conflicting drivers
echo "Checking for conflicting drivers..."
for module in inv_mpu6050_i2c inv_mpu6050 mpu6050_i2c; do
    if lsmod | grep -q "^$module"; then
        echo "Removing conflicting module: $module"
        sudo rmmod $module || true
    fi
done

# Install kernel module
echo "Installing kernel module..."
sudo rmmod hexapod-driver 2>/dev/null || true

# Clear dmesg to make our debug messages easier to find
sudo dmesg -C

# Install the module
sudo insmod hexapod-driver.ko

# Show debug messages
echo "Driver messages:"
sudo dmesg

# Set permissions for device files
echo "Setting up permissions..."
sudo chmod 666 /dev/hexapod

# Install libraries
echo "Installing libraries..."
sudo cp libhexapod.* /usr/local/lib/
sudo ldconfig

# Set up library path
if ! grep -q "/usr/local/lib" /etc/ld.so.conf.d/hexapod.conf 2>/dev/null; then
    echo "/usr/local/lib" | sudo tee /etc/ld.so.conf.d/hexapod.conf
    sudo ldconfig
fi

echo "Installation complete!"
EOF
    chmod +x "${DEPLOY_DIR}/install.sh"
}

# Parse command line arguments
case "$1" in
    clean)
        log "${YELLOW}" "Cleaning build artifacts..."

        # Clean common library using Docker
        docker run --rm \
            -v "${COMMON_DIR}:/build/common" \
            hexapod-builder clean || {
            # If Docker image doesn't exist, that's fine - continue cleaning
            if [ $? -ne 125 ]; then
                log "${RED}" "Docker clean failed!"
                exit 1
            fi
        }
        
        # Clean kernel driver and user space using Docker
        docker run --rm \
            -v "${KERNEL_DRIVER_DIR}:/build/module" \
            -v "${USER_SPACE_DIR}:/build/user_space" \
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
        
        # Remove Docker image if it exists
        if docker images | grep -q hexapod-builder; then
            log "${GREEN}" "Removing Docker image..."
            docker rmi hexapod-builder || true
        fi
        
        log "${GREEN}" "Clean completed successfully!"
        exit 0
        ;;
    --no-cache)
        build_docker_image "--no-cache"
        build_common_library
        build_kernel_module
        build_user_space
        create_install_script
        log "${GREEN}" "Build completed successfully!"
        log "${GREEN}" "Deployment package created in: ${DEPLOY_DIR}"
        ;;
    -h|--help)
        usage
        ;;
    "")
        build_docker_image ""
        build_common_library
        build_kernel_module
        build_user_space
        create_install_script
        log "${GREEN}" "Build completed successfully!"
        log "${GREEN}" "Deployment package created in: ${DEPLOY_DIR}"
        ;;
    common)
        build_docker_image ""
        build_common_library
        log "${GREEN}" "Common library build completed successfully!"
        ;;
    kernel)
        build_docker_image ""
        build_kernel_module
        log "${GREEN}" "Kernel module build completed successfully!"
        ;;
    user)
        build_docker_image ""
        build_user_space
        log "${GREEN}" "User space program build completed successfully!"
        ;;
    *)
        usage
        ;;
esac
