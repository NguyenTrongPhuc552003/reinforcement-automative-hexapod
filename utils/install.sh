#!/bin/bash

# Define I2C port
I2C_PORT=3

# Define the driver name
DRIVER_NAME=hexapod_driver

# Function to show usage
usage() {
    echo "Usage: $0 [OPTIONS]"
    echo "Options:"
    echo "  (no argument) Install the hexapod driver"
    echo "  -r, --remove  Remove/uninstall the hexapod driver"
    echo "  -h, --help    Show this help message"
    exit 1
}

# Function to uninstall the driver
uninstall_driver() {
    # Check if the driver is installed
    if ! lsmod | grep -q ${DRIVER_NAME}; then
        echo "Hexapod driver is not installed!"
        exit 1
    fi
    echo "Uninstalling Hexapod driver..."

    # Remove the module
    echo "Removing kernel module..."
    sudo rmmod ${DRIVER_NAME} 2>/dev/null || true

    echo "Uninstallation complete!"
    exit 0
}

# Parse command line arguments
case "$1" in
    -r|--remove)
        uninstall_driver
        ;;
    -h|--help)
        usage
        ;;
esac

# Continue with installation if no arguments or not matched above

echo "Installing Hexapod driver..."

# Stop if any command fails
set -e

# Check for i2c-tools
if ! command -v i2cdetect &> /dev/null; then
    echo "Installing i2c-tools..."
    sudo apt-get update
    sudo apt-get install -y i2c-tools
fi

# Check I2C bus ${I2C_PORT}
echo "Checking I2C bus ${I2C_PORT}..."
if ! i2cdetect -l | grep -q "i2c-${I2C_PORT}"; then
    echo "Error: I2C bus ${I2C_PORT} not found!"
    echo "Please enable I2C${I2C_PORT} in your device tree overlay."
    exit 1
fi

# Scan I2C bus ${I2C_PORT} for devices
echo "Scanning I2C bus ${I2C_PORT} for devices..."
i2cdetect -y -r ${I2C_PORT}

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
sudo rmmod ${DRIVER_NAME} 2>/dev/null || true

# Clear dmesg to make our debug messages easier to find
sudo dmesg -C

# Install the module
sudo insmod ${DRIVER_NAME}.ko

# Show debug messages
echo "Driver messages:"
sudo dmesg

# Set permissions for device files
echo "Setting up permissions..."
sudo chmod 666 /dev/hexapod

echo "Installation complete!"
