#!/bin/bash

# Stop any existing module
sudo rmmod hexapod_main 2>/dev/null

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
