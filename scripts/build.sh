#!/bin/bash

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Get the project root directory
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KERNEL_DRIVER_DIR="${PROJECT_ROOT}/kernel_driver"
USER_SPACE_DIR="${PROJECT_ROOT}/user_space"

echo -e "${YELLOW}Building Docker image...${NC}"

# Check if --no-cache flag is passed
if [[ "$1" == "--no-cache" ]]; then
    docker build --no-cache -t hexapod-builder "${PROJECT_ROOT}"
else
    docker build -t hexapod-builder "${PROJECT_ROOT}"
fi

if [ $? -ne 0 ]; then
    echo -e "${RED}Docker build failed!${NC}"
    exit 1
fi

echo -e "${GREEN}Docker image built successfully!${NC}"

# Create deploy directory
DEPLOY_DIR="${PROJECT_ROOT}/deploy"
mkdir -p "${DEPLOY_DIR}"

# Build kernel module
echo -e "${YELLOW}Building kernel module...${NC}"
docker run --rm \
    -v "${KERNEL_DRIVER_DIR}:/build/module" \
    -v "${DEPLOY_DIR}:/build/deploy" \
    hexapod-builder

if [ $? -eq 0 ]; then
    echo -e "${GREEN}Build completed successfully!${NC}"
else
    echo -e "${RED}Build failed!${NC}"
    exit 1
fi

# Build user space program
echo "Building user space program..."
cd "${USER_SPACE_DIR}"
make clean && make
if [ $? -eq 0 ]; then
    cp servo_test "${DEPLOY_DIR}/"
else
    echo -e "${RED}User space program build failed!${NC}"
    exit 1
fi

# Create install script
cat > "${DEPLOY_DIR}/install.sh" << 'EOF'
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
EOF

chmod +x "${DEPLOY_DIR}/install.sh"

echo -e "${GREEN}Build completed successfully!${NC}"
echo "Deployment package created in: ${DEPLOY_DIR}"
