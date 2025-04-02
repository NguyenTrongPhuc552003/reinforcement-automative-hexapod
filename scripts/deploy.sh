#!/bin/bash

# Configuration
BEAGLEBONE_IP="beaglebone.local"  # BeagleBone IP or hostname
BEAGLEBONE_USER="debian"
DEPLOY_DIR="../deploy"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Get the project root directory
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEPLOY_DIR="${PROJECT_ROOT}/deploy"

# Check if deployment package exists
if [ ! -d "${DEPLOY_DIR}" ]; then
    echo -e "${RED}Deployment package not found. Run build.sh first.${NC}"
    exit 1
fi

# Check SSH connection
echo -e "${YELLOW}Testing SSH connection to BeagleBone...${NC}"
if ! ssh -q ${BEAGLEBONE_USER}@${BEAGLEBONE_IP} exit; then
    echo -e "${RED}Cannot connect to BeagleBone at ${BEAGLEBONE_IP} address. Please check your connection.${NC}"
    exit 1
fi

# Get kernel version from BeagleBone
echo -e "${YELLOW}Getting kernel version from BeagleBone...${NC}"
KERNEL_VERSION=$(ssh ${BEAGLEBONE_USER}@${BEAGLEBONE_IP} 'uname -r')
echo -e "BeagleBone kernel version: ${GREEN}${KERNEL_VERSION}${NC}"

# Check if hexapod driver directory exists, if not create it
echo -e "${YELLOW}Checking if hexapod driver directory exists...${NC}"
if ! ssh ${BEAGLEBONE_USER}@${BEAGLEBONE_IP} '[ -d ~/hexapod_driver ]'; then
    echo -e "${YELLOW}Creating hexapod driver directory...${NC}"
    ssh ${BEAGLEBONE_USER}@${BEAGLEBONE_IP} 'mkdir ~/hexapod_driver'
fi

# Deploy to BeagleBone
echo "Deploying to BeagleBone..."
scp -r ${DEPLOY_DIR}/* ${BEAGLEBONE_USER}@${BEAGLEBONE_IP}:~/hexapod_driver/

if [ $? -ne 0 ]; then
    echo -e "${RED}Failed to copy files to BeagleBone${NC}"
    exit 1
fi

# Install on BeagleBone
echo "Installing on BeagleBone..."
ssh -t ${BEAGLEBONE_USER}@${BEAGLEBONE_IP} "cd ~/hexapod_driver && sudo ./install.sh"

if [ $? -eq 0 ]; then
    echo -e "${GREEN}Deployment successful!${NC}"
    echo -e "You can now test the driver using your ${YELLOW}testing programs${NC}"
else
    echo -e "${RED}Deployment failed!${NC}"
    exit 1
fi