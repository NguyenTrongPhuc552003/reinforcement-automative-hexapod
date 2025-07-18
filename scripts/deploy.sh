#!/bin/bash

# Configuration
BEAGLEBONE_IP="beaglebone.local"  # BeagleBone IP or hostname
BEAGLEBONE_MAC="80:91:33:49:FB:D1"
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

# Flags for script options
RUN_INSTALL=0  # Default: do NOT run install after deployment
SPECIFIC_FILES=""  # Default: deploy all files

# Function to show usage
usage() {
    echo "Usage: $0 [OPTIONS]"
    echo "Options:"
    echo "  (no argument)        Deploy all files to BeagleBone"
    echo "  -i, --install        Deploy and run install.sh script on BeagleBone"
    echo "  -f, --file FILE      Deploy only specified file(s) to BeagleBone"
    echo "                       Multiple files can be specified with multiple -f options"
    echo "  -h, --help           Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                   Deploy all files without running install.sh"
    echo "  $0 -i                Deploy all files and run install.sh"
    echo "  $0 -f hexapod_app    Deploy only the specified hexapod application"
    echo ""
    exit 1
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        -i|--install)
            RUN_INSTALL=1
            shift
            ;;
        -f|--file)
            if [[ -z "$2" || "$2" == -* ]]; then
                echo -e "${RED}Error: -f/--file requires a filename argument${NC}"
                usage
            fi
            if [[ -z "$SPECIFIC_FILES" ]]; then
                SPECIFIC_FILES="$2"
            else
                SPECIFIC_FILES="$SPECIFIC_FILES $2"
            fi
            shift 2
            ;;
        -h|--help)
            usage
            ;;
        *)
            echo -e "${RED}Error: Unknown option: $1${NC}"
            usage
            ;;
    esac
done

# Check if deployment package exists
if [ ! -d "${DEPLOY_DIR}" ]; then
    echo -e "${RED}Deployment package not found. Run build.sh first.${NC}"
    exit 1
fi

# WSL2 workaround: resolve IP from MAC via PowerShell
if grep -qEi "(microsoft|wsl)" /proc/version; then
    echo -e "${YELLOW}WSL2 detected. Attempting to resolve BeagleBone IP via MAC...${NC}"
    BB_REAL_IP=$(powershell.exe -Command "Get-NetNeighbor -AddressFamily IPv4 | Where-Object { \$_.LinkLayerAddress -ieq '${BEAGLEBONE_MAC}' } | Select-Object -ExpandProperty IPAddress" | tr -d '\r')
    if [[ -n "$BB_REAL_IP" ]]; then
        echo -e "${GREEN}Resolved BeagleBone IP: $BB_REAL_IP${NC}"
        BEAGLEBONE_IP="$BB_REAL_IP"
    else
        echo -e "${RED}Could not resolve BeagleBone IP from MAC address in WSL2.${NC}"
    fi
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

if [[ -n "$SPECIFIC_FILES" ]]; then
    # Deploy specific files
    for file in $SPECIFIC_FILES; do
        if [ -f "${DEPLOY_DIR}/${file}" ]; then
            echo -e "${YELLOW}Copying ${file} to BeagleBone...${NC}"
            scp "${DEPLOY_DIR}/${file}" ${BEAGLEBONE_USER}@${BEAGLEBONE_IP}:~/hexapod_driver/
            
            if [ $? -ne 0 ]; then
                echo -e "${RED}Failed to copy ${file} to BeagleBone${NC}"
                exit 1
            fi
        else
            echo -e "${RED}File not found: ${DEPLOY_DIR}/${file}${NC}"
            exit 1
        fi
    done
else
    # Deploy all files
    echo -e "${YELLOW}Copying all files from ${DEPLOY_DIR}/ to BeagleBone...${NC}"
    scp -r ${DEPLOY_DIR}/* ${BEAGLEBONE_USER}@${BEAGLEBONE_IP}:~/hexapod_driver/

    if [ $? -ne 0 ]; then
        echo -e "${RED}Failed to copy files to BeagleBone${NC}"
        exit 1
    fi
fi

echo -e "${GREEN}Files deployed successfully!${NC}"

# Run install script on BeagleBone if requested
if [ $RUN_INSTALL -eq 1 ]; then
    echo -e "${YELLOW}Running install.sh on BeagleBone...${NC}"
    ssh -t ${BEAGLEBONE_USER}@${BEAGLEBONE_IP} "cd ~/hexapod_driver && sudo ./install.sh"

    if [ $? -eq 0 ]; then
        echo -e "${GREEN}Installation completed successfully!${NC}"
    else
        echo -e "${RED}Installation failed!${NC}"
        exit 1
    fi
else
    echo -e "${YELLOW}Installation script not run (use -i option to install)${NC}"
    echo -e "You can manually install by running: ${GREEN}ssh ${BEAGLEBONE_USER}@${BEAGLEBONE_IP} 'cd ~/hexapod_driver && sudo ./install.sh'${NC}"
fi

echo -e "${GREEN}Deployment successful!${NC}"
echo -e "You can now test the driver using your ${YELLOW}testing programs${NC}"