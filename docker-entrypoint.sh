#!/bin/bash

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Build the kernel module
cd /build/module

echo -e "${YELLOW}Building kernel module...${NC}"
make ARCH=${ARCH} \
     CROSS_COMPILE=${CROSS_COMPILE} \
     KERNEL_DIR=${KERNEL_DIR} \
     $@ || {
    echo -e "${RED}Build failed!${NC}"
    exit 1
}

# Copy built modules to deploy directory
echo -e "${YELLOW}Copying built modules to deploy directory...${NC}"
cp *.ko /build/deploy/ || {
    echo -e "${RED}Failed to copy modules!${NC}"
    exit 1
}

# Fix permissions (docker runs as root)
echo -e "${YELLOW}Fixing permissions...${NC}"
chown -R 1000:1000 /build/deploy
chown -R 1000:1000 /build/module

echo -e "${GREEN}Build completed successfully!${NC}"
