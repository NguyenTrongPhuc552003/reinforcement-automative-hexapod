#!/bin/bash

# Exit on any error
set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Function to print colored messages
log() {
    local color=$1
    shift
    echo -e "${color}$@${NC}"
}

# Function to organize build artifacts for kernel module
organize_build_artifacts() {
    log "${YELLOW}" "Organizing build artifacts..."
    
    # Create directories for organized artifacts
    mkdir -p /build/module/obj
    mkdir -p /build/module/cmd
    mkdir -p /build/module/deps
    
    # Move .o files
    find /build/module -maxdepth 1 -name "*.o" -exec mv {} /build/module/obj/ \;
    find /build/module/src -name "*.o" -exec mv {} /build/module/obj/ \;
    
    # Move .cmd files
    find /build/module -maxdepth 1 -name "*.cmd" -exec mv {} /build/module/cmd/ \;
    find /build/module/src -name "*.cmd" -exec mv {} /build/module/cmd/ \;
    
    # Move .d files (dependency files)
    find /build/module -maxdepth 1 -name "*.d" -exec mv {} /build/module/deps/ \;
    find /build/module/src -name "*.d" -exec mv {} /build/module/deps/ \;
    
    log "${GREEN}" "Build artifacts organized successfully!"
}

# Common make parameters for kernel module
MAKE_PARAMS="ARCH=${ARCH} CROSS_COMPILE=${CROSS_COMPILE} KERNEL_DIR=${KERNEL_DIR}"

case "$1" in
    "module")
        log "${GREEN}" "Building kernel module..."
        cd /build/module
        
        # Configure build type
        if [ "$BUILD_TYPE" = "debug" ]; then
            EXTRA_CFLAGS="-DDEBUG -I/build/module/inc"
            log "${GREEN}" "Debug build enabled"
        else
            EXTRA_CFLAGS="-DNDEBUG -O2 -I/build/module/inc"
            log "${GREEN}" "Release build enabled"
        fi
        
        # Build the module
        make ${MAKE_PARAMS} EXTRA_CFLAGS="$EXTRA_CFLAGS" || {
            log "${RED}" "Build failed!"
            exit 1
        }
        
        # Organize artifacts after successful build
        organize_build_artifacts
        
        log "${GREEN}" "Copying built modules to deploy directory..."
        mkdir -p /build/deploy
        cp /build/module/*.ko /build/deploy/ 2>/dev/null || true
        log "${GREEN}" "Build successful!"
        ;;
        
    "user")
        log "${GREEN}" "Building user space applications..."
        cd /build/user
        log "${GREEN}" "Building user space programs..."
        
        # Set debug flag if specified
        if [ "$DEBUG" = "1" ]; then
            make DEBUG=1 || {
                log "${RED}" "Build failed!"
                exit 1
            }
        else
            make || {
                log "${RED}" "Build failed!"
                exit 1
            }
        fi
        
        log "${GREEN}" "Copying programs to deploy directory..."
        mkdir -p /build/deploy
        cp /build/user/bin/* /build/deploy/ 2>/dev/null || true
        log "${GREEN}" "Build successful!"
        ;;
        
    "pytd3")
        cd /build/pytd3
        log "${GREEN}" "Building PyTD3 reinforcement learning module..."
        
        # Check if virtual environment exists, create if needed
        if [ ! -d "/build/pytd3/venv" ]; then
            log "${YELLOW}" "Virtual environment not found, please run setup_env first"
            log "${YELLOW}" "Continuing with build anyway..."
        fi
        
        # Create build directory if it doesn't exist
        mkdir -p /build/pytd3/build
        
        # Activate virtual environment if it exists
        if [ -d "/build/pytd3/venv" ]; then
            source /build/pytd3/venv/bin/activate
            log "${GREEN}" "Virtual environment activated"
        fi
        
        # Build the package
        cd /build/pytd3
        python setup.py build || {
            log "${RED}" "Python build failed!"
            exit 1
        }
        
        # Copy model files and utilities to deploy
        log "${GREEN}" "Copying PyTD3 artifacts to deploy directory..."
        mkdir -p /build/deploy/pytd3
        cp -r /build/pytd3/build/* /build/deploy/pytd3/ 2>/dev/null || true
        cp /build/pytd3/requirements.txt /build/deploy/pytd3/ 2>/dev/null || true
        cp /build/pytd3/*.py /build/deploy/pytd3/ 2>/dev/null || true
        
        # Copy default model if available
        if [ -d "/build/pytd3/models" ]; then
            mkdir -p /build/deploy/pytd3/models
            cp -r /build/pytd3/models/* /build/deploy/pytd3/models/ 2>/dev/null || true
        fi
        
        log "${GREEN}" "PyTD3 build successful!"
        ;;
        
    "setup_env")
        cd /build/pytd3
        log "${GREEN}" "Setting up Python virtual environment for PyTD3..."
        
        # Install required system packages if possible
        log "${YELLOW}" "Checking system dependencies..."
        if command -v apt-get &> /dev/null; then
            apt-get update
            apt-get install -y python3-dev python3-pip python3-venv
        fi
        
        # Create virtual environment
        log "${YELLOW}" "Creating virtual environment..."
        python3 -m venv venv
        
        # Activate environment and install dependencies
        source venv/bin/activate
        
        log "${YELLOW}" "Upgrading pip..."
        pip install --upgrade pip
        
        log "${YELLOW}" "Installing dependencies..."
        if [ -f "requirements.txt" ]; then
            pip install -r requirements.txt
        else
            log "${YELLOW}" "requirements.txt not found! Installing basic dependencies..."
            pip install numpy torch gymnasium pybind11
        fi
        
        # Install in development mode if setup.py exists
        if [ -f "setup.py" ]; then
            log "${YELLOW}" "Installing PyTD3 in development mode..."
            pip install -e .
        fi
        
        # Create configuration directory
        if [ ! -d "/build/pytd3/config" ]; then
            log "${YELLOW}" "Creating default config directory..."
            mkdir -p /build/pytd3/config
        fi
        
        log "${GREEN}" "Python virtual environment setup completed!"
        ;;
        
    "clean")
        # Clean kernel modules
        if [ -d "/build/module" ]; then
            cd /build/module
            log "${GREEN}" "Cleaning kernel modules..."
            
            # Remove organized build artifacts directories first
            log "${YELLOW}" "Removing organized build artifacts..."
            rm -rf /build/module/{obj,cmd,deps}
            
            # Run the normal make clean
            make ${MAKE_PARAMS} clean || true
        fi
        
        # Clean user space programs
        if [ -d "/build/user" ]; then
            cd /build/user
            log "${GREEN}" "Cleaning user space programs..."
            make clean || true
        fi
        
        # Clean pytd3 build directory
        if [ -d "/build/pytd3/build" ]; then
            log "${GREEN}" "Cleaning PyTD3 build directory..."
            rm -rf /build/pytd3/build/*
            log "${GREEN}" "PyTD3 build directory cleaned"
        fi
        
        # Optional: clean virtual environment
        if [ "$CLEAN_VENV" = "1" ] && [ -d "/build/pytd3/venv" ]; then
            log "${YELLOW}" "Removing PyTD3 virtual environment..."
            rm -rf /build/pytd3/venv
            log "${GREEN}" "PyTD3 virtual environment removed"
        fi
        
        # Clean Python cache files
        find /build/pytd3 -name "__pycache__" -type d -exec rm -rf {} +
        find /build/pytd3 -name "*.pyc" -delete
        
        log "${GREEN}" "Clean completed!"
        ;;
        
    *)
        # Default: execute the command directly
        exec "$@"
        ;;
esac