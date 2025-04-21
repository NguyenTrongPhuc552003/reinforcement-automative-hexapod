#!/bin/bash

# Exit on error
set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Function to print section header
print_header() {
    echo -e "${BLUE}==== $1 ====${NC}"
}

# Function to print success
print_success() {
    echo -e "  ${GREEN}✓ $1${NC}"
}

# Function to print warning
print_warning() {
    echo -e "  ${YELLOW}⚠ $1${NC}"
}

# Function to print error
print_error() {
    echo -e "  ${RED}✗ $1${NC}"
}

# Check for BeagleBone hardware
print_header "System Information"
if [ -f /proc/device-tree/model ]; then
    MODEL=$(cat /proc/device-tree/model | tr -d '\0')
    echo -e "Device: ${GREEN}$MODEL${NC}"
    
    # Check if this is a BeagleBone device
    if echo "$MODEL" | grep -i "beaglebone" > /dev/null; then
        print_success "BeagleBone hardware detected"
    else
        print_warning "Non-BeagleBone hardware detected"
    fi
else
    print_error "Unable to determine device model"
fi

# Check for processor information
if [ -f /proc/cpuinfo ]; then
    CPU_MODEL=$(grep -m1 "model name" /proc/cpuinfo | cut -d: -f2 | sed 's/^ *//')
    CPU_CORES=$(grep -c "processor" /proc/cpuinfo)
    
    echo -e "Processor: ${GREEN}$CPU_MODEL ($CPU_CORES cores)${NC}"
    
    # Look for ARM Cortex-A15 processor which would indicate BeagleBone AI
    if grep -i "cortex-a15" /proc/cpuinfo > /dev/null; then
        print_success "ARM Cortex-A15 processor detected (BeagleBone AI compatible)"
    fi
elif [ -f /proc/device-tree/compatible ]; then
    COMPATIBLE=$(cat /proc/device-tree/compatible | tr -d '\0')
    echo -e "Compatible: ${GREEN}$COMPATIBLE${NC}"
else
    print_error "Unable to determine processor information"
fi

# Check for TI EVE (Embedded Vision Engine) availability
print_header "TIDL Hardware"
if [ -d /sys/class/remoteproc ]; then
    PROCS=$(ls -1 /sys/class/remoteproc/)
    EVE_FOUND=0
    DSP_FOUND=0
    
    for proc in $PROCS; do
        NAME=$(cat /sys/class/remoteproc/$proc/name 2>/dev/null || echo "unknown")
        echo -e "RemoteProc: ${YELLOW}$proc${NC} - $NAME"
        
        if echo "$NAME" | grep -i "eve" > /dev/null; then
            EVE_FOUND=1
            print_success "EVE processor found ($NAME)"
        fi
        
        if echo "$NAME" | grep -i "dsp" > /dev/null; then
            DSP_FOUND=1
            print_success "DSP processor found ($NAME)"
        fi
    done
    
    if [ $EVE_FOUND -eq 0 ]; then
        print_warning "No EVE processors found, TIDL acceleration may not be available"
    fi
    
    if [ $DSP_FOUND -eq 0 ]; then
        print_warning "No DSP processors found, parallel processing may be limited"
    fi
else
    print_error "RemoteProc subsystem not found, acceleration unavailable"
fi

# Check for OpenCL availability
print_header "OpenCL Support"
if [ -f /usr/lib/libOpenCL.so ] || [ -f /usr/lib/arm-linux-gnueabihf/libOpenCL.so ]; then
    print_success "OpenCL library found"
    
    if command -v clinfo > /dev/null; then
        echo -e "OpenCL platforms/devices:"
        clinfo | grep -E "Platform Name|Device Name" | sed 's/^/  /'
    else
        print_warning "clinfo not installed, cannot query OpenCL devices"
    fi
else
    print_error "OpenCL library not found"
fi

# Check for TIDL API availability
print_header "TIDL API"
TIDL_PATHS=(
    "/opt/tidl/api/tidl_api"
    "/usr/local/tidl/api"
    "/usr/local/include/tidl"
    "/usr/include/tidl"
)

TIDL_FOUND=0
for path in "${TIDL_PATHS[@]}"; do
    if [ -d "$path" ]; then
        print_success "TIDL API found at: $path"
        if [ -f "$path/inc/tidl/tidl_api.h" ]; then
            print_success "TIDL API header verified"
            TIDL_FOUND=1
        elif [ -f "$path/tidl_api.h" ]; then
            print_success "TIDL API header verified"
            TIDL_FOUND=1
        else
            print_warning "TIDL API directory exists but header not found"
        fi
    fi
done

if [ $TIDL_FOUND -eq 0 ]; then
    print_error "TIDL API not found. Consider installing it or creating symlinks."
    echo -e "  You can manually create a symlink from one of these locations:"
    for path in "${TIDL_PATHS[@]}"; do
        echo -e "  - ${YELLOW}$path${NC}"
    done
fi

# Summary
print_header "Summary"
echo "Hardware detection complete."
echo ""

echo "To use TIDL acceleration, specify the API path during build:"
echo -e "${YELLOW}cmake -DTIDL_API_DIR=/path/to/tidl_api ..${NC}"
echo ""

echo "To use OpenCL acceleration, ensure the headers are available:"
echo -e "${YELLOW}sudo apt-get install opencl-headers${NC}"
echo ""

echo "For hardware information in your model, run:"
echo -e "${YELLOW}td3learn_run --info${NC}"
