#!/bin/bash

# Exit on error
set -e

# Color codes for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

# Project paths
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KERNEL_DRIVER_DIR="${PROJECT_ROOT}/driver"
USER_SPACE_DIR="${PROJECT_ROOT}/app"
DOCS_DIR="${PROJECT_ROOT}/docs"
DEPLOY_DIR="${PROJECT_ROOT}/deploy"

# Function to print colored messages
print_header() {
    echo -e "\n${BLUE}╔════════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║ ${GREEN}$1${BLUE}                              ║${NC}"
    echo -e "${BLUE}╚════════════════════════════════════════════════╝${NC}\n"
}

print_step() {
    echo -e "${GREEN}[✓] $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}[!] $1${NC}"
}

print_error() {
    echo -e "${RED}[✗] $1${NC}"
}

# Function to check if a command exists
check_command() {
    if ! command -v $1 &> /dev/null; then
        print_error "$1 is not installed. Please install it first."
        echo "  Installation hint: $2"
        return 1
    fi
    print_step "$1 is available"
    return 0
}

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    print_error "Please do not run as root"
    echo "  This script should be run as a regular user with sudo privileges"
    exit 1
fi

print_header "Hexapod Robot Project Setup"
echo "This script will prepare your environment for hexapod development."
echo "Setting up in directory: ${PROJECT_ROOT}"
echo ""

# Check for required tools with installation hints
print_header "Checking Required Dependencies"

DEPS_MISSING=0

check_command "docker" "sudo apt-get install docker.io && sudo usermod -aG docker $USER" || DEPS_MISSING=1
check_command "git" "sudo apt-get install git" || DEPS_MISSING=1
check_command "make" "sudo apt-get install build-essential" || DEPS_MISSING=1
check_command "arm-linux-gnueabihf-gcc" "sudo apt-get install gcc-arm-linux-gnueabihf" || DEPS_MISSING=1

if [ $DEPS_MISSING -eq 1 ]; then
    print_error "Please install missing dependencies and run the script again"
    exit 1
fi

# Create directory structure
print_header "Creating Directory Structure"

mkdir -p "${DEPLOY_DIR}"
mkdir -p "${KERNEL_DRIVER_DIR}/inc"
mkdir -p "${KERNEL_DRIVER_DIR}/src"
mkdir -p "${KERNEL_DRIVER_DIR}/dts"
mkdir -p "${USER_SPACE_DIR}/src"
mkdir -p "${USER_SPACE_DIR}/inc"
mkdir -p "${USER_SPACE_DIR}/test"
mkdir -p "${USER_SPACE_DIR}/bin"
mkdir -p "${DOCS_DIR}/images"

print_step "Directory structure created"

# Create default configuration file
print_header "Creating Default Configuration"

# Create calibration.cfg if it doesn't exist
if [ ! -f "${USER_SPACE_DIR}/calibration.cfg" ]; then
    print_step "Creating default calibration file"
    touch "${USER_SPACE_DIR}/calibration.cfg"
    # This will be properly initialized when the calibration test runs
fi

# Build Docker image for cross-compilation
print_header "Building Docker Image for Cross-Compilation"

print_step "Building hexapod-builder Docker image (this may take a few minutes)..."
if ! docker build -t hexapod-builder "${PROJECT_ROOT}" 2>&1 | grep -v "^Step"; then
    print_error "Failed to build Docker image"
    exit 1
fi
print_step "Docker image built successfully"

# Create symlinks for convenient access
print_header "Creating Helper Scripts"

ln -sf "${PROJECT_ROOT}/scripts/build.sh" "${PROJECT_ROOT}/build.sh"
ln -sf "${PROJECT_ROOT}/scripts/clean.sh" "${PROJECT_ROOT}/clean.sh"
ln -sf "${PROJECT_ROOT}/scripts/deploy.sh" "${PROJECT_ROOT}/deploy.sh"
chmod +x "${PROJECT_ROOT}/scripts"/*.sh

print_step "Helper scripts created"

# Set up git hooks
print_header "Setting Up Git Hooks"

if [ -d "${PROJECT_ROOT}/.git" ]; then
    # Pre-commit hook to run tests and check code
    cat > "${PROJECT_ROOT}/.git/hooks/pre-commit" << 'EOF'
#!/bin/bash
./scripts/build.sh user
if [ $? -ne 0 ]; then
    echo "Build failed. Please fix the errors before committing."
    exit 1
fi
EOF
    chmod +x "${PROJECT_ROOT}/.git/hooks/pre-commit"
    print_step "Git pre-commit hook installed"
else
    print_warning "Not a git repository. Skipping git hooks."
fi

# Build the project
print_header "Building Initial Project"

print_step "Building the project for the first time..."
cd "${PROJECT_ROOT}"
if ! ./scripts/build.sh; then
    print_warning "Initial build failed, but setup will continue."
    print_warning "Please fix build issues before deploying."
else
    print_step "Initial build successful"
fi

# Generate sample device connections diagram
print_header "Finalizing Setup"

# Check if deploy directory contains built files
if ls "${DEPLOY_DIR}"/*.ko &> /dev/null; then
    print_step "Kernel module built successfully"
else
    print_warning "Kernel module not found in deploy directory"
fi

if ls "${DEPLOY_DIR}"/test_* &> /dev/null; then
    print_step "Test utilities built successfully"
else
    print_warning "Test utilities not found in deploy directory"
fi

# Final instructions
print_header "Setup Complete!"

cat << 'EOF'
Your hexapod robot development environment is now set up.
Here's how to use it:

1. To build the project:
   ./build.sh         # Build everything
   ./build.sh driver  # Build only the kernel driver
   ./build.sh user    # Build only the user-space applications

2. To deploy to your BeagleBone:
   ./deploy.sh        # Follow the prompts to configure your BeagleBone IP

3. To test on your BeagleBone:
   sudo ./test_calibration reset  # Initialize calibration
   sudo ./test_servo              # Test servo movements
   sudo ./test_mpu6050            # Test IMU sensor readings
   sudo ./test_movement           # Test different gaits

For more information, see the documentation in ./docs/

Happy hacking!
EOF

echo ""