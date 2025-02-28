#!/bin/bash

# Color codes for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Function to print colored messages
print_message() {
    echo -e "${GREEN}$1${NC}"
}

print_warning() {
    echo -e "${YELLOW}$1${NC}"
}

print_error() {
    echo -e "${RED}$1${NC}"
}

# Function to check if a command exists
check_command() {
    if ! command -v $1 &> /dev/null; then
        print_error "$1 is not installed. Please install it first."
        exit 1
    fi
}

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    print_error "Please do not run as root"
    exit 1
fi

print_message "Setting up development environment for Hexapod Robot project..."

# Check for required tools
print_message "Checking required tools..."
check_command "docker"
check_command "git"
check_command "make"

# Create necessary directories
print_message "Creating directory structure..."
mkdir -p deploy
mkdir -p kernel_driver/include
mkdir -p kernel_driver/src
mkdir -p kernel_driver/device_tree
mkdir -p scripts
mkdir -p user_space/src
mkdir -p user_space/include

# Build Docker image for cross-compilation
print_message "Building Docker image for cross-compilation..."
if ! docker build -t hexapod-builder .; then
    print_error "Failed to build Docker image"
    exit 1
fi

# Create symlinks for convenient access
print_message "Creating symlinks..."
ln -sf scripts/build.sh ./build.sh
ln -sf scripts/clean.sh ./clean.sh

# Set up git hooks
print_message "Setting up git hooks..."
if [ -d .git ]; then
    # Pre-commit hook to run tests
    cat > .git/hooks/pre-commit << 'EOF'
#!/bin/bash
./scripts/build.sh
if [ $? -ne 0 ]; then
    echo "Build failed. Please fix the errors before committing."
    exit 1
fi
EOF
    chmod +x .git/hooks/pre-commit
fi

# Create initial build script if it doesn't exist
if [ ! -f scripts/build.sh ]; then
    print_message "Creating build script..."
    cat > scripts/build.sh << 'EOF'
#!/bin/bash
docker run --rm -v $(pwd):/build/module hexapod-builder
EOF
    chmod +x scripts/build.sh
fi

# Create clean script if it doesn't exist
if [ ! -f scripts/clean.sh ]; then
    print_message "Creating clean script..."
    cat > scripts/clean.sh << 'EOF'
#!/bin/bash
make clean
rm -rf deploy/*
docker rmi hexapod-builder
EOF
    chmod +x scripts/clean.sh
fi

print_message "\nSetup completed successfully!"
print_message "\nNext steps:"
print_message "1. Edit kernel_driver/src files to implement your driver"
print_message "2. Run './build.sh' to build the driver"
print_message "3. Copy the contents of 'deploy' directory to your BeagleBone"
print_message "4. On the BeagleBone, run 'sudo insmod hexapod_driver.ko'"
print_message "\nHappy coding!"