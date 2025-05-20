#!/bin/bash

# Exit on any error
set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Get the project root directory
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KERNEL_MODULE_DIR="${PROJECT_ROOT}/driver"
USER_SPACE_DIR="${PROJECT_ROOT}/app"
PYTD3_DIR="${PROJECT_ROOT}/pytd3"
DEPLOY_DIR="${PROJECT_ROOT}/deploy"
UTILS_DIR="${PROJECT_ROOT}/utils"

# Create deploy directory with "whoami" user permission if it doesn't exist
if [ ! -d "${DEPLOY_DIR}" ]; then
    mkdir -p "${DEPLOY_DIR}"
    chown -R "$(whoami)":"$(whoami)" "${DEPLOY_DIR}"
fi

# Function to show usage
usage() {
    echo "Usage: $0 [OPTIONS]"
    echo "Options:"
    echo "  (no argument)     Build all components"
    echo "  -m, module        Build kernel modules"
    echo "  -u, user          Build user space programs"
    echo "  -d, pytd3         Build PyTD3 reinforcement learning module"
    echo "  -s, setup         Setup Python virtual environment for PyTD3"
    echo "  -l, uml           Build UML diagrams (no Docker required)"
    echo "  -t, utility       Create utility scripts (no Docker required)"
    echo "  -c, clean         Clean build artifacts"
    echo "  -n, --no-cache    Build all without cache"
    echo "  -h, --help        Show this help message"
    echo ""
    echo "Options can be combined, e.g., -mt to build modules & utility scripts"
    echo "Note: UML diagrams (-l) and utility scripts (-t) can be built without Docker"
    echo ""
    exit 1
}

# Function to print colored messages
log() {
    local color=$1
    shift
    echo -e "${color}$@${NC}"
}

# Function to build Docker image
build_docker_image() {
    local cache_flag=$1
    log "${YELLOW}" "Preparing Docker image..."
    
    # Check if Docker is installed and running
    if ! command -v docker &> /dev/null; then
        log "${RED}" "Docker is not installed or not in PATH"
        exit 1
    fi
    
    # Verify entrypoint.sh exists
    if [ ! -f "${PROJECT_ROOT}/entrypoint.sh" ]; then
        log "${RED}" "entrypoint.sh not found in project root!"
        exit 1
    fi
    
    # Build the Docker image
    docker build ${cache_flag} -t hexapod-builder "${PROJECT_ROOT}" || {
        log "${RED}" "Docker build failed!"
        exit 1
    }
    
    # Verify image was created
    if ! docker images | grep -q hexapod-builder; then
        log "${RED}" "Failed to create Docker image 'hexapod-builder'"
        exit 1
    fi
    
    log "${GREEN}" "Docker image 'hexapod-builder' created successfully"
}

# Function to build kernel module
build_kernel_module() {
    log "${YELLOW}" "Preparing kernel modules..."
    
    # Release build by default, add DEBUG=1 for debug build
    local build_type="release"
    if [ "$DEBUG" = "1" ]; then
        build_type="debug"
        log "${YELLOW}" "Building DEBUG version of kernel modules..."
    else
        log "${YELLOW}" "Building RELEASE version of kernel modules..."
    fi
    
    docker run --rm \
        -v "${KERNEL_MODULE_DIR}:/build/module" \
        -v "${DEPLOY_DIR}:/build/deploy" \
        -e BUILD_TYPE=${build_type} \
        hexapod-builder module || {
        log "${RED}" "Kernel modules build failed!"
        exit 1
    }
}

# Function to build user space program
build_user_space() {
    log "${YELLOW}" "Preparing user space programs..."
    
    # Release build by default, add DEBUG=1 for debug build
    local env_params=""
    if [ "$DEBUG" = "1" ]; then
        env_params="-e DEBUG=1"
        log "${YELLOW}" "Building DEBUG version of user space programs..."
    else
        log "${YELLOW}" "Building RELEASE version of user space programs..."
    fi
    
    # Run Docker with proper command format
    docker run --rm \
        -v "${USER_SPACE_DIR}:/build/user" \
        -v "${DEPLOY_DIR}:/build/deploy" \
        ${env_params} \
        hexapod-builder user || {
        log "${RED}" "User space programs build failed!"
        exit 1
    }
}

# Function to setup Python virtual environment for PyTD3
setup_pytd3_env() {
    log "${YELLOW}" "Setting up Python virtual environment for PyTD3..."
    
    # Run Docker with proper command format
    docker run --rm \
        -v "${PYTD3_DIR}:/build/pytd3" \
        -v "${DEPLOY_DIR}:/build/deploy" \
        hexapod-builder setup_env || {
        log "${RED}" "PyTD3 environment setup failed!"
        exit 1
    }
    
    # Create default config directory if it doesn't exist
    if [ ! -d "${PYTD3_DIR}/config" ]; then
        mkdir -p "${PYTD3_DIR}/config"
    fi
    
    log "${GREEN}" "PyTD3 environment setup successful!"
}

# Function to build PyTD3 module
build_pytd3() {
    log "${YELLOW}" "Building PyTD3 module..."
    
    # Create build directory if it doesn't exist
    if [ ! -d "${PYTD3_DIR}/build" ]; then
        mkdir -p "${PYTD3_DIR}/build"
    fi
    
    # Run Docker with proper command format
    docker run --rm \
        -v "${PYTD3_DIR}:/build/pytd3" \
        -v "${USER_SPACE_DIR}:/build/app" \
        -v "${DEPLOY_DIR}:/build/deploy" \
        hexapod-builder pytd3
    
    if [ $? -eq 0 ]; then
        log "${GREEN}" "PyTD3 build successful!"
        # Copy relevant files to deploy directory (if needed)
        mkdir -p "${DEPLOY_DIR}/pytd3"
        cp -r "${PYTD3_DIR}/build/"*.so \
            "${PYTD3_DIR}/build/pytd3_"* \
            "${DEPLOY_DIR}/pytd3/" 2>/dev/null || true
    else
        log "${RED}" "PyTD3 build failed!"
        return 1
    fi
}

# Function to build UML diagrams using export.sh script (no Docker required)
build_uml_diagrams() {
    log "${YELLOW}" "Building UML diagrams..."
    
    # Check if export.sh exists
    if [ ! -f "${PROJECT_ROOT}/scripts/export.sh" ]; then
        log "${RED}" "export.sh not found in scripts directory!"
        exit 1
    fi
    
    # Run export.sh script directly (no Docker needed)
    bash "${PROJECT_ROOT}/scripts/export.sh"
}

# Function to create utility scripts (no Docker required)
build_utility_scripts() {
    log "${YELLOW}" "Copying utility scripts..."
    
    # Create deploy directory if it doesn't exist
    if [ ! -d "${DEPLOY_DIR}" ]; then
        mkdir -p "${DEPLOY_DIR}"
    fi
    
    # Copy installation script
    if [ -f "${UTILS_DIR}/install.sh" ]; then
        cp "${UTILS_DIR}/install.sh" "${DEPLOY_DIR}/"
        chmod +x "${DEPLOY_DIR}/install.sh"
        log "${GREEN}" "Copied install.sh to deploy directory"
    else
        log "${RED}" "install.sh not found in utils directory!"
    fi
    
    # Copy monitoring script
    if [ -f "${UTILS_DIR}/monitor.sh" ]; then
        cp "${UTILS_DIR}/monitor.sh" "${DEPLOY_DIR}/"
        chmod +x "${DEPLOY_DIR}/monitor.sh"
        log "${GREEN}" "Copied monitor.sh to deploy directory"
    else
        log "${RED}" "monitor.sh not found in utils directory!"
    fi
    
    log "${GREEN}" "Utility scripts prepared successfully"
}

# Function to clean UML diagrams (no Docker required)
clean_uml_diagrams() {
    log "${YELLOW}" "Cleaning UML diagram files..."
    
    # Check if export.sh exists
    if [ ! -f "${PROJECT_ROOT}/scripts/export.sh" ]; then
        log "${RED}" "export.sh not found in scripts directory!"
        return 1
    fi
    
    # Run export.sh script with clean option
    bash "${PROJECT_ROOT}/scripts/export.sh" -c    
}

# Initialize option flags
DO_CLEAN=0
DO_MODULE=0
DO_USER=0
DO_UTILITY=0
DO_UML=0
DO_NO_CACHE=0
DO_PYTD3=0
DO_SETUP_ENV=0
DO_DOCKER_REQUIRED=0

# Parse command-line arguments
if [ $# -eq 0 ]; then
    # Default: build everything
    DO_MODULE=1
    DO_UTILITY=1
    DO_UML=1
    DO_USER=1
    DO_PYTD3=1
    DO_DOCKER_REQUIRED=1
else
    for arg in "$@"; do
        if [[ "$arg" == "--no-cache" ]]; then
            DO_NO_CACHE=1
        elif [[ "$arg" == "--help" || "$arg" == "-h" ]]; then
            usage
        elif [[ "$arg" == "clean" || "$arg" == "-c" ]]; then
            DO_CLEAN=1
        elif [[ "$arg" == "module" || "$arg" == "-m"  ]]; then
            DO_MODULE=1
            DO_DOCKER_REQUIRED=1
        elif [[ "$arg" == "user" || "$arg" == "-u"  ]]; then
            DO_USER=1
            DO_DOCKER_REQUIRED=1
        elif [[ "$arg" == "utility" || "$arg" == "-t"  ]]; then
            DO_UTILITY=1
        elif [[ "$arg" == "pytd3" || "$arg" == "-d" ]]; then
            DO_PYTD3=1
            DO_DOCKER_REQUIRED=1
        elif [[ "$arg" == "setup" || "$arg" == "-s" ]]; then
            DO_SETUP_ENV=1
            DO_DOCKER_REQUIRED=1
        elif [[ "$arg" == "uml" || "$arg" == "-l" ]]; then
            DO_UML=1
        elif [[ "$arg" == -* && "$arg" != "--"* ]]; then
            # Process combined short options like -imu
            flags=${arg#-}
            for (( i=0; i<${#flags}; i++ )); do
                flag=${flags:$i:1}
                case "$flag" in
                    c) DO_CLEAN=1 ;;
                    m) DO_MODULE=1; DO_DOCKER_REQUIRED=1 ;;
                    u) DO_USER=1; DO_DOCKER_REQUIRED=1 ;;
                    t) DO_UTILITY=1 ;;
                    d) DO_PYTD3=1; DO_DOCKER_REQUIRED=1 ;;
                    s) DO_SETUP_ENV=1; DO_DOCKER_REQUIRED=1 ;;
                    l) DO_UML=1 ;;
                    n) DO_NO_CACHE=1 ;;
                    h) usage ;;
                    *) 
                        log "${RED}" "Unknown option: -$flag"
                        usage
                        ;;
                esac
            done
        else
            log "${RED}" "Unknown argument: $arg"
            usage
        fi
    done
fi

# Handle clean first (as it exits)
if [ $DO_CLEAN -eq 1 ]; then
    log "${YELLOW}" "Cleaning build artifacts..."
    
    # Clean deploy directory utility scripts (no Docker required)
    if [ -d "${DEPLOY_DIR}" ]; then
        log "${YELLOW}" "Cleaning deploy directory..."
        if ! rm -rf "${DEPLOY_DIR}" 2>/dev/null; then
            log "${YELLOW}" "Using sudo to clean deploy directory..."
            sudo rm -rf "${DEPLOY_DIR}"
        fi
    fi
    
    # Docker-based cleaning (only if Docker is available)
    if command -v docker &> /dev/null && docker images | grep -q hexapod-builder; then
        log "${YELLOW}" "Cleaning Docker-based components..."
        # Clean kernel driver and user space using Docker
        docker run --rm \
            -v "${KERNEL_MODULE_DIR}:/build/module" \
            -v "${USER_SPACE_DIR}:/build/user" \
            hexapod-builder clean || {
            # If Docker run failed for a reason other than missing image
            if [ $? -ne 125 ]; then
                log "${RED}" "Docker clean failed!"
                exit 1
            fi
        }
        
        # Clean user space binaries with sudo if needed
        if [ -d "${USER_SPACE_DIR}/bin" ]; then
            log "${GREEN}" "Cleaning user space binaries..."
            if ! rm -rf "${USER_SPACE_DIR}/bin" 2>/dev/null; then
                log "${YELLOW}" "Using sudo to clean user space binaries..."
                sudo rm -rf "${USER_SPACE_DIR}/bin"
            fi
        fi

        # Clean PyTD3 build directory and virtual environment with sudo if needed
        if [ -d "${PYTD3_DIR}/build" ] || [ -d "${PYTD3_DIR}/venv" ]; then
            log "${GREEN}" "Cleaning PyTD3 build and virtual environment..."
            if ! rm -rf "${PYTD3_DIR}/build" "${PYTD3_DIR}/venv" 2>/dev/null; then
                log "${YELLOW}" "Using sudo to clean PyTD3 directories..."
                sudo rm -rf "${PYTD3_DIR}/build" "${PYTD3_DIR}/venv"
            fi
        fi
    else
        log "${YELLOW}" "Docker not available or image doesn't exist, skipping Docker-based cleaning"
    fi

    # Clean UML diagrams without requiring Docker
    clean_uml_diagrams
    
    log "${GREEN}" "Clean completed successfully!"
    exit 0
fi

# Components that don't require Docker can be built first
COMPONENTS_BUILT=0

# Build utility scripts (no Docker required)
if [ $DO_UTILITY -eq 1 ]; then
    build_utility_scripts
    log "${GREEN}" "Utility scripts created successfully!"
    COMPONENTS_BUILT=1
fi

# Build UML diagrams (no Docker required)
if [ $DO_UML -eq 1 ]; then
    build_uml_diagrams
    # COMPONENTS_BUILT=1
fi

# Only prepare Docker if needed for other components
if [ $DO_DOCKER_REQUIRED -eq 1 ]; then
    # Handle Docker image building with or without cache
    if [ $DO_NO_CACHE -eq 1 ]; then
        # Remove Docker image if it exists
        if docker images | grep -q hexapod-builder; then
            log "${GREEN}" "Removing Docker image..."
            docker rmi hexapod-builder || true
        fi
        build_docker_image "--no-cache"
    else
        build_docker_image ""
    fi

    # Setup Python virtual environment if requested (must be before building PyTD3)
    if [ $DO_SETUP_ENV -eq 1 ]; then
        setup_pytd3_env
        COMPONENTS_BUILT=1
    fi
    
    # Build Docker-dependent components
    if [ $DO_MODULE -eq 1 ]; then
        build_kernel_module
        log "${GREEN}" "Kernel modules build completed successfully!"
        COMPONENTS_BUILT=1
    fi

    if [ $DO_USER -eq 1 ]; then
        build_user_space
        log "${GREEN}" "User space programs build completed successfully!"
        COMPONENTS_BUILT=1
    fi

    # Build PyTD3 if requested
    if [ $DO_PYTD3 -eq 1 ]; then
        build_pytd3 || EXIT_CODE=1
        COMPONENTS_BUILT=1
    fi
fi

# Print overall success message if anything was built
if [ $COMPONENTS_BUILT -eq 1 ]; then
    log "${GREEN}" "Build completed successfully!"
    log "${GREEN}" "Deployment package created in: ${DEPLOY_DIR}"
fi