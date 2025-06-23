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
    echo "  -i, --image       Build Docker images"
    echo "  -m, --module      Build kernel modules"
    echo "  -k, --kernel      Build kernel and builtin drivers"
    echo "  -u, --user        Build user space programs"
    echo "  -d, --pytd3       Build PyTD3 reinforcement learning module"
    echo "  -s, --setup       Setup Python virtual environment for PyTD3"
    echo "  -l, --uml         Build UML diagrams (no Docker required)"
    echo "  -t, --utility     Create utility scripts (no Docker required)"
    echo "  -c, --clean       Clean build artifacts"
    echo "  -p, --purge       Purge all build artifacts and Docker images"
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
    local image_type=$1
    local cache_flag=$2
    
    log "${YELLOW}" "Building Docker image: ${image_type}"
    
    # Check if Docker is installed and running
    if ! command -v docker &> /dev/null; then
        log "${RED}" "Docker is not installed or not in PATH"
        exit 1
    fi
    
    # Verify Docker directory and Dockerfile exist
    if [ ! -d "${PROJECT_ROOT}/docker" ]; then
        log "${RED}" "docker/ directory not found in project root!"
        exit 1
    fi
    
    # Build the specified Docker image
    case "${image_type}" in
        app)
            if [ ! -f "${PROJECT_ROOT}/docker/app.Dockerfile" ]; then
                log "${RED}" "app.Dockerfile not found in docker/ directory!"
                exit 1
            fi
            docker build ${cache_flag} -t hexapod-app -f "${PROJECT_ROOT}/docker/app.Dockerfile" "${PROJECT_ROOT}" || {
                log "${RED}" "Docker build for app failed!"
                exit 1
            }
            ;;
        driver)
            if [ ! -f "${PROJECT_ROOT}/docker/driver.Dockerfile" ]; then
                log "${RED}" "driver.Dockerfile not found in docker/ directory!"
                exit 1
            fi
            docker build ${cache_flag} -t hexapod-driver -f "${PROJECT_ROOT}/docker/driver.Dockerfile" "${PROJECT_ROOT}" || {
                log "${RED}" "Docker build for driver failed!"
                exit 1
            }
            ;;
        kernel)
            if [ ! -f "${PROJECT_ROOT}/docker/kernel.Dockerfile" ]; then
                log "${RED}" "kernel.Dockerfile not found in docker/ directory!"
                exit 1
            fi
            docker build ${cache_flag} -t hexapod-kernel -f "${PROJECT_ROOT}/docker/kernel.Dockerfile" "${PROJECT_ROOT}" || {
                log "${RED}" "Docker build for kernel failed!"
                exit 1
            }
            ;;
        pytd3)
            if [ ! -f "${PROJECT_ROOT}/docker/pytd3.Dockerfile" ]; then
                log "${RED}" "pytd3.Dockerfile not found in docker/ directory!"
                exit 1
            fi
            docker build ${cache_flag} -t hexapod-pytd3 -f "${PROJECT_ROOT}/docker/pytd3.Dockerfile" "${PROJECT_ROOT}" || {
                log "${RED}" "Docker build for PyTD3 failed!"
                exit 1
            }
            ;;
        all)
            # Build all images
            log "${YELLOW}" "Building all Docker images..."
            build_docker_image "app" ${cache_flag}
            build_docker_image "driver" ${cache_flag}
            build_docker_image "kernel" ${cache_flag}
            build_docker_image "pytd3" ${cache_flag}
            ;;
        *)
            log "${RED}" "Unknown image type: ${image_type}"
            exit 1
            ;;
    esac
    
    log "${GREEN}" "Docker image ${image_type} built successfully!"
}

# Function to build kernel module
build_device_driver() {
    log "${YELLOW}" "Preparing kernel modules..."
    
    # First build the driver image if not already built
    if ! docker image inspect hexapod-driver &> /dev/null; then
        build_docker_image "driver" ""
    fi
    
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
        hexapod-driver module || {
        log "${RED}" "Kernel modules build failed!"
        exit 1
    }
}

# Function to build user space program
build_user_space() {
    log "${YELLOW}" "Preparing user space programs..."
    
    # First build the app image if not already built
    if ! docker image inspect hexapod-app &> /dev/null; then
        build_docker_image "app" ""
    fi
    
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
        hexapod-app user || {
        log "${RED}" "User space programs build failed!"
        exit 1
    }
}

# Function to setup Python virtual environment for PyTD3
setup_pytd3_env() {
    log "${YELLOW}" "Setting up Python virtual environment for PyTD3..."
    
    # First build the pytd3 image if not already built
    if ! docker image inspect hexapod-pytd3 &> /dev/null; then
        build_docker_image "pytd3" ""
    fi
    
    # Run Docker with proper command format
    docker run --rm \
        -v "${PYTD3_DIR}:/build/pytd3" \
        -v "${DEPLOY_DIR}:/build/deploy" \
        hexapod-pytd3 setup_env || {
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
    
    # First build the pytd3 image if not already built
    if ! docker image inspect hexapod-pytd3 &> /dev/null; then
        build_docker_image "pytd3" ""
    fi
    
    # Create build directory if it doesn't exist
    if [ ! -d "${PYTD3_DIR}/build" ]; then
        mkdir -p "${PYTD3_DIR}/build"
    fi
    
    # Run Docker with proper command format
    docker run --rm \
        -v "${PYTD3_DIR}:/build/pytd3" \
        -v "${DEPLOY_DIR}:/build/deploy" \
        hexapod-pytd3 pytd3
    
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

# Function to build kernel and builtin drivers
build_kernel_enable() {
    log "${YELLOW}" "Building kernel and builtin drivers..."
    
    # First build the kernel image if not already built
    if ! docker image inspect hexapod-kernel &> /dev/null; then
        build_docker_image "kernel" ""
    fi

    # Run with terminal settings for menuconfig
    docker run --rm -it \
        -e TERM=xterm-256color \
        -v "${PROJECT_ROOT}/utils/ti-linux-kernel-dev:/build/kernel" \
        -v "${DEPLOY_DIR}:/build/deploy" \
        -w /build/kernel \
        hexapod-kernel kernel || {
        log "${RED}" "Kernel and builtin drivers build failed!"
        exit 1
    }
    log "${GREEN}" "Kernel and builtin drivers build completed successfully!"
}

# Initialize option flags
DO_CLEAN=0
DO_IMAGE=0
DO_MODULE=0
DO_USER=0
DO_UTILITY=0
DO_UML=0
DO_NO_CACHE=0
DO_PYTD3=0
DO_SETUP_ENV=0
DO_KERNEL=0
DO_PURGE=0

# Parse command-line arguments
if [ $# -eq 0 ]; then
    # Default: build everything
    DO_IMAGE=1
    DO_MODULE=1
    DO_UTILITY=1
    DO_UML=1
    DO_USER=1
    DO_SETUP_ENV=1
    DO_PYTD3=1
    DO_KERNEL=1
else
    for arg in "$@"; do
        if [[ "$arg" == "--no-cache" || "$arg" == "-n" ]]; then
            DO_NO_CACHE=1
            DO_DOCKER_REQUIRED=1
        elif [[ "$arg" == "--help" || "$arg" == "-h" ]]; then
            usage
        elif [[ "$arg" == "--clean" || "$arg" == "-c" ]]; then
            DO_CLEAN=1
        elif [[ "$arg" == "--purge" || "$arg" == "-p" ]]; then
            DO_PURGE=1
        elif [[ "$arg" == "--image" || "$arg" == "-i" ]]; then
            DO_IMAGE=1
            DO_DOCKER_REQUIRED=1
        elif [[ "$arg" == "--module" || "$arg" == "-m"  ]]; then
            DO_MODULE=1
            DO_DOCKER_REQUIRED=1
        elif [[ "$arg" == "--user" || "$arg" == "-u"  ]]; then
            DO_USER=1
            DO_DOCKER_REQUIRED=1
        elif [[ "$arg" == "--utility" || "$arg" == "-t"  ]]; then
            DO_UTILITY=1
        elif [[ "$arg" == "--pytd3" || "$arg" == "-d" ]]; then
            DO_PYTD3=1
            DO_DOCKER_REQUIRED=1
        elif [[ "$arg" == "--setup" || "$arg" == "-s" ]]; then
            DO_SETUP_ENV=1
            DO_DOCKER_REQUIRED=1
        elif [[ "$arg" == "--uml" || "$arg" == "-l" ]]; then
            DO_UML=1
        elif [[ "$arg" == "--kernel" || "$arg" == "-k" ]]; then
            DO_KERNEL=1
            DO_DOCKER_REQUIRED=1
        elif [[ "$arg" == -* && "$arg" != "--"* ]]; then
            # Process combined short options like -tmu
            flags=${arg#-}
            for (( i=0; i<${#flags}; i++ )); do
                flag=${flags:$i:1}
                case "$flag" in
                    c) DO_CLEAN=1 ;;
                    p) DO_PURGE=1 ;;
                    i) DO_IMAGE=1; DO_DOCKER_REQUIRED=1 ;;
                    m) DO_MODULE=1; DO_DOCKER_REQUIRED=1 ;;
                    u) DO_USER=1; DO_DOCKER_REQUIRED=1 ;;
                    t) DO_UTILITY=1 ;;
                    d) DO_PYTD3=1; DO_DOCKER_REQUIRED=1 ;;
                    s) DO_SETUP_ENV=1; DO_DOCKER_REQUIRED=1 ;;
                    l) DO_UML=1 ;;
                    n) DO_NO_CACHE=1; DO_DOCKER_REQUIRED=1 ;;
                    k) DO_KERNEL=1; DO_DOCKER_REQUIRED=1 ;;
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
    
    # Clean with Docker only if relevant images exist
    if command -v docker &> /dev/null; then
        # Clean using driver image if it exists
        if docker image inspect hexapod-driver &> /dev/null; then
            log "${YELLOW}" "Cleaning kernel modules..."
            docker run --rm \
                -v "${KERNEL_MODULE_DIR}:/build/module" \
                hexapod-driver clean || true
        fi
        
        # Clean using app image if it exists 
        if docker image inspect hexapod-app &> /dev/null; then
            log "${YELLOW}" "Cleaning user space applications..."
            docker run --rm \
                -v "${USER_SPACE_DIR}:/build/user" \
                hexapod-app clean || true
        fi
        
        # Clean using pytd3 image if it exists
        if docker image inspect hexapod-pytd3 &> /dev/null; then
            log "${YELLOW}" "Cleaning PyTD3..."
            docker run --rm \
                -v "${PYTD3_DIR}:/build/pytd3" \
                hexapod-pytd3 clean || true
        fi
    else
        log "${YELLOW}" "Docker not available, skipping Docker-based cleaning"
    fi
    
    # Clean local directories manually if needed
    if [ -d "${USER_SPACE_DIR}/bin" ]; then
        log "${GREEN}" "Cleaning user space binaries..."
        if ! rm -rf "${USER_SPACE_DIR}/bin" 2>/dev/null; then
            log "${YELLOW}" "Using sudo to clean user space binaries..."
            sudo rm -rf "${USER_SPACE_DIR}/bin"
        fi
    fi

    if [ -d "${PYTD3_DIR}/build" ] || [ -d "${PYTD3_DIR}/venv" ]; then
        log "${GREEN}" "Cleaning PyTD3 build and virtual environment..."
        if ! rm -rf "${PYTD3_DIR}/build" "${PYTD3_DIR}/venv" 2>/dev/null; then
            log "${YELLOW}" "Using sudo to clean PyTD3 directories..."
            sudo rm -rf "${PYTD3_DIR}/build" "${PYTD3_DIR}/venv"
        fi
    fi

    # Clean UML diagrams without requiring Docker
    log "${YELLOW}" "Cleaning UML diagram files..."
    
    # Check if export.sh exists
    if [ -f "${PROJECT_ROOT}/scripts/export.sh" ]; then
        bash "${PROJECT_ROOT}/scripts/export.sh" -c
    else
        log "${RED}" "export.sh not found in scripts directory!"
    fi
    
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

# Handle purge option
if [ $DO_PURGE -eq 1 ]; then
    # Remove Docker containers, images, volumes, and builders
    log "${YELLOW}" "Pruning all Docker containers..."
    sudo docker container prune -f
    log "${YELLOW}" "Pruning all Docker images..."
    sudo docker image prune -a -f
    log "${YELLOW}" "Pruning all Docker volumes..."
    sudo docker volume prune -f
    log "${YELLOW}" "Pruning all Docker builders..."
    sudo docker builder prune --all -f
    log "${GREEN}" "Purge completed!"
    exit 0
fi

# Build images if explicitly requested or needed for components
if [ $DO_IMAGE -eq 1 ]; then
    # Handle Docker image building with or without cache
    local cache_flag=""
    if [ $DO_NO_CACHE -eq 1 ]; then
        cache_flag="--no-cache"
    fi
    build_docker_image "all" "${cache_flag}"
    COMPONENTS_BUILT=1
fi

# Setup Python virtual environment if requested (must be before building PyTD3)
if [ $DO_SETUP_ENV -eq 1 ]; then
    setup_pytd3_env
    COMPONENTS_BUILT=1
fi

# Build Docker-dependent components
if [ $DO_MODULE -eq 1 ]; then
    build_device_driver
    log "${GREEN}" "Kernel modules build completed successfully!"
    COMPONENTS_BUILT=1
fi

if [ $DO_USER -eq 1 ]; then
    build_user_space
    log "${GREEN}" "User space programs build completed successfully!"
    COMPONENTS_BUILT=1
fi

if [ $DO_KERNEL -eq 1 ]; then
    build_kernel_enable
    log "${GREEN}" "Kernel and builtin drivers build completed successfully!"
    COMPONENTS_BUILT=1
fi

if [ $DO_PYTD3 -eq 1 ]; then
    build_pytd3
    log "${GREEN}" "PyTD3 build completed successfully!"
    COMPONENTS_BUILT=1
fi

# Print overall success message if anything was built
if [ $COMPONENTS_BUILT -eq 1 ]; then
    log "${GREEN}" "Build completed successfully!"
    log "${GREEN}" "Deployment package created in: ${DEPLOY_DIR}"
fi