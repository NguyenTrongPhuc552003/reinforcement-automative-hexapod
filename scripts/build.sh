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
    echo "  (no argument)       Build all components"
    echo "  -i, --image [TYPE]  Build Docker images (optional TYPE: app|driver|pytd3)"
    echo "  -b, --build [TYPE]  Build components (TYPE: app|driver|pytd3|all)"
    echo "  -s, --setup         Setup Python virtual environment for PyTD3"
    echo "  -l, --uml           Build UML diagrams (no Docker required)"
    echo "  -t, --utility       Create utility scripts (no Docker required)"
    echo "  -c, --clean [TYPE]  Clean build artifacts (optional TYPE: app|driver|pytd3|deploy)"
    echo "  -p, --purge [TYPE]  Purge all build artifacts and Docker images (optional TYPE: app|driver|pytd3)"
    echo "  -n, --no-cache      Build all without cache"
    echo "  -h, --help          Show this help message"
    echo ""
    echo "Examples:"
    echo "  ./scripts/build.sh -i app        Build only the app Docker image"
    echo "  ./scripts/build.sh -b app        Build only user space programs"
    echo "  ./scripts/build.sh -b driver     Build only kernel modules"
    echo "  ./scripts/build.sh -b pytd3      Build only PyTD3 module"
    echo "  ./scripts/build.sh -b            Build all components (app, driver, pytd3)"
    echo "  ./scripts/build.sh -ib app       Build app image and components"
    echo "  ./scripts/build.sh -ib           Build all images and components"
    echo "  ./scripts/build.sh -c driver     Clean only the driver build artifacts"
    echo "  ./scripts/build.sh -p pytd3      Purge only the PyTD3 Docker image"
    echo ""
    echo "Options can be combined, e.g., -bt to build all components & utility scripts"
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
            build_docker_image "pytd3" ${cache_flag}
            ;;
        *)
            log "${RED}" "Unknown image type: ${image_type}"
            exit 1
            ;;
    esac
    
    log "${GREEN}" "Docker image ${image_type} built successfully!"
}

# Function to build kernel module (now called by driver build type)
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

# Function to build user space program (now called by app build type)
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

# Initialize option flags
DO_CLEAN=0
DO_IMAGE=0
DO_BUILD=0
DO_UTILITY=0
DO_UML=0
DO_NO_CACHE=0
DO_SETUP_ENV=0
DO_PURGE=0
IMAGE_TYPE="all" # Default to all images
BUILD_TYPE="all" # Default to all components
CLEAN_TYPE="all" # Default to all modules
PURGE_TYPE="all" # Default to all images

# Parse command-line arguments
if [ $# -eq 0 ]; then
    # Default: build everything
    DO_IMAGE=1
    DO_BUILD=1
    BUILD_TYPE="all"
    DO_UTILITY=1
    DO_UML=1
    DO_SETUP_ENV=1
else
    i=0
    while [ $i -lt $# ]; do
        i=$((i+1))
        arg="${!i}"
        
        if [[ "$arg" == "--no-cache" || "$arg" == "-n" ]]; then
            DO_NO_CACHE=1
            DO_DOCKER_REQUIRED=1
        elif [[ "$arg" == "--help" || "$arg" == "-h" ]]; then
            usage
        elif [[ "$arg" == "--clean" || "$arg" == "-c" ]]; then
            DO_CLEAN=1
            # Check if next argument specifies a module to clean
            next_i=$((i+1))
            if [ $next_i -le $# ]; then
                next_arg="${!next_i}"
                if [[ "$next_arg" == "app" || "$next_arg" == "driver" || "$next_arg" == "pytd3" || "$next_arg" == "deploy" ]]; then
                    CLEAN_TYPE="$next_arg"
                    i=$next_i  # Skip the next argument since we've consumed it
                fi
            fi
        elif [[ "$arg" == "--purge" || "$arg" == "-p" ]]; then
            DO_PURGE=1
            # Check if next argument specifies a module to purge
            next_i=$((i+1))
            if [ $next_i -le $# ]; then
                next_arg="${!next_i}"
                if [[ "$next_arg" == "app" || "$next_arg" == "driver" || "$next_arg" == "pytd3" ]]; then
                    PURGE_TYPE="$next_arg"
                    i=$next_i  # Skip the next argument since we've consumed it
                fi
            fi
        elif [[ "$arg" == "--image" || "$arg" == "-i" ]]; then
            DO_IMAGE=1
            DO_DOCKER_REQUIRED=1
            # Check if next argument is an image type
            next_i=$((i+1))
            if [ $next_i -le $# ]; then
                next_arg="${!next_i}"
                if [[ "$next_arg" == "app" || "$next_arg" == "driver" || "$next_arg" == "pytd3" ]]; then
                    IMAGE_TYPE="$next_arg"
                    i=$next_i  # Skip the next argument since we've consumed it
                fi
            fi
        elif [[ "$arg" == "--build" || "$arg" == "-b" ]]; then
            DO_BUILD=1
            DO_DOCKER_REQUIRED=1
            # Check if next argument is a build type
            next_i=$((i+1))
            if [ $next_i -le $# ]; then
                next_arg="${!next_i}"
                if [[ "$next_arg" == "app" || "$next_arg" == "driver" || "$next_arg" == "pytd3" || "$next_arg" == "all" ]]; then
                    BUILD_TYPE="$next_arg"
                    i=$next_i  # Skip the next argument since we've consumed it
                fi
            fi
        elif [[ "$arg" == "--utility" || "$arg" == "-t"  ]]; then
            DO_UTILITY=1
        elif [[ "$arg" == "--setup" || "$arg" == "-s" ]]; then
            DO_SETUP_ENV=1
            DO_DOCKER_REQUIRED=1
        elif [[ "$arg" == "--uml" || "$arg" == "-l" ]]; then
            DO_UML=1
        elif [[ "$arg" == -* && "$arg" != "--"* ]]; then
            # Process combined short options like -tbl
            flags=${arg#-}
            for (( j=0; j<${#flags}; j++ )); do
                flag=${flags:$j:1}
                case "$flag" in
                    c) DO_CLEAN=1 ;;
                    p) DO_PURGE=1 ;;
                    i) DO_IMAGE=1; DO_DOCKER_REQUIRED=1 ;;
                    b) DO_BUILD=1; DO_DOCKER_REQUIRED=1 ;;
                    t) DO_UTILITY=1 ;;
                    s) DO_SETUP_ENV=1; DO_DOCKER_REQUIRED=1 ;;
                    l) DO_UML=1 ;;
                    n) DO_NO_CACHE=1; DO_DOCKER_REQUIRED=1 ;;
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

# Function to clean app specific artifacts
clean_app() {
    local anything_cleaned=0
    
    # Check if app output directory exists
    if [ -d "${USER_SPACE_DIR}/bin" ]; then
        log "${GREEN}" "Cleaning user space binaries..."
        if ! rm -rf "${USER_SPACE_DIR}/bin" 2>/dev/null; then
            log "${YELLOW}" "Using sudo to clean user space binaries..."
            sudo rm -rf "${USER_SPACE_DIR}/bin"
        fi
        anything_cleaned=1
    else
        log "${YELLOW}" "No user space binaries found to clean"
    fi
    
    # Clean using app Docker image if available
    if command -v docker &> /dev/null && docker image inspect hexapod-app &> /dev/null; then
        log "${YELLOW}" "Cleaning user space applications using Docker..."
        docker run --rm \
            -v "${USER_SPACE_DIR}:/build/user" \
            hexapod-app clean || true
        anything_cleaned=1
    fi
    
    if [ $anything_cleaned -eq 0 ]; then
        log "${GREEN}" "Nothing to clean for app module"
    fi
}

# Function to clean driver specific artifacts
clean_driver() {
    local anything_cleaned=0
    
    # Check if driver build artifacts exist
    if [ -d "${KERNEL_MODULE_DIR}/obj" ] || [ -d "${KERNEL_MODULE_DIR}/cmd" ] || \
       [ -d "${KERNEL_MODULE_DIR}/deps" ] || [ -f "${KERNEL_MODULE_DIR}/hexapod_driver.ko" ]; then
        anything_cleaned=1
    fi
    
    # Clean using driver Docker image if available
    if command -v docker &> /dev/null && docker image inspect hexapod-driver &> /dev/null; then
        log "${YELLOW}" "Cleaning kernel modules using Docker..."
        docker run --rm \
            -v "${KERNEL_MODULE_DIR}:/build/module" \
            hexapod-driver clean || true
        anything_cleaned=1
    fi
    
    if [ $anything_cleaned -eq 0 ]; then
        log "${GREEN}" "Nothing to clean for driver module"
    fi
}

# Function to clean pytd3 specific artifacts
clean_pytd3() {
    local anything_cleaned=0
    
    # Check if pytd3 build artifacts exist
    if [ -d "${PYTD3_DIR}/build" ] || [ -d "${PYTD3_DIR}/venv" ]; then
        log "${GREEN}" "Cleaning PyTD3 build and virtual environment..."
        if ! rm -rf "${PYTD3_DIR}/build" "${PYTD3_DIR}/venv" 2>/dev/null; then
            log "${YELLOW}" "Using sudo to clean PyTD3 directories..."
            sudo rm -rf "${PYTD3_DIR}/build" "${PYTD3_DIR}/venv"
        fi
        anything_cleaned=1
    else
        log "${YELLOW}" "No PyTD3 build artifacts found to clean"
    fi
    
    # Clean using pytd3 Docker image if available
    if command -v docker &> /dev/null && docker image inspect hexapod-pytd3 &> /dev/null; then
        log "${YELLOW}" "Cleaning PyTD3 using Docker..."
        docker run --rm \
            -v "${PYTD3_DIR}:/build/pytd3" \
            hexapod-pytd3 clean || true
        anything_cleaned=1
    fi
    
    if [ $anything_cleaned -eq 0 ]; then
        log "${GREEN}" "Nothing to clean for PyTD3 module"
    fi
}

# Function to clean deploy directory
clean_deploy() {
    local anything_cleaned=0
    
    # Check if deploy directory exists
    if [ -d "${DEPLOY_DIR}" ]; then
        log "${YELLOW}" "Cleaning deploy directory..."
        if ! rm -rf "${DEPLOY_DIR}"/* 2>/dev/null; then
            log "${YELLOW}" "Using sudo to clean deploy directory..."
            sudo rm -rf "${DEPLOY_DIR}"/*
        fi
        anything_cleaned=1
    else
        log "${GREEN}" "Nothing to clean in deploy directory"
    fi
    
    return $anything_cleaned
}

# Function to purge Docker images
purge_docker_images() {
    local image_type=$1
    local anything_purged=0
    
    # Check if Docker is available
    if ! command -v docker &> /dev/null; then
        log "${RED}" "Docker is not installed or not in PATH"
        return 0
    fi
    
    case "$image_type" in
        app)
            if docker image inspect hexapod-app &> /dev/null; then
                log "${YELLOW}" "Removing hexapod-app Docker image..."
                docker rmi hexapod-app || true
                anything_purged=1
            else
                log "${GREEN}" "No hexapod-app Docker image found to purge"
            fi
            ;;
        driver)
            if docker image inspect hexapod-driver &> /dev/null; then
                log "${YELLOW}" "Removing hexapod-driver Docker image..."
                docker rmi hexapod-driver || true
                anything_purged=1
            else
                log "${GREEN}" "No hexapod-driver Docker image found to purge"
            fi
            ;;
        pytd3)
            if docker image inspect hexapod-pytd3 &> /dev/null; then
                log "${YELLOW}" "Removing hexapod-pytd3 Docker image..."
                docker rmi hexapod-pytd3 || true
                anything_purged=1
            else
                log "${GREEN}" "No hexapod-pytd3 Docker image found to purge"
            fi
            ;;
        all)
            local purged_any=0
            if docker image inspect hexapod-app &> /dev/null; then
                log "${YELLOW}" "Removing hexapod-app Docker image..."
                docker rmi hexapod-app || true
                purged_any=1
            fi
            if docker image inspect hexapod-driver &> /dev/null; then
                log "${YELLOW}" "Removing hexapod-driver Docker image..."
                docker rmi hexapod-driver || true
                purged_any=1
            fi
            if docker image inspect hexapod-pytd3 &> /dev/null; then
                log "${YELLOW}" "Removing hexapod-pytd3 Docker image..."
                docker rmi hexapod-pytd3 || true
                purged_any=1
            fi
            
            if [ $purged_any -eq 0 ]; then
                log "${GREEN}" "No hexapod Docker images found to purge"
            else
                anything_purged=1
            fi
            ;;
    esac
    
    return $anything_purged
}

# Handle clean first (as it exits)
if [ $DO_CLEAN -eq 1 ]; then
    log "${YELLOW}" "Cleaning build artifacts..."
    
    case "$CLEAN_TYPE" in
        app)
            clean_app
            ;;
        driver)
            clean_driver
            ;;
        pytd3)
            clean_pytd3
            ;;
        deploy)
            clean_deploy
            ;;
        all)
            # Clean all components
            clean_app
            clean_driver
            clean_pytd3
            clean_deploy
            
            # Clean UML diagrams without requiring Docker
            log "${YELLOW}" "Cleaning UML diagram files..."
            
            # Check if export.sh exists
            if [ -f "${PROJECT_ROOT}/scripts/export.sh" ]; then
                bash "${PROJECT_ROOT}/scripts/export.sh" -c
            else
                log "${RED}" "export.sh not found in scripts directory!"
            fi
            ;;
    esac
    
    log "${GREEN}" "Clean completed successfully!"
    exit 0
fi

# Handle purge option
if [ $DO_PURGE -eq 1 ]; then
    case "$PURGE_TYPE" in
        app|driver|pytd3)
            purge_docker_images "$PURGE_TYPE"
            ;;
        all)
            # Remove Docker containers, images, volumes, and builders
            log "${YELLOW}" "Checking for Docker containers to purge..."
            if docker ps -a | grep -q hexapod; then
                log "${YELLOW}" "Pruning all Docker containers..."
                docker container prune -f
            else
                log "${GREEN}" "No Docker containers found to purge"
            fi
            
            purge_docker_images "all"
            
            log "${YELLOW}" "Checking for Docker volumes to purge..."
            if docker volume ls | grep -q hexapod; then
                log "${YELLOW}" "Pruning all Docker volumes..."
                docker volume prune -f
            else
                log "${GREEN}" "No Docker volumes found to purge"
            fi
            
            log "${YELLOW}" "Pruning Docker builder cache..."
            docker builder prune --all -f
            ;;
    esac
    
    log "${GREEN}" "Purge completed!"
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
    cache_flag=""
    if [ $DO_NO_CACHE -eq 1 ]; then
        cache_flag="--no-cache"
    fi
    build_docker_image "${IMAGE_TYPE}" "${cache_flag}"
    COMPONENTS_BUILT=1
fi

# Setup Python virtual environment if requested (must be before building PyTD3)
if [ $DO_SETUP_ENV -eq 1 ]; then
    setup_pytd3_env
    COMPONENTS_BUILT=1
fi

# Build Docker-dependent components using the unified -b/--build option
if [ $DO_BUILD -eq 1 ]; then
    case "$BUILD_TYPE" in
        app)
            build_user_space
            log "${GREEN}" "User space programs build completed successfully!"
            COMPONENTS_BUILT=1
            ;;
        driver)
            build_device_driver
            log "${GREEN}" "Kernel modules build completed successfully!"
            COMPONENTS_BUILT=1
            ;;
        pytd3)
            build_pytd3
            log "${GREEN}" "PyTD3 build completed successfully!"
            COMPONENTS_BUILT=1
            ;;
        all)
            # Build all components in order
            build_user_space
            log "${GREEN}" "User space programs build completed successfully!"
            
            build_device_driver
            log "${GREEN}" "Kernel modules build completed successfully!"
            
            build_pytd3
            log "${GREEN}" "PyTD3 build completed successfully!"
            
            COMPONENTS_BUILT=1
            ;;
        *)
            log "${RED}" "Unknown build type: ${BUILD_TYPE}"
            log "${YELLOW}" "Valid build types: app, driver, pytd3, all"
            exit 1
            ;;
    esac
fi

# Print overall success message if anything was built
if [ $COMPONENTS_BUILT -eq 1 ]; then
    log "${GREEN}" "Build completed successfully!"
    log "${GREEN}" "Deployment package created in: ${DEPLOY_DIR}"
fi