#!/bin/bash

# TD3Learn - Build, Train, Test and Deploy Helper Script
# This script provides an easy interface for working with TD3Learn models

# Exit on error
set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Project paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
TD3_DIR="${PROJECT_ROOT}/td3learn"
MODELS_DIR="${PROJECT_ROOT}/models"

# Create models directory if it doesn't exist
mkdir -p "${MODELS_DIR}"

# Default options
MODE="help"
MODEL_NAME="default_model"
EPOCHS=100
CONFIG="${TD3_DIR}/configs/default.yaml"
TESTING_EPISODES=10
USE_DOCKER=true
USE_HARDWARE=false
USE_TIDL=false
VERBOSE=false

# Display help message
function show_help {
    echo -e "${BLUE}TD3Learn - Build, Train and Test Script${NC}"
    echo
    echo "Usage: $0 [MODE] [OPTIONS]"
    echo
    echo "Modes:"
    echo "  build               Build the TD3Learn module"
    echo "  train               Train a new TD3Learn model"
    echo "  test                Test/evaluate a trained model"
    echo "  deploy              Deploy model to BeagleBone AI"
    echo "  help                Show this help message"
    echo
    echo "Options:"
    echo "  -m, --model NAME    Model name (default: default_model)"
    echo "  -e, --epochs NUM    Number of training epochs (default: 100)"
    echo "  -c, --config FILE   Configuration file (default: configs/default.yaml)"
    echo "  -t, --test NUM      Number of test episodes (default: 10)"
    echo "  --no-docker         Don't use Docker (use local environment)"
    echo "  --hardware          Use hardware for testing (requires connection)"
    echo "  --tidl              Use TIDL acceleration"
    echo "  --debug             Build with debug flags"
    echo "  -v, --verbose       Enable verbose output"
    echo "  -h, --help          Show this help message"
    echo
    echo "Examples:"
    echo "  $0 build                      # Build the TD3Learn module"
    echo "  $0 train -m my_model -e 200   # Train model for 200 epochs"
    echo "  $0 test -m my_model -t 5      # Test model for 5 episodes"
    echo "  $0 deploy -m my_model --tidl  # Deploy model with TIDL acceleration"
    echo "  $0 build --debug              # Build with debug flags enabled"
}

# Parse command line arguments
# First parameter is the mode
if [ $# -gt 0 ]; then
    case "$1" in
        build|train|test|deploy|help)
            MODE="$1"
            shift
            ;;
        *)
            echo -e "${RED}Error: Unknown mode $1${NC}"
            show_help
            exit 1
            ;;
    esac
fi

# Parse remaining options
DEBUG=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        -m|--model)
            MODEL_NAME="$2"
            shift 2
            ;;
        -e|--epochs)
            EPOCHS="$2"
            shift 2
            ;;
        -c|--config)
            CONFIG="$2"
            shift 2
            ;;
        -t|--test)
            TESTING_EPISODES="$2"
            shift 2
            ;;
        --no-docker)
            USE_DOCKER=false
            shift
            ;;
        --hardware)
            USE_HARDWARE=true
            shift
            ;;
        --tidl)
            USE_TIDL=true
            shift
            ;;
        --debug)
            DEBUG=1
            shift
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo -e "${RED}Error: Unknown option $1${NC}"
            show_help
            exit 1
            ;;
    esac
done

# Show help if no mode specified
if [ "$MODE" = "help" ]; then
    show_help
    exit 0
fi

# Build the TD3Learn module
function build_td3learn {
    echo -e "${BLUE}Building TD3Learn module...${NC}"
    
    cd "${PROJECT_ROOT}"
    
    # Pass the DEBUG flag to the build script
    BUILD_CMD="./scripts/build.sh -d"
    if [ "$DEBUG" -eq 1 ]; then
        echo -e "${YELLOW}Building with DEBUG enabled${NC}"
        export DEBUG=1
    fi
    
    if [ "$USE_DOCKER" = true ]; then
        echo -e "${YELLOW}Using Docker build environment${NC}"
        ${BUILD_CMD}
    else
        echo -e "${YELLOW}Using local build environment${NC}"
        cd "${TD3_DIR}" && mkdir -p build && cd build
        
        CMAKE_OPTS=""
        if [ "$USE_TIDL" = false ]; then
            CMAKE_OPTS="${CMAKE_OPTS} -DENABLE_TIDL=OFF"
        fi
        
        if [ "$DEBUG" -eq 1 ]; then
            CMAKE_OPTS="${CMAKE_OPTS} -DCMAKE_BUILD_TYPE=Debug"
        fi
        
        echo "Running: cmake .. ${CMAKE_OPTS}"
        cmake .. ${CMAKE_OPTS} && make -j$(nproc)
    fi
    
    BUILD_RESULT=$?
    if [ $BUILD_RESULT -eq 0 ]; then
        echo -e "${GREEN}TD3Learn build complete!${NC}"
        return 0
    else
        echo -e "${RED}TD3Learn build failed with code $BUILD_RESULT${NC}"
        echo -e "${YELLOW}Try running with --debug for more information${NC}"
        return 1
    fi
}

# Train a new model
function train_model {
    echo -e "${BLUE}Training TD3Learn model '${MODEL_NAME}'...${NC}"
    echo -e "${YELLOW}Training for ${EPOCHS} epochs using config ${CONFIG}${NC}"
    
    # Create model directory if it doesn't exist
    mkdir -p "${MODELS_DIR}/${MODEL_NAME}"
    
    if [ "$USE_DOCKER" = true ]; then
        echo -e "${YELLOW}Using Docker training environment${NC}"
        cd "${PROJECT_ROOT}" && docker-compose run --rm train \
            td3learn_train --config "${CONFIG}" \
            --output "${MODELS_DIR}/${MODEL_NAME}" \
            --epochs "${EPOCHS}" \
            --train
    else
        echo -e "${YELLOW}Using local training environment${NC}"
        "${TD3_DIR}/build/td3learn_train" \
            --config "${CONFIG}" \
            --output "${MODELS_DIR}/${MODEL_NAME}" \
            --epochs "${EPOCHS}" \
            --train
    fi
    
    echo -e "${GREEN}Training complete!${NC}"
    echo -e "Model saved to: ${MODELS_DIR}/${MODEL_NAME}"
}

# Test/evaluate a trained model
function test_model {
    echo -e "${BLUE}Testing TD3Learn model '${MODEL_NAME}'...${NC}"
    
    # Check if model exists
    if [ ! -d "${MODELS_DIR}/${MODEL_NAME}" ]; then
        echo -e "${RED}Model not found: ${MODELS_DIR}/${MODEL_NAME}${NC}"
        exit 1
    fi
    
    # Set hardware flag if enabled
    HARDWARE_FLAG=""
    if [ "$USE_HARDWARE" = true ]; then
        HARDWARE_FLAG="--hardware"
    fi
    
    if [ "$USE_DOCKER" = true ]; then
        echo -e "${YELLOW}Using Docker testing environment${NC}"
        cd "${PROJECT_ROOT}" && docker-compose run --rm train \
            td3learn_run \
            --model "${MODELS_DIR}/${MODEL_NAME}" \
            --episodes "${TESTING_EPISODES}" \
            ${HARDWARE_FLAG}
    else
        echo -e "${YELLOW}Using local testing environment${NC}"
        "${TD3_DIR}/build/td3learn_run" \
            --model "${MODELS_DIR}/${MODEL_NAME}" \
            --episodes "${TESTING_EPISODES}" \
            ${HARDWARE_FLAG}
    fi
    
    echo -e "${GREEN}Testing complete!${NC}"
}

# Deploy model to BeagleBone AI
function deploy_model {
    echo -e "${BLUE}Deploying TD3Learn model '${MODEL_NAME}'...${NC}"
    
    # Check if model exists
    if [ ! -d "${MODELS_DIR}/${MODEL_NAME}" ]; then
        echo -e "${RED}Model not found: ${MODELS_DIR}/${MODEL_NAME}${NC}"
        exit 1
    fi
    
    # Set TIDL flag if enabled
    TIDL_FLAG=""
    if [ "$USE_TIDL" = true ]; then
        TIDL_FLAG="--tidl"
    fi
    
    # Set hardware flag if enabled
    HARDWARE_FLAG=""
    if [ "$USE_HARDWARE" = true ]; then
        HARDWARE_FLAG="--hardware"
    else
        HARDWARE_FLAG="--simulation"
    fi
    
    echo -e "${YELLOW}Deploying model: ${MODELS_DIR}/${MODEL_NAME}${NC}"
    "${TD3_DIR}/tools/deploy.sh" --model "${MODELS_DIR}/${MODEL_NAME}" ${TIDL_FLAG} ${HARDWARE_FLAG}
    
    echo -e "${GREEN}Deployment complete!${NC}"
}

# Execute the requested mode
case "$MODE" in
    build)
        build_td3learn
        ;;
    train)
        train_model
        ;;
    test)
        test_model
        ;;
    deploy)
        deploy_model
        ;;
esac
