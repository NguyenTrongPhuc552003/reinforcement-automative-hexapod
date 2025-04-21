#!/bin/bash

# TD3Learn model deployment script for BeagleBone AI
# This script handles optimized deployment with TIDL and OpenCL acceleration

# Configuration with defaults
BEAGLEBONE_IP="beaglebone.local"
BEAGLEBONE_USER="debian"
TARGET_PATH="/home/debian/td3learn_model"
MODEL_PATH=""
IS_DOCKER=false
USE_TIDL=false
USE_OPENCL=false
USE_HARDWARE=true

# Parse command-line arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --model)
            if [ -z "$2" ] || [[ "$2" == --* ]]; then
                echo "Error: --model requires a directory path"
                exit 1
            fi
            MODEL_PATH="$2"
            shift 2
            ;;
        --host)
            if [ -z "$2" ] || [[ "$2" == --* ]]; then
                echo "Error: --host requires an address"
                exit 1
            fi
            BEAGLEBONE_IP="$2"
            shift 2
            ;;
        --user)
            if [ -z "$2" ] || [[ "$2" == --* ]]; then
                echo "Error: --user requires a username"
                exit 1
            fi
            BEAGLEBONE_USER="$2"
            shift 2
            ;;
        --output)
            if [ -z "$2" ] || [[ "$2" == --* ]]; then
                echo "Error: --output requires a path"
                exit 1
            fi
            TARGET_PATH="$2"
            shift 2
            ;;
        --tidl)
            USE_TIDL=true
            shift
            ;;
        --opencl)
            USE_OPENCL=true
            shift
            ;;
        --no-tidl)
            USE_TIDL=false
            shift
            ;;
        --simulation)
            USE_HARDWARE=false
            shift
            ;;
        --docker)
            IS_DOCKER=true
            shift
            ;;
        --help)
            echo "Usage: $(basename $0) [options]"
            echo ""
            echo "Options:"
            echo "  --model PATH       Model directory path (required)"
            echo "  --host HOST        BeagleBone hostname or IP (default: beaglebone.local)"
            echo "  --user USER        SSH username (default: debian)"
            echo "  --output PATH      Target path on BeagleBone (default: /home/debian/td3learn_model)"
            echo "  --tidl             Use TIDL acceleration"
            echo "  --opencl           Use OpenCL acceleration"
            echo "  --no-tidl          Don't use TIDL acceleration"
            echo "  --simulation       Use simulation mode instead of hardware"
            echo "  --docker           Use Docker for deployment (for CI/CD pipelines)"
            echo "  --help             Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Check if model path is provided
if [ -z "$MODEL_PATH" ]; then
    echo "Error: --model option is required"
    echo "Use --help for usage information"
    exit 1
fi

# Check if model directory exists
if [ ! -d "$MODEL_PATH" ]; then
    echo "Error: Model directory does not exist: $MODEL_PATH"
    exit 1
fi

# Create temporary working directory
TMP_DIR=$(mktemp -d)
trap "rm -rf $TMP_DIR" EXIT

echo "Preparing model for deployment..."
mkdir -p "$TMP_DIR/model"
cp -r "$MODEL_PATH"/* "$TMP_DIR/model/"

# Create run script
echo "Creating run script..."
cat > "$TMP_DIR/run.sh" << EOL
#!/bin/bash

# TD3Learn model runner script
MODEL_DIR="\$(dirname "\$0")"

# Run the model with appropriate options
"\$MODEL_DIR/td3learn_run" \\
    --model "\$MODEL_DIR/model" \\
EOL

# Add hardware acceleration options to run script
if [ "$USE_TIDL" = true ]; then
    echo "    --tidl \\" >> "$TMP_DIR/run.sh"
fi

if [ "$USE_OPENCL" = true ]; then
    echo "    --opencl \\" >> "$TMP_DIR/run.sh"
fi

# Add hardware/simulation mode to run script
if [ "$USE_HARDWARE" = true ]; then
    echo "    --hardware" >> "$TMP_DIR/run.sh"
else
    echo "    --simulation" >> "$TMP_DIR/run.sh"
fi

# Make run script executable
chmod +x "$TMP_DIR/run.sh"

# Copy hardware detection script
cp "$(dirname "$0")/hwdetect.sh" "$TMP_DIR/hwdetect.sh"
chmod +x "$TMP_DIR/hwdetect.sh"

# Copy runtime executable if it exists
if [[ -f "$(dirname "$MODEL_PATH")/../build/td3learn_run" ]]; then
    echo "Copying runtime executable..."
    cp "$(dirname "$MODEL_PATH")/../build/td3learn_run" "$TMP_DIR/"
fi

# Deploy to BeagleBone
if [[ "$IS_DOCKER" == "true" ]]; then
    echo "Docker deployment mode - skipping SSH connection check"
else
    echo "Testing SSH connection to BeagleBone..."
    if ! ssh -q ${BEAGLEBONE_USER}@${BEAGLEBONE_IP} exit; then
        echo "Cannot connect to BeagleBone at ${BEAGLEBONE_IP}. Please check your connection."
        exit 1
    fi
    
    # Create target directory on BeagleBone
    echo "Creating target directory on BeagleBone..."
    ssh ${BEAGLEBONE_USER}@${BEAGLEBONE_IP} "mkdir -p ${TARGET_PATH}"
    
    # Copy files to BeagleBone
    echo "Copying files to BeagleBone..."
    scp -r $TMP_DIR/* ${BEAGLEBONE_USER}@${BEAGLEBONE_IP}:${TARGET_PATH}/
    
    echo "Deployment complete!"
    echo "To run the model on the BeagleBone:"
    echo "  1. SSH to the BeagleBone: ssh ${BEAGLEBONE_USER}@${BEAGLEBONE_IP}"
    echo "  2. Navigate to: cd ${TARGET_PATH}"
    echo "  3. Run: ./run.sh"
fi

# If in Docker mode, just copy files to output directory
if [[ "$IS_DOCKER" == "true" ]]; then
    echo "Copying files to Docker output directory..."
    mkdir -p "/output"
    cp -r $TMP_DIR/* /output/
    echo "Files ready in Docker output directory"
fi

exit 0
