#!/bin/bash
# filepath: /home/strongfood/Projects/reinforcement-automative-hexapod/td3learn/tools/deploy

# Deployment script for TD3Learn

# Default values
MODEL_PATH=""
CONFIG_FILE="configs/default.yaml"
OUTPUT_DIR="deploy"
SIMULATION=true
VERBOSE=false
TIDL_CONVERT=false
LOG_LEVEL="info"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --model)
            MODEL_PATH="$2"
            shift 2
            ;;
        --config)
            CONFIG_FILE="$2"
            shift 2
            ;;
        --output)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --hardware)
            SIMULATION=false
            shift
            ;;
        --tidl)
            TIDL_CONVERT=true
            shift
            ;;
        --verbose)
            VERBOSE=true
            shift
            ;;
        --debug)
            LOG_LEVEL="debug"
            shift
            ;;
        --help)
            echo "Usage: $(basename $0) [options]"
            echo ""
            echo "Options:"
            echo "  --model PATH        Path to trained model"
            echo "  --config FILE       Configuration file (default: configs/default.yaml)"
            echo "  --output DIR        Output directory (default: deploy)"
            echo "  --hardware          Deploy for real hardware (not simulation)"
            echo "  --tidl              Convert to TIDL format for acceleration"
            echo "  --verbose           Show verbose output"
            echo "  --debug             Enable debug logging"
            echo "  --help              Show this help message"
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
if [[ -z "$MODEL_PATH" ]]; then
    echo "Error: Model path is required"
    echo "Use --model PATH to specify the model path"
    exit 1
fi

# Check if model exists
if [[ ! -d "$MODEL_PATH" ]]; then
    echo "Error: Model directory not found: $MODEL_PATH"
    exit 1
fi

# Create output directory if it doesn't exist
mkdir -p "$OUTPUT_DIR"

# Run the deployment
CMD="../build/td3learn_deploy"
CMD+=" --model $MODEL_PATH"
CMD+=" --config $CONFIG_FILE"
CMD+=" --output $OUTPUT_DIR"
CMD+=" --loglevel $LOG_LEVEL"

if [[ "$SIMULATION" == "false" ]]; then
    CMD+=" --hardware"
fi

if [[ "$TIDL_CONVERT" == "true" ]]; then
    CMD+=" --tidl"
fi

if [[ "$VERBOSE" == "true" ]]; then
    CMD+=" --verbose"
fi

# Print command if verbose
if [[ "$VERBOSE" == "true" ]]; then
    echo "Running: $CMD"
fi

# Execute
eval $CMD

if [ $? -eq 0 ]; then
    echo "Model deployed successfully to $OUTPUT_DIR"
    
    if [[ "$TIDL_CONVERT" == "true" ]]; then
        echo "TIDL model available at $OUTPUT_DIR/tidl_model"
    fi
    
    echo "To run the deployed model:"
    if [[ "$SIMULATION" == "true" ]]; then
        echo "  ../build/td3learn_run --model $OUTPUT_DIR --simulation"
    else
        echo "  ../build/td3learn_run --model $OUTPUT_DIR --hardware"
    fi
else
    echo "Deployment failed"
    exit 1
fi