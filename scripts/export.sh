#!/bin/bash

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Directories
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DIAGRAM_DIR="${SCRIPT_DIR}/../docs/diagrams"
SRC_DIR="${DIAGRAM_DIR}/src"
OUT_DIR="${DIAGRAM_DIR}/out"

# Output format - can be png, svg, pdf
FORMAT="png"

# Parse command line arguments
CLEAN_MODE=0

# Check if clean option is specified
if [ "$#" -gt 0 ]; then
    case "$1" in
        -c|clean)
            CLEAN_MODE=1
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            echo "Usage: $0 [-c|clean]"
            exit 1
            ;;
    esac
fi

# Handle clean mode
if [ $CLEAN_MODE -eq 1 ]; then
    echo -e "${YELLOW}Cleaning generated diagram files...${NC}"
    
    if [ -d "${OUT_DIR}" ]; then
        rm -f "${OUT_DIR}"/*.${FORMAT} 2>/dev/null
        echo -e "${GREEN}Removed all .${FORMAT} files from ${OUT_DIR}${NC}"
    else
        echo -e "${YELLOW}Output directory does not exist: ${OUT_DIR}${NC}"
    fi
    exit 0
fi

# Using PlantUML jar file
PLANTUML_JAR="${DIAGRAM_DIR}/plantuml-1.2025.2.jar"
if [ ! -f "${PLANTUML_JAR}" ]; then
    echo -e "${RED}PlantUML jar file not found! Please ensure it is in the script directory.${NC}"
    exit 1
fi

# Create output directory if it doesn't exist
if [ ! -d "${OUT_DIR}" ]; then
    echo -e "${YELLOW}Output directory does not exist. Creating: ${OUT_DIR}${NC}"
    mkdir -p "${OUT_DIR}"
fi

# Track counts
TOTAL=0
SUCCESS=0
FAILED=0

echo -e "${YELLOW}Starting PlantUML diagram generation...${NC}"
echo "Source directory: ${SRC_DIR}"
echo "Output directory: ${OUT_DIR}"
echo "Format: ${FORMAT}"

# Process each PUML file
for PUML_FILE in "${SRC_DIR}"/*.puml; do
    if [ -f "$PUML_FILE" ]; then
        TOTAL=$((TOTAL + 1))
        BASE_NAME=$(basename "$PUML_FILE" .puml)
        OUTPUT_FILE="${OUT_DIR}/${BASE_NAME}.${FORMAT}"
        
        echo -e "\nProcessing: ${YELLOW}${BASE_NAME}${NC}"
        
        # Run PlantUML with the specified format from the jar file
        java -jar "${PLANTUML_JAR}" "${PUML_FILE}" -t${FORMAT} -o "${OUT_DIR}"
        
        # Check if the command was successful and the output file was created
        if [ $? -eq 0 ]; then
            SUCCESS=$((SUCCESS + 1))
            echo -e "  ${GREEN}✓ Successfully generated: ${OUTPUT_FILE}${NC}"
        else
            FAILED=$((FAILED + 1))
            echo -e "  ${RED}✗ Failed to generate: ${OUTPUT_FILE}${NC}"
        fi
    fi
done

# Print summary
echo -e "\n${YELLOW}Diagram Generation Summary:${NC}"
echo -e "  ${GREEN}Total: ${TOTAL}${NC}"
echo -e "  ${GREEN}Successful: ${SUCCESS}${NC}"
if [ $FAILED -gt 0 ]; then
    echo -e "  ${RED}Failed: ${FAILED}${NC}"
fi

# Exit with appropriate status code
if [ $FAILED -gt 0 ]; then
    echo -e "\n${RED}Some diagrams failed to generate.${NC}"
    exit 1
else
    echo -e "\n${GREEN}All diagrams successfully generated!${NC}"
    exit 0
fi
