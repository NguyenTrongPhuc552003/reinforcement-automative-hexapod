#!/bin/bash

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Get the project root directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo -e "${YELLOW}Setting up PyTD3 reinforcement learning environment...${NC}"

# Create virtual environment if it doesn't exist
if [ ! -d "${SCRIPT_DIR}/venv" ]; then
    echo -e "${YELLOW}Creating Python virtual environment...${NC}"
    python3 -m venv "${SCRIPT_DIR}/venv"
else
    echo -e "${GREEN}Virtual environment already exists.${NC}"
fi

# Activate virtual environment
source "${SCRIPT_DIR}/venv/bin/activate"

# Upgrade pip
echo -e "${YELLOW}Upgrading pip...${NC}"
pip install --upgrade pip

# Install dependencies
echo -e "${YELLOW}Installing dependencies...${NC}"
if [ -f "${SCRIPT_DIR}/requirements.txt" ]; then
    pip install -r "${SCRIPT_DIR}/requirements.txt"
else
    echo -e "${YELLOW}requirements.txt not found, installing default dependencies...${NC}"
    pip install numpy torch matplotlib pyyaml
    
    # Create requirements.txt
    cat > "${SCRIPT_DIR}/requirements.txt" << EOF
numpy>=1.16.5
torch>=1.4.0
matplotlib>=3.1.2
pyyaml>=5.3
EOF
    echo -e "${GREEN}Created requirements.txt${NC}"
fi

# Create directory structure if needed
for dir in agent models environment utils bridge config tests; do
    if [ ! -d "${SCRIPT_DIR}/${dir}" ]; then
        echo -e "${YELLOW}Creating ${dir} directory...${NC}"
        mkdir -p "${SCRIPT_DIR}/${dir}"
    fi
done

# Create setup.py if doesn't exist
if [ ! -f "${SCRIPT_DIR}/setup.py" ]; then
    echo -e "${YELLOW}Creating setup.py...${NC}"
    cat > "${SCRIPT_DIR}/setup.py" << EOF
from setuptools import setup, find_packages

setup(
    name="pytd3",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "numpy>=1.16.5",
        "torch>=1.4.0",
        "matplotlib>=3.1.2",
        "pyyaml>=5.3",
    ],
    author="Hexapod Team",
    author_email="hexapod@example.com",
    description="TD3 reinforcement learning for hexapod robot",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)
EOF
    echo -e "${GREEN}Created setup.py${NC}"
fi

# Create __init__.py files
for dir in . agent models environment utils bridge config tests; do
    if [ ! -f "${SCRIPT_DIR}/${dir}/__init__.py" ]; then
        echo -e "${YELLOW}Creating ${dir}/__init__.py...${NC}"
        touch "${SCRIPT_DIR}/${dir}/__init__.py"
    fi
done

# Install package in development mode
echo -e "${YELLOW}Installing pytd3 in development mode...${NC}"
pip install -e "${SCRIPT_DIR}"

echo -e "${GREEN}PyTD3 environment setup complete!${NC}"
echo -e "${YELLOW}Activate the environment with: source ${SCRIPT_DIR}/venv/bin/activate${NC}"