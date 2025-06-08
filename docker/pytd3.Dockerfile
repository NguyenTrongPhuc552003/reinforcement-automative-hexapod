# --------------------------------------
# Base Stage - Common Dependencies
# --------------------------------------
FROM python:3.8-slim-buster AS base

# Set noninteractive installation and better Python defaults
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PIP_NO_CACHE_DIR=1 \
    PYTHONPATH=/app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    libopenblas-dev \
    libblas-dev \
    liblapack-dev \
    libatlas-base-dev \
    gfortran \
    git \
    wget \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Create application directory structure
WORKDIR /app
RUN mkdir -p /app/pytd3 /app/models /app/data /app/configs

# --------------------------------------
# Development Stage - For Interactive Development
# --------------------------------------
FROM base AS dev

# Install development tools
RUN apt-get update && apt-get install -y \
    vim \
    less \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements and install dependencies
COPY pytd3/requirements.txt /app/pytd3/
RUN pip install --upgrade pip && \
    pip install -r /app/pytd3/requirements.txt && \
    pip install jupyter matplotlib pytest ipython

# Install any additional development packages for interactive testing
RUN pip install gym[box2d] pybullet

# Set up Jupyter
EXPOSE 8888
ENV JUPYTER_PORT=8888

# Default command for development environment
CMD ["bash"]

# --------------------------------------
# Training Stage - For Model Training
# --------------------------------------
FROM base AS train

# Copy requirements and install dependencies
COPY pytd3/requirements.txt /app/pytd3/
RUN pip install --upgrade pip && \
    pip install -r /app/pytd3/requirements.txt

# Create helper scripts directory
RUN mkdir -p /app/scripts

# Add helper scripts for common training operations
RUN echo '#!/bin/bash\npython /app/pytd3/train.py "$@"' > /app/scripts/train.sh && \
    echo '#!/bin/bash\npython /app/pytd3/evaluate.py "$@"' > /app/scripts/evaluate.sh && \
    echo '#!/bin/bash\npython /app/pytd3/visualize.py "$@"' > /app/scripts/visualize.sh && \
    chmod +x /app/scripts/*.sh

# Add scripts to PATH
ENV PATH="/app/scripts:${PATH}"

# Copy application code
COPY pytd3 /app/pytd3/

# Default command for training
CMD ["python", "/app/pytd3/train.py"]

# --------------------------------------
# Deployment Stage - For Inference
# --------------------------------------
FROM base AS deploy

# Use slim installation for deployment - only what's needed
COPY pytd3/requirements.txt /app/pytd3/
RUN pip install --upgrade pip && \
    grep -v "dev\|test" /app/pytd3/requirements.txt | pip install -r /dev/stdin

# Copy only the necessary code for inference
COPY pytd3/pytd3 /app/pytd3/pytd3/
COPY pytd3/*.py /app/pytd3/

# Environment variables for deployment
ENV TIDL_PLATFORM=AM5729 \
    OCL_PLATFORM=0 \
    DEPLOYMENT_MODE=1

# Default command for deployment
CMD ["python", "/app/pytd3/deploy.py"]
