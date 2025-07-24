# Base image for building PyTD3 reinforcement learning module
FROM debian:bullseye

# Set noninteractive installation
ENV DEBIAN_FRONTEND=noninteractive

# Global build environment variables
# ENV KERNEL_VERSION=4.14.108-ti-r144
ENV KERNEL_VERSION=5.10.168-ti-r71 \
    DEBIAN_VERSION=bullseye \
    GCC_VERSION=10 \
    BUILD_TYPE=pytd3 \
    INSTALL_DIR=/build/deploy

# Install essential build tools
RUN apt-get update && apt-get install -y \
    build-essential \
    wget \
    bc \
    cmake \
    g++ \
    && rm -rf /var/lib/apt/lists/*

# Install Python development tools
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-dev \
    python3-venv \
    libopenblas-dev \
    && rm -rf /var/lib/apt/lists/*

# Create build directories
WORKDIR /build
RUN mkdir -p /build/pytd3 /build/deploy

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Copy pytd3 source code
COPY pytd3/ /build/pytd3/

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["pytd3"]
