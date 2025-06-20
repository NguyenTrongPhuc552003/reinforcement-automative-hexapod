# Base image with required build tools
FROM debian:buster

# Set common environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    KERNEL_VERSION=4.14.108-ti-r144 \
    DEBIAN_VERSION=buster \
    GCC_VERSION=8 \
    ARCH=arm \
    CROSS_COMPILE=arm-linux-gnueabihf- \
    KERNEL_DIR=/build/kernel/usr/src/linux-headers-${KERNEL_VERSION}

# Install essential build tools
RUN apt-get update && apt-get install -y \
    build-essential \
    wget \
    bc \
    cmake \
    g++ \
    && rm -rf /var/lib/apt/lists/*

# Install ARM cross-compiler
RUN apt-get update && apt-get install -y \
    gcc-${GCC_VERSION}-arm-linux-gnueabihf \
    g++-${GCC_VERSION}-arm-linux-gnueabihf \
    && rm -rf /var/lib/apt/lists/* && \
    ln -s /usr/bin/arm-linux-gnueabihf-gcc-${GCC_VERSION} /usr/bin/arm-linux-gnueabihf-gcc && \
    ln -s /usr/bin/arm-linux-gnueabihf-g++-${GCC_VERSION} /usr/bin/arm-linux-gnueabihf-g++

# Create common build directories
WORKDIR /build
RUN mkdir -p kernel module deploy user pytd3
