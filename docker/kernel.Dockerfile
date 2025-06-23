# Base image for building hexapod kernel and builtin drivers
FROM debian:buster

# Set noninteractive installation
ENV DEBIAN_FRONTEND=noninteractive

# Global build environment variables
ENV KERNEL_VERSION=4.14.108-ti-r144 \
    DEBIAN_VERSION=buster \
    GCC_VERSION=8 \
    ARCH=arm \
    CROSS_COMPILE=arm-linux-gnueabihf- \
    BUILD_TYPE=kernel \
    INSTALL_DIR=/build/deploy \
    # Set a default terminal
    TERM=xterm-256color

# Install essential build tools for kernel compilation
RUN apt-get update && apt-get install -y \
    build-essential \
    wget \
    bc \
    bison \
    flex \
    kmod \
    cpio \
    rsync \
    cmake \
    g++ \
    git \
    fakeroot \
    lsb-release \
    libssl-dev \
    libncurses5-dev \
    # Additional dependencies for kernel build
    lz4 \
    man-db \
    gettext \
    pkg-config \
    libmpc-dev \
    u-boot-tools \
    zstd \
    ccache \
    xz-utils \
    # Terminal support packages for menuconfig
    ncurses-base \
    ncurses-term \
    && rm -rf /var/lib/apt/lists/*

# Install cross-compilation toolchain for BeagleBone ARM platform
RUN apt-get update && apt-get install -y \
    gcc-${GCC_VERSION}-arm-linux-gnueabihf \
    g++-${GCC_VERSION}-arm-linux-gnueabihf \
    && rm -rf /var/lib/apt/lists/*

# Create symlink for cross-compiler
RUN ln -s /usr/bin/arm-linux-gnueabihf-gcc-${GCC_VERSION} /usr/bin/arm-linux-gnueabihf-gcc && \
    ln -s /usr/bin/arm-linux-gnueabihf-g++-${GCC_VERSION} /usr/bin/arm-linux-gnueabihf-g++

# Create build directories
WORKDIR /build
RUN mkdir -p /build/kernel /build/deploy

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Copy utils for kernel build
COPY utils/ti-linux-kernel-dev/ /build/kernel/

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["kernel"]
