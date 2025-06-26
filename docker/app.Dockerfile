# Base image for building hexapod user space applications
FROM debian:buster

# Set noninteractive installation
ENV DEBIAN_FRONTEND=noninteractive

# Global build environment variables
ENV KERNEL_VERSION=4.14.108-ti-r144 \
    DEBIAN_VERSION=buster \
    GCC_VERSION=8 \
    ARCH=arm \
    CROSS_COMPILE=arm-linux-gnueabihf- \
    BUILD_TYPE=user \
    INSTALL_DIR=/build/deploy

# Install essential build tools for user space applications
RUN apt-get update && apt-get install -y \
    build-essential \
    wget \
    bc \
    cmake \
    g++ \
    && rm -rf /var/lib/apt/lists/*

# Install cross-compilation toolchain for BeagleBone ARM platform
RUN apt-get update && apt-get install -y \
    gcc-${GCC_VERSION}-arm-linux-gnueabihf \
    g++-${GCC_VERSION}-arm-linux-gnueabihf \
    libssl-dev \
    libncurses5-dev \
    && rm -rf /var/lib/apt/lists/*

# Create symlink for cross-compiler
RUN ln -s /usr/bin/arm-linux-gnueabihf-gcc-${GCC_VERSION} /usr/bin/arm-linux-gnueabihf-gcc && \
    ln -s /usr/bin/arm-linux-gnueabihf-g++-${GCC_VERSION} /usr/bin/arm-linux-gnueabihf-g++

# Create build directories
WORKDIR /build
RUN mkdir -p /build/user /build/deploy

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["user"]