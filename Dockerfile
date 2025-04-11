# =============================================================================
# BASE IMAGE AND ENVIRONMENT
# =============================================================================
FROM debian:buster

# Set noninteractive installation
ENV DEBIAN_FRONTEND=noninteractive

# Global build environment variables
ENV KERNEL_VERSION=4.14.108-ti-r144
ENV DEBIAN_VERSION=buster
ENV GCC_VERSION=8

# =============================================================================
# BASE SYSTEM DEPENDENCIES
# =============================================================================
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
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# KERNEL MODULE DEVELOPMENT
# =============================================================================
# Cross-compilation toolchain for BeagleBone ARM platform
RUN apt-get update && apt-get install -y \
    gcc-${GCC_VERSION}-arm-linux-gnueabihf \
    g++-${GCC_VERSION}-arm-linux-gnueabihf \
    libssl-dev \
    libncurses5-dev \
    && rm -rf /var/lib/apt/lists/*

# Create symlink for cross-compiler
RUN ln -s /usr/bin/arm-linux-gnueabihf-gcc-${GCC_VERSION} /usr/bin/arm-linux-gnueabihf-gcc && \
    ln -s /usr/bin/arm-linux-gnueabihf-g++-${GCC_VERSION} /usr/bin/arm-linux-gnueabihf-g++

# Setup kernel environment
ENV ARCH=arm
ENV CROSS_COMPILE=arm-linux-gnueabihf-
ENV KERNEL_DIR=/build/kernel/usr/src/linux-headers-${KERNEL_VERSION}

# =============================================================================
# DIRECTORY STRUCTURE SETUP
# =============================================================================
WORKDIR /build
RUN mkdir -p \
    /build/kernel \
    /build/module \
    /build/deploy \
    /build/user \
    /build/td3learn

# Download kernel headers for BeagleBone AI
RUN wget https://rcn-ee.com/repos/debian/pool/main/l/linux-upstream/linux-headers-${KERNEL_VERSION}_1${DEBIAN_VERSION}_armhf.deb && \
    dpkg -x linux-headers-${KERNEL_VERSION}_1${DEBIAN_VERSION}_armhf.deb /build/kernel

# =============================================================================
# USER SPACE APPLICATION DEVELOPMENT
# =============================================================================
# Install user-space libraries and tools
RUN apt-get update && apt-get install -y \
    libmicrohttpd-dev \
    # Add more user-space dependencies here
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# TD3LEARN REINFORCEMENT LEARNING ENVIRONMENT
# =============================================================================
# Install ML/AI dependencies
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-dev \
    libopenblas-dev \
    # Add more ML dependencies here
    && rm -rf /var/lib/apt/lists/*

# Install Python packages for ML
#RUN pip3 install --no-cache-dir \
#    numpy \
#    scipy \
#    matplotlib

# =============================================================================
# BUILD TOOLS SETUP
# =============================================================================
# Build kernel tools required for module compilation
RUN cd ${KERNEL_DIR} && \
    mkdir -p scripts/basic scripts/mod scripts/genksyms && \
    gcc -o scripts/basic/fixdep scripts/basic/fixdep.c && \
    gcc -o scripts/recordmcount scripts/recordmcount.c && \
    echo '#include <stdio.h>\n#include <stdlib.h>\n#include <string.h>\n#include <sys/types.h>\n#include <sys/stat.h>\n#include <unistd.h>\n#include <fcntl.h>\n' > scripts/mod/modpost_local.h && \
    gcc -o scripts/mod/modpost \
    scripts/mod/modpost.c \
    scripts/mod/file2alias.c \
    scripts/mod/sumversion.c \
    -DCONFIG_MODVERSIONS -DCONFIG_MODULE_SRCVERSION_ALL \
    -I scripts/mod \
    -include scripts/mod/modpost_local.h && \
    gcc -o scripts/genksyms/genksyms \
    scripts/genksyms/genksyms.c \
    scripts/genksyms/parse.tab.c \
    scripts/genksyms/lex.lex.c \
    -I scripts/genksyms

# =============================================================================
# ENTRYPOINT AND CONFIGURATION
# =============================================================================
# Copy source files and entrypoint script
COPY entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/entrypoint.sh

# Default command for kernel module build
COPY . /build/module
CMD cd /build/module && make

ENTRYPOINT ["entrypoint.sh"]
