# Base image with required build tools
FROM debian:buster

# Set noninteractive installation
ENV DEBIAN_FRONTEND=noninteractive

# Global build environment variables
ENV KERNEL_VERSION=4.14.108-ti-r144
ENV DEBIAN_VERSION=buster
ENV GCC_VERSION=8

# Install essential build tools
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

# Setup kernel environment
ENV ARCH=arm
ENV CROSS_COMPILE=arm-linux-gnueabihf-
ENV KERNEL_DIR=/build/kernel/usr/src/linux-headers-${KERNEL_VERSION}

# Create build directories
WORKDIR /build
RUN mkdir -p \
    /build/kernel \
    /build/module \
    /build/deploy \
    /build/user \
    /build/pytd3

# Download and extract kernel headers for BeagleBone AI
RUN wget https://rcn-ee.com/repos/debian/pool/main/l/linux-upstream/linux-headers-${KERNEL_VERSION}_1${DEBIAN_VERSION}_armhf.deb && \
    dpkg -x linux-headers-${KERNEL_VERSION}_1${DEBIAN_VERSION}_armhf.deb /build/kernel && \
    rm linux-headers-${KERNEL_VERSION}_1${DEBIAN_VERSION}_armhf.deb

# Install user-space libraries and Python for PyTD3
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-dev \
    python3-venv \
    libopenblas-dev \
    && rm -rf /var/lib/apt/lists/*

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

# Copy entrypoint script
COPY entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

# Default command (will be overridden by docker-compose or command line)
CMD ["module"]
