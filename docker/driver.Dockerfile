# Base image for building hexapod kernel modules
FROM debian:buster

# Set noninteractive installation
ENV DEBIAN_FRONTEND=noninteractive

# Global build environment variables - Define KERNEL_VERSION first so it's available to other variables
ENV KERNEL_VERSION=4.14.108-ti-r144
ENV DEBIAN_VERSION=buster \
    GCC_VERSION=8 \
    ARCH=arm \
    CROSS_COMPILE=arm-linux-gnueabihf- \
    KERNEL_DIR=/build/kernel/usr/src/linux-headers-${KERNEL_VERSION} \
    BUILD_TYPE=module \
    INSTALL_DIR=/build/deploy

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

# Create build directories
WORKDIR /build
RUN mkdir -p /build/kernel /build/module /build/deploy

# Download and extract kernel headers for BeagleBone AI
RUN echo "Downloading kernel headers for version ${KERNEL_VERSION}" && \
    wget https://rcn-ee.com/repos/debian/pool/main/l/linux-upstream/linux-headers-${KERNEL_VERSION}_1${DEBIAN_VERSION}_armhf.deb && \
    dpkg -x linux-headers-${KERNEL_VERSION}_1${DEBIAN_VERSION}_armhf.deb /build/kernel && \
    rm linux-headers-${KERNEL_VERSION}_1${DEBIAN_VERSION}_armhf.deb

# Verify kernel headers directory exists
RUN test -d ${KERNEL_DIR} || (echo "Kernel headers directory ${KERNEL_DIR} not found!" && exit 1)

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
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["module"]