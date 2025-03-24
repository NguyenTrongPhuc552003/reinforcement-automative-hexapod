# Set the base image to Debian Bookworm
FROM debian:buster

# Set noninteractive installation
ENV DEBIAN_FRONTEND=noninteractive

# Set kernel header version
ENV KERNEL_VERSION=4.14.108-ti-r144

# Set Debian version
ENV DEBIAN_VERSION=buster

# Set GCC version
ENV GCC_VERSION=8

# 1. Base Tools and Build Dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    wget \
    bc \
    bison \
    flex \
    libssl-dev \
    libncurses5-dev \
    kmod \
    cpio \
    rsync \
    libmicrohttpd-dev \
    && rm -rf /var/lib/apt/lists/*

# 2. Install ARM cross-compiler toolchain for BeagleBone (GLIBC 2.28 compatible)
RUN apt-get update && apt-get install -y \
    gcc-${GCC_VERSION}-arm-linux-gnueabihf \
    g++-${GCC_VERSION}-arm-linux-gnueabihf \
    && rm -rf /var/lib/apt/lists/*

# Create symlink for cross-compiler
RUN ln -s /usr/bin/arm-linux-gnueabihf-gcc-${GCC_VERSION} /usr/bin/arm-linux-gnueabihf-gcc
RUN ln -s /usr/bin/arm-linux-gnueabihf-g++-${GCC_VERSION} /usr/bin/arm-linux-gnueabihf-g++

# 3. Directory Structure
WORKDIR /build
RUN mkdir -p /build/kernel /build/module /build/deploy /build/user

# 4. Kernel Headers for BeagleBone AI
RUN wget https://rcn-ee.com/repos/debian/pool/main/l/linux-upstream/linux-headers-${KERNEL_VERSION}_1${DEBIAN_VERSION}_armhf.deb && dpkg -x linux-headers-${KERNEL_VERSION}_1${DEBIAN_VERSION}_armhf.deb /build/kernel

# 5. Environment Setup
ENV ARCH=arm
ENV CROSS_COMPILE=arm-linux-gnueabihf-
ENV KERNEL_DIR=/build/kernel/usr/src/linux-headers-${KERNEL_VERSION}

# 6. Build host tools
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

# 7. Build Script
COPY entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/entrypoint.sh

# 8. Copy source files
COPY . /build/module/

# Default command for kernel module build
CMD cd /build/module && make

ENTRYPOINT ["entrypoint.sh"]
