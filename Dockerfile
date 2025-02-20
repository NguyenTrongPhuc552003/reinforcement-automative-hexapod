FROM ubuntu:18.04

# Set noninteractive installation
ENV DEBIAN_FRONTEND=noninteractive

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
    && rm -rf /var/lib/apt/lists/*

# 2. Install ARM cross-compiler toolchain for BeagleBone (GLIBC 2.28 compatible)
RUN apt-get update && apt-get install -y \
    gcc-8-arm-linux-gnueabihf \
    g++-8-arm-linux-gnueabihf \
    && rm -rf /var/lib/apt/lists/*

# Create symlink for cross-compiler
RUN ln -s /usr/bin/arm-linux-gnueabihf-gcc-8 /usr/bin/arm-linux-gnueabihf-gcc

# 3. Directory Structure
WORKDIR /build
RUN mkdir -p /build/kernel /build/module /build/deploy /build/user_space

# 4. Kernel Headers
RUN wget https://rcn-ee.com/repos/debian/pool/main/l/linux-upstream/linux-headers-4.14.108-ti-r144_1buster_armhf.deb && dpkg -x linux-headers-4.14.108-ti-r144_1buster_armhf.deb /build/kernel

# 5. Environment Setup
ENV ARCH=arm
ENV CROSS_COMPILE=arm-linux-gnueabihf-
ENV KERNEL_DIR=/build/kernel/usr/src/linux-headers-4.14.108-ti-r144

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

# 6. Build Script
COPY docker-entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/docker-entrypoint.sh

# Copy source files
COPY . /build/module/

# Default command for kernel module build
CMD cd /build/module && make

ENTRYPOINT ["docker-entrypoint.sh"]
