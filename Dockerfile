FROM ubuntu:20.04

# 1. Base Tools and Build Dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    wget \
    gcc-arm-linux-gnueabihf \
    bc \
    bison \
    flex \
    libssl-dev \
    libncurses5-dev \
    kmod \
    cpio \
    rsync \
    && rm -rf /var/lib/apt/lists/*

# 2. Directory Structure
WORKDIR /build
RUN mkdir -p /build/kernel /build/module /build/deploy

# 3. Kernel Headers
RUN wget https://rcn-ee.com/repos/debian/pool/main/l/linux-upstream/linux-headers-4.14.108-ti-r144_1buster_armhf.deb && dpkg -x linux-headers-4.14.108-ti-r144_1buster_armhf.deb /build/kernel

# 4. Environment Setup
ENV ARCH=arm
ENV CROSS_COMPILE=arm-linux-gnueabihf-
ENV KERNEL_DIR=/build/kernel/usr/src/linux-headers-4.14.108-ti-r144

# 5. Build host tools
RUN cd ${KERNEL_DIR} && \
    mkdir -p scripts/basic scripts/mod scripts/genksyms && \
    # Build fixdep
    gcc -o scripts/basic/fixdep scripts/basic/fixdep.c && \
    # Build recordmcount
    gcc -o scripts/recordmcount scripts/recordmcount.c && \
    # Build modpost without kernel headers
    echo '#include <stdio.h>\n#include <stdlib.h>\n#include <string.h>\n#include <sys/types.h>\n#include <sys/stat.h>\n#include <unistd.h>\n#include <fcntl.h>\n' > scripts/mod/modpost_local.h && \
    gcc -o scripts/mod/modpost \
        scripts/mod/modpost.c \
        scripts/mod/file2alias.c \
        scripts/mod/sumversion.c \
        -DCONFIG_MODVERSIONS -DCONFIG_MODULE_SRCVERSION_ALL \
        -I scripts/mod \
        -include scripts/mod/modpost_local.h && \
    # Build genksyms
    gcc -o scripts/genksyms/genksyms \
        scripts/genksyms/genksyms.c \
        scripts/genksyms/parse.tab.c \
        scripts/genksyms/lex.lex.c \
        -I scripts/genksyms

# 6. Build Script
COPY docker-entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/docker-entrypoint.sh

ENTRYPOINT ["docker-entrypoint.sh"]
