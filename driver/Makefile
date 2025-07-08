#/																			\
 * Hexapod Project - A Reinforcement Learning-based Autonomous Hexapod		\
 * Copyright (C) 2025  Nguyen Trong Phuc									\
 *																			\
 * This program is free software; you can redistribute it and/or modify		\
 * it under the terms of the GNU General Public License as published by		\
 * the Free Software Foundation; either version 2 of the License, or		\
 * (at your option) any later version.										\
 *																			\
 * This program is distributed in the hope that it will be useful,			\
 * but WITHOUT ANY WARRANTY; without even the implied warranty of			\
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the			\
 * GNU General Public License for more details.								\
 *																			\
 * You should have received a copy of the GNU General Public License		\
 * along with this program; if not, write to the Free Software Foundation,	\
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.		\
 */

# Check if we're in Docker build environment
ifneq ($(wildcard /build/module),)
    # We're in Docker, use the kernel headers from the container
    KERNEL_DIR ?= /build/kernel/usr/src/linux-headers-${KERNEL_VERSION}
    INSTALL_MOD_PATH ?= $(PWD)/target
else
    $(error Please use the Docker build environment by running ./scripts/build.sh)
endif

# Define source directory
SRC_DIR := src

# Define objects from source files
hexapod_driver-objs := \
                $(SRC_DIR)/main.o \
                $(SRC_DIR)/mpu6050.o \
                $(SRC_DIR)/pca9685.o \
                $(SRC_DIR)/servo.o \
                $(SRC_DIR)/adxl345.o

# Define the final module
obj-m += hexapod_driver.o

# Current directory
PWD := $(shell pwd)

# Add include directory and build options
EXTRA_CFLAGS += -I$(PWD)/inc
EXTRA_CFLAGS += -DDEBUG
EXTRA_CFLAGS += -march=armv7-a -mfpu=vfp -mfloat-abi=softfp

# Add ARM libgcc path
EXTRA_LDFLAGS := -L/usr/lib/gcc-cross/arm-linux-gnueabihf/${GCC_VERSION}

# Fix circular dependency issue by only including external symbol files
# Look for Module.symvers in other paths but not the current build directory
KBUILD_EXTRA_SYMBOLS := $(shell find /build/module -path "$(PWD)" -prune -o -name Module.symvers -print)

# Build modules
all: modules

modules:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) KBUILD_EXTRA_SYMBOLS="$(KBUILD_EXTRA_SYMBOLS)" modules

# Install modules without using modules_install (which requires System.map)
install: modules
	@mkdir -p $(INSTALL_MOD_PATH)/lib/modules/$(KERNEL_VERSION)/extra
	@cp -f hexapod_driver.ko $(INSTALL_MOD_PATH)/lib/modules/$(KERNEL_VERSION)/extra/
	@echo "Installed module to $(INSTALL_MOD_PATH)/lib/modules/$(KERNEL_VERSION)/extra/"

# Run depmod if available
depmod: install
	@if [ -f "$(KERNEL_DIR)/System.map" ]; then \
		depmod -b $(INSTALL_MOD_PATH) $(KERNEL_VERSION); \
	else \
		echo "System.map not available - skipping depmod (this is expected in cross-compile)"; \
	fi

# Clean target
clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean
	rm -f Module.symvers modules.order
	rm -rf $(INSTALL_MOD_PATH)
	rm -rf $(PWD)/{obj,cmd,deps}