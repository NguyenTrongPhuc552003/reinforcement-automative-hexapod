# Define the module name
obj-m := hexapod_main.o

# Define the source files
hexapod_main-y := src/gpio_control.o \
                  src/servo.o \
                  src/pca9685.o \
                  src/i2c_comm.o \
                  src/uart_comm.o \
                  src/pwm_control.o \
                  src/mpu6050.o \
                  src/hexapod_main.o

# Add include directory
ccflags-y := -I$(PWD)/include

# Check if we're building inside Docker
ifeq ($(shell test -e /.dockerenv && echo yes),yes)
    # Inside Docker: Use the kernel source we downloaded
    KERNEL_DIR ?= /build/kernel
else
    # Outside Docker: Don't try to build directly
    $(error Please use the Docker build environment by running ./scripts/build.sh)
endif

# Architecture and cross-compiler settings
ARCH ?= arm
CROSS_COMPILE ?= arm-linux-gnueabihf-

# Ensure the make is available
MAKE := /usr/bin/make

# Current directory
PWD := $(shell pwd)

# Default target
all:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules

# Clean target
clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean

# Install target (for development)
install:
	$(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL_DIR) M=$(PWD) modules_install
	depmod -a

.PHONY: all clean install