# Check if we're in Docker build environment
ifneq ($(wildcard /build/module),)
    # We're in Docker, use the kernel headers from the container
    KERNEL_DIR ?= /build/kernel/usr/src/linux-headers-4.14.108-ti-r144
else
    $(error Please use the Docker build environment by running ./scripts/build.sh)
endif

# Define source directory
SRC_DIR := src

# Define objects from source files
hexapod_driver-objs := \
                $(SRC_DIR)/hexapod.o \
                $(SRC_DIR)/mpu6050.o \
                $(SRC_DIR)/pca9685.o \
                $(SRC_DIR)/servo.o

# Define the module
obj-m += hexapod_driver.o

# Current directory
PWD := $(shell pwd)

# Add include directory and floating point support
EXTRA_CFLAGS := -I/build/common/include
EXTRA_CFLAGS += -I/build/module/include

# Build modules
all:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules

modules:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules

# Clean target
clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean
	rm -f Module.symvers modules.order

# Install target (for development)
install:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules_install