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
hexapod-driver-objs := \
                $(SRC_DIR)/hexapod.o \
                $(SRC_DIR)/mpu6050.o \
                $(SRC_DIR)/pca9685.o \
                $(SRC_DIR)/servo.o

# Define the module
obj-m += hexapod-driver.o

# Current directory
PWD := $(shell pwd)

# Add include directory and build options
EXTRA_CFLAGS := -I/build/common/inc
EXTRA_CFLAGS += -I/build/module/inc
EXTRA_CFLAGS += -DDEBUG
EXTRA_CFLAGS += -mfpu=vfp -mfloat-abi=softfp

# Add ARM libgcc path
EXTRA_LDFLAGS := -L/usr/lib/gcc-cross/arm-linux-gnueabihf/8

# Add dependency handling
KBUILD_EXTRA_SYMBOLS := $(shell find /build/common -name Module.symvers)

# Build modules with verbose output
all:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) KBUILD_EXTRA_SYMBOLS="$(KBUILD_EXTRA_SYMBOLS)" modules
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules_install INSTALL_MOD_PATH=$(PWD)/target

modules:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules

# Clean target
clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean
	rm -f Module.symvers modules.order

# Install target (for development)
install:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules_install