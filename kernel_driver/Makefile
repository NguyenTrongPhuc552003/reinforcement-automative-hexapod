# Define the module
obj-m += src/hexapod_main.o
obj-m += src/i2c_comm.o
obj-m += src/uart_comm.o
obj-m += src/mpu6050.o
obj-m += src/pca9685.o
obj-m += src/servo.o
obj-m += src/gpio_control.o
obj-m += src/pwm_control.o

# Add include directory and floating point support
ccflags-y := -I$(PWD)/include -mfpu=vfp -mfloat-abi=softfp

# Check if we're building inside Docker
ifeq ($(shell test -e /.dockerenv && echo yes),yes)
    # Use the Docker kernel directory
    KDIR ?= /build/kernel/usr/src/linux-headers-4.14.108-ti-r144
else
    # Check if we're building on the BeagleBone
    ifeq ($(shell uname -r),4.14.108-ti-r144)
        # Use the local kernel directory
        KDIR ?= /lib/modules/$(shell uname -r)/build
    else
        $(error Please use the Docker build environment by running ./scripts/build.sh)
    endif
endif

# Current directory
PWD := $(shell pwd)

# Build modules
all:
	@if [ -z "$(INSIDE_DOCKER)" ]; then \
		echo "Please use the Docker build environment by running ./scripts/build.sh"; \
		exit 1; \
	fi
	$(MAKE) -C $(KDIR) M=$(PWD) modules

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

# Clean target
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -f Module.symvers modules.order

# Install target (for development)
install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install