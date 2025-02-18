# Kernel module name
MODULE_NAME := hexapod_main

# Kernel build directory
KERNEL_DIR := /lib/modules/$(shell uname -r)/build

# Object files
obj-m := $(MODULE_NAME).o

# Module sources
SRC := hexapod_main.c hexapod_utils.c

# Default target: build the module
all:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules

# Clean target
clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean

