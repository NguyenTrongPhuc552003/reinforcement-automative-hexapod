# Cross-compiler for BeagleBone
CC = arm-linux-gnueabihf-gcc
CFLAGS = -Wall -Wextra -O2 -I./include
LDFLAGS = -lm

# Add include directory and floating point support
EXTRA_CFLAGS := -I$(PWD)/include

TARGETS = servo_test mpu6050_test movement_test
BIN_DIR = bin

all: $(BIN_DIR) $(addprefix $(BIN_DIR)/, $(TARGETS))

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

$(BIN_DIR)/servo_test: src/servo_test.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

$(BIN_DIR)/mpu6050_test: src/mpu6050_test.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

$(BIN_DIR)/movement_test: src/movement_test.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -rf $(BIN_DIR)

.PHONY: all clean