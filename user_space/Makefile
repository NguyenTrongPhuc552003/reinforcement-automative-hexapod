CC = gcc
CFLAGS = -Wall -Wextra -O2 -I./include
LDFLAGS = -lm

TARGETS = servo_test mpu6050_test movement_test

all: $(TARGETS)

servo_test: src/servo_test.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

mpu6050_test: src/mpu6050_test.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

movement_test: src/movement_test.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm -f $(TARGETS)

.PHONY: all clean
