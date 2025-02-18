CC = gcc
CFLAGS = -Wall -Wextra -O2
TARGET = servo_test

all: $(TARGET)

$(TARGET): src/servo_test.c
	$(CC) $(CFLAGS) -o $@ $<

clean:
	rm -f $(TARGET)

.PHONY: all clean
