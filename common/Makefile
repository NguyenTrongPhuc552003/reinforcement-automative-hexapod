# Compiler settings - explicitly set cross compiler
CC = arm-linux-gnueabihf-gcc
AR = arm-linux-gnueabihf-ar
CFLAGS = -Wall -Wextra -I./inc -fPIC -O2
CFLAGS += -march=armv7-a -mfpu=vfp -mfloat-abi=hard
LDFLAGS = -shared

# Library settings
LIB_NAME = hexapod
STATIC_LIB = lib$(LIB_NAME).a
SHARED_LIB = lib$(LIB_NAME).so

# Source files
SRCS = $(wildcard src/*.c)
OBJS = $(SRCS:.c=.o)

# Build targets
.PHONY: all clean install

all: $(STATIC_LIB) $(SHARED_LIB)

$(STATIC_LIB): $(OBJS)
	$(AR) rcs $@ $^

$(SHARED_LIB): $(OBJS)
	$(CC) -shared -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(STATIC_LIB) $(SHARED_LIB)

install: all
	mkdir -p $(DESTDIR)/usr/local/lib
	mkdir -p $(DESTDIR)/usr/local/include
	cp $(STATIC_LIB) $(SHARED_LIB) $(DESTDIR)/usr/local/lib/
	cp inc/*.h $(DESTDIR)/usr/local/include/
