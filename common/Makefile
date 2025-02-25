# Compiler settings - explicitly set cross compiler
CC = arm-linux-gnueabihf-gcc
AR = arm-linux-gnueabihf-ar
CFLAGS = -Wall -Wextra -I./include -fPIC
CFLAGS += -march=armv7-a
LDFLAGS = -shared

# Library settings
LIB_NAME = hexapod
STATIC_LIB = lib$(LIB_NAME).a
SHARED_LIB = lib$(LIB_NAME).so

# Source files
SRCS = $(wildcard src/*.c)
OBJS = $(SRCS:.c=.o)
DEPS = $(SRCS:.c=.d)

# Build targets
.PHONY: all clean install

all: $(STATIC_LIB) $(SHARED_LIB)

$(STATIC_LIB): $(OBJS)
	$(AR) $(ARFLAGS) $@ $^

$(SHARED_LIB): $(OBJS)
	$(CC) -shared -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

-include $(DEPS)

clean:
	rm -f $(OBJS) $(DEPS) $(STATIC_LIB) $(SHARED_LIB)

install: all
	mkdir -p $(DESTDIR)/usr/local/lib
	mkdir -p $(DESTDIR)/usr/local/include/hexapod
	cp $(STATIC_LIB) $(SHARED_LIB) $(DESTDIR)/usr/local/lib/
	cp include/*.h $(DESTDIR)/usr/local/include/hexapod/
