# Cross-compiler for BeagleBone
CC = arm-linux-gnueabihf-gcc
CFLAGS = -Wall -Wextra -O2 -I./inc
CFLAGS += -I/build/common/inc
CFLAGS += -march=armv7-a -mfpu=vfp -mfloat-abi=hard

# Add library linking with rpath
LDFLAGS = -lm
LDFLAGS += -L/build/deploy -Wl,-rpath=/usr/local/lib -lhexapod

# Add pthread to LDFLAGS
LDFLAGS += -pthread

# Source files
SRC_DIR = src
TEST_DIR = test
BIN_DIR = bin
OBJ_DIR = obj

# Source files
SRCS = $(wildcard $(SRC_DIR)/*.c)
OBJS = $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SRCS))

# Test files - get the base name without extension
TEST_SRCS = $(wildcard $(TEST_DIR)/*.c)
TEST_BASES = $(basename $(notdir $(TEST_SRCS)))
TEST_BINS = $(addprefix $(BIN_DIR)/test_,$(TEST_BASES))

# Main targets
all: dirs $(TEST_BINS)

dirs:
	@mkdir -p $(BIN_DIR) $(OBJ_DIR)

# Compile source files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

# Link test executables with "test_" prefix
$(BIN_DIR)/test_%: $(TEST_DIR)/%.c $(OBJS)
	$(CC) $(CFLAGS) $< $(OBJS) -o $@ $(LDFLAGS)

# Run tests
test: all
	@echo "Running tests..."
	@for test in $(TEST_BINS); do \
		echo "\nRunning $$test:"; \
		$$test; \
	done

clean:
	rm -rf $(BIN_DIR) $(OBJ_DIR)

.PHONY: all clean test dirs