# Cross-compiler for BeagleBone
CC = arm-linux-gnueabihf-gcc
CFLAGS = -Wall -Wextra -O2 -I./inc
CFLAGS += -I/build/common/inc
CFLAGS += -march=armv7-a -mfpu=vfp -mfloat-abi=hard

# Add library linking with rpath
LDFLAGS = -lm -L/build/deploy

# Add pthread to LDFLAGS if needed
# LDFLAGS += -pthread

# Directories
SRC_DIR = src
TEST_DIR = test
BIN_DIR = bin
OBJ_DIR = obj

# Source files
SRCS = $(wildcard $(SRC_DIR)/*.c)
OBJS = $(SRCS:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)

# Test files - add "test_" prefix to output binary names
TEST_SRCS = $(wildcard $(TEST_DIR)/*.c)
TEST_BINS = $(TEST_SRCS:$(TEST_DIR)/%.c=$(BIN_DIR)/test_%)

# Main targets
all: dirs $(TEST_BINS)

dirs:
	@mkdir -p $(BIN_DIR) $(OBJ_DIR)

# Compile source files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

# Link test executables with "test_" prefix
$(BIN_DIR)/test_%: $(TEST_DIR)/%.c $(OBJS)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

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