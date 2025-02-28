# Cross-compiler for BeagleBone
CC = arm-linux-gnueabihf-gcc
CFLAGS = -Wall -Wextra -O2 -I./include -I/build/common/include
CFLAGS += -march=armv7-a -mfpu=vfp -mfloat-abi=hard
LDFLAGS = -lm

# Add library linking with rpath
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
OBJS = $(SRCS:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)

# Test files
TEST_SRCS = $(wildcard $(TEST_DIR)/*.c)
TEST_BINS = $(TEST_SRCS:$(TEST_DIR)/%.c=$(BIN_DIR)/%)

# Main targets
all: dirs $(TEST_BINS)

dirs:
	@mkdir -p $(BIN_DIR) $(OBJ_DIR)

# Compile source files with explicit floating-point flags
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

# Link test executables with explicit floating-point flags
$(BIN_DIR)/test_%: $(TEST_DIR)/test_%.c $(OBJS)
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