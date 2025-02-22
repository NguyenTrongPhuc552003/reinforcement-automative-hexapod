# Cross-compiler for BeagleBone
CC = arm-linux-gnueabihf-gcc
CFLAGS = -Wall -Wextra -O2 -I./include
LDFLAGS = -lm

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

# Compile source files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

# Link test executables
$(BIN_DIR)/test_kinematics: $(TEST_DIR)/test_kinematics.c $(OBJS)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

$(BIN_DIR)/test_gait: $(TEST_DIR)/test_gait.c $(OBJS)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

$(BIN_DIR)/test_hexapod: $(TEST_DIR)/test_hexapod.c $(OBJS)
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