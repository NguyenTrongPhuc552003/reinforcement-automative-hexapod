# Cross-compiler for BeagleBone
# Check if we're in Docker build environment
ifneq ($(wildcard /build/user_space),)
	# We're in Docker, use the cross-compiler from the container
	CC = arm-linux-gnueabihf-gcc
	CFLAGS = -Wall -Wextra -O2
	CFLAGS +=  -I/build/user_space/include -I/build/common/include
	LDFLAGS = -lm
else
    $(error Please use the Docker build environment by running ./scripts/build.sh)
endif

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

$(BIN_DIR)/test_mpu6050: $(TEST_DIR)/test_mpu6050.c $(OBJS)
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