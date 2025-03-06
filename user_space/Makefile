# Cross-compiler for BeagleBone
CC = arm-linux-gnueabihf-gcc
CFLAGS = -Wall -Wextra -O2 -I./inc
CFLAGS += -I/build/common/inc
CFLAGS += -march=armv7-a -mfpu=vfp -mfloat-abi=hard

# Add library linking with rpath
LDFLAGS = -lm -L/build/deploy

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

# Common source files (exclude main.c)
COMMON_SRCS = $(filter-out $(SRC_DIR)/main.c, $(wildcard $(SRC_DIR)/*.c))
COMMON_OBJS = $(COMMON_SRCS:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)

# Test-specific object files
TEST_OBJS = $(TEST_SRCS:$(TEST_DIR)/%.c=$(OBJ_DIR)/test_%.o)

# Main program object
MAIN_OBJ = $(OBJ_DIR)/main.o

# Main program
MAIN_PROG = $(BIN_DIR)/hexapod_controller

# Main targets
all: dirs $(TEST_BINS) $(MAIN_PROG)

dirs:
	@mkdir -p $(BIN_DIR) $(OBJ_DIR)

# Compile source files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

# Compile test files
$(OBJ_DIR)/test_%.o: $(TEST_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

# Link test executables with only their specific objects
$(BIN_DIR)/test_%: $(OBJ_DIR)/test_%.o $(COMMON_OBJS)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

# Compile main program
$(MAIN_OBJ): $(SRC_DIR)/main.c
	$(CC) $(CFLAGS) -c $< -o $@

# Link main controller program
$(MAIN_PROG): $(MAIN_OBJ) $(COMMON_OBJS)
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

# Install target (optional)
install: all
	install -d $(DESTDIR)/usr/local/bin
	install -m 755 $(MAIN_PROG) $(DESTDIR)/usr/local/bin/
	install -m 755 $(TEST_BINS) $(DESTDIR)/usr/local/bin/

.PHONY: all clean test dirs install