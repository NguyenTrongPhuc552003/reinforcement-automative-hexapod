# Cross-compiler for BeagleBone
CXX = arm-linux-gnueabihf-g++
CXXFLAGS = -Wall -Wextra -O2 -std=c++17 -I./inc -fPIC
CXXFLAGS += -march=armv7-a -mfpu=vfp -mfloat-abi=hard
CXXFLAGS += -pipe -fno-rtti

# Add additional optimization flags
RELEASE_FLAGS = -DNDEBUG -O3 -flto -ffast-math -fomit-frame-pointer
# Reduce debug flags - remove overly verbose options
DEBUG_FLAGS = -g -DDEBUG -O0

# Choose between release and debug
ifdef DEBUG
    CXXFLAGS += $(DEBUG_FLAGS)
else
    CXXFLAGS += $(RELEASE_FLAGS)
endif

# Add library linking with C++17 filesystem support
LDFLAGS = -lm -L/build/deploy -lgpiodcxx -lstdc++fs
LDFLAGS += -Wl,-O1 -Wl,--as-needed

# Directories for source, test, binary, object, and dependency files
SRC_DIR = src
TEST_DIR = test
BIN_DIR = bin
OBJ_DIR = obj
DEP_DIR = deps

# Source files - all .cpp files in src directory
SRCS = $(wildcard $(SRC_DIR)/*.cpp)
OBJS = $(SRCS:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)
DEPS = $(SRCS:$(SRC_DIR)/%.cpp=$(DEP_DIR)/%.d)

# Test files - add "test_" prefix to output binary names
TEST_SRCS = $(wildcard $(TEST_DIR)/*.cpp)
TEST_BINS = $(TEST_SRCS:$(TEST_DIR)/%.cpp=$(BIN_DIR)/test_%)
TEST_OBJS = $(TEST_SRCS:$(TEST_DIR)/%.cpp=$(OBJ_DIR)/test_%.o)
TEST_DEPS = $(TEST_SRCS:$(TEST_DIR)/%.cpp=$(DEP_DIR)/test_%.d)

# Common source files (exclude main.cpp)
COMMON_SRCS = $(filter-out $(SRC_DIR)/main.cpp, $(wildcard $(SRC_DIR)/*.cpp))
COMMON_OBJS = $(COMMON_SRCS:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)

# Main program object and binary
MAIN_OBJ = $(OBJ_DIR)/main.o
MAIN_PROG = $(BIN_DIR)/hexapod_controller

# All dependency files
ALL_DEPS = $(DEPS) $(TEST_DEPS)

# Main targets
all: dirs $(TEST_BINS) $(MAIN_PROG)

dirs:
	@mkdir -p $(BIN_DIR) $(OBJ_DIR) $(DEP_DIR)

# Include dependency files
-include $(ALL_DEPS)

# Compile source files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -MMD -MP -MF $(DEP_DIR)/$*.d -c $< -o $@

# Compile test files
$(OBJ_DIR)/test_%.o: $(TEST_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -MMD -MP -MF $(DEP_DIR)/test_$*.d -c $< -o $@

# Link test executables
$(BIN_DIR)/test_%: $(OBJ_DIR)/test_%.o $(COMMON_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

# Link main controller program
$(MAIN_PROG): $(MAIN_OBJ) $(COMMON_OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LDFLAGS)

# Run tests
test: all
	@echo "Running tests..."
	@for test in $(TEST_BINS); do \
		echo "\nRunning $$test:"; \
		$$test; \
	done

clean:
	rm -rf $(BIN_DIR) $(OBJ_DIR) $(DEP_DIR)

# Install target (optional)
install: all
	install -d $(DESTDIR)/usr/local/bin
	install -m 755 $(MAIN_PROG) $(DESTDIR)/usr/local/bin/
	install -m 755 $(TEST_BINS) $(DESTDIR)/usr/local/bin/

.PHONY: all clean test dirs install
