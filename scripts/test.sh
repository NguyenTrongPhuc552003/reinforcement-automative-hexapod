#!/bin/bash

# Configuration 
BEAGLEBONE_IP="beaglebone.local"
BEAGLEBONE_USER="debian"
TEST_TIMEOUT=60
LOG_DIR="./test_logs"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Remote deploy directory  
REMOTE_DIR="~/hexapod_driver"
REMOTE_PYTD3_DIR="${REMOTE_DIR}/pytd3"

# Function to show usage
usage() {
    echo "Hexapod Test Script"
    echo "Usage: $0 <command>"
    echo "Commands:"
    echo "  test-driver          Test the kernel driver functionality"
    echo "  test-app            Test the user space application" 
    echo "  test-pytd3         Test PyTD3 reinforcement learning"
    echo "  test-calibration   Test servo calibration"
    echo "  test-balance       Test balance mode"
    echo "  test-all           Run all tests"
    echo "  status             Check system status"
    echo "  restart            Restart the hexapod services" 
    echo "  custom \"COMMAND\"   Run a custom command in quotes"
}

# Function to execute commands remotely
remote_exec() {
    ssh ${BEAGLEBONE_USER}@${BEAGLEBONE_IP} "$1"
}

# Create log directory if doesn't exist
mkdir -p "$LOG_DIR"
LOG_FILE="${LOG_DIR}/test_$(date +%Y%m%d_%H%M%S).log"

# Parse command line arguments
COMMAND="$1"
shift

case "$COMMAND" in
    test-driver)
        echo -e "${GREEN}Testing hexapod kernel driver...${NC}"
        echo "Test starting at $(date)" > "$LOG_FILE"

        # Test driver installation
        remote_exec "cd $REMOTE_DIR && sudo ./install.sh"
        if [ $? -ne 0 ]; then
            echo -e "${RED}Driver installation failed${NC}"
            exit 1
        fi

        # Test basic I/O operations
        remote_exec "cd $REMOTE_DIR && sudo ./test_servo"
        if [ $? -ne 0 ]; then
            echo -e "${RED}Basic I/O tests failed${NC}"
            exit 1  
        fi

        echo -e "${GREEN}Driver tests completed successfully${NC}"
        exit 0
        ;;

    test-app)
        echo -e "${GREEN}Testing hexapod user space application...${NC}"
        echo "Test starting at $(date)" > "$LOG_FILE"

        # Test app with basic movements
        remote_exec "cd $REMOTE_DIR && ./hexapod_controller"
        if [ $? -ne 0 ]; then
            echo -e "${RED}Application tests failed${NC}"
            exit 1
        fi

        echo -e "${GREEN}Application tests completed successfully${NC}"
        exit 0
        ;;

    test-pytd3)
        echo -e "${GREEN}Testing PyTD3 reinforcement learning module...${NC}"
        echo "Test starting at $(date)" > "$LOG_FILE"

        # Activate virtual environment and run tests
        remote_exec "cd $REMOTE_PYTD3_DIR && source venv/bin/activate && \
                    python -m pytest tests/ -v"
        if [ $? -ne 0 ]; then
            echo -e "${RED}PyTD3 tests failed${NC}"
            exit 1
        fi

        # Test model loading
        remote_exec "cd $REMOTE_PYTD3_DIR && source venv/bin/activate && \
                    python main.py --test-models"
        if [ $? -ne 0 ]; then
            echo -e "${RED}Model loading test failed${NC}"
            exit 1
        fi

        echo -e "${GREEN}PyTD3 tests completed successfully${NC}"
        exit 0
        ;;

    test-calibration)
        echo -e "${GREEN}Testing calibration system...${NC}"
        echo "Test starting at $(date)" > "$LOG_FILE"
        
        remote_exec "cd $REMOTE_DIR && ./test_calibration"
        if [ $? -ne 0 ]; then
            echo -e "${RED}Calibration tests failed${NC}"
            exit 1
        fi

        echo -e "${GREEN}Calibration tests completed successfully${NC}"
        exit 0
        ;;

    test-balance)
        echo -e "${GREEN}Testing balance mode...${NC}"
        echo "Test starting at $(date)" > "$LOG_FILE"
        
        # Test IMU readings
        remote_exec "cd $REMOTE_DIR && ./test_mpu6050"
        if [ $? -ne 0 ]; then
            echo -e "${RED}IMU test failed${NC}"
            exit 1
        fi

        # Test balance controller
        remote_exec "cd $REMOTE_DIR && ./test_balance"
        if [ $? -ne 0 ]; then
            echo -e "${RED}Balance controller test failed${NC}"
            exit 1
        fi

        echo -e "${GREEN}Balance tests completed successfully${NC}"
        exit 0
        ;;

    test-all)
        echo -e "${GREEN}Running all tests...${NC}"
        echo "Full test suite starting at $(date)" > "$LOG_FILE"

        # Run all test commands
        for test in "test-driver" "test-app" "test-pytd3" "test-calibration" "test-balance"; do
            echo -e "${YELLOW}Running $test...${NC}"
            $0 $test
            if [ $? -ne 0 ]; then
                echo -e "${RED}$test failed${NC}"
                exit 1
            fi
        done

        echo -e "${GREEN}All tests completed successfully${NC}"
        exit 0
        ;;

    status)
        echo -e "${YELLOW}Checking hexapod system status...${NC}"
        remote_exec "cd $REMOTE_DIR && ./hexapod_controller --status"
        exit $?
        ;;

    restart)
        echo -e "${YELLOW}Restarting hexapod services...${NC}"
        remote_exec "cd $REMOTE_DIR && sudo ./install.sh --restart"
        exit $?
        ;;

    custom)
        if [ -z "$1" ]; then
            echo -e "${RED}Error: Custom command required${NC}"
            usage
            exit 1
        fi
        echo -e "${YELLOW}Running custom command: $1${NC}"
        remote_exec "cd $REMOTE_DIR && $1"
        exit $?
        ;;

    *)
        echo -e "${RED}Error: Unknown command${NC}"
        usage
        exit 1
        ;;
esac
