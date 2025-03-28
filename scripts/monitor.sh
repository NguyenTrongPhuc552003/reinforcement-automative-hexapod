#!/bin/bash

# Hexapod System Monitoring Script
# This script helps monitor system health and debug issues with the hexapod controller

# Color definitions
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

# Runtime variables
REFRESH_RATE=5  # Default refresh rate in seconds
MONITOR_MODE="all"  # Default monitoring mode
LOG_FILE="/tmp/hexapod_monitor.log"
DRIVER_NAME="hexapod_driver"
DEVICE_PATH="/dev/hexapod"
KERNEL_LOG_LINES=20  # Number of kernel log lines to display
TARGET_PID=""       # Specific PID to monitor
CONTINUOUS_MODE=0   # Whether to run continuous monitoring
CHECK_DEPS_ONLY=0   # Whether to only check dependencies and exit

# Check dependencies with better package management
check_dependencies() {
    local missing_deps=0
    local required_packages=(
        "procps"      # For ps, top commands
        "lsof"        # For checking open files/devices
        "bsdutils"    # For logger
        "strace"      # For tracing system calls
    )
    
    echo -e "${BLUE}Checking for required tools...${NC}"
    
    # Use apt-cache to check if packages are installed - more efficient than dpkg -l
    for pkg in "${required_packages[@]}"; do
        if ! dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
            echo -e "${RED}Missing package: $pkg${NC}"
            missing_deps=1
        else
            echo -e "${GREEN}✓ $pkg is installed${NC}"
        fi
    done
    
    if [ $missing_deps -eq 1 ]; then
        echo -e "${YELLOW}Please install missing packages with:${NC}"
        echo "sudo apt-get update && sudo apt-get install -y procps lsof bsdutils strace"
        return 1
    else
        echo -e "${GREEN}All required monitoring tools are installed.${NC}"
        return 0
    fi
}

# Show script usage
show_usage() {
    cat << EOT
Hexapod System Monitoring Script - Kernel Object Flow Monitor

Usage: $(basename "$0") [options]

Options:
  -r, --refresh RATE    Set refresh rate in seconds (default: $REFRESH_RATE)
  -m, --mode MODE       Set monitoring mode:
                          all:      Show all information (default)
                          kernel:   Show kernel driver and object flow 
                          process:  Show process info and interactions
                          ioctl:    Monitor IOCTL commands
                          dmesg:    Show detailed kernel messages
                          live:     Real-time IOCTL monitoring
                          imu:      Focus on MPU6050/IMU monitoring
  -p, --pid PID         Target a specific process by PID
  -c, --check           Check for required dependencies and exit
  -o, --continuous      Run in continuous mode (for IOCTL monitoring)
  -l, --log FILE        Log output to file (default: $LOG_FILE)
  -k, --kernel-lines N  Number of kernel log lines (default: $KERNEL_LOG_LINES)
  -h, --help            Show this help message

Examples:
  $(basename "$0")                          # Monitor everything with default settings
  $(basename "$0") --check                  # Verify all required packages are installed
  $(basename "$0") --mode ioctl             # Focus on IOCTL command monitoring
  $(basename "$0") --mode imu               # Focus on MPU6050/IMU monitoring
  $(basename "$0") --mode ioctl --pid 1234  # Monitor IOCTLs for specific process
  $(basename "$0") --mode live              # Show IOCTL calls in real-time
EOT
}

# Parse command-line arguments
parse_args() {
    while [ "$#" -gt 0 ]; do
        case "$1" in
            -r|--refresh)
                REFRESH_RATE="$2"
                shift 2
                ;;
            -m|--mode)
                MONITOR_MODE="$2"
                shift 2
                ;;
            -p|--pid)
                TARGET_PID="$2"
                shift 2
                ;;
            -c|--check)
                CHECK_DEPS_ONLY=1
                shift
                ;;
            -o|--continuous)
                CONTINUOUS_MODE=1
                shift
                ;;
            -l|--log)
                LOG_FILE="$2"
                shift 2
                ;;
            -k|--kernel-lines)
                KERNEL_LOG_LINES="$2"
                shift 2
                ;;
            -h|--help)
                show_usage
                exit 0
                ;;
            *)
                echo -e "${RED}Unknown option: $1${NC}" 
                show_usage
                exit 1
                ;;
        esac
    done
}

# Check driver status with enhanced kernel object info
check_driver_status() {
    echo -e "${BLUE}=== Kernel Driver Status & Object Flow ===${NC}"
    if lsmod | grep -q "$DRIVER_NAME"; then
        echo -e "${GREEN}✓ $DRIVER_NAME module is loaded${NC}"
        
        # Show driver details
        echo -e "\n${BLUE}Driver Information:${NC}"
        modinfo "$DRIVER_NAME" 2>/dev/null | grep -E 'version|author|description|depends' || \
            echo -e "${YELLOW}! Unable to get module info${NC}"
        
        # Show module parameters if any
        echo -e "\n${BLUE}Module Parameters:${NC}"
        if [ -d "/sys/module/$DRIVER_NAME/parameters" ]; then
            ls -1 "/sys/module/$DRIVER_NAME/parameters" | while read param; do
                value=$(cat "/sys/module/$DRIVER_NAME/parameters/$param" 2>/dev/null || echo "N/A")
                echo "  $param = $value"
            done
        else
            echo "  No parameters available"
        fi
        
        # Show memory usage
        echo -e "\n${BLUE}Module Memory Usage:${NC}"
        grep -A 1 "$DRIVER_NAME" /proc/modules | awk '{print "  Size: " $2 " bytes"}' || echo "  Unable to determine size"
        
        # Show kernel objects created by the driver
        echo -e "\n${BLUE}Kernel Objects:${NC}"
        ls -la /sys/class/hexapod* 2>/dev/null || ls -la /sys/devices/*hexapod* 2>/dev/null || echo "  No hexapod objects found in /sys"
        
    else
        echo -e "${RED}✗ $DRIVER_NAME module is not loaded${NC}"
        echo -e "${YELLOW}  Try loading it with: sudo modprobe $DRIVER_NAME${NC}"
    fi
    
    # Check device file with detailed permissions
    if [ -c "$DEVICE_PATH" ]; then
        echo -e "\n${GREEN}✓ Device file $DEVICE_PATH exists${NC}"
        
        # Get detailed device info
        echo -e "${BLUE}Device Information:${NC}"
        ls -l $DEVICE_PATH
        stat $DEVICE_PATH | grep -E "Device|Access|Modify|Change" | sed 's/^/  /'
        
        # Show who created the device
        echo -e "\n${BLUE}Device Creation Info:${NC}"
        CREATOR=$(dmesg | grep -i "hexapod.*created.*$DEVICE_PATH" | tail -1)
        if [ -n "$CREATOR" ]; then
            echo "  $CREATOR"
        else
            echo "  Device creation information not found in dmesg"
        fi
    else
        echo -e "\n${RED}✗ Device file $DEVICE_PATH does not exist${NC}"
        echo -e "${YELLOW}  Check if the driver loaded properly${NC}"
        echo -e "${YELLOW}  Run 'dmesg | grep hexapod' to see any errors${NC}"
    fi
    
    # Check who's using the device with enhanced details
    echo -e "\n${BLUE}Device Usage:${NC}"
    local users=$(lsof "$DEVICE_PATH" 2>/dev/null)
    if [ -n "$users" ]; then
        echo -e "${GREEN}Device is currently in use:${NC}"
        echo "$users"
        
        # Show more detailed info about each process using the device
        echo -e "\n${BLUE}Detailed Process Info for Users:${NC}"
        pids=$(lsof -t "$DEVICE_PATH" 2>/dev/null)
        for pid in $pids; do
            echo -e "${GREEN}Process $pid:${NC}"
            ps -p $pid -o pid,ppid,user,%cpu,%mem,start,cmd --no-headers | sed 's/^/  /'
            ls -l /proc/$pid/fd | grep "$DEVICE_PATH" | sed 's/^/  FD: /'
        done
    else
        echo -e "${YELLOW}Device is not currently in use by any process${NC}"
        echo "  This may indicate user space programs aren't connecting successfully"
    fi
}

# Monitor kernel messages related to hexapod with better filtering
check_kernel_messages() {
    echo -e "${BLUE}=== Kernel Message Analysis (Driver Communication) ===${NC}"
    
    # First check if there are any errors reported
    ERROR_LOGS=$(dmesg | grep -i "hexapod\|pca9685\|servo\|mpu6050" | grep -i "error\|fail\|warn")
    if [ -n "$ERROR_LOGS" ]; then
        echo -e "${RED}Errors detected in kernel logs:${NC}"
        echo "$ERROR_LOGS" | sed 's/^/  /'
    else
        echo -e "${GREEN}No driver errors detected in kernel logs${NC}"
    fi
    
    # Show most recent driver initialization messages
    echo -e "\n${BLUE}Driver Initialization:${NC}"
    dmesg | grep -i "hexapod.*init" | tail -5 | sed 's/^/  /'
    
    # Show servo controller messages
    echo -e "\n${BLUE}Servo Controller Messages:${NC}"
    dmesg | grep -i "servo\|pca9685" | tail -5 | sed 's/^/  /'
    
    # Show sensor messages with improved MPU6050 info
    echo -e "\n${BLUE}Sensor Messages:${NC}"
    dmesg | grep -i "mpu6050\|imu\|sensor" | tail -5 | sed 's/^/  /'
    
    # Look for sleep/wake cycles in MPU6050 with improved patterns
    echo -e "\n${BLUE}MPU6050 Sleep/Wake Activity:${NC}"
    MPU_SLEEP=$(dmesg | grep -i "mpu6050" | grep -i "sleep\|wake\|power\|reset" | tail -5)
    if [ -n "$MPU_SLEEP" ]; then
        echo "  Recent sleep/wake activity detected:"
        echo "$MPU_SLEEP" | sed 's/^/    /'
    else
        echo "  No recent sleep/wake activity detected in logs"
        echo "  Note: Driver may handle sleep/wake internally without logging"
    fi
    
    # Show most recent driver messages
    echo -e "\n${BLUE}Recent Driver Messages (last $KERNEL_LOG_LINES):${NC}"
    dmesg | grep -i "hexapod\|pca9685\|servo\|mpu6050" | tail -$KERNEL_LOG_LINES | sed 's/^/  /'
    
    # Check for recurring patterns that might indicate issues
    echo -e "\n${BLUE}Message Pattern Analysis:${NC}"
    PATTERNS=$(dmesg | grep -i "hexapod\|pca9685\|servo\|mpu6050" | sort | uniq -c | sort -nr | head -5)
    if [ -n "$PATTERNS" ]; then
        echo "  Most common message patterns:"
        echo "$PATTERNS" | sed 's/^/    /'
    else
        echo "  No recurring message patterns detected"
    fi
}

# Monitor IOCTL commands and data flow
monitor_ioctl_flow() {
    echo -e "${BLUE}=== IOCTL Command Flow Monitor ===${NC}"
    
    # Use the specified PID or find hexapod processes
    if [ -n "$TARGET_PID" ]; then
        # Verify the target PID exists and is a hexapod-related process
        if ! ps -p "$TARGET_PID" &>/dev/null; then
            echo -e "${RED}Process with PID $TARGET_PID does not exist${NC}"
            return 1
        fi
        
        # Check if it looks like a hexapod process
        if ! ps -p "$TARGET_PID" -o cmd= | grep -E "test_|hexapod" &>/dev/null; then
            echo -e "${YELLOW}Warning: PID $TARGET_PID does not appear to be a hexapod process${NC}"
            echo "Command: $(ps -p "$TARGET_PID" -o cmd=)"
            echo -e "Continue monitoring anyway? (y/n)"
            read -r response
            if [[ ! "$response" =~ ^[Yy]$ ]]; then
                return 1
            fi
        fi
        
        HEXAPOD_PID=$TARGET_PID
        echo -e "${GREEN}Monitoring specific PID: $HEXAPOD_PID${NC}"
        echo "Command: $(ps -p "$HEXAPOD_PID" -o cmd=)"
    else
        # Find all hexapod-related processes using a better pattern
        # Look for test_servo, test_movement, etc.
        HEXAPOD_PROCS=$(pgrep -f "test_|hexapod" | grep -v "$$")
        if [ -z "$HEXAPOD_PROCS" ]; then
            echo -e "${YELLOW}No hexapod user space programs currently running${NC}"
            echo -e "To monitor IOCTL flow, start a test program in another terminal:"
            echo "  sudo ./test_servo"
            echo "  sudo ./test_movement"
            return
        fi
        
        echo -e "${GREEN}Active hexapod programs detected:${NC}"
        for pid in $HEXAPOD_PROCS; do
            echo "  PID $pid: $(ps -p "$pid" -o cmd=)"
        done
        
        # Select the first process for analysis
        HEXAPOD_PID=$(echo "$HEXAPOD_PROCS" | head -1)
        echo -e "\n${GREEN}Selected PID $HEXAPOD_PID for monitoring${NC}"
        echo "Command: $(ps -p "$HEXAPOD_PID" -o cmd=)"
    fi
    
    # Handle continuous monitoring mode for live IOCTL viewing
    if [ "$MONITOR_MODE" = "live" ] || [ $CONTINUOUS_MODE -eq 1 ]; then
        echo -e "\n${BLUE}Starting live IOCTL monitoring (Press Ctrl+C to stop)...${NC}"
        echo -e "${YELLOW}Note: This requires sudo permission to attach to the process${NC}"
        
        # Create unique fifo for this invocation
        FIFO="/tmp/hexapod_monitor_$$.fifo"
        rm -f "$FIFO"
        mkfifo "$FIFO"
        
        # Start background parser process
        (
            while read -r line; do
                if echo "$line" | grep -q "ioctl"; then
                    # Extract the command and format it nicely
                    cmd=$(echo "$line" | grep -o "cmd=0x[0-9a-f]*")
                    
                    # Translate IOCTL command codes to human-readable names
                    case "$cmd" in
                        "cmd=0x4801")
                            name="SET_LEG_POSITION"
                            ;;
                        "cmd=0x8002")
                            name="GET_IMU_DATA"
                            data_info="Reading accelerometer and gyroscope data"
                            ;;
                        "cmd=0x4803")
                            name="CALIBRATE"
                            ;;
                        "cmd=0x4" | "cmd=0x0004") 
                            name="CENTER_ALL"
                            ;;
                        *)
                            name="UNKNOWN"
                            ;;
                    esac
                    
                    # Check for error return value with better diagnostics
                    if echo "$line" | grep -q "= -1"; then
                        if [ "$name" = "GET_IMU_DATA" ]; then
                            echo -e "$(date +%H:%M:%S.%N | cut -c1-15) ${RED}IOCTL $name ($cmd) FAILED - Possible MPU6050 sleep mode issue${NC}"
                        else
                            echo -e "$(date +%H:%M:%S.%N | cut -c1-15) ${RED}IOCTL $name ($cmd) FAILED${NC}"
                        fi
                    else
                        # Shows success in green and command details
                        result=$(echo "$line" | grep -o "= [0-9]*")
                        data=$(echo "$line" | grep -o "{.*}")
                        if [ "$name" = "GET_IMU_DATA" ]; then
                            echo -e "$(date +%H:%M:%S.%N | cut -c1-15) ${GREEN}IOCTL $name${NC} $result $data [MPU6050 active]"
                        else
                            echo -e "$(date +%H:%M:%S.%N | cut -c1-15) ${GREEN}IOCTL $name ($cmd)${NC} $result $data"
                        fi
                    fi
                fi
            done < "$FIFO"
            rm -f "$FIFO"
        ) &
        PARSER_PID=$!
        
        # Cleanup on exit
        trap 'kill $PARSER_PID 2>/dev/null; rm -f "$FIFO"; exit' INT TERM EXIT
        
        # Use strace with sudo if needed
        if ! strace -e ioctl -p "$HEXAPOD_PID" 2> "$FIFO"; then
            echo -e "${YELLOW}Trying with sudo privileges...${NC}"
            sudo strace -e ioctl -p "$HEXAPOD_PID" 2> "$FIFO"
        fi
        
        # Clean up
        kill $PARSER_PID 2>/dev/null
        rm -f "$FIFO"
        trap - INT TERM EXIT
        return
    fi
    
    # Regular (non-continuous) IOCTL analysis
    echo -e "\n${BLUE}Recent IOCTL Flow Analysis:${NC}"
    if command -v strace &>/dev/null; then
        echo "Running brief IOCTL analysis (5 second sample)..."
        
        # Run strace for 5 seconds on this process to capture IOCTLs
        TEMP_FILE=$(mktemp)
        
        # Try without sudo first, then with sudo if needed
        if ! timeout 5 strace -e ioctl -p "$HEXAPOD_PID" -o "$TEMP_FILE" 2>/dev/null; then
            echo -e "${YELLOW}Trying with sudo privileges...${NC}"
            sudo timeout 5 strace -e ioctl -p "$HEXAPOD_PID" -o "$TEMP_FILE" 2>/dev/null
        fi
        
        # Analyze the captured IOCTLs
        IOCTL_COUNT=$(grep -c "ioctl" "$TEMP_FILE" || echo 0)
        echo -e "  Captured $IOCTL_COUNT IOCTL calls in 5 seconds"
        
        # Show the most common IOCTL commands
        if [ "$IOCTL_COUNT" -gt 0 ]; then
            echo -e "\n${GREEN}Most frequent IOCTL commands:${NC}"
            grep "ioctl" "$TEMP_FILE" | grep -o "cmd=0x[0-9a-f]*" | sort | uniq -c | sort -nr | head -5 | sed 's/^/  /'
            
            # Check for specific hexapod commands
            echo -e "\n${GREEN}Hexapod command breakdown:${NC}"
            SET_LEG_COUNT=$(grep -c "cmd=0x4801" "$TEMP_FILE" || echo 0)
            GET_IMU_COUNT=$(grep -c "cmd=0x8002" "$TEMP_FILE" || echo 0)
            CALIBRATE_COUNT=$(grep -c "cmd=0x4803" "$TEMP_FILE" || echo 0)
            CENTER_ALL_COUNT=$(grep -c "cmd=0x0004" "$TEMP_FILE" || echo 0)
            
            echo "  SET_LEG_POSITION commands: $SET_LEG_COUNT"
            echo "  GET_IMU_DATA commands: $GET_IMU_COUNT"
            echo "  CALIBRATE commands: $CALIBRATE_COUNT"
            echo "  CENTER_ALL commands: $CENTER_ALL_COUNT"
            
            # Check for IOCTL errors
            IOCTL_ERRORS=$(grep "ioctl" "$TEMP_FILE" | grep -c "= -1" || echo 0)
            if [ "$IOCTL_ERRORS" -gt 0 ]; then
                echo -e "\n${RED}IOCTL errors detected: $IOCTL_ERRORS${NC}"
                grep "ioctl" "$TEMP_FILE" | grep "= -1" | head -3 | sed 's/^/  /'
            else
                echo -e "\n${GREEN}No IOCTL errors detected${NC}"
            fi
            
            # Show some complete IOCTL calls for detailed analysis
            if [ "$IOCTL_COUNT" -gt 0 ]; then
                echo -e "\n${BLUE}Sample IOCTL calls:${NC}"
                grep "ioctl" "$TEMP_FILE" | head -3 | sed 's/^/  /'
            fi
        fi
        
        # Clean up temp file
        rm -f "$TEMP_FILE"
    else
        echo -e "${YELLOW}strace not available - please install to enable IOCTL analysis${NC}"
    fi
    
    # Recommend tools for more detailed analysis
    echo -e "\n${BLUE}For detailed IOCTL flow analysis:${NC}"
    echo "  Run: $(basename "$0") --mode live"
    echo "  This will show all IOCTL calls in real-time"
}

# New function for focused MPU6050 monitoring
monitor_mpu6050() {
    echo -e "${BLUE}=== MPU6050 IMU Sensor Monitor ===${NC}"
    
    # Check if driver is loaded
    if ! lsmod | grep -q "$DRIVER_NAME"; then
        echo -e "${RED}Hexapod driver not loaded - cannot monitor MPU6050${NC}"
        return 1
    fi
    
    # Find MPU6050 test program if running
    MPU_PROC=$(pgrep -f "test_mpu6050" | head -1)
    if [ -n "$MPU_PROC" ]; then
        echo -e "${GREEN}MPU6050 test program running (PID: $MPU_PROC)${NC}"
        echo "Command: $(ps -p $MPU_PROC -o cmd=)"
        
        # Show resource usage
        echo -e "\n${BLUE}Resource Usage:${NC}"
        ps -p $MPU_PROC -o pid,ppid,%cpu,%mem,start,time --no-headers | sed 's/^/  /'
        
        # Set this as target for IOCTL monitoring
        TARGET_PID=$MPU_PROC
    else
        echo -e "${YELLOW}No MPU6050 test program running${NC}"
    fi
    
    # Show MPU6050 kernel messages
    echo -e "\n${BLUE}MPU6050 Kernel Messages:${NC}"
    dmesg | grep -i "mpu6050" | tail -15 | sed 's/^/  /'
    
    # Look for specific MPU6050 errors
    MPU_ERRORS=$(dmesg | grep -i "mpu6050" | grep -i "error\|fail\|timeout\|invalid")
    if [ -n "$MPU_ERRORS" ]; then
        echo -e "\n${RED}MPU6050 Errors Detected:${NC}"
        echo "$MPU_ERRORS" | sed 's/^/  /'
    fi
    
    # Check for I2C bus errors that might affect MPU6050
    I2C_ERRORS=$(dmesg | grep -i "i2c" | grep -i "error\|timeout\|fail" | tail -5)
    if [ -n "$I2C_ERRORS" ]; then
        echo -e "\n${YELLOW}I2C Bus Errors (may affect MPU6050):${NC}"
        echo "$I2C_ERRORS" | sed 's/^/  /'
    fi
    
    # Show successful read patterns
    echo -e "\n${BLUE}MPU6050 Read Pattern:${NC}"
    READ_PATTERN=$(dmesg | grep -i "mpu6050.*read" | sort | uniq -c | sort -nr | head -3)
    if [ -n "$READ_PATTERN" ]; then
        echo "  Read patterns detected:"
        echo "$READ_PATTERN" | sed 's/^/    /'
    else
        echo "  No read patterns detected in kernel logs"
    fi
    
    # Show sleep/wake activity with detailed explanation
    echo -e "\n${BLUE}MPU6050 Sleep/Wake Management:${NC}"
    echo "  The MPU6050 automatically enters sleep mode to conserve power."
    echo "  The driver detects this and wakes it when readings are requested."
    
    WAKE_COUNT=$(dmesg | grep -i "mpu6050.*wake" | wc -l)
    SLEEP_COUNT=$(dmesg | grep -i "mpu6050.*sleep" | wc -l)
    
    if [ $WAKE_COUNT -gt 0 ] || [ $SLEEP_COUNT -gt 0 ]; then
        echo "  Sleep events detected: $SLEEP_COUNT"
        echo "  Wake events detected: $WAKE_COUNT"
        
        # Show recent events
        echo -e "\n  Recent sleep/wake events:"
        dmesg | grep -i "mpu6050" | grep -i "sleep\|wake\|power" | tail -5 | sed 's/^/    /'
    else
        echo "  No explicit sleep/wake events logged"
        echo "  Note: Sleep/wake may be handled internally without kernel logging"
    fi
    
    # If a test program is running, monitor its IOCTL calls
    if [ -n "$MPU_PROC" ]; then
        echo -e "\n${BLUE}Monitoring IMU data requests from test program...${NC}"
        monitor_ioctl_flow
    fi
}

# Check running processes with focus on driver interaction
check_processes() {
    echo -e "${BLUE}=== Process & Driver Interaction Analysis ===${NC}"
    
    # Check for hexapod-related processes
    local hexapod_procs=$(ps aux | grep -i "hexapod\|test_\|servo" | grep -v "grep\|monitor")
    if [ -n "$hexapod_procs" ]; then
        echo -e "${GREEN}Hexapod-related processes:${NC}"
        echo "$hexapod_procs"
        
        # Get PIDs for detailed analysis
        local pids=$(ps aux | grep -i "hexapod\|test_\|servo" | grep -v "grep\|monitor" | awk '{print $2}')
        
        # Check open files for each process
        echo -e "\n${BLUE}Open files & Driver interactions:${NC}"
        for pid in $pids; do
            echo -e "${GREEN}PID $pid:${NC}"
            local cmd=$(ps -p $pid -o cmd= 2>/dev/null)
            echo "  Command: $cmd"
            
            # Show file descriptors - focus on device files
            echo "  Open files:"
            ls -l /proc/$pid/fd 2>/dev/null | grep -E "/$|/dev/" | sed 's/^/    /'
            
            # Check for driver interaction
            if ls -l /proc/$pid/fd 2>/dev/null | grep -q "$DEVICE_PATH"; then
                echo -e "  ${GREEN}✓ Process has open connection to driver${NC}"
            else
                echo -e "  ${YELLOW}✗ No direct connection to hexapod driver${NC}"
            fi
            
            # Check memory maps for driver regions
            echo "  Memory mappings (kernel driver regions):"
            grep -E "hexapod|pca9685|mpu6050|servo" /proc/$pid/maps 2>/dev/null || echo "    None found"
        done
    else
        echo -e "${YELLOW}No hexapod-related processes found${NC}"
        echo "  Try running test_servo or test_movement in another terminal"
    fi
    
    # Show resource usage for key processes
    echo -e "\n${BLUE}Resource usage summary:${NC}"
    ps -eo pid,ppid,user,%cpu,%mem,stat,start_time,command | head -1
    ps -eo pid,ppid,user,%cpu,%mem,stat,start_time,command | grep -i "hexapod\|test_\|servo" | grep -v "grep\|monitor"
    
    # Show system resource stats that may affect driver performance
    echo -e "\n${BLUE}System Resource State:${NC}"
    echo "  Load average: $(cat /proc/loadavg)"
    echo "  Memory free: $(free -m | grep Mem | awk '{print $4 "MB"}' 2>/dev/null || echo "N/A")"
    echo "  CPU usage: $(top -bn1 | grep "Cpu(s)" | awk '{print $2 + $4 "%"}' 2>/dev/null || echo "N/A")"
}

# Show complete kernel log flow for driver
show_dmesg_flow() {
    echo -e "${BLUE}=== Complete Kernel Message Flow ===${NC}"
    
    # Temporally ordered driver loading sequence
    echo -e "${BLUE}Driver Loading Sequence:${NC}"
    dmesg | grep -i "hexapod" | grep -i "init\|load" | sed 's/^/  /'
    
    # Servo initialization sequence
    echo -e "\n${BLUE}PCA9685 Servo Controller Initialization:${NC}"
    dmesg | grep -i "pca9685" | grep -i "init\|detect\|register" | sed 's/^/  /'
    
    # Sensor initialization sequence
    echo -e "\n${BLUE}MPU6050 Sensor Initialization:${NC}"
    dmesg | grep -i "mpu6050" | grep -i "init\|detect\|register" | sed 's/^/  /'
    
    # Show all hexapod related messages
    echo -e "\n${BLUE}All Driver Related Messages:${NC}"
    dmesg | grep -i "hexapod\|pca9685\|servo\|mpu6050" | sed 's/^/  /'
    
    # Show critical errors
    echo -e "\n${BLUE}Critical Errors:${NC}"
    ERRORS=$(dmesg | grep -i "hexapod\|pca9685\|servo\|mpu6050" | grep -i "error\|fail\|fault")
    if [ -n "$ERRORS" ]; then
        echo "$ERRORS" | sed 's/^/  /'
    else
        echo "  No critical errors detected"
    fi
}

# Run monitoring based on selected mode
run_monitoring() {
    case "$MONITOR_MODE" in
        all)
            clear
            echo -e "${BLUE}====== Hexapod Kernel Object Flow Monitor ======${NC}"
            echo "Refreshing every $REFRESH_RATE seconds. Press Ctrl+C to exit."
            echo -e "Timestamp: $(date)\n"
            
            check_driver_status
            echo -e "\n${YELLOW}----------------------------------------------${NC}\n"
            check_kernel_messages
            echo -e "\n${YELLOW}----------------------------------------------${NC}\n"
            check_processes
            echo -e "\n${YELLOW}----------------------------------------------${NC}\n"
            monitor_ioctl_flow
            ;;
        kernel)
            clear
            check_driver_status
            ;;
        process)
            clear
            check_processes
            ;;
        ioctl)
            clear
            monitor_ioctl_flow
            ;;
        live)
            clear
            # Live mode is continuous IOCTL monitoring
            CONTINUOUS_MODE=1
            monitor_ioctl_flow
            # Don't refresh for live mode - it's interactive until Ctrl+C
            exit 0
            ;;
        dmesg)
            clear
            show_dmesg_flow
            ;;
        imu)
            clear
            # IMU specific monitoring mode
            monitor_mpu6050
            ;;
        *)
            echo -e "${RED}Invalid monitoring mode: $MONITOR_MODE${NC}"
            show_usage
            exit 1
            ;;
    esac
}

# Main function
main() {
    # Parse command-line arguments first
    parse_args "$@"
    
    # If check-only mode is requested, just check dependencies and exit
    if [ $CHECK_DEPS_ONLY -eq 1 ]; then
        check_dependencies
        exit $?
    fi
    
    # For live monitoring mode, just run once
    if [ "$MONITOR_MODE" = "live" ]; then
        # Before monitoring in live mode, verify dependencies
        if ! check_dependencies >/dev/null 2>&1; then
            echo -e "${RED}Required dependencies are missing. Run with --check for details.${NC}"
            exit 1
        fi
        run_monitoring
        exit 0
    fi
    
    # Before continuous monitoring, verify dependencies (silently)
    if ! check_dependencies >/dev/null 2>&1; then
        echo -e "${RED}Required dependencies are missing. Run with --check for details.${NC}"
        exit 1
    fi
    
    # Run continuously with refresh rate
    while true; do
        run_monitoring | tee -a "$LOG_FILE"
        echo -e "\nPress Ctrl+C to exit. Next update in $REFRESH_RATE seconds..."
        sleep "$REFRESH_RATE"
    done
}

# Execute main function with all arguments
main "$@"
