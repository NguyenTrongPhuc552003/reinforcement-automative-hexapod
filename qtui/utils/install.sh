#!/bin/bash

# Hexapod Control System Installation Script for BeagleBone
# This script installs both the server components and required dependencies

# Configuration
SERVER_NAME="hexapod-server"
SERVER_DIR="/opt/$SERVER_NAME"
SERVICE_NAME="hexapod-server"
USER="debian"
GROUP="debian"
LOG_DIR="/var/log/$SERVER_NAME"
HEXAPOD_LIB="/usr/local/lib/libhexapod.so"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check if running as root
if [ "$EUID" -ne 0 ]; then
  echo -e "${RED}Please run as root (use sudo)${NC}"
  exit 1
fi

# Print welcome message
echo -e "${GREEN}=====================================================${NC}"
echo -e "${GREEN}    Hexapod Control System Installation Script${NC}"
echo -e "${GREEN}=====================================================${NC}"

# Install required dependencies
echo -e "${YELLOW}Installing required dependencies...${NC}"
apt-get update
apt-get install -y python3 python3-pip i2c-tools net-tools lsof

# Install Python dependencies
echo -e "${YELLOW}Installing Python dependencies...${NC}"
pip3 install pyserial

# Check for hexapod library
echo -e "${YELLOW}Checking for hexapod library...${NC}"
if [ ! -f "$HEXAPOD_LIB" ]; then
    echo -e "${RED}Warning: Hexapod library not found at $HEXAPOD_LIB${NC}"
    echo -e "${BLUE}The server will run in simulation mode${NC}"
fi

# Create server directory
echo -e "${YELLOW}Creating server directory...${NC}"
mkdir -p $SERVER_DIR

# Copy server files
echo -e "${YELLOW}Copying server files...${NC}"
cp "$(dirname "$0")/server.py" $SERVER_DIR/
chmod +x $SERVER_DIR/server.py

# Create log directory with proper permissions
echo -e "${YELLOW}Creating log directory...${NC}"
mkdir -p $LOG_DIR
chown $USER:$GROUP $LOG_DIR
chmod 755 $LOG_DIR

# Check if default port is available
PORT=8080
echo -e "${YELLOW}Checking if port $PORT is available...${NC}"
if lsof -i :$PORT > /dev/null; then
    echo -e "${RED}Warning: Port $PORT is already in use.${NC}"
    echo -e "${BLUE}Server will try to use alternative ports.${NC}"
else
    echo -e "${GREEN}Port $PORT is available.${NC}"
fi

# Create systemd service file with improved configuration
echo -e "${YELLOW}Creating systemd service...${NC}"
cat > /etc/systemd/system/$SERVICE_NAME.service << EOF
[Unit]
Description=Hexapod Remote Control Server
After=network.target
StartLimitIntervalSec=60
StartLimitBurst=3

[Service]
Type=simple
User=$USER
Group=$GROUP
WorkingDirectory=$SERVER_DIR
ExecStart=/usr/bin/python3 $SERVER_DIR/server.py
Restart=on-failure
RestartSec=10s
StandardOutput=journal
StandardError=journal
SyslogIdentifier=hexapod-server
Environment=PYTHONUNBUFFERED=1

# Hardening
PrivateTmp=true
ProtectSystem=full
NoNewPrivileges=true

[Install]
WantedBy=multi-user.target
EOF

# Reload systemd
echo -e "${YELLOW}Reloading systemd...${NC}"
systemctl daemon-reload

# Enable and start service
echo -e "${YELLOW}Enabling and starting service...${NC}"
systemctl enable $SERVICE_NAME
systemctl restart $SERVICE_NAME

# Wait a moment for service to start properly
echo -e "${YELLOW}Waiting for service to start...${NC}"
sleep 5

# Check service status
if systemctl is-active --quiet $SERVICE_NAME; then
  echo -e "${GREEN}Hexapod server installed and running!${NC}"
  echo -e "Service status: ${GREEN}ACTIVE${NC}"
  
  # Display server IP address and check which port was actually used
  IP_ADDR=$(hostname -I | awk '{print $1}')
  
  # Try to find which port the server is actually using
  PORT_INFO=$(netstat -tlnp 2>/dev/null | grep "python3.*server.py" | head -1 | awk '{print $4}' | rev | cut -d':' -f1 | rev)
  
  if [ -n "$PORT_INFO" ]; then
    echo -e "Server running at: ${GREEN}$IP_ADDR:$PORT_INFO${NC}"
  else
    echo -e "Server running at: ${GREEN}$IP_ADDR:8080${NC} (or an alternative port)"
  fi
else
  echo -e "${RED}Failed to start hexapod server service${NC}"
  echo -e "Check status with: ${YELLOW}systemctl status $SERVICE_NAME${NC}"
  echo -e "Check logs with: ${YELLOW}journalctl -u $SERVICE_NAME -f${NC}"
  echo -e "Detailed error information: ${YELLOW}journalctl -u $SERVICE_NAME --no-pager${NC}"
fi

# Print instructions
echo -e "${GREEN}=====================================================${NC}"
echo -e "${GREEN}Installation complete!${NC}"
echo -e "Service name: ${YELLOW}$SERVICE_NAME${NC}"
echo -e "Server directory: ${YELLOW}$SERVER_DIR${NC}"
echo -e "Log directory: ${YELLOW}$LOG_DIR${NC}"
echo -e ""
echo -e "Commands:"
echo -e "  ${YELLOW}systemctl status $SERVICE_NAME${NC} - Check service status"
echo -e "  ${YELLOW}systemctl restart $SERVICE_NAME${NC} - Restart service"
echo -e "  ${YELLOW}systemctl stop $SERVICE_NAME${NC} - Stop service"
echo -e "  ${YELLOW}journalctl -u $SERVICE_NAME -f${NC} - View live service logs"
echo -e "  ${YELLOW}netstat -tlnp | grep python3${NC} - Find which port the server is using"
echo -e "${GREEN}=====================================================${NC}"
