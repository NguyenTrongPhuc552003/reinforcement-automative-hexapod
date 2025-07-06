# Hexapod Utilities

This directory contains utility scripts for the Hexapod Robot Control System.

## Available Scripts

### install.sh
Kernel module management utility
```bash
# Install/load kernel module
sudo ./install.sh

# Remove/unload kernel module  
sudo ./install.sh -r

# Show status
./install.sh -s
```

### monitor.sh  
System monitoring and diagnostic tool
```bash
# Monitor system activities
./monitor.sh

# Monitor with logging
./monitor.sh -l

# Continuous monitoring
./monitor.sh -c
```

## Additional Utilities
If PyTD3 reinforcement learning module is installed:
- **pytd3/**: Python reinforcement learning components
- **config/**: Configuration files and calibration data

## System Integration
These utilities are designed to work with:
- BeagleBone AI hardware platform
- Debian-based Linux distributions  
- I2C hardware interface
- GPIO control systems

## Maintenance
Regular maintenance tasks:
```bash
# Check system health
sudo ./monitor.sh -h

# Backup calibration data
cp /etc/hexapod/calibration.cfg ~/hexapod_backup.cfg

# Update kernel module
sudo ./install.sh -r && sudo ./install.sh
```

## Support
For technical support:
- Check system logs: `journalctl -u hexapod`
- Review hardware connections
- Verify I2C functionality: `i2cdetect -r 1`
