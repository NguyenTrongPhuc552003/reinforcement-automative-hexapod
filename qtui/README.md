# Qt Hexapod Controller Application

A Qt-based graphical user interface application to control a hexapod robot running on BeagleBone AI. The application provides both classic widget-based and modern QML-based interfaces for robot control and monitoring.

## Features

- Real-time hexapod visualization and control
- IMU data monitoring and visualization
- Support for both keyboard and on-screen controls
- Dark/Light theme support
- Connection management with auto-discovery
- Balance mode and speed control
- Server simulation mode for testing
- Configurable keyboard shortcuts

## Prerequisites

- Qt 6.8.2 or later
- GCC with C++17 support
- BeagleBone AI board with hexapod server running
- Linux development environment

## Building the Application

1. Clone the repository:
```bash
git clone https://github.com/NguyenTrongPhuc552003/qt-application-hexapod.git
cd qt-application-hexapod
```

2. Build the application:
```bash
make -C build/Desktop_Qt_6_8_2-Debug/
```

After successful build, you'll find the executable `qtui` in the build directory.

## Deployment to BeagleBone AI

1. Copy the executable and required files:
```bash
scp build/Desktop_Qt_6_8_2-Debug/qtui debian@beaglebone.local:~/hexapod/
scp -r utils/* debian@beaglebone.local:~/hexapod/utils/
```

2. Install the server service on BeagleBone:
```bash
ssh debian@beaglebone.local
cd ~/hexapod/utils
sudo ./install.sh
```

## Usage

1. Start the application:
```bash
./qtui
```

2. Connect to your BeagleBone:
   - Use the default address: `beaglebone.local:8080`
   - Or click "Auto-Discover" to find available servers

3. Control the hexapod using:
   - Keyboard controls (WASD)
   - On-screen buttons
   - Touch-enabled joystick (QML interface)

### Keyboard Shortcuts

- `W` - Move forward
- `S` - Move backward
- `A` - Turn left
- `D` - Turn right
- `Space` - Stop movement
- `B` - Toggle balance mode
- `Ctrl+T` - Toggle dark/light theme
- `F1` - Show keyboard shortcuts

## Configuration

Settings are automatically saved and include:
- Last used connection details
- Window position and size
- Theme preference
- Control settings
- Recent connections

## Development

### Project Structure

```
qt-application-hexapod/
├── components/       # QML interface components
├── icons/            # Application icons
├── styles/           # QSS style sheets
├── utils/            # Server installation scripts
│   ├── install.sh    # BeagleBone installation script
│   └── server.py     # Python test server
├── *.cpp             # C++ source files
├── *.h               # Header files
├── *.ui              # Qt Designer UI files
├── *.qrc             # Qt Resource files
└── qtui.pro          # Qt Project file
```

### Adding New Features

1. UI modifications can be made through:
   - Qt Designer (*.ui files)
   - QML components (components/*.qml)
   - Direct code changes

2. Implement new controls in:
   - `mainwindow.cpp` for widget-based interface
   - QML files for modern interface

## License

[MIT License](LICENSE)

## Support

For support, please contact us at trong552003@gmail.com or visit our [GitHub Issues page](https://github.com/NguyenTrongPhuc552003/qt-application-hexapod/issues).