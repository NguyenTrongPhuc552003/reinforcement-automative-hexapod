#!/bin/bash

# Exit on any error
set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Get the project root directory
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KERNEL_MODULE_DIR="${PROJECT_ROOT}/driver"
USER_SPACE_DIR="${PROJECT_ROOT}/app"
PYTD3_DIR="${PROJECT_ROOT}/pytd3"
DEPLOY_DIR="${PROJECT_ROOT}/deploy"
UTILS_DIR="${PROJECT_ROOT}/utils"
PACKAGE_DIR="${PROJECT_ROOT}/package"

# Create deploy directory with "whoami" user permission if it doesn't exist
if [ ! -d "${DEPLOY_DIR}" ]; then
    mkdir -p "${DEPLOY_DIR}"
    chown -R "$(whoami)":"$(whoami)" "${DEPLOY_DIR}"
fi

# Function to show usage
usage() {
    echo "Usage: $0 [OPTIONS]"
    echo "Options:"
    echo "  (no argument)         Build all components"
    echo "  -i, --image [TYPE]    Build Docker images (optional TYPE: app|driver|pytd3)"
    echo "  -b, --build [TYPE]    Build components (TYPE: app|driver|pytd3|all)"
    echo "  -g, --generate [TYPE] Generate package (TYPE: deb|tar.xz|rpm, default: deb)"
    echo "  -s, --setup           Setup Python virtual environment for PyTD3"
    echo "  -l, --uml             Build UML diagrams (no Docker required)"
    echo "  -t, --utility         Create utility scripts (no Docker required)"
    echo "  -c, --clean [TYPE]    Clean build artifacts (optional TYPE: app|driver|pytd3|deploy|package)"
    echo "  -p, --purge [TYPE]    Purge all build artifacts and Docker images (optional TYPE: app|driver|pytd3)"
    echo "  -n, --no-cache        Build all without cache"
    echo "  -h, --help            Show this help message"
    echo ""
    echo "Examples:"
    echo "  ./scripts/build.sh -i app           Build only the app Docker image"
    echo "  ./scripts/build.sh -b app           Build only user space programs"
    echo "  ./scripts/build.sh -b driver        Build only kernel modules"
    echo "  ./scripts/build.sh -g               Generate Debian package (default)"
    echo "  ./scripts/build.sh -g tar.xz        Generate tar.xz package"
    echo "  ./scripts/build.sh -bg              Build all and generate Debian package"
    echo "  ./scripts/build.sh -b driver -g rpm Build driver and generate RPM package"
    echo ""
    echo "Options can be combined, e.g., -bt to build all components & utility scripts"
    echo "Note: UML diagrams (-l) and utility scripts (-t) can be built without Docker"
    echo ""
    exit 1
}

# Function to print colored messages
log() {
    local color=$1
    shift
    echo -e "${color}$@${NC}"
}

# Function to build Docker image
build_docker_image() {
    local image_type=$1
    local cache_flag=$2
    
    log "${YELLOW}" "Building Docker image: ${image_type}"
    
    # Check if Docker is installed and running
    if ! command -v docker &> /dev/null; then
        log "${RED}" "Docker is not installed or not in PATH"
        exit 1
    fi
    
    # Verify Docker directory and Dockerfile exist
    if [ ! -d "${PROJECT_ROOT}/docker" ]; then
        log "${RED}" "docker/ directory not found in project root!"
        exit 1
    fi
    
    # Build the specified Docker image
    case "${image_type}" in
        app)
            if [ ! -f "${PROJECT_ROOT}/docker/app.Dockerfile" ]; then
                log "${RED}" "app.Dockerfile not found in docker/ directory!"
                exit 1
            fi
            docker build ${cache_flag} -t hexapod-app -f "${PROJECT_ROOT}/docker/app.Dockerfile" "${PROJECT_ROOT}" || {
                log "${RED}" "Docker build for app failed!"
                exit 1
            }
            ;;
        driver)
            if [ ! -f "${PROJECT_ROOT}/docker/driver.Dockerfile" ]; then
                log "${RED}" "driver.Dockerfile not found in docker/ directory!"
                exit 1
            fi
            docker build ${cache_flag} -t hexapod-driver -f "${PROJECT_ROOT}/docker/driver.Dockerfile" "${PROJECT_ROOT}" || {
                log "${RED}" "Docker build for driver failed!"
                exit 1
            }
            ;;
        pytd3)
            if [ ! -f "${PROJECT_ROOT}/docker/pytd3.Dockerfile" ]; then
                log "${RED}" "pytd3.Dockerfile not found in docker/ directory!"
                exit 1
            fi
            docker build ${cache_flag} -t hexapod-pytd3 -f "${PROJECT_ROOT}/docker/pytd3.Dockerfile" "${PROJECT_ROOT}" || {
                log "${RED}" "Docker build for PyTD3 failed!"
                exit 1
            }
            ;;
        all)
            # Build all images
            log "${YELLOW}" "Building all Docker images..."
            build_docker_image "app" ${cache_flag}
            build_docker_image "driver" ${cache_flag}
            build_docker_image "pytd3" ${cache_flag}
            ;;
        *)
            log "${RED}" "Unknown image type: ${image_type}"
            exit 1
            ;;
    esac
    
    log "${GREEN}" "Docker image ${image_type} built successfully!"
}

# Function to build kernel module (now called by driver build type)
build_device_driver() {
    log "${YELLOW}" "Preparing kernel modules..."
    
    # First build the driver image if not already built
    if ! docker image inspect hexapod-driver &> /dev/null; then
        build_docker_image "driver" ""
    fi
    
    # Release build by default, add DEBUG=1 for debug build
    local build_type="release"
    if [ "$DEBUG" = "1" ]; then
        build_type="debug"
        log "${YELLOW}" "Building DEBUG version of kernel modules..."
    else
        log "${YELLOW}" "Building RELEASE version of kernel modules..."
    fi
    
    docker run --rm \
        -v "${KERNEL_MODULE_DIR}:/build/module" \
        -v "${DEPLOY_DIR}:/build/deploy" \
        -e BUILD_TYPE=${build_type} \
        hexapod-driver module || {
        log "${RED}" "Kernel modules build failed!"
        exit 1
    }
}

# Function to build user space program (now called by app build type)
build_user_space() {
    log "${YELLOW}" "Preparing user space programs..."
    
    # First build the app image if not already built
    if ! docker image inspect hexapod-app &> /dev/null; then
        build_docker_image "app" ""
    fi
    
    # Release build by default, add DEBUG=1 for debug build
    local env_params=""
    if [ "$DEBUG" = "1" ]; then
        env_params="-e DEBUG=1"
        log "${YELLOW}" "Building DEBUG version of user space programs..."
    else
        log "${YELLOW}" "Building RELEASE version of user space programs..."
    fi
    
    # Run Docker with proper command format
    docker run --rm \
        -v "${USER_SPACE_DIR}:/build/user" \
        -v "${DEPLOY_DIR}:/build/deploy" \
        ${env_params} \
        hexapod-app user || {
        log "${RED}" "User space programs build failed!"
        exit 1
    }
}

# Function to setup Python virtual environment for PyTD3
setup_pytd3_env() {
    log "${YELLOW}" "Setting up Python virtual environment for PyTD3..."
    
    # First build the pytd3 image if not already built
    if ! docker image inspect hexapod-pytd3 &> /dev/null; then
        build_docker_image "pytd3" ""
    fi
    
    # Run Docker with proper command format
    docker run --rm \
        -v "${PYTD3_DIR}:/build/pytd3" \
        -v "${DEPLOY_DIR}:/build/deploy" \
        hexapod-pytd3 setup_env || {
        log "${RED}" "PyTD3 environment setup failed!"
        exit 1
    }
    
    # Create default config directory if it doesn't exist
    if [ ! -d "${PYTD3_DIR}/config" ]; then
        mkdir -p "${PYTD3_DIR}/config"
    fi
    
    log "${GREEN}" "PyTD3 environment setup successful!"
}

# Function to build PyTD3 module
build_pytd3() {
    log "${YELLOW}" "Building PyTD3 module..."
    
    # First build the pytd3 image if not already built
    if ! docker image inspect hexapod-pytd3 &> /dev/null; then
        build_docker_image "pytd3" ""
    fi
    
    # Create build directory if it doesn't exist
    if [ ! -d "${PYTD3_DIR}/build" ]; then
        mkdir -p "${PYTD3_DIR}/build"
    fi
    
    # Run Docker with proper command format
    docker run --rm \
        -v "${PYTD3_DIR}:/build/pytd3" \
        -v "${DEPLOY_DIR}:/build/deploy" \
        hexapod-pytd3 pytd3
    
    if [ $? -eq 0 ]; then
        log "${GREEN}" "PyTD3 build successful!"
        # Copy relevant files to deploy directory (if needed)
        mkdir -p "${DEPLOY_DIR}/pytd3"
        cp -r "${PYTD3_DIR}/build/"*.so \
            "${PYTD3_DIR}/build/pytd3_"* \
            "${DEPLOY_DIR}/pytd3/" 2>/dev/null || true
    else
        log "${RED}" "PyTD3 build failed!"
        return 1
    fi
}

# Function to build UML diagrams using export.sh script (no Docker required)
build_uml_diagrams() {
    log "${YELLOW}" "Building UML diagrams..."
    
    # Check if export.sh exists
    if [ ! -f "${PROJECT_ROOT}/scripts/export.sh" ]; then
        log "${RED}" "export.sh not found in scripts directory!"
        exit 1
    fi
    
    # Run export.sh script directly (no Docker needed)
    bash "${PROJECT_ROOT}/scripts/export.sh"
}

# Function to create utility scripts (no Docker required)
build_utility_scripts() {
    log "${YELLOW}" "Copying utility scripts..."
    
    # Create deploy directory if it doesn't exist
    if [ ! -d "${DEPLOY_DIR}" ]; then
        mkdir -p "${DEPLOY_DIR}"
    fi
    
    # Copy installation script
    if [ -f "${UTILS_DIR}/install.sh" ]; then
        cp "${UTILS_DIR}/install.sh" "${DEPLOY_DIR}/"
        chmod +x "${DEPLOY_DIR}/install.sh"
        log "${GREEN}" "Copied install.sh to deploy directory"
    else
        log "${RED}" "install.sh not found in utils directory!"
    fi
    
    # Copy monitoring script
    if [ -f "${UTILS_DIR}/monitor.sh" ]; then
        cp "${UTILS_DIR}/monitor.sh" "${DEPLOY_DIR}/"
        chmod +x "${DEPLOY_DIR}/monitor.sh"
        log "${GREEN}" "Copied monitor.sh to deploy directory"
    else
        log "${RED}" "monitor.sh not found in utils directory!"
    fi
    
    log "${GREEN}" "Utility scripts prepared successfully"
}

# Function to generate packages
generate_package() {
    local package_type=$1
    
    log "${YELLOW}" "Generating ${package_type} package..."
    
    # Check if deploy directory has content
    if [ ! -d "${DEPLOY_DIR}" ] || [ -z "$(ls -A ${DEPLOY_DIR} 2>/dev/null)" ]; then
        log "${RED}" "Deploy directory is empty or doesn't exist!"
        log "${YELLOW}" "Please run build first: ./scripts/build.sh -b"
        return 1
    fi
    
    # Get version from version.txt or use current date
    local VERSION="1.0.0"
    if [ -f "${PACKAGE_DIR}/version.txt" ]; then
        VERSION=$(cat "${PACKAGE_DIR}/version.txt" | tr -d '\n\r')
    else
        VERSION="1.0.0-$(date +%Y%m%d)"
    fi
    
    log "${GREEN}" "Creating package version: ${VERSION}"
    
    # Create package output directory
    local PACKAGE_OUTPUT_DIR="${PROJECT_ROOT}/package"
    mkdir -p "${PACKAGE_OUTPUT_DIR}"
    
    case "${package_type}" in
        deb)
            generate_deb_package "${VERSION}" "${PACKAGE_OUTPUT_DIR}"
            ;;
        tar.xz)
            generate_tarxz_package "${VERSION}" "${PACKAGE_OUTPUT_DIR}"
            ;;
        rpm)
            generate_rpm_package "${VERSION}" "${PACKAGE_OUTPUT_DIR}"
            ;;
        *)
            log "${RED}" "Unsupported package type: ${package_type}"
            log "${YELLOW}" "Supported types: deb, tar.xz, rpm"
            return 1
            ;;
    esac
}

# Function to generate Debian package
generate_deb_package() {
    local version=$1
    local output_dir=$2
    local package_name="hexapod"
    
    log "${YELLOW}" "Building Debian package..."
    
    # Create temporary build directory
    local build_dir="${output_dir}/build_deb"
    rm -rf "${build_dir}"
    mkdir -p "${build_dir}"
    
    # Copy DEBIAN control files
    mkdir -p "${build_dir}/DEBIAN"
    
    # Create proper control file with correct formatting
    if [ -f "${PROJECT_ROOT}/package/DEBIAN/control" ]; then
        # If template exists, use it but substitute variables
        sed -e "s/\${package_name}/${package_name}/g" \
            -e "s/\${version}/${version}/g" \
            "${PROJECT_ROOT}/package/DEBIAN/control" > "${build_dir}/DEBIAN/control"
    else
        # Create a default control file with proper Debian format
        cat > "${build_dir}/DEBIAN/control" << EOF
Package: ${package_name}
Version: ${version}
Section: misc
Priority: optional
Architecture: armhf
Maintainer: StrongFood <trong552003@gmail.com>
Depends: libc6 (>= 2.28), python3 (>= 3.6)
Description: Hexapod Robot Control System
 Complete control system for hexapod robots with kernel drivers,
 user space applications, and reinforcement learning modules.
 .
 This package includes:
 .
  * Kernel driver for hardware interface
  * Main control application
  * Test and calibration utilities
  * System monitoring tools
  * PyTD3 reinforcement learning support
EOF
    fi
    
    # Verify control file has proper line endings (no Windows CRLF)
    sed -i 's/\r$//' "${build_dir}/DEBIAN/control"
    
    # Ensure control file ends with a newline
    sed -i -e '$a\' "${build_dir}/DEBIAN/control"
    
    # Copy and process postinst script
    if [ -f "${PROJECT_ROOT}/package/DEBIAN/postinst" ]; then
        cp "${PROJECT_ROOT}/package/DEBIAN/postinst" "${build_dir}/DEBIAN/"
    else
        # Create a default postinst script
        cat > "${build_dir}/DEBIAN/postinst" << 'EOF'
#!/bin/bash
set -e

# Install kernel module
if [ -f /lib/modules/$(uname -r)/extra/hexapod_driver.ko ]; then
    depmod -a
    modprobe hexapod_driver || true
    echo "hexapod_driver" >> /etc/modules || true
fi

# Set executable permissions
chmod +x /usr/local/bin/hexapod_app || true
chmod +x /usr/local/bin/test_* || true
chmod +x /opt/hexapod/install.sh || true

echo "Hexapod Robot Control System installed successfully"
exit 0
EOF
    fi
    chmod 755 "${build_dir}/DEBIAN/postinst"
    
    # Copy and process prerm script
    if [ -f "${PROJECT_ROOT}/package/DEBIAN/prerm" ]; then
        cp "${PROJECT_ROOT}/package/DEBIAN/prerm" "${build_dir}/DEBIAN/"
    else
        # Create a default prerm script
        cat > "${build_dir}/DEBIAN/prerm" << 'EOF'
#!/bin/bash
set -e

# Remove kernel module
if lsmod | grep -q hexapod_driver; then
    rmmod hexapod_driver || true
fi

# Remove from auto-load
sed -i '/hexapod_driver/d' /etc/modules 2>/dev/null || true

exit 0
EOF
    fi
    chmod 755 "${build_dir}/DEBIAN/prerm"
    
    # Create directory structure and copy files
    create_package_structure "${build_dir}" "${version}"
    
    # Validate the control file before building
    log "${YELLOW}" "Validating package structure..."
    
    # Check control file syntax using dpkg-deb --info on a temporary package
    if ! dpkg-deb --info "${build_dir}" &>/dev/null 2>&1; then
        # If dpkg-deb validation fails, do manual validation
        log "${YELLOW}" "Running manual control file validation..."
        
        # Check for required fields and proper format
        local required_fields=("Package" "Version" "Architecture" "Maintainer" "Description")
        for field in "${required_fields[@]}"; do
            if ! grep -q "^${field}:" "${build_dir}/DEBIAN/control"; then
                log "${RED}" "Package validation failed - missing required field: ${field}"
                return 1
            fi
        done
        
        # Check that Depends field has proper format (if present)
        if grep -q "^Depends:" "${build_dir}/DEBIAN/control"; then
            local depends_line=$(grep "^Depends:" "${build_dir}/DEBIAN/control")
            # Basic validation - ensure no trailing spaces or malformed syntax
            if [[ "$depends_line" =~ [[:space:]]+$ ]]; then
                log "${RED}" "Package validation failed - Depends field has trailing whitespace"
                return 1
            fi
        fi
        
        # Check file permissions and basic structure
        if [ ! -f "${build_dir}/DEBIAN/control" ] || [ ! -s "${build_dir}/DEBIAN/control" ]; then
            log "${RED}" "Package validation failed - control file is missing or empty"
            return 1
        fi
    fi
    
    # Build the .deb package with proper output filename
    local output_file="${output_dir}/${package_name}_${version}_armhf.deb"
    
    log "${YELLOW}" "Building Debian package..."
    if dpkg-deb --root-owner-group --build "${build_dir}" "${output_file}"; then
        log "${GREEN}" "Debian package created: ${output_file}"
        
        # Create a symlink for easy access
        ln -sf "${package_name}_${version}_armhf.deb" "${output_dir}/hexapod_latest.deb" 2>/dev/null || true
        
        return 0
    else
        log "${RED}" "Failed to create Debian package"
        return 1
    fi
}

# Function to generate tar.xz package
generate_tarxz_package() {
    local version=$1
    local output_dir=$2
    local package_name="hexapodrobot"
    
    log "${YELLOW}" "Building tar.xz package..."
    
    # Create temporary build directory
    local build_dir="${output_dir}/build_tarxz"
    rm -rf "${build_dir}"
    mkdir -p "${build_dir}/${package_name}"
    
    # Create directory structure and copy files
    create_package_structure "${build_dir}/${package_name}" "${version}"
    
    # Create installation script
    cat > "${build_dir}/${package_name}/install.sh" << 'EOF'
#!/bin/bash
set -e

echo "Installing Hexapod Robot Control System..."

# Copy kernel module
if [ -f "lib/modules/4.14.108-ti-r144/extra/hexapod_driver.ko" ]; then
    mkdir -p "/lib/modules/$(uname -r)/extra"
    cp "lib/modules/4.14.108-ti-r144/extra/hexapod_driver.ko" "/lib/modules/$(uname -r)/extra/"
    depmod -a
    modprobe hexapod_driver || true
    echo "hexapod_driver" >> /etc/modules || true
    echo "Kernel module installed"
fi

# Copy executables
mkdir -p /usr/local/bin
cp usr/local/bin/* /usr/local/bin/
chmod +x /usr/local/bin/hexapod_app
chmod +x /usr/local/bin/test_*
echo "Executables installed"

# Copy utilities
mkdir -p /opt/hexapod
cp opt/hexapod/* /opt/hexapod/
chmod +x /opt/hexapod/install.sh
echo "Utilities installed"

echo "Installation completed successfully!"
echo "You can now run: hexapod_app"
EOF
    chmod +x "${build_dir}/${package_name}/install.sh"
    
    # Create the tar.xz package
    local output_file="${output_dir}/${package_name}_${version}.tar.xz"
    cd "${build_dir}"
    tar -cJf "${output_file}" "${package_name}"
    
    if [ $? -eq 0 ]; then
        log "${GREEN}" "tar.xz package created: ${output_file}"
        return 0
    else
        log "${RED}" "Failed to create tar.xz package"
        return 1
    fi
}

# Function to generate RPM package
generate_rpm_package() {
    local version=$1
    local output_dir=$2
    local package_name="hexapodrobot"
    
    log "${YELLOW}" "Building RPM package..."
    
    # Check if rpmbuild is available
    if ! command -v rpmbuild &> /dev/null; then
        log "${RED}" "rpmbuild not found. Please install rpm-build package."
        return 1
    fi
    
    # Create RPM build environment
    local rpm_build_dir="${output_dir}/build_rpm"
    rm -rf "${rpm_build_dir}"
    mkdir -p "${rpm_build_dir}"/{BUILD,RPMS,SOURCES,SPECS,SRPMS}
    
    # Create source directory
    mkdir -p "${rpm_build_dir}/SOURCES/${package_name}-${version}"
    create_package_structure "${rpm_build_dir}/SOURCES/${package_name}-${version}" "${version}"
    
    # Create source tarball
    cd "${rpm_build_dir}/SOURCES"
    tar -czf "${package_name}-${version}.tar.gz" "${package_name}-${version}"
    
    # Create RPM spec file
    cat > "${rpm_build_dir}/SPECS/${package_name}.spec" << EOF
Name:           ${package_name}
Version:        ${version}
Release:        1%{?dist}
Summary:        Hexapod Robot Control System
License:        MIT
Source0:        %{name}-%{version}.tar.gz
BuildArch:      armv7hl

%description
Complete control system for hexapod robots with kernel drivers,
user space applications, and reinforcement learning modules.

%prep
%setup -q

%install
rm -rf %{buildroot}

# Install kernel module
mkdir -p %{buildroot}/lib/modules/4.14.108-ti-r144/extra
cp lib/modules/4.14.108-ti-r144/extra/* %{buildroot}/lib/modules/4.14.108-ti-r144/extra/

# Install executables
mkdir -p %{buildroot}/usr/local/bin
cp usr/local/bin/* %{buildroot}/usr/local/bin/

# Install utilities
mkdir -p %{buildroot}/opt/hexapod
cp opt/hexapod/* %{buildroot}/opt/hexapod/

%files
/lib/modules/4.14.108-ti-r144/extra/*
/usr/local/bin/*
/opt/hexapod/*

%post
depmod -a || true
modprobe hexapod_driver || true
echo "hexapod_driver" >> /etc/modules || true

%preun
rmmod hexapod_driver || true
sed -i '/hexapod_driver/d' /etc/modules || true

%changelog
* $(date '+%a %b %d %Y') Build System <build@hexapod.local> - ${version}-1
- Automated build
EOF
    
    # Build the RPM
    rpmbuild --define "_topdir ${rpm_build_dir}" -bb "${rpm_build_dir}/SPECS/${package_name}.spec"
    
    # Find and copy the built RPM
    local built_rpm=$(find "${rpm_build_dir}/RPMS" -name "*.rpm" | head -1)
    if [ -n "${built_rpm}" ]; then
        local output_file="${output_dir}/${package_name}_${version}.rpm"
        cp "${built_rpm}" "${output_file}"
        log "${GREEN}" "RPM package created: ${output_file}"
        return 0
    else
        log "${RED}" "Failed to create RPM package"
        return 1
    fi
}

# Function to create common package structure
create_package_structure() {
    local build_dir=$1
    local version=$2
    
    log "${YELLOW}" "Creating package structure..."
    
    # Create kernel module directory
    mkdir -p "${build_dir}/lib/modules/4.14.108-ti-r144/extra"
    if [ -f "${DEPLOY_DIR}/hexapod_driver.ko" ]; then
        cp "${DEPLOY_DIR}/hexapod_driver.ko" "${build_dir}/lib/modules/4.14.108-ti-r144/extra/"
        log "${GREEN}" "Copied kernel module"
    else
        log "${YELLOW}" "Warning: No kernel module found in deploy directory"
    fi
    
    # Create executables directory
    mkdir -p "${build_dir}/usr/local/bin"
    for exe in "${DEPLOY_DIR}"/hexapod_app "${DEPLOY_DIR}"/test_*; do
        if [ -f "${exe}" ]; then
            cp "${exe}" "${build_dir}/usr/local/bin/"
        fi
    done
    log "${GREEN}" "Copied executables"
    
    # Create utilities directory
    mkdir -p "${build_dir}/opt/hexapod"
    if [ -f "${DEPLOY_DIR}/install.sh" ]; then
        cp "${DEPLOY_DIR}/install.sh" "${build_dir}/opt/hexapod/"
    fi
    if [ -f "${DEPLOY_DIR}/monitor.sh" ]; then
        cp "${DEPLOY_DIR}/monitor.sh" "${build_dir}/opt/hexapod/"
    fi
    log "${GREEN}" "Copied utilities"
    
    # Copy PyTD3 files if they exist
    if [ -d "${DEPLOY_DIR}/pytd3" ]; then
        mkdir -p "${build_dir}/opt/hexapod/pytd3"
        cp -r "${DEPLOY_DIR}/pytd3"/* "${build_dir}/opt/hexapod/pytd3/"
        log "${GREEN}" "Copied PyTD3 modules"
    fi
    
    # Create README files in appropriate directories
    cat > "${build_dir}/lib/modules/4.14.108-ti-r144/extra/README.md" << EOF
# Hexapod Driver Kernel Module

This directory contains the hexapod_driver.ko kernel module for the Hexapod Robot Control System.

## Installation
The kernel module is automatically loaded during package installation.

## Manual Installation
\`\`\`bash
sudo insmod hexapod_driver.ko
\`\`\`

## Removal
\`\`\`bash
sudo rmmod hexapod_driver
\`\`\`

Version: ${version}
EOF
    
    cat > "${build_dir}/usr/local/bin/README.md" << EOF
# Hexapod Executables

This directory contains the main applications and test programs for the Hexapod Robot Control System.

## Main Application
- **hexapod_app**: Main hexapod control application

## Test Programs
- **test_mpu6050**: Test MPU6050 IMU sensor
- **test_adxl345**: Test ADXL345 accelerometer
- **test_servo**: Test servo motor control
- **test_movement**: Test basic movement patterns
- **test_balance**: Test balance control system
- **test_calibration**: Servo calibration utility
- **test_hcsr04**: Test ultrasonic sensor

## Usage
Run any program with:
\`\`\`bash
sudo ./program_name
\`\`\`

Version: ${version}
EOF
    
    cat > "${build_dir}/opt/hexapod/README.md" << EOF
# Hexapod Utilities

This directory contains utility scripts for the Hexapod Robot Control System.

## Available Scripts
- **install.sh**: Install/remove kernel module
- **monitor.sh**: Monitor system activities

## Usage
\`\`\`bash
# Install kernel module
sudo ./install.sh

# Remove kernel module  
sudo ./install.sh -r

# Monitor system
./monitor.sh
\`\`\`

Version: ${version}
EOF
    
    # Create main README
    cat > "${build_dir}/README.md" << EOF
# Hexapod Robot Control System

Version: ${version}

This package contains the complete Hexapod Robot Control System including:

- Kernel driver for hardware control
- User space applications and test programs
- Utility scripts for system management
- PyTD3 reinforcement learning modules (if available)

## Installation
This package has been installed using your system's package manager.

## Quick Start
1. The kernel module should be automatically loaded
2. Run the main application: \`sudo hexapod_app\`
3. Use test programs to verify functionality
4. Monitor system with: \`monitor.sh\`

## Documentation
For more information, see the README files in:
- /usr/local/bin/README.md (executables)
- /opt/hexapod/README.md (utilities)
- /lib/modules/\$(uname -r)/extra/README.md (kernel module)

## Support
For support and updates, visit: https://github.com/NguyenTrongPhuc552003/reinforcement-automative-hexapod.git
EOF
    
    # Set proper permissions for executables
    find "${build_dir}/usr/local/bin/" -type f -exec chmod 755 {} \;
    
    # Set consistent ownership for all files in the package
    if [ "$(id -u)" -eq 0 ]; then
        chown -R root:root "${build_dir}"
    fi
    
    log "${GREEN}" "Package structure created successfully"
}

# Initialize option flags
DO_CLEAN=0
DO_IMAGE=0
DO_BUILD=0
DO_UTILITY=0
DO_UML=0
DO_NO_CACHE=0
DO_SETUP_ENV=0
DO_PURGE=0
DO_GENERATE=0
IMAGE_TYPE="all" # Default to all images
BUILD_TYPE="all" # Default to all components
CLEAN_TYPE="all" # Default to all modules
PURGE_TYPE="all" # Default to all images
GENERATE_TYPE="deb" # Default to Debian package

# Parse command-line arguments
if [ $# -eq 0 ]; then
    # Default: build everything
    DO_IMAGE=1
    DO_BUILD=1
    BUILD_TYPE="all"
    DO_UTILITY=1
    DO_UML=1
    DO_SETUP_ENV=1
else
    i=0
    while [ $i -lt $# ]; do
        i=$((i+1))
        arg="${!i}"
        
        if [[ "$arg" == "--no-cache" || "$arg" == "-n" ]]; then
            DO_NO_CACHE=1
            DO_DOCKER_REQUIRED=1
        elif [[ "$arg" == "--help" || "$arg" == "-h" ]]; then
            usage
        elif [[ "$arg" == "--clean" || "$arg" == "-c" ]]; then
            DO_CLEAN=1
            # Check if next argument specifies a module to clean
            next_i=$((i+1))
            if [ $next_i -le $# ]; then
                next_arg="${!next_i}"
                if [[ "$next_arg" == "app" || \
                      "$next_arg" == "driver" || \
                      "$next_arg" == "pytd3" || \
                      "$next_arg" == "deploy" || \
                      "$next_arg" == "package" ]]; then
                    CLEAN_TYPE="$next_arg"
                    i=$next_i  # Skip the next argument since we've consumed it
                fi
            fi
        elif [[ "$arg" == "--purge" || "$arg" == "-p" ]]; then
            DO_PURGE=1
            # Check if next argument specifies a module to purge
            next_i=$((i+1))
            if [ $next_i -le $# ]; then
                next_arg="${!next_i}"
                if [[ "$next_arg" == "app" || "$next_arg" == "driver" || "$next_arg" == "pytd3" ]]; then
                    PURGE_TYPE="$next_arg"
                    i=$next_i  # Skip the next argument since we've consumed it
                fi
            fi
        elif [[ "$arg" == "--image" || "$arg" == "-i" ]]; then
            DO_IMAGE=1
            DO_DOCKER_REQUIRED=1
            # Check if next argument is an image type
            next_i=$((i+1))
            if [ $next_i -le $# ]; then
                next_arg="${!next_i}"
                if [[ "$next_arg" == "app" || "$next_arg" == "driver" || "$next_arg" == "pytd3" ]]; then
                    IMAGE_TYPE="$next_arg"
                    i=$next_i  # Skip the next argument since we've consumed it
                fi
            fi
        elif [[ "$arg" == "--build" || "$arg" == "-b" ]]; then
            DO_BUILD=1
            DO_DOCKER_REQUIRED=1
            # Check if next argument is a build type
            next_i=$((i+1))
            if [ $next_i -le $# ]; then
                next_arg="${!next_i}"
                if [[ "$next_arg" == "app" || "$next_arg" == "driver" || "$next_arg" == "pytd3" || "$next_arg" == "all" ]]; then
                    BUILD_TYPE="$next_arg"
                    i=$next_i  # Skip the next argument since we've consumed it
                fi
            fi
        elif [[ "$arg" == "--utility" || "$arg" == "-t"  ]]; then
            DO_UTILITY=1
        elif [[ "$arg" == "--setup" || "$arg" == "-s" ]]; then
            DO_SETUP_ENV=1
            DO_DOCKER_REQUIRED=1
        elif [[ "$arg" == "--uml" || "$arg" == "-l" ]]; then
            DO_UML=1
        elif [[ "$arg" == "--generate" || "$arg" == "-g" ]]; then
            DO_GENERATE=1
            # Check if next argument specifies a package type
            next_i=$((i+1))
            if [ $next_i -le $# ]; then
                next_arg="${!next_i}"
                if [[ "$next_arg" == "deb" || "$next_arg" == "tar.xz" || "$next_arg" == "rpm" ]]; then
                    GENERATE_TYPE="$next_arg"
                    i=$next_i  # Skip the next argument since we've consumed it
                fi
            fi
        elif [[ "$arg" == -* && "$arg" != "--"* ]]; then
            # Process combined short options like -tbl
            flags=${arg#-}
            for (( j=0; j<${#flags}; j++ )); do
                flag=${flags:$j:1}
                case "$flag" in
                    c) DO_CLEAN=1 ;;
                    p) DO_PURGE=1 ;;
                    i) DO_IMAGE=1; DO_DOCKER_REQUIRED=1 ;;
                    b) DO_BUILD=1; DO_DOCKER_REQUIRED=1 ;;
                    g) DO_GENERATE=1 ;;
                    t) DO_UTILITY=1 ;;
                    s) DO_SETUP_ENV=1; DO_DOCKER_REQUIRED=1 ;;
                    l) DO_UML=1 ;;
                    n) DO_NO_CACHE=1; DO_DOCKER_REQUIRED=1 ;;
                    h) usage ;;
                    *) 
                        log "${RED}" "Unknown option: -$flag"
                        usage
                        ;;
                esac
            done
        else
            log "${RED}" "Unknown argument: $arg"
            usage
        fi
    done
fi

# Function to clean app specific artifacts
clean_app() {
    local anything_cleaned=0
    
    # Check if app output directory exists
    if [ -d "${USER_SPACE_DIR}/bin" ]; then
        log "${GREEN}" "Cleaning user space binaries..."
        if ! rm -rf "${USER_SPACE_DIR}/bin" 2>/dev/null; then
            log "${YELLOW}" "Using sudo to clean user space binaries..."
            sudo rm -rf "${USER_SPACE_DIR}/bin"
        fi
        anything_cleaned=1
    else
        log "${YELLOW}" "No user space binaries found to clean"
    fi
    
    # Clean using app Docker image if available
    if command -v docker &> /dev/null && docker image inspect hexapod-app &> /dev/null; then
        log "${YELLOW}" "Cleaning user space applications using Docker..."
        docker run --rm \
            -v "${USER_SPACE_DIR}:/build/user" \
            hexapod-app clean || true
        anything_cleaned=1
    fi
    
    if [ $anything_cleaned -eq 0 ]; then
        log "${GREEN}" "Nothing to clean for app module"
    fi
}

# Function to clean driver specific artifacts
clean_driver() {
    local anything_cleaned=0
    
    # Check if driver build artifacts exist
    if [ -d "${KERNEL_MODULE_DIR}/obj" ] || [ -d "${KERNEL_MODULE_DIR}/cmd" ] || \
       [ -d "${KERNEL_MODULE_DIR}/deps" ] || [ -f "${KERNEL_MODULE_DIR}/hexapod_driver.ko" ]; then
        anything_cleaned=1
    fi
    
    # Clean using driver Docker image if available
    if command -v docker &> /dev/null && docker image inspect hexapod-driver &> /dev/null; then
        log "${YELLOW}" "Cleaning kernel modules using Docker..."
        docker run --rm \
            -v "${KERNEL_MODULE_DIR}:/build/module" \
            hexapod-driver clean || true
        anything_cleaned=1
    fi
    
    if [ $anything_cleaned -eq 0 ]; then
        log "${GREEN}" "Nothing to clean for driver module"
    fi
}

# Function to clean pytd3 specific artifacts
clean_pytd3() {
    local anything_cleaned=0
    
    # Check if pytd3 build artifacts exist
    if [ -d "${PYTD3_DIR}/build" ] || [ -d "${PYTD3_DIR}/venv" ]; then
        log "${GREEN}" "Cleaning PyTD3 build and virtual environment..."
        if ! rm -rf "${PYTD3_DIR}/build" "${PYTD3_DIR}/venv" 2>/dev/null; then
            log "${YELLOW}" "Using sudo to clean PyTD3 directories..."
            sudo rm -rf "${PYTD3_DIR}/build" "${PYTD3_DIR}/venv"
        fi
        anything_cleaned=1
    else
        log "${YELLOW}" "No PyTD3 build artifacts found to clean"
    fi
    
    # Clean using pytd3 Docker image if available
    if command -v docker &> /dev/null && docker image inspect hexapod-pytd3 &> /dev/null; then
        log "${YELLOW}" "Cleaning PyTD3 using Docker..."
        docker run --rm \
            -v "${PYTD3_DIR}:/build/pytd3" \
            hexapod-pytd3 clean || true
        anything_cleaned=1
    fi
    
    if [ $anything_cleaned -eq 0 ]; then
        log "${GREEN}" "Nothing to clean for PyTD3 module"
    fi
}

# Function to clean deploy directory
clean_deploy() {
    local anything_cleaned=0
    
    # Check if deploy directory exists
    if [ -d "${DEPLOY_DIR}" ]; then
        log "${YELLOW}" "Cleaning deploy directory..."
        if ! rm -rf "${DEPLOY_DIR}"/* 2>/dev/null; then
            log "${YELLOW}" "Using sudo to clean deploy directory..."
            sudo rm -rf "${DEPLOY_DIR}"/*
        fi
        anything_cleaned=1
    else
        log "${GREEN}" "Nothing to clean in deploy directory"
    fi
}

# Function to clean package directory
clean_package() {
    log "${YELLOW}" "Cleaning package directory..."
    
    # Remove all build_* directories
    local build_dirs=$(find "${PACKAGE_DIR}" -type d -name "build_*" 2>/dev/null)
    if [ -n "$build_dirs" ]; then
        log "${YELLOW}" "Removing package build directories..."
        for dir in $build_dirs; do
            if ! rm -rf "$dir" 2>/dev/null; then
                log "${YELLOW}" "Using sudo to remove directory: $dir"
                sudo rm -rf "$dir"
            fi
        done
        log "${GREEN}" "Package build directories removed"
    else
        log "${GREEN}" "No package build directories found"
    fi
    
    # Remove all hexapod_*.deb, hexapod_*.tar.xz, and hexapod_*.rpm files
    local package_files=$(find "${PACKAGE_DIR}" -type f -name "hexapod_*.*" -o -name "hexapodrobot_*.*" 2>/dev/null)
    if [ -n "$package_files" ]; then
        log "${YELLOW}" "Removing package files..."
        for file in $package_files; do
            if ! rm -f "$file" 2>/dev/null; then
                log "${YELLOW}" "Using sudo to remove file: $file"
                sudo rm -f "$file"
            fi
        done
        log "${GREEN}" "Package files removed"
    else
        log "${GREEN}" "No package files found"
    fi
    
    # Remove hexapod_latest.deb symlink if it exists
    if [ -L "${PACKAGE_DIR}/hexapod_latest.deb" ]; then
        log "${YELLOW}" "Removing latest package symlink..."
        if ! rm -f "${PACKAGE_DIR}/hexapod_latest.deb" 2>/dev/null; then
            sudo rm -f "${PACKAGE_DIR}/hexapod_latest.deb"
        fi
        log "${GREEN}" "Latest package symlink removed"
    fi
    
    log "${GREEN}" "Package directory cleaned successfully"
}

# Function to purge Docker images
purge_docker_images() {
    local image_type=$1
    local anything_purged=0
    
    # Check if Docker is available
    if ! command -v docker &> /dev/null; then
        log "${RED}" "Docker is not installed or not in PATH"
        return 0
    fi
    
    case "$image_type" in
        app)
            if docker image inspect hexapod-app &> /dev/null; then
                log "${YELLOW}" "Removing hexapod-app Docker image..."
                docker rmi hexapod-app || true
                anything_purged=1
            else
                log "${GREEN}" "No hexapod-app Docker image found to purge"
            fi
            ;;
        driver)
            if docker image inspect hexapod-driver &> /dev/null; then
                log "${YELLOW}" "Removing hexapod-driver Docker image..."
                docker rmi hexapod-driver || true
                anything_purged=1
            else
                log "${GREEN}" "No hexapod-driver Docker image found to purge"
            fi
            ;;
        pytd3)
            if docker image inspect hexapod-pytd3 &> /dev/null; then
                log "${YELLOW}" "Removing hexapod-pytd3 Docker image..."
                docker rmi hexapod-pytd3 || true
                anything_purged=1
            else
                log "${GREEN}" "No hexapod-pytd3 Docker image found to purge"
            fi
            ;;
        all)
            local purged_any=0
            if docker image inspect hexapod-app &> /dev/null; then
                log "${YELLOW}" "Removing hexapod-app Docker image..."
                docker rmi hexapod-app || true
                purged_any=1
            fi
            if docker image inspect hexapod-driver &> /dev/null; then
                log "${YELLOW}" "Removing hexapod-driver Docker image..."
                docker rmi hexapod-driver || true
                purged_any=1
            fi
            if docker image inspect hexapod-pytd3 &> /dev/null; then
                log "${YELLOW}" "Removing hexapod-pytd3 Docker image..."
                docker rmi hexapod-pytd3 || true
                purged_any=1
            fi
            
            if [ $purged_any -eq 0 ]; then
                log "${GREEN}" "No hexapod Docker images found to purge"
            else
                anything_purged=1
            fi
            ;;
    esac
    
    return $anything_purged
}

# Handle clean first (as it exits)
if [ $DO_CLEAN -eq 1 ]; then
    log "${YELLOW}" "Cleaning build artifacts..."
    
    case "$CLEAN_TYPE" in
        app)
            clean_app
            ;;
        driver)
            clean_driver
            ;;
        pytd3)
            clean_pytd3
            ;;
        deploy)
            clean_deploy
            ;;
        package)
            clean_package
            ;;
        all)
            # Clean all components
            clean_app
            clean_driver
            clean_pytd3
            clean_deploy
            clean_package
            
            # Clean UML diagrams without requiring Docker
            log "${YELLOW}" "Cleaning UML diagram files..."
            
            # Check if export.sh exists
            if [ -f "${PROJECT_ROOT}/scripts/export.sh" ]; then
                bash "${PROJECT_ROOT}/scripts/export.sh" -c
            else
                log "${RED}" "export.sh not found in scripts directory!"
            fi
            ;;
    esac
    
    log "${GREEN}" "Clean completed successfully!"
    exit 0
fi

# Handle purge option
if [ $DO_PURGE -eq 1 ]; then
    case "$PURGE_TYPE" in
        app|driver|pytd3)
            purge_docker_images "$PURGE_TYPE"
            ;;
        all)
            # Remove Docker containers, images, volumes, and builders
            log "${YELLOW}" "Checking for Docker containers to purge..."
            if docker ps -a | grep -q hexapod; then
                log "${YELLOW}" "Pruning all Docker containers..."
                docker container prune -f
            else
                log "${GREEN}" "No Docker containers found to purge"
            fi
            
            purge_docker_images "all"
            
            log "${YELLOW}" "Checking for Docker volumes to purge..."
            if docker volume ls | grep -q hexapod; then
                log "${YELLOW}" "Pruning all Docker volumes..."
                docker volume prune -f
            else
                log "${GREEN}" "No Docker volumes found to purge"
            fi
            
            log "${YELLOW}" "Pruning Docker builder cache..."
            docker builder prune --all -f
            ;;
    esac
    
    log "${GREEN}" "Purge completed!"
    exit 0
fi

# Components that don't require Docker can be built first
COMPONENTS_BUILT=0

# Build utility scripts (no Docker required)
if [ $DO_UTILITY -eq 1 ]; then
    build_utility_scripts
    log "${GREEN}" "Utility scripts created successfully!"
    COMPONENTS_BUILT=1
fi

# Build UML diagrams (no Docker required)
if [ $DO_UML -eq 1 ]; then
    build_uml_diagrams
fi

# Handle purge option
if [ $DO_PURGE -eq 1 ]; then
    # Remove Docker containers, images, volumes, and builders
    log "${YELLOW}" "Pruning all Docker containers..."
    sudo docker container prune -f
    log "${YELLOW}" "Pruning all Docker images..."
    sudo docker image prune -a -f
    log "${YELLOW}" "Pruning all Docker volumes..."
    sudo docker volume prune -f
    log "${YELLOW}" "Pruning all Docker builders..."
    sudo docker builder prune --all -f
    log "${GREEN}" "Purge completed!"
    exit 0
fi

# Build images if explicitly requested or needed for components
if [ $DO_IMAGE -eq 1 ]; then
    # Handle Docker image building with or without cache
    cache_flag=""
    if [ $DO_NO_CACHE -eq 1 ]; then
        cache_flag="--no-cache"
    fi
    build_docker_image "${IMAGE_TYPE}" "${cache_flag}"
    COMPONENTS_BUILT=1
fi

# Setup Python virtual environment if requested (must be before building PyTD3)
if [ $DO_SETUP_ENV -eq 1 ]; then
    setup_pytd3_env
    COMPONENTS_BUILT=1
fi

# Build Docker-dependent components using the unified -b/--build option
if [ $DO_BUILD -eq 1 ]; then
    case "$BUILD_TYPE" in
        app)
            build_user_space
            log "${GREEN}" "User space programs build completed successfully!"
            COMPONENTS_BUILT=1
            ;;
        driver)
            build_device_driver
            log "${GREEN}" "Kernel modules build completed successfully!"
            COMPONENTS_BUILT=1
            ;;
        pytd3)
            build_pytd3
            log "${GREEN}" "PyTD3 build completed successfully!"
            COMPONENTS_BUILT=1
            ;;
        all)
            # Build all components in order
            build_user_space
            log "${GREEN}" "User space programs build completed successfully!"
            
            build_device_driver
            log "${GREEN}" "Kernel modules build completed successfully!"
            
            build_pytd3
            log "${GREEN}" "PyTD3 build completed successfully!"
            
            COMPONENTS_BUILT=1
            ;;
        *)
            log "${RED}" "Unknown build type: ${BUILD_TYPE}"
            log "${YELLOW}" "Valid build types: app, driver, pytd3, all"
            exit 1
            ;;
    esac
fi

# Generate package if requested
if [ $DO_GENERATE -eq 1 ]; then
    log "${YELLOW}" "Starting package generation..."
    if generate_package "${GENERATE_TYPE}"; then
        log "${GREEN}" "Package generation completed successfully!"
        log "${GREEN}" "Package files created in: ${PROJECT_ROOT}/package/"
    else
        log "${RED}" "Package generation failed!"
        exit 1
    fi
fi

# Print overall success message if anything was built
if [ $COMPONENTS_BUILT -eq 1 ]; then
    log "${GREEN}" "Build completed successfully!"
    log "${GREEN}" "Deployment package created in: ${DEPLOY_DIR}"
fi