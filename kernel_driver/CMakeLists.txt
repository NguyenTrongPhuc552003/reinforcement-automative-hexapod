cmake_minimum_required(VERSION 3.16)
project(hexapod_driver C)

# Set kernel module name
set(MODULE_NAME hexapod_main)

# Get current kernel version
execute_process(COMMAND uname -r OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE KERNEL_VERSION)

# Define kernel build directory
set(KERNEL_DIR /lib/modules/${KERNEL_VERSION}/build)

# Source directory
set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)

# Define the kernel module sources
set(SRC_FILES
    ${SRC_DIR}/hexapod_main.c
    ${SRC_DIR}/hexapod_utils.c
)

# Build output directory
set(BUILD_DIR ${CMAKE_BINARY_DIR})

# Add a custom target to build the kernel module
add_custom_target(build_module
    COMMAND make -C ${KERNEL_DIR} M=${SRC_DIR} modules
    WORKING_DIRECTORY ${SRC_DIR}
    COMMENT "Building kernel module"
)

# Add a custom target to clean the module
add_custom_target(clean_module
    COMMAND make -C ${KERNEL_DIR} M=${SRC_DIR} clean
    WORKING_DIRECTORY ${SRC_DIR}
    COMMENT "Cleaning kernel module"
)

# Install target (if needed)
install(FILES ${SRC_DIR}/${MODULE_NAME}.ko DESTINATION /lib/modules/${KERNEL_VERSION}/extra)


