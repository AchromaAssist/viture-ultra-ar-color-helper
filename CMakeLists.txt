cmake_minimum_required(VERSION 3.16)
project(viture_ar_demo VERSION 0.1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# Export compile commands for IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Platform detection
message(STATUS "System processor: ${CMAKE_SYSTEM_PROCESSOR}")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    set(ARCH "aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    set(ARCH "x86_64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

message(STATUS "Using architecture: ${ARCH}")

# Paths
set(XRDRIVER_DIR ${CMAKE_SOURCE_DIR}/external/XRLinuxDriver)
set(XRDRIVER_INCLUDE ${XRDRIVER_DIR}/include)
set(VITURE_LIB_DIR ${XRDRIVER_DIR}/lib/${ARCH}/viture)

# Check for required directories
if(NOT EXISTS ${XRDRIVER_INCLUDE})
    message(FATAL_ERROR "XRLinuxDriver include directory not found: ${XRDRIVER_INCLUDE}")
endif()

if(NOT EXISTS ${VITURE_LIB_DIR})
    message(FATAL_ERROR "VITURE library directory not found: ${VITURE_LIB_DIR}")
endif()

# Find required packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

# Find VITURE libraries
file(GLOB VITURE_LIBS "${VITURE_LIB_DIR}/*.so")
message(STATUS "Found VITURE libraries: ${VITURE_LIBS}")

# Define the executable
add_executable(${PROJECT_NAME}
    src/main.c
)

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    ${XRDRIVER_INCLUDE}
    ${LIBUSB_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(${PROJECT_NAME} PRIVATE
    ${VITURE_LIBS}
    ${LIBUSB_LIBRARIES}
    pthread
    m
)

# Set RPATH for finding shared libraries at runtime
set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_RPATH "${VITURE_LIB_DIR}"
    INSTALL_RPATH "${VITURE_LIB_DIR}"
)

# Compiler flags
target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)

# Define VITURE_SUPPORTED
target_compile_definitions(${PROJECT_NAME} PRIVATE
    VITURE_SUPPORTED
)

# Installation
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
)

# Print configuration summary
message(STATUS "")
message(STATUS "Configuration Summary:")
message(STATUS "  Architecture:     ${ARCH}")
message(STATUS "  VITURE lib dir:   ${VITURE_LIB_DIR}")
message(STATUS "  libusb includes:  ${LIBUSB_INCLUDE_DIRS}")
message(STATUS "")
