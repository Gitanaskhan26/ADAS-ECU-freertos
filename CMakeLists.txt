cmake_minimum_required(VERSION 3.16)
project(ADAS_ECU VERSION 1.0.0 LANGUAGES C CXX)

# C++17 standard (embedded subset)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags for embedded safety
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-exceptions -fno-rtti")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

# Coverage flags (enable with -DCMAKE_BUILD_TYPE=Coverage)
set(CMAKE_CXX_FLAGS_COVERAGE "${CMAKE_CXX_FLAGS_DEBUG} --coverage -fprofile-arcs -ftest-coverage")
set(CMAKE_C_FLAGS_COVERAGE "${CMAKE_C_FLAGS_DEBUG} --coverage -fprofile-arcs -ftest-coverage")
set(CMAKE_EXE_LINKER_FLAGS_COVERAGE "${CMAKE_EXE_LINKER_FLAGS_DEBUG} --coverage")

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)

# Find Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# FreeRTOS configuration
set(FREERTOS_DIR "/opt/FreeRTOS-Kernel" CACHE PATH "FreeRTOS source directory")
set(FREERTOS_PORT_DIR "${FREERTOS_DIR}/portable/ThirdParty/GCC/Posix")

# FreeRTOS include directories
include_directories(
    ${FREERTOS_DIR}/include
    ${FREERTOS_PORT_DIR}
    ${FREERTOS_PORT_DIR}/utils
    ${CMAKE_SOURCE_DIR}/include  # For FreeRTOSConfig.h
)

# FreeRTOS source files
set(FREERTOS_SOURCES
    ${FREERTOS_DIR}/tasks.c
    ${FREERTOS_DIR}/queue.c
    ${FREERTOS_DIR}/list.c
    ${FREERTOS_DIR}/timers.c
    ${FREERTOS_DIR}/event_groups.c
    ${FREERTOS_DIR}/stream_buffer.c
    ${FREERTOS_DIR}/portable/MemMang/heap_3.c
    ${FREERTOS_PORT_DIR}/port.c
    ${FREERTOS_PORT_DIR}/utils/wait_for_event.c
)

# Source files
set(SOURCES
    src/main.cpp
    src/task_sensor.cpp
    src/task_compute.cpp
    src/task_watchdog.cpp
    src/kalman.cpp
    src/can_interface.cpp
    src/safety.cpp
    src/collision_detection.cpp
    src/data_logger.cpp
    src/multi_object_tracker.cpp
    src/path_planner.cpp
    src/autosar_services.cpp
    src/can_fd_interface.cpp
    src/hil_test_harness.cpp
    ${FREERTOS_SOURCES}
)

# Executable target
add_executable(adas_ecu ${SOURCES})

# Link libraries
target_link_libraries(adas_ecu
    Eigen3::Eigen
    pthread
    rt
)

# Install target
install(TARGETS adas_ecu DESTINATION bin)

# Testing (optional)
enable_testing()
add_subdirectory(tests EXCLUDE_FROM_ALL)

# Generate compile_commands.json for IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
