cmake_minimum_required(VERSION 3.16)

# Test executable for Kalman filter
add_executable(test_kalman test_kalman.cpp ../src/kalman.cpp)
target_link_libraries(test_kalman Eigen3::Eigen)
target_include_directories(test_kalman PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Test executable for CAN interface
add_executable(test_can_interface test_can_interface.cpp ../src/can_interface.cpp)
target_include_directories(test_can_interface PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Test executable for safety functions
add_executable(test_safety test_safety.cpp ../src/safety.cpp ../src/can_interface.cpp)
target_link_libraries(test_safety pthread rt)
target_include_directories(test_safety PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Test executable for collision detection
add_executable(test_collision_detection test_collision_detection.cpp ../src/collision_detection.cpp ../src/kalman.cpp)
target_link_libraries(test_collision_detection Eigen3::Eigen)
target_include_directories(test_collision_detection PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Test executable for multi-object tracker
add_executable(test_multi_object_tracker test_multi_object_tracker.cpp ../src/multi_object_tracker.cpp ../src/kalman.cpp)
target_link_libraries(test_multi_object_tracker Eigen3::Eigen)
target_include_directories(test_multi_object_tracker PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Test executable for path planner
add_executable(test_path_planner test_path_planner.cpp ../src/path_planner.cpp ../src/multi_object_tracker.cpp ../src/kalman.cpp)
target_link_libraries(test_path_planner Eigen3::Eigen)
target_include_directories(test_path_planner PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Test executable for CAN FD interface
add_executable(test_can_fd_interface test_can_fd_interface.cpp ../src/can_fd_interface.cpp)
target_include_directories(test_can_fd_interface PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Test executable for AUTOSAR services
add_executable(test_autosar_services test_autosar_services.cpp ../src/autosar_services.cpp ../src/multi_object_tracker.cpp ../src/path_planner.cpp ../src/kalman.cpp)
target_link_libraries(test_autosar_services Eigen3::Eigen)
target_include_directories(test_autosar_services PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Add tests to CTest
add_test(NAME KalmanFilterTest COMMAND test_kalman)
add_test(NAME CANInterfaceTest COMMAND test_can_interface)
add_test(NAME SafetyTest COMMAND test_safety)
add_test(NAME CollisionDetectionTest COMMAND test_collision_detection)
add_test(NAME MultiObjectTrackerTest COMMAND test_multi_object_tracker)
add_test(NAME PathPlannerTest COMMAND test_path_planner)
add_test(NAME CANFDInterfaceTest COMMAND test_can_fd_interface)
add_test(NAME AUTOSARServicesTest COMMAND test_autosar_services)

