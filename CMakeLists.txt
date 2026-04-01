cmake_minimum_required(VERSION 3.15 FATAL_ERROR)
project(yahboom_imu_ros2
  LANGUAGES C CXX
  VERSION 1.0.0
)

# Standard ROS 2 C++ settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    "$<$<CONFIG:Debug>:-Og;-g;-ggdb;-fno-eliminate-unused-debug-symbols>"
    "$<$<CONFIG:Release>:-O2;-Wl,-s>"
    -Wall -Wextra -march=native -flto
    -Wno-unused-parameter -Wno-reorder -Werror=return-type
    -fvisibility=hidden
    -Werror=missing-declarations -Werror=redundant-decls
  )
endif()

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)

# =====================================
# Create the Shared Library
# =====================================
add_library(${PROJECT_NAME} SHARED
  src/serial.cc
  src/yahboom_math.cc
  src/yahboom_driver.cc
)

# For ament to export headers properly, we use generator expressions
# to separate build-time includes from install-time includes.
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_definitions(${PROJECT_NAME}
  PUBLIC
  UNICODE _UNICODE $<$<CONFIG:Release>:-DNDEBUG>
)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
  Threads::Threads
)

# =====================================
# Create the Executable
# =====================================

# add non-ROS node
add_executable(driver_main
  src/imu_driver_main.cc
)
target_include_directories(driver_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_definitions(driver_main
  PUBLIC
  UNICODE _UNICODE $<$<CONFIG:Release>:-DNDEBUG>
)

target_link_libraries(driver_main
  PRIVATE
  ${PROJECT_NAME}
  Threads::Threads
)

# add ROS2 node
add_executable(yahboom_imu_node
  src/yahboom_imu_node.cc
)

target_include_directories(yahboom_imu_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(yahboom_imu_node
  ${PROJECT_NAME}
)
ament_target_dependencies(yahboom_imu_node rclcpp sensor_msgs nav_msgs)


# =====================================
# Install Rules (Crucial for ROS 2)
# =====================================
# 1. Install the library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 2. Install the headers
install(DIRECTORY include/
  DESTINATION include
)

# 3. Install the executable into the package-specific lib folder
install(
  TARGETS 
    driver_main yahboom_imu_node
  DESTINATION lib/${PROJECT_NAME}
)

# 4. Install utility scripts (optional, if you want your Python scripts available)
install(PROGRAMS
  scripts/imu_test.py
  scripts/imu_usb.py
  scripts/sniffer.py
  DESTINATION lib/${PROJECT_NAME}
)

# =====================================
# Testing
# =====================================
# ROS 2 uses BUILD_TESTING natively via CTest
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Add your GTest binaries
  # ament_add_gtest(test_circular_buffer test/test_circular_buffer.cpp)
  # target_link_libraries(test_circular_buffer ${PROJECT_NAME})

  # ament_add_gtest(test_serial test/test_serial.cc)
  # target_link_libraries(test_serial ${PROJECT_NAME})

  # ament_add_gtest(test_imu_driver test/test_imu_driver.cc)
  # target_link_libraries(test_imu_driver ${PROJECT_NAME})
endif()

# =====================================
# Ament Exports
# =====================================
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

# This must be the absolute last line
ament_package()