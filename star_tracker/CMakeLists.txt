cmake_minimum_required(VERSION 3.8)
project(star_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  star_tracker/star_tracker_node.py
  star_tracker/gps_interface.py
  star_tracker/bno055_interface.py
  star_tracker/alignment_calibration.py
  star_tracker/test_framework.py
  star_tracker/integration_tests.py
  star_tracker/mock_providers.py
  star_tracker/gps_provider.py
  star_tracker/imu_provider.py
  star_tracker/arm_emulator.py
  star_tracker/test_astropy_validation.py
  star_tracker/validate_ros2.py
  star_tracker/validate_tests.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()