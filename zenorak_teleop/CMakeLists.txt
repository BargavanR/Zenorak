cmake_minimum_required(VERSION 3.8)
# This sets the minimum required version of CMake to 3.8.
# ROS 2 generally requires at least CMake 3.5+, but 3.8+ is safe.

project(zenorak_teleop)
# Name of your ROS 2 package/project. Used by CMake and ROS 2 to identify the package.

# --- Compiler options ---
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # If using GCC or Clang compilers, add extra warning flags
  add_compile_options(-Wall -Wextra -Wpedantic)
  # -Wall: show all warnings
  # -Wextra: show extra warnings
  # -Wpedantic: enforce strict ISO C++ standard
endif()

# --- Find ROS 2 dependencies ---
find_package(ament_cmake REQUIRED)
# Required to build any ROS 2 package using ament build system

find_package(rclpy REQUIRED)
# Required to use ROS 2 Python client library

find_package(std_msgs REQUIRED)
# Required to use standard ROS 2 message types (like std_msgs/String)

# --- Install Python scripts ---

install(
  PROGRAMS
    scripts/zenorak_teleop.py  # Path to your teleop Python script
    scripts/joy_debug.py
    scripts/joy_teleop.py 
  DESTINATION lib/${PROJECT_NAME}  # ROS 2 expects executables here
)
# --- Install launch files (optional) ---
install(
  DIRECTORY launch  # Path to a folder containing launch files
  DESTINATION share/${PROJECT_NAME}  # Installed into package share directory
)

# --- Testing & linting (optional) ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Automatically find linters (Python, C++, etc.)

  # Skip copyright check (useful if your project has no license header yet)
  set(ament_cmake_copyright_FOUND TRUE)

  # Skip cpplint check (only works if repo is a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)

  # Find dependencies needed for running tests
  ament_lint_auto_find_test_dependencies()
endif()

# --- Declare this directory as a ROS 2 package ---
ament_package()
