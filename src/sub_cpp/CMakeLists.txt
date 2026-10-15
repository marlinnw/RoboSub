cmake_minimum_required(VERSION 3.8)
project(sub_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Compile src/depth_filter.cpp into an executable program named depth_filter
add_executable(depth_filter src/depth_filter.cpp)

# Tell CMake which ROS packages that program ruses. This ass their header
# folders (so #include "rclcpp/rclcpp.hpp" is found) and links their libraries.
# Every package listed here also needs a find_package(... REQUIRED) line above.
ament_target_dependencies(depth_filter rclcpp std_msgs)

# Copy the compiled program to install/sub_cpp/lib/sub_cpp/,
# which is exactly where 'ros2 run sub_cpp depth_filter' looks for it
install(TARGETS depth_filter 
          DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
