cmake_minimum_required(VERSION 3.8)
project(reactive_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # Include this if your code depends on rclcpp
find_package(std_msgs REQUIRED)
find_package(cg_interfaces REQUIRED)

# Add the executable
add_executable(reactive_navigation include/reactive_navigation/reactive_navigation.cpp)
ament_target_dependencies(reactive_navigation rclcpp std_msgs cg_interfaces)   # Specify dependencies here

# Install the executable
install(TARGETS reactive_navigation
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
