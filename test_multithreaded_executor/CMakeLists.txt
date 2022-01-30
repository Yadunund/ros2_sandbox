cmake_minimum_required(VERSION 3.8)
project(test_multithreaded_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

################################################################################
add_executable(
  test_executor
  src/main.cpp
)

################################################################################
ament_target_dependencies(
  test_executor
  rclcpp
  std_msgs
)

################################################################################
install(
  TARGETS test_executor
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
