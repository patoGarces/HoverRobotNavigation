cmake_minimum_required(VERSION 3.8)
project(disparity_to_laserscan_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(stereo_msgs REQUIRED)

add_executable(disparity_to_laserscan
  src/disparity_to_laserscan.cpp
)

ament_target_dependencies(disparity_to_laserscan
  rclcpp
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
  stereo_msgs
)

install(TARGETS
  disparity_to_laserscan
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
