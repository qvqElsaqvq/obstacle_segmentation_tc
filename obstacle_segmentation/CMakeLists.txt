cmake_minimum_required(VERSION 3.8)
project(obstacle_segmentation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)

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
add_executable(obstacle_segmentation_node src/obstacle_segmentation_node.cpp)
ament_target_dependencies(obstacle_segmentation_node rclcpp pcl_ros pcl_conversions sensor_msgs)
install(TARGETS
        obstacle_segmentation_node
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY yaml launch
        DESTINATION share/${PROJECT_NAME})
