cmake_minimum_required(VERSION 3.5)
project(bounding_box_detection)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    scripts/bounding_box_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
