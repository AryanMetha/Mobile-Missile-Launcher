cmake_minimum_required(VERSION 3.5)
project(mycarsim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(pid_steer_publisher src/pid_steer_publisher.cpp)
ament_target_dependencies(pid_steer_publisher rclcpp geometry_msgs)



install(TARGETS pid_steer_publisher DESTINATION lib/${PROJECT_NAME})


# Install launch files
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/)

# Install models
install(DIRECTORY models
    DESTINATION share/${PROJECT_NAME}/)

ament_package()
