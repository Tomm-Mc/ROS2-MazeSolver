cmake_minimum_required(VERSION 3.5)

project(ros_gz_example_bringup)

find_package(ament_cmake REQUIRED)
find_package(ros_gz_example_description REQUIRED)
find_package(ros_gz_example_gazebo REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install project launch files
install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install project configuration files
install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  PROGRAMS
    launch/lidar_subscriber.py  # Ruta actual del archivo dentro de "launch"
  DESTINATION lib/${PROJECT_NAME}  # Directorio donde se instalará
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
