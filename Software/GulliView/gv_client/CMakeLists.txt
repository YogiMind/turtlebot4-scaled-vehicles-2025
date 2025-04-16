cmake_minimum_required(VERSION 3.5)
project(gv_client)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 1) Register your Python code as an installable package
ament_python_install_package(gv_client_node
  PACKAGE_DIR gv_client_node
)

# 2) Generate ROS 2 messages
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/LaptopSpeed.msg
  msg/GulliViewPosition.msg
  DEPENDENCIES std_msgs
)

# 3) Copy launch files to Install
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)
install(
  DIRECTORY gv_client_node
  DESTINATION lib/${PROJECT_NAME}
)

# 4) Export message runtime dependency
ament_export_dependencies(rosidl_default_runtime)

ament_package()
