cmake_minimum_required(VERSION 3.5)
project(road_navigate_cpp)

set(lib_name ${PROJECT_NAME}_core)

# === Environment ===

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencies ===

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(OpenCV REQUIRED)

set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    pluginlib)

# === Build ===

add_library(${lib_name} SHARED
            src/lane_costmap.cpp)
include_directories(include
                    ${OpenCV_INCLUDE_DIRS}) # Include OpenCV directories after finding the package


# === Installation ===

install(TARGETS ${lib_name}
        DESTINATION lib)

# === Ament work ===

# pluginlib_export_plugin_description_file() installs lane_costmap.xml
# file into "share" directory and sets ament indexes for it.
# This allows the plugin to be discovered as a plugin of required type.
pluginlib_export_plugin_description_file(nav2_costmap_2d lane_costmap.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
target_link_libraries(${lib_name} ${OpenCV_LIBS}) # Link OpenCV libraries
ament_package()