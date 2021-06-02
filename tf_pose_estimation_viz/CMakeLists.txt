cmake_minimum_required(VERSION 3.5)
project(tf_pose_estimation_viz)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf_pose_estimation_msgs REQUIRED)

set(dependencies
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    tf_pose_estimation_msgs
    visualization_msgs
)

include_directories(include)

add_executable(publish_markers src/publish_markers.cpp)
ament_target_dependencies(publish_markers ${dependencies} )

install(TARGETS
  publish_markers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})

ament_package()
