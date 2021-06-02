cmake_minimum_required(VERSION 3.5)
project(tf_pose_estimation_3d)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf_pose_estimation_msgs REQUIRED)
find_package(gb_perception_utils REQUIRED)

set(dependencies
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    pcl_ros
    tf_pose_estimation_msgs
    gb_perception_utils
)

include_directories(include)

add_executable(pixel_2_point src/pixel_2_point.cpp)
ament_target_dependencies(pixel_2_point ${dependencies} )

install(TARGETS
  pixel_2_point
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(${dependencies})

ament_package()
