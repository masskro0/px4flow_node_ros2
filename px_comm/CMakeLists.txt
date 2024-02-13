cmake_minimum_required(VERSION 3.8)
project(px_comm)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
)

set(msg_files
  "msg/CameraInfo.msg"
  "msg/Mavlink.msg"
  "msg/OpticalFlow.msg"
)

set(srv_files
  "srv/SetCameraInfo.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

ament_export_dependencies(
  rosidl_default_runtime
  geometry_msgs
  std_msgs
)

ament_package()