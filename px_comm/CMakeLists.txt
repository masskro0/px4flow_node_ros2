cmake_minimum_required(VERSION 3.8)
project(px_comm)

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



# if RTI DDS is found
#if(DEFINED ENV{NDDSHOME})
#  catkin_package(
#    INCLUDE_DIRS include
#    LIBRARIES px_comm_dds
#    CATKIN_DEPENDS message_runtime
#  )

#  set(NDDSHOME $ENV{NDDSHOME})

#  include_directories(
#    include
#    include/px_comm
#    ${NDDSHOME}/include
#    ${NDDSHOME}/include/ndds
#  )

#  add_definitions(-DRTI_UNIX)

#  add_library(px_comm_dds
#    dds_msg/DDSImage.cxx
#    dds_msg/DDSImagePlugin.cxx
#    dds_msg/DDSImageSupport.cxx
#  )
#else()
#  catkin_package(
#    INCLUDE_DIRS include
#    CATKIN_DEPENDS message_runtime
#  )
#endif()
