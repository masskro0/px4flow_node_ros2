cmake_minimum_required(VERSION 3.5)
project(mavlink_serial_client)

add_compile_options(-DBOOST_ERROR_CODE_HEADER_ONLY)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px_comm REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system)
find_package(Eigen3 REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
)

###########
## Build ##
###########

include_directories(include
  ../../mavlink/include/mavlink/v1.0
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(mavlink_serial_client
  src/SerialComm.cc
  src/mavlink_serial_client.cc
)

ament_target_dependencies(mavlink_serial_client
  rclcpp
  std_msgs
  sensor_msgs
  image_transport
  px_comm
)

target_link_libraries(mavlink_serial_client
  Boost::thread
)

install(TARGETS mavlink_serial_client
  DESTINATION lib/mavlink_serial_client
)

install(
  DIRECTORY config/
  DESTINATION share/mavlink_serial_client
)

install(
  DIRECTORY launch/
  DESTINATION share/mavlink_serial_client
)

ament_package()
