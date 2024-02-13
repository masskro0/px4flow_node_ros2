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
find_package(Boost REQUIRED COMPONENTS thread system)
find_package(Eigen3 REQUIRED)
find_package(image_transport REQUIRED)
find_package(px_comm REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
)

include_directories(include
  ../../mavlink/include/mavlink/v1.0
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/SerialComm.cc
  src/mavlink_serial_client.cc
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  image_transport
  px_comm
  sensor_msgs
  std_msgs
)

target_link_libraries(${PROJECT_NAME}
  Boost::thread
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()