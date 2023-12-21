cmake_minimum_required(VERSION 3.5)
project(px4flow)

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(px_comm REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system)

###########
## Build ##
###########

include_directories(include
  ../../mavlink/include/mavlink/v1.0
)

add_executable(${PROJECT_NAME}
  src/SerialComm.cc
  src/px4flow_node.cc
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    std_msgs
    sensor_msgs
    image_transport
    px_comm
)

target_link_libraries(${PROJECT_NAME}
  Boost::thread
)

#add_dependencies(px4flow_node px_comm_gencpp)

#############
## Install ##
#############

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
