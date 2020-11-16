cmake_minimum_required(VERSION 3.5)
project(ros_roundtrip_latency_test)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
#find_package(roslib REQUIRED)
#find_package(roscpp REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/LatencyTestPayload.msg"
)

add_executable(ros_roundtrip_latency_test_p src/ros_roundtrip_latency_test.cpp)
target_include_directories(ros_roundtrip_latency_test_p PRIVATE ${roslib_INCLUDE_DIRS} ${roscpp_LIBRARIES} )
#target_link_libraries(ros_roundtrip_latency_test_p rclcpp)
ament_target_dependencies(ros_roundtrip_latency_test_p
  rclcpp Boost)
rosidl_target_interfaces(ros_roundtrip_latency_test_p ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS ros_roundtrip_latency_test_p DESTINATION bin)
