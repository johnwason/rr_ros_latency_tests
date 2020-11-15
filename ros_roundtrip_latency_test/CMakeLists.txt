cmake_minimum_required(VERSION 2.8.3)
project(ros_roundtrip_latency_test)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED)
find_package(genmsg REQUIRED)
find_package(roslib REQUIRED)
find_package(roscpp REQUIRED)

add_message_files(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} FILES msg/LatencyTestPayload.msg)

generate_messages(DEPENDENCIES)

add_executable(ros_roundtrip_latency_test src/ros_roundtrip_latency_test.cpp)
target_include_directories(ros_roundtrip_latency_test PRIVATE ${roslib_INCLUDE_DIRS} ${roscpp_LIBRARIES})
target_link_libraries(ros_roundtrip_latency_test ${roslib_LIBRARIES} ${roscpp_LIBRARIES})
add_dependencies(ros_roundtrip_latency_test ${PROJECT_NAME}_generate_messages)

install(TARGETS ros_roundtrip_latency_test DESTINATION bin)
