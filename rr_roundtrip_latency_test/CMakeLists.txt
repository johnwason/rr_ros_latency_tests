cmake_minimum_required(VERSION 2.8.12)
project(rr_roundtrip_latency_test)

set(CMAKE_CXX_STANDARD 11)

find_package(RobotRaconteur REQUIRED)

include_directories(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

ROBOTRACONTEUR_GENERATE_THUNK(RR_THUNK_SRCS RR_THUNK_HDRS 
	experimental.latency_test.robdef
	MASTER_HEADER
	)

add_executable(rr_roundtrip_latency_test
	${RR_THUNK_SRCS}
	rr_roundtrip_latency_test.cpp)

target_link_libraries(rr_roundtrip_latency_test RobotRaconteurCore )

