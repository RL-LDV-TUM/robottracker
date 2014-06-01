cmake_minimum_required(VERSION 2.8)
project(robottracker)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/obj)

SET(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/ )
find_package(aruco REQUIRED )

include_directories(inc)

add_library(tracker
  src/PlaygroundDetector.cpp
  src/Playground.cpp
  src/RobotTraffic.cpp
  src/TrafficServer.cpp
)

target_link_libraries(tracker ${aruco_LIBS})

add_executable(tracker_main src/tracker_main.cpp)
target_link_libraries(tracker_main  tracker)

add_executable(servertest_main src/servertest_main.cpp)
target_link_libraries(servertest_main  tracker)
