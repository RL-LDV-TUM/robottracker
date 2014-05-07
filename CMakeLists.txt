cmake_minimum_required(VERSION 2.8)
project(robottracker)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/obj)

SET(CMAKE_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/lib/cmake/ )
find_package(aruco REQUIRED )

add_library(tracker
  src/Playground.cpp
)

target_link_libraries(tracker ${aruco_LIBS})

add_executable(tracker_main src/tracker_main.cpp)
target_link_libraries(tracker_main  tracker)
