cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(joystick_control SHARED joystick_control.cpp joystick.cpp servos.cpp)
target_link_libraries(joystick_control ${GAZEBO_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

target_link_libraries(joystick_control ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
