cmake_minimum_required(VERSION 3.10)
project(riser_inspection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)


find_package(ignition-math4)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS actionlib cv_bridge image_transport roscpp actionlib_msgs sensor_msgs
            std_msgs message_generation message_filters dji_osdk_ros)

find_package(Boost REQUIRED COMPONENTS system)

add_action_files(DIRECTORY action
        FILES PathGen.action)

add_service_files(DIRECTORY srv FILES WPgenerate.srv)

generate_messages(DEPENDENCIES std_msgs sensor_msgs nav_msgs actionlib_msgs)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES riser_inspection
        CATKIN_DEPENDS roscpp sensor_msgs std_msgs actionlib_msgs)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

#set(CMAKE_CXX_TARGET_INCLUDE_PATH "/usr/include/ignition/math2")

## Declare a C++ library
add_library(RiserInspectionLib src/path_generator.cpp ${catkin_LIBRARIES})
## Specify libraries to link a library or executable target against
target_link_libraries(RiserInspectionLib ${catkin_LIBRARIES} ignition-math4::ignition-math4)

add_executable(path_generator src/main.cpp include/path_generator.hh)
target_link_libraries(path_generator RiserInspectionLib ${catkin_LIBRARIES})

## Trying some action server and client to path generator
add_executable(pathgenerator_server src/pathgenerator_server.cpp)
target_link_libraries(pathgenerator_server ${catkin_LIBRARIES})
add_dependencies(pathgenerator_server ${riser_inspection_EXPORTED_TARGETS})