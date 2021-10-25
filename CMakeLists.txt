cmake_minimum_required(VERSION 3.10)
project(riser_inspection)

find_package(ignition-math4)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS actionlib cv_bridge
        image_transport roscpp rospy actionlib_msgs sensor_msgs
        std_msgs message_generation message_filters nav_msgs dji_sdk)

find_package(DJIOSDK REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

## Compile as C++11, supported in ROS Kinetic and newer
#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 14)


find_package(OpenCV 3 REQUIRED)
if (OpenCV_FOUND)
    message(STATUS "Found OpenCV ${OpenCV_VERSION} installed in the system")
    message(STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
    include_directories( ${OpenCV_INCLUDE_DIRS})
endif()


find_package(CUDA QUIET)
if(CUDA_FOUND)
    message(STATUS "Found ${CUDA_VERSION} CUDA installed in the system, will use it for depth perception sample")
    message(STATUS " - Includes: ${CUDA_INCLUDE_DIRS}")
    add_definitions(-DUSE_GPU)
elseif()
    message(STATUS "Did not find CUDA in the system")
endif()

################################################
## Declare ROS messages, services and actions ##
################################################


add_service_files(FILES wpGenerate.srv wpFolders.srv wpStartMission.srv)

generate_messages(DEPENDENCIES std_msgs sensor_msgs nav_msgs actionlib_msgs)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES riser_inspection
        CATKIN_DEPENDS roscpp sensor_msgs std_msgs actionlib_msgs dji_sdk dji_sdk_node)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${DJIOSDK_INCLUDE_DIRS}
        ${dji_sdk_node_INCLUDE_DIRS})

set(CMAKE_CXX_TARGET_INCLUDE_PATH "/usr/include/ignition/math4")

## Declare a C++ library
## Specify libraries to link a library or executable target against
add_executable(path_generator src/path/main_path_generator.cpp src/path/path_generator.cpp)
target_link_libraries(path_generator ${catkin_LIBRARIES} )

add_executable(read_file src/read_file_test.cpp src/path/path_generator.cpp )
target_link_libraries(read_file ${catkin_LIBRARIES} )

add_executable(stereo_disparity src/stereo/stereo_disparity.cpp)
target_link_libraries(stereo_disparity ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(stereo_calib src/stereo/stereo_calib.cpp)
target_link_libraries(stereo_calib ${OpenCV_LIBS})

add_executable(riser_inspection_node src/ros/ros_path_planning_node.cpp src/ros/ros_path_planning_generate.cpp src/path/path_generator.cpp)
target_link_libraries(riser_inspection_node ${catkin_LIBRARIES} ${dji_sdk_LIBRARIES} ${DJIOSDK_LIBRARIES} ignition-math4::ignition-math4)

## Trying some action server and client to path generator
#add_executable(pathgenerator_server src/pathgenerator_server.cpp)
#target_link_libraries(pathgenerator_server ${catkin_LIBRARIES})
#add_dependencies(pathgenerator_server ${riser_inspection_EXPORTED_TARGETS})