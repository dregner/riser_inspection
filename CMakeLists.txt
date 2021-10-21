cmake_minimum_required(VERSION 3.10)
project(riser_inspection)

## Compile as C++11, supported in ROS Kinetic and newer
#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 14)



find_package(ignition-math4)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS actionlib cv_bridge image_transport roscpp rospy actionlib_msgs sensor_msgs
        std_msgs message_generation message_filters)
find_package(dji_sdk REQUIRED)
include_directories(${dji_sdk_INCLUDE_DIRS})
include_directories(${dji_sdk_DIRS})

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


find_package(Boost REQUIRED COMPONENTS system)

add_action_files(DIRECTORY action FILES PathGen.action)

add_service_files(DIRECTORY srv FILES wpGenerate.srv wpFolders.srv)

generate_messages(DEPENDENCIES std_msgs sensor_msgs nav_msgs actionlib_msgs)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES riser_inspection
        CATKIN_DEPENDS roscpp sensor_msgs std_msgs actionlib_msgs)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_library(RiserPathGenLIB src/path/path_generator.cpp ${catkin_LIBRARIES})
add_library(RiserPathGenROSLIB src/ros/ros_path_planning_generate.cpp ${catkin_LIBRARIES})

#set(CMAKE_CXX_TARGET_INCLUDE_PATH "/usr/include/ignition/math2")

## Declare a C++ library
## Specify libraries to link a library or executable target against
add_executable(path_generator src/path/path_generator.cpp)
target_link_libraries(path_generator RiserPathGenLIB ${catkin_LIBRARIES} ignition-math4::ignition-math4)

#add_executable(ros_pathGen src/ros_pathGen.cpp)
#target_link_libraries(ros_pathGen RiserPathGenROSLIB ${catkin_LIBRARIES} ignition-math4::ignition-math4)

add_executable(stereo_disparity src/stereo/stereo_disparity.cpp)
target_link_libraries(stereo_disparity ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(stereo_calib src/stereo/stereo_calib.cpp)
target_link_libraries(stereo_calib ${OpenCV_LIBS})

add_executable(riser_inspection_node src/ros/ros_path_planning_node.cpp )
target_link_libraries(riser_inspection_node RiserPathGenROSLIB RiserPathGenLIB ${catkin_LIBRARIES} ${dji_sdk_LIBRARIES} ignition-math4::ignition-math4)

## Trying some action server and client to path generator
#add_executable(pathgenerator_server src/pathgenerator_server.cpp)
#target_link_libraries(pathgenerator_server ${catkin_LIBRARIES})
#add_dependencies(pathgenerator_server ${riser_inspection_EXPORTED_TARGETS})