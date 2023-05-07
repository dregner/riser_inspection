cmake_minimum_required(VERSION 2.8)
project(riser_inspection)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Wextra")
# Add module path for <name>.cmake conf files
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cv_bridge
        roscpp rospy sensor_msgs
        message_generation message_filters stereo_vant dji_osdk_ros darknet_ros_msgs)
find_package(ignition-math4)
find_package(DJIOSDK REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV 3 QUIET)
if (OpenCV_FOUND)
    message(STATUS "Found OpenCV ${OpenCV_VERSION} installed in the system")
    message(STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
    add_definitions(-DOPEN_CV_INSTALLED)
    set(OPEN_CV_3_3_0_INSTALLED TRUE)

else ()
    find_package(OpenCV QUIET)
    if (OpenCV_FOUND)
        message(STATUS "Found OpenCV ${OpenCV_VERSION} installed in the system")
        message(STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
        add_definitions(-DOPEN_CV_INSTALLED)
    else ()
        message(STATUS "Did not find OpenCV in the system")
    endif ()
endif ()



find_package(CUDA QUIET)
if (CUDA_FOUND)
    message(STATUS "Found ${CUDA_VERSION} CUDA installed in the system")
    message(STATUS " - Includes: ${CUDA_INCLUDE_DIRS}")
    add_definitions(-DUSE_GPU)
elseif ()
    message(STATUS "Did not find CUDA in the system")
endif ()


## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 14)

################################################
## Declare ROS messages, services and actions ##
################################################


add_service_files(FILES StartMission.srv LocalPosition.srv CameraSetting.srv)

generate_messages(DEPENDENCIES std_msgs sensor_msgs nav_msgs actionlib_msgs)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp sensor_msgs std_msgs dji_osdk_ros)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${DJIOSDK_INCLUDE_DIRS})
#${dji_sdk_node_INCLUDE_DIRS})

set(CMAKE_CXX_TARGET_INCLUDE_PATH "/usr/include/ignition/math4")
install(DIRECTORY include DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY srv     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srv)
## Declare a C++ library
## Specify libraries to link a library or executable target against
add_executable(path_generator src/path/create_path.cpp src/path/path_generator.cpp)
target_link_libraries(path_generator ${catkin_LIBRARIES})

add_executable(read_file src/read_file_test.cpp src/path/path_generator.cpp)
target_link_libraries(read_file ${catkin_LIBRARIES})

add_executable(stereo_disparity src/stereo/stereo_disparity.cpp)
target_link_libraries(stereo_disparity ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES} ${OpenCV_LIBS})

add_executable(name_image src/stereo/change_name.cpp)
target_link_libraries(name_image ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES} ${OpenCV_LIBS})

add_executable(change_txt src/stereo/change_text.cpp)
target_link_libraries(change_txt ${catkin_LIBRARIES})

add_executable(local_controller_node src/ros/local_controller_node.cpp src/ros/local_position_control.cpp src/path/path_generator.cpp)
target_link_libraries(local_controller_node ${catkin_LIBRARIES} ${dji_sdk_LIBRARIES} ${DJIOSDK_LIBRARIES} ignition-math4::ignition-math4)

add_executable(camera_setting_node src/ros/dji_camera_setting_node.cpp src/ros/dji_camera_setting.cpp)
target_link_libraries(camera_setting_node ${catkin_LIBRARIES} ${dji_sdk_LIBRARIES} ${DJIOSDK_LIBRARIES})

add_executable(gps_atti src/ros/sensors/read_gps_atti.cpp)
target_link_libraries(gps_atti ${catkin_LIBRARIES} ${dji_sdk_LIBRARIES} ignition-math4::ignition-math4)

add_executable(save_gps_atti src/ros/sensors/save_gps_atti.cpp)
target_link_libraries(save_gps_atti ${catkin_LIBRARIES} ${dji_sdk_LIBRARIES} ignition-math4::ignition-math4)

add_executable(save_positions src/ros/sensors/save_gps_local_atti.cpp)
target_link_libraries(save_positions ${catkin_LIBRARIES} ${dji_sdk_LIBRARIES} ignition-math4::ignition-math4)

add_executable(save_battery src/ros/sensors/save_battery.cpp)
target_link_libraries(save_battery ${catkin_LIBRARIES} ${dji_sdk_LIBRARIES})

add_executable(save_vel src/ros/sensors/save_velocities.cpp)
target_link_libraries(save_vel ${catkin_LIBRARIES} ${dji_sdk_LIBRARIES} ignition-math4::ignition-math4)

add_executable(gps_rtk_atti src/ros/sensors/read_gps_rtk_atti.cpp)
target_link_libraries(gps_rtk_atti ${catkin_LIBRARIES} ${dji_sdk_LIBRARIES} ignition-math4::ignition-math4)

add_executable(position_sensor src/ros/sensors/read_gps_local_atti.cpp)
target_link_libraries(position_sensor ${catkin_LIBRARIES} ${dji_sdk_LIBRARIES} ignition-math4::ignition-math4)

add_executable(save_disp_zed src/ros/sensors/save_disp_gps_atti.cpp)
target_link_libraries(save_disp_zed ${catkin_LIBRARIES} ${dji_sdk_LIBRARIES} ignition-math4::ignition-math4)

add_executable(show_disp src/ros/sensors/show_disp.cpp)
target_link_libraries(show_disp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${dji_sdk_LIBRARIES} ignition-math4::ignition-math4)

add_executable(darknet_disparity_node src/ros/darknet_disparity_node.cpp src/ros/darknet_disparity.cpp)
target_link_libraries(darknet_disparity_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(m210_stereo_rect_depth src/stereo/m210_stereo_vga.cpp src/stereo/stereo_utility/camera_param.cpp src/stereo/stereo_utility/config.cpp src/stereo/stereo_utility/stereo_frame.cpp)
target_link_libraries(m210_stereo_rect_depth ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES} ${OpenCV_LIBS})

add_executable(vga_rosservice src/ros/stereo_vga_subscription.cpp)
target_link_libraries(vga_rosservice ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})

install(TARGETS local_controller_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

