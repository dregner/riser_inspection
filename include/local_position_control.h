/** @file path_generator.h
 *  @author Daniel Regner
 *  @version 2.0
 *  @date Oct, 2021
 *
 *  @brief
 *  A simple local position control class to perform some tests.
 *
 *  @copyright 2021 VANT3D. All rights reserved.
 */
// ROS includes
#include <ros/service_server.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <ignition/math/Pose3.hh>
#include <std_msgs/Float32.h>
#include <tf/tf.h>

// Services
#include <ros/service_client.h>
#include <std_srvs/SetBool.h>
#include <iostream>
#include <riser_inspection/LocalPosition.h>
#include <riser_inspection/StartMission.h>
#include <stereo_vant/PointGray.h>
// DJI SDK includes
#include <dji_osdk_ros/FlightTaskControl.h>
#include <dji_osdk_ros/ObtainControlAuthority.h>
#include <dji_osdk_ros/SetLocalPosRef.h>
#include <dji_osdk_ros/CameraStartShootSinglePhoto.h>
#include <dji_osdk_ros/CameraRecordVideoAction.h>
#include <dji_osdk_ros/common_type.h>
#include <dji_osdk_ros/GimbalAction.h>


#include <path_generator.hh>

class LocalController {
private:
    ros::NodeHandle nh_;
    /// Filter to acquire same time GPS and RTK
    ros::Subscriber gps_sub, attitude_sub, local_pos_sub, height_sub, rtk_status;

    /// XYZ service
    ros::ServiceServer local_position_service;
    ros::ServiceServer start_mission_service;

    /// Stereo VANT3D
    ros::ServiceClient sv3d_client;

    /// DJI Services
    ros::ServiceClient obtain_crl_authority_client;
    ros::ServiceClient task_control_client;
    ros::ServiceClient camera_photo_client;
    ros::ServiceClient camera_record_video_client;
    ros::ServiceClient set_local_ref_client;
    ros::ServiceClient gimbal_control_client;

    /// Messages from GPS, RTK and Attitude
    sensor_msgs::NavSatFix current_gps;
    geometry_msgs::PointStamped current_local_pos;
    geometry_msgs::QuaternionStamped current_atti;
    ignition::math::Quaterniond current_atti_euler;
    tf::Matrix3x3 atti_matrix;
    tf::Quaternion q_atti_matrix;

    float rpa_height = 0;

    /// Internal references
    bool doing_mission = false;
    int wp_n = 1;
    double yaw_error=1, pos_error=0.1;
    bool use_gimbal = false;
    bool camera_gimbal = false;
    bool video_gimbal = false;
    bool use_stereo = false;
    int camera_count = 1;
    int stereo_count = 1;
    double init_heading = 0;

    PathGenerate pathGenerator;
    std::vector<std::vector<float>> waypoint_list;

public:
    LocalController();

    ~LocalController();

    void subscribing(ros::NodeHandle &nh);

    bool set_local_position();


    void local_position_callback(const geometry_msgs::PointStamped::ConstPtr &msg);

    void gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msg);

    void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr &msg);

    void height_callback(const std_msgs::Float32::ConstPtr &msg);

    bool local_pos_service_cb(riser_inspection::LocalPosition::Request &req,
                              riser_inspection::LocalPosition::Response &res);

    bool start_mission_service_cb(riser_inspection::StartMission::Request &req,
                                  riser_inspection::StartMission::Response &res);

    bool obtain_control(bool ask);

    void set_gimbal_angles(float roll, float pitch, float yaw);

    bool gimbal_camera(bool record_video);
    bool stereo_photo();

    bool local_position_ctrl(float xCmd, float yCmd, float zCmd, float yawCmd, float pos_thresh, float yaw_thresh);

    bool local_position_ctrl_mission();

    bool generate_WP(int csv_type);
};

#ifndef RISER_INSPECTION_LOCAL_POSITION_CONTROL_H
#define RISER_INSPECTION_LOCAL_POSITION_CONTROL_H

#endif //RISER_INSPECTION_LOCAL_POSITION_CONTROL_H

