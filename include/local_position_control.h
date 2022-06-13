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

// Opencv includes
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// Services
#include <ros/service_client.h>
#include <std_srvs/SetBool.h>
#include <iostream>
#include <riser_inspection/LocalPosition.h>
#include <riser_inspection/LocalVelocity.h>
#include <riser_inspection/wpStartMission.h>
// DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <dji_sdk/CameraAction.h>
#include <dji_sdk/dji_sdk.h>

#include <path_generator.hh>

#define DEG2RAD(DEG) ((DEG) * ((3.141592653589793) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (3.141592653589793))

class LocalController {
private:
    ros::NodeHandle nh_;
    /// Filter to acquire same time GPS and RTK
    ros::Subscriber gps_sub, rtk_sub, attitude_sub, local_pos_sub, height_sub;
    ros::Publisher ctrlPosYawPub, velocityPosYawPub;

    /// XYZ service
    ros::ServiceServer local_position_service;
    ros::ServiceServer local_velocity_service;
    ros::ServiceServer start_mission_service;

    /// DJI Services
    ros::ServiceClient sdk_ctrl_authority_service;
    ros::ServiceClient camera_action_service;
    ros::ServiceClient set_local_pos_reference;

    /// Messages from GPS, RTK and Attitude
    sensor_msgs::NavSatFix current_gps;
    sensor_msgs::NavSatFix current_rtk;
    geometry_msgs::PointStamped current_local_pos;
    geometry_msgs::QuaternionStamped current_atti;
    ignition::math::Quaterniond current_atti_euler;

    float target_offset[4];
    float rpa_height;

    /// Internal references
    bool use_rtk = false, doing_mission = false, first_time = true;
    bool use_wp_list = false;
    int wp_n = 1;
    double v_error, h_error;

    PathGenerate pathGenerator;
    std::vector<std::vector<float>> waypoint_l;

public:
    LocalController();

    ~LocalController();

    void subscribing(ros::NodeHandle &nh);

    bool set_local_position();

    void setTarget(float x, float y, float z, float yaw);

    void local_position_callback(const geometry_msgs::PointStamped::ConstPtr &msg);

    void gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msg);

    void rtk_callback(const sensor_msgs::NavSatFix::ConstPtr &msg);

    void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr &msg);

    void height_callback(const std_msgs::Float32::ConstPtr &msg);

    bool local_pos_service_cb(riser_inspection::LocalPosition::Request &req,
                              riser_inspection::LocalPosition::Response &res);

    bool local_velocity_service_cb(riser_inspection::LocalVelocity::Request &req,
                                   riser_inspection::LocalVelocity::Response &res);

    bool start_mission_service_cb(riser_inspection::wpStartMission::Request &req,
                                  riser_inspection::wpStartMission::Response &res);

    bool obtain_control(bool ask);

    bool local_position_velocity(float vx, float vy, float vz, float vyaw);

    void local_position_ctrl(double &xCmd, double &yCmd, double &zCmd, double &yawCmd);

    void local_position_ctrl_mission(double &xCmd, double &yCmd, double &zCmd, double &yawCmd, int waypoint_n);

    void elapse_control_mission(bool mission);

    void elapse_control(bool mission);

    bool generate_WP();


};

#ifndef RISER_INSPECTION_LOCAL_POSITION_CONTROL_H
#define RISER_INSPECTION_LOCAL_POSITION_CONTROL_H

#endif //RISER_INSPECTION_LOCAL_POSITION_CONTROL_H
