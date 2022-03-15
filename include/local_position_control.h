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
// DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <dji_sdk/CameraAction.h>
#include <dji_sdk/dji_sdk.h>

#define DEG2RAD(DEG) ((DEG) * ((3.141592653589793) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (3.141592653589793))
class LocalController {
private:
    ros::NodeHandle nh_;
    /// Filter to acquire same time GPS and RTK
    ros::Subscriber gps_sub, rtk_sub,  attitude_sub, local_pos_sub, height_sub;
    ros::Publisher ctrlPosYawPub, velocityPosYawPub;

    /// XYZ service
    ros::ServiceServer local_position_service;
    ros::ServiceServer local_velocity_service;
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
    sensor_msgs::NavSatFix start_gps;
    geometry_msgs::QuaternionStamped start_attitude;
    ignition::math::Quaterniond start_atti_eul;

    float target_offset_x;
    float target_offset_y;
    float target_offset_z;
    float target_yaw;
    float height;
    double yaw_value;

    /// Internal references
    bool use_rtk = false, doing_mission = false, first_time = true;
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

    bool local_pos_service_cb(riser_inspection::LocalPosition::Request &req, riser_inspection::LocalPosition::Response &res);

    bool local_velocity_cb(riser_inspection::LocalVelocity::Request &req, riser_inspection::LocalVelocity::Response &res);

    bool obtain_control(bool ask);

    bool local_position_velocity(float vx, float vy, float vz, float vyaw) ;

    void local_position_ctrl(double &xCmd, double &yCmd, double &zCmd, double &yawCmd);
};

#ifndef RISER_INSPECTION_LOCAL_POSITION_CONTROL_H
#define RISER_INSPECTION_LOCAL_POSITION_CONTROL_H

#endif //RISER_INSPECTION_LOCAL_POSITION_CONTROL_H
