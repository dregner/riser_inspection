/** @file path_generator.h
 *  @author Daniel Regner
 *  @version 2.0
 *  @date Oct, 2021
 *
 *  @brief
 *  An class used to generate and export to CSV waypoints based
 *  on semi-circular trajectory for photogrammetry inspection
 *
 *  @copyright 2021 VANT3D. All rights reserved.
 */
#ifndef RISER_INSPECTION_H
#define RISER_INSPECTION_H
// ROS includes
#include <ros/service_server.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <ignition/math/Pose3.hh>

// Service from another node
#include <ros/service_client.h>
#include <std_srvs/SetBool.h>

// DJI SDK includes
#include <dji_sdk/Activation.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <djiosdk/dji_vehicle.hpp>
#include <djiosdk/dji_telemetry.hpp>
#include <dji_sdk/CameraAction.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <dji_sdk/MissionWpUpload.h>
#include <dji_sdk/MissionWpAction.h>
//Riser inspection includes
#include <riser_inspection/wpStartMission.h>
#include <path_generator.hh>
#include <service_ack_type.hpp>
//System includes
#include <string>
#include <experimental/filesystem>
#include <sys/stat.h>
#include <dirent.h>


class RiserInspection {
private:
    ros::NodeHandle nh_;
    /// Filter to acquire same time GPS and RTK
    ros::Subscriber gps_sub;
    ros::Subscriber attitude_sub;
    ros::Subscriber local_pos_sub;
    ros::Publisher ctrlPosYawPub;

    /// XYZ service
    ros::ServiceServer local_position_service;
    /// DJI Services
    ros::ServiceClient drone_activation_service;
    ros::ServiceClient sdk_ctrl_authority_service;
    ros::ServiceClient camera_action_service;
    ros::ServiceClient set_local_pos_reference;
    ros::ServiceClient waypoint_action_service;
    ros::ServiceClient waypoint_upload_service;

    /// Messages from GPS, RTK and Attitude
    sensor_msgs::NavSatFix current_gps;
    sensor_msgs::NavSatFix old_gps;
    sensor_msgs::NavSatFix current_rtk;
    geometry_msgs::PointStamped current_local_pos;
    geometry_msgs::QuaternionStamped current_atti;
    ignition::math::Quaterniond current_atti_euler;
    sensor_msgs::NavSatFix start_gps_location;
    geometry_msgs::QuaternionStamped start_attitude;
    ignition::math::Quaterniond start_atti_eul;

    /// Local position targets
    float target_offset_x, target_offset_y, target_offset_z, target_yaw;

    /// Path Generator
    PathGenerate pathGenerator;
    std::vector<std::vector<float>> waypoint_l; // List of waypoints.
    int state;
    int mission_type;
    /// Internal references
    bool use_rtk = false, doing_mission = false;
public:
    RiserInspection();

    ~RiserInspection();

    void subscribing(ros::NodeHandle &nh);

    void set_waypoint_list(std::vector<std::vector<std::string>> vector, std::vector<float> initial_position);

    bool obtain_control(bool ask);



    /// Service and Topics Callback functions

    void local_position_callback(const geometry_msgs::PointStamped::ConstPtr &msg);

    void gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msg);

    void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr &msg);

    bool start_mision_callback(riser_inspection::wpStartMission::Request &req,
                               riser_inspection::wpStartMission::Response &res);

    /// Functions to local position control

    bool set_local_position();

    void setTarget(float x, float y, float z, float yaw);


    void local_position_ctrl(double &xCmd, double &yCmd, double &zCmd);

    /// Functions to waypoint mission control

    ServiceAck missionAction(DJI::OSDK::MISSION_ACTION action);

    ServiceAck initWaypointMission(dji_sdk::MissionWaypointTask &waypointTask);

    std::vector<DJI::OSDK::WayPointSettings>
    createWayPoint(const std::vector<std::vector<std::string>> &csv_file, dji_sdk::MissionWaypointTask &waypointTask);

    void uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings> &wp_list,
                         dji_sdk::MissionWaypointTask &waypointTask);

    bool runWaypointMission(int responseTimeout);

    void setWaypointDefaults(DJI::OSDK::WayPointSettings *wp);

    static void setWaypointInitDefaults(dji_sdk::MissionWaypointTask &waypointTask);

    bool askControlAuthority();

};

#endif // RISER_INSPECTION_H
