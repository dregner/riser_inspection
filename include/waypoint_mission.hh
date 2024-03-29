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
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <ignition/math/Pose3.hh>
#include <std_msgs/Float32.h>
// Message filter includes
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// Opencv includes
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Service from another node
#include <ros/service_client.h>
#include <std_srvs/SetBool.h>

// DJI SDK includes
#include <dji_osdk_ros/Activation.h>
#include <dji_osdk_ros//DroneTaskControl.h>
#include <dji_osdk_ros/SDKControlAuthority.h>
#include <dji_osdk_ros/MissionWpUpload.h>
#include <dji_osdk_ros/MissionWpAction.h>
#include <djiosdk/dji_vehicle.hpp>
#include <djiosdk/dji_telemetry.hpp>
#include <dji_osdk_ros/CameraAction.h>

//Riser inspection includes
#include <riser_inspection/askControl.h>
#include <riser_inspection/wpFolders.h>
#include <riser_inspection/StartMission.h>
#include <stereo_vant/PointGray.h>
//System includes
#include <string>
#include <experimental/filesystem>
#include <sys/stat.h>
#include <dirent.h>

//Class of Path Generate
#include <path_generator.hh>
#include <service_ack_type.hpp>


class WaypointControl {
private:
    ros::NodeHandle nh_;
    /// Filter to acquire same time GPS and RTK
    ros::Subscriber gps_sub_, rtk_sub_, attitude_sub_, height_above_takeoff_sub_;

    /// Message Filter callback
    message_filters::Subscriber<sensor_msgs::Image> img_sub_filter;
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub_filter;

    /// Foler and File services
    ros::ServiceServer ask_control_service;
    ros::ServiceServer wp_folders_service;
    ros::ServiceServer start_mission_service;

    /// DJI Services
    ros::ServiceClient drone_activation_service;
    ros::ServiceClient sdk_ctrl_authority_service;
    ros::ServiceClient waypoint_action_service;
    ros::ServiceClient waypoint_upload_service;
    ros::ServiceClient camera_action_service;
    ros::ServiceClient stereo_acquisition;

    /// Messages from GPS, RTK and Attitude
    // store old data to determine photo action
    sensor_msgs::NavSatFix current_gps;
    sensor_msgs::NavSatFix old_gps;
    std_msgs::Float32 old_height;

    // store current values of rtk, atti and gps
    sensor_msgs::NavSatFix current_rtk;
    geometry_msgs::QuaternionStamped current_atti;
    ignition::math::Quaterniond current_atti_euler;
    std_msgs::Float32::ConstPtr current_height;

    // store initial values when start mission service initilized
    sensor_msgs::NavSatFix start_gps_location;
    geometry_msgs::QuaternionStamped start_attitude;
    ignition::math::Quaterniond start_atti_eul;

    // List values
    PathGenerate pathGenerator;
     std::vector<std::vector<float>> waypoint_l;
    // Internal info
    bool use_rtk = false, doing_mission = false;
    int img_counter = 1, wp_counter = 1;
    int wait_time = 6;
    int voo = 0;
    double h_error, v_error;



public:
    WaypointControl();

    ~WaypointControl();

    void initSubscribers(ros::NodeHandle &nh);

    void initServices(ros::NodeHandle &nh);

    bool folders_serviceCB(riser_inspection::wpFolders::Request &req, riser_inspection::wpFolders::Response &res);

    bool ask_control(riser_inspection::askControl::Request &req, riser_inspection::askControl::Response &res);

    bool startMission_serviceCB(riser_inspection::StartMission::Request &req,
                                riser_inspection::StartMission::Response &res);
    void mission(const sensor_msgs::NavSatFix::ConstPtr &gps_msg, const std_msgs::Float32::ConstPtr &height_msg);

    bool take_photo();

    void gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msg);

    void rtk_callback(const sensor_msgs::NavSatFix::ConstPtr &msg);

    void atti_callback(const geometry_msgs::QuaternionStamped::ConstPtr &msg);

    void height_callback(const std_msgs::Float32::ConstPtr &msg);

    ServiceAck missionAction(DJI::OSDK::MISSION_ACTION action);

    ServiceAck initWaypointMission(dji_osdk_ros::MissionWaypointTask &waypointTask);

    bool askControlAuthority();

    std::vector<DJI::OSDK::WayPointSettings>
    createWayPoint(const std::vector<std::vector<std::string>> &csv_file, dji_osdk_ros::MissionWaypointTask &waypointTask);

    void uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings> &wp_list,
                         dji_osdk_ros::MissionWaypointTask &waypointTask);

    bool runWaypointMission(int responseTimeout);

    void setWaypointDefaults(DJI::OSDK::WayPointSettings *wp);

    static void setWaypointInitDefaults(dji_osdk_ros::MissionWaypointTask &waypointTask);
};

#endif // RISER_INSPECTION_H
