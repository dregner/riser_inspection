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
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <std_msgs/UInt8.h>
#include <ignition/math/Pose3.hh>
#include <tf/tf.h>

/// DJI SDK includes
// DJI ROS services msgs
#include <dji_sdk/Activation.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/MissionWpUpload.h>
#include <dji_sdk/MissionWpAction.h>
#include <dji_sdk/CameraAction.h>
// DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>
// DJI OSDK classes
#include <djiosdk/dji_vehicle.hpp>
#include <djiosdk/dji_telemetry.hpp>
#include "dji_sdk/dji_sdk.h"

//Riser inspection includes
#include <riser_inspection/wpGenerate.h>
#include <riser_inspection/wpFolders.h>
#include <riser_inspection/wpStartMission.h>

//System includes
#include <string>
#include <experimental/filesystem>
#include <sys/stat.h>
#include <dirent.h>

//Class of Path Generate
#include <path_generator.hh>
#include <service_ack_type.hpp>



class RiserInspection {
private:
    ros::NodeHandle nh_;

    int inbound_counter;
    int outbound_counter;
    int break_counter;

    float target_offset_x;
    float target_offset_y;
    float target_offset_z;
    float target_yaw;
    uint8_t flight_status_ = 255;
    uint8_t display_mode_  = 255;

    sensor_msgs::NavSatFix start_gps_location;
    geometry_msgs::Point start_local_position;

    /// Subscribers
    ros::Subscriber gps_sub_;
    ros::Subscriber rtk_sub_;
    ros::Subscriber attitude_sub_;
    /// Publishers
    ros::Publisher ctrlPosYawPub;
    /// Foler and File services
    ros::ServiceServer generate_pathway_srv_;
    ros::ServiceServer wp_folders_srv_;
    ros::ServiceServer start_mission_srv_;


    /// DJI Services
    ros::ServiceClient drone_activation_service;
    ros::ServiceClient waypoint_action_service;
    ros::ServiceClient waypoint_upload_service;
    ros::ServiceClient take_photo_service;

    ros::ServiceClient set_local_pos_reference;
    ros::ServiceClient sdk_ctrl_authority_service;
    ros::ServiceClient drone_task_service;
    ros::ServiceClient query_version_service;

    /// Messages from GPS, RTK and Attitude
    //Start
    sensor_msgs::NavSatFix start_gps_location_;
    geometry_msgs::Point start_local_position_;
    //Current
    sensor_msgs::NavSatFix current_gps_;
    sensor_msgs::NavSatFix current_rtk_;
    geometry_msgs::Quaternion current_atti_;
    geometry_msgs::Point current_local_pos_;


    ignition::math::Quaterniond atti_Eul;



    /// Initial position to waypoint creates
    double lat0_ = -27.605299;  // Starting latitude
    double lon0_ = -48.520547;  // Starting longitude
    float alt0_ = 3;              // Starting altitude
    float head0_ = 30;            // Starting heading

    std::ofstream saved_wp_;

    PathGenerate pathGenerator;

    bool firstTime;
    bool node_start_ = true; // a variable used to indentify the start of node to get initial position
public:
    RiserInspection();

    ~RiserInspection();

    void initSubscribers(ros::NodeHandle &nh);

    void initServices(ros::NodeHandle &nh);

    /// ROS Service CALLBACKS

    bool folders_serviceCB(riser_inspection::wpFolders::Request &req, riser_inspection::wpFolders::Response &res);

    bool pathGen_serviceCB(riser_inspection::wpGenerate::Request &req, riser_inspection::wpGenerate::Response &res);

    bool startMission_serviceCB(riser_inspection::wpStartMission::Request &req,
                                riser_inspection::wpStartMission::Response &res);



    /// ROS Topic subscription CALLBACKS
    void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg);

    void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);

    void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);

    void rtk_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);

    void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);

    void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);

    /// Functions from demo_flight_controller
    void localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                                  sensor_msgs::NavSatFix& target,
                                  sensor_msgs::NavSatFix& origin);

    bool set_local_position();

    bool obtain_control();

    void step();

    bool takeoff_land(int task);

    bool monitoredTakeoff();

    /// Function to mission waypoint

//    bool askControlAuthority();

    ServiceAck missionAction(DJI::OSDK::DJI_MISSION_TYPE type,
                             DJI::OSDK::MISSION_ACTION action);

    ServiceAck initWaypointMission(dji_sdk::MissionWaypointTask &waypointTask);

    std::vector<DJI::OSDK::WayPointSettings>
    createWayPoint(const std::vector<std::vector<std::string>> csv_file, dji_sdk::MissionWaypointTask &waypointTask);

    void uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings> &wp_list, int responseTimeout,
                         dji_sdk::MissionWaypointTask &waypointTask);

    bool runWaypointMission(int responseTimeout);

    void setWaypointDefaults(DJI::OSDK::WayPointSettings *wp);

    static void setWaypointInitDefaults(dji_sdk::MissionWaypointTask &waypointTask);

    std::vector<DJI::OSDK::WayPointSettings> DJI_waypoints(std::vector<std::vector<std::string>> wp_list);
};

#endif // RISER_INSPECTION_H