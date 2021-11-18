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
#include <geometry_msgs/QuaternionStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <ignition/math/Pose3.hh>

// Service from another node
#include <ros/service_client.h>
#include <std_srvs/SetBool.h>

// DJI SDK includes
#include <dji_sdk/Activation.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/MissionWpUpload.h>
#include <dji_sdk/MissionWpAction.h>
#include <djiosdk/dji_vehicle.hpp>
#include <djiosdk/dji_telemetry.hpp>
#include <dji_sdk/CameraAction.h>

//Riser inspection includes
#include <riser_inspection/askControl.h>
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
    /// Filter to acquire same time GPS and RTK
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub_;
    message_filters::Subscriber<sensor_msgs::NavSatFix> rtk_sub_;
    message_filters::Subscriber<geometry_msgs::QuaternionStamped> attitude_sub_;

    /// Foler and File services
    ros::ServiceServer ask_control_service;
    ros::ServiceServer wp_folders_service;
    ros::ServiceServer start_mission_service;


    /// DJI Services
    ros::ServiceClient drone_activation_service;
    ros::ServiceClient sdk_ctrl_authority_service;
    ros::ServiceClient waypoint_action_service;
    ros::ServiceClient waypoint_upload_service;

    /// Messages from GPS, RTK and Attitude
    sensor_msgs::NavSatFix current_gps_;
    sensor_msgs::NavSatFix current_rtk_;
    geometry_msgs::Quaternion current_atti_;
    ignition::math::Quaterniond current_atti_euler_;
    sensor_msgs::NavSatFix start_gnss_;
    geometry_msgs::Quaternion start_atti_;
    ignition::math::Quaterniond start_atti_eul;


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix,
            sensor_msgs::NavSatFix, geometry_msgs::QuaternionStamped> RiserInspectionPolicy;
    typedef message_filters::Synchronizer<RiserInspectionPolicy> Sync;
    boost::shared_ptr<Sync> sync_;

    PathGenerate pathGenerator;

    bool use_rtk;
public:
    RiserInspection();

    ~RiserInspection();

    void initSubscribers(ros::NodeHandle &nh);

    void initServices(ros::NodeHandle &nh);

    bool folders_serviceCB(riser_inspection::wpFolders::Request &req, riser_inspection::wpFolders::Response &res);

    bool ask_control(riser_inspection::askControl::Request &req, riser_inspection::askControl::Response &res);

    bool startMission_serviceCB(riser_inspection::wpStartMission::Request &req,
                                riser_inspection::wpStartMission::Response &res);

    void
    position_subscribeCB(const sensor_msgs::NavSatFixConstPtr &msg_gps, const sensor_msgs::NavSatFixConstPtr &msg_rtk,
                         const geometry_msgs::QuaternionStampedConstPtr &msg_att);

    ServiceAck missionAction(DJI::OSDK::DJI_MISSION_TYPE type,
                             DJI::OSDK::MISSION_ACTION action);

    ServiceAck initWaypointMission(dji_sdk::MissionWaypointTask &waypointTask);

    bool askControlAuthority();

    std::vector<DJI::OSDK::WayPointSettings>
    createWayPoint(std::vector<std::vector<std::string>> csv_file, dji_sdk::MissionWaypointTask &waypointTask);

    void uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings> &wp_list, int responseTimeout,
                         dji_sdk::MissionWaypointTask &waypointTask);

    bool runWaypointMission(int responseTimeout);

    void setWaypointDefaults(DJI::OSDK::WayPointSettings *wp);

    static void setWaypointInitDefaults(dji_sdk::MissionWaypointTask &waypointTask);

    std::vector<DJI::OSDK::WayPointSettings> DJI_waypoints(std::vector<std::vector<std::string>> wp_list);
};

#endif // RISER_INSPECTION_H