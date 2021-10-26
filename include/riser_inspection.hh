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

// DJI SDK includes
#include <dji_sdk/Activation.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/MissionWpUpload.h>
#include <dji_sdk/MissionWpAction.h>
#include <djiosdk/dji_vehicle.hpp>
#include <djiosdk/dji_telemetry.hpp>


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

typedef struct ServiceAck {
    bool result;
    int cmd_set;
    int cmd_id;
    unsigned int ack_data;

    ServiceAck(bool res, int set, int id, unsigned int ack)
            : result(res), cmd_set(set), cmd_id(id), ack_data(ack) {
    }

    ServiceAck() {
    }
} ServiceAck;

class RiserInspection {
private:
    ros::NodeHandle nh_;
    /// Filter to acquire same time GPS and RTK
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub_;
    message_filters::Subscriber<sensor_msgs::NavSatFix> rtk_sub_;
    message_filters::Subscriber<geometry_msgs::QuaternionStamped> attitude_sub_;

    /// Foler and File services
    ros::ServiceServer generate_pathway_srv_;
    ros::ServiceServer wp_folders_srv_;
    ros::ServiceServer start_mission_srv_;

    /// DJI Services
    ros::ServiceClient drone_activation_service;
    ros::ServiceClient sdk_ctrl_authority_service;
    ros::ServiceClient drone_task_service;
    ros::ServiceClient waypoint_action_service;
    ros::ServiceClient waypoint_upload_service;

    /// Messages from GPS, RTK and Attitude
    sensor_msgs::NavSatFix ptr_gps_position_;
    sensor_msgs::NavSatFix ptr_rtk_position_;
    geometry_msgs::QuaternionStamped ptr_attitude_;
    ignition::math::Quaterniond drone_rpy_;


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix,
            sensor_msgs::NavSatFix, geometry_msgs::QuaternionStamped> RiserInspectionPolicy;
    typedef message_filters::Synchronizer<RiserInspectionPolicy> Sync;
    boost::shared_ptr<Sync> sync_;


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

    bool folders_serviceCB(riser_inspection::wpFolders::Request &req, riser_inspection::wpFolders::Response &res);

    bool pathGen_serviceCB(riser_inspection::wpGenerate::Request &req, riser_inspection::wpGenerate::Response &res);

    bool startMission_serviceCB(riser_inspection::wpStartMission::Request &req,
                                riser_inspection::wpStartMission::Response &res);

    void
    position_subscribeCB(const sensor_msgs::NavSatFixConstPtr &msg_gps, const sensor_msgs::NavSatFixConstPtr &msg_rtk,
                         const geometry_msgs::QuaternionStampedConstPtr &msg_att);


    ServiceAck missionAction(DJI::OSDK::DJI_MISSION_TYPE type,
                             DJI::OSDK::MISSION_ACTION action);

    ServiceAck initWaypointMission(dji_sdk::MissionWaypointTask &waypointTask);

    ServiceAck activate();

    ServiceAck obtainCtrlAuthority();

    bool askControlAuthority();

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