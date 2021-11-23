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
#include <ignition/math/Pose3.hh>
#include <tf/tf.h>

// DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <dji_sdk/CameraAction.h>
#include <dji_sdk/dji_sdk.h>
// DJI OSDK classes
#include <djiosdk/dji_vehicle.hpp>
#include <djiosdk/dji_telemetry.hpp>

//Riser inspection includes
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

    /// node Subscribers
    ros::Subscriber gpsSub;
    ros::Subscriber rtkSub;
    ros::Subscriber attiSub;
    /// node Publishers
    ros::Publisher ctrlPosYawPub;
    ros::Publisher ctrlBrakePub;
    /// node Services
    ros::ServiceServer wp_folders_service;
    ros::ServiceServer start_mission_service;
    /// DJI Services Clients
    ros::ServiceClient drone_activation_service;
    ros::ServiceClient set_local_pos_reference;
    ros::ServiceClient sdk_ctrl_authority_service;
    ros::ServiceClient drone_task_service;
    ros::ServiceClient camera_action_service;
    /// Messages from GPS, RTK and Attitude
    //Start
    sensor_msgs::NavSatFix start_gps_location;
    geometry_msgs::Quaternion start_atti_;
    ignition::math::Quaterniond start_atti_eul; // Transfor Atti Quaternion to Euler Angles
    geometry_msgs::Point start_local_position_;
    //Current
    sensor_msgs::NavSatFix current_gps_;
    sensor_msgs::NavSatFix current_rtk_;
    geometry_msgs::Quaternion current_atti_;
    ignition::math::Quaterniond atti_Eul; // Transfor Atti Quaternion to Euler Angles
    geometry_msgs::Point current_local_pos_;
    /// Class to create and save Waypoints
    PathGenerate pathGenerator;
    std::vector<std::vector<std::string>> waypoint_list;
    /// Bool parameters
    bool use_rtk = false; // Default: using GPS signal
    bool startMission = false; // Default: to not start mission
    bool finished = false; // Default: false
    int  state = 0; // Specify state of waypoints

public:
    RiserInspection();

    ~RiserInspection();

    void initSubscribers(ros::NodeHandle &nh);

    void initServices(ros::NodeHandle &nh);

    /// ROS Service CALLBACKS
    bool folders_serviceCB(riser_inspection::wpFolders::Request &req, riser_inspection::wpFolders::Response &res);

    bool startMission_serviceCB(riser_inspection::wpStartMission::Request &req,
                                riser_inspection::wpStartMission::Response &res);

    /// ROS Topic subscription CALLBACKS

    void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);

    void rtk_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);

    void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);

    void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);

    /// Functions from demo_flight_controller
    void localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                                  sensor_msgs::NavSatFix& target,
                                  sensor_msgs::NavSatFix& origin);

    bool set_local_position();

    bool askControlAuthority();

    void step();

    void setTarget(float x, float y, float z, float yaw);

    /// Function to mission waypoint
};

#endif // RISER_INSPECTION_H
