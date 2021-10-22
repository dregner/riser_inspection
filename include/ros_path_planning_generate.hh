//
// Created by regner on 06/09/2021.
//

// ROS includes
#include <ros/service_server.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

// DJI SDK includes
#include <dji_sdk/Activation.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/MissionWpUpload.h>
#include <dji_sdk/MissionWpAction.h>

//Riser inspection includes
#include <riser_inspection/wpGenerate.h>
#include <riser_inspection/wpFolders.h>

//System includes
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdlib>
#include <experimental/filesystem>
#include <sys/stat.h>
#include <dirent.h>

//Class of Path Generate
#include <path_generator.hh>

typedef struct ServiceAck
{
    bool         result;
    int          cmd_set;
    int          cmd_id;
    unsigned int ack_data;
    ServiceAck(bool res, int set, int id, unsigned int ack)
            : result(res)
            , cmd_set(set)
            , cmd_id(id)
            , ack_data(ack)
    {
    }
    ServiceAck()
    {
    }
} ServiceAck;

class RiserInspection {
private:
    /**
     * ROS STUFF
    * */
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_position_sub_;
    message_filters::Subscriber<sensor_msgs::NavSatFix> rtk_position_sub_;
    ros::ServiceServer generate_pathway_srv_;
    ros::ServiceServer wp_folders_srv;

    ros::ServiceClient     drone_activation_service;
    ros::ServiceClient     sdk_ctrl_authority_service;
    ros::ServiceClient     drone_task_service;


    sensor_msgs::NavSatFix ptr_gps_position_;
    sensor_msgs::NavSatFix ptr_rtk_position_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix,
            sensor_msgs::NavSatFix> RiserInspectionPolicy;
    typedef message_filters::Synchronizer<RiserInspectionPolicy> Sync;
    boost::shared_ptr<Sync> sync_;


    /// Initial position to waypoint creates
    // TODO: Must come as initialize parameters
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

    bool askControlAuthority();

    ServiceAck activate();

    ServiceAck obtainCtrlAuthority();

    ServiceAck takeoff();

    void initSubscribers(ros::NodeHandle &nh);

    void initServices(ros::NodeHandle &nh);

    bool Folders_serviceCB(riser_inspection::wpFolders::Request &req, riser_inspection::wpFolders::Response &res);

    bool PathGen_serviceCB(riser_inspection::wpGenerate::Request &req, riser_inspection::wpGenerate::Response &res);

    inline bool exists(const std::string &name);

    void get_gps_position(const sensor_msgs::NavSatFixConstPtr &msg_gps, const sensor_msgs::NavSatFixConstPtr &msg_rtk);

    std::vector<std::pair<std::string, std::vector<float>>> read_csv(std::string filename);

};
