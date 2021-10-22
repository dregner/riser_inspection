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


//Riser inspection includes
#include <riser_inspection/wpGenerate.h>
#include <riser_inspection/wpFolders.h>
//System includes
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdlib>

#include <path_generator.hh>
#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))

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
public:
    RiserInspection();

    ~RiserInspection();

    void initSubscribers(ros::NodeHandle &nh);

    void initServices(ros::NodeHandle &nh);

    bool Folders_serviceCB(riser_inspection::wpFolders::Request &req, riser_inspection::wpFolders::Response &res);

    bool PathGen_serviceCB(riser_inspection::wpGenerate::Request &req, riser_inspection::wpGenerate::Response &res);

    inline bool exists(const std::string &name);

    void get_gps_position(const sensor_msgs::NavSatFixConstPtr &msg_gps, const sensor_msgs::NavSatFixConstPtr &msg_rtk);

    std::vector<std::pair<std::string, std::vector<float>>> read_csv(std::string filename);

};
