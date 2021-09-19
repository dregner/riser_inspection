//
// Created by regner on 06/09/2021.
//
#include <ros/service_server.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <riser_inspection/wpGenerate.h>
#include <riser_inspection/wpFolders.h>

#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdlib>



#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))

class PathGenerate {
private:
    /// topic subscription
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_position_sub_;
    message_filters::Subscriber<sensor_msgs::NavSatFix> rtk_position_sub_;
    ros::ServiceServer generate_pathway_srv_;
    ros::ServiceServer wp_folders_srv;
    std::string file_path_ = "/home/regner/Documents";
    std::string file_name_ ="wp_generate.csv";

    sensor_msgs::NavSatFixConstPtr ptr_gps_position_;
    sensor_msgs::NavSatFixConstPtr ptr_rtk_position_;
    /// Riser distance
    double riser_distance_ = 7;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix,
            sensor_msgs::NavSatFix> PathGeneratePolicy;
    typedef message_filters::Synchronizer<PathGeneratePolicy> Sync;
    boost::shared_ptr<Sync> sync_;


    /// Arrays used to store waypoints and initial values
    float waypointTaskDJI_[8]; // Store DJI waypoint Task parameters
    double cart_array_[6]; // [x y z dz dy dz]' [6x1]
    double coord_array_[5]; // [x y z dz dy dz]' [6x1]

    /// Initial position to waypoint creates
    double lat0_ = -27.605299; // Starting latitude
    double lon0_ = -48.520547; // Starting longitude
    int alt0_ = 3; // Starting altitude
    int head0_ = 30; // Starting heading
    bool first_time = true;
    int rotz_[3][3];
    std::ofstream saved_wp_;
public:
    PathGenerate();

    ~PathGenerate();

    void initSubscribers(ros::NodeHandle &nh);

    void initServices(ros::NodeHandle &nh);

    bool Folders_serviceCB(riser_inspection::wpFolders::Request &req, riser_inspection::wpFolders::Response &res);

    bool PathGen_serviceCB(riser_inspection::wpGenerate::Request &req, riser_inspection::wpGenerate::Response &res);

    inline bool exists (const std::string& name);

    void setInitCoord(double lon, double lat, int alt, int head);

    void setDJIwaypointTask(float velocity_range, float idle_velocity, int action_on_finish,
                            int mission_exec_times, int yaw_mode, int trace_mode,
                            int action_on_rc_lost, int gimbal_pitch_mode);

    void get_gps_position(const sensor_msgs::NavSatFixConstPtr &msg_gps, const sensor_msgs::NavSatFixConstPtr &msg_rtk);

    void rotz_cartP(int yaw);

    void print_wp(double *wp_array, int size, int n);

    void csv_save_wp(double *wp_array, int row);

    /** @param cart_wp Array of 6 elements provided from cartesian [x y z dz dy dz]'
      * @return coord_wp - Lat, lon, alt, roll (north heading) and pitch (gimbal) [Nx5] */

    /** @param cart_wp Array of 6 elements provided from cartesian [x y z dz dy dz]'
         @return coord_wp - Lat, lon, alt, roll (north heading) and pitch (gimbal) [Nx5] */

    void pointCartToCord(double cart_wp[6], int nCount);

    /** @param phi  Riser diameter
        @param d    Distance to riser wall
        @param da   Angle delta. Degrees
        @param nh   Number of acquisitions on same height
        @param dv   Delta height (altitude or Z)
        @param nv   Number of acquisitions levels
        @return Format: [x y z dz dy dz]' [6x1]  */

    void createInspectionPoints(const double phi, const float d, const float da,
                                const float nh, const float dv, const float nv);
};