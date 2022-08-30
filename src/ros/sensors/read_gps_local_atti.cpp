//
// Created by vant3d on 09/12/2021.
//

#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <ignition/math/Pose3.hh>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#define RAD2DEG(RAD) ((RAD) * 180 / M_PI)

void callback(const sensor_msgs::NavSatFix::ConstPtr &gps_msg,
              const geometry_msgs::QuaternionStamped::ConstPtr &atti_msg,
              const geometry_msgs::PointStamped::ConstPtr &local_msg) {

    ignition::math::Quaterniond rpy;
    rpy.Set(atti_msg->quaternion.w, atti_msg->quaternion.x, atti_msg->quaternion.y, atti_msg->quaternion.z);

    std::cout << "R: " << RAD2DEG(rpy.Roll()) << "\tP: " << RAD2DEG(rpy.Pitch()) << "\tY: " << RAD2DEG(rpy.Yaw())
              << std::endl;
    std::cout << "GPS" << std::endl;
    std::cout << "LAT: " << gps_msg->latitude << "\tLON: " << gps_msg->longitude << "\tALT: " << gps_msg->altitude
              << std::endl;
    std::cout << "LOCAL POS" << std::endl;
    std::cout << "X: " << local_msg->point.x << "\tY: " << local_msg->point.y << "\tZ: "
              << local_msg->point.z << std::endl;
    std::cout << "\033[2J\033[1;1H";     // clear terminal

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "gps_atti_local_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::NavSatFix> gps(nh, "/dji_osdk_ros/gps_position", 1);
    message_filters::Subscriber<geometry_msgs::QuaternionStamped> atti(nh, "/dji_osdk_ros/attitude", 1);
    message_filters::Subscriber<geometry_msgs::PointStamped> local(nh, "/dji_osdk_ros/local_position", 1);


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, geometry_msgs::QuaternionStamped, geometry_msgs::PointStamped> MySyncPolicy;
    // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), gps, atti, local);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));

    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
