//
// Created by vant3d on 09/12/2021.
//

#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <ros/ros.h>
#include <ignition/math/Pose3.hh>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#define RAD2DEG(RAD) ((RAD) * 180 / M_PI)
std::ofstream sensor_data;

void callback(const sensor_msgs::NavSatFix::ConstPtr &gps_msg,
              const geometry_msgs::QuaternionStamped::ConstPtr &atti_msg) {

    ignition::math::Quaterniond rpy;
    rpy.Set(atti_msg->quaternion.w, atti_msg->quaternion.x, atti_msg->quaternion.y, atti_msg->quaternion.z);
    float yaw = RAD2DEG(rpy.Yaw()) - 90;
    if (yaw < -180) { yaw = RAD2DEG(rpy.Yaw()) - 90 + 360; }
    if (yaw > 180) { yaw = RAD2DEG(rpy.Yaw()) - 90 - 360; }

    if (sensor_data.is_open()) {
        sensor_data << RAD2DEG(rpy.Roll()) << "," << RAD2DEG(rpy.Pitch()) << "," << RAD2DEG(rpy.Yaw())
                    << "\t " << std::setprecision(10) << gps_msg->latitude << "\t" << std::setprecision(10)
                    << gps_msg->longitude << "\t " << std::setprecision(10) << gps_msg->altitude << "\n";
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "save_sensor_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::NavSatFix> gps(nh, "/dji_sdk/gps_position", 1);
    message_filters::Subscriber<geometry_msgs::QuaternionStamped> atti(nh, "/dji_sdk/attitude", 1);

    sensor_data.open("sensor_data.txt");
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, geometry_msgs::QuaternionStamped> MySyncPolicy;
    // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), gps, atti);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
