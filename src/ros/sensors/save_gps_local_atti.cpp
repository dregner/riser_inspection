//
// Created by vant3d on 09/12/2021.
//

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <ignition/math/Pose3.hh>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <fstream>
#include <sensor_msgs/BatteryState.h>

std::ofstream sensor_data;

#define RAD2DEG(RAD) ((RAD) * 180 / M_PI)


void callback(const sensor_msgs::NavSatFix::ConstPtr &gps_msg,
              const geometry_msgs::QuaternionStamped::ConstPtr &atti_msg,
              const geometry_msgs::PointStamped::ConstPtr &local_msg,
              const sensor_msgs::BatteryState::ConstPtr &bat_msg) {

    ignition::math::Quaterniond rpy;
    rpy.Set(atti_msg->quaternion.w, atti_msg->quaternion.x, atti_msg->quaternion.y, atti_msg->quaternion.z);
    if (sensor_data.is_open()) {
        sensor_data << RAD2DEG(rpy.Roll()) << "\t" << RAD2DEG(rpy.Pitch()) << "\t" << RAD2DEG(rpy.Yaw())
                    << "\t " << std::setprecision(10) << gps_msg->latitude << "\t" << std::setprecision(10)
                    << gps_msg->longitude << "\t" << std::setprecision(10) << gps_msg->altitude
                    << "\t " << std::setprecision(6) << local_msg->point.x << "\t" << std::setprecision(6)
                    << local_msg->point.y << "\t " << std::setprecision(6) << local_msg->point.z << "\t"
                    << bat_msg->percentage << "\t" << bat_msg->voltage << "\t" << bat_msg->current << "\n";
    }

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "position_save_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::NavSatFix> gps(nh, "/dji_osdk_ros/gps_position", 1);
    message_filters::Subscriber<geometry_msgs::QuaternionStamped> atti(nh, "/dji_osdk_ros/attitude", 1);
    message_filters::Subscriber<geometry_msgs::PointStamped> local(nh, "/dji_osdk_ros/local_position", 1);
    message_filters::Subscriber<sensor_msgs::BatteryState> battery(nh, "/dji_osdk_ros/battery_state", 1);

    sensor_data.open("position_data.txt");

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, geometry_msgs::QuaternionStamped, geometry_msgs::PointStamped, sensor_msgs::BatteryState> MySyncPolicy;
    // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), gps, atti, local, battery);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
