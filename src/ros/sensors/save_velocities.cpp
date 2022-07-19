//
// Created by vant3d on 09/12/2021.
//

#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "fstream"
#define RAD2DEG(RAD) ((RAD) * 180 / M_PI)
std::ofstream sensor_data;

void callback(const geometry_msgs::Vector3Stamped::ConstPtr &ground_v,
              const geometry_msgs::Vector3Stamped::ConstPtr &fused_w) {


    if (sensor_data.is_open()) {
        sensor_data << std::setprecision(10) << ground_v->vector.x << "\t" << std::setprecision(10)
                    << ground_v->vector.y << "\t " << std::setprecision(10) << ground_v->vector.z
                    << std::setprecision(10) << fused_w->vector.x << "\t" << std::setprecision(10)
                    << fused_w->vector.y << "\t " << std::setprecision(10) << fused_w->vector.z << "\n";
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "save_velocities");
    ros::NodeHandle nh;

    message_filters::Subscriber<geometry_msgs::Vector3Stamped> ground_vel(nh, "/dji_sdk/acceleration_ground_fused", 1);
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> angular_vel(nh, "/dji_sdk/angular_velocity_fused", 1);

    sensor_data.open("velocity_data.txt");
    sensor_data << "Ground E" << "\t" << "Ground N" << "\t" << "Up" << "\t" << "p" << "\t" << "q" << "\t" << "r" << std::endl;
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::Vector3Stamped , geometry_msgs::Vector3Stamped> MySyncPolicy;
    // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), ground_vel, angular_vel);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
