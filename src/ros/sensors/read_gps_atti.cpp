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
#include <dji_telemetry.hpp>
#include <tf/tf.h>
#define RAD2DEG(RAD) ((RAD) * 180 / M_PI)


DJI::OSDK::Telemetry::Vector3f quaternionToEulerAngle(const geometry_msgs::QuaternionStamped::ConstPtr &quat)
{
    DJI::OSDK::Telemetry::Vector3f eulerAngle;
    double q2sqr = quat->quaternion.y * quat->quaternion.y;
    double t0 = -2.0 * (q2sqr + quat->quaternion.z * quat->quaternion.z) + 1.0;
    double t1 = 2.0 * (quat->quaternion.x * quat->quaternion.y + quat->quaternion.w * quat->quaternion.z);
    double t2 = -2.0 * (quat->quaternion.x * quat->quaternion.z - quat->quaternion.w * quat->quaternion.y);
    double t3 = 2.0 * (quat->quaternion.y * quat->quaternion.z + quat->quaternion.w * quat->quaternion.x);
    double t4 = -2.0 * (quat->quaternion.x * quat->quaternion.x + q2sqr) + 1.0;
    t2 = (t2 > 1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;
    eulerAngle.x = asin(t2);
    eulerAngle.y = atan2(t3, t4);
    eulerAngle.z = atan2(t1, t0);
    return eulerAngle;
}
float stupid_offset(float yaw){
    if(0 <= yaw <= 90){
        yaw = -yaw +90;
        return yaw;
    }
    if(90 < yaw <= 180){
        yaw = -yaw +180;
        return yaw;
    }
    if(-90 < yaw < 0){
        yaw = yaw -180;
        return yaw;
    }
    if(-180 < yaw < -90){
        yaw = yaw - 90;
        return yaw;
    }
}

void callback(const sensor_msgs::NavSatFix::ConstPtr &gps_msg,
              const geometry_msgs::QuaternionStamped::ConstPtr &atti_msg) {

    ignition::math::Quaterniond rpy, rpy_total;
    rpy.Set(atti_msg->quaternion.w, atti_msg->quaternion.x, atti_msg->quaternion.y, atti_msg->quaternion.z);

    float yaw_c = stupid_offset(RAD2DEG(rpy.Yaw()));
    if (yaw_c < -180) { yaw_c = yaw_c + 360; }
    if (yaw_c > 180) { yaw_c = yaw_c - 360; }

    tf::Matrix3x3 R_ATTI;
    tf::Quaternion q_total;
    R_ATTI.setRotation(tf::Quaternion(atti_msg->quaternion.x,atti_msg->quaternion.y,atti_msg->quaternion.z, atti_msg->quaternion.w));
    tf::Matrix3x3 ROT = {0,-1,0,-1,0,0,0,0,-1};
    tf::Matrix3x3 R_total = R_ATTI * ROT;
    R_total.getRotation(q_total);
    rpy_total.Set(q_total.getW(), q_total.getX(), q_total.getY(), q_total.getZ());

    std::cout << "R: " << RAD2DEG(rpy.Roll()) << "\tP: " << RAD2DEG(rpy.Pitch()) << "\tY: " << RAD2DEG(rpy.Yaw()) << std::endl;
    std::cout << "AFTER rotx(180)*rotz(90)" << std::endl;
    std::cout << "R: " << RAD2DEG(rpy_total.Pitch()) << "\tP: " << RAD2DEG(rpy_total.Roll()) << "\tY: " << -RAD2DEG(rpy_total.Yaw()) << std::endl;
//    std::cout << "R: " << rpy.Roll() << "\tP: " <<rpy.Pitch() << "\tY: " << rpy.Yaw()<< std::endl;
    std::cout << "GPS" << std::endl;
    std::cout << "LAT: " << gps_msg->latitude << "\tLON: " << gps_msg->longitude << "\tALT: " << gps_msg->altitude << std::endl;
    std::cout << "\033[2J\033[1;1H";     // clear terminal

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "gps_atti_node");
    ros::NodeHandle nh;

    message_filters::Subscriber <sensor_msgs::NavSatFix> gps(nh, "/dji_osdk_ros/gps_position", 1);
    message_filters::Subscriber <geometry_msgs::QuaternionStamped> atti(nh, "/dji_osdk_ros/attitude", 1);


    typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::NavSatFix, geometry_msgs::QuaternionStamped> MySyncPolicy;
    // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer <MySyncPolicy> sync(MySyncPolicy(100), gps, atti);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
