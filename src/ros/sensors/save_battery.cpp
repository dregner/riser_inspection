//
// Created by daniel.regner on 14/09/2020.
//

#include <ros/ros.h>
#include <dji_osdk_ros/BatteryState.h>
#include <fstream>

std::ofstream battery_data;


void callback(const dji_osdk_ros::BatteryState::ConstPtr &msg){

    if(battery_data.is_open()){
        battery_data << msg->
    }
}

int main(int argc, char **argv) {


    ros::init(argc, argv, "battery_state_save");

    ros::NodeHandle nh;
    battery_data.open("sensor_data.txt");
    ros::Subscriber sub_battery = nh.subscribe("/dji_osdk_ros/battery_state", 1, callback);

    ros::spin();
    return 0;
}