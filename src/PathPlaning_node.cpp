#include <iostream>
#include <ros_pathGen.hh>

int main(int argc, char **argv) {
    ros::init(argc, argv, "PathGen");
    PathGenerate path;
    ros::spin();
//    while(ros::ok()) {
//        ros::spinOnce();
//    }
    return 0;
}