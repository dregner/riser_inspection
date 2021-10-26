#include <iostream>
#include <riser_inspection.hh>

int main(int argc, char **argv) {
    ros::init(argc, argv, "PathGen");
    PathGenerate path;
    ros::spin();
//    while(ros::ok()) {
//        ros::spinOnce();
//    }
    return 0;
}