#include <iostream>
#include <riser_inspection_xyz.hh>
#include <path_generator.hh>

int main(int argc, char **argv) {
    ros::init(argc, argv, "riser_inspection_xyz");
    RiserInspection riserInspection;
//    ros::spin();
    while(ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
