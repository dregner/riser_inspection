#include <iostream>
#include <riser_inspection_flight_control.hh>
#include <path_generator.hh>

int main(int argc, char **argv) {
    ros::init(argc, argv, "riser_inspection_wp");
    RiserInspection riserInspection;
//    ros::spin();
    while(ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
