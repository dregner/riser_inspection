
#include <local_position_control.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "local_pos_controller");
    LocalController local_controller;
//    ros::spin();
    while(ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
