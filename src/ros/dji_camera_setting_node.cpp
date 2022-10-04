
#include <dji_camera_setting.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "dji_camera_setting");
    DJIcameraSetting CameraSetting;
//    ros::spin();
    while(ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
