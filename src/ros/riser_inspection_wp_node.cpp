#include <waypoint_mission.hh>

int main(int argc, char **argv) {
    ros::init(argc, argv, "riser_inspection_wp");
    WaypointControl riserInspection;
//    ros::spin();
    while(ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
