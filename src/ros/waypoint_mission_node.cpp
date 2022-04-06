#include <waypoint_mission.hh>

int main(int argc, char **argv) {
    ros::init(argc, argv, "waypoint_misison_node");
    WaypointControl riserInspection;
//    ros::spin();
    while(ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
