#include <iostream>
#include <path_generator.hh>

int main(int argc, char **argv) {
    ros::init(argc, argv, "waypointV2_node");
    ros::NodeHandle nh;

    ros::Subscriber gpsPositionSub = nh.subscribe("dji_osdk_ros/gps_position", 10, &gpsPositionSubCallback);
    auto obtain_ctrl_authority_client = nh.serviceClient<dji_osdk_ros::ObtainControlAuthority>(
            "obtain_release_control_authority");

    //if you want to fly without rc ,you need to obtain ctrl authority.Or it will enter rc lost.
    dji_osdk_ros::ObtainControlAuthority obtainCtrlAuthority;
    obtainCtrlAuthority.request.enable_obtain = true;
    obtain_ctrl_authority_client.call(obtainCtrlAuthority);

    return 0;
}