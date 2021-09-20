//
// Created by regner on 20/09/2021.
//

#include <mission_node.hh>
#include <dji_osdk_ros/FlightTaskControl.h>
#include <dji_osdk_ros/ObtainControlAuthority.h>

//using namespace DJI::OSDK;

// global variables
ros::ServiceClient obtain_ctrl_authority_client;
ros::ServiceClient waypoint_upload_client;
ros::ServiceClient waypoint_action_client;
ros::ServiceClient flight_control_client;
sensor_msgs::NavSatFix gps_pos;
ros::Subscriber gps_pos_subscriber;

void gpsPosCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    gps_pos = *msg;
}

bool runWaypointMission(uint8_t numWaypoints, int responseTimeout) {
    ros::spinOnce();

    // Waypoint Mission : Initialization
    dji_osdk_ros::MissionWaypointTask waypointTask;
    setWaypointInitDefaults(waypointTask);

    // Waypoint Mission: Create Waypoints
    float64_t increment = 0.000001 / C_PI * 180;
    float32_t start_alt = 10;
    ROS_INFO("Creating Waypoints..\n");
    std::vector<WayPointSettings> generatedWaypts =
            createWaypoints(numWaypoints, start_alt);

    // Waypoint Mission: Upload the waypoints
    ROS_INFO("Uploading Waypoints..\n");
    uploadWaypoints(generatedWaypts, responseTimeout, waypointTask);

    // Waypoint Mission: Init mission
    ROS_INFO("Initializing Waypoint Mission..\n");
    if (initWaypointMission(waypointTask).result) {
        ROS_INFO("Waypoint upload command sent successfully");
    } else {
        ROS_WARN("Failed sending waypoint upload command");
        return false;
    }

    // Waypoint Mission: Start
    if (missionAction(DJI_MISSION_TYPE::WAYPOINT,
                      MISSION_ACTION::START).result) {
        ROS_INFO("Mission start command sent successfully");
    } else {
        ROS_WARN("Failed sending mission start command");
        return false;
    }

    return true;
}

void setWaypointDefaults(WayPointSettings *wp) {
    wp->damping = 0;
    wp->yaw = 0;
    wp->gimbalPitch = 0;
    wp->turnMode = 0;
    wp->hasAction = 0;
    wp->actionTimeLimit = 100;
    wp->actionNumber = 0;
    wp->actionRepeat = 0;
    for (int i = 0; i < 16; ++i) {
        wp->commandList[i] = 0;
        wp->commandParameter[i] = 0;
    }
}

void setWaypointInitDefaults(dji_osdk_ros::MissionWaypointTask &waypointTask) {
    waypointTask.velocity_range = 10;
    waypointTask.idle_velocity = 5;
    waypointTask.action_on_finish = dji_osdk_ros::MissionWaypointTask::FINISH_NO_ACTION;
    waypointTask.mission_exec_times = 1;
    waypointTask.yaw_mode = dji_osdk_ros::MissionWaypointTask::YAW_MODE_AUTO;
    waypointTask.trace_mode = dji_osdk_ros::MissionWaypointTask::TRACE_POINT;
    waypointTask.action_on_rc_lost = dji_osdk_ros::MissionWaypointTask::ACTION_AUTO;
    waypointTask.gimbal_pitch_mode = dji_osdk_ros::MissionWaypointTask::GIMBAL_PITCH_FREE;
}

std::vector<DJI::OSDK::WayPointSettings> createWaypoints(int numWaypoints, float32_t start_alt) {
    // Create Start Waypoint
    WayPointSettings start_wp;
    setWaypointDefaults(&start_wp);
    start_wp.latitude = gps_pos.latitude;
    start_wp.longitude = gps_pos.longitude;
    start_wp.altitude = start_alt;
    ROS_INFO("Waypoint created at (LLA): %f \t%f \t%f\n", gps_pos.latitude,
             gps_pos.longitude, start_alt);

    std::vector<DJI::OSDK::WayPointSettings> wpVector = importWaypoints(&start_wp, numWaypoints);
    return wpVector;
}


std::vector<std::pair<std::string, std::vector<float>>> read_csv(std::string filename) {
    // Reads a CSV file into a vector of <string, vector<int>> pairs where
    // each pair represents <column name, column values>

    // Create a vector of <string, int vector> pairs to store the result
    std::vector<std::pair<std::string, std::vector<float>>> result;

    // Create an input filestream
    std::ifstream myFile(filename);

    // Make sure the file is open
    if (!myFile.is_open()) throw std::runtime_error("Could not open file");

    // Helper vars
    std::string line, colname;
    int val;

    // Read the column names
    if (myFile.good()) {
        // Extract the first line in the file
        std::getline(myFile, line);

        // Create a stringstream from line
        std::stringstream ss(line);

        // Extract each column name
        while (std::getline(ss, colname, ',')) {

            // Initialize and add <colname, int vector> pairs to result
            result.push_back({colname, std::vector<float>{}});
        }
    }

    // Read data, line by line
    while (std::getline(myFile, line)) {
        // Create a stringstream of the current line
        std::stringstream ss(line);

        // Keep track of the current column index
        int colIdx = 0;

        // Extract each integer
        while (ss >> val) {

            // Add the current integer to the 'colIdx' column's values vector
            result.at(colIdx).second.push_back(val);

            // If the next token is a comma, ignore it and move on
            if (ss.peek() == ',') ss.ignore();

            // Increment the column index
            colIdx++;
        }
    }

    // Close file
    myFile.close();

    return result;
}

std::vector<DJI::OSDK::WayPointSettings>
importWaypoints(WayPointSettings *start_data, int num_wp) {
    std::vector<std::pair<std::string, std::vector<float>>> wp_file = read_csv(
            "~/catkin_ws/src/riser_inspection/wp_generate.csv");

    std::vector<DJI::OSDK::WayPointSettings> wp_list;


    // Some calculation for the polygon
    float64_t extAngle = 2 * M_PI / num_wp;

    // First waypoint
    start_data->index = 0;
    wp_list.push_back(*start_data);

    // Iterative algorithm
    for (int i = 1; i < wp_file.size(); i++) {
        WayPointSettings wp;
        setWaypointDefaults(&wp);
        wp.index = i;
        wp.latitude = wp_file.at(i).second.at(0);
        wp.longitude = wp_file.at(i).second.at(1);
        wp.altitude = wp_file.at(i).second.at(2);
        wp.yaw = wp_file.at(i).second.at(7);
        wp_list.push_back(wp);
    }

    // Come back home
    start_data->index = num_wp;
    wp_list.push_back(*start_data);

    return wp_list;
}

void uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings> &wp_list,
                     int responseTimeout,
                     dji_osdk_ros::MissionWaypointTask &waypointTask) {
    dji_osdk_ros::MissionWaypoint waypoint;
    for (std::vector<WayPointSettings>::iterator wp = wp_list.begin();
         wp != wp_list.end(); ++wp) {
        ROS_INFO("Waypoint created at (LLA): %f \t%f \t%f\n ", wp->latitude,
                 wp->longitude, wp->altitude);
        waypoint.latitude = wp->latitude;
        waypoint.longitude = wp->longitude;
        waypoint.altitude = wp->altitude;
        waypoint.damping_distance = 0;
        waypoint.target_yaw = 0;
        waypoint.target_gimbal_pitch = 0;
        waypoint.turn_mode = 0;
        waypoint.has_action = 0;
        waypointTask.mission_waypoint.push_back(waypoint);
    }
}


ServiceAck initWaypointMission(dji_osdk_ros::MissionWaypointTask &waypointTask) {
    dji_osdk_ros::MissionWpUpload missionWpUpload;
    missionWpUpload.request.waypoint_task = waypointTask;
    waypoint_upload_client.call(missionWpUpload);
    if (!missionWpUpload.response.result) {
        ROS_WARN("ack.info: set = %i id = %i", missionWpUpload.response.cmd_set,
                 missionWpUpload.response.cmd_id);
        ROS_WARN("ack.data: %i", missionWpUpload.response.ack_data);
    }
    return ServiceAck(
            missionWpUpload.response.result, missionWpUpload.response.cmd_set,
            missionWpUpload.response.cmd_id, missionWpUpload.response.ack_data);
}


ServiceAck missionAction(DJI::OSDK::DJI_MISSION_TYPE type,
                         DJI::OSDK::MISSION_ACTION action) {
    dji_osdk_ros::MissionWpAction missionWpAction;
    dji_osdk_ros::MissionHpAction missionHpAction;
    switch (type) {
        case DJI::OSDK::WAYPOINT:
            missionWpAction.request.action = action;
            waypoint_action_client.call(missionWpAction);
            if (!missionWpAction.response.result) {
                ROS_WARN("ack.info: set = %i id = %i", missionWpAction.response.cmd_set,
                         missionWpAction.response.cmd_id);
                ROS_WARN("ack.data: %i", missionWpAction.response.ack_data);
            }
            return ServiceAck(missionWpAction.response.result,
                              missionWpAction.response.cmd_set,
                              missionWpAction.response.cmd_id,
                              missionWpAction.response.ack_data);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mission_node");
    ros::NodeHandle nh;

    // ROS stuff
    obtain_ctrl_authority_client = nh.serviceClient<dji_osdk_ros::ObtainControlAuthority>(
            "obtain_release_control_authority");
    waypoint_upload_client = nh.serviceClient<dji_osdk_ros::MissionWpUpload>(
            "dji_osdk_ros/mission_waypoint_upload");
    waypoint_action_client = nh.serviceClient<dji_osdk_ros::MissionWpAction>(
            "dji_osdk_ros/mission_waypoint_action");
    flight_control_client =
            nh.serviceClient<dji_osdk_ros::FlightTaskControl>("flight_task_control");
    gps_pos_subscriber = nh.subscribe<sensor_msgs::NavSatFix>(
            "dji_osdk_ros/gps_position", 10, &gpsPosCallback);

    // Setup variables for use
    dji_osdk_ros::ObtainControlAuthority obtainCtrlAuthority;
    obtainCtrlAuthority.request.enable_obtain = true;
    uint8_t wayptPolygonSides;
    int responseTimeout = 1;

    obtain_ctrl_authority_client.call(obtainCtrlAuthority);

    // Display interactive prompt
    std::cout
            << "| Available commands:                                            |"
            << std::endl;
    std::cout
            << "| [a] Waypoint Mission(just for M210V2)                          |"
            << std::endl;
    char inputChar;
    std::cin >> inputChar;
    switch (inputChar) {
        case 'a':
            // Waypoint call
            wayptPolygonSides = 6;
            runWaypointMission(wayptPolygonSides, responseTimeout);
            break;
        case 'b':
            // Hotpoint call
            std::cout << "Error" << std::endl;
            break;
        default:
            break;
    }

    ros::spin();

    return 0;
}

