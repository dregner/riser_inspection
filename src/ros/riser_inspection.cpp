#include <riser_inspection.hh>


RiserInspection::RiserInspection() {
    subscribing(nh_);
}

RiserInspection::~RiserInspection() = default;

void RiserInspection::subscribing(ros::NodeHandle &nh) {
    std::string gps_topic, rtk_topic, attitude_topic, local_pos_topic;
    // Topic parameters
    nh.param("/riser_inspection/gps_topic", gps_topic, std::string("/dji_sdk/gps_position"));
    nh.param("/riser_inspection/attitude_topic", attitude_topic, std::string("/dji_sdk/attitude"));
    nh.param("/riser_inspection/local_position_topic", local_pos_topic, std::string("/dji_sdk/local_position"));
    // Subscribe topics
    gps_sub = nh.subscribe<sensor_msgs::NavSatFix>(gps_topic, 1, &RiserInspection::gps_callback, this);
    attitude_sub = nh.subscribe<geometry_msgs::QuaternionStamped>(attitude_topic, 1,
                                                                  &RiserInspection::attitude_callback, this);
    local_pos_sub = nh.subscribe<geometry_msgs::PointStamped>(local_pos_topic, 1,
                                                              &RiserInspection::local_position_callback, this);
    // Publish topics
    ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
    // Service topics
    sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
    set_local_pos_reference = nh.serviceClient<dji_sdk::SetLocalPosRef>("dji_sdk/set_local_pos_ref");
    camera_action_service = nh.serviceClient<dji_sdk::CameraAction>("/dji_sdk/camera_action");

    local_position_service = nh.advertiseService("/riser_inspection/start_mission",
                                                 &RiserInspection::start_mision_callback, this);

}

void
RiserInspection::set_waypoint_list(std::vector<std::vector<std::string>> vector, std::vector<float> initial_position) {
    waypoint_l.push_back({0, initial_position[0], initial_position[1], initial_position[2], initial_position[3]});
    for (int i = 1; i < vector.size(); i++) {
        waypoint_l.push_back({std::stof(vector[i][0]), std::stof(vector[i][1]),
                              std::stof(vector[i][2]), std::stof(vector[i][3]), std::stof(vector[i][4])});
    }
    waypoint_l.push_back(
            {static_cast<float>(vector.size()), initial_position[0], initial_position[1], initial_position[2],
             initial_position[3]});

}


bool RiserInspection::obtain_control(bool ask) {
    dji_sdk::SDKControlAuthority authority;
    authority.request.control_enable = ask;
    sdk_ctrl_authority_service.call(authority);

    if (!authority.response.result) {
        ROS_ERROR("Do not understood");
        return false;
    }
    ROS_INFO("SDK Control %s", ask ? "true" : "false");
    return true;
}

bool RiserInspection::start_mision_callback(riser_inspection::wpStartMission::Request &req,
                                            riser_inspection::wpStartMission::Response &res) {

    doing_mission = false;
    ROS_INFO("Received Points");
    pathGenerator.reset();
    waypoint_l.clear();
    // Path generate parameters
    int riser_distance, riser_diameter, h_points, v_points, delta_h, delta_v;
    nh_.param("/riser_inspection/riser_distance", riser_distance, 5);
    nh_.param("/riser_inspection/riser_diameter", riser_diameter, 300);
    nh_.param("/riser_inspection/horizontal_points", h_points, 5);
    nh_.param("/riser_inspection/vertical_points", v_points, 4);
    nh_.param("/riser_inspection/delta_H", delta_h, 15);
    nh_.param("/riser_inspection/delta_V", delta_v, 300);

    // Setting intial parameters to create waypoints
    pathGenerator.setInspectionParam(riser_distance, (float) riser_diameter, h_points, v_points, delta_h,
                                     (float) delta_v);

    // Define where comes the initial value

    if (!req.use_rtk) { start_gps_location = current_gps; }
    else { start_gps_location = current_rtk; }

    start_attitude = current_atti;
    start_atti_eul.Set(start_attitude.quaternion.w, start_attitude.quaternion.x, start_attitude.quaternion.y,
                       start_attitude.quaternion.z);
    // Define start positions to create waypoints
    pathGenerator.setInitCoord(start_gps_location.latitude,
                               start_gps_location.longitude, (float) start_gps_location.altitude,
                               (float) RAD2DEG(start_atti_eul.Yaw()) - 90);
    ROS_INFO("Set initial values");
    if (req.control_type == 1) {
        ROS_INFO("Mission waypoint control");
        mission_type = 1;
        try {
            pathGenerator.createInspectionPoints(3);
            ROS_WARN("Waypoints created at %s/%s", pathGenerator.getFolderName().c_str(),
                     pathGenerator.getFileName().c_str());
        } catch (ros::Exception &e) {
            ROS_WARN("ROS error %s", e.what());
            return res.result = false;
        }
        ROS_INFO("Mission will be started using file from %s/%s", pathGenerator.getFolderName().c_str(),
                 pathGenerator.getFileName().c_str());
        if (!RiserInspection::obtain_control(true)) {
            ROS_WARN("Cannot get Authority Control");
            return res.result = false;
        } else {
            ROS_INFO("Starting Waypoint Mission");
            if (runWaypointMission(100)) {
                doing_mission = true;
                old_gps = current_gps;
                return res.result = true;
            } else {
                ROS_WARN("Error");
                return res.result = false;
            }
        }
    }
    if (req.control_type == 0) {
        ROS_INFO("Local Position control");
        RiserInspection::set_local_position();
        mission_type = 0;
        ROS_INFO("Set Actual position as (0,0,altitude) m");
        try {
            pathGenerator.createInspectionPoints(2);
            ROS_WARN("Waypoints created at %s/%s", pathGenerator.getFolderName().c_str(),
                     pathGenerator.getFileName().c_str());
            std::vector<std::vector<std::string>> waypoint_list = pathGenerator.read_csv(
                    pathGenerator.getFolderName() + "/" + pathGenerator.getFileName(), ",");
            std::vector<float> initial_position = {(float) current_local_pos.point.x, (float) current_local_pos.point.y,
                                                   (float) current_gps.altitude,
                                                   (float) RAD2DEG(current_atti_euler.Yaw())};
            RiserInspection::set_waypoint_list(waypoint_list, initial_position);
        } catch (ros::Exception &e) {
            ROS_WARN("ROS error %s", e.what());
            return res.result = false;
        }
        if (obtain_control(true)) {
            state = 0;
            RiserInspection::setTarget(waypoint_l[state][1], waypoint_l[state][2],
                                       waypoint_l[state][3], DEG2RAD(waypoint_l[state][4]));
            ROS_INFO("Received points x: %f, y: %f, z: %f, yaw: %f",
                     waypoint_l[0][1], waypoint_l[0][2], waypoint_l[0][3], waypoint_l[0][4]);
            doing_mission = true;

            return res.result = true;
        }
    }
}

void RiserInspection::gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_gps = *msg;
    if (doing_mission && mission_type == 1) {
        static int img_counter = 1, wp_counter = 1;
        if (std::abs(current_gps.altitude - old_gps.altitude) > 0.15) {
            if (missionAction(MISSION_ACTION::PAUSE).result) {
                ROS_INFO("Wait Altitude");
                dji_sdk::CameraAction cameraAction;
                cameraAction.request.camera_action = 0;
                camera_action_service.call(cameraAction);
                if (cameraAction.response.result) {
                    ROS_INFO("TOOK PHOTO- %i", img_counter);
                    ros::Duration(1.0).sleep();
                }
                missionAction(MISSION_ACTION::RESUME);
                old_gps = current_gps;
                img_counter++;
            }
        }
        if (std::abs(current_gps.latitude - waypoint_l[wp_counter][1]) < 5e-6 &&
            std::abs(current_gps.longitude - waypoint_l[wp_counter][2]) < 6e-6 &&
            std::abs(current_gps.altitude - waypoint_l[wp_counter][3]) < 0.04) {
            ROS_INFO("REACH WP %i", wp_counter);
            ROS_INFO("Current: lat: %f\tlon: %f\talt: %f", current_gps.latitude, current_gps.longitude,
                     current_gps.altitude);
            ROS_INFO("WP - %f: lat: %f\tlon: %f\talt: %f", waypoint_l[wp_counter][0], waypoint_l[wp_counter][1],
                     waypoint_l[wp_counter][2], waypoint_l[wp_counter][3]);
            if (missionAction(MISSION_ACTION::PAUSE).result) {
                ROS_INFO("Wait Altitude - %i", img_counter);
                dji_sdk::CameraAction cameraAction;
                cameraAction.request.camera_action = 0;
                camera_action_service.call(cameraAction);
                if (cameraAction.response.result) {
                    ROS_INFO("TOOK PHOTO");
                    ros::Duration(6.0).sleep();
                }
                missionAction(MISSION_ACTION::RESUME);
                old_gps = current_gps;
                img_counter++;
                wp_counter++;
            }
        }
        if (std::abs(current_gps.latitude - waypoint_l[waypoint_l.size() - 2][1]) < 1e-5 &&
            std::abs(current_gps.longitude - waypoint_l[waypoint_l.size() - 2][2]) < 1e-5 &&
            std::abs(current_gps.altitude - waypoint_l[waypoint_l.size() - 2][3]) < 0.04) {
            ROS_INFO("FINISHED MISSION");
            doing_mission = false;
        }
    }
}


void RiserInspection::attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr &msg) {
    current_atti = *msg;
    current_atti_euler.Set(current_atti.quaternion.w, current_atti.quaternion.x, current_atti.quaternion.y,
                           current_atti.quaternion.z);

}

void RiserInspection::local_position_callback(const geometry_msgs::PointStamped::ConstPtr &msg) {
    static ros::Time start_time = ros::Time::now();
    ros::Duration elapsed_time = ros::Time::now() - start_time;
    current_local_pos = *msg;
    double xCmd, yCmd, zCmd;
    // Down sampled to 50Hz loop
    if (doing_mission && mission_type == 0) {
        if (elapsed_time > ros::Duration(0.02)) {
            start_time = ros::Time::now();
            RiserInspection::local_position_ctrl(xCmd, yCmd, zCmd);
        }
    }
}

void RiserInspection::setTarget(float x, float y, float z, float yaw) {
    target_offset_x = x;
    target_offset_y = y;
    target_offset_z = z;
    target_yaw = yaw;
}

bool RiserInspection::set_local_position() {
    dji_sdk::SetLocalPosRef localPosReferenceSetter;
    set_local_pos_reference.call(localPosReferenceSetter);

    return (bool) localPosReferenceSetter.response.result;
}

void RiserInspection::local_position_ctrl(double &xCmd, double &yCmd, double &zCmd) {
    xCmd = target_offset_x - current_local_pos.point.x;
    yCmd = target_offset_y - current_local_pos.point.y;
    zCmd = target_offset_z - current_gps.altitude; // - current_local_pos.point.z;


    sensor_msgs::Joy controlPosYaw;
    controlPosYaw.axes.push_back(xCmd);
    controlPosYaw.axes.push_back(yCmd);
    controlPosYaw.axes.push_back(zCmd);
    controlPosYaw.axes.push_back(target_yaw);
    ctrlPosYawPub.publish(controlPosYaw);

    // 0.1m or 10cms is the minimum error to reach target in x y and z axes.
    // This error threshold will have to change depending on aircraft/payload/wind conditions.
    if (((std::abs(xCmd)) < 0.1) && ((std::abs(yCmd)) < 0.1)
        && (std::abs(target_offset_z / 2 - current_gps.altitude) < 0.1)
        && std::abs(target_yaw - current_atti_euler.Yaw()) < 0.03) {
        ROS_INFO("(%f, %f, %f) m @ %f deg target complete",
                 target_offset_x, target_offset_y, target_offset_z, target_yaw);
        ros::Duration(1).sleep();
        if (state <= waypoint_l.size()) {
            state++;
            RiserInspection::setTarget(waypoint_l[state][1], waypoint_l[state][2],
                                       waypoint_l[state][3], DEG2RAD(waypoint_l[state][4]));
            ROS_INFO("Next point x: %f, y: %f, z: %f, yaw: %f",
                     waypoint_l[0][1], waypoint_l[0][2], waypoint_l[0][3], waypoint_l[0][4]);
        } else {
            RiserInspection::obtain_control(false);
            doing_mission = false;
        }
    }
}


ServiceAck
RiserInspection::missionAction(DJI::OSDK::MISSION_ACTION action) {
    dji_sdk::MissionWpAction missionWpAction;
    missionWpAction.request.action = action;
    waypoint_action_service.call(missionWpAction);
    if (!missionWpAction.response.result) {
        ROS_WARN("ack.info: set = %i id = %i", missionWpAction.response.cmd_set,
                 missionWpAction.response.cmd_id);
        ROS_WARN("ack.data: %i", missionWpAction.response.ack_data);
    }
    return {static_cast<bool>(missionWpAction.response.result),
            missionWpAction.response.cmd_set,
            missionWpAction.response.cmd_id,
            missionWpAction.response.ack_data};

}

ServiceAck
RiserInspection::initWaypointMission(dji_sdk::MissionWaypointTask &waypointTask) {
    dji_sdk::MissionWpUpload missionWpUpload;
    missionWpUpload.request.waypoint_task = waypointTask;
    waypoint_upload_service.call(missionWpUpload);
    if (!missionWpUpload.response.result) {
        ROS_WARN("ack.info: set = %i id = %i", missionWpUpload.response.cmd_set,
                 missionWpUpload.response.cmd_id);
        ROS_WARN("ack.data: %i", missionWpUpload.response.ack_data);
    }
    return ServiceAck(
            missionWpUpload.response.result, missionWpUpload.response.cmd_set,
            missionWpUpload.response.cmd_id, missionWpUpload.response.ack_data);
}


std::vector<DJI::OSDK::WayPointSettings>
RiserInspection::createWayPoint(const std::vector<std::vector<std::string>> &csv_file,
                                dji_sdk::MissionWaypointTask &waypointTask) {
    std::vector<DJI::OSDK::WayPointSettings> wp_list; // create a list (vector) containing waypoint structures

    // Push first waypoint as a initial position
    DJI::OSDK::WayPointSettings start_wp;
    setWaypointDefaults(&start_wp);
    start_wp.index = 0;
    start_wp.latitude = start_gps_location.latitude;
    start_wp.longitude = start_gps_location.longitude;
    start_wp.altitude = (float) start_gps_location.altitude - 0.1;
    if (int16_t(RAD2DEG(start_atti_eul.Yaw()) - 90) < -180) {
        start_wp.yaw = int16_t(RAD2DEG(start_atti_eul.Yaw()) - 90 + 360);
    }
    if (int16_t(RAD2DEG(start_atti_eul.Yaw()) - 90) > 180) {
        start_wp.yaw = int16_t(RAD2DEG(start_atti_eul.Yaw()) - 90 - 360);
    }
    waypoint_l.push_back(
            {(float) start_wp.index, (float) start_wp.latitude, (float) start_wp.longitude, start_wp.altitude});
    wp_list.push_back(start_wp);
//    ROS_INFO("Waypoint created at (LLA): %f \t%f \t%f\theading:%f\n", start_gps_location.latitude,
//             start_gps_location.longitude, start_gps_location.altitude, RAD2DEG(start_atti_eul.Yaw()) - 90);

    for (int k = 1; k < csv_file.size(); k++) {
        /// "WP,Latitude,Longitude,AltitudeAMSL,UavYaw,Speed,WaitTime,Picture"
        DJI::OSDK::WayPointSettings wp;
        setWaypointDefaults(&wp);
        wp.index = std::stoi(csv_file[k][0]);
        wp.latitude = std::stod(csv_file[k][1]);
        wp.longitude = std::stod(csv_file[k][2]);
        wp.altitude = std::stof(csv_file[k][3]) - 0.1;
        wp.yaw = std::stof(csv_file[k][4]);
        waypoint_l.push_back({(float) std::stoi(csv_file[k][0]), (float) std::stod(csv_file[k][1]),
                              (float) std::stod(csv_file[k][2]), (float) std::stof(csv_file[k][3])});
        wp_list.push_back(wp);

    }
    // Come back home
    start_wp.index = wp_list.size();
    wp_list.push_back(start_wp);
    waypoint_l.push_back(
            {(float) start_wp.index, (float) start_wp.latitude, (float) start_wp.longitude, start_wp.altitude});
    return wp_list;
}


bool RiserInspection::runWaypointMission(int responseTimeout) {
    ros::spinOnce();

    // Waypoint Mission : Initialization
    dji_sdk::MissionWaypointTask waypointTask;
    setWaypointInitDefaults(waypointTask);


    ROS_INFO("Creating Waypoints..");
    // Transform from CSV to DJI vector
    std::vector<std::vector<std::string>> waypoint_list = pathGenerator.read_csv(
            pathGenerator.getFolderName() + "/" + pathGenerator.getFileName(), ",");
    std::vector<WayPointSettings> generatedWP = createWayPoint(waypoint_list, waypointTask);

    // Waypoint Mission: Upload the waypoints
    ROS_INFO("Uploading Waypoints..");
    uploadWaypoints(generatedWP, waypointTask);

    // Waypoint Mission: Init mission
    ROS_INFO("Initializing Waypoint Mission.");
    if (initWaypointMission(waypointTask).result) {
        ROS_INFO("Waypoint upload command sent successfully");
    } else {
        ROS_WARN("Failed sending waypoint upload command");
        return false;
    }

    // Waypoint Mission: Start
    if (missionAction(MISSION_ACTION::START).result) {
        ROS_INFO("Mission start command sent successfully");
    } else {
        ROS_WARN("Failed sending mission start command");
        return false;
    }
    return true;
}

void RiserInspection::uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings> &wp_list,
                                      dji_sdk::MissionWaypointTask &waypointTask) {
    dji_sdk::MissionWaypoint waypoint;
    for (auto wp = wp_list.begin();
         wp != wp_list.end(); ++wp) {
        waypoint.latitude = wp->latitude;
        waypoint.longitude = wp->longitude;
        waypoint.altitude = wp->altitude;
        waypoint.target_yaw = wp->yaw;
        waypoint.damping_distance = 0;
        waypoint.target_gimbal_pitch = 0;
        waypoint.turn_mode = 1;
        waypoint.has_action = 1;
        waypoint.action_time_limit = 5000;
        waypoint.waypoint_action.action_repeat = 1;
        waypoint.waypoint_action.command_list[0] = 1;
        waypoint.waypoint_action.command_parameter[0] = 2000;
        waypoint.waypoint_action.command_list[1] = 1;
        waypoint.waypoint_action.command_parameter[1] = 1;
        if (wp->index <= waypoint_l[0][0]) {
            waypoint.turn_mode = 0;
        }
        if (wp->index >= waypoint_l.size() - 2) {
            waypoint.turn_mode = 0;
        }
        waypointTask.mission_waypoint.push_back(waypoint);
        ROS_INFO("Waypoint %i at (LLA): %f,%f,%f - %i", wp->index, waypoint.latitude,
                 waypoint.longitude, waypoint.altitude, waypoint.target_yaw);
        ROS_INFO("wp -%i - Turn mode %i", wp->index, waypoint.turn_mode);
    }
}

void RiserInspection::setWaypointDefaults(DJI::OSDK::WayPointSettings *wp) {
    wp->damping = 0;
    wp->yaw = 0;
    wp->gimbalPitch = 0;
    wp->turnMode = 1; // 0 - clockwise, 1 - counter-clockwise
    wp->hasAction = 1;
    wp->actionTimeLimit = 5000;
    wp->actionNumber = 2;
    wp->actionRepeat = 1;
    for (int i = 0; i < 16; ++i) {
        wp->commandList[i] = 0;
        wp->commandParameter[i] = 0;
    }
//    wp->commandList[0] = 0; // commandList[0] = WP_ACTION_STAY (ms)
//    wp->commandParameter[0] = 5000; // Set command to wait milliseconds
//    wp->commandList[1] = 1; // commandList[2] = WP_ACTION_SIMPLE_SHOT
//    wp->commandParameter[1] = 1; // Set command to take photo
//    wp->commandList[4] = 1; // commandList[4] = WP_ACTION_CRAFT_YAW
//    wp->commandParameter[4] = 0; // Set command to take photo
}

void RiserInspection::setWaypointInitDefaults(dji_sdk::MissionWaypointTask &waypointTask) {
    waypointTask.velocity_range = 2; // Maximum speed joystick input(2~15m)
    waypointTask.idle_velocity = 0.1; //Cruising Speed (without joystick input, no more than vel_cmd_range)
    waypointTask.action_on_finish = dji_sdk::MissionWaypointTask::FINISH_NO_ACTION;
    waypointTask.mission_exec_times = 1;
    waypointTask.yaw_mode = dji_sdk::MissionWaypointTask::YAW_MODE_WAYPOINT;
    waypointTask.trace_mode = dji_sdk::MissionWaypointTask::TRACE_POINT;
    waypointTask.action_on_rc_lost = dji_sdk::MissionWaypointTask::ACTION_FREE;
    waypointTask.gimbal_pitch_mode = dji_sdk::MissionWaypointTask::GIMBAL_PITCH_FREE;
}