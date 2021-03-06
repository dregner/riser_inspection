#include <waypoint_mission.hh>


WaypointControl::WaypointControl() {
    initServices(nh_);
    initSubscribers(nh_);

}

WaypointControl::~WaypointControl() = default;

void WaypointControl::initSubscribers(ros::NodeHandle &nh) {
    try {
        ros::NodeHandle nh_private("~");

        std::string gps_topic, rtk_topic, attitude_topic, height_topic, root_directory;
        // Topic parameters
        nh_private.param("/riser_inspection/gps_topic", gps_topic, std::string("/dji_sdk/gps_position"));
        nh_private.param("/riser_inspection/rtk_topic", rtk_topic, std::string("/dji_sdk/rtk_position"));
        nh_private.param("/riser_inspection/attitude_topic", attitude_topic, std::string("/dji_sdk/attitude"));
        nh_private.param("/riser_inspection/height_topic", height_topic, std::string("/dji_sdk/height_above_takeoff"));
        nh_private.param("/riser_inspection/root_directory", root_directory, std::string("/home/vant3d/Documents"));
        nh_private.param("/riser_inspection/wait_time", wait_time, 6);

        pathGenerator.setFolderName(root_directory);
        gps_sub_ = nh.subscribe<sensor_msgs::NavSatFix>(gps_topic, 1, &WaypointControl::gps_callback, this);
        rtk_sub_ = nh.subscribe<sensor_msgs::NavSatFix>(rtk_topic, 1, &WaypointControl::rtk_callback, this);
        height_above_takeoff_sub_ = nh.subscribe<std_msgs::Float32>(height_topic, 1, &WaypointControl::height_callback, this);
        attitude_sub_ = nh.subscribe<geometry_msgs::QuaternionStamped>(attitude_topic, 1,
                                                                       &WaypointControl::atti_callback, this);

        ROS_INFO("Subscribe complet");
    } catch (ros::Exception &e) {
        ROS_ERROR("Subscribe topics exception: %s", e.what());
    }
}   

void WaypointControl::initServices(ros::NodeHandle &nh) {
    try {
        ask_control_service = nh.advertiseService("riser_inspection/ask_control",
                                                  &WaypointControl::ask_control, this);
        ROS_INFO("Service riser_inspection/waypoint_generator initialize");

        wp_folders_service = nh.advertiseService("riser_inspection/folder_and_file",
                                                 &WaypointControl::folders_serviceCB,
                                                 this);
        ROS_INFO("Service riser_inspection/Folder initialized.");
        ROS_INFO("Waypoint file: %s/%s", pathGenerator.getFolderName().c_str(), pathGenerator.getFileName().c_str());

        start_mission_service = nh.advertiseService("riser_inspection/start_mission",
                                                    &WaypointControl::startMission_serviceCB, this);
        ROS_INFO("service riser_inspection/start_mission initialized");

        // DJI Mission Service Clients
        waypoint_upload_service = nh.serviceClient<dji_sdk::MissionWpUpload>("dji_sdk/mission_waypoint_upload");
        waypoint_action_service = nh.serviceClient<dji_sdk::MissionWpAction>("dji_sdk/mission_waypoint_action");
        drone_activation_service = nh.serviceClient<dji_sdk::Activation>("dji_sdk/activation");
        sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
        camera_action_service = nh.serviceClient<dji_sdk::CameraAction>("dji_sdk/camera_action");
        stereo_acquisition = nh.serviceClient<stereo_vant::PointGray>("stereo_point_grey/take_picture");


    } catch (ros::Exception &e) {
        ROS_ERROR("Subscribe topics exception: %s", e.what());
    }
}

bool
WaypointControl::ask_control(riser_inspection::askControl::Request &req, riser_inspection::askControl::Response &res) {
    ROS_INFO("Received Points");
    dji_sdk::Activation activation;
    drone_activation_service.call(activation);
    if (!activation.response.result) {
        ROS_WARN("ack.info: set = %i id = %i", activation.response.cmd_set,
                 activation.response.cmd_id);
        ROS_WARN("ack.data: %i", activation.response.ack_data);
    } else {
        ROS_WARN("Activated");
        ROS_INFO("ack.info: set = %i id = %i", activation.response.cmd_set,
                 activation.response.cmd_id);
        ROS_INFO("ack.data: %i", activation.response.ack_data);
        dji_sdk::SDKControlAuthority sdkAuthority;
        sdkAuthority.request.control_enable = 1;
        sdk_ctrl_authority_service.call(sdkAuthority);
        if (!sdkAuthority.response.result) {
            ROS_WARN("Ask authority for second time");
            ROS_WARN("ack.info: set = %i id = %i", sdkAuthority.response.cmd_set,
                     sdkAuthority.response.cmd_id);
            ROS_WARN("ack.data: %i", sdkAuthority.response.ack_data);
        } else {
            ROS_WARN("Got Authority");
            ROS_INFO("ack.info: set = %i id = %i", sdkAuthority.response.cmd_set,
                     sdkAuthority.response.cmd_id);
            ROS_INFO("ack.data: %i", sdkAuthority.response.ack_data);
            return true;
        }
    }
}

bool WaypointControl::folders_serviceCB(riser_inspection::wpFolders::Request &req,
                                        riser_inspection::wpFolders::Response &res) {

    if (req.file_name.c_str() != nullptr) {
        ROS_INFO("Changing waypoint archive name %s", req.file_name.c_str());
        pathGenerator.setFileName(req.file_name);
    }
    if (pathGenerator.exists(req.file_path)) {
        ROS_INFO("Waypoint folder changed %s/%s", req.file_path.c_str(), pathGenerator.getFileName().c_str());
        pathGenerator.setFolderName(req.file_path);
        res.result = true;    nh_.param("/riser_inspection/horizontal_error", h_error, 0.2);
    nh_.param("/riser_inspeciton/vertical_error", v_error, 0.1);
        return res.result;
    } else {
        ROS_ERROR("Folder does not exist, file will be written in %s/%s", pathGenerator.getFolderName().c_str(),
                  pathGenerator.getFileName().c_str());
        res.result = false;
        return res.result;
    }
}

bool WaypointControl::startMission_serviceCB(riser_inspection::StartMission::Request &req,
                                             riser_inspection::StartMission::Response &res) {
    // Initial setting
    pathGenerator.reset(); // clear pathGen
    waypoint_l.clear(); // clear waypoint list
    img_counter = 1, wp_counter = 1; // Reset counter of image and waypoint
    voo++; // change images file name
    start_gps_location = current_gps; // Get Initial position
    // Path generate parameters
    int riser_distance, riser_diameter, h_points, v_points, delta_h, delta_v;
    nh_.param("/riser_inspection/riser_distance", riser_distance, 5);
    nh_.param("/riser_inspection/riser_diameter", riser_diameter, 300);
    nh_.param("/riser_inspection/horizontal_points", h_points, 5);
    nh_.param("/riser_inspection/vertical_points", v_points, 4);
    nh_.param("/riser_inspection/delta_H", delta_h, 15);
    nh_.param("/riser_inspection/delta_V", delta_v, 300);
    nh_.param("/riser_inspection/horizontal_error", h_error, 0.2);
    nh_.param("/riser_inspection/vertical_error", v_error, 0.1);

    // Setting intial parameters to create waypoints
    pathGenerator.setInspectionParam(riser_distance, (float) riser_diameter, h_points, v_points, delta_h,
                                     (float) delta_v);




    start_attitude = current_atti;
    start_gps_location.altitude = current_height->data;
    start_atti_eul.Set(start_attitude.quaternion.w, start_attitude.quaternion.x, start_attitude.quaternion.y,
                       start_attitude.quaternion.z);
    // Define start positions to create waypoints
    pathGenerator.setInitCoord(start_gps_location.latitude,
                               start_gps_location.longitude, (float) start_gps_location.altitude,
                               (int) RAD2DEG(start_atti_eul.Yaw()));
    ROS_INFO("Set initial values");

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
    if (!askControlAuthority()) {
        ROS_WARN("Cannot get Authority Control");
        return res.result = false;
    } else {
        ROS_INFO("Starting Waypoint Mission");
        if (runWaypointMission(100)) {
            doing_mission = true;
            old_gps = current_gps;
            dji_sdk::CameraAction cameraAction;
            cameraAction.request.camera_action = 0;
            stereo_vant::PointGray pointGray;
            pointGray.request.file_name = "stereo";
            pointGray.request.file_path = pathGenerator.getFolderName() + "/voo" + std::to_string(voo);
            pointGray.request.reset_counter = true;
            stereo_acquisition.call(pointGray);
            camera_action_service.call(cameraAction);
            return res.result = true;
        } else {
            ROS_WARN("Error");
            return res.result = false;
        }
    }
}

void WaypointControl::height_callback(const std_msgs::Float32::ConstPtr &msg) {
    current_height = msg;
}

void WaypointControl::gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_gps = *msg;
    if (!use_rtk) {
        mission(msg, current_height);
    }
}

void WaypointControl::rtk_callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_rtk = *msg;
    if (use_rtk) {
        mission(msg, current_height);
    }
}

void WaypointControl::atti_callback(const geometry_msgs::QuaternionStamped::ConstPtr &msg) {
    current_atti = *msg;
    current_atti_euler.Set(current_atti.quaternion.w, current_atti.quaternion.x, current_atti.quaternion.y,
                           current_atti.quaternion.z);
}



ServiceAck
WaypointControl::missionAction(DJI::OSDK::MISSION_ACTION action) {
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
WaypointControl::initWaypointMission(dji_sdk::MissionWaypointTask &waypointTask) {
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


bool WaypointControl::askControlAuthority() {
    // Activate
    dji_sdk::Activation activation;
    drone_activation_service.call(activation);
    if (!activation.response.result) {
        ROS_WARN("ack.info: set = %i id = %i", activation.response.cmd_set,
                 activation.response.cmd_id);
        ROS_WARN("ack.data: %i", activation.response.ack_data);
        return false;
    } else {
        ROS_INFO("Activated successfully");
        dji_sdk::SDKControlAuthority sdkAuthority;
        sdkAuthority.request.control_enable = 1;
        sdk_ctrl_authority_service.call(sdkAuthority);
        if (sdkAuthority.response.result) {
            ROS_INFO("Obtain SDK control Authority successfully");
            return true;
        } else {
            if (sdkAuthority.response.ack_data == 3 &&
                sdkAuthority.response.cmd_set == 1 && sdkAuthority.response.cmd_id == 0) {
                ROS_INFO("Obtain SDK control Authority in progess, "
                         "send the cmd again");
                sdk_ctrl_authority_service.call(sdkAuthority);
            } else {
                ROS_WARN("Failed Obtain SDK control Authority");
                return false;
                ROS_WARN("ack.info: set = %i id = %i", sdkAuthority.response.cmd_set,
                         sdkAuthority.response.cmd_id);
                ROS_WARN("ack.data: %i", sdkAuthority.response.ack_data);
            }
        }
    }
}

std::vector<DJI::OSDK::WayPointSettings>
WaypointControl::createWayPoint(const std::vector<std::vector<std::string>> &csv_file,
                                dji_sdk::MissionWaypointTask &waypointTask) {
    std::vector<DJI::OSDK::WayPointSettings> wp_list; // create a list (vector) containing waypoint structures

    // Push first waypoint as a initial position
    DJI::OSDK::WayPointSettings start_wp;
    setWaypointDefaults(&start_wp);
    start_wp.index = 0;
    start_wp.latitude = start_gps_location.latitude;
    start_wp.longitude = start_gps_location.longitude;
    start_wp.altitude = (float) start_gps_location.altitude;
    if (int16_t(RAD2DEG(start_atti_eul.Yaw()) ) < -180) {
        start_wp.yaw = int16_t(RAD2DEG(start_atti_eul.Yaw()) + 360);
    } else { start_wp.yaw = int16_t(RAD2DEG(start_atti_eul.Yaw())); }
    if (int16_t(RAD2DEG(start_atti_eul.Yaw())) > 180) {
        start_wp.yaw = int16_t(RAD2DEG(start_atti_eul.Yaw()) - 360);
    } else { start_wp.yaw = int16_t(RAD2DEG(start_atti_eul.Yaw())); }
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
        wp.altitude = std::stof(csv_file[k][3]);
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


bool WaypointControl::runWaypointMission(int responseTimeout) {
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

void WaypointControl::uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings> &wp_list,
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

void WaypointControl::setWaypointDefaults(DJI::OSDK::WayPointSettings *wp) {
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

void WaypointControl::setWaypointInitDefaults(dji_sdk::MissionWaypointTask &waypointTask) {
    waypointTask.velocity_range = 2; // Maximum speed joystick input(2~15m)
    waypointTask.idle_velocity = 0.1; //Cruising Speed (without joystick input, no more than vel_cmd_range)
    waypointTask.action_on_finish = dji_sdk::MissionWaypointTask::FINISH_NO_ACTION;
    waypointTask.mission_exec_times = 1;
    waypointTask.yaw_mode = dji_sdk::MissionWaypointTask::YAW_MODE_WAYPOINT;
    waypointTask.trace_mode = dji_sdk::MissionWaypointTask::TRACE_POINT;
    waypointTask.action_on_rc_lost = dji_sdk::MissionWaypointTask::ACTION_FREE;
    waypointTask.gimbal_pitch_mode = dji_sdk::MissionWaypointTask::GIMBAL_PITCH_FREE;
}

void WaypointControl::mission(const sensor_msgs::NavSatFix::ConstPtr &gps_msg, const std_msgs::Float32::ConstPtr &height_msg) {
    if (doing_mission) {
        if (std::abs(height_msg->data - old_gps.altitude) > 0.15) {
            if (missionAction(MISSION_ACTION::PAUSE).result) {
                ROS_INFO("Wait Altitude - %f m", gps_msg->altitude);

                if (take_photo()) {
                    ROS_INFO("TOOK PHOTO- %i", img_counter);
                    ros::Duration(wait_time).sleep();
                }
                missionAction(MISSION_ACTION::RESUME);
                old_gps = *gps_msg;
                old_height = *height_msg;
                img_counter++;
            }
        }
        if (std::abs(gps_msg->latitude - waypoint_l[wp_counter][1]) < h_error &&
            std::abs(gps_msg->longitude - waypoint_l[wp_counter][2]) < h_error &&
            std::abs(height_msg->data - waypoint_l[wp_counter][3]) < v_error) {
            ROS_INFO("REACH WP %i", wp_counter);
            ROS_INFO("Current: lat: %f\tlon: %f\talt: %f", gps_msg->latitude, gps_msg->longitude,
                     height_msg->data);
            ROS_INFO("WP - %f: lat: %f\tlon: %f\talt: %f", waypoint_l[wp_counter][0], waypoint_l[wp_counter][1],
                     waypoint_l[wp_counter][2], waypoint_l[wp_counter][3]);
            if (missionAction(MISSION_ACTION::PAUSE).result) {
                ROS_INFO("Wait WP %i", wp_counter);
                dji_sdk::CameraAction cameraAction;
                cameraAction.request.camera_action = 0;
                camera_action_service.call(cameraAction);
                if (take_photo()) {
                    ROS_INFO("TOOK PHOTO- %i", img_counter);
                    ros::Duration(wait_time).sleep();
                }
                missionAction(MISSION_ACTION::RESUME);
                old_gps = *gps_msg;
                old_height = *height_msg;
                img_counter++;
                wp_counter++;
            }
        }
        if (std::abs(gps_msg->latitude - waypoint_l[waypoint_l.size() - 2][1]) < h_error &&
            std::abs(gps_msg->longitude - waypoint_l[waypoint_l.size() - 2][2]) < h_error &&
            std::abs(height_msg->data - waypoint_l[waypoint_l.size() - 2][3]) < v_error) {
            ROS_INFO("FINISHED MISSION");
            doing_mission = false;

        }
    }
}

bool WaypointControl::take_photo() {
    dji_sdk::CameraAction cameraAction;
    cameraAction.request.camera_action = 0;
    stereo_vant::PointGray pointGray;
    pointGray.request.file_name = "stereo";
    pointGray.request.file_path = pathGenerator.getFolderName() + "/voo" + std::to_string(voo);
    pointGray.request.reset_counter = false;

    stereo_acquisition.call(pointGray);
    camera_action_service.call(cameraAction);

    return cameraAction.response.result && pointGray.response.result;
}