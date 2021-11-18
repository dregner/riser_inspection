#include <riser_inspection.hh>


RiserInspection::RiserInspection() {
    initServices(nh_);
    initSubscribers(nh_);

}

RiserInspection::~RiserInspection() {}

void RiserInspection::initSubscribers(ros::NodeHandle &nh) {
    try {
        ros::NodeHandle nh_private("~");

        std::string gps_topic, rtk_topic, attitude_topic;
        int riser_distance, riser_diameter, h_points, v_points, delta_h, delta_v;
        // Topic parameters
        nh_private.param("gps_topic", gps_topic, std::string("/dji_sdk/gps_position"));
        nh_private.param("rtk_topic", rtk_topic, std::string("/dji_sdk/rtk_position"));
        nh_private.param("attitude_topic", attitude_topic, std::string("/dji_sdk/attitude"));

        // Path generate parameters
        nh_private.param("riser_distance", riser_distance, 5);
        nh_private.param("riser_diameter", riser_diameter, 5);
        nh_private.param("horizontal_points", h_points, 5);
        nh_private.param("vertical_points", v_points, 5);
        nh_private.param("delta_H", delta_h, 5);
        nh_private.param("delta_V", delta_v, 5);

        // Setting intial parameters to create waypoints
        pathGenerator.setInspectionParam(riser_distance, riser_diameter, h_points, v_points, delta_h, delta_v);


        gps_sub_.subscribe(nh, gps_topic, 1);
        ROS_INFO("Subscriber in M210 GPS Topic: %s", gps_topic.c_str());
        rtk_sub_.subscribe(nh, rtk_topic, 1);
        ROS_INFO("Subscriber in M210 RTK Topic: %s", rtk_topic.c_str());
        attitude_sub_.subscribe(nh, attitude_topic, 1);
        ROS_INFO("Subscriber in M210 Attitude Topic: %s", attitude_topic.c_str());

        sync_.reset(new Sync(RiserInspectionPolicy(10), gps_sub_, rtk_sub_, attitude_sub_));
        sync_->registerCallback(boost::bind(&RiserInspection::position_subscribeCB, this, _1, _2, _3));

        ROS_INFO("Subscribe complet");
    } catch (ros::Exception &e) {
        ROS_ERROR("Subscribe topics exception: %s", e.what());
    }
}

void RiserInspection::initServices(ros::NodeHandle &nh) {
    try {
        ask_control_service = nh.advertiseService("riser_inspection/ask_control",
                                                  &RiserInspection::ask_control, this);
        ROS_INFO("Service riser_inspection/waypoint_generator initialize");

        wp_folders_service = nh.advertiseService("riser_inspection/folder_and_file",
                                                 &RiserInspection::folders_serviceCB,
                                                 this);
        ROS_INFO("Service riser_inspection/Folder initialized. Waypoint file: %s/%s",
                 pathGenerator.getFolderName().c_str(),
                 pathGenerator.getFileName().c_str());

        start_mission_service = nh.advertiseService("riser_inspection/start_mission",
                                                    &RiserInspection::startMission_serviceCB, this);
        ROS_INFO("service riser_inspection/start_mission initialized");

        // DJI Mission Service Clients
        waypoint_upload_service = nh.serviceClient<dji_sdk::MissionWpUpload>("dji_sdk/mission_waypoint_upload");
        waypoint_action_service = nh.serviceClient<dji_sdk::MissionWpAction>("dji_sdk/mission_waypoint_action");
        drone_activation_service = nh.serviceClient<dji_sdk::Activation>("dji_sdk/activation");
        sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");


    } catch (ros::Exception &e) {
        ROS_ERROR("Subscribe topics exception: %s", e.what());
    }
}

bool RiserInspection::ask_control(riser_inspection::askControl::Request &req, riser_inspection::askControl::Response &res) {
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
            ROS_WARN("ack.info: set = %i id = %i", sdkAuthority.response.cmd_set,
                     sdkAuthority.response.cmd_id);
            ROS_WARN("ack.data: %i", sdkAuthority.response.ack_data);
        } else {
            ROS_WARN("Got Authority");
            ROS_INFO("ack.info: set = %i id = %i", sdkAuthority.response.cmd_set,
                     sdkAuthority.response.cmd_id);
            ROS_INFO("ack.data: %i", sdkAuthority.response.ack_data);
            return sdkAuthority.response.result;
        }
    }
}

bool RiserInspection::folders_serviceCB(riser_inspection::wpFolders::Request &req,
                                        riser_inspection::wpFolders::Response &res) {

    if (req.file_name.c_str() != NULL) {
        ROS_INFO("Changing waypoint archive name %s", req.file_name.c_str());
        pathGenerator.setFileName(req.file_name);
    }
    if (pathGenerator.exists(req.file_path.c_str())) {
        ROS_INFO("Waypoint folder changed %s/%s", req.file_path.c_str(), pathGenerator.getFileName().c_str());
        pathGenerator.setFolderName(req.file_path);
        res.result = true;
        return res.result;
    } else {
        ROS_ERROR("Folder does not exist, file will be written in %s/%s", pathGenerator.getFolderName().c_str(),
                  pathGenerator.getFileName().c_str());
        res.result = false;
        return res.result;
    }
}

bool RiserInspection::startMission_serviceCB(riser_inspection::wpStartMission::Request &req,
                                             riser_inspection::wpStartMission::Response &res) {
    ROS_INFO("Received Points");
    // Define where comes the initial value
    if (!req.use_rtk) { start_gnss_ = current_gps_; }
    else { start_gnss_ = current_rtk_; }
    start_atti_ = current_atti_;
    start_atti_eul.Set(start_atti_.w, start_atti_.x, start_atti_.y, start_atti_.z);
    // Define start positions to create waypoints
    pathGenerator.setInitCoord(start_gnss_.latitude,
                               start_gnss_.longitude, start_gnss_.altitude, start_atti_eul.Yaw());
    ROS_INFO("Set initial values");

    try {
        pathGenerator.createInspectionPoints();
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

        if(runWaypointMission(100)){
            ROS_INFO("Finished");
            return true;
        }else{
            ROS_WARN("Error");
            return false;
        }
    }
}

void RiserInspection::position_subscribeCB(const sensor_msgs::NavSatFix::ConstPtr &msg_gps,
                                           const sensor_msgs::NavSatFix::ConstPtr &msg_rtk,
                                           const geometry_msgs::QuaternionStamped::ConstPtr &msg_att) {
    current_atti_ = msg_att->quaternion; // Quaternion w x y z from drone's attitude
    current_gps_ = *msg_gps; // lat lon alt from GPS sensor
    current_rtk_ = *msg_rtk; // lat lon alt from RTK sensor
    current_atti_euler_.Set(current_atti_.w, current_atti_.x, current_atti_.y,
                            current_atti_.z);
}


ServiceAck
RiserInspection::missionAction(DJI::OSDK::DJI_MISSION_TYPE type,
                               DJI::OSDK::MISSION_ACTION action) {
    dji_sdk::MissionWpAction missionWpAction;
    switch (type) {
        case DJI::OSDK::WAYPOINT:
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


bool RiserInspection::askControlAuthority() {
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
RiserInspection::createWayPoint(const std::vector<std::vector<std::string>> csv_file,
                                dji_sdk::MissionWaypointTask &waypointTask) {
    std::vector<DJI::OSDK::WayPointSettings> wp_list; // create a list (vector) containing waypoint structures

    // Push first waypoint as a initial position
    DJI::OSDK::WayPointSettings start_wp;
    setWaypointDefaults(&start_wp);
    start_wp.latitude = start_gnss_.latitude;
    start_wp.longitude = start_gnss_.longitude;
    start_wp.altitude = start_gnss_.altitude;
    start_wp.yaw = start_atti_eul.Yaw();
    ROS_INFO("Waypoint created at (LLA): %f \t%f \t%f\theading:%f\n", start_gnss_.latitude,
             start_gnss_.longitude, start_gnss_.altitude, start_atti_eul.Yaw());
    start_wp.index = 0;
    wp_list.push_back(start_wp);


    for (int k = 1; k < csv_file[0].size(); k++) {
        /// "WP,Latitude,Longitude,AltitudeAMSL,UavYaw,Speed,WaitTime,Picture"
        DJI::OSDK::WayPointSettings wp;
        setWaypointDefaults(&wp);
        wp.index = std::stoi(csv_file[0][k]);
        wp.latitude = std::stod(csv_file[1][k]);
        wp.longitude = std::stod(csv_file[2][k]);
        wp.altitude = std::stod(csv_file[3][k]);
        wp.yaw = std::stoi(csv_file[4][k]);
        wp_list.push_back(wp);
    }
    // Come back home
    start_wp.index = csv_file[0].size() + 1;
    wp_list.push_back(start_wp);
    return wp_list;
}


bool RiserInspection::runWaypointMission(int responseTimeout) {
    ros::spinOnce();

    // Waypoint Mission : Initialization
    dji_sdk::MissionWaypointTask waypointTask;
    setWaypointInitDefaults(waypointTask);


    ROS_INFO("Creating Waypoints..\n");
    // Transform from CSV to DJI vector
    std::vector<std::vector<std::string>> filePathGen = pathGenerator.read_csv(
            pathGenerator.getFolderName() + "/" + pathGenerator.getFileName(), ",");
    std::vector<WayPointSettings> generatedWP = createWayPoint(filePathGen, waypointTask);

    // Waypoint Mission: Upload the waypoints
    ROS_INFO("Uploading Waypoints..\n");
    uploadWaypoints(generatedWP, responseTimeout, waypointTask);

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
                      MISSION_ACTION::START)
            .result) {
        ROS_INFO("Mission start command sent successfully");
    } else {
        ROS_WARN("Failed sending mission start command");
        return false;
    }

    return true;
}

void RiserInspection::uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings> &wp_list,
                                      int responseTimeout, dji_sdk::MissionWaypointTask &waypointTask) {
    dji_sdk::MissionWaypoint waypoint;
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

void RiserInspection::setWaypointDefaults(DJI::OSDK::WayPointSettings *wp) {
    wp->damping = 0;
    wp->yaw = 0;
    wp->gimbalPitch = 0;
    wp->turnMode = 0;
    wp->hasAction = 0;
    wp->actionTimeLimit = 100;
    wp->actionNumber = 2;
    wp->actionRepeat = 1;
    for (int i = 0; i < 16; ++i) {
        wp->commandList[i] = 0;
        wp->commandParameter[i] = 0;
    }
    wp->commandList[0] = 1; // commandList[0] = WP_ACTION_STAY (ms)
    wp->commandParameter[0] = 1000; // Set command to wait 1 second
    wp->commandList[2] = 1; // commandList[2] = WP_ACTION_SIMPLE_SHOT
    wp->commandParameter[2] = 1; // Set command to take photo
//    wp->commandList[4] = 1; // commandList[4] = WP_ACTION_CRAFT_YAW
//    wp->commandParameter[4] = 0; // Set command to take photo
}

void RiserInspection::setWaypointInitDefaults(dji_sdk::MissionWaypointTask &waypointTask) {
    waypointTask.velocity_range = 5; // Maximum speed joystick input(2~15m)
    waypointTask.idle_velocity = 2; //Cruising Speed (without joystick input, no more than vel_cmd_range)
    waypointTask.action_on_finish = dji_sdk::MissionWaypointTask::FINISH_NO_ACTION;
    waypointTask.mission_exec_times = 1;
    waypointTask.yaw_mode = dji_sdk::MissionWaypointTask::YAW_MODE_WAYPOINT;
    waypointTask.trace_mode = dji_sdk::MissionWaypointTask::TRACE_POINT;
    waypointTask.action_on_rc_lost = dji_sdk::MissionWaypointTask::ACTION_FREE;
    waypointTask.gimbal_pitch_mode = dji_sdk::MissionWaypointTask::GIMBAL_PITCH_FREE;
}
