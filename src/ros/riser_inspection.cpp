#include <riser_inspection.hh>


RiserInspection::RiserInspection() {
    initServices(nh_);
    initSubscribers(nh_);

}

RiserInspection::~RiserInspection() {}

void RiserInspection::initSubscribers(ros::NodeHandle &nh) {
    try {
        ros::NodeHandle nh_private("~");

        std::string left_topic_, right_topic_, gps_topic_, rtk_topic_, attitude_topic_;
        nh_private.param("gps_topic", gps_topic_, std::string("/dji_sdk/gps_position"));
        nh_private.param("rtk_topic", rtk_topic_, std::string("/dji_sdk/rtk_position"));
        nh_private.param("attitude_topic", attitude_topic_, std::string("/dji_sdk/attitude"));

        gps_sub_.subscribe(nh, gps_topic_, 1);
        ROS_INFO("Subscriber in M210 GPS Topic: %s", gps_topic_.c_str());
        rtk_sub_.subscribe(nh, rtk_topic_, 1);
        ROS_INFO("Subscriber in M210 RTK Topic: %s", rtk_topic_.c_str());
        attitude_sub_.subscribe(nh, attitude_topic_, 1);
        ROS_INFO("Subscriber in M210 Attitude Topic: %s", attitude_topic_.c_str());

        sync_.reset(new Sync(RiserInspectionPolicy(10), gps_sub_, rtk_sub_, attitude_sub_));
        sync_->registerCallback(boost::bind(&RiserInspection::position_subscribeCB, this, _1, _2, _3));

        ROS_INFO("Subscribe complet");
    } catch (ros::Exception &e) {
        ROS_ERROR("Subscribe topics exception: %s", e.what());
    }
}

void RiserInspection::initServices(ros::NodeHandle &nh) {
    try {
        generate_pathway_srv_ = nh.advertiseService("riser_inspection/waypoint_generator",
                                                    &RiserInspection::pathGen_serviceCB, this);
        ROS_INFO("Service riser_inspection/waypoint_generator initialize");

        wp_folders_srv_ = nh.advertiseService("riser_inspection/folder_and_file", &RiserInspection::folders_serviceCB,
                                              this);
        ROS_INFO("Service riser_inspection/Folder initialized. Waypoint file: %s/%s",
                 pathGenerator.getFolderName().c_str(),
                 pathGenerator.getFileName().c_str());

        start_mission_srv_ = nh.advertiseService("riser_inspection/start_mission",
                                                 &RiserInspection::startMission_serviceCB, this);
        ROS_INFO("service riser_inspection/start_mission initialized");

        take_photo_srv_client_ = nh.serviceClient<dji_sdk::CameraAction>("/dji_sdk/camera_action");
        ROS_INFO("Created client for /dji_sdk/camera_action");

        // ROS stuff
        waypoint_upload_service = nh.serviceClient<dji_sdk::MissionWpUpload>(
                "dji_sdk/mission_waypoint_upload");
        waypoint_action_service = nh.serviceClient<dji_sdk::MissionWpAction>(
                "dji_sdk/mission_waypoint_action");
        drone_activation_service =
                nh.serviceClient<dji_sdk::Activation>("dji_sdk/activation");
        sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>(
                "dji_sdk/sdk_control_authority");
        drone_task_service =
                nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");


    } catch (ros::Exception &e) {
        ROS_ERROR("Subscribe topics exception: %s", e.what());
    }
}

bool RiserInspection::pathGen_serviceCB(riser_inspection::wpGenerate::Request &req,
                                        riser_inspection::wpGenerate::Response &res) {
    ROS_INFO("Received Points");
    pathGenerator.setInitCoord(req.riser_distance, float(req.riser_diameter / 1000), lat0_, lon0_, alt0_, head0_);
    pathGenerator.setInspectionParam(req.horizontal_number, req.vertical_number, req.delta_angle, req.delta_height);
    ROS_INFO("Set initial values");

    try {
        pathGenerator.createInspectionPoints();
        ROS_INFO("Waypoints created at %s/%s", pathGenerator.getFolderName().c_str(),
                 pathGenerator.getFileName().c_str());
        return res.result = true;
    } catch (ros::Exception &e) {
        ROS_INFO("ROS error %s", e.what());
        return res.result = false;
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
    if (req.start) {
        ROS_INFO("Mission will be started using file from %s/%s", pathGenerator.getFolderName().c_str(),
                 pathGenerator.getFileName().c_str());
        if (!askControlAuthority()) {
            ROS_INFO("Cannot get Authority Control");
            return res.result = false;
        } else {
            ROS_INFO("Starting Waypoint Mission");

            return res.result = runWaypointMission(100);
        }
    }

}

void RiserInspection::position_subscribeCB(const sensor_msgs::NavSatFixConstPtr &msg_gps,
                                           const sensor_msgs::NavSatFixConstPtr &msg_rtk,
                                           const geometry_msgs::QuaternionStampedConstPtr &msg_att) {
    ptr_attitude_ = *msg_att; // Quaternion w x y z from drone's attitude
    ptr_gps_position_ = *msg_gps; // lat lon alt from GPS sensor
    ptr_rtk_position_ = *msg_rtk; // lat lon alt from RTK sensor
    drone_rpy_.Set(ptr_attitude_.quaternion.w, ptr_attitude_.quaternion.x, ptr_attitude_.quaternion.y,
                   ptr_attitude_.quaternion.z);

    if (node_start_) {
        /// Using GPS position error +- 2000 mmm
        lat0_ = ptr_gps_position_.latitude;
        lon0_ = ptr_gps_position_.longitude;
        alt0_ = ptr_gps_position_.altitude;
        head0_ = drone_rpy_.Yaw(); // heading value??
        node_start_ = false;
    }
}


ServiceAck
RiserInspection::activate() {
    dji_sdk::Activation activation;
    drone_activation_service.call(activation);
    if (!activation.response.result) {
        ROS_WARN("ack.info: set = %i id = %i", activation.response.cmd_set,
                 activation.response.cmd_id);
        ROS_WARN("ack.data: %i", activation.response.ack_data);
    }
    return ServiceAck(activation.response.result, activation.response.cmd_set,
                      activation.response.cmd_id, activation.response.ack_data);
}

ServiceAck
RiserInspection::obtainCtrlAuthority() {
    dji_sdk::SDKControlAuthority sdkAuthority;
    sdkAuthority.request.control_enable = 1;
    sdk_ctrl_authority_service.call(sdkAuthority);
    if (!sdkAuthority.response.result) {
        ROS_WARN("ack.info: set = %i id = %i", sdkAuthority.response.cmd_set,
                 sdkAuthority.response.cmd_id);
        ROS_WARN("ack.data: %i", sdkAuthority.response.ack_data);
    }
    return ServiceAck(sdkAuthority.response.result, sdkAuthority.response.cmd_set,
                      sdkAuthority.response.cmd_id,
                      sdkAuthority.response.ack_data);
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

    // Obtain Control Authority
    ServiceAck ack = obtainCtrlAuthority();
    if (ack.result) {
        ROS_INFO("Obtain SDK control Authority successfully");
        return true;
    } else {
        if (ack.ack_data == 3 && ack.cmd_set == 1 && ack.cmd_id == 0) {
            ROS_INFO("Obtain SDK control Authority in progess, "
                     "send the cmd again");
            obtainCtrlAuthority();
        } else {
            ROS_WARN("Failed Obtain SDK control Authority");
            return false;

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
    start_wp.latitude = lat0_;
    start_wp.longitude = lon0_;
    start_wp.altitude = alt0_;
    start_wp.yaw = head0_;
    ROS_INFO("Waypoint created at (LLA): %f \t%f \t%f\theading:%f\n", lat0_, lon0_, alt0_, head0_);
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
        dji_sdk::CameraAction camera_action;
        camera_action.request.camera_action = 0;
        if (take_photo_srv_client_.call(camera_action)) { ROS_INFO("Took photo"); }
        else { ROS_ERROR("Failed to call service /dji_sdk/camera_action"); }
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
