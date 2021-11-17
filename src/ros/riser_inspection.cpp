#include <riser_inspection.hh>


RiserInspection::RiserInspection() {
    initServices(nh_);
    initSubscribers(nh_);

}

RiserInspection::~RiserInspection() {}

void RiserInspection::initSubscribers(ros::NodeHandle &nh) {
    try {
        ros::NodeHandle nh_private("~");

        std::string gps_topic_, rtk_topic_, attitude_topic_;
        nh_private.param("gps_topic", gps_topic_, std::string("/dji_sdk/gps_position"));
        nh_private.param("rtk_topic", rtk_topic_, std::string("/dji_sdk/rtk_position"));
        nh_private.param("attitude_topic", attitude_topic_, std::string("/dji_sdk/attitude"));

        gps_sub_ = nh.subscribe<sensor_msgs::NavSatFix>(gps_topic_, 10, &RiserInspection::gps_callback, this);
        rtk_sub_ = nh.subscribe<sensor_msgs::NavSatFix>(rtk_topic_, 10, &RiserInspection::rtk_callback, this);
        attitude_sub_ = nh.subscribe<geometry_msgs::QuaternionStamped>(attitude_topic_, 10,
                                                                       &RiserInspection::attitude_callback, this);

        ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);


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

        sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
        drone_task_service = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
        set_local_pos_reference = nh.serviceClient<dji_sdk::SetLocalPosRef>("dji_sdk/set_local_pos_ref");

    } catch (ros::Exception &e) {
        ROS_ERROR("Subscribe topics exception: %s", e.what());
    }
}

bool RiserInspection::pathGen_serviceCB(riser_inspection::wpGenerate::Request &req,
                                        riser_inspection::wpGenerate::Response &res) {
    ROS_INFO("Received Points");
    start_gps_ = current_gps_;
    start_atti_ = current_atti_;
    start_atti_eul.Set(start_atti_.w, start_atti_.x, start_atti_.y, start_atti_.z);

    pathGenerator.setInitCoord(req.riser_distance, float(req.riser_diameter / 1000), start_gps_.latitude,
                               start_gps_.longitude, start_gps_.altitude, start_atti_eul.Yaw());
    pathGenerator.setInspectionParam(req.horizontal_number, req.vertical_number, req.delta_angle, req.delta_height);
    ROS_INFO("Set initial values");

    try {
        /// csv_type: 1 - ugcs complete; 2 - ugcs EMU; 3 - UgCS TOP/BOTTOM; 4 - DJI
        pathGenerator.createInspectionPoints(3);
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

        if (!obtain_control()) {
            ROS_INFO("Cannot get Authority Control");
            return res.result = false;
        } else {

            ROS_INFO("Starting Waypoint Mission");

//            return res.result = runWaypointMission(100);
        }
    }

}


void RiserInspection::localOffsetFromGpsOffset(geometry_msgs::Vector3 &deltaNed,
                                               sensor_msgs::NavSatFix &target,
                                               sensor_msgs::NavSatFix &origin) {
    double deltaLon = target.longitude - origin.longitude;
    double deltaLat = target.latitude - origin.latitude;

    deltaNed.y = DEG2RAD(deltaLat * C_EARTH);
    deltaNed.x = DEG2RAD(deltaLon * C_EARTH * cos(DEG2RAD(target.latitude)));
    deltaNed.z = target.altitude - origin.altitude;
}

/*!
 * This function demos how to use the flight_status
 * and the more detailed display_mode (only for A3/N3)
 * to monitor the take off process with some error
 * handling. Note M100 flight status is different
 * from A3/N3 flight status.
 */
bool RiserInspection::monitoredTakeoff() {
    ros::Time start_time = ros::Time::now();

    if (!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
        return false;
    }

    ros::Duration(0.01).sleep();
    ros::spinOnce();

    // Step 1.1: Spin the motor
    while (flight_status_ != DJISDK::FlightStatus::STATUS_ON_GROUND &&
           display_mode_ != DJISDK::DisplayMode::MODE_ENGINE_START &&
           ros::Time::now() - start_time < ros::Duration(5)) {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

    if (ros::Time::now() - start_time > ros::Duration(5)) {
        ROS_ERROR("Takeoff failed. Motors are not spinnning.");
        return false;
    } else {
        start_time = ros::Time::now();
        ROS_INFO("Motor Spinning ...");
        ros::spinOnce();
    }


    // Step 1.2: Get in to the air
    while (flight_status_ != DJISDK::FlightStatus::STATUS_IN_AIR &&
           (display_mode_ != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF ||
            display_mode_ != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
           ros::Time::now() - start_time < ros::Duration(20)) {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

    if (ros::Time::now() - start_time > ros::Duration(20)) {
        ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
        return false;
    } else {
        start_time = ros::Time::now();
        ROS_INFO("Ascending...");
        ros::spinOnce();
    }

    // Final check: Finished takeoff
    while ((display_mode_ == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF ||
            display_mode_ == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
           ros::Time::now() - start_time < ros::Duration(20)) {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

    if (display_mode_ != DJISDK::DisplayMode::MODE_P_GPS || display_mode_ != DJISDK::DisplayMode::MODE_ATTITUDE) {
        ROS_INFO("Successful takeoff!");
        start_time = ros::Time::now();
    } else {
        ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
        return false;
    }

    return true;
}


bool RiserInspection::takeoff_land(int task) {
    dji_sdk::DroneTaskControl droneTaskControl;

    droneTaskControl.request.task = task;

    drone_task_service.call(droneTaskControl);

    if (!droneTaskControl.response.result) {
        ROS_ERROR("takeoff_land fail");
        return false;
    }

    return true;
}

bool RiserInspection::obtain_control() {
    dji_sdk::SDKControlAuthority authority;
    authority.request.control_enable = 1;
    sdk_ctrl_authority_service.call(authority);

    if (!authority.response.result) {
        ROS_ERROR("obtain control failed!");
        return false;
    }

    return true;
}


void RiserInspection::step() {
    static int info_counter = 0;
    geometry_msgs::Vector3 localOffset;

    float speedFactor = 2;
    float yawThresholdInDeg = 2;

    float xCmd, yCmd, zCmd;

    localOffsetFromGpsOffset(localOffset, current_gps_, start_gps_);

    double xOffsetRemaining = target_offset_x - localOffset.x;
    double yOffsetRemaining = target_offset_y - localOffset.y;
    double zOffsetRemaining = target_offset_z - localOffset.z;

    double yawDesiredRad = DEG2RAD(target_yaw);
    double yawThresholdInRad = DEG2RAD(yawThresholdInDeg);
    double yawInRad = atti_Eul.Yaw();

    info_counter++;
    if (info_counter > 25) {
        info_counter = 0;
        ROS_INFO("-----x=%f, y=%f, z=%f, yaw=%f ...", localOffset.x, localOffset.y, localOffset.z, yawInRad);
        ROS_INFO("+++++dx=%f, dy=%f, dz=%f, dyaw=%f ...", xOffsetRemaining, yOffsetRemaining, zOffsetRemaining,
                 yawInRad - yawDesiredRad);
    }
    if (abs(xOffsetRemaining) >= speedFactor)
        xCmd = (xOffsetRemaining > 0) ? speedFactor : -1 * speedFactor;
    else
        xCmd = xOffsetRemaining;

    if (abs(yOffsetRemaining) >= speedFactor)
        yCmd = (yOffsetRemaining > 0) ? speedFactor : -1 * speedFactor;
    else
        yCmd = yOffsetRemaining;

    zCmd = start_local_position_.z + target_offset_z;


    /*!
     * @brief: if we already started breaking, keep break for 50 sample (1sec)
     *         and call it done, else we send normal command
     */

    if (break_counter > 50) {
        ROS_INFO("##### Route %d finished....", state);
        finished = true;
        return;
    } else if (break_counter > 0) {
        sensor_msgs::Joy controlVelYawRate;
        uint8_t flag = (DJISDK::VERTICAL_VELOCITY |
                        DJISDK::HORIZONTAL_VELOCITY |
                        DJISDK::YAW_RATE |
                        DJISDK::HORIZONTAL_GROUND |
                        DJISDK::STABLE_ENABLE);
        controlVelYawRate.axes.push_back(0);
        controlVelYawRate.axes.push_back(0);
        controlVelYawRate.axes.push_back(0);
        controlVelYawRate.axes.push_back(0);
        controlVelYawRate.axes.push_back(flag);

        ctrlBrakePub.publish(controlVelYawRate);
        break_counter++;
        return;
    } else //break_counter = 0, not in break stage
    {
        sensor_msgs::Joy controlPosYaw;


        controlPosYaw.axes.push_back(xCmd);
        controlPosYaw.axes.push_back(yCmd);
        controlPosYaw.axes.push_back(zCmd);
        controlPosYaw.axes.push_back(yawDesiredRad);
        ctrlPosYawPub.publish(controlPosYaw);
    }

    if (std::abs(xOffsetRemaining) < 0.5 &&
        std::abs(yOffsetRemaining) < 0.5 &&
        std::abs(zOffsetRemaining) < 0.5 &&
        std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad) {
        //! 1. We are within bounds; start incrementing our in-bound counter
        inbound_counter++;
    } else {
        if (inbound_counter != 0) {
            //! 2. Start incrementing an out-of-bounds counter
            outbound_counter++;
        }
    }

    //! 3. Reset withinBoundsCounter if necessary
    if (outbound_counter > 10) {
        ROS_INFO("##### Route %d: out of bounds, reset....", state);
        inbound_counter = 0;
        outbound_counter = 0;
    }

    if (inbound_counter > 50) {
        ROS_INFO("##### Route %d start break....", state);
        break_counter = 1;
    }

}

bool RiserInspection::set_local_position() {
    dji_sdk::SetLocalPosRef localPosReferenceSetter;
    set_local_pos_reference.call(localPosReferenceSetter);
    return localPosReferenceSetter.response.result;
}

void RiserInspection::flight_status_callback(const std_msgs::UInt8::ConstPtr &msg) {
    flight_status_ = msg->data;
}

void RiserInspection::display_mode_callback(const std_msgs::UInt8::ConstPtr &msg) {
    display_mode_ = msg->data;
}

void RiserInspection::attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr &msg) {
    current_atti_ = msg->quaternion;
    atti_Eul.Set(msg->quaternion.w, msg->quaternion.x, msg->quaternion.y, msg->quaternion.z);
}

void RiserInspection::local_position_callback(const geometry_msgs::PointStamped::ConstPtr &msg) {
    current_local_pos_ = msg->point;
}

void RiserInspection::gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    static ros::Time start_time = ros::Time::now();
    ros::Duration elapsed_time = ros::Time::now() - start_time;
    current_gps_ = *msg;

    // Down sampled to 50Hz loop
    if (elapsed_time > ros::Duration(0.02)) {
        start_time = ros::Time::now();

    }
}

void RiserInspection::rtk_callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_rtk_ = *msg;
}
//mission action
/*
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
*/
// init waypoint mission
/*
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
*/

std::vector<DJI::OSDK::WayPointSettings>
RiserInspection::createWayPoint(const std::vector<std::vector<std::string>> csv_file,
                                dji_sdk::MissionWaypointTask &waypointTask) {
    std::vector<DJI::OSDK::WayPointSettings> wp_list; // create a list (vector) containing waypoint structures

    // Push first waypoint as a initial position
    DJI::OSDK::WayPointSettings start_wp;
    setWaypointDefaults(&start_wp);
    start_wp.latitude = start_gps_.latitude;
    start_wp.longitude = start_gps_.longitude;
    start_wp.altitude = start_gps_.altitude;
    start_wp.yaw = start_atti_eul.Yaw();
    ROS_INFO("Waypoint created at (LLA): %f \t%f \t%f\theading:%f\n", start_gps_.latitude, start_gps_.longitude,
             start_gps_.altitude, start_atti_eul.Yaw());
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

void RiserInspection::uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings> &wp_list,
                                      int wp_number, dji_sdk::MissionWaypointTask &waypointTask) {
    dji_sdk::MissionWaypoint waypoint;
    DJI::OSDK::WayPointSettings wp = wp_list.at(wp_number);
    ROS_INFO("Waypoint created at (LLA): %f \t%f \t%f\n ", wp.latitude,
             wp.longitude, wp.altitude);
    waypoint.latitude = wp.latitude;
    waypoint.longitude = wp.longitude;
    waypoint.altitude = wp.altitude;
    waypoint.damping_distance = 0;
    waypoint.target_yaw = 0;
    waypoint.target_gimbal_pitch = 0;
    waypoint.turn_mode = 0;
    waypoint.has_action = 0;
    waypointTask.mission_waypoint.push_back(waypoint);
}


void RiserInspection::setWaypointDefaults(DJI::OSDK::WayPointSettings *wp) {
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

void RiserInspection::setWaypointInitDefaults(dji_sdk::MissionWaypointTask &waypointTask) {
    waypointTask.velocity_range = 2; // Maximum speed joystick input(2~15m)
    waypointTask.idle_velocity = 2; //Cruising Speed (without joystick input, no more than vel_cmd_range)
    waypointTask.action_on_finish = dji_sdk::MissionWaypointTask::FINISH_NO_ACTION;
    waypointTask.mission_exec_times = 1;
    waypointTask.yaw_mode = dji_sdk::MissionWaypointTask::YAW_MODE_WAYPOINT;
    waypointTask.trace_mode = dji_sdk::MissionWaypointTask::TRACE_POINT;
    waypointTask.action_on_rc_lost = dji_sdk::MissionWaypointTask::ACTION_FREE;
    waypointTask.gimbal_pitch_mode = dji_sdk::MissionWaypointTask::GIMBAL_PITCH_FREE;
}


