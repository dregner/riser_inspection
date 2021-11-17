#include <riser_inspection_flight_control.hh>


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

        gpsSub = nh.subscribe<sensor_msgs::NavSatFix>(gps_topic_, 1, &RiserInspection::gps_callback, this);
        rtkSub = nh.subscribe<sensor_msgs::NavSatFix>(rtk_topic_, 1, &RiserInspection::rtk_callback, this);
        attiSub = nh.subscribe<geometry_msgs::QuaternionStamped>(attitude_topic_, 1,
                                                                 &RiserInspection::attitude_callback, this);

        ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);


        ROS_INFO("Subscribe complet");
    } catch (ros::Exception &e) {
        ROS_ERROR("Subscribe topics exception: %s", e.what());
    }
}

void RiserInspection::initServices(ros::NodeHandle &nh) {
    try {
        generate_pathway_service = nh.advertiseService("riser_inspection/waypoint_generator",
                                                       &RiserInspection::pathGen_serviceCB, this);
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
    use_rtk = req.use_rtk;
    // Define where comes the initial value
    if (!use_rtk) { start_location_gnss_ = current_gps_; }
    else { start_location_gnss_ = current_rtk_; }
    start_atti_ = current_atti_;
    start_atti_eul.Set(start_atti_.w, start_atti_.x, start_atti_.y, start_atti_.z);
    // Define start positions to create waypoints
    pathGenerator.setInitCoord(req.riser_distance, float(req.riser_diameter / 1000), start_location_gnss_.latitude,
                               start_location_gnss_.longitude, start_location_gnss_.altitude, start_atti_eul.Yaw());
    pathGenerator.setInspectionParam(req.horizontal_number, req.vertical_number, req.delta_angle, req.delta_height);
    ROS_INFO("Set initial values");

    try {
        /// csv_type: 1 - ugcs complete; 2 - ugcs EMU; 3 - UgCS TOP/BOTTOM; 4 - DJI
        pathGenerator.createInspectionPoints(3);
        ROS_INFO("Waypoints created at %s/%s", pathGenerator.getFolderName().c_str(),
                 pathGenerator.getFileName().c_str());
        set_local_position();
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
        startMission = true;

        if (!obtain_control()) {
            ROS_INFO("Cannot get Authority Control");
            return res.result = false;
        } else {
            std::vector<std::vector<std::string>> waypoint_list = pathGenerator.read_csv(
                    pathGenerator.getFolderName() + "/" + pathGenerator.getFileName(), ",");
            ROS_INFO("Starting Waypoint Mission");

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

    float speedFactor = 0.3;
    float yawThresholdInDeg = 2;

    float xCmd, yCmd, zCmd;

    localOffsetFromGpsOffset(localOffset, current_gps_, start_location_gnss_);

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
    if (startMission) {
        // Down sampled to 50Hz loop
        if (elapsed_time > ros::Duration(0.02)) {
            start_time = ros::Time::now();
            if(!finished) {
                step();
            }
            else{
                if (!use_rtk) { start_location_gnss_ = current_gps_; }
                else { start_location_gnss_ = current_rtk_; }
                start_local_position_ = current_local_pos_;
                setTarget(waypoint_list[state][1], waypoint_list[state][2], );
            }
            if (state > waypoint_list[0].size()) { finished = true; }
            else { state++; }
        }
    }
}

void RiserInspection::rtk_callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_rtk_ = *msg;
}

void RiserInspection::setTarget(float x, float y, float z, float yaw) {
    target_offset_x = x;
    target_offset_y = y;
    target_offset_z = z;
    target_yaw = yaw;
}

