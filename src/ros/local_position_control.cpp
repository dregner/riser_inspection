//
// Created by vant3d on 16/12/2021.
//

#include <local_position_control.h>

LocalController::LocalController() {
    subscribing(nh_);
}

LocalController::~LocalController() = default;

void LocalController::subscribing(ros::NodeHandle &nh) {
    std::string gps_topic, rtk_topic, attitude_topic, height_topic, local_pos_topic;

    //! Topic parameters
    nh.param("/riser_inspection/gps_topic", gps_topic, std::string("/dji_sdk/gps_position"));
    nh.param("/riser_inspection/rtk_topic", rtk_topic, std::string("/dji_sdk/rtk_position"));
    nh.param("/riser_inspection/attitude_topic", attitude_topic, std::string("/dji_sdk/attitude"));
    nh.param("/riser_inspection/height_takeoff", height_topic, std::string("/dji_sdk/height_above_takeoff"));
    nh.param("/riser_inspection/local_position_topic", local_pos_topic, std::string("/dji_sdk/local_position"));
    nh.param("/riser_inspection/horizontal_error", h_error, 0.2);
    nh.param("/riser_inspection/vertical_error", v_error, 0.1);
    nh.param("/riser_inspeciton/use_rtk", use_rtk, false);
    //! Subscribe topics
    gps_sub = nh.subscribe<sensor_msgs::NavSatFix>(gps_topic, 1, &LocalController::gps_callback, this);
    rtk_sub = nh.subscribe<sensor_msgs::NavSatFix>(rtk_topic, 1, &LocalController::rtk_callback, this);
    attitude_sub = nh.subscribe<geometry_msgs::QuaternionStamped>(attitude_topic, 1,
                                                                  &LocalController::attitude_callback, this);
    local_pos_sub = nh.subscribe<geometry_msgs::PointStamped>(local_pos_topic, 1,
                                                              &LocalController::local_position_callback, this);
    height_sub = nh.subscribe<std_msgs::Float32>(height_topic, 1, &LocalController::height_callback, this);

    //! Publish topics
    ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
    velocityPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);
    gimbalAnglePub = nh.advertise<dji_sdk::Gimbal>("/dji_sdk/gimbal_angle_cmd", 10);

    //! Service topics
    sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
    set_local_pos_reference = nh.serviceClient<dji_sdk::SetLocalPosRef>("dji_sdk/set_local_pos_ref");
    camera_action_service = nh.serviceClient<dji_sdk::CameraAction>("dji_sdk/camera_action");
    stereo_v3d_service = nh.serviceClient<stereo_vant::PointGray>("/stereo_point_grey/take_picture");
    ROS_INFO("Subscribed Complet");

    local_position_service = nh.advertiseService("/local_position/set_position",
                                                 &LocalController::local_pos_service_cb, this);
    local_velocity_service = nh.advertiseService("/local_position/set_velocity",
                                                 &LocalController::local_velocity_service_cb,
                                                 this);

    start_mission_service = nh.advertiseService("/local_position/start_mission",
                                                &LocalController::start_mission_service_cb, this);
    horizontal_point_service = nh.advertiseService("/local_position/horizontal_point",
                                                   &LocalController::horizontal_pt_service_cb, this);


}

bool LocalController::obtain_control(bool ask) {
    dji_sdk::SDKControlAuthority authority;
    authority.request.control_enable = ask;
    sdk_ctrl_authority_service.call(authority);

    if (!authority.response.result) {
        ROS_ERROR("Do not understood");
        return false;
    }
    ROS_INFO("SDK Controlling: %s", ask ? "True" : "False");
    return true;
}

bool LocalController::local_velocity_service_cb(riser_inspection::LocalVelocity::Request &req,
                                                riser_inspection::LocalVelocity::Response &res) {
    try {
        ROS_INFO("Set velocity: x - %f m/s, y - %f m/s, z - %f m/s, yaw - %f rad/s", req.v_x, req.v_y, req.v_z,
                 req.v_yaw);
        res.result = local_position_velocity(req.v_x, req.v_y, req.v_z, req.v_yaw);
        return res.result;
    } catch (ros::Exception &e) {
        ROS_ERROR("ROS error %s", e.what());
        return false;
    }
}

bool LocalController::local_pos_service_cb(riser_inspection::LocalPosition::Request &req,
                                           riser_inspection::LocalPosition::Response &res) {
    try {
        if (LocalController::obtain_control(true)) {
            LocalController::setTarget(req.x, req.y, 2 * req.z, DEG2RAD(2 * req.yaw));
            ROS_INFO("Received points x: %f, y: %f, z: %f, yaw: %f", req.x, req.y, req.z, req.yaw);
            doing_mission = true;
            return res.result = true;
        } else {
            ROS_ERROR("Did not get Control Authority");
            return res.result = false;
        }
    }
    catch (ros::Exception &e) {
        ROS_ERROR("ROS error %s", e.what());
        return res.result = false;
    }
}

bool LocalController::start_mission_service_cb(riser_inspection::StartMission::Request &req,
                                               riser_inspection::StartMission::Response &res) {

    doing_mission = true;
    use_wp_list = true;
    take_photo = req.take_photo;
    if(req.use_stereo){
        use_stereo = req.use_stereo;
        stereo_vant::PointGray stereoAction;
        stereo_voo++;
        stereoAction.request.reset_counter = true;
        stereoAction.request.file_path = pathGenerator.getFileName() + "/stereo_voo" + std::to_string(stereo_voo);
        stereoAction.request.file_name = "stereo_vant";
        stereo_v3d_service.call(stereoAction);
    }else {
        LocalController::set_gimbal_angles(0, 0, 0);
    }
    LocalController::local_position_velocity(0.5,0.5,0.2,1);
    generate_WP(req.control_type);
    if (req.control_type == 4) {
        type_mission = 0; // full mision
        return res.result = LocalController::obtain_control(true);
    }
    if (req.control_type == 3) {
        type_mission = 1;
        return res.result = true;
    }
}

bool LocalController::horizontal_pt_service_cb(riser_inspection::hPoint::Request &req,
                                               riser_inspection::hPoint::Response &res) {
    ROS_INFO("Received new horizontal point number %i", req.number);
    int wp_number = req.number - 1;

    LocalController::setTarget(waypoint_l[wp_number][1], waypoint_l[wp_number][2],  2*rpa_height,
                               DEG2RAD(waypoint_l[wp_number][3]));
    ROS_INFO("Set new H point [%f, %f, %f] @ %f", waypoint_l[wp_number][1], waypoint_l[wp_number][2],
             rpa_height, waypoint_l[wp_number][3]/2);
    doing_mission = true;
    type_mission = 1;
    return res.result = LocalController::obtain_control(true);

}

void LocalController::height_callback(const std_msgs::Float32::ConstPtr &msg) {
    rpa_height = msg->data;
}

void LocalController::gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_gps = *msg;
    if (first_time && !use_rtk) {
        ROS_INFO("Set Local posiition: %s", set_local_position() ? "true" : "false");
        ROS_INFO("RTK? %s", (use_rtk ? "true" : "false"));
        ROS_INFO("Current position X: %f , Y: %f, Z: %f", current_local_pos.point.x, current_local_pos.point.y,
                 (use_rtk ? current_rtk.altitude : current_gps.altitude));
        first_time = false;
    }

}

void LocalController::rtk_callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_rtk = *msg;
    if (first_time && use_rtk) {
        ROS_INFO("Set Local posiition: %s", set_local_position() ? "true" : "false");
        ROS_INFO("RTK? %s", (use_rtk ? "true" : "false"));
        ROS_INFO("Current position X: %f , Y: %f, Z: %f", current_local_pos.point.x, current_local_pos.point.y,
                 (use_rtk ? current_rtk.altitude : current_gps.altitude));
        first_time = false;
    }

}


void LocalController::attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr &msg) {
    current_atti = *msg;
    current_atti_euler.Set(current_atti.quaternion.w, current_atti.quaternion.x, current_atti.quaternion.y,
                           current_atti.quaternion.z);

}

void LocalController::local_position_callback(const geometry_msgs::PointStamped::ConstPtr &msg) {
    current_local_pos = *msg;
    use_wp_list ? elapse_control_mission(doing_mission, type_mission) : elapse_control(doing_mission);
}


void LocalController::elapse_control(bool mission) {
    static ros::Time start_time = ros::Time::now();
    ros::Duration elapsed_time = ros::Time::now() - start_time;
    double xCmd, yCmd, zCmd, yawCmd;
    if (mission) {
        if (elapsed_time > ros::Duration(1 / 20)) {
            start_time = ros::Time::now();
            local_position_ctrl(xCmd, yCmd, zCmd, yawCmd);
        }
    }
}

void LocalController::elapse_control_mission(bool mission, int type_mission) {
    static ros::Time start_time = ros::Time::now();
    ros::Duration elapsed_time = ros::Time::now() - start_time;
    double xCmd, yCmd, zCmd, yawCmd;
    if (mission) {
        if (type_mission == 0) {
            if (elapsed_time > ros::Duration(1 / 20)) {
                start_time = ros::Time::now();
                local_position_ctrl_mission(xCmd, yCmd, zCmd, yawCmd, wp_n);
            }
        }
        if (type_mission == 1) {
            if (elapsed_time > ros::Duration(1 / 20)) {
                start_time = ros::Time::now();

                local_position_ctrl_semi_mission(xCmd, yCmd, zCmd, yawCmd, wp_n);
            }
        }
    }
}

void LocalController::setTarget(float x, float y, float z, float yaw) {
    target_offset[0] = x;
    target_offset[1] = y;
    target_offset[2] = z;
    target_offset[3] = yaw;
}

bool LocalController::set_local_position() {
    dji_sdk::SetLocalPosRef localPosReferenceSetter;
    set_local_pos_reference.call(localPosReferenceSetter);

    return (bool) localPosReferenceSetter.response.result;
}

bool LocalController::local_position_velocity(float vx, float vy, float vz, float vyaw) {
    sensor_msgs::Joy vel_PosYaw;
    uint8_t flag = (DJISDK::VERTICAL_VELOCITY |
                    DJISDK::HORIZONTAL_VELOCITY |
                    DJISDK::YAW_RATE |
                    DJISDK::HORIZONTAL_GROUND |
                    DJISDK::STABLE_ENABLE);
    vel_PosYaw.axes.push_back(vx);
    vel_PosYaw.axes.push_back(vy);
    vel_PosYaw.axes.push_back(vz);
    vel_PosYaw.axes.push_back(vyaw);
    vel_PosYaw.axes.push_back(flag);
    velocityPosYawPub.publish(vel_PosYaw);
    ros::spinOnce();
    ROS_INFO("Changed velocity: [%f, %f, %f, %f]", vx, vy, vz, vyaw);
    return true;

}

void LocalController::set_gimbal_angles(float roll, float pitch, float yaw) {
    dji_sdk::Gimbal gimbal_angle_data;
    gimbal_angle_data.mode |= 0;
    gimbal_angle_data.mode |= 1;
    gimbal_angle_data.mode |= 0 << 1;
    gimbal_angle_data.mode |= 1 << 2;
    gimbal_angle_data.mode |= 1 << 3;
    gimbal_angle_data.ts    = 2;
    gimbal_angle_data.roll  = DEG2RAD(roll);
    gimbal_angle_data.pitch = DEG2RAD(pitch);
    gimbal_angle_data.yaw   = DEG2RAD(yaw);

    gimbalAnglePub.publish(gimbal_angle_data);
    // Give time for gimbal to sync
    sleep(2);
}

void LocalController::local_position_ctrl(double &xCmd, double &yCmd, double &zCmd, double &yawCmd) {
    xCmd = target_offset[0] - current_local_pos.point.x;
    yCmd = target_offset[1] - current_local_pos.point.y;
    zCmd = target_offset[2] - (use_rtk ? current_rtk.altitude : rpa_height);
    yawCmd = target_offset[3] - current_atti_euler.Yaw();

    sensor_msgs::Joy controlPosYaw;
    controlPosYaw.axes.push_back(xCmd);
    controlPosYaw.axes.push_back(yCmd);
    controlPosYaw.axes.push_back(zCmd);
    controlPosYaw.axes.push_back(yawCmd);
    ctrlPosYawPub.publish(controlPosYaw);
    /** 0.1m or 10cms is the minimum error to reach target in x y and z axes.
     This error threshold will have to change depending on aircraft/payload/wind conditions. */
    float yaw_result = target_offset[3] / 2 - current_atti_euler.Yaw();
    float z_result = target_offset[2] / 2 - (use_rtk ? current_rtk.altitude : rpa_height);

    if ((std::abs(xCmd) < h_error) &&
        (std::abs(yCmd) < h_error) &&
        (std::abs(target_offset[2] / 2 - (use_rtk ? current_rtk.altitude : rpa_height)) < v_error) &&
        (std::abs(target_offset[3] / 2 - current_atti_euler.Yaw()) < DEG2RAD(2))) {
        ROS_INFO("(%f, %f, %f) m @ %f deg target complete",
                 current_local_pos.point.x, current_local_pos.point.y, rpa_height, RAD2DEG(current_atti_euler.Yaw()));
        LocalController::obtain_control(false);
        doing_mission = false;
    }
}

void
LocalController::local_position_ctrl_mission(double &xCmd, double &yCmd, double &zCmd, double &yawCmd, int waypoint_n) {
    xCmd = waypoint_l[waypoint_n][1] - current_local_pos.point.x;
    yCmd = waypoint_l[waypoint_n][2] - current_local_pos.point.y;
    zCmd = waypoint_l[waypoint_n][3] - (use_rtk ? current_rtk.altitude : rpa_height);
    yawCmd = DEG2RAD(waypoint_l[waypoint_n][4]) - current_atti_euler.Yaw();
    sensor_msgs::Joy controlPosYaw;
    controlPosYaw.axes.push_back(xCmd);
    controlPosYaw.axes.push_back(yCmd);
    controlPosYaw.axes.push_back(zCmd);
    controlPosYaw.axes.push_back(yawCmd);
    ctrlPosYawPub.publish(controlPosYaw);
    /** 0.1m or 10cms is the minimum error to reach target in x y and z axes.
     This error threshold will have to change depending on aircraft/payload/wind conditions. */
    float yaw_result = waypoint_l[waypoint_n][2] / 2 - current_atti_euler.Yaw();
    float z_result = waypoint_l[waypoint_n][3] / 2 - (use_rtk ? current_rtk.altitude : rpa_height);

    if ((std::abs(xCmd) < h_error) &&
        (std::abs(yCmd) < h_error) &&
        (std::abs(waypoint_l[waypoint_n][3] / 2 - (use_rtk ? current_rtk.altitude : rpa_height)) < v_error) &&
        (std::abs(DEG2RAD(waypoint_l[waypoint_n][4]) / 2 - current_atti_euler.Yaw()) < DEG2RAD(2))) {
        ROS_INFO("WP %i - (%f, %f, %f) m @ %f deg target complete", wp_n,
                 current_local_pos.point.x, current_local_pos.point.y, rpa_height, RAD2DEG(current_atti_euler.Yaw()));
        if (waypoint_n >= (int) waypoint_l.size() - 1) {
            LocalController::obtain_control(false);
            doing_mission = false;
            use_wp_list = false;
        } else {
            if(take_photo){
                LocalController::acquire_photo(use_stereo);
            }
            ros::Duration(2).sleep();
            wp_n++;
        }
    }
}

void
LocalController::local_position_ctrl_semi_mission(double &xCmd, double &yCmd, double &zCmd, double &yawCmd,
                                                  int waypoint_n) {
    xCmd = target_offset[0] - current_local_pos.point.x;
    yCmd = target_offset[1] - current_local_pos.point.y;
    zCmd = target_offset[2] - rpa_height;
    yawCmd = DEG2RAD(target_offset[3]) - current_atti_euler.Yaw();
    sensor_msgs::Joy controlPosYaw;
    controlPosYaw.axes.push_back(xCmd);
    controlPosYaw.axes.push_back(yCmd);
    controlPosYaw.axes.push_back(zCmd);
    controlPosYaw.axes.push_back(yawCmd);
    ctrlPosYawPub.publish(controlPosYaw);
    /** 0.1m or 10cms is the minimum error to reach target in x y and z axes.
     This error threshold will have to change depending on aircraft/payload/wind conditions. */

    if ((std::abs(xCmd) < h_error) &&
        (std::abs(yCmd) < h_error) &&
        (std::abs(target_offset[2] / 2 - rpa_height) < v_error) &&
        (std::abs(DEG2RAD(target_offset[3]) / 2 - current_atti_euler.Yaw()) < DEG2RAD(2))) {
        ROS_INFO("REACHED WP - [%f. %f, %f] m @ %f deg", current_local_pos.point.x, current_local_pos.point.y,
                 rpa_height, RAD2DEG(current_atti_euler.Yaw()));
        doing_mission = false;
        LocalController::obtain_control(false);
        use_wp_list = false;
    }
}

bool LocalController::acquire_photo(bool stereo) {
    if (stereo) {
        stereo_vant::PointGray stereoAction;
        stereoAction.request.reset_counter = false;
        stereoAction.request.file_path = pathGenerator.getFileName() + "/stereo_voo" + std::to_string(stereo_voo);
        stereoAction.request.file_name = "stereo_vant";
        stereo_v3d_service.call(stereoAction);

    } else {
        dji_sdk::CameraAction cameraAction;
        cameraAction.request.camera_action = 0;
        camera_action_service.call(cameraAction);
        if (cameraAction.response.result) {
            ROS_INFO("Picture %i - %b", camera_count, cameraAction.response.result);
            camera_count++;
        }
        return cameraAction.response.result;
    }
}
bool LocalController::generate_WP(int csv_type) {
    /** Initial setting and parameters to generate trajectory*/
    pathGenerator.reset(); // clear pathGen
    waypoint_l.clear(); // clear waypoint list]
    wp_n = 0;
    int riser_distance, riser_diameter, h_points, v_points, delta_h, delta_v;
    std::string root_directory;
    nh_.param("/riser_inspection/riser_distance", riser_distance, 10);
    nh_.param("/riser_inspection/riser_diameter", riser_diameter, 300);
    nh_.param("/riser_inspection/horizontal_points", h_points, 5);
    nh_.param("/riser_inspection/vertical_points", v_points, 8);
    nh_.param("/riser_inspection/delta_H", delta_h, 15);
    nh_.param("/riser_inspection/delta_V", delta_v, 300);
    nh_.param("/riser_inspection/root_directory", root_directory, std::string("/home/vant3d/Documents"));

    /** Setting intial parameters to create waypoints */
    pathGenerator.setFolderName(root_directory);
    pathGenerator.setInspectionParam(riser_distance, (float) riser_diameter, h_points, v_points, delta_h,
                                     (float) delta_v);
    /** Define start positions to create waypoints */
    pathGenerator.setInitCoord_XY(current_local_pos.point.x, current_local_pos.point.y, rpa_height,
                                  (int) RAD2DEG(current_atti_euler.Yaw()));
    try {
        pathGenerator.createInspectionPoints(csv_type); // type 4 refers to XYZ YAW waypoints
        ROS_WARN("Waypoints created at %s/%s", pathGenerator.getFolderName().c_str(),
                 pathGenerator.getFileName().c_str());

        std::vector<std::vector<std::string>> csv_file = pathGenerator.read_csv(
                pathGenerator.getFolderName() + "/" + pathGenerator.getFileName(), ",");
        //!Vector of XYZ and Yaw positions to generate a fully automated control
        if (csv_type == 4) {
            for (int k = 1; k < (int) csv_file.size(); k++) {
                waypoint_l.push_back(
                        {(float) std::stoi(csv_file[k][0]), std::stof(csv_file[k][1]), std::stof(csv_file[k][2]),
                         2 * std::stof(csv_file[k][3]), 2 * std::stof(csv_file[k][4])});
            }
            waypoint_l.push_back(
                    {(float) csv_file.size(), (float) current_local_pos.point.x, (float) current_local_pos.point.y,
                     2 * rpa_height, 2 * (float) RAD2DEG(current_atti_euler.Yaw())});
            return true;

        }
        //! Vector of XY and Yaw positions to generate a semi-automated control, sending RPA to horizontal position and
        //! the SDK controller release to pilot change manually its vertical position
        if (csv_type == 3) {
            for (int k = 1; k < (int) csv_file.size(); k++) {
                waypoint_l.push_back(
                        {(float) std::stoi(csv_file[k][0]), std::stof(csv_file[k][1]), std::stof(csv_file[k][2]),
                         2*std::stof(csv_file[k][3])});
            }
            waypoint_l.push_back(
                    {(float) csv_file.size(), (float) current_local_pos.point.x, (float) current_local_pos.point.y,
                     2 * (float) RAD2DEG(current_atti_euler.Yaw())});
            ROS_INFO("Created %i horizontal points", h_points);
            ROS_INFO("To send RPA to the points use service /riser_inspection/horizontal_point");
            return true;
        }
    } catch (ros::Exception &e) {
        ROS_WARN("ROS error %s", e.what());
        return false;
    }
}

