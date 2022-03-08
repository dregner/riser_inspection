//
// Created by vant3d on 16/12/2021.
//

#include <local_position_control.h>

LocalController::LocalController() {
    subscribing(nh_);
}

LocalController::~LocalController() = default;

void LocalController::subscribing(ros::NodeHandle &nh) {
    std::string gps_topic, rtk_topic, attitude_topic, local_pos_topic;

    // Topic parameters
    nh.param("/riser_inspection/gps_topic", gps_topic, std::string("/dji_sdk/gps_position"));
    nh.param("/riser_inspection/rtk_topic", rtk_topic, std::string("/dji_sdk/rtk_position"));
    nh.param("/riser_inspection/attitude_topic", attitude_topic, std::string("/dji_sdk/attitude"));
    nh.param("/riser_inspection/local_position_topic", local_pos_topic, std::string("/dji_sdk/local_position"));

    // Subscribe topics
    gps_sub = nh.subscribe<sensor_msgs::NavSatFix>(gps_topic, 1, &LocalController::gps_callback, this);
    rtk_sub = nh.subscribe<sensor_msgs::NavSatFix>(rtk_topic, 1, &LocalController::rtk_callback, this);
    attitude_sub = nh.subscribe<geometry_msgs::QuaternionStamped>(attitude_topic, 1,
                                                                  &LocalController::attitude_callback, this);
    local_pos_sub = nh.subscribe<geometry_msgs::PointStamped>(local_pos_topic, 1,
                                                              &LocalController::local_position_callback, this);

    // Publish topics
    ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
    velocityPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10);
    // Service topics
    sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
    set_local_pos_reference = nh.serviceClient<dji_sdk::SetLocalPosRef>("dji_sdk/set_local_pos_ref");
    camera_action_service = nh.serviceClient<dji_sdk::CameraAction>("dji_sdk/camera_action");
    ROS_INFO("Subscribed Complet");

    local_position_service = nh.advertiseService("/local_position/set_position",
                                                 &LocalController::local_pos_service_cb, this);
    local_velocity_service = nh.advertiseService("/local_position/set_velocity", &LocalController::local_velocity_cb,
                                                 this);


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

bool LocalController::local_velocity_cb(riser_inspection::LocalVelocity::Request &req,
                                        riser_inspection::LocalVelocity::Response &res) {

    try {
        res.result = local_position_velocity(req.v_x, req.v_y, req.v_z, req.v_yaw);
        return res.result;
        ROS_INFO("Set velocity: x - %f m/s, y - %f m/s, z - %f m/s, yaw - %f rad/s", req.v_x, req.v_y, req.v_z,
                 req.v_yaw);
    } catch (ros::Exception &e) {
        ROS_ERROR("ROS error %s", e.what());
        return false;
    }
}

bool LocalController::local_pos_service_cb(riser_inspection::LocalPosition::Request &req,
                                           riser_inspection::LocalPosition::Response &res) {
    try {
        if (LocalController::obtain_control(true)) {
            LocalController::setTarget(req.x, req.y, 2 * req.z, DEG2RAD(req.yaw));
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

void LocalController::gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_gps = *msg;
    if (first_time) {
        ROS_INFO("Set Local posiition: %s", set_local_position() ? "true" : "false");
        ROS_INFO("RTK? %s", (use_rtk ? "true" : "false"));
        ROS_INFO("Current position X: %f , Y: %f, Z: %f", current_local_pos.point.x, current_local_pos.point.y,
                 (use_rtk ? current_rtk.altitude : current_gps.altitude));
        first_time = false;
    }

}

void LocalController::rtk_callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_rtk = *msg;

}


void LocalController::attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr &msg) {
    current_atti = *msg;
    current_atti_euler.Set(current_atti.quaternion.w, current_atti.quaternion.x, current_atti.quaternion.y,
                           current_atti.quaternion.z);

}

void LocalController::local_position_callback(const geometry_msgs::PointStamped::ConstPtr &msg) {
    static ros::Time start_time = ros::Time::now();
    ros::Duration elapsed_time = ros::Time::now() - start_time;
    current_local_pos = *msg;
    double xCmd, yCmd, zCmd, yawCmd;
    // Down sampled to 20Hz loop
    if (doing_mission) {
        if (elapsed_time > ros::Duration(1 / 20)) {
            start_time = ros::Time::now();
            local_position_ctrl(xCmd, yCmd, zCmd, yawCmd);
        }
    }
}

void LocalController::setTarget(float x, float y, float z, float yaw) {
    target_offset_x = x;
    target_offset_y = y;
    target_offset_z = z;
    target_yaw = yaw;
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
    return true;
}

void LocalController::local_position_ctrl(double &xCmd, double &yCmd, double &zCmd, double &yawCmd) {
    xCmd = target_offset_x - current_local_pos.point.x;
    yCmd = target_offset_y - current_local_pos.point.y;
    zCmd = target_offset_z - (use_rtk ? current_rtk.altitude : current_gps.altitude); // - current_local_pos.point.z;
    yawCmd = target_yaw - current_atti_euler.Yaw();
    sensor_msgs::Joy controlPosYaw;
    controlPosYaw.axes.push_back(xCmd);
    controlPosYaw.axes.push_back(yCmd);
    controlPosYaw.axes.push_back(zCmd);
    controlPosYaw.axes.push_back(yawCmd);
    ctrlPosYawPub.publish(controlPosYaw);
    float zcmd = (target_offset_z/2 - current_gps.altitude);
    float yawcmd = target_yaw - current_atti_euler.Yaw();
    // 0.1m or 10cms is the minimum error to reach target in x y and z axes.
    // This error threshold will have to change depending on aircraft/payload/wind conditions.
    if ((std::abs(xCmd) < 0.1) &&
        (std::abs(yCmd) < 0.1) &&
        (std::abs(zcmd) < 0.1) &&
        (std::abs(yawcmd) < 0.0175)) {
        ROS_INFO("(%f, %f, %f) m @ %f deg target complete",
                 target_offset_x, target_offset_y, target_offset_z / 2, RAD2DEG(target_yaw));
        LocalController::obtain_control(false);
        doing_mission = false;

    }
}

