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
    nh.param("/riser_inspection/attitude_topic", attitude_topic, std::string("/dji_sdk/attitude"));
    nh.param("/riser_inspection/local_position_topic", local_pos_topic, std::string("/dji_sdk/local_position"));
    // Subscribe topics
    gps_sub = nh.subscribe<sensor_msgs::NavSatFix>(gps_topic, 1, &LocalController::gps_callback, this);
    attitude_sub = nh.subscribe<geometry_msgs::QuaternionStamped>(attitude_topic, 1,
                                                                  &LocalController::attitude_callback, this);
    local_pos_sub = nh.subscribe<geometry_msgs::PointStamped>(local_pos_topic, 1,
                                                              &LocalController::local_position_callback, this);
    // Publish topics
    ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
    // Service topics
    sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
    set_local_pos_reference = nh.serviceClient<dji_sdk::SetLocalPosRef>("dji_sdk/set_local_pos_ref");
    camera_action_service = nh.serviceClient<dji_sdk::CameraAction>("/dji_sdk/camera_action");
    ROS_INFO("Subscribed Complet");

    local_position_service = nh.advertiseService("/local_position_ctrl/get_position",
                                                 &LocalController::local_pos_service_cb, this);

}

bool LocalController::obtain_control(bool ask) {
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

bool LocalController::local_pos_service_cb(riser_inspection::LocalPosition::Request &req,
                                           riser_inspection::LocalPosition::Response &res) {
    try {
        if (LocalController::obtain_control(true)) {
            LocalController::setTarget(req.x, req.y, 2 * req.z, DEG2RAD(req.yaw - 90));
            ROS_INFO("Received points x: %f, y: %f, z: %f, yaw: %f", req.x, req.y, req.z, (req.yaw - 90));
            doing_mission = true;
            return res.result = true;
        } else {
            ROS_WARN("Do not get Control Authority");
            return res.result = false;
        }
    }
    catch (ros::Exception &e) {
        ROS_WARN("ROS error %s", e.what());
        return res.result = false;
    }


}

void LocalController::gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_gps = *msg;
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
    double xCmd, yCmd, zCmd;
    // Down sampled to 50Hz loop
    if (doing_mission) {
        if (elapsed_time > ros::Duration(0.02)) {
            start_time = ros::Time::now();
            local_position_ctrl(xCmd, yCmd, zCmd);
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

void LocalController::local_position_ctrl(double &xCmd, double &yCmd, double &zCmd) {
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
        LocalController::obtain_control(false);
        doing_mission = false;

    }
}

