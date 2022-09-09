//
// Created by vant3d on 16/12/2021.
//

#include <local_position_control.h>
#include <dji_control.hpp>

LocalController::LocalController() {
    subscribing(nh_);
}

LocalController::~LocalController() = default;

void LocalController::subscribing(ros::NodeHandle &nh) {
    std::string gps_topic, rtk_topic, attitude_topic, height_topic, local_pos_topic;

    //! Topic parameters
    nh.param("/riser_inspection/gps_topic", gps_topic, std::string("/dji_osdk_sdk/gps_position"));
    nh.param("/riser_inspection/rtk_topic", rtk_topic, std::string("/dji_osdk_sdk/rtk_position"));
    nh.param("/riser_inspection/attitude_topic", attitude_topic, std::string("/dji_osdk_sdk/attitude"));
    nh.param("/riser_inspection/height_takeoff", height_topic, std::string("/dji_osdk_sdk/height_above_takeoff"));
    nh.param("/riser_inspection/local_position_topic", local_pos_topic, std::string("/dji_osdk_sdk/local_position"));
    //! Subscribe topics
//    gps_sub = nh.subscribe<sensor_msgs::NavSatFix>(gps_topic, 1, &LocalController::gps_callback, this);
//    rtk_sub = nh.subscribe<sensor_msgs::NavSatFix>(rtk_topic, 1, &LocalController::rtk_callback, this);
    attitude_sub = nh.subscribe<geometry_msgs::QuaternionStamped>(attitude_topic, 1,
                                                                  &LocalController::attitude_callback, this);
    local_pos_sub = nh.subscribe<geometry_msgs::PointStamped>(local_pos_topic, 1,
                                                              &LocalController::local_position_callback, this);
    height_sub = nh.subscribe<std_msgs::Float32>(height_topic, 1, &LocalController::height_callback, this);


    //! Service topics
    obtain_crl_authority_client = nh.serviceClient<dji_osdk_ros::ObtainControlAuthority>(
            "/obtain_release_control_authority");
    set_local_ref_client = nh.serviceClient<dji_osdk_ros::SetLocalPosRef>("/set_local_pos_reference");
    task_control_client = nh.serviceClient<dji_osdk_ros::FlightTaskControl>("/flight_task_control");
    gimbal_control_client = nh.serviceClient<dji_osdk_ros::GimbalAction>("/gimbal_task_control");
    //! Camera services
    camera_action_client = nh.serviceClient<dji_osdk_ros::CameraAction>("camera_action");
    sv3d_client = nh.serviceClient<stereo_vant::PointGray>("/stereo_point_grey/take_picture");
    ROS_INFO("Subscribed Complet");

    local_position_service = nh.advertiseService("/local_position/set_position",
                                                 &LocalController::local_pos_service_cb, this);

    start_mission_service = nh.advertiseService("/local_position/start_mission",
                                                &LocalController::start_mission_service_cb, this);


}

bool LocalController::obtain_control(bool ask) {
    dji_osdk_ros::ObtainControlAuthority authority;
    authority.request.enable_obtain = ask;
    obtain_crl_authority_client.call(authority);

    if (!authority.response.result) {
        ROS_ERROR("Can't get control authority");
        return authority.response.result;
    }
    ROS_INFO("SDK Controlling: %s", ask ? "True" : "False");
    return authority.response.result;
}


bool LocalController::local_pos_service_cb(riser_inspection::LocalPosition::Request &req,
                                           riser_inspection::LocalPosition::Response &res) {
    try {
        if (LocalController::obtain_control(true)) {
            ROS_INFO("Received points x: %f, y: %f, z: %f, yaw: %f", req.x, req.y, req.z, req.yaw);
            ROS_INFO("Threshold: %f m - %f deg", req.position_thresh, req.yaw_thresh);
            res.result = local_position_ctrl(req.x, req.y, req.z, req.yaw, req.position_thresh,
                                             req.yaw_thresh);
            return res.result;
        } else {
            ROS_ERROR("Did not get Control Authority");
            res.result = false;
        }
    }
    catch (ros::Exception &e) {
        ROS_ERROR("ROS error %s", e.what());
        res.result = false;
    }
    return res.result;
}

bool LocalController::start_mission_service_cb(riser_inspection::StartMission::Request &req,
                                               riser_inspection::StartMission::Response &res) {

    doing_mission = true;
    use_gimbal = req.use_gimbal;
    use_stereo = req.use_stereo;
    if (LocalController::set_local_position()) {
        if (use_stereo) {
            stereo_vant::PointGray stereoAction;
            stereo_voo++;
            stereoAction.request.reset_counter = true;
            stereoAction.request.file_path = pathGenerator.getFileName() + "/stereo_voo" + std::to_string(stereo_voo);
            stereoAction.request.file_name = "sv3d";
            sv3d_client.call(stereoAction);
        }
        if (use_gimbal) {
            LocalController::set_gimbal_angles(0, 0, 0);
        }
        generate_WP(2);
        res.result = LocalController::obtain_control(true);
    } else { res.result = false; }
    return res.result;
}


void LocalController::height_callback(const std_msgs::Float32::ConstPtr &msg) {
    rpa_height = msg->data;
}


void LocalController::attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr &msg) {
    current_atti = *msg;
    atti_matrix.setRotation(tf::Quaternion(current_atti.quaternion.x, current_atti.quaternion.y,
                                           current_atti.quaternion.z, current_atti.quaternion.w));
    tf::Matrix3x3 ROT = {0, -1, 0, -1, 0, 0, 0, 0, -1};
    atti_matrix *= ROT;
    atti_matrix.getRotation(q_atti_matrix);
    current_atti_euler.Set(q_atti_matrix.getW(), q_atti_matrix.getX(), q_atti_matrix.getY(), q_atti_matrix.getZ());

}

void LocalController::local_position_callback(const geometry_msgs::PointStamped::ConstPtr &msg) {
    current_local_pos = *msg;
    if (doing_mission) {
        local_position_ctrl_mission();
    }
}

void LocalController::gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_gps = *msg;
}

void LocalController::rtk_callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_rtk = *msg;
}

bool LocalController::set_local_position() {
    dji_osdk_ros::SetLocalPosRef localPosReferenceSetter;
    set_local_ref_client.call(localPosReferenceSetter);
    ROS_INFO("SET LOCAL POSITION TO [0,0,0]");
    return (bool) localPosReferenceSetter.response.result;
}

void LocalController::set_gimbal_angles(float roll, float pitch, float yaw) {
    dji_osdk_ros::GimbalAction gimbalAction;
    gimbalAction.request.is_reset = false;
    gimbalAction.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
    gimbalAction.request.rotationMode = 0;
    gimbalAction.request.pitch = pitch;
    gimbalAction.request.roll = roll;
    gimbalAction.request.yaw = yaw;
    gimbalAction.request.time = 0.5;
    gimbal_control_client.call(gimbalAction);

}

bool LocalController::local_position_ctrl(float xCmd, float yCmd, float zCmd, float yawCmd, float pos_thresh,
                                          float yaw_thresh) {
    dji_osdk_ros::FlightTaskControl control_task_point;
    control_task_point.request.task = dji_osdk_ros::FlightTaskControl::Request::TASK_POSITION_AND_YAW_CONTROL;
    control_task_point.request.joystickCommand.x = xCmd;
    control_task_point.request.joystickCommand.y = yCmd;
    control_task_point.request.joystickCommand.z = zCmd;
    control_task_point.request.joystickCommand.yaw = yawCmd;
    control_task_point.request.posThresholdInM = pos_thresh;
    control_task_point.request.yawThresholdInDeg = yaw_thresh;

    task_control_client.call(control_task_point);

    if (control_task_point.response.result) {
        ROS_INFO("(%f, %f, %f) m @ %f deg target complete",
                 current_local_pos.point.x, current_local_pos.point.y, rpa_height,
                 -RAD2DEG(current_atti_euler.Yaw()));
        LocalController::obtain_control(false);
        return control_task_point.response.result;
    } else { return control_task_point.response.result; }
}

void
LocalController::local_position_ctrl_mission() {
    dji_osdk_ros::FlightTaskControl control_task_mission;
    control_task_mission.request.task = dji_osdk_ros::FlightTaskControl::Request::TASK_POSITION_AND_YAW_CONTROL;
    control_task_mission.request.joystickCommand.x = waypoint_list[wp_n][1];
    control_task_mission.request.joystickCommand.y = waypoint_list[wp_n][2];
    control_task_mission.request.joystickCommand.z = waypoint_list[wp_n][3];
    control_task_mission.request.joystickCommand.yaw = waypoint_list[wp_n][4];
    control_task_mission.request.posThresholdInM = pos_error;
    control_task_mission.request.yawThresholdInDeg = yaw_error;

    task_control_client.call(control_task_mission);

    if (control_task_mission.response.result) {
        ROS_INFO("WP %i - %f m @ %f deg target complete", wp_n + 1, rpa_height,
                 -RAD2DEG(current_atti_euler.Yaw()));

        if (wp_n < (int) waypoint_list.size()) {
            if (use_gimbal || use_stereo) {
                LocalController::acquire_photo(use_stereo, use_gimbal);
                ros::Duration(4).sleep();
            }
            wp_n++;
        } else {
            doing_mission = false;
            ROS_WARN("BACK TO INITIAL POSITION");
            if (local_position_ctrl((float) -current_local_pos.point.x, (float) -current_local_pos.point.y,
                                    (float) -current_local_pos.point.z,
                                    (float) -RAD2DEG(current_atti_euler.Yaw()), pos_error, yaw_error)) {
                ROS_INFO("MISSION FINISHED");
            } else { ROS_ERROR("UNKOWN ERROR"); }
        }
    }
}

bool LocalController::acquire_photo(bool stereo, bool gimbal) {
    dji_osdk_ros::CameraAction cameraAction;
    stereo_vant::PointGray stereoAction;

    if (stereo) {
        stereoAction.request.reset_counter = false;
        stereoAction.request.file_path = pathGenerator.getFileName() + "/stereo_voo" + std::to_string(stereo_voo);
        stereoAction.request.file_name = "stereo_vant";
        sv3d_client.call(stereoAction);
        if (stereoAction.response.result) {
            ROS_INFO("Stereo image %i - %hhu", camera_count, stereoAction.response.result);
            camera_count++;
        }
    }
    if (gimbal) {
        cameraAction.request.camera_action = 0;
        camera_action_client.call(cameraAction);
        if (cameraAction.response.result) {
            ROS_INFO("Picture %i - %hhu", camera_count, cameraAction.response.result);
            camera_count++;
        }
    }
    if (cameraAction.response.result || stereoAction.response.result) {
        return true;
    } else { return false; }

}

bool LocalController::generate_WP(int csv_type) {
    /** Initial setting and parameters to generate trajectory*/
    pathGenerator.reset(); // clear pathGen
    waypoint_list.clear(); // clear waypoint list
    wp_n = 0; //! vector initiate at 0 and goes to (h_n*h_v-1)

    int riser_distance, riser_diameter, h_points, v_points, delta_h, delta_v;
    std::string root_directory;
    nh_.param("/riser_inspection/riser_distance", riser_distance, 10);
    nh_.param("/riser_inspection/riser_diameter", riser_diameter, 300);
    nh_.param("/riser_inspection/horizontal_points", h_points, 5);
    nh_.param("/riser_inspection/vertical_points", v_points, 2);
    nh_.param("/riser_inspection/delta_H", delta_h, 15);
    nh_.param("/riser_inspection/delta_V", delta_v, 300);
    nh_.param("/riser_inspection/pos_thresh", pos_error, 0.1);
    nh_.param("/riser_inspection/angle_thresh", yaw_error, 1.0);
    nh_.param("/riser_inspection/root_directory", root_directory, std::string("/home/vant3d/Documents"));

    /** Setting intial parameters to create waypoints */
    pathGenerator.setFolderName(root_directory);
    pathGenerator.setInspectionParam(riser_distance, (float) riser_diameter, h_points, v_points, delta_h,
                                     (float) delta_v);
    /** Define start positions to create waypoints */
    pathGenerator.setInitCoord_XY(current_local_pos.point.x, current_local_pos.point.y, rpa_height,
                                  (int) -RAD2DEG(current_atti_euler.Yaw()));
    try {
        pathGenerator.createInspectionPoints(csv_type); // type 4 refers to XYZ YAW waypoints
        ROS_WARN("Waypoints created at %s/%s", pathGenerator.getFolderName().c_str(),
                 pathGenerator.getFileName().c_str());
        std::vector<std::vector<std::string>> csv_file = pathGenerator.read_csv(
                pathGenerator.getFolderName() + "/" + pathGenerator.getFileName(), ",");
        switch (csv_type) {
            case 1:
                for (int k = 1; k < (int) csv_file.size(); k++) {
                    waypoint_list.push_back(
                            {(float) std::stoi(csv_file[k][0]), std::stof(csv_file[k][1]), std::stof(csv_file[k][2]),
                             std::stof(csv_file[k][3]), std::stof(csv_file[k][4])});
                }
                break;
            case 2:
                for (int k = 1; k < (int) csv_file.size(); k++) {
                    waypoint_list.push_back(
                            {(float) std::stoi(csv_file[k][0]), std::stof(csv_file[k][1]), std::stof(csv_file[k][2]),
                             std::stof(csv_file[k][3]), std::stof(csv_file[k][4])});
                }
                break;
        }
        return true;
    } catch (ros::Exception &e) {
        ROS_WARN("ROS error %s", e.what());
        return false;
    }
}

