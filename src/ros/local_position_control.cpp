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
    gps_sub = nh.subscribe<sensor_msgs::NavSatFix>(gps_topic, 1, &LocalController::gps_callback, this);
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
    camera_photo_client = nh.serviceClient<dji_osdk_ros::CameraStartShootSinglePhoto>(
            "camera_start_shoot_single_photo");
    camera_record_video_client = nh.serviceClient<dji_osdk_ros::CameraRecordVideoAction>("camera_record_video_action");
    sv3d_client = nh.serviceClient<stereo_vant::PointGray>("/stereo_point_grey/take_picture");

    local_position_service = nh.advertiseService("/riser_inspection/set_position",
                                                 &LocalController::local_pos_service_cb, this);

    start_mission_service = nh.advertiseService("/riser_inspection/start_mission",
                                                &LocalController::start_mission_service_cb, this);

    ROS_INFO("Subscribed Complet");
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
    if (req.use_gimbal) {
        use_gimbal = true;
        camera_gimbal = !req.video;
        video_gimbal = req.video;
    }
    use_stereo = req.use_stereo;
    camera_count = 1;
    init_heading = -RAD2DEG(current_atti_euler.Yaw());
    if (LocalController::set_local_position()) {
        if (use_stereo) {
            stereo_vant::PointGray stereoAction;
            stereo_count++;
            stereoAction.request.reset_counter = true;
            stereoAction.request.file_path = pathGenerator.getFileName() + "/stereo_voo" + std::to_string(stereo_count);
            stereoAction.request.file_name = "sv3d";
            sv3d_client.call(stereoAction);
        }
        if (use_gimbal) {
            ROS_INFO("Set Gimbal to follow aircraft heading");
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
    //! current_atti_euler after rotations returns [roll=pitch, pitch=roll, yaw = -yaw]
    //TODO: rotation matrix's to return properly yaw
    current_atti_euler.Set(q_atti_matrix.getW(), q_atti_matrix.getX(), q_atti_matrix.getY(), q_atti_matrix.getZ());

}

void LocalController::local_position_callback(const geometry_msgs::PointStamped::ConstPtr &msg) {
    current_local_pos = *msg;
}

void LocalController::gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_gps = *msg;
    if (doing_mission) {
        if (wp_n < (int) waypoint_list.size()) {
            if (local_position_ctrl_mission()) {
                if (use_gimbal) {
                    LocalController::set_gimbal_angles(0, 0, 0);
                    if (camera_gimbal) { LocalController::gimbal_camera(false); }
                    if( video_gimbal && wp_n == 0){ LocalController::gimbal_camera(true); }
                }
                if (use_stereo) {
                    ROS_INFO("SV3D IMG");
                    LocalController::stereo_photo();
                }
                wp_n++;
            }
        } else {
            ROS_WARN("BACK TO INITIAL POSITION");
            if( video_gimbal){ LocalController::gimbal_camera(false); }
            if (local_position_ctrl((float) -current_local_pos.point.y, (float) -current_local_pos.point.x,
                                    (float) -current_local_pos.point.z,
                                    (float) init_heading, (float) pos_error, (float) yaw_error)) {
                ROS_INFO("MISSION FINISHED");
                doing_mission = false;
            } else { ROS_ERROR("UNKOWN ERROR"); }
        }
    }
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
        ROS_INFO("Lat: %f, Lon: %f, Height: %f m @ %f deg target complete",
                 current_gps.latitude, current_gps.longitude, rpa_height,
                 -RAD2DEG(current_atti_euler.Yaw()));
        LocalController::obtain_control(false);
        return control_task_point.response.result;
    } else { return control_task_point.response.result; }
}

bool
LocalController::local_position_ctrl_mission() {
    dji_osdk_ros::FlightTaskControl control_task_mission;
    control_task_mission.request.task = dji_osdk_ros::FlightTaskControl::Request::TASK_POSITION_AND_YAW_CONTROL;
    control_task_mission.request.joystickCommand.x = waypoint_list[wp_n][1];
    control_task_mission.request.joystickCommand.y = waypoint_list[wp_n][2];
    control_task_mission.request.joystickCommand.z = waypoint_list[wp_n][3];
    control_task_mission.request.joystickCommand.yaw = waypoint_list[wp_n][4];
    control_task_mission.request.posThresholdInM = (float) pos_error;
    control_task_mission.request.yawThresholdInDeg = (float) yaw_error;

    task_control_client.call(control_task_mission);

    if (control_task_mission.response.result) {
        ROS_INFO("WP %i @ %f deg target complete", wp_n + 1, rpa_height,
                 -RAD2DEG(current_atti_euler.Yaw()));
        ROS_INFO("Lat: %f, Lon: %f, Height: %f m @ %f deg target complete",
                 current_gps.latitude, current_gps.longitude, rpa_height,
                 -RAD2DEG(current_atti_euler.Yaw()));
    }
    return control_task_mission.response.result;
}

bool LocalController::gimbal_camera(bool record_video) {


    if (camera_gimbal) {
        dji_osdk_ros::CameraStartShootSinglePhoto cameraAction;
        cameraAction.request.payload_index = 0;
        camera_photo_client.call(cameraAction);
        if (cameraAction.response.result) {
            ROS_INFO("Picture %i", camera_count);
            camera_count++;
            return cameraAction.response.result;
        } else { return cameraAction.response.result; }
    }

    if (video_gimbal) {
        dji_osdk_ros::CameraRecordVideoAction cameraRecordVideoAction;
        cameraRecordVideoAction.request.start_stop = record_video;
        cameraRecordVideoAction.request.payload_index = static_cast<uint8_t>(dji_osdk_ros::PayloadIndex::PAYLOAD_INDEX_0);
        camera_record_video_client.call(cameraRecordVideoAction);
        if (cameraRecordVideoAction.response.result) {
            ROS_INFO("Video %s", record_video ? "Record" : "Stop");
            return cameraRecordVideoAction.response.result;
        } else { return cameraRecordVideoAction.response.result; }
    }

    if (!video_gimbal && !camera_gimbal) {
        return false;
    }

}

bool LocalController::stereo_photo() {
    stereo_vant::PointGray stereoAction;
    ROS_INFO("Got on function");

    if (use_stereo) {
        stereoAction.request.reset_counter = false;
        stereoAction.request.file_path = pathGenerator.getFileName() + "/stereo_voo" + std::to_string(stereo_count);
        stereoAction.request.file_name = "stereo_vant";
        sv3d_client.call(stereoAction);
        if (stereoAction.response.result) {
            ROS_INFO("Stereo image %i", camera_count);
            camera_count++;
            return stereoAction.response.result;
        } else { return stereoAction.response.result; }
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
                                  (int) init_heading);
    try {
        pathGenerator.createInspectionPoints(csv_type); // type 4 refers to XYZ YAW waypoints
        ROS_WARN("Waypoints created at %s/%s", pathGenerator.getFolderName().c_str(),
                 pathGenerator.getFileName().c_str());
        std::vector<std::vector<std::string>> csv_file = PathGenerate::read_csv(
                pathGenerator.getFolderName() + "/" + pathGenerator.getFileName(), ",");

        for (int k = 1; k < (int) csv_file.size(); k++) {
            waypoint_list.push_back(
                    {(float) std::stoi(csv_file[k][0]), std::stof(csv_file[k][1]), std::stof(csv_file[k][2]),
                     std::stof(csv_file[k][3]), std::stof(csv_file[k][4])});
        }
        return true;
    } catch (ros::Exception &e) {
        ROS_WARN("ROS error %s", e.what());
        return false;
    }
}


