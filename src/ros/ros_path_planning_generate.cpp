#include <ros_path_planning_generate.hh>



RiserInspection::RiserInspection() {
    initServices(nh_);
    initSubscribers(nh_);

}

RiserInspection::~RiserInspection() {

}
bool RiserInspection::askControlAuthority() {
    if (activate().result)
    {
        ROS_INFO("Activated successfully");
    }
    else
    {
        ROS_WARN("Failed activation");
        return false;
    }

    // Obtain Control Authority
    ServiceAck ack = obtainCtrlAuthority();
    if (ack.result)
    {
        ROS_INFO("Obtain SDK control Authority successfully");
        return true;
    }
    else
    {
        if (ack.ack_data == 3 && ack.cmd_set == 1 && ack.cmd_id == 0)
        {
            ROS_INFO("Obtain SDK control Authority in progess, "
                     "send the cmd again");
            obtainCtrlAuthority();
        }
        else
        {
            ROS_WARN("Failed Obtain SDK control Authority");
            return false;

        }
    }
}
ServiceAck
RiserInspection::activate() {
    dji_sdk::Activation activation;
    drone_activation_service.call(activation);
    if (!activation.response.result)
    {
        ROS_WARN("ack.info: set = %i id = %i", activation.response.cmd_set,
                 activation.response.cmd_id);
        ROS_WARN("ack.data: %i", activation.response.ack_data);
    }
    return ServiceAck(activation.response.result, activation.response.cmd_set,
                      activation.response.cmd_id, activation.response.ack_data);
}

ServiceAck
RiserInspection::obtainCtrlAuthority()
{
    dji_sdk::SDKControlAuthority sdkAuthority;
    sdkAuthority.request.control_enable = 1;
    sdk_ctrl_authority_service.call(sdkAuthority);
    if (!sdkAuthority.response.result)
    {
        ROS_WARN("ack.info: set = %i id = %i", sdkAuthority.response.cmd_set,
                 sdkAuthority.response.cmd_id);
        ROS_WARN("ack.data: %i", sdkAuthority.response.ack_data);
    }
    return ServiceAck(sdkAuthority.response.result, sdkAuthority.response.cmd_set,
                      sdkAuthority.response.cmd_id,
                      sdkAuthority.response.ack_data);
}

ServiceAck
RiserInspection::takeoff()
{
    dji_sdk::DroneTaskControl droneTaskControl;
    droneTaskControl.request.task = 4;
    drone_task_service.call(droneTaskControl);
    if (!droneTaskControl.response.result)
    {
        ROS_WARN("ack.info: set = %i id = %i", droneTaskControl.response.cmd_set,
                 droneTaskControl.response.cmd_id);
        ROS_WARN("ack.data: %i", droneTaskControl.response.ack_data);
    }
    return ServiceAck(
            droneTaskControl.response.result, droneTaskControl.response.cmd_set,
            droneTaskControl.response.cmd_id, droneTaskControl.response.ack_data);
}
void RiserInspection::initServices(ros::NodeHandle &nh) {
    try {
        generate_pathway_srv_ = nh.advertiseService("riser_inspection/waypoint_generator",
                                                    &RiserInspection::PathGen_serviceCB, this);
        ROS_INFO("Service riser_inspection/waypoint_generator initialize");
        wp_folders_srv = nh.advertiseService("riser_inspection/Folder", &RiserInspection::Folders_serviceCB, this);
        ROS_INFO("Service riser_inspection/Folder initialized");
        ROS_INFO("Waypoint Archieve is saving in: %s/%s", pathGenerator.getFileFolder(), pathGenerator.getFileName());
    } catch (ros::Exception &e) {
        ROS_ERROR("Subscribe topics exception: %s", e.what());
    }
}


void RiserInspection::initSubscribers(ros::NodeHandle &nh) {
    try {
        ros::NodeHandle nh_private("~");

        std::string left_topic_, right_topic_, gps_topic_, rtk_topic_, imu_topic_;
        nh_private.param("gps_topic", gps_topic_, std::string("/dji_ros/gps_position"));
        nh_private.param("rtk_topic", rtk_topic_, std::string("/dji_ros/rtk_position"));

        gps_position_sub_.subscribe(nh, gps_topic_, 1);
        ROS_INFO("Subscriber in Camera GPS Topic: %s", gps_topic_.c_str());
        rtk_position_sub_.subscribe(nh, rtk_topic_, 1);
        ROS_INFO("Subscriber in Camera RTK Topic: %s", rtk_topic_.c_str());

        sync_.reset(new Sync(RiserInspectionPolicy(10), gps_position_sub_, rtk_position_sub_));
        sync_->registerCallback(boost::bind(&RiserInspection::get_gps_position, this, _1, _2));

        ROS_INFO("Subscribe complet");
    } catch (ros::Exception &e) {
        ROS_ERROR("Subscribe topics exception: %s", e.what());
    }
}

void RiserInspection::get_gps_position(const sensor_msgs::NavSatFixConstPtr &msg_gps,
                                    const sensor_msgs::NavSatFixConstPtr &msg_rtk) {
    ptr_gps_position_ = *msg_gps;
    ptr_rtk_position_ = *msg_rtk;
    if (node_start_) {
        lat0_ = ptr_rtk_position_.latitude;
        lon0_ = ptr_rtk_position_.longitude;
        alt0_ = ptr_rtk_position_.altitude;
        //TODO: which topic provides heading value?
        node_start_ = false;
    }
}

bool RiserInspection::PathGen_serviceCB(riser_inspection::wpGenerate::Request &req,
                                     riser_inspection::wpGenerate::Response &res) {

    pathGenerator.setInitCoord(5, 0.3, lat0_, lon0_, alt0_, 74.2);
    pathGenerator.setInspectionParam(5, 4, 15, -0.3);
    try {
        pathGenerator.createInspectionPoints();
        ROS_INFO("Waypoints created");
    } catch (ros::Exception &e) {
        ROS_INFO("ROS error %s", e.what());
        res.result = false;
        pathGenerator.closeFile();
        return res.result;
    }
    res.result = true;
    saved_wp_.close();
    return res.result;
}

bool RiserInspection::Folders_serviceCB(riser_inspection::wpFolders::Request &req,
                                     riser_inspection::wpFolders::Response &res) {

    if (req.file_name.c_str() != NULL) {
        ROS_INFO("Changing waypoint archive name %s", req.file_name.c_str());
        pathGenerator.setFileName(req.file_name);
    }
    if (exists(req.file_path.c_str())) {
        ROS_INFO("Waypoint folder changed %s/%s", req.file_path.c_str(), pathGenerator.getFileName());
        pathGenerator.setFileFolder(req.file_path);
        res.result = true;
        return res.result;
    } else {
        ROS_ERROR("Folder does not exist, file will be written in %s/%s", pathGenerator.getFileFolder(), pathGenerator.getFileName());
        res.result = false;
        return res.result;
    }
}


inline bool RiserInspection::exists(const std::string &name) {
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}

std::vector<std::pair<std::string, std::vector<float>>> RiserInspection::read_csv(std::string filename) {
    // Reads a CSV file into a vector of <string, vector<int>> pairs where
    // each pair represents <column name, column values>

    // Create a vector of <string, int vector> pairs to store the result
    std::vector<std::pair<std::string, std::vector<float>>> result;

    // Create an input filestream
    std::ifstream myFile(filename);

    // Make sure the file is open
    if (!myFile.is_open()) throw std::runtime_error("Could not open file");

    // Helper vars
    std::string line, colname;
    int val;

    // Read the column names
    if (myFile.good()) {
        // Extract the first line in the file
        std::getline(myFile, line);

        // Create a stringstream from line
        std::stringstream ss(line);

        // Extract each column name
        while (std::getline(ss, colname, ',')) {

            // Initialize and add <colname, int vector> pairs to result
            result.push_back({colname, std::vector<float>{}});
        }
    }
}

