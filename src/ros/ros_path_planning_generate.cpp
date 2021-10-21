#include <ros_path_planning_generate.hh>
#include <experimental/filesystem>
#include <sys/stat.h>
#include <dirent.h>


PathGenerate::PathGenerate() {
    initServices(nh_);
    initSubscribers(nh_);

}

PathGenerate::~PathGenerate() {

}

void PathGenerate::initServices(ros::NodeHandle &nh) {
    try {
        generate_pathway_srv_ = nh.advertiseService("riser_inspection/waypoint_generator",
                                                    &PathGenerate::PathGen_serviceCB, this);
        ROS_INFO("Service riser_inspection/waypoint_generator initialize");
        wp_folders_srv = nh.advertiseService("riser_inspection/Folder", &PathGenerate::Folders_serviceCB, this);
        ROS_INFO("Service riser_inspection/Folder initialized");
    } catch (ros::Exception &e) {
        ROS_ERROR("Subscribe topics exception: %s", e.what());
    }
}


void PathGenerate::initSubscribers(ros::NodeHandle &nh) {
    try {
        ros::NodeHandle nh_private("~");

        std::string left_topic_, right_topic_, gps_topic_, rtk_topic_, imu_topic_;
        nh_private.param("gps_topic", gps_topic_, std::string("/dji_osdk_ros/gps_position"));
        nh_private.param("rtk_topic", rtk_topic_, std::string("/dji_osdk_ros/rtk_position"));

        gps_position_sub_.subscribe(nh, gps_topic_, 1);
        ROS_INFO("Subscriber in Camera GPS Topic: %s", gps_topic_.c_str());
        rtk_position_sub_.subscribe(nh, rtk_topic_, 1);
        ROS_INFO("Subscriber in Camera RTK Topic: %s", rtk_topic_.c_str());

        sync_.reset(new Sync(PathGeneratePolicy(10), gps_position_sub_, rtk_position_sub_));
        sync_->registerCallback(boost::bind(&PathGenerate::get_gps_position, this, _1, _2));

        ROS_INFO("Subscribe complet");
    } catch (ros::Exception &e) {
        ROS_ERROR("Subscribe topics exception: %s", e.what());
    }
}

void PathGenerate::get_gps_position(const sensor_msgs::NavSatFixConstPtr &msg_gps,
                                    const sensor_msgs::NavSatFixConstPtr &msg_rtk) {
    ptr_gps_position_ = *msg_gps;
    ptr_rtk_position_ = *msg_rtk;
    if (firstTime) {
        lat0_ = ptr_rtk_position_.latitude;
        lon0_ = ptr_rtk_position_.longitude;
        alt0_ = ptr_rtk_position_.altitude;
        //TODO: which topic provides heading value?
        firstTime = false;
    }
}

bool PathGenerate::PathGen_serviceCB(riser_inspection::wpGenerate::Request &req,
                                     riser_inspection::wpGenerate::Response &res) {

    riserInspection.setInitCoord(5, 0.3, lat0_, lon0_, alt0_, 74.2);
    riserInspection.setInspectionParam(5, 4, 15, -0.3);
    try {
        riserInspection.createInspectionPoints();
        ROS_INFO("Waypoints created");
    } catch (ros::Exception &e) {
        ROS_INFO("ROS error %s", e.what());
        res.result = false;
        riserInspection.closeFile();
        return res.result;
    }
    res.result = true;
    saved_wp_.close();
    return res.result;
}

bool PathGenerate::Folders_serviceCB(riser_inspection::wpFolders::Request &req,
                                     riser_inspection::wpFolders::Response &res) {

    if (req.file_name.c_str() != NULL) {
        ROS_INFO("Changing waypoint archive name %s", req.file_name.c_str());
        riserInspection.changeFileName(req.file_name);
    }
    if (exists(req.file_path.c_str())) {
        ROS_INFO("Waypoint folder changed %s/%s", req.file_path.c_str(), riserInspection.getFileName());
        riserInspection.changeFileFolder(req.file_path);
        res.result = true;
        return res.result;
    } else {
        ROS_ERROR("Folder does not exist, file will be written in %s/%s", riserInspection.getFileFolder(), riserInspection.getFileName(););
        res.result = false;
        return res.result;
    }
}


inline bool PathGenerate::exists(const std::string &name) {
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}

std::vector<std::pair<std::string, std::vector<float>>> PathGenerate::read_csv(std::string filename) {
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

