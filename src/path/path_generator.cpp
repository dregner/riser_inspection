#include <path_generator.hh>
#include <utility>


PathGenerate::PathGenerate() {
    firstTime = true;


}

PathGenerate::~PathGenerate() = default;

void PathGenerate::reset() {
    firstTime = true;
    delta_cartesian_points_.clear();
    h_xy_points_.clear();
    gnss_points_.clear();
}


void PathGenerate::setInspectionParam(double dist, float d_cyl, int n_h, int n_v, int deltaDEG, float deltaALT) {
    d_cyl_ = d_cyl / 1000;
    riser_dist_ = dist;
    horizontal_pts_ = n_h;
    vertical_pts_ = n_v;
    delta_angle_ = deltaDEG;
    delta_altitude_ = deltaALT / 1000;
}

void PathGenerate::setInitCoord(double lat, double lon, float alt, int head) {
    gnss_initial.clear();
    gnss_initial.push_back((float) lat);
    gnss_initial.push_back((float) lon);
    gnss_initial.push_back((float) alt);
    gnss_initial.push_back((float) head);
}

void PathGenerate::setInitCoord_XY(double x, double y, double alt, int head) {
    xyz_initial.clear();
    xyz_initial.push_back((float) x);
    xyz_initial.push_back((float) y);
    xyz_initial.push_back((float) alt);
    xyz_initial.push_back((float) head);
}

void PathGenerate::setCartesianPoints() {
    float start_angle;
    if (horizontal_pts_ % 2 == 1) { start_angle = (-delta_angle_ * horizontal_pts_ / 2) + delta_angle_ / 2; }
    else { start_angle = (-horizontal_pts_ * round(delta_angle_ / 2)); }

    float xref = (riser_dist_ + d_cyl_ / 2) * cos(DEG2RAD(xyz_initial.at(3)));
    float yref = (riser_dist_ + d_cyl_ / 2) * sin(DEG2RAD(xyz_initial.at(3)));
    float abs_alt = xyz_initial.at(2);
    for (int i = 0; i < horizontal_pts_; i++) {
        float r = (float) riser_dist_ + d_cyl_ / 2;
        float angle = start_angle + (float) (i * delta_angle_) + xyz_initial.at(3);
        if (angle < -180) { angle = angle + 360; }
        if (angle > 180) { angle = angle - 360; }
        float x = r * cos(DEG2RAD(angle)) - xref;
        float y = r * sin(DEG2RAD(angle)) - yref;
        polar_points_.push_back({r, angle});
        h_xy_points_.push_back({x, y});
    }

    for (int i = 0; i < (int) h_xy_points_.size(); i++) {
        float dx, dy;
        if (i == 0) {
            dx = -h_xy_points_[i][0];
            dy = -h_xy_points_[i][1];
        } else {
            dx = h_xy_points_[i - 1][0] - h_xy_points_[i][0];
            dy = h_xy_points_[i - 1][1] - h_xy_points_[i][1];
        }
        for (int j = 0; j < vertical_pts_; j++) {
            int vertical;
            float z;
            i % 2 == 1 ? vertical = 1 : vertical = -1;
            if (j == 0) {
                delta_cartesian_points_.push_back({dx, dy, 0, polar_points_[i][1]});
            } else {
                delta_cartesian_points_.push_back({0, 0, (float) vertical * delta_altitude_, polar_points_[i][1]});
            }
            cartesian_points_.push_back({h_xy_points_[i][0], h_xy_points_[i][1],
                                         abs_alt + (float) j * vertical * delta_altitude_, polar_points_[i][1]});
            abs_alt = abs_alt + (float) j * vertical * delta_altitude_;
        }
    }
}

void PathGenerate::setGNSSpoints() {
    int R = 6371000; // Earth radius
    double x0 = R * DEG2RAD(gnss_initial.at(1)) * cos(DEG2RAD(gnss_initial.at(1))); // Initial Position X
    double y0 = R * DEG2RAD(gnss_initial.at(0)); // Initial Position Y
    for (int i = 0; i < (int) h_xy_points_.size(); i++) {
        double x = h_xy_points_[i][0] + x0;
        double y = h_xy_points_[i][0] + y0;
        for (int j = 0; j < vertical_pts_; j++) {
            int vertical;
            if (i % 2 == 1) { vertical = 1; } else { vertical = -1; }
            float altitude = gnss_initial.at(2) + j * vertical * delta_altitude_;
            gnss_points_.push_back(
                    {(float) RAD2DEG(y / R), (float) RAD2DEG(x / (R * cos(DEG2RAD(gnss_initial.at(1))))), altitude,
                     polar_points_[i][1]});
        }
    }
}

void PathGenerate::save_cartesian() {
    if (firstTime) {
        saved_wp_ << "WP,X,Y,Z,Yaw" << std::endl;
        firstTime = false;
    }
    if (saved_wp_.is_open()) {
        for (int k = 0; k < (int) cartesian_points_.size(); k++) {
            saved_wp_ << k + 1 << ","
                      << std::setprecision(10) << cartesian_points_[k][0] << ","
                      << std::setprecision(10) << cartesian_points_[k][1] << ","
                      << std::setprecision(4) << cartesian_points_[k][2] << ","
                      << std::setprecision(4) << cartesian_points_[k][3] << "\n";
        }
    }
}


void PathGenerate::save_delta_cartesian() {
    if (firstTime) {
        saved_wp_ << "WP,X,Y,Z,Yaw" << std::endl;
        firstTime = false;
    }
    if (saved_wp_.is_open()) {
        for (int k = 0; k < (int) delta_cartesian_points_.size(); k++) {
            saved_wp_ << k + 1 << ","
                      << std::setprecision(10) << delta_cartesian_points_[k][0] << ","
                      << std::setprecision(10) << delta_cartesian_points_[k][1] << ","
                      << std::setprecision(4) << delta_cartesian_points_[k][2] << ","
                      << std::setprecision(4) << delta_cartesian_points_[k][3] << "\n";
        }
    }
}

void PathGenerate::save_gnss() {
    if (firstTime) {
        saved_wp_gps_ << "WP,LAT,LON,ALT,Yaw" << std::endl;
        firstTime = false;
    }
    if (saved_wp_.is_open()) {
        for (int k = 0; k < (int) gnss_points_.size(); k++) {
            saved_wp_gps_ << k + 1 << ","
                      << std::setprecision(10) << gnss_points_[k][0] << ","
                      << std::setprecision(10) << gnss_points_[k][1] << ","
                      << std::setprecision(4) << gnss_points_[k][2] << ","
                      << std::setprecision(4) << gnss_points_[k][3] << "\n";
        }
    }
}

void PathGenerate::save_ugcs() {
    if (firstTime) {
        saved_wp_ << "WP,Latitude,Longitude,AltitudeAGL,UavYaw,Speed,WaitTime,Picture" << std::endl;
        firstTime = false;
    }
    if (saved_wp_.is_open()) {
        for (int k = 0; k < (int) gnss_points_.size(); k++) {
            saved_wp_ << k + 1 << ","
                      << std::setprecision(10) << gnss_points_[k][0] << ","
                      << std::setprecision(10) << gnss_points_[k][1] << ","
                      << std::setprecision(4) << gnss_points_[k][2] << ","
                      << std::setprecision(4) << gnss_points_[k][3] << ","
                      << 0.1 << 2 << "TRUE" << "\n";
        }
    }
}

void PathGenerate::createInspectionPoints(int csv_type) {
    setCartesianPoints();
    setGNSSpoints();
    openFile();
    /// Export to CSV file
    switch (csv_type) {
        case 1:
            PathGenerate::save_cartesian();
            break;
        case 2:
            PathGenerate::save_delta_cartesian();
            PathGenerate::save_gnss();
            break;
        case 3:
            PathGenerate::save_gnss();
            break;
        case 4:
            PathGenerate::save_ugcs();
    }
    closeFile();
}

void PathGenerate::openFile() {
    std::string slash = "/";
    saved_wp_.open(file_path_ + slash + file_name_);
    saved_wp_gps_.open(file_path_+slash+file_name_gps_);
}

void PathGenerate::closeFile() {
    saved_wp_.close();
    saved_wp_gps_.close();
}

bool PathGenerate::exists(const std::string &name) {
    struct stat buffer{};
    return (stat(name.c_str(), &buffer) == 0);
}

void PathGenerate::setFileName(std::string file_name) {
    file_name_ = std::move(file_name);
}

std::string PathGenerate::getFileName() {
    return file_name_;
}

std::string PathGenerate::getFolderName() {
    return file_path_;
}

void PathGenerate::setFolderName(const std::string &file_name) {
    if (exists(file_name)) { file_path_ = file_name; }
    else { std::cout << "Cannot change! Folder does not exist!" << std::endl; }
}

std::vector<std::vector<std::string>>
PathGenerate::read_csv(const std::string &filepath, const std::string &delimeter) {
    std::ifstream file(filepath);
    std::vector<std::vector<std::string>> dataList;
    std::string line;
    // Iterate through each line and split the content using delimeter
    while (getline(file, line)) {
        std::vector<std::string> vec;
        boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
        dataList.push_back(vec);
    }
    // Close the File
    file.close();
    return dataList;
}
