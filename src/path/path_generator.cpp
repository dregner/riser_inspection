#include <path_generator.hh>
#include <utility>


PathGenerate::PathGenerate() {
    firstTime = true;


}

PathGenerate::~PathGenerate() = default;

void PathGenerate::reset() {
    firstTime = true;
}


void PathGenerate::setInspectionParam(double dist, float d_cyl, int n_h, int n_v, int deltaDEG, float deltaALT) {
    d_cyl_ = d_cyl / 1000;
    dist_ = dist;
    angleCount_ = n_h;
    altitudeCount_ = n_v;
    deltaAngle_ = deltaDEG;
    deltaAltitude_ = deltaALT / 1000;
}

void PathGenerate::setInitCoord(double lat, double lon, float alt, int head) {
    lon0_ = lon;
    lat0_ = lat;
    alt0_ = alt;
    head0_ = head;
}

void PathGenerate::setInitCoord_XY(double x, double y, double alt, int head) {
    xyz[0] = x;
    xyz[1] = y;
    xyz[2] = alt;
    head0_ = head;
}

void PathGenerate::inspectionAngle2Heading(int polar_angle) {

    int heading = -polar_angle + head0_;
    /// Conditions to keep range - 180 to 180
    if (heading < -180) { heading = heading + 360; }
    if (heading > 180) { heading = heading - 360; }
    heading_.push_back(polar_angle);
    waypoint_[3] = heading;
}

void PathGenerate::polar2cart(double r, double alpha, double r_ref, double alpha_ref) {
    xy_array_[0] = r * cos(DEG2RAD(alpha)) - (r_ref * cos(DEG2RAD(alpha_ref)));
    xy_array_[1] = r * sin(DEG2RAD(alpha)) - (r_ref * sin(DEG2RAD(alpha_ref)));

}

void PathGenerate::cart2gcs(double altitude) {
    int R = 6371000; // Earth radius
    double x0 = R * DEG2RAD(lon0_) * cos(DEG2RAD(lon0_)); // Initial Position X
    double y0 = R * DEG2RAD(lat0_); // Initial Position Y
    double x = xy_array_[0] + x0;
    double y = xy_array_[1] + y0;
    waypoint_[0] = RAD2DEG(y / R);
    waypoint_[1] = RAD2DEG(x / (R * cos(DEG2RAD(lon0_))));
    waypoint_[2] = altitude;
}

void PathGenerate::csv_save_UGCS(double *wp_array, float altitude) {
    if (firstTime) {
        saved_wp_ << "WP,Latitude,Longitude,AltitudeAMSL,UavYaw,Speed,WaitTime,Picture" << std::endl;
        firstTime = false;
        waypoint_counter = 1;
    }
    if (saved_wp_.is_open()) {
        if (std::abs(alt0_ - altitude) < 0.01 ||
            std::abs((alt0_ + deltaAltitude_ * ((float) altitudeCount_ - 1)) - altitude) < 0.01) {
            saved_wp_ << waypoint_counter << ","
                      << std::setprecision(10) << wp_array[0] << ","
                      << std::setprecision(10) << wp_array[1] << ","
                      << std::setprecision(10) << wp_array[2] << ","
                      << std::setprecision(10) << wp_array[3] << ","
                      << 0.1 << "," << 2 << ",TRUE" << "\n";
            waypoint_counter++;
        }
    }
}

void PathGenerate::csv_save_DJI(double *wp_array, int row) {
    for (int i = 0; i < row; i++) {
        if (saved_wp_.is_open()) {
            if (i != row - 1) { saved_wp_ << std::setprecision(10) << wp_array[i] << ", "; }
            else { saved_wp_ << std::setprecision(10) << wp_array[i] << "\n"; }
        }
    }
}

void PathGenerate::csv_save_XY_yaw(double *wp_array, double yaw, int i) {
    static int angle_count;
    if (firstTime) {
        saved_wp_ << "WP,X,Y,UavYaw" << std::endl;
        firstTime = false;
        waypoint_counter = 1;
        angle_count = i;
    }
    if (saved_wp_.is_open()) {
        if(angle_count == i) {
            saved_wp_ << waypoint_counter << ","
                      << std::setprecision(10) << wp_array[1] + xyz[0] << ","
                      << std::setprecision(10) << wp_array[0] + xyz[1] << ","
                      << std::setprecision(3) << yaw << "\n";
            waypoint_counter++;
            angle_count++;
        }
    }
}

void PathGenerate::csv_save_XYZ_yaw(double *wp_array, double z, double yaw) {
    if (firstTime) {
        saved_wp_ << "WP,X,Y,Z,UavYaw" << std::endl;
        firstTime = false;
        waypoint_counter = 1;
    }
    if (saved_wp_.is_open()) {
        saved_wp_ << waypoint_counter << ","
                  << std::setprecision(10) << wp_array[1] + xyz[0] << ","
                  << std::setprecision(10) << wp_array[0] + xyz[1] << ","
                  << std::setprecision(3) << z << ","
                  << std::setprecision(3) << yaw << "\n";
        waypoint_counter++;
    }
}


void PathGenerate::findCenterHeading(int deltaAngle, int angleCount) {
    if (angleCount % 2 == 1) { start_angle_ = (-deltaAngle * angleCount / 2) + deltaAngle / 2; }
    else { start_angle_ = (int) (-angleCount * round(angleCount / 2)); }
}

void PathGenerate::createInspectionPoints(int csv_type) {
    openFile();
    findCenterHeading(deltaAngle_, angleCount_);
    int count_wp = 1;
    float initial = csv_type == 4 ? xyz[2] : alt0_;

    float vertical; // Set if will move drone down or up INITIALLY DOWN.
    for (int i = 0; i < angleCount_; i++) {
        if (i % 2 == 1) { vertical = 1; } else { vertical = -1; }
        for (int k = 0; k < altitudeCount_; k++) {
            /// Set altitude of this waypoint
            float altitude = initial - (float) k * vertical * deltaAltitude_;
            /// Set Polar values
            polar_array_[0] = dist_ + d_cyl_ / 2; // distance riser and drone
            polar_array_[1] = start_angle_ + i * deltaAngle_; // angle of inspection r^angle (Polar)
            inspectionAngle2Heading((float) polar_array_[1]);
            polar_array_[1] -= +head0_ + 90; // Compense heading orientation and -90 to transform N to 0 deg
            /// Convert Polar to Cartesian
            polar2cart(polar_array_[0], polar_array_[1], dist_ + d_cyl_ / 2, -(head0_ + 90));
            /// Introduce values to waypoint array to be printed
            cart2gcs(altitude);
            /// Export to CSV file
            switch (csv_type) {
                case 1:
                    csv_save_UGCS(waypoint_, altitude);
                    if (count_wp >= angleCount_ * altitudeCount_) {
                        std::cout << "Saved on UgCS struct Simplified (Top and Bottom)" << std::endl; }
                    break;
                case 2:
                    csv_save_DJI(waypoint_, count_wp);
                    if (count_wp >= angleCount_ * altitudeCount_) { std::cout << "Saved on DJI struct" << std::endl; }
                    break;
                case 3:
                    csv_save_XY_yaw(xy_array_, waypoint_[3], i);
                    if (count_wp >= angleCount_ * altitudeCount_) {
                        std::cout << "Saved on XY struct" << std::endl;
                    }
                    break;
                case 4:
                    csv_save_XYZ_yaw(xy_array_, altitude, waypoint_[3]);
                    if (count_wp >= angleCount_ * altitudeCount_) {
                        std::cout << "Saved on XYZ struct" << std::endl;
                    }
                    break;
            }
            if (k == altitudeCount_ - 1) {
                initial = altitude;
            } // set initial altitude value when finished de vertical movement.
            count_wp += 1;
        }
    }
    closeFile();
}

void PathGenerate::openFile() {
    std::string slash = "/";
    saved_wp_.open(file_path_ + slash + file_name_);
}

void PathGenerate::closeFile() {
    saved_wp_.close();
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
