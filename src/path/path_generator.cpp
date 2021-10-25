#include <path_generator.hh>


PathGenerate::PathGenerate() {
    firstTime = true;


}

PathGenerate::~PathGenerate() {

}

void PathGenerate::setInspectionParam(int n_h, int n_v, int deltaDEG, float deltaALT) {
    angleCount_ = n_h;
    altitudeCount_ = n_v;
    deltaAngle_ = deltaDEG;
    deltaAltitude_ = deltaALT;
}

void PathGenerate::setInitCoord(double dist, float d_cyl, double lat, double lon, float alt, float head) {
    d_cyl_ = d_cyl;
    dist_ = dist;
    lon0_ = lon;
    lat0_ = lat;
    alt0_ = alt;
    head0_ = head;
}

void PathGenerate::inspectionAngle2Heading(float polar_angle) {

    float heading = -polar_angle + head0_;
    /// Conditions to keep range - 180 to 180
    if (heading < -180) { heading = heading - 360; }
    if (heading > 180) { heading = heading + 360; }
    waypoint_[3] = heading;
}

void PathGenerate::polar2cart(double r, double alpha) {
    xy_array_[0] = r * cos(DEG2RAD(alpha));
    xy_array_[1] = r * sin(DEG2RAD(alpha));

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

void PathGenerate::csv_save_ugcs(double *wp_array, int wp_number) {
    if (firstTime == true) {
        saved_wp_ << "Latitude,Longitude,AltitudeAMSL,Speed,Picture,WP,CameraTilt,UavYaw,WaitTime" << std::endl;
        firstTime = false;
    }
    if (saved_wp_.is_open()) {
        saved_wp_ << std::setprecision(10) << wp_array[0] << "," << std::setprecision(10) << wp_array[1] << ",";
        saved_wp_ << std::setprecision(10) << wp_array[2] << "," << 1 << ",TRUE," << std::setprecision(2)
                  << wp_number << "," << 0 << ",";
        saved_wp_ << std::setprecision(10) << wp_array[3] << "," << 2 << "\n";
    }
}

void PathGenerate::csv_save_ugcs_EMU(double *wp_array, int row, int wp_number) {
    if (firstTime == true) {
        saved_wp_ << "Latitude,Longitude,AltitudeAMSL,Speed,WP,UavYaw,WaitTime" << std::endl;
        firstTime = false;
    }
    if (saved_wp_.is_open()) {
        saved_wp_ << std::setprecision(11) << wp_array[0] << "," << std::setprecision(11) << wp_array[1] << ", ";
        saved_wp_ << std::setprecision(11) << wp_array[2] << "," << 1 << ',' << wp_number << ",";
        saved_wp_ << std::setprecision(5) << wp_array[3] << "," << 2 << "\n";
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


void PathGenerate::findCenterHeading(int deltaAngle, int angleCount) {
    if (angleCount % 2 == 1) { start_angle_ = (-deltaAngle * angleCount / 2) + deltaAngle / 2; }
    else { start_angle_ = (int) (-angleCount  * round(angleCount / 2)); }
}

void PathGenerate::createInspectionPoints() {
    openFile();
    findCenterHeading(deltaAngle_, angleCount_);
    int count_wp = 1;
    float initial = alt0_; // initiate altitude value
    float vertical; // Set if will move drone down or up INITIALLY DOWN.
    for (int i = 0; i < angleCount_; i++) {
        if (i % 2 == 1) { vertical = 1; } else { vertical = -1; }
        for (int k = 0; k < altitudeCount_; k++) {
            /// Set altitude of this waypoint
            float altitude = initial + (float) k * vertical * deltaAltitude_;
            /// Set Polar values
            polar_array_[0] = dist_ + d_cyl_ / 2; // distance riser and drone
            polar_array_[1] = start_angle_ + i * deltaAngle_; // angle of inspection r^angle (Polar)
            inspectionAngle2Heading((float) polar_array_[1]);
            polar_array_[1] -= +head0_ + 90; // Compense heading orientation and -90 to transform N to 0 deg
            /// Convert Polar to Cartesian
            polar2cart(polar_array_[0], polar_array_[1]);
            /// Introduce values to waypoint array to be printed
            cart2gcs(altitude);
            /// Export to CSV file
//            csv_save_wp(waypoint_, sizeof(waypoint_) / sizeof(waypoint_[0]));
            csv_save_ugcs(waypoint_, count_wp);
            if (k == altitudeCount_ -
                     1) { initial = altitude; } // set initial altitude value when finished de vertical movement.
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
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}

void PathGenerate::setFileName(std::string file_name) {
    file_name_ = file_name;
}

std::string PathGenerate::getFileName() {

    return file_name_;
}

std::string PathGenerate::getFolderName() {
    return file_path_;
}

void PathGenerate::setFolderName(std::string file_name) {
    if (exists(file_name.c_str())) { file_path_ = file_name + "/"; }
    else { std::cout << "Cannot change! Folder does not exist!" << std::endl; }
}

std::vector<std::pair<std::string, std::vector<double>>> PathGenerate::read_csv(std::string filename){
    // Reads a CSV file into a vector of <string, vector<int>> pairs where
    // each pair represents <column name, column values>

    // Create a vector of <string, int vector> pairs to store the result
    std::vector<std::pair<std::string, std::vector<double>>> result;

    // Create an input filestream
    std::ifstream myFile(filename);

    // Make sure the file is open
    if(!myFile.is_open()) throw std::runtime_error("Could not open file");

    // Helper vars
    std::string line, colname;
    int val;

    // Read the column names
    if(myFile.good())
    {
        // Extract the first line in the file
        std::getline(myFile, line);

        // Create a stringstream from line
        std::stringstream ss(line);

        // Extract each column name
        while(std::getline(ss, colname, ',')){

            // Initialize and add <colname, int vector> pairs to result
            result.push_back({colname, std::vector<double> {}});
        }
    }

    // Read data, line by line
    while(std::getline(myFile, line))
    {
        // Create a stringstream of the current line
        std::stringstream ss(line);

        // Keep track of the current column index
        int colIdx = 0;

        // Extract each integer
        while(ss >> val){

            // Add the current integer to the 'colIdx' column's values vector
            result.at(colIdx).second.push_back(val);

            // If the next token is a comma, ignore it and move on
            if(ss.peek() == ',') ss.ignore();

            // Increment the column index
            colIdx++;
        }
    }

    // Close file
    myFile.close();

    return result;
}

