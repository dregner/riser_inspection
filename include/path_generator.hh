//
// Created by regner on 06/09/2021.
//

#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <iomanip>

#define DEG2RAD(DEG) ((DEG) * ((3.141592653589793) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (3.141592653589793))

class PathGenerate {
private:


    /// file name to store WP
    std::ofstream saved_wp_;
    std::string file_path_ = "~/Documents";
    std::string file_name_ = "pathGenerator_WP.csv";
    /// Initialize parameters
    int altitudeCount_ = 4; // number of horizontal path
    float deltaAltitude_ = 0.3; // 80% image overlap (CAMERA PARAMETER) TODO: Find a way to bring it from image.
    int angleCount_ = 5;    // number of vertical path
    int deltaAngle_ = 15;   // variation of angle for each vertical path
    float d_cyl_ = 0.3;     // Riser diameters
    double dist_ = 5;       // Distance between riser and drone
    int start_angle_ = (-deltaAngle_ * angleCount_ / 2) + deltaAngle_ / 2;

    /// Arrays to store WP creation
    double polar_array_[2]{0, 0}; // Polar array [r deg]
    double xy_array_[2]{0, 0};    // Cart array [x y]
    double waypoint_[4]{0, 0, 0, 0}; // Waypoint array [lat lon alt heading]

    /// Initial position to waypoint creates
    // TODO: Must come as initialize parameters
    double lat0_ = -27.605299;  // Starting latitude
    double lon0_ = -48.520547;  // Starting longitude
    float alt0_ = 3;              // Starting altitude
    float head0_ = 30;            // Starting heading

    /// Internal parameters
    int Y_North_ZeroDegrees_ = -90;
    bool firstTime;
public:
    PathGenerate();

    ~PathGenerate();

    void setInitCoord(double dist, float d_cyl, double lat, double lon, float alt, float head);

    void setInspectionParam(int n_h, int n_v, int deltaDEG, float deltaALT);

    void inspectionAngletoHeading(float polar_angle);

    void polar2cart(double r, double alpha);

    void cart2gcs(double altitude);

    void findCenterHeading(int deltaAngle, int angleCount);

    void createInspectionPoints();

    void csv_save_ugcs(double *wp_array, int row, int wp_number); // Used to export on UgCS

    void csv_save_DJI(double *wp_array, int row); // Used to export on

    void csv_save_ugcs_EMU(double *wp_array, int row, int wp_number);

    void openFile();

    void closeFile();

    void setFileFolder(std::string file_name);

    char *getFileName();

    void setFileName(std::string file_name);

    char *getFileFolder();


};