/** @file path_generator.h
 *  @author Daniel Regner
 *  @version 2.0
 *  @date Oct, 2021
 *
 *  @brief
 *  An class used to generate and export to CSV waypoints based
 *  on semi-circular trajectory for photogrammetry inspection
 *
 *  @copyright 2021 VANT3D. All rights reserved.
 */

#ifndef PATH_GEN_H
#define PATH_GEN_H

// System includes
#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <iomanip>
#include <sys/stat.h>
#include <boost/algorithm/string.hpp>

#define DEG2RAD(DEG) ((DEG) * ((3.141592653589793) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (3.141592653589793))

class PathGenerate {
private:


    /// file name to store WP
    std::ofstream saved_wp_;
    std::string file_path_ = "/home/vant3d/Documents";
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

    void setInitCoord( double lat, double lon, float alt, float head);

    void setInspectionParam(double dist, float d_cyl, int n_h, int n_v, int deltaDEG, int deltaALT);

    void inspectionAngle2Heading(float polar_angle);

    void polar2cart(double r, double alpha);

    void cart2gcs(double altitude);

    void findCenterHeading(int deltaAngle, int angleCount);

    void createInspectionPoints();

    void csv_save_ugcs(double *wp_array, int wp_number); // Used to export on UgCS

    void csv_save_DJI(double *wp_array, int row); // Used to export on

    void csv_save_ugcs_EMU(double *wp_array, int row, int wp_number);

    void openFile();

    void closeFile();

    bool exists(const std::string &name);

    std::string getFileName();

    std::string getFolderName();

    void setFileName(std::string file_name);

    void setFolderName(std::string file_name);

   // Function to fetch data from a CSV File
    std::vector<std::vector<std::string> > read_csv(const std::string& filepath, const std::string& delimiter);
};

#endif // PATH_GEN_H