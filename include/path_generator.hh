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
    std::ofstream saved_wp_gps_;
    std::string file_path_ = "/home/vant3d/Documents";
    std::string file_name_ = "pathGenerator_WP.csv";
    std::string file_name_gps_ = "pathGenerator_WP_GPS.csv";
    /// Initialize parameters
    int vertical_pts_ = 4; // number of horizontal path
    float delta_altitude_ = 0.3; // 80% image overlap (CAMERA PARAMETER) TODO: Find a way to bring it from image.
    int horizontal_pts_ = 5;    // number of vertical path
    int delta_angle_ = 15;   // variation of angle for each vertical path
    float d_cyl_ = 0.3;     // Riser diameters
    double riser_dist_ = 5;       // Distance between riser and drone

    std::vector<std::vector<float>> polar_points_;
    std::vector<std::vector<float>> h_xy_points_;
    std::vector<std::vector<float>> delta_cartesian_points_;
    std::vector<std::vector<float>> cartesian_points_;
    std::vector<std::vector<float>> gnss_points_;
    /// Initial position to waypoint creates
    // TODO: Must come as initialize parameters
    std::vector<float> gnss_initial{0, 0, 0, 0};
    std::vector<float> xyz_initial{0, 0, 0, 0};

    /// Internal parameters
    bool firstTime;
public:
    PathGenerate();

    ~PathGenerate();

    void reset();

    void setInspectionParam(double dist, float d_cyl, int n_h, int n_v, int deltaDEG, float deltaALT);

    void setInitCoord(double lat, double lon, float alt, int head);

    void setInitCoord_XY(double x, double y, double alt, int head);

    void setCartesianPoints();

    void setGNSSpoints();

    void setFileName(std::string file_name);

    void setFolderName(const std::string &file_name);

    void createInspectionPoints(int csv_type);

    std::string getFileName();

    std::string getFolderName();

    void save_cartesian();

    void save_delta_cartesian();

    void save_gnss();

    void save_ugcs();

    void openFile();

    void closeFile();

    static bool exists(const std::string &name);

    // Function to fetch data from a CSV File
    static std::vector<std::vector<std::string> > read_csv(const std::string &filepath, const std::string &delimiter);
};

#endif // PATH_GEN_H
