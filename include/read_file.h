/** @file read_file.h
 *  @author Daniel Regner
 *  @version 1.0
 *  @date Oct, 2021
 *
 *  @brief
 *  Class created to read CSV files using comma as divisor.
 *
 *  @copyright 2021 VANT3D. All rights reserved.
 */


#ifndef RISER_INSPECTION_READ_FILE_H
#define RISER_INSPECTION_READ_FILE_H
#include <boost/algorithm/string.hpp>
#include <string>
#include <vector>
#include <istream>
#include <fstream>

/*
 * A class to read data from a csv file.
 */
class CSVReader
{
    std::string fileName;
    std::string delimeter;
public:
    CSVReader(std::string filename, std::string delm = ",") :
            fileName(filename), delimeter(delm)
    { }
    // Function to fetch data from a CSV File
    std::vector<std::vector<std::string> > getData();
};
#endif //RISER_INSPECTION_READ_FILE_H
