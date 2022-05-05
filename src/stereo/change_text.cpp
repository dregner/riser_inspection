#include <ostream>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <boost/algorithm/string.hpp>


int main()//(int argv, char** argc)
{
    // Extracting path of individual image stored in a given directory
    // Path of the folder containing checkerboard images
    std::string img_counter_;
    for (int i = 1; i < 922; i++) {
        if (i < 10) {
            img_counter_ = "0";
        } else if (i >= 10) {
            img_counter_ = "";
        }
        std::string path = "/home/vant3d/Documents/labels/labmetro_image_" + img_counter_ + std::to_string(i) + ".txt";

        std::ifstream input(path);
        std::ofstream output;
        std::vector<std::vector<std::string>> dataList;
        std::string line;
        std::vector<std::string> vec;
        // Iterate through each line and split the content using delimeter
        while (getline(input, line)) {
            boost::algorithm::split(vec, line, boost::is_any_of(" "));
            vec[0] = std::to_string(1);
        }
        output.open(path);
            if (output.is_open()) {
                for (int k = 0; k<5; k++) {
                    if(k==4){
                    output << vec[k];
                }else{
                    output << vec[k] << " ";
                    }
                }
            }
//        0 0.124512 0.491167 0.248535 0.978333
        output.close();
    }

    return 0;

}
