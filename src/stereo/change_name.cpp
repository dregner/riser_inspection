#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>


int main()//(int argv, char** argc)
{
    // Extracting path of individual image stored in a given directory
    std::vector<cv::String> images;
    // Path of the folder containing checkerboard images
    std::string path = "/home/vant3d/Documents/stereo-calibration/calib_imgs/3/R*.png";    cv::glob(path, images);

    cv::Mat frame;
    // Looping over all the images in the directory
    for(int i = 0; i<images.size(); i++) {
        // Renaming images to the correct format.

        std::string img_counter_ = "";
//        if (i < 10) {
//            img_counter_ = "00";
//        } else if (i >= 10 && i < 100) {
//            img_counter_ = "0";
//        }
        frame = cv::imread(images[i], cv::IMREAD_COLOR);
//        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
        std::string file_name = "/home/vant3d/Documents/stereo-calibration/calib_imgs/3/right" +img_counter_+ std::to_string(i+1) + ".png";
        cv::imwrite(file_name, frame);

    }
    return 0;
}
