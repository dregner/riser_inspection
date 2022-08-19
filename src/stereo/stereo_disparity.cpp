//
// Created by regner on 24/09/2021.
//
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <stdio.h>
#include <string.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

// initialize values for StereoSGBM parameters
int numDisparities = 16;
int blockSize = 13;
int preFilterType = 1;
int preFilterSize = 7;
int preFilterCap = 57;
int minDisparity = 0;
int textureThreshold = 0;
int uniquenessRatio = 50;
int speckleRange = 0;
int speckleWindowSize = 0;
int disp12MaxDiff = -1;
int dispType = CV_16S;
// Creating an object of StereoSGBM algorithm
cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();
cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(stereo);
cv::Mat left_intrinic = (cv::Mat_<double>(3, 3) << 459.15, 0.22858, 319.31, 0, 459.06, 231.4, 0, 0, 1.0000);
cv::Mat left_dist = (cv::Mat_<double>(5, 1) << 0.041614, -0.031405, 0.0077662, -0.0027449, 0.00023564);
cv::Mat right_intrinic = (cv::Mat_<double>(3, 3) << 460.4, -0.0056765, 329.22, 0, 460.58, 234.58, 0, 0, 1.0000);
cv::Mat right_dist = (cv::Mat_<double>(5, 1) << 0.032129, 0.024637, -0.076301, 0.0001222, -0.0037569);
cv::Mat R = (cv::Mat_<double>(3, 3)
        << 0.999969, -0.004767, 0.006283, -0.004779, 0.999987, 0.001786, -0.006275, -0.001816, 0.999979);
cv::Mat T = (cv::Mat_<double>(3, 1) << -112.8730, -0.2326, 0.0152);



static const std::string OPENCV_WINDOW_S = "Stereo window";
static const std::string OPENCV_WINDOW_D = "Image D window";

class ImageConverter {
    ros::NodeHandle nh_;

    message_filters::Subscriber<sensor_msgs::Image> image_left_sub_;
    message_filters::Subscriber<sensor_msgs::Image> image_right_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>
            StereoPolicy;
    typedef message_filters::Synchronizer<StereoPolicy> Sync;
    boost::shared_ptr<Sync> sync_;

public:
    ImageConverter() {
        initSubscriber(nh_);
        cv::namedWindow(OPENCV_WINDOW_S);
        cv::namedWindow(OPENCV_WINDOW_D);

//        create_trackbars();
        initStereoParam();

    }

    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW_S);
        cv::destroyWindow(OPENCV_WINDOW_D);
    }

    void create_trackbars() {
        cv::resizeWindow(OPENCV_WINDOW_D, 600, 600);

        //         Creating trackbars to dynamically update the StereoBM parameters
        cv::createTrackbar("numDisparities", OPENCV_WINDOW_D, &numDisparities, 18, on_trackbar1);
        cv::createTrackbar("blockSize", OPENCV_WINDOW_D, &blockSize, 50, on_trackbar2);
        cv::createTrackbar("preFilterType", OPENCV_WINDOW_D, &preFilterType, 1, on_trackbar3);
        cv::createTrackbar("preFilterSize", OPENCV_WINDOW_D, &preFilterSize, 25, on_trackbar4);
        cv::createTrackbar("preFilterCap", OPENCV_WINDOW_D, &preFilterCap, 62, on_trackbar5);
        cv::createTrackbar("textureThreshold", OPENCV_WINDOW_D, &textureThreshold, 100, on_trackbar6);
        cv::createTrackbar("uniquenessRatio", OPENCV_WINDOW_D, &uniquenessRatio, 100, on_trackbar7);
        cv::createTrackbar("speckleRange", OPENCV_WINDOW_D, &speckleRange, 100, on_trackbar8);
        cv::createTrackbar("speckleWindowSize", OPENCV_WINDOW_D, &speckleWindowSize, 25, on_trackbar9);
        cv::createTrackbar("disp12MaxDiff", OPENCV_WINDOW_D, &disp12MaxDiff, 25, on_trackbar10);
        cv::createTrackbar("minDisparity", OPENCV_WINDOW_D, &minDisparity, 25, on_trackbar11);
    }

    void initStereoParam() {
        cv::resizeWindow(OPENCV_WINDOW_D, 880, 680);

        stereo->setNumDisparities(32);
        stereo->setBlockSize(13);
        stereo->setPreFilterType(1);
        stereo->setPreFilterSize(13);
        stereo->setPreFilterCap(36);
        stereo->setTextureThreshold(0);
        stereo->setUniquenessRatio(61);
        stereo->setSpeckleRange(0);
        stereo->setSpeckleWindowSize(0);
        stereo->setDisp12MaxDiff(0);
        stereo->setMinDisparity(0);
    }

    void initSubscriber(ros::NodeHandle &nh) {
        ros::NodeHandle nh_private("~");
        image_left_sub_.subscribe(nh, "/dji_osdk_ros/stereo_vga_front_left_images", 1);
        image_right_sub_.subscribe(nh, "/dji_osdk_ros/stereo_vga_front_right_images", 1);

        sync_.reset(new Sync(StereoPolicy(10), image_left_sub_, image_right_sub_));
        sync_->registerCallback(boost::bind(&ImageConverter::imageCb_disp, this, _1, _2));
    }


    void imageCb_disp(const sensor_msgs::ImageConstPtr &imgL, const sensor_msgs::ImageConstPtr &imgR) {
        cv_bridge::CvImagePtr cv_ptr_L, cv_ptr_R;
        try {
            cv_ptr_L = cv_bridge::toCvCopy(imgL, sensor_msgs::image_encodings::MONO8);
            cv_ptr_R = cv_bridge::toCvCopy(imgR, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat imgL_grey = cv_ptr_L->image;
        cv::Mat imgR_grey = cv_ptr_R->image;



        // Calculating disparith using the StereoSGBM algorithm
        cv::Mat stereoBM_disp, stereoBM_16_disp;
        cv::Mat right_disp, filter_disp;
        cv::Mat Q;
        cv::Mat rect_L, rect_R, proj_L, proj_R;
        cv::Mat rectified_mapping1,rectified_mapping2,img_rect_L, img_rect_R;
        cv::stereoRectify(left_intrinic, left_dist, right_intrinic,
                          right_dist, cv::Size(480, 640), R, T, rect_L, rect_R,
                          proj_L, proj_R, Q, CV_CALIB_ZERO_DISPARITY, -1, cv::Size(0, 0));

        // Calculating disparith using the StereoBM algorithm
        cv::initUndistortRectifyMap(left_intrinic, left_dist, rect_L, proj_L,
                                    cv::Size(480, 680), CV_32F,
                                    rectified_mapping1, rectified_mapping2);
        cv::remap(imgL_grey, img_rect_L, rectified_mapping1, rectified_mapping2, cv::INTER_LINEAR);
        cv::initUndistortRectifyMap(right_intrinic, right_dist, rect_R, proj_R,
                                    cv::Size(480, 680), CV_32F,
                                    rectified_mapping1, rectified_mapping2);
        cv::remap(imgR_grey, img_rect_R, rectified_mapping1, rectified_mapping2, cv::INTER_LINEAR);

        stereo->compute(img_rect_L,img_rect_R,stereoBM_disp);
        stereo->compute(img_rect_L,img_rect_R,stereoBM_16_disp);

        stereoBM_disp.convertTo(stereoBM_disp, CV_32F, 0.01);
        stereoBM_16_disp.convertTo(stereoBM_16_disp, CV_16S, 10);


//        cv::Ptr<cv::StereoMatcher> right_stereo = cv::ximgproc::createRightMatcher(stereo);
//        right_matcher->compute(img_rect_L,img_rect_R,stereoBM_16_disp);
//        disparity.convertTo(disparity,CV_8UC1, 1);
        // NOTE: compute returns a 16bit signed single channel image,
        // CV_16S containing a disparity map scaled by 16. Hence it
        // is essential to convert it to CV_16S and scale it down 16 times.

        // Converting disparity values to CV_32F from CV_16S

        // Scaling down the disparity values and normalizing them
        stereoBM_disp = (stereoBM_disp / (float) 16.0 - (float) minDisparity) / ((float) numDisparities);
//        stereoBM_16_disp = (stereoBM_16_disp / (float) 16.0 - (float) minDisparity) / ((float) numDisparities);
        double min_val, max_val;
        cv::minMaxLoc(stereoBM_16_disp, &min_val, &max_val, NULL, NULL);
        stereoBM_16_disp.convertTo(stereoBM_16_disp, CV_8U, 255 / (max_val - min_val), -min_val / (max_val - min_val));




        // Normalizing the disparity map for better visualisation
        cv::normalize(stereoBM_disp, stereoBM_disp, 0, 255, cv::NORM_MINMAX, CV_8U); //CV_8UC1
        cv::normalize(stereoBM_16_disp, stereoBM_16_disp, 0, 255, cv::NORM_MINMAX, CV_8U); //CV_8UC1

//        // Displaying the disparity map
//        cv::imshow(OPENCV_WINDOW_D, stereoBM_16_disp);
        view_stereo_images(img_rect_L, img_rect_R, OPENCV_WINDOW_S);
        view_stereo_images(stereoBM_disp, stereoBM_16_disp, OPENCV_WINDOW_D);

        cv::waitKey(3);
    }

    void view_stereo_images(cv::Mat image1, cv::Mat image2, std::string win_name) {
        cv::Mat image_to_show;
        cv::hconcat(image1, image2, image_to_show);

        cv::resize(image_to_show, image_to_show,
                   cv::Size(image1.cols * 2, image1.rows),
                   (0, 0), (0, 0), cv::INTER_LINEAR);
        cv::imshow(win_name, image_to_show);
    }
    // Defining callback functions for the trackbars to update parameter values


    static void on_trackbar1(int, void *) {
        stereo->setNumDisparities(numDisparities * 16);
        numDisparities = numDisparities * 16;
    }

    static void on_trackbar2(int, void *) {
        stereo->setBlockSize(blockSize * 2 + 5);
        blockSize = blockSize * 2 + 5;
    }

    static void on_trackbar3(int, void *) {
        stereo->setPreFilterType(preFilterType);
    }

    static void on_trackbar4(int, void *) {
        stereo->setPreFilterSize(preFilterSize * 2 + 5);
        preFilterSize = preFilterSize * 2 + 5;
    }

    static void on_trackbar5(int, void *) {
        stereo->setPreFilterCap(preFilterCap);
    }

    static void on_trackbar6(int, void *) {
        stereo->setTextureThreshold(textureThreshold);
    }


    static void on_trackbar7(int, void *) {
        stereo->setUniquenessRatio(uniquenessRatio);
    }

    static void on_trackbar8(int, void *) {
        stereo->setSpeckleRange(speckleRange);
    }

    static void on_trackbar9(int, void *) {
        stereo->setSpeckleWindowSize(speckleWindowSize * 2);
        speckleWindowSize = speckleWindowSize * 2;
    }

    static void on_trackbar10(int, void *) {
        stereo->setDisp12MaxDiff(disp12MaxDiff);
    }

    static void on_trackbar11(int, void *) {
        stereo->setMinDisparity(minDisparity);
    }


};


int main(int argc, char **argv) {
    ros::init(argc, argv, "disparity_param");
    ImageConverter ic;
    ros::spin();
    return 0;
}