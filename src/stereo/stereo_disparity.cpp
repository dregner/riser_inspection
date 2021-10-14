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
#include <stdio.h>
#include <string.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>


static const std::string OPENCV_WINDOW_L = "Image L window";
static const std::string OPENCV_WINDOW_R = "Image R window";
//static const std::string OPENCV_WINDOW_D = "Image D window";

class ImageConverter
{
    ros::NodeHandle nh_;

    message_filters::Subscriber<sensor_msgs::Image> image_left_sub_;
    message_filters::Subscriber<sensor_msgs::Image> image_right_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>
            StereoPolicy;
    typedef message_filters::Synchronizer<StereoPolicy> Sync;
    boost::shared_ptr<Sync> sync_;

public:
    ImageConverter()
    {
        initSubscriber(nh_);
        cv::namedWindow(OPENCV_WINDOW_L);
        cv::namedWindow(OPENCV_WINDOW_R);
//        cv::namedWindow(OPENCV_WINDOW_D);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW_L);
        cv::destroyWindow(OPENCV_WINDOW_R);
//        cv::destroyWindow(OPENCV_WINDOW_D);
    }
    void initSubscriber(ros::NodeHandle &nh) {
        ros::NodeHandle nh_private("~");
        image_left_sub_.subscribe(nh, "/dji_sdk/stereo_vga_front_left_images", 1);
        image_right_sub_.subscribe(nh, "/dji_sdk/stereo_vga_front_right_images", 1);

        sync_.reset(new Sync(StereoPolicy(10), image_left_sub_, image_right_sub_));
        sync_->registerCallback(boost::bind(&ImageConverter::imageCb_disp, this, _1, _2));
    }

    void imageCb_show(const sensor_msgs::ImageConstPtr &imgL, const sensor_msgs::ImageConstPtr &imgR)
    {
        cv_bridge::CvImagePtr cv_ptr_L, cv_ptr_R;
        try
        {
            cv_ptr_L = cv_bridge::toCvCopy(imgL, sensor_msgs::image_encodings::MONO8);
            cv_ptr_R = cv_bridge::toCvCopy(imgR, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Update GUI Window
        cv::imshow(OPENCV_WINDOW_L, cv_ptr_L->image);
        cv::imshow(OPENCV_WINDOW_R, cv_ptr_R->image);
        cv::waitKey(3);
    }
    void imageCb_disp(const sensor_msgs::ImageConstPtr &imgL, const sensor_msgs::ImageConstPtr &imgR)
    {
        cv_bridge::CvImagePtr cv_ptr_L, cv_ptr_R;
        try
        {
            cv_ptr_L = cv_bridge::toCvCopy(imgL, sensor_msgs::image_encodings::MONO8);
            cv_ptr_R = cv_bridge::toCvCopy(imgR, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat imgL_grey = cv_ptr_L->image;
        cv::Mat imgR_grey = cv_ptr_R->image;
//        cv::cvtColor(cv_ptr_L->image, imgL_grey, cv::GRAY);
//        cv::cvtColor(cv_ptr_R->image, imgR_grey, cv::COLOR_BGR2GRAY);

        // Setting parameters for StereoSGBM algorithm
        int minDisparity = 0.1;
        int numDisparities = 64;
        int blockSize = 8;
        int disp12MaxDiff = 1;
        int uniquenessRatio = 10;
        int speckleWindowSize = 10;
        int speckleRange = 8;

        // Creating an object of StereoSGBM algorithm
        cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(minDisparity, numDisparities, blockSize,
                                                                disp12MaxDiff, uniquenessRatio, speckleWindowSize,
                                                                speckleRange);

        // Calculating disparith using the StereoSGBM algorithm
        cv::Mat disp, disparity;
//        stereo->compute(cv_ptr_L->image, cv_ptr_R->image, disp);
        // Calculating disparith using the StereoBM algorithm
        stereo->compute(imgL_grey,imgR_grey,disp);

        // NOTE: compute returns a 16bit signed single channel image,
        // CV_16S containing a disparity map scaled by 16. Hence it
        // is essential to convert it to CV_16S and scale it down 16 times.

        // Converting disparity values to CV_32F from CV_16S
        disp.convertTo(disparity,CV_32F, 1.0);

        // Scaling down the disparity values and normalizing them
        disparity = (disparity/(float)16.0 - (float)minDisparity)/((float)numDisparities);

        // Displaying the disparity map
//        cv::imshow("disparity",disparity);

        // Normalizing the disparity map for better visualisation
        cv::normalize(disp, disp, 0, 255, cv::NORM_MINMAX, CV_8UC1); //CV_8UC1
//        // Displaying the disparity map
        cv::imshow("OPENCV_WINDOW_D", disp);

        // Update GUI Window
        cv::imshow(OPENCV_WINDOW_L, imgL_grey);
        cv::imshow(OPENCV_WINDOW_R, imgR_grey);
        cv::waitKey(3);
    }

};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}