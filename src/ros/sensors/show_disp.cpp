#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <fstream>
#include <image_transport/image_transport.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <stereo_msgs/DisparityImage.h>
/// Variavel para leitura GPS RTK

#include "colormap.h"


cv::Mat_<cv::Vec3b> disparity_color;


void callback(const stereo_msgs::DisparityImageConstPtr &msg) {

    cv_bridge::CvImagePtr cv_ptr;

    namespace enc = sensor_msgs::image_encodings;
    ROS_INFO("MIN: %f, MAX: %f", msg->min_disparity, msg->max_disparity);
    try {
        cv_ptr = cv_bridge::toCvCopy(msg->image, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    float multiplier = 255.0f / (msg->max_disparity - msg->min_disparity);
    const cv::Mat_<float> dmat(msg->image.height, msg->image.width,
                               (float *) &msg->image.data[0], msg->image.step);
    disparity_color.create(msg->image.height, msg->image.width);

    for (int row = 0; row < disparity_color.rows; ++row) {
        const float *d = dmat[row];
        cv::Vec3b *disparity_color_v = disparity_color[row],
                *disparity_color_end = disparity_color_v + disparity_color.cols;
        for (; disparity_color_v < disparity_color_end; ++disparity_color_v, ++d) {
            int index = (*d - msg->min_disparity) * multiplier + 0.5;
            index = std::min(255, std::max(0, index));
            // Fill as BGR
            (*disparity_color_v)[2] = colormap[3 * index + 0];
            (*disparity_color_v)[1] = colormap[3 * index + 1];
            (*disparity_color_v)[0] = colormap[3 * index + 2];
        }
    }

    cv::imshow("Disparity", disparity_color);
    cv::waitKey(10);


}


int main(int argc, char **argv) {

    ros::init(argc, argv, "disparity_show");
    ros::NodeHandle nh;

    ros::Subscriber disp = nh.subscribe("/stereo_depth_perception/disparity_front_left_image", 10, callback);

    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
