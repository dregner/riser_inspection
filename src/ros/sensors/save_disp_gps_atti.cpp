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
#include "colormap.h"
/// Variavel para leitura GPS RTK


double RadToDeg = 180 / M_PI;


static int counter = 0;


static std::ofstream images_file;
cv::Mat_<cv::Vec3b> disparity_color;
cv::Mat disp_img;
void callback(const stereo_msgs::DisparityImageConstPtr &msg,
              const sensor_msgs::NavSatFixConstPtr &pose_GPS,
              const geometry_msgs::QuaternionStampedConstPtr &atti) {

    cv_bridge::CvImagePtr cv_ptr;

    namespace enc = sensor_msgs::image_encodings;
//    ROS_INFO("MIN: %f, MAX: %f", msg->min_disparity, msg->max_disparity);
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
    std::stringstream write;

   write << "zed_D" << counter << ".png";
    cv::imwrite(write.str(), disparity_color);
    if (images_file.is_open()) {
        images_file << write.str()
                    << "\t" << std::setprecision(10) << pose_GPS->longitude << "\t" << std::setprecision(10)
                    << pose_GPS->latitude
                    << "\t" << std::setprecision(10) << pose_GPS->altitude
                    << "\t" << atti->quaternion.w << "\t" << atti->quaternion.x << "\t" << atti->quaternion.y << "\t"
                    << atti->quaternion.z
                    << "\n";
    }
    ++counter;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "disp_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<stereo_msgs::DisparityImage> disp_sub(nh, "/zed2/zed_node/disparity/disparity_image",
                                                                      10);
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_pose(nh, "/dji_osdk_ros/gps_position", 100);
    message_filters::Subscriber<geometry_msgs::QuaternionStamped> atti_sub(nh, "/dji_osdk_ros/attitude", 100);


    images_file.open("disparity_zed.txt");


    typedef message_filters::sync_policies::ApproximateTime<stereo_msgs::DisparityImage, sensor_msgs::NavSatFix, geometry_msgs::QuaternionStamped> MySyncPolicy;
    // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1000), disp_sub, gps_pose, atti_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));

    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
