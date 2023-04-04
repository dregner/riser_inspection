//
// Created by vant3d on 04/04/23.
//

#include <dji_osdk_ros/StereoVGASubscription.h>
#include "ros/ros.h"
#include <signal.h>


ros::ServiceClient stereo_vga_subscription_client;
dji_osdk_ros::StereoVGASubscription subscription;

bool vga_imgs_subscribed = false;


bool imgSubscriptionHelper(dji_osdk_ros::StereoVGASubscription &service) {

    std::string action;
    if (service.request.unsubscribe_vga) {
        action = "unsubscribed";
    } else {
        action = "subscribed";
    }

    stereo_vga_subscription_client.call(service);
    if (service.response.result == true) {
        ROS_INFO("Successfully %s to VGA images", action.c_str());
        if (service.request.unsubscribe_vga) {
            vga_imgs_subscribed = false;
        } else {
            vga_imgs_subscribed = true;
        }
    } else {
        ROS_ERROR("Failed to %s to VGA images", action.c_str());
        return false;
    }

    return true;
}

void shutDownHandler(int s) {
    ROS_INFO("Caught signal %d", s);

    if (vga_imgs_subscribed) {
        subscription.request.unsubscribe_vga = 1;
        imgSubscriptionHelper(subscription);
    }

    exit(1);
}

int
main(int argc, char **argv) {
    ros::init(argc, argv, "rosservice_stereo_vga");
    ros::NodeHandle nh;

    stereo_vga_subscription_client = nh.serviceClient<dji_osdk_ros::StereoVGASubscription>("stereo_vga_subscription");

    subscription.request.vga_freq = subscription.request.VGA_20_HZ;
    subscription.request.front_vga = 1;
    subscription.request.unsubscribe_vga = 0;

    if (!imgSubscriptionHelper(subscription)) {
        return -1;
    }
    ros::Duration(1).sleep();

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = shutDownHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

}