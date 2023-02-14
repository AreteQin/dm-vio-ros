//
// Created by qin on 13/02/23.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <tf/transform_listener.h>
#include <sophus/se3.hpp>

// sync sensor data
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

// sync image
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/Imu.h>

void SyncCallback(const sensor_msgs::ImuConstPtr &imu, const sensor_msgs::ImageConstPtr &color) {
    // print imu data
    std::cout << "imu data: " << imu->angular_velocity.x << " " << imu->angular_velocity.y << " "
              << imu->angular_velocity.z << imu->linear_acceleration.x << " " << imu->linear_acceleration.y << " "
              << imu->linear_acceleration.z << std::endl;
    try {
        cv::imshow("D435/color", cv_bridge::toCvShare(color, "bgr8")->image);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", color->encoding.c_str());
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sync_msgs");
    ros::NodeHandle nh_sync_msg;
    message_filters::Subscriber<sensor_msgs::Imu> sub_imu;
    sub_imu.subscribe(nh_sync_msg, "qcar_imu/raw", 50);

    cv::namedWindow("D435/color");
    image_transport::ImageTransport it(nh_sync_msg);
    image_transport::SubscriberFilter sub_color(it, "D435/color", 3);

    //将话题的数据进行同步
    message_filters::TimeSynchronizer<sensor_msgs::Imu, sensor_msgs::Image>
            sync(sub_imu, sub_color, 10);
    //指定一个回调函数, 实现两个数据的同步读取
    sync.registerCallback(boost::bind(&SyncCallback, _1, _2));

    ros::Rate rate(30.0);
    while (nh_sync_msg.ok()) {
        ros::spinOnce();
        rate.sleep();
    }
}