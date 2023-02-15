//
// Created by qin on 09/02/23.
//
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <librealsense2/rs.hpp> // Include Intel RealSense Cross Platform API

int main(int argc, char **argv) {
    ros::init(argc, argv, "pub_cam_node");
    ros::NodeHandle nh;

    // Declare the RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    // Start streaming with the default recommended configuration
    pipe.start(cfg);

    //image_transport will publish the video that can be compressed
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_color = it.advertise("D435/color", 3);

    cv::Mat color_cv, depth_cv;

    while (ros::ok()) {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera (must be called in a loop)

        rs2::frame color_frame = data.get_color_frame();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        color_cv = cv::Mat(cv::Size(640, 480), CV_8UC3, (void *) color_frame.get_data(), cv::Mat::AUTO_STEP);
        // Convert to grey image
        cv::cvtColor(color_cv, color_cv, cv::COLOR_BGR2GRAY);

//        cv::imshow("D435/color", color_cv);
//        cv::waitKey(1);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", color_cv).toImageMsg();
        msg->header.stamp = ros::Time::now();
        pub_color.publish(msg);
        ros::spinOnce();
    }
}