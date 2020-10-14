//
// Created by cdj on 2020/10/13.
//
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "network_camera_publisher");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    string camera;
    nh.getParam("camera", camera);
    image_transport::Publisher image_pub = it.advertise(camera + "image", 1);

    string url;  // rtsp://admin:HIKVISION7720@192.168.1.63:554/codec/channel/subtype/av_stream
    nh.getParam("url", url);
    cv::VideoCapture cap(url);
    cv::Mat frame;

    ros::Rate r(30);
    while (ros::ok()) {
        cap.read(frame);
        cv_bridge::CvImage out_msg;
        out_msg.header.frame_id = "hikvision";
        out_msg.header.stamp = ros::Time::now(); // Same timestamp and tf frame as input image
        out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
        out_msg.image = frame; // Your cv::Mat
        image_pub.publish(out_msg.toImageMsg());

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
