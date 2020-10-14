//
// Created by cdj on 2020/10/13.
//
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    cv::VideoCapture cap;
public:
    ImageConverter()
            : it_(nh_), cap("rtsp://admin:HIKVISION7720@192.168.1.63:554/codec/channel/subtype/av_stream") {
        // Subscrive to input video feed and publish output video feed
        // image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/rtsp/image", 1);

        cv::namedWindow(OPENCV_WINDOW);
        publish();
    }

    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void publish() {
        cv::Mat frame;
        //--- INITIALIZE VIDEOCAPTURE
        // open the default camera using default API
        // cap.open(0);
        // OR advance usage: select any API backend
        int deviceID = 0;             // 0 = open default camera
        int apiID = cv::CAP_ANY;      // 0 = autodetect default API
        // open selected camera using selected API
//        cap.open(deviceID, apiID);
        // check if we succeeded
        if (!cap.isOpened()) {
            cerr << "ERROR! Unable to open camera\n";
        }
        //--- GRAB AND WRITE LOOP
        cout << "Start grabbing" << endl
             << "Press any key to terminate" << endl;
        for (;;) {
            // wait for a new frame from camera and store it into 'frame'
            cap.read(frame);
            // check if we succeeded
            if (frame.empty()) {
                cerr << "ERROR! blank frame grabbed\n";
                break;
            }
            // show live and wait for a key with timeout long enough to show images
            cout << frame.size() << endl;
//            imshow(OPENCV_WINDOW, frame);
//            cv::waitKey(1);
            cv_bridge::CvImage out_msg;
            out_msg.header.frame_id = "hikvision";
            out_msg.header.stamp = ros::Time::now(); // Same timestamp and tf frame as input image
            out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
            out_msg.image = frame; // Your cv::Mat
            image_pub_.publish(out_msg.toImageMsg());
        }
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Draw an example circle on the video stream
        if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
            cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));

        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
