#include "camera_config.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

const int buf_size = CAM_HEIGHT*CAM_WIDTH;

int main(int argc, char *argv[]){
    // Open camera stream
    cv::VideoCapture camera(cv::CAP_V4L2);
    if (!camera.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        exit(1);
    }

    // Setup camera prarameters
    std::cout << camera.set(cv::CAP_PROP_FRAME_WIDTH,  CAM_WIDTH) << ": ";
    std::cout << "CAP_PROP_FRAME_WIDTH:" << camera.get(cv::CAP_PROP_FRAME_WIDTH) << std::endl;

    std::cout << camera.set(cv::CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT) << ": ";
    std::cout << "CAP_PROP_FRAME_HEIGHT:" << camera.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
    
    std::cout << camera.set(cv::CAP_PROP_FPS, CAM_FPS) << ": ";
    std::cout << "CAP_PROP_FPS:" << camera.get(cv::CAP_PROP_FPS) << std::endl;

    std::cout << camera.set(cv::CAP_PROP_ZOOM, 0) << ": ";
    std::cout << "CAP_PROP_ZOOM:" << camera.get(cv::CAP_PROP_ZOOM) << std::endl;

    // Define  variables for opencv
    cv::Mat frame_rgb;
    cv::Size frame_size(CAM_WIDTH, CAM_HEIGHT);
    cv::Mat frame_gray(frame_size, CV_8UC1);
    
    // Need to rotate image
    cv::Point2f image_center(frame_gray.cols/2.0, frame_gray.rows/2.0);
    cv::Mat rotation = cv::getRotationMatrix2D(image_center, 180.0, 1.0);

    // Initializw ROS
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("cam0/image_raw", 1);
    ros::Rate loop_rate(30);
    sensor_msgs::Image msg;
    msg.height = CAM_HEIGHT;
    msg.width = CAM_WIDTH;
    msg.encoding = "mono8";
    msg.is_bigendian = 0;
    msg.step = CAM_WIDTH;

    cv_bridge::CvImage cv_image;
    cv_image.encoding = msg.encoding;

    while (nh.ok()){
        // Read image
        camera >> frame_rgb;
        // Convert to gray scale
        cv::cvtColor(frame_rgb, frame_gray, cv::COLOR_RGB2GRAY);
        // Rotate image
        cv::warpAffine(frame_gray, frame_gray, rotation, cv::Size(frame_gray.cols, frame_gray.rows));
        // Make message
        cv_image.image = frame_gray;
        cv_image.toImageMsg(msg);
        msg.header.frame_id = "cam0";
        msg.header.stamp = ros::Time::now();
        
        pub.publish(msg);
        // ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
