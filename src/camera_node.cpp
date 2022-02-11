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
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(20);

    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("cam0/image_raw", 1);

    sensor_msgs::Image msg;
    msg.height = CAM_HEIGHT;
    msg.width = CAM_WIDTH;
    msg.encoding = "mono8";
    msg.is_bigendian = 0;
    msg.step = CAM_WIDTH;

    cv::VideoCapture camera(cv::CAP_V4L2);
    std::cout << camera.set(cv::CAP_PROP_FRAME_WIDTH,  CAM_WIDTH) << ": ";
    std::cout << "CAP_PROP_FRAME_WIDTH:" << camera.get(cv::CAP_PROP_FRAME_WIDTH) << std::endl;

    std::cout << camera.set(cv::CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT) << ": ";
    std::cout << "CAP_PROP_FRAME_HEIGHT:" << camera.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
    
    std::cout << camera.set(cv::CAP_PROP_FPS, CAM_FPS) << ": ";
    std::cout << "CAP_PROP_FPS:" << camera.get(cv::CAP_PROP_FPS) << std::endl;
    
    std::cout << camera.set(cv::CAP_PROP_AUTOFOCUS, 0) << ": ";
    std::cout << "CAP_PROP_AUTOFOCUS:" << camera.get(cv::CAP_PROP_AUTOFOCUS) << std::endl;
    
    std::cout << camera.set(cv::CAP_PROP_AUTO_WB, 0) << ": ";
    std::cout << "CAP_PROP_AUTO_WB:" << camera.get(cv::CAP_PROP_AUTO_WB) << std::endl;
    
    
    
    
    // std::cout << "CAP_PROP_FRAME_WIDTH:" << camera.get(cv::CAP_PROP_FRAME_WIDTH) << std::endl;
    // std::cout << "CAP_PROP_FRAME_WIDTH:" << camera.get(cv::CAP_PROP_FRAME_WIDTH) << std::endl;
    // std::cout << "CAP_PROP_FRAME_WIDTH:" << camera.get(cv::CAP_PROP_FRAME_WIDTH) << std::endl;
    // std::cout << "CAP_PROP_FRAME_WIDTH:" << camera.get(cv::CAP_PROP_FRAME_WIDTH) << std::endl;
    // std::cout << "CAP_PROP_FRAME_WIDTH:" << camera.get(cv::CAP_PROP_FRAME_WIDTH) << std::endl;

    if (!camera.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        exit(1);
    }

    cv::Mat frame;
    cv::Size frame_size(CAM_WIDTH, CAM_HEIGHT);
    cv::Mat frame_gray(frame_size, CV_8UC1);

    std::cout << "Resolution: " << frame_gray.size << std::endl;
    std::cout << "elemSize: " << frame_gray.elemSize() << std::endl;

    cv_bridge::CvImage cv_image;
    cv_image.encoding = msg.encoding;

    cv::namedWindow("test", cv::WINDOW_AUTOSIZE);

    // camera >> frame_gray;
    // cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
    // cv::Point2f image_center(frame_gray.cols/2.0, frame_gray.rows/2.0);
    // cv::Mat rotation = cv::getRotationMatrix2D(image_center, 180.0, 1.0);
    // cv::imshow("test", frame_gray);

    while (nh.ok()){
        camera >> frame_gray;
        cv::imshow("test", frame_gray);

        // std::cout << frame_gray.cols << "x" << frame_gray.rows << std::endl;
        int k = cv::waitKey(30); // Wait for a keystroke in the window

    /*
        cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

        cv::warpAffine(frame_gray, frame_gray, rotation, cv::Size(frame_gray.cols, frame_gray.rows));

        // msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", frame_gray).toImageMsg();
        cv_image.image = frame_gray;
        cv_image.toImageMsg(msg);
        
        msg.header.frame_id = "cam0";
        msg.header.stamp = ros::Time::now();
        
        pub.publish(msg);
*/
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
