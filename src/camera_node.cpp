// #include "netconfig.hpp"
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
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);
    sensor_msgs::ImagePtr msg;
    
    cv::VideoCapture camera(0);
    camera.set(cv::CAP_PROP_FRAME_WIDTH,  CAM_WIDTH);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT);

    if (!camera.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        exit(1);
    }

    cv::Mat frame;
    cv::Size frame_size(CAM_WIDTH, CAM_HEIGHT);
    cv::Mat frame_gray(frame_size, CV_8UC1);

    std::cout << "Resolution: " << frame_gray.size << std::endl;
    std::cout << "elemSize: " << frame_gray.elemSize() << std::endl;

    cv::namedWindow("Client", cv::WINDOW_AUTOSIZE);
    while (nh.ok()){
        camera >> frame;
        cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

        // cv::imshow("Server", frame_gray);
        // if (cv::waitKey(10) >= 0)
        //     break;
        msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", frame_gray).toImageMsg();
        pub.publish(msg);
        ros::spinOnce();
        // loop_rate.sleep();
    }
    return 0;
}