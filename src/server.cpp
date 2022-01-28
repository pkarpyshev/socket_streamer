#include "netconfig.hpp"
#include "camera_config.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

const int buf_size = CAM_HEIGHT*CAM_WIDTH;

int main(int argc, char *argv[]){
    
    SocketServer server(TARGET_ADDR, TARGET_PORT);
    
    listen(server.getID(), 1);
        
    cv::Size frame_size(CAM_WIDTH, CAM_HEIGHT);
    cv::Mat frame_gray(frame_size, CV_8UC1);
    cv::namedWindow("Server", cv::WINDOW_AUTOSIZE);
    
    ros::init(argc, argv, "video_streamer");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", frame_gray).toImageMsg();
    ros::Rate loop_rate(5);
    // while(1) {
        int client_sock = accept(server.getID(), NULL, NULL);
        if(client_sock < 0) {
            perror("accept");
            exit(3);
        }

        for (;;){
            auto addr_from = server.getSockaddr();
            int bytes_read = recv(client_sock, frame_gray.data, buf_size, MSG_WAITALL);

            cv::imshow("Server", frame_gray);
            if (cv::waitKey(10) >= 0)
                break;

            pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
        }
        
        close(client_sock);
    // }

    return 0;
}