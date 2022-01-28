#include "netconfig.hpp"
#include "camera_config.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main()
{
    SocketClient client(TARGET_ADDR, TARGET_PORT);

    cv::VideoCapture camera(0);
    camera.set(cv::CAP_PROP_FRAME_WIDTH,  CAM_WIDTH);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT);
    if (!camera.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return 1;
    }

    cv::Mat frame;
    cv::Size frame_size(CAM_WIDTH, CAM_HEIGHT);
    cv::Mat frame_gray(frame_size, CV_8UC1);

    std::cout << "Resolution: " << frame_gray.size << std::endl;
    std::cout << "elemSize: " << frame_gray.elemSize() << std::endl;


    // create a window to display the images from the webcam
    cv::namedWindow("Client", cv::WINDOW_AUTOSIZE);
    while (1) {
        // capture the next frame from the webcam
        camera >> frame;
        cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
        
        int bytes = frame_gray.total() * frame_gray.elemSize();
        if (send(client.getID(), frame_gray.data, bytes, MSG_CONFIRM) < 0){
            perror("send");
            break;
        }
    }

    close(client.getID());
    return 0;
}