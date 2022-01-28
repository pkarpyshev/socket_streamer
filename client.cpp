#include "netconfig.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>

char message[] = "Hello there!\n";
char buf[sizeof(message)];

int main()
{
    // SocketClient client(TARGET_ADDR, TARGET_PORT);

    cv::VideoCapture camera(0);

    if (!camera.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
        return 1;
    }

    // create a window to display the images from the webcam
    cv::namedWindow("Webcam", CV_WINDOW_AUTOSIZE);

    // this will contain the image from the webcam
    cv::Mat frame;


    // display the frame until you press a key
    while (1) {
        // capture the next frame from the webcam
        camera >> frame;
        // show the image on the window
        cv::imshow("Webcam", frame);
        // wait (10ms) for a key to be pressed
        if (cv::waitKey(10) >= 0)
            break;
    }

    // send(sock, message, sizeof(message), 0);
    // recv(sock, buf, sizeof(message), 0);
    
    // std::cout << buf << std::endl;
    // close(sock);

    return 0;
}