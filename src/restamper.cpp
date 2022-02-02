#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <time.h>

static sensor_msgs::Imu imu_msg;
static sensor_msgs::Image image_msg;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "imu0";

    imu_msg.linear_acceleration = msg->linear_acceleration;
    imu_msg.angular_velocity = msg->angular_velocity;
    imu_msg.orientation = msg->orientation;

    // std::cout << "Imu received. stamp: " << 
    //     imu_msg.header.stamp.sec << "." <<  imu_msg.header.stamp.nsec << std::endl;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    image_msg.header.stamp = ros::Time::now();
    image_msg.header.frame_id = "cam0";

    image_msg.height = msg->height;
    image_msg.width = msg->width;
    image_msg.encoding = msg->encoding;
    image_msg.is_bigendian = msg->is_bigendian;
    image_msg.step = msg->step;
    image_msg.data = msg->data;

    // std::cout << "Image received. stamp: " << 
    //     image_msg.header.stamp.sec << "." <<  image_msg.header.stamp.nsec << std::endl;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "restamper_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // subscribe to topics
    ros::Subscriber imu_sub = nh.subscribe("cf1/imu", 10, imuCallback);
    image_transport::Subscriber image_sub = it.subscribe("/camera/image", 10, imageCallback);
    
    // subscribe to topics
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 10);
    image_transport::Publisher image_pub = it.advertise("image_raw", 10);

    ros::Rate loop_rate(20);
    while(ros::ok()){
        imu_pub.publish(imu_msg);
        image_pub.publish(image_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}