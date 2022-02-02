#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <time.h>


// publish to topics
ros::Publisher imu_pub;
image_transport::Publisher image_pub;

static sensor_msgs::Imu imu_msg;
static sensor_msgs::Image image_msg;
ros::Time last_synch_time;
ros::Time current_synch_time;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    
    imu_msg.header.frame_id = "imu0";

    imu_msg.linear_acceleration = msg->linear_acceleration;
    imu_msg.angular_velocity = msg->angular_velocity;
    imu_msg.orientation = msg->orientation;
    imu_msg.orientation.w = 1;

    imu_msg.orientation_covariance = {99999.9, 0.0, 0.0, 0.0, 99999.9, 0.0, 0.0, 0.0, 99999.9};
    imu_msg.linear_acceleration_covariance = {99999.9, 0.0, 0.0, 0.0, 99999.9, 0.0, 0.0, 0.0, 99999.9};
    imu_msg.angular_velocity_covariance = {99999.9, 0.0, 0.0, 0.0, 99999.9, 0.0, 0.0, 0.0, 99999.9};

    current_synch_time = ros::Time::now();
    imu_msg.header.stamp = current_synch_time;
    if ((current_synch_time.nsec > last_synch_time.nsec) || (current_synch_time.sec > last_synch_time.sec)){
        imu_pub.publish(imu_msg);
        last_synch_time = current_synch_time;
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    
    image_msg.header.frame_id = "cam0";

    image_msg.height = msg->height;
    image_msg.width = msg->width;
    image_msg.encoding = msg->encoding;
    image_msg.is_bigendian = msg->is_bigendian;
    image_msg.step = msg->step;
    image_msg.data = msg->data;
    
    current_synch_time = ros::Time::now();
    image_msg.header.stamp = current_synch_time;
    if ((current_synch_time.nsec > last_synch_time.nsec) || (current_synch_time.sec > last_synch_time.sec)){
        image_pub.publish(image_msg);
        last_synch_time = current_synch_time;
    }
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "restamper_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    
    imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 0);
    image_pub = it.advertise("image_raw", 0);

    // subscribe to topics
    ros::Subscriber imu_sub = nh.subscribe("cf1/imu", 1, imuCallback);
    image_transport::Subscriber image_sub = it.subscribe("/camera/image", 1, imageCallback);
    
    ros::spin();
    // ros::Rate loop_rate(60);
    while(ros::ok()){
        
        // current_synch_time = ros::Time::now();
        // if ((current_synch_time.nsec > last_synch_time.nsec) || (current_synch_time.sec > last_synch_time.sec)){
        //     imu_pub.publish(imu_msg);
        //     image_pub.publish(image_msg);
            // std::cout << current_synch_time.sec << current_synch_time.nsec << " vs. " <<
            //     last_synch_time.sec << last_synch_time.nsec << std::endl;
        // }

        // last_synch_time = current_synch_time;
        ros::spinOnce();
        // loop_rate.sleep();
    }

    return 0;
}