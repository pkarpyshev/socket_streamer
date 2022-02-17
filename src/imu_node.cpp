#include <iostream>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

extern "C"{
    #include "LIS331DLH.h"
    #include "I3G4250D.h"
}

static const double ros_freq = 110;

uint8_t select_device(int file, int address){
    if (ioctl(file, I2C_SLAVE, address) < 0){
        return -1;
    }
    return 0;
}


int main(int argc, char *argv[]){
    char filename[20];
    int adapter_nr = 1;
    snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
    int file = open(filename, O_RDWR);
    if (file < 0){
        exit(1);
    }

    I3G4250D gyroscope(file);
    LIS331DLH accelerometer(file);

    if (accelerometer.init() < 0) {
        std::cout << "LIS331DLH.init() error." << std::endl;
    } else {
        std::cout << "LIS331DLH initialized." << std::endl;
    }

    if (gyroscope.init() < 0){
        std::cout << "I3G4250D.init() error." << std::endl;
    } else {
        std::cout << "I3G4250D initialized." << std::endl;
    }
    
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle nh;

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu0", 10);
    static sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = "imu0";
    imu_msg.orientation_covariance = {99999.9, 0.0, 0.0, 0.0, 99999.9, 0.0, 0.0, 0.0, 99999.9};
    imu_msg.linear_acceleration_covariance = {99999.9, 0.0, 0.0, 0.0, 99999.9, 0.0, 0.0, 0.0, 99999.9};
    imu_msg.angular_velocity_covariance = {99999.9, 0.0, 0.0, 0.0, 99999.9, 0.0, 0.0, 0.0, 99999.9};
    imu_msg.orientation.w = 1.0;

    ros::Rate publish_rate(ros_freq);
    while(nh.ok()){
        if (accelerometer.connect() == 0){
            accelerometer.read_xyz();
            imu_msg.linear_acceleration.x = accelerometer.accelerations.x;
            imu_msg.linear_acceleration.y = accelerometer.accelerations.y;
            imu_msg.linear_acceleration.z = accelerometer.accelerations.z;
        } else {
            std::cout << "Acceleromter: connect error" << std::endl;
        }

<<<<<<< HEAD
        // if (gyroscope.connect() == 0){
        //     gyroscope.read_xyz();
        //     imu_msg.angular_velocity.x = gyroscope.velocity.x;
        //     imu_msg.angular_velocity.y = gyroscope.velocity.y;
        //     imu_msg.angular_velocity.z = gyroscope.velocity.z;
        // } else {
        //     std::cout << "Gyroscope: connect error" << std::endl;
        // }

        if (imu_sensor.connect() == 0){
            imu_sensor.read_xyz();
            imu_msg.linear_acceleration.x = imu_sensor.linear_accel.x;
            imu_msg.linear_acceleration.y = imu_sensor.linear_accel.y;
            imu_msg.linear_acceleration.z = imu_sensor.linear_accel.z;

            imu_msg.angular_velocity.x = imu_sensor.angular_velocity.x;
            imu_msg.angular_velocity.y = imu_sensor.angular_velocity.y;
            imu_msg.angular_velocity.z = imu_sensor.angular_velocity.z;
        } else {
            std::cout << "Imu_sensor: connect error" << std::endl;
=======
        if (gyroscope.connect() == 0){
            gyroscope.read_xyz();
            imu_msg.angular_velocity.x = gyroscope.velocity.x;
            imu_msg.angular_velocity.y = gyroscope.velocity.y;
            imu_msg.angular_velocity.z = gyroscope.velocity.z;
        } else {
            std::cout << "Gyroscope: connect error" << std::endl;
>>>>>>> parent of 40ccd85... Test MPU6050 connection
        }
        
        imu_msg.header.stamp = ros::Time::now();

        imu_pub.publish(imu_msg);
        publish_rate.sleep();
    }
    return 0;
}