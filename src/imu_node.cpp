#include <iostream>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

extern "C"{
    #include "LIS331DLH.h"
    #include "I3G4250D.h"
}

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
    ros::Rate publish_rate(200);

    auto start = std::chrono::steady_clock::now();
    while(nh.ok()){
        start = std::chrono::steady_clock::now();
        if (accelerometer.connect() == 0){
            accelerometer.read_xyz();
            imu_msg.linear_acceleration.x = accelerometer.accelerations.x;
            imu_msg.linear_acceleration.y = accelerometer.accelerations.y;
            imu_msg.linear_acceleration.z = accelerometer.accelerations.z;
        } else {
            std::cout << "Acceleromter: connect error" << std::endl;
        }
        auto accel_duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();

        if (gyroscope.connect() == 0){
            gyroscope.read_xyz();
            imu_msg.angular_velocity.x = gyroscope.velocity.x;
            imu_msg.angular_velocity.y = gyroscope.velocity.y;
            imu_msg.angular_velocity.z = gyroscope.velocity.z;
        } else {
            std::cout << "Gyroscope: connect error" << std::endl;
        }
        auto gyro_duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();

        imu_pub.publish(imu_msg);
        auto common_duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
        std::cout << accel_duration << "; " << gyro_duration << "; " << common_duration <<std::endl;
        publish_rate.sleep();
    }
    return 0;
}