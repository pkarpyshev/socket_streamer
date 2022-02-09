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
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu0", 1);
    static sensor_msgs::Imu imu_msg;
    
    // auto start = std::chrono::steady_clock::now();
    while(nh.ok()){
        if (accelerometer.connect() == 0){
            accelerometer.read_xyz();
            imu_msg.linear_acceleration.x = accelerometer.accelerations.x;
            imu_msg.linear_acceleration.y = accelerometer.accelerations.y;
            imu_msg.linear_acceleration.z = accelerometer.accelerations.z;
        } else {
            std::cout << "Acceleromter: connect error" << std::endl;
        }

        if (gyroscope.connect() == 0){
            gyroscope.read_xyz();
            // for (int i = 15; i >= 0; i--){
            //     std::cout << ((gyroscope.accelerations.z >> i) & 1);
            //     if (i % 4 == 0)
            //         std::cout << " ";
            // }
            std::cout << gyroscope.velocity.z << std::endl;
            imu_msg.angular_velocity.x = gyroscope.velocity.x;
            imu_msg.angular_velocity.y = gyroscope.velocity.y;
            imu_msg.angular_velocity.z = gyroscope.velocity.z;
        } else {
            std::cout << "Gyroscope: connect error" << std::endl;
        }

        imu_pub.publish(imu_msg);
        ros::spinOnce();
    }
    return 0;
}