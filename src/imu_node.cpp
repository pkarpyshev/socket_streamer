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
        if (accelerometer.connect() < 0){
            std::cout << "Acceleromter: connect error" << std::endl;
        } else {
            uint8_t id = accelerometer.who_am_i();
            std::cout << (int)id << std::endl;
            accelerometer.read_xyz();
            imu_msg.linear_acceleration.x = accelerometer.accelerations.x;
            imu_msg.linear_acceleration.y = accelerometer.accelerations.y;
            imu_msg.linear_acceleration.z = accelerometer.accelerations.z;
        }

        if (gyroscope.connect() < 0){
            std::cout << "Gyroscope: connect error" << std::endl;
        } else {
        //     res = i2c_smbus_read_byte_data(file, STATUS_REG);
        //     if( xyz_available(res) ) {
        //         LIS331DHL_accelerations xyz_accel = read_xyz_accel(file);
        //         imu_msg.linear_acceleration.x = xyz_accel.x;
        //         imu_msg.linear_acceleration.y = xyz_accel.y;
        //         imu_msg.linear_acceleration.z = xyz_accel.z;
        }

        imu_pub.publish(imu_msg);
        ros::spinOnce();
    }
    return 0;
}