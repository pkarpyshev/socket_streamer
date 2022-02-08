#include <iostream>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "LIS331DLH.h"
#include "I3G4250D.h"
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

    if (init_LIS331DHL(file) < 0) {
        std::cout << "init_LIS331DHL error." << std::endl;
    }

    if (init_I3G4250D(file) < 0){
        std::cout << "init_I3G4250D error." << std::endl;
    }
    
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle nh;
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu0", 1);
    static sensor_msgs::Imu imu_msg;
    
    // auto start = std::chrono::steady_clock::now();
    int res = -1;
    while(nh.ok()){
        if (select_device(file, imu_address) < 0){
            std::cout << "Selecting accel error" << std::endl;
        }
        res = i2c_smbus_read_byte_data(file, WHO_AM_I);
        std::cout << "Accel responce:" << res << std::endl;
        // int res = i2c_smbus_read_byte_data(file, STATUS_REG);
        // uint8_t xyz_status = xyz_available(res);
        // if (xyz_status){
        //     LIS331DHL_accelerations xyz_accel = read_xyz(file);
        //     imu_msg.linear_acceleration.x = xyz_accel.x;
        //     imu_msg.linear_acceleration.y = xyz_accel.y;
        //     imu_msg.linear_acceleration.z = xyz_accel.z;

        //     // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
        //     // start = std::chrono::steady_clock::now()
        //     // std::cout << "imu elapsed: " << elapsed.count() << "ms" << std::endl;
        // }

        if (select_device(file, gyro_address) < 0){
            std::cout << "Selecting gyro error" << std::endl;
        }
        res = i2c_smbus_read_byte_data(file, WHO_AM_I);
        std::cout << "Gyro responce:" << res << std::endl;

        imu_pub.publish(imu_msg);
        ros::spinOnce();
    }
    return 0;
}