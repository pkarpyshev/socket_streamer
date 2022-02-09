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

I3G4250D gyroscope;
LIS331DLH accelerometer;

int main(int argc, char *argv[]){
    char filename[20];
    int adapter_nr = 1;
    snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
    int file = open(filename, O_RDWR);
    if (file < 0){
        exit(1);
    }

    if (accelerometer.init(file) < 0) {
        std::cout << "LIS331DLH.init() error." << std::endl;
    }

    if (gyroscope.init(file) < 0){
        std::cout << "I3G4250D.init() error." << std::endl;
    }
    
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle nh;
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu0", 1);
    static sensor_msgs::Imu imu_msg;
    
    // auto start = std::chrono::steady_clock::now();
    int res = -1;
    // while(nh.ok()){
        // if (select_device(file, imu_address) < 0){
        //     std::cout << "Selecting accel error" << std::endl;
        // } else {
        //     res = i2c_smbus_read_byte_data(file, STATUS_REG);
        //     if( xyz_available(res) ) {
        //         LIS331DHL_accelerations xyz_accel = read_xyz_accel(file);
        //         imu_msg.linear_acceleration.x = xyz_accel.x;
        //         imu_msg.linear_acceleration.y = xyz_accel.y;
        //         imu_msg.linear_acceleration.z = xyz_accel.z;
        //     }

        // }
            // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
            // start = std::chrono::steady_clock::now()
            // std::cout << "imu elapsed: " << elapsed.count() << "ms" << std::endl;

        // if (select_device(file, gyro_address) < 0){
        //     std::cout << "Selecting gyro error" << std::endl;
        // } else {
        //     res = i2c_smbus_read_byte_data(file, STATUS_REG);
        //     if( xyz_available_gyro(res) ) {
        //         I3G4250D_gyroscope xyz_gyro = read_xyz_gyro(file);

        //         for (int i = 15; i >= 0; i--){
        //             std::cout << ((xyz_gyro.z >> i) & 1);
        //             if (i % 4 == 0) std::cout << " ";
        //         }
        //         std::cout << std::endl;
        //         // imu_msg.linear_acceleration.x = xyz_accel.x;
        //         // imu_msg.linear_acceleration.y = xyz_accel.y;
        //         // imu_msg.linear_acceleration.z = xyz_accel.z;
        //     }

        // }

    //     imu_pub.publish(imu_msg);
    //     ros::spinOnce();
    // }
    return 0;
}