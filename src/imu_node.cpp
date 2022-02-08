#include <iostream>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "LIS331DLH.h"

int main(int argc, char *argv[]){
    int imu_file = init_LIS331DHL();
    
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle nh;
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu0", 1);

    static sensor_msgs::Imu imu_msg;
    
    // auto start = std::chrono::steady_clock::now();
    while(nh.ok()){
        int res = i2c_smbus_read_byte_data(imu_file, STATUS_REG);
        uint8_t xyz_status = xyz_available(res);
        if (xyz_status){
            LIS331DHL_accelerations xyz_accel = read_xyz(imu_file);
            imu_msg.linear_acceleration.x = xyz_accel.x;
            imu_msg.linear_acceleration.y = xyz_accel.y;
            imu_msg.linear_acceleration.z = xyz_accel.z;

            imu_pub.publish(imu_msg);
            ros::spinOnce();
            // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
            // start = std::chrono::steady_clock::now()
            // std::cout << "elapsed: " << elapsed.count() << "ms" << std::endl;
        }
    }
    return 0;
}