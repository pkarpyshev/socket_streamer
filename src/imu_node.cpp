extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
    #include <stdio.h>
    #include <fcntl.h>
    #include <stdlib.h>
    #include <sys/ioctl.h>
}

#include <iostream>
#include <chrono>
#include "LIS331DLH.h"

int file;
int adapter_nr = 1;
char filename[20];
int addr_accel = 0x18;



int main(){
    snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
    file = open(filename, O_RDWR);
    if (file < 0){
        exit(1);
    }

    if (ioctl(file, I2C_SLAVE, addr_accel) < 0){
        exit(2);
    }

    __s32 res = 1;
    res = i2c_smbus_read_byte_data(file, (__u8)0x0F);
    std::cout << "Who am i: " << res << std::endl;
    
    // std::cout << (int)ctrl_reg1_value << std::endl;
    res = i2c_smbus_write_byte_data(file, CTRL_REG1, ctrl_reg1_value);
    std::cout << "CTRL_REG1 status: " << res << std::endl;

    // std::cout << (int)ctrl_reg4_value << std::endl;
    res = i2c_smbus_write_byte_data(file, CTRL_REG4, ctrl_reg4_value);
    std::cout << "CTRL_REG4 status: " << res << std::endl;

    res = i2c_smbus_read_byte_data(file, STATUS_REG);
    
    auto start = std::chrono::steady_clock::now();
    while(true){
        uint8_t xyz_status = zyx_available(res);
        // std::cout << "STATUS_REG: " << res << 
        //     "; XYZ status: " << (int)xyz_status << std::endl;
        if (xyz_status){
            accell xyz_accel = read_xyz(file);
            // std::cout << xyz_accel.x << "; " 
            //         << xyz_accel.y << "; " 
            //         << xyz_accel.z << std::endl;

            auto end = std::chrono::steady_clock::now();
            auto elapsed = 
                std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                // std::chrono::duration_cast<std::chrono::milliseconds>(start);
            start = end;
            std::cout << "elapsed: " << elapsed.count() << "ms" << std::endl;
        }
    }
    return 0;
}