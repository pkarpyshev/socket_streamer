#ifndef _I3G4250D_H_
#define _I3G4250D_H_

extern "C"{
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
    #include <stdio.h>
    #include <fcntl.h>
    #include <stdlib.h>
    #include <sys/ioctl.h>
    #include <stdint.h>
}

#include <iostream>

const int gyro_address = 0x68;
int init_I3G4250D(int file){


    if (ioctl(file, I2C_SLAVE, gyro_address) < 0){
        return -1;
    }

    __s32 res = 1;
    res = i2c_smbus_read_byte_data(file, (__u8)0x0F);
    if (res < 0){
        std::cout << "WHO_AM_I: " << res << std::endl;
        return -1;
    }
    
    // res = i2c_smbus_write_byte_data(file, CTRL_REG1, ctrl_reg1_value);
    // if (res < 0){
    //     std::cout << "CTRL_REG1 status: " << res << std::endl;
    //     return -1;
    // }

    // res = i2c_smbus_write_byte_data(file, CTRL_REG4, ctrl_reg4_value);
    // if (res < 0){
    //     std::cout << "CTRL_REG4 status: " << res << std::endl;
    //     return -1;
    // }

    // res = i2c_smbus_read_byte_data(file, STATUS_REG);
    return file;
}

#endif