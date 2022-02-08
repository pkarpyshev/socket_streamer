#ifndef _LIS331DLH_H_
#define _LIS331DLH_H_

extern "C"{
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
    #include <stdio.h>
    #include <fcntl.h>
    #include <stdlib.h>
    #include <sys/ioctl.h>
    #include <stdint.h>
}

#define WHO_AM_I 0x0F

#define CTRL_REG1 0x20
static const uint8_t power_mode = (1 << 5);
static const uint8_t data_rate = (1 << 4);
static const uint8_t axes_enable = (1 << 2) | (1 << 1) | (1 << 0);
static const uint8_t ctrl_reg1_value = power_mode | data_rate | axes_enable;

#define CTRL_REG4 0x23
static const uint8_t block_data_update = (0 << 7);
static const uint8_t big_endian_selection = (0 << 6);
static const uint8_t full_sclae_selection = (0 << 5) | (1 << 4);
static const uint8_t self_test_sign = (0 << 2);
static const uint8_t self_test_enable = (0 << 1);
static const uint8_t spi_mode_selection = (0 << 0);
static const uint8_t ctrl_reg4_value = 
    block_data_update | big_endian_selection | full_sclae_selection | 
    self_test_sign | self_test_enable |spi_mode_selection;

#define STATUS_REG 0x27

#define OUT_XL 0x28
#define OUT_XH 0x29
#define OUT_YL 0x2A
#define OUT_YH 0x2B
#define OUT_ZL 0x2C
#define OUT_ZH 0x2D

static const float scale = 4.0f / 2048;
struct LIS331DHL_accelerations {
    float x;
    float y;
    float z;
};

// int file;
const int imu_address = 0x18;

int init_LIS331DHL(int file){
    // char filename[20];
    // int adapter_nr = 1;

    // snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
    // int file = open(filename, O_RDWR);
    // if (file < 0){
    //     return -1;
    // }

    if (ioctl(file, I2C_SLAVE, imu_address) < 0){
        return -1;
    }

    __s32 res = 1;
    res = i2c_smbus_read_byte_data(file, (__u8)0x0F);
    if (res < 0){
        std::cout << "WHO_AM_I: " << res << std::endl;
        return -1;
    }
    
    res = i2c_smbus_write_byte_data(file, CTRL_REG1, ctrl_reg1_value);
    if (res < 0){
        std::cout << "CTRL_REG1 status: " << res << std::endl;
        return -1;
    }

    res = i2c_smbus_write_byte_data(file, CTRL_REG4, ctrl_reg4_value);
    if (res < 0){
        std::cout << "CTRL_REG4 status: " << res << std::endl;
        return -1;
    }

    // res = i2c_smbus_read_byte_data(file, STATUS_REG);
    return 0;
}



uint8_t xyz_available(uint8_t status_reg){
    return (status_reg & (1 << 3)) >> 3;
}

static inline int16_t read_z_axis(int file){
    __s32 msb, lsb;
    msb = i2c_smbus_read_byte_data(file, OUT_ZH);
    lsb = i2c_smbus_read_byte_data(file, OUT_ZL);
    return (msb << 8) | (lsb);
}

static inline int16_t read_x_axis(int file){
    __s32 msb, lsb;
    msb = i2c_smbus_read_byte_data(file, OUT_XH);
    lsb = i2c_smbus_read_byte_data(file, OUT_XL);
    return (msb << 8) | (lsb);
}

static inline int16_t read_y_axis(int file){
    __s32 msb, lsb;    
    msb = i2c_smbus_read_byte_data(file, OUT_YH);
    lsb = i2c_smbus_read_byte_data(file, OUT_YL);
    return (msb << 8) | (lsb);
}

LIS331DHL_accelerations read_xyz(int file){
    LIS331DHL_accelerations res;
    res.x = (read_x_axis(file) >> 4) * scale;
    res.y = (read_y_axis(file) >> 4) * scale;
    res.z = (read_z_axis(file) >> 4) * scale;
    return res;
}
#endif