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

// 

// #define STATUS_REG 0x27

// #define OUT_XL 0x28
// #define OUT_XH 0x29
// #define OUT_YL 0x2A
// #define OUT_YH 0x2B
// #define OUT_ZL 0x2C
// #define OUT_ZH 0x2D

// struct I3G4250D_gyroscope {
//     int16_t x;
//     int16_t y;
//     int16_t z;
// };


// static const float scale2 = 245.0f / 2048;

// uint8_t xyz_available_gyro(uint8_t status_reg){
//     return (status_reg & (1 << 3)) >> 3;
// }

// static inline int16_t read_z_axis2(int file){
//     __s32 msb, lsb;
//     msb = i2c_smbus_read_byte_data(file, OUT_ZH);
//     lsb = i2c_smbus_read_byte_data(file, OUT_ZL);
//     return (msb << 8) | (lsb);
// }

// static inline int16_t read_x_axis2(int file){
//     __s32 msb, lsb;
//     msb = i2c_smbus_read_byte_data(file, OUT_XH);
//     lsb = i2c_smbus_read_byte_data(file, OUT_XL);
//     return (msb << 8) | (lsb);
// }

// static inline int16_t read_y_axis2(int file){
//     __s32 msb, lsb;    
//     msb = i2c_smbus_read_byte_data(file, OUT_YH);
//     lsb = i2c_smbus_read_byte_data(file, OUT_YL);
//     return (msb << 8) | (lsb);
// }

// I3G4250D_gyroscope read_xyz_gyro(int file){
//     I3G4250D_gyroscope res;
//     res.x = (read_x_axis2(file) );
//     res.y = (read_y_axis2(file) );
//     res.z = (read_z_axis2(file) );
//     return res;
// }

class I3G4250D
{
private:
    const int address = 0x68;
    const int file_id;
    const __u8 who_am_i_reg = 0x0F;

    const uint8_t ctrl_reg1 = 0x20;
    const uint8_t data_rate = (1 << 7) | (0 << 6);
    const uint8_t bandwitdh = (0 << 5) | (0 << 4);
    const uint8_t power_mode = (1 << 3);
    const uint8_t axes_enable = (1 << 2) | (1 << 1) | (1 << 0);
    const uint8_t ctrl_reg1_value = 
        data_rate | bandwitdh |
        power_mode | axes_enable;

    const uint8_t ctrl_reg4 = 0x23;
    const uint8_t big_endian_selection = (0 << 6);
    const uint8_t full_sclae_selection = (0 << 5) | (0 << 4);
    const uint8_t self_test_enable = (0 << 2) | (0 << 1);
    const uint8_t spi_mode_selection = (0 << 0);
    const uint8_t ctrl_reg4_value = 
        big_endian_selection | full_sclae_selection | 
        self_test_enable |spi_mode_selection;

public:
    I3G4250D(int file): file_id(file) {

    };

    ~I3G4250D(){

    };

    // Connect to device
    int connect(){
        if (ioctl(file_id, I2C_SLAVE, address) < 0){
            printf("Gyroscope: ioctl failed. Errno %s \n",strerror(errno));
            return -1;
        }
        return 0;
    };

    uint8_t who_am_i(){
        return i2c_smbus_read_byte_data(file_id, who_am_i_reg);
    }

    int init(){
        if (connect()){
            return -1;
        }
        
        if (i2c_smbus_read_byte_data(file_id, who_am_i_reg) < 0){
            printf("Gyroscope: who_am_i failed. Errno %s \n",strerror(errno));
            return -1;
        }
        
        if (i2c_smbus_write_byte_data(file_id, ctrl_reg1, ctrl_reg1_value) < 0){
            printf("Gyroscope: ctrl_reg1 failed. Errno %s \n",strerror(errno));
            return -1;
        }

        if (i2c_smbus_write_byte_data(file_id, ctrl_reg4, ctrl_reg4_value) < 0){
            printf("Gyroscope: ctrl_reg4 failed. Errno %s \n",strerror(errno));
            return -1;
        }
        return 0;
    };
};

#endif