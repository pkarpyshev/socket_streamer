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


struct LIS331DHL_accelerations {
    float x;
    float y;
    float z;
};

// uint8_t xyz_available(uint8_t status_reg){
//     return (status_reg & (1 << 3)) >> 3;
// }

// static inline int16_t read_z_axis(int file){
//     __s32 msb, lsb;
//     msb = i2c_smbus_read_byte_data(file, OUT_ZH);
//     lsb = i2c_smbus_read_byte_data(file, OUT_ZL);
//     return (msb << 8) | (lsb);
// }

// static inline int16_t read_x_axis(int file){
//     __s32 msb, lsb;
//     msb = i2c_smbus_read_byte_data(file, OUT_XH);
//     lsb = i2c_smbus_read_byte_data(file, OUT_XL);
//     return (msb << 8) | (lsb);
// }

// static inline int16_t read_y_axis(int file){
//     __s32 msb, lsb;    
//     msb = i2c_smbus_read_byte_data(file, OUT_YH);
//     lsb = i2c_smbus_read_byte_data(file, OUT_YL);
//     return (msb << 8) | (lsb);
// }

// LIS331DHL_accelerations read_xyz_accel(int file){
//     LIS331DHL_accelerations res;
//     res.x = (read_x_axis(file) >> 4) * scale;
//     res.y = (read_y_axis(file) >> 4) * scale;
//     res.z = (read_z_axis(file) >> 4) * scale;
//     return res;
// }

#endif

class LIS331DLH {
private:
    const int address = 0x18;
    const int file_id;
    const __u8 who_am_i_reg = 0x0F;

    const uint8_t ctrl_reg1 = 0x20;
    const uint8_t power_mode = (1 << 5);
    const uint8_t data_rate = (1 << 4);
    const uint8_t axes_enable = (1 << 2) | (1 << 1) | (1 << 0);
    const uint8_t ctrl_reg1_value = power_mode | data_rate | axes_enable;

    const uint8_t ctrl_reg4 = 0x23;
    const uint8_t block_data_update = (0 << 7);
    const uint8_t big_endian_selection = (0 << 6);
    const uint8_t full_sclae_selection = (0 << 5) | (1 << 4);
    const uint8_t self_test_sign = (0 << 2);
    const uint8_t self_test_enable = (0 << 1);
    const uint8_t spi_mode_selection = (0 << 0);
    const uint8_t ctrl_reg4_value = 
        block_data_update | big_endian_selection | full_sclae_selection | 
        self_test_sign | self_test_enable |spi_mode_selection;

    const double scale = 4.0f / 2048.0f;
    const uint8_t status_reg = 0x27;
    const uint8_t out_XL = 0x28;
    const uint8_t out_XH = 0x29;
    const uint8_t out_YL = 0x2A;
    const uint8_t out_YH = 0x2B;
    const uint8_t out_ZL = 0x2C;
    const uint8_t out_ZH = 0x2D;
    
    struct data_t {
        double x;
        double y;
        double z;
    };

    int read_axis(const uint8_t msb_reg, const uint8_t lsb_reg) const {
        __s32 msb, lsb;
        msb = i2c_smbus_read_byte_data(file_id, msb_reg);
        lsb = i2c_smbus_read_byte_data(file_id, lsb_reg);
        return (msb << 8) | (lsb);
    };
public:
    LIS331DLH(int file): file_id(file){

    };

    ~LIS331DLH(){

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
    };

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

    int read_xyz(){
        accelerations.x = (read_axis(out_XH, out_XL) >> 4) * scale;
        accelerations.y = (read_axis(out_YH, out_YL) >> 4) * scale;
        accelerations.z = (read_axis(out_ZH, out_ZL) >> 4) * scale;
        return 0;
    };

    data_t accelerations = {0.0f, 0.0f, 0.0f};
};