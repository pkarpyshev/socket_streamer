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

// static const float scale2 = 245.0f / 2048;

class I3G4250D {
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

    const uint8_t status_reg = 0x27;
    const uint8_t out_XL = 0x28;
    const uint8_t out_XH = 0x29;
    const uint8_t out_YL = 0x2A;
    const uint8_t out_YH = 0x2B;
    const uint8_t out_ZL = 0x2C;
    const uint8_t out_ZH = 0x2D;

    struct data_t {
        int16_t x;
        int16_t y;
        int16_t z;
    };

    inline int16_t read_axis(const uint8_t msb_reg, const uint8_t lsb_reg) const {
        __s32 msb, lsb;
        msb = i2c_smbus_read_byte_data(file_id, msb_reg);
        lsb = i2c_smbus_read_byte_data(file_id, lsb_reg);
        return (msb << 8) | (lsb);
    };

    inline int get_data_status(){
        return (i2c_smbus_read_byte_data(file_id, status_reg) & (1 << 3) >> 3);
    };

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

    int who_am_i(){
        if(i2c_smbus_read_byte_data(file_id, who_am_i_reg) < 0){
            printf("Accelleromter: who_am_i failed. Errno %s \n",strerror(errno));
            return -1;
        }
        return 0;
    }

    int init(){
        if (connect()){
            return -1;
        }
        
        if (who_am_i()){
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

    int16_t read_xyz(){
        if (get_data_status()){
            accelerations.x = read_axis(out_XH, out_XL);
            accelerations.y = read_axis(out_YH, out_YL);
            accelerations.z = read_axis(out_ZH, out_ZL);
        }
        return 0;
    };
    
    data_t accelerations = {0, 0, 0};
};

#endif