#ifndef _MPU6050_H_
#define _MPU6050_H_

// https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

extern "C"{
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
    #include <stdio.h>
    #include <fcntl.h>
    #include <stdlib.h>
    #include <sys/ioctl.h>
    #include <stdint.h>
}

#define GYRO_CONFIG_REG     0x1B
#define ACCEL_CONFIG_REG    0x1C
#define I2C_MST_CTRL_REG    0x24
#define FIFO_EN_REG         0x23



class MPU6050 {
private:
    const int file_id;
    const int address = 0x68;
    const __u8 who_am_i_reg = 0x75;

    const uint8_t gyro_self_test = 0;
    const uint8_t gyro_full_scale = 0;
    const uint8_t gyro_config = gyro_self_test | gyro_full_scale;

    const uint8_t accel_self_test = 0;
    const uint8_t accel_full_scale = 0;
    const uint8_t accel_config = accel_self_test | accel_full_scale;

    const uint8_t i2c_master_ctrl = 0;

    const uint8_t fifo_en = 0;
    // const uint8_t ctrl_reg4 = 0x23;
    // const uint8_t block_data_update = (0 << 7);
    // const uint8_t big_endian_selection = (0 << 6);
    // const uint8_t full_sclae_selection = (0 << 5) | (1 << 4);
    // const uint8_t self_test_sign = (0 << 2);
    // const uint8_t self_test_enable = (0 << 1);
    // const uint8_t spi_mode_selection = (0 << 0);
    // const uint8_t ctrl_reg4_value = 
    //     block_data_update | big_endian_selection | full_sclae_selection | 
    //     self_test_sign | self_test_enable |spi_mode_selection;

    // const float scale = 9.81f;              // from g to m/s^2
    // const float resolution = 4.0f / 2048;   // from bits to g
    // const uint8_t status_reg = 0x27;
    // const uint8_t out_XL = 0x28;
    // const uint8_t out_XH = 0x29;
    // const uint8_t out_YL = 0x2A;
    // const uint8_t out_YH = 0x2B;
    // const uint8_t out_ZL = 0x2C;
    // const uint8_t out_ZH = 0x2D;
    
    struct data_t {
        float x;
        float y;
        float z;
    };

    inline int16_t read_axis(const uint8_t msb_reg, const uint8_t lsb_reg) const {
        __s32 msb, lsb;
        // msb = i2c_smbus_read_byte_data(file_id, msb_reg);
        // lsb = i2c_smbus_read_byte_data(file_id, lsb_reg);
        return (msb << 8) | (lsb);
    };

    inline int get_data_status(){
        // return (i2c_smbus_read_byte_data(file_id, status_reg) & (1 << 3) >> 3);

        // detele
        return 0;
    };
public:
    MPU6050(int file): file_id(file){};

    ~MPU6050(){};
    
    // Connect to device
    int connect(){
        if (ioctl(file_id, I2C_SLAVE, address) < 0){
            printf("Accelleromter: ioctl failed. Errno %s \n", strerror(errno));
            return -1;
        }
        return 0;
    };

    int who_am_i(){
        // if(i2c_smbus_read_byte_data(file_id, who_am_i_reg) < 0){
        //     printf("Accelleromter: who_am_i failed. Errno %s \n",strerror(errno));
        //     return -1;
        // }
        return 0;
    };

    int init(){
        if (connect()){
            return -1;
        }
        
        // if (who_am_i()){
        //     return -1;
        // }
        
        if (i2c_smbus_write_byte_data(file_id, GYRO_CONFIG_REG, gyro_config) < 0){
            printf("GYRO: gyro_config failed. Errno %s \n",strerror(errno));
            return -1;
        }

        if (i2c_smbus_write_byte_data(file_id, ACCEL_CONFIG_REG, accel_config) < 0){
            printf("ACCEL: accel_config failed. Errno %s \n",strerror(errno));
            return -1;
        }
        
        if (i2c_smbus_write_byte_data(file_id, I2C_MST_CTRL_REG, i2c_master_ctrl) < 0){
            printf("SENSOR: i2c_master_ctrl failed. Errno %s \n",strerror(errno));
            return -1;
        }

        if (i2c_smbus_write_byte_data(file_id, FIFO_EN_REG, fifo_en) < 0){
            printf("SENSOR: fifo_en failed. Errno %s \n",strerror(errno));
            return -1;
        }
        // if (i2c_smbus_write_byte_data(file_id, ctrl_reg4, ctrl_reg4_value) < 0){
        //     printf("Accelleromter: ctrl_reg4 failed. Errno %s \n",strerror(errno));
        //     return -1;
        // }
        return 0;
    };

    int16_t read_xyz(){
        // if (get_data_status()){
        //     accelerations.x = (read_axis(out_XH, out_XL) >> 4) * resolution * scale;
        //     accelerations.y = (read_axis(out_YH, out_YL) >> 4) * resolution * scale;
        //     accelerations.z = (read_axis(out_ZH, out_ZL) >> 4) * resolution * scale;
        // }
        return 0;
    };

    data_t accelerations = {0, 0, 0};
};
#endif