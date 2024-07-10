#ifndef MPU_6050__H__
#define MPU_6050__H__

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>

#define MPU_COMPAT invensense_mpu6050

struct printk_data_t {
	void *fifo_reserved; /* 1st word reserved for use by fifo */
	struct sensor_value accelerometer[3];
    struct sensor_value gyro[3];
};

struct mpu_6050_data{
    struct sensor_value accelerometer[3];
    struct sensor_value gyro[3];
};

const char *now_str(void);
int process_mpu6050(const struct device *dev, struct mpu_6050_data *mpu_sensor_data);


#endif // MPU_6050__H__