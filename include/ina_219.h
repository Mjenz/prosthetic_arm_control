#ifndef INA_219__H__
#define INA_219__H__

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <inttypes.h>
#include <stdio.h>

#define INA_COMPAT ti_ina219

struct ina_219_data{
    struct sensor_value vbus;
    struct sensor_value power;
    struct sensor_value current;
};

int process_ina219(const struct device *dev, struct ina_219_data *ina_sensor_data);

#endif // INA_219__H__