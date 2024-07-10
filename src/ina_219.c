#include "ina_219.h"

int process_ina219(const struct device *dev, struct ina_219_data *ina_sensor_data)
{
    int rc = sensor_sample_fetch(dev);
    if (rc) {
        printf("\nCould not fetch sensor data.\n");
        return rc;
    }

    extern struct k_mutex ina_sensor_mutex; 

    k_mutex_lock(&ina_sensor_mutex, K_FOREVER);

    sensor_channel_get(dev, SENSOR_CHAN_CURRENT, &ina_sensor_data->current);

    k_mutex_unlock(&ina_sensor_mutex);

    return rc;
}
