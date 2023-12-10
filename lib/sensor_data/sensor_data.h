#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <stdint.h>

#define N_THERMISTORS 8
#define N_GPIO 3
#define N_DRIVE 8
#define N_BATTERY_CELLS 12

typedef struct{
    float battery_cell_voltage[N_BATTERY_CELLS];
    float battery_temp;
    float battery_voltage;
    float battery_current;
    float battery_percent;//(0 to 1)
    uint8_t drive_state[N_DRIVE];
    float thermistors[N_THERMISTORS];
    float ambiant_temp;
    float humidity;
    uint8_t gpio_out[N_GPIO];
    uint8_t estop_pwr_out;
    uint8_t estop_status;
}sensor_data_t;

extern sensor_data_t sensor_data;

#endif