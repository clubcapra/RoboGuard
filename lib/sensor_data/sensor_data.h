#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <stdint.h>

#define N_THERMISTORS 1
#define N_BATTERY_CELLS 12

typedef struct{
    float battery_cell_voltage[N_BATTERY_CELLS];
    float battery_temp;
    float battery_voltage;
    float battery_current;
    float battery_percent;//(0 to 1)
    float ambiant_temp;
    float humidity;
    uint8_t estop_pwr_out;
    uint8_t estop_status;
}sensor_data_t;

extern sensor_data_t sensor_data;

#endif