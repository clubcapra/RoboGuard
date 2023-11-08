#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#define N_THERMISTORS 8
#define N_BATTERY_CELLS 12

typedef struct{
    float thermistors[N_THERMISTORS];
    float battery_cell_voltage[N_BATTERY_CELLS];
    float battery_temp;
}sensor_data_t;

extern sensor_data_t sensor_data;

#endif