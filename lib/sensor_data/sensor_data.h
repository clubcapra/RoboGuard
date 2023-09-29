#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H
#define N_THERMISTORS 8
typedef struct{
    float thermistors[N_THERMISTORS];
}sensor_data_t;

extern sensor_data_t sensor_data;

#endif