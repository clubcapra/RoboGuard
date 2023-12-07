#ifndef INTERFACES_H
#define INTERFACES_H

#define THERM_R0 10000.0        // Nominal thermistance resistance
#define THERM_T0 (25+273.15)    // Nominal thermistance temp (degree K)
#define THERM_B 3984.0          // Thermistor B value
#define THERM_PULL_UP 1000.0   // Thermistor pull-up value
#define ADC_MAX_VALUE 4095.0    // Max adc value
#define ADC_N_BITS 12
#define ADC_VCC 3.3
#define CELL_RDIV_RATIO 16


#define THERM_TAU 1
#define CELL_TAU 1
 
void setup_interfaces();
void update_interfaces();

uint8_t check_estop();

float thermistor_calc_temp(int adc_reading);
float battery_calc_cell_v(uint16_t cell_reading, uint16_t prev_cell_reading);
float lowpass_filter(float previous, float input, float tau, float dt);
float calc_current(uint16_t adc_reading);

#endif