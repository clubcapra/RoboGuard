#ifndef INTERFACES_H
#define INTERFACES_H

#define THERM_R0 10000.0        // Nominal thermistance resistance
#define THERM_T0 (25+273.15)    // Nominal thermistance temp (degree K)
#define THERM_B 3984.0          // Thermistor B value
#define THERM_PULL_UP 10000.0   // Thermistor pull-up value
#define ADC_MAX_VALUE 4095.0    // Max adc value

void setup_interfaces();
void update_interfaces();

float thermistor_calc_temp(int adc_reading);

#endif