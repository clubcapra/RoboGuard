#include <Arduino.h>
#include "interfaces.h"
#include "sensor_data.h"

#define ADC_N_BITS 12
const int thermistor_pin[N_THERMISTORS] = {PA0, PA1, PA2, PA3, PC0, PC1, PC2, PC3};

void setup_interfaces(){
    //Setup ADC
    analogReadResolution(ADC_N_BITS);
    for(int i = 0; i < N_THERMISTORS; i++){
        pinMode(thermistor_pin[i], INPUT_ANALOG);
    }
}
void update_interfaces(){
    for(int i = 0; i < N_THERMISTORS; i++){
        sensor_data.thermistors[i] = thermistor_calc_temp(analogRead(thermistor_pin[i]));
    }
}

float thermistor_calc_temp(int adc_reading){
    //Find thermistor resistance from rdivider equation
    float resistance;
    resistance = THERM_PULL_UP/((ADC_MAX_VALUE / adc_reading) - 1);
    
    //Find thermistor temperature from steinart equation
    float temp;
    temp = (THERM_B*THERM_T0)/(THERM_B+THERM_T0*log(resistance/THERM_R0));
    // Degree K to degree C
    temp -= 273.15;

    return(temp);
}