#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include "ADS7828.h"

#include "interfaces.h"
#include "sensor_data.h"

#define BME680_ADDRESS 0x77
#define ADS7828_ADDRESS 72
#define ADC_TO_CURRENT 0.122100122100122

const int bat_therm_pin = PB0;
const int cell_pins[N_BATTERY_CELLS] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PC0, PC1, PC2, PC3};
const int drive_status_pins[N_DRIVE] = {PB5, PB8, PB9, PB10, PB12, PB13, PB14, PB15};
const int gpio_pins[N_GPIO] = {PC4, PC5, PC6};
const int estop_pin = PA12;
const int estop_status_pin = PA11;
const int current_sensor_pin = PB1;

//To compensate thermistors label mismatch between connector and adc channels
const uint8_t thermistor_map[N_THERMISTORS] = {3,2,1,0,7,6,5,4};

TwoWire wire1(PB7,PB6);

Adafruit_BME680 bme(&wire1);
ADS7828 ext_adc(ADS7828_ADDRESS,&wire1,ADS7828_SINGLE_ENDED, 1, 0);

void setup_interfaces(){
    bme.begin(BME680_ADDRESS);
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(0, 150); // 320*C for 150 ms
    //Setup ADC
    analogReadResolution(ADC_N_BITS);

    pinMode(bat_therm_pin, INPUT_ANALOG); 

    for(int i = 0; i < N_BATTERY_CELLS; i++){
        pinMode(cell_pins[i], INPUT_ANALOG);
    }
    for(int i = 0; i < N_DRIVE; i++){
        pinMode(drive_status_pins[i], INPUT);
    }
    for(int i = 0; i < N_GPIO; i++){
        pinMode(gpio_pins[i], OUTPUT);
    }
    pinMode(estop_pin, OUTPUT);
    pinMode(estop_status_pin, INPUT);
    pinMode(current_sensor_pin, INPUT_ANALOG);
}

/*
typedef struct{
    float battery_cell_voltage[N_BATTERY_CELLS];
    float battery_temp;
    float battery_current;
    uint8_t drive_state[N_DRIVE];
    float thermistors[N_THERMISTORS];
    float ambiant_temp;
    float humidity;
    uint8_t gpio_out[N_GPIO];
    uint8_t estop_pwr_out;
    uint8_t estop_status;
}sensor_data_t;
\*/

void update_interfaces(){
    sensor_data.battery_cell_voltage[0] = battery_calc_cell_v(analogRead(cell_pins[0]), 0);
    for(int i = 1; i < N_BATTERY_CELLS; i++){
        sensor_data.battery_cell_voltage[i] = battery_calc_cell_v(analogRead(cell_pins[i]), analogRead(cell_pins[i-1]));
    }
    sensor_data.battery_temp = thermistor_calc_temp(analogRead(bat_therm_pin));
    sensor_data.battery_current = calc_current(analogRead(current_sensor_pin));

    for(int i = 0; i < N_DRIVE; i++){
        sensor_data.drive_state[i] = digitalRead(drive_status_pins[i]);
    }

    for(int i = 0; i < N_THERMISTORS; i++){
        sensor_data.thermistors[i] = thermistor_calc_temp(ext_adc.read(thermistor_map[i]));
    }

    #ifndef USE_MICRO_ROS
        Serial3.println(ext_adc.read(thermistor_map[0]));
    #endif

    bme.performReading();
    sensor_data.ambiant_temp = bme.readTemperature();
    sensor_data.humidity = bme.readHumidity();
    
    for(int i = 0; i < N_GPIO; i++){
        digitalWrite(gpio_pins[i],sensor_data.gpio_out[i]);
    }
    digitalWrite(estop_pin,sensor_data.estop_pwr_out);

}

float thermistor_calc_temp(int adc_reading){
    //Find thermistor resistance from rdivider equation
    float resistance;
    resistance = THERM_PULL_UP/((ADC_MAX_VALUE / adc_reading) - 1);
    
    //Find thermistor temperature from steinart equation
    float temp;
    temp = (THERM_B*THERM_T0)/(THERM_B+THERM_T0*log(resistance/THERM_R0));
    //Degree K to degree C
    temp -= 273.15;

    return(temp);
}

float battery_calc_cell_v(uint16_t cell_reading, uint16_t prev_cell_reading){
    float result;
    result = ((cell_reading - prev_cell_reading)/ADC_MAX_VALUE) * ADC_VCC * CELL_RDIV_RATIO;
    return result;
}


float calc_current(uint16_t adc_reading){
    //6.6mv/A (+- 200A) centered at vcc/2
    float return_value = (adc_reading - (ADC_MAX_VALUE/2)) * ADC_TO_CURRENT;
    return(return_value);
}

float lowpass_filter(float previous, float input, float tau, float dt){
    float alpha = dt / (tau + dt);
    float output = alpha * input + (1 - alpha) * previous;
    return output;
}