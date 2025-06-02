#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include "ADS7828.h"
#include "bms_Bq769x0.h"
// #include "bq769x0.h"

#include "interfaces.h"
#include "sensor_data.h"

#define FAULT_NO_FAULT 0
#define FAULT_BATT_UNDER_V 1
#define FAULT_BATT_OVER_V 2
#define FAULT_BATT_CELL_UNDER_V 3
#define FAULT_BATT_CELL_OVER_V 4
#define FAULT_BATT_OVER_CURR 5
#define FAULT_BATT_OVER_TEMP 6
#define FAULT_AMBIANT_OVERTEMP 7
#define FAULT_AMBIANT_HUMIDITY 8

#define BATT_MIN_V 38.4
#define BATT_MAX_V 51.4
#define BATT_CELL_MIN_V 3.5
#define BATT_CELL_MAX_V 4.2
#define BATT_MAX_TEMP 80 // in degree C
#define BATT_MIN_TEMP 10
#define BATT_CELL_MIN_V_BMS 3500 // in mV
#define BATT_CELL_MAX_V_BMS 4200 // in mV

#define MOTOR_MAX_TEMP 80   // in degree c
#define AMBIANT_MAX_TEMP 80 // in degree c
#define AMBIANT_MAX_HUM 50  // in TBD

#define BME680_ADDRESS 0x77
#define b2I2CAddress 0x08
#define ADS7828_ADDRESS 72
#define ADC_TO_CURRENT 0.122100122100122

const int bat_therm_pin = PB0;
// const int cell_pins[N_BATTERY_CELLS] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PC0, PC1, PC2, PC3};

const int alert_BMS = PA15;
const int boot_BMS = PC7;

const int pwr_supply_mode_pin = PA0;

const int estop_pin = PA12;
const int estop_status_pin = PA11;
const int current_sensor_pin = PB1;

int status_pwr_sup_mode = 0;

// To compensate thermistors label mismatch between connector and adc channels
// const uint8_t thermistor_map[N_THERMISTORS] = {3,2,1,0,7,6,5,4};

TwoWire wire1(PB7, PB6);
TwoWire wire2(PC12, PB10);

Adafruit_BME680 bme(&wire1);

bms_bq769x0 bms(&wire2);
// bq769x0 bms(&wire2,bq76940, b2I2CAddress);

void setup_interfaces()
{
    pinMode(estop_pin, OUTPUT);
    pinMode(estop_status_pin, INPUT);
    pinMode(pwr_supply_mode_pin, INPUT);
    //status_pwr_sup_mode = digitalRead(pwr_supply_mode_pin);
    bms.init_bq769x0(alert_BMS, boot_BMS, THERM_B, BATT_MIN_TEMP, BATT_MAX_TEMP, BATT_CELL_MIN_V_BMS, BATT_CELL_MAX_V_BMS);

    bme.begin(BME680_ADDRESS);
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(0, 150); // 320*C for 150 ms
    // Setup ADC
    analogReadResolution(ADC_N_BITS);

    pinMode(bat_therm_pin, INPUT_ANALOG);

    /*for(int i = 0; i < N_BATTERY_CELLS; i++){
        pinMode(cell_pins[i], INPUT_ANALOG);
    }*/

    pinMode(current_sensor_pin, INPUT_ANALOG);
}

void update_interfaces()
{
    
    if (status_pwr_sup_mode != digitalRead(pwr_supply_mode_pin)&&(sensor_data.battery_voltage==0))
    {
        status_pwr_sup_mode = digitalRead(pwr_supply_mode_pin);
        switch (status_pwr_sup_mode)
        {
        case 0:
            bms.shutdown();
            break;
        case 1:
        default:
            bms.init_bq769x0(alert_BMS, boot_BMS, THERM_B, BATT_MIN_TEMP, BATT_MAX_TEMP, BATT_CELL_MIN_V_BMS, BATT_CELL_MAX_V_BMS);
            break;
        }
    }
    bms.update();

    // sensor_data.battery_cell_voltage[0] = battery_calc_cell_v(analogRead(cell_pins[0]), 0);
    for (int i = 1; i <= N_BATTERY_CELLS; i++)
    {
        sensor_data.battery_cell_voltage[i - 1] = bms.get_voltages_cell(i);
    }

    sensor_data.battery_voltage = bms.get_batterie_voltage();
    sensor_data.battery_percent = battery_calc_charge(sensor_data.battery_voltage) / 100;

    sensor_data.battery_temp = bms.get_temperatures();
    // sensor_data.battery_temp = thermistor_calc_temp(analogRead(bat_therm_pin));
    // sensor_data.battery_current = calc_current(analogRead(current_sensor_pin));

#ifndef USE_MICRO_ROS
// Serial3.println(ext_adc.read(thermistor_map[0]));
#endif

    // bme.performReading();
    sensor_data.ambiant_temp = bme.readTemperature();
    sensor_data.humidity = bme.readHumidity();

    sensor_data.estop_status=digitalRead(estop_status_pin);

    uint8_t fault_code;
    
    if ((status_pwr_sup_mode == 0) &&(sensor_data.battery_voltage==0))
    {
        fault_code = 0;
    }
    else
    {
        fault_code = check_estop();
    }
    
    // require a call to estop service to reset the fault
    if (fault_code)
    {
        sensor_data.estop_pwr_out = 0;
    }
    // digitalWrite(estop_pin,1);
    digitalWrite(estop_pin, (fault_code == 0) && !sensor_data.estop_pwr_out);
}

uint8_t check_estop()
{
    if (sensor_data.battery_voltage < BATT_MIN_V)
    {
        return (FAULT_BATT_UNDER_V);
    }

    if (sensor_data.battery_voltage > BATT_MAX_V)
    {
        return (FAULT_BATT_OVER_V);
    }

    for (int i = 0; i < N_BATTERY_CELLS; i++)
    {
        if (sensor_data.battery_cell_voltage[i] < BATT_CELL_MIN_V)
        {
            return (FAULT_BATT_CELL_UNDER_V);
        }
    }

    for (int i = 0; i < N_BATTERY_CELLS; i++)
    {
        if (sensor_data.battery_cell_voltage[i] > BATT_CELL_MAX_V)
        {
            return (FAULT_BATT_CELL_OVER_V);
        }
    }

    if (sensor_data.battery_temp > BATT_MAX_TEMP)
    {
        return (FAULT_BATT_OVER_TEMP);
    }

    /*
    for(int i = 0; i < N_THERMISTORS; i++){
        if(sensor_data.thermistors[i] > MOTOR_MAX_TEMP) {
            return(FAULT_MOTOR_OVERTEMP);
        }
    }

    if(sensor_data.ambiant_temp > AMBIANT_MAX_TEMP){
        return(FAULT_AMBIANT_OVERTEMP);
    }

    if(sensor_data.humidity > AMBIANT_MAX_HUM){
        return(FAULT_AMBIANT_HUMIDITY);
    }*/

    return FAULT_NO_FAULT;
}

float thermistor_calc_temp(int adc_reading)
{
    // Find thermistor resistance from rdivider equation
    float resistance;
    resistance = THERM_PULL_UP / ((ADC_MAX_VALUE / adc_reading) - 1);

    // Find thermistor temperature from steinart equation
    float temp;
    temp = (THERM_B * THERM_T0) / (THERM_B + THERM_T0 * log(resistance / THERM_R0));
    // Degree K to degree C
    temp -= 273.15;

    return (temp);
}

double battery_calc_charge(double voltage)
{
    // calibration table meant for a 12s battery
    int capacities[] = {100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50, 45, 40, 35, 30, 25, 20, 15, 10, 5, 0};
    double voltages[] = {50.4, 49.8, 49.32, 48.96, 48.24, 47.76, 47.4, 46.92, 46.44, 46.2, 46.08, 45.84, 45.6, 45.48, 45.24, 45, 44.76, 44.52, 44.28, 43.32, 39.24};

    // Clip voltage within the range
    if (voltage > voltages[0])
    {
        voltage = voltages[0];
    }
    else if (voltage < voltages[20])
    {
        voltage = voltages[20];
    }

    // Linear interpolation
    int i;
    for (i = 0; i < 20; ++i)
    {
        if (voltage >= voltages[i])
        {
            break;
        }
    }

    // Calculate charge percentage using linear interpolation
    double charge = capacities[i] + (capacities[i + 1] - capacities[i]) * (voltage - voltages[i]) / (voltages[i + 1] - voltages[i]);

    return charge;
}

float calc_current(uint16_t adc_reading)
{
    // 6.6mv/A (+- 200A) centered at vcc/2
    float return_value = (adc_reading - (ADC_MAX_VALUE / 2)) * ADC_TO_CURRENT;
    return (return_value);
}

float lowpass_filter(float previous, float input, float tau, float dt)
{
    float alpha = dt / (tau + dt);
    float output = alpha * input + (1 - alpha) * previous;
    return output;
}