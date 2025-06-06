/**
 * @file interfaces.cpp
 * @brief Implementation of sensor interfaces and system monitoring for RoboGuard
 * @author Philippe Desbiens & Benoit Malenfant
 * @date June 5, 2025
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include "bms_Bq769x0.h"

#include "interfaces.h"
#include "sensor_data.h"

/** @defgroup FaultCodes System Fault Code Definitions
 *  @brief Error codes for system fault conditions
 *  @{
 */
#define FAULT_NO_FAULT 0          /**< No fault condition */
#define FAULT_BATT_UNDER_V 1      /**< Battery undervoltage */
#define FAULT_BATT_OVER_V 2       /**< Battery overvoltage */
#define FAULT_BATT_CELL_UNDER_V 3 /**< Cell undervoltage */
#define FAULT_BATT_CELL_OVER_V 4  /**< Cell overvoltage */
#define FAULT_BATT_OVER_CURR 5    /**< Battery overcurrent */
#define FAULT_BATT_OVER_TEMP 6    /**< Battery overtemperature */
#define FAULT_AMBIANT_OVERTEMP 7  /**< Ambient overtemperature */
#define FAULT_AMBIANT_HUMIDITY 8  /**< Ambient humidity fault */
/** @} */

/** @defgroup SafetyLimits System Safety Limit Definitions
 *  @brief Safety thresholds for battery and environmental monitoring
 *  @{
 */
#define BATT_MIN_V 38.4          /**< Minimum battery pack voltage (V) */
#define BATT_MAX_V 51.4          /**< Maximum battery pack voltage (V) */
#define BATT_CELL_MIN_V 3.5      /**< Minimum cell voltage (V) */
#define BATT_CELL_MAX_V 4.2      /**< Maximum cell voltage (V) */
#define BATT_MAX_TEMP 80         /**< Maximum battery temperature (°C) */
#define BATT_MIN_TEMP 10         /**< Minimum battery temperature (°C) */
#define BATT_CELL_MIN_V_BMS 3500 /**< Minimum cell voltage for BMS (mV) */
#define BATT_CELL_MAX_V_BMS 4200 /**< Maximum cell voltage for BMS (mV) */

#define MOTOR_MAX_TEMP 80   /**< Maximum motor temperature (°C) */
#define AMBIANT_MAX_TEMP 80 /**< Maximum ambient temperature (°C) */
#define AMBIANT_MAX_HUM 50  /**< Maximum ambient humidity (%) */
/** @} */

/** @defgroup I2CAddresses I2C Device Address Definitions
 *  @brief I2C addresses for connected devices
 *  @{
 */
#define BME680_ADDRESS 0x77 /**< BME680 environmental sensor address */
#define b2I2CAddress 0x08   /**< BMS I2C address */
#define ADS7828_ADDRESS 72  /**< External ADC address */
/** @} */

/** @defgroup ConversionConstants Sensor Conversion Constants
 *  @brief Constants for converting sensor readings to physical units
 *  @{
 */
#define ADC_TO_CURRENT 0.122100122100122 /**< ADC to current conversion factor */
/** @} */

/** @defgroup PinDefinitions GPIO Pin Assignments
 *  @brief Pin assignments for various system interfaces
 *  @{
 */
const int bat_therm_pin = PB0;       /**< Battery thermistor ADC pin */
const int alert_BMS = PA15;          /**< BMS alert pin */
const int boot_BMS = PC7;            /**< BMS boot pin */
const int pwr_supply_mode_pin = PA0; /**< Power supply mode selection pin */
const int estop_pin = PA12;          /**< Emergency stop output pin */
const int estop_status_pin = PA11;   /**< Emergency stop status input pin */
const int current_sensor_pin = PB1;  /**< Current sensor ADC pin */
/** @} */

/** @defgroup GlobalVariables System State Variables
 *  @brief Global variables tracking system state
 *  @{
 */
int status_pwr_sup_mode = 1;    /**< Power supply mode status */
bool status_voltage_BMS = true; /**< BMS voltage status flag */
/** @} */

/** @defgroup I2CInterfaces I2C Bus Instances
 *  @brief I2C interface instances for different sensors
 *  @{
 */
TwoWire wire1(PB7, PB6);   /**< I2C bus 1 for environmental sensor */
TwoWire wire2(PC12, PB10); /**< I2C bus 2 for BMS */
/** @} */

/** @defgroup SensorInstances Sensor Object Instances
 *  @brief Instances of sensor objects
 *  @{
 */
Adafruit_BME680 bme(&wire1); /**< Environmental sensor instance */
bms_bq769x0 bms(&wire2);     /**< Battery management system instance */
/** @} */

void setup_interfaces()
{
    // Configure GPIO pins
    pinMode(estop_pin, OUTPUT);
    pinMode(estop_status_pin, INPUT);
    pinMode(pwr_supply_mode_pin, INPUT);

    // Initialize BMS with protection parameters
    bms.init_bq769x0(alert_BMS, boot_BMS, THERM_B, BATT_MIN_TEMP, BATT_MAX_TEMP, BATT_CELL_MIN_V_BMS, BATT_CELL_MAX_V_BMS);

    // Initialize environmental sensor
    bme.begin(BME680_ADDRESS);
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(0, 150); // 320°C for 150 ms

    // Configure ADC
    analogReadResolution(ADC_N_BITS);
    pinMode(bat_therm_pin, INPUT_ANALOG);
    pinMode(current_sensor_pin, INPUT_ANALOG);
    bms.update();
    if (bms.get_batterie_voltage() == 0)
    {
        status_voltage_BMS = false;
    }

}

void update_interfaces()
{
    bms.update();
    

    // Read individual cell voltages (BMS channels start at 1)
    for (int i = 1; i <= N_BATTERY_CELLS; i++)
    {
        sensor_data.battery_cell_voltage[i - 1] = bms.get_voltages_cell(i);
    }

    sensor_data.battery_voltage = bms.get_batterie_voltage();
    sensor_data.battery_percent = battery_calc_charge(sensor_data.battery_voltage) / 100;
    sensor_data.battery_temp = bms.get_temperatures();

    // Check BMS voltage status
    // INUTILE A CORRIGER NE LIT PAS CAR PAS INIT
    if ((sensor_data.battery_voltage > 0) && (!status_voltage_BMS))
    {
        status_voltage_BMS = true;
        bms.init_bq769x0(alert_BMS, boot_BMS, THERM_B, BATT_MIN_TEMP, BATT_MAX_TEMP, BATT_CELL_MIN_V_BMS, BATT_CELL_MAX_V_BMS);
    }

    // Handle power supply mode changes and BMS state
    // A VERIFIER
    if ((status_pwr_sup_mode != digitalRead(pwr_supply_mode_pin) && (!status_voltage_BMS)) || (!status_voltage_BMS))
    {
        status_pwr_sup_mode = digitalRead(pwr_supply_mode_pin);
        if(status_pwr_sup_mode==0 || (!status_voltage_BMS))
        {
            bms.shutdown();
        }
        else
        {
            bms.init_bq769x0(alert_BMS, boot_BMS, THERM_B, BATT_MIN_TEMP, BATT_MAX_TEMP, BATT_CELL_MIN_V_BMS, BATT_CELL_MAX_V_BMS);
        }
        
    }

    // Read environmental data
    sensor_data.ambiant_temp = bme.readTemperature();
    sensor_data.humidity = bme.readHumidity();

    // Read emergency stop button status
    sensor_data.estop_status_boutons = digitalRead(estop_status_pin);

    // Check for fault conditions
    uint8_t fault_code;

    if ((status_pwr_sup_mode == 0) && (!status_voltage_BMS))
    {
        fault_code = 0;
    }
    else
    {
        fault_code = check_estop();
    }

    // Require a call to estop service to reset the fault
    if (fault_code)
    {
        sensor_data.estop_pwr_out = 0;
    }

    // Update system emergency stop status
    sensor_data.estop_status_stm32 = (fault_code == 0) && !sensor_data.estop_pwr_out;
    digitalWrite(estop_pin, sensor_data.estop_status_stm32);
}

uint8_t check_estop()
{
    // Check battery pack voltage limits
    if (sensor_data.battery_voltage < BATT_MIN_V)
    {
        return (FAULT_BATT_UNDER_V);
    }

    if (sensor_data.battery_voltage > BATT_MAX_V)
    {
        return (FAULT_BATT_OVER_V);
    }

    // Check individual cell voltages
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

    // Check battery temperature
    if (sensor_data.battery_temp > BATT_MAX_TEMP)
    {
        return (FAULT_BATT_OVER_TEMP);
    }

    // Check environmental conditions
    if (sensor_data.ambiant_temp > AMBIANT_MAX_TEMP)
    {
        return (FAULT_AMBIANT_OVERTEMP);
    }

    if (sensor_data.humidity > AMBIANT_MAX_HUM)
    {
        return (FAULT_AMBIANT_HUMIDITY);
    }

    return FAULT_NO_FAULT;
}

float thermistor_calc_temp(int adc_reading)
{
    // Calculate thermistor resistance from voltage divider
    float resistance;
    resistance = THERM_PULL_UP / ((ADC_MAX_VALUE / adc_reading) - 1);

    // Apply Steinhart-Hart equation for temperature
    float temp;
    temp = (THERM_B * THERM_T0) / (THERM_B + THERM_T0 * log(resistance / THERM_R0));
    temp -= 273.15; // Convert from Kelvin to Celsius

    return (temp);
}

double battery_calc_charge(double voltage)
{
    // Calibration table for 12S battery pack
    int capacities[] = {100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50, 45, 40, 35, 30, 25, 20, 15, 10, 5, 0};
    double voltages[] = {50.4, 49.8, 49.32, 48.96, 48.24, 47.76, 47.4, 46.92, 46.44, 46.2, 46.08, 45.84, 45.6, 45.48, 45.24, 45, 44.76, 44.52, 44.28, 43.32, 39.24};

    // Clip voltage to table range
    if (voltage > voltages[0])
    {
        voltage = voltages[0];
    }
    else if (voltage < voltages[20])
    {
        voltage = voltages[20];
    }

    // Find interpolation range
    int i;
    for (i = 0; i < 20; ++i)
    {
        if (voltage >= voltages[i])
        {
            break;
        }
    }

    // Linear interpolation between table points
    double charge = capacities[i] + (capacities[i + 1] - capacities[i]) * (voltage - voltages[i]) / (voltages[i + 1] - voltages[i]);

    return charge;
}

float calc_current(uint16_t adc_reading)
{
    // 6.6mV/A (±200A) centered at VCC/2
    float return_value = (adc_reading - (ADC_MAX_VALUE / 2)) * ADC_TO_CURRENT;
    return (return_value);
}

float lowpass_filter(float previous, float input, float tau, float dt)
{
    float alpha = dt / (tau + dt);
    float output = alpha * input + (1 - alpha) * previous;
    return output;
}