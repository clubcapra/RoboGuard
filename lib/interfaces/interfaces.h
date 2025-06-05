/**
 * @file interfaces.h
 * @brief Interface definitions and sensor management for RoboGuard system
 * @author Philippe Desbiens & Benoit Malenfant
 * @date June 5, 2025
 * @version 1.0
 */

#ifndef INTERFACES_H
#define INTERFACES_H

/** @defgroup ThermistorConstants Thermistor Configuration Constants
 *  @brief Constants for thermistor temperature calculations
 *  @{
 */
#define THERM_R0 10000.0        /**< Nominal thermistor resistance at T0 (Ohms) */
#define THERM_T0 (25+273.15)    /**< Nominal thermistor temperature (Kelvin) */
#define THERM_B 3450.0          /**< Thermistor B coefficient for battery */
#define THERM_PULL_UP 10000.0   /**< Thermistor pull-up resistor value (Ohms) */
/** @} */

/** @defgroup ADCConstants ADC Configuration Constants
 *  @brief Constants for analog-to-digital converter
 *  @{
 */
#define ADC_MAX_VALUE 4095.0    /**< Maximum ADC reading value (12-bit) */
#define ADC_N_BITS 12           /**< ADC resolution in bits */
#define ADC_VCC 3.3             /**< ADC reference voltage */
/** @} */

/** @defgroup BatteryConstants Battery Monitoring Constants
 *  @brief Constants for battery cell monitoring
 *  @{
 */
#define CELL_RDIV_RATIO 16      /**< Voltage divider ratio for cell monitoring */
/** @} */

/** @defgroup FilterConstants Filter Time Constants
 *  @brief Time constants for low-pass filtering
 *  @{
 */
#define THERM_TAU 1             /**< Thermistor filter time constant */
#define CELL_TAU 1              /**< Cell voltage filter time constant */
/** @} */

/**
 * @brief Initialize all system interfaces and sensors
 * 
 * Configures GPIO pins, I2C interfaces, BMS, environmental sensor,
 * and ADC settings for the RoboGuard system.
 */
void setup_interfaces();

/**
 * @brief Update all sensor readings and system status
 * 
 * Reads battery voltages, temperatures, environmental conditions,
 * and updates emergency stop status based on fault conditions.
 */
void update_interfaces();

/**
 * @brief Check for emergency stop conditions
 * 
 * Monitors battery voltage, cell voltages, temperatures, and environmental
 * conditions to determine if an emergency stop is required.
 * 
 * @return Fault code (0 = no fault, >0 = specific fault condition)
 */
uint8_t check_estop();

/**
 * @brief Calculate temperature from thermistor ADC reading
 * 
 * Uses Steinhart-Hart equation to convert thermistor resistance
 * (derived from ADC reading) to temperature in Celsius.
 * 
 * @param adc_reading Raw ADC value from thermistor
 * @return Temperature in degrees Celsius
 */
float thermistor_calc_temp(int adc_reading);

/**
 * @brief Calculate battery cell voltage with filtering
 * 
 * Converts ADC reading to actual cell voltage accounting for
 * voltage divider and applies low-pass filtering.
 * 
 * @param cell_reading Current ADC reading
 * @param prev_cell_reading Previous filtered reading
 * @return Filtered cell voltage in volts
 */
float battery_calc_cell_v(uint16_t cell_reading, uint16_t prev_cell_reading);

/**
 * @brief Calculate battery charge percentage from voltage
 * 
 * Uses lookup table and linear interpolation to estimate
 * battery state of charge from total pack voltage.
 * 
 * @param voltage Battery pack voltage in volts
 * @return Charge percentage (0-100%)
 */
double battery_calc_charge(double voltage);

/**
 * @brief Apply low-pass filter to sensor data
 * 
 * Implements first-order low-pass filter to smooth noisy sensor readings.
 * 
 * @param previous Previous filtered value
 * @param input Current input value
 * @param tau Filter time constant
 * @param dt Sample time interval
 * @return Filtered output value
 */
float lowpass_filter(float previous, float input, float tau, float dt);

/**
 * @brief Calculate current from ADC reading
 * 
 * Converts ADC reading from current sensor to actual current value.
 * Assumes sensor output is centered at VCC/2 with 6.6mV/A sensitivity.
 * 
 * @param adc_reading Raw ADC value from current sensor
 * @return Current in amperes (positive = charge, negative = discharge)
 */
float calc_current(uint16_t adc_reading);

#endif