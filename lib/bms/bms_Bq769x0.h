/**
 * @file bms_Bq769x0.h
 * @brief Battery Management System (BMS) wrapper class for BQ769x0 chip
 * @author Philippe Desbiens
 * @date June 5, 2025
 * @version 1.0
 */

#ifndef BMS_BQ769X0_H
#define BMS_BQ769X0_H

#include <bq769x0.h>
#include <Wire.h>

/**
 * @def b2I2CAddress
 * @brief I2C address for the BQ769x0 chip
 */
#define b2I2CAddress 0x08

/**
 * @class bms_bq769x0
 * @brief Wrapper class for BQ769x0 Battery Management System
 * 
 * This class provides a simplified interface for the BQ769x0 BMS chip,
 * inheriting privately from the bq769x0 library to encapsulate functionality
 * and provide a clean API for battery monitoring and protection.
 */
class bms_bq769x0 : private bq769x0 {
    public:
        /**
         * @brief Constructor for bms_bq769x0
         * @param theWire Pointer to TwoWire (I2C) interface
         */
        bms_bq769x0(TwoWire *theWire);
        
        /**
         * @brief Destructor for bms_bq769x0
         */
        ~bms_bq769x0();
        
        /**
         * @brief Initialize the BQ769x0 BMS with protection parameters
         * @param alert_pin GPIO pin for alert signal
         * @param boot_pin GPIO pin for boot signal
         * @param beta_therm_batterie Beta value for battery thermistor
         * @param min_temp Minimum temperature threshold (°C)
         * @param max_temp Maximum temperature threshold (°C)
         * @param volt_under Cell undervoltage protection threshold (mV)
         * @param volt_over Cell overvoltage protection threshold (mV)
         */
        void init_bq769x0(uint8_t alert_pin, uint8_t boot_pin, int16_t beta_therm_batterie, int16_t min_temp, 
                        int16_t max_temp, int16_t volt_under, int16_t volt_over);
        
        /**
         * @brief Update BMS readings and status
         * 
         * This method should be called regularly to refresh sensor readings
         * and update the BMS status.
         */
        void update();

        int check_status();
        
        /**
         * @brief Get voltage of a specific cell
         * @param cell_id Cell identifier (0-based index)
         * @return Cell voltage in volts
         */
        float get_voltages_cell(uint8_t cell_id);
        
        /**
         * @brief Get total battery pack voltage
         * @return Battery pack voltage in volts
         */
        float get_batterie_voltage();
        
        /**
         * @brief Get battery temperature
         * @return Temperature in degrees Celsius
         */
        float get_temperatures();
        
        /**
         * @brief Shutdown the BMS
         * 
         * Puts the BMS into shutdown mode to conserve power.
         */
        void shutdown();
        
    private: 
        /**
         * @brief Instance of the BQ769x0 library
         */
        bq769x0 BMS;
};

#endif