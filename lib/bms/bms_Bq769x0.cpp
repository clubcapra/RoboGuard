
// bms_Bq769x0.cpp
// Implementation file for the Bq769x0 Battery Management System (BMS) class.

#include <Arduino.h>

#include "bms_Bq769x0.h"

// Constructor
bms_bq769x0::bms_bq769x0(TwoWire *theWire): BMS(theWire, bq76940, b2I2CAddress) {
    // Initialization code
}

// Destructor
bms_bq769x0::~bms_bq769x0() {
    // Cleanup code
}

// Example method implementation
void bms_bq769x0::init_bq769x0(uint8_t alert_pin, uint8_t boot_pin, int16_t beta_therm_batterie, 
        int16_t min_temp, int16_t max_temp, int16_t volt_under, int16_t volt_over) 
{
    // Code to initialize the BMS
    BMS.begin(alert_pin, boot_pin);
    
    /*
    // NOT DEFINE
    BMS.disableAutoBalancing();
    BMS.disableDischarging();
    BMS.disableCharging();
    */

    BMS.setThermistorBetaValue(beta_therm_batterie);
    BMS.setTemperatureLimits(min_temp, max_temp, 100, 100);
    BMS.setCellUndervoltageProtection(volt_under);
    BMS.setCellOvervoltageProtection(volt_over);


    BMS.setShuntResistorValue(0);
    BMS.setOvercurrentChargeProtection(100);
    BMS.setOvercurrentDischargeProtection(100);
    BMS.setShortCircuitProtection(100);

}

void bms_bq769x0::update() 
{
    // Code to initialize the BMS
    BMS.update();
    /*
    if (BMS.checkStatus() == 0) {
        // No errors
    } else {
        // Handle errors
    }*/
}

float bms_bq769x0::get_voltages_cell(uint8_t cell_id)
{
    return (float)BMS.getCellVoltage(cell_id)/1000;
}

float bms_bq769x0::get_temperatures()
{
    return BMS.getTemperatureDegC(1);
}

float bms_bq769x0::get_batterie_voltage()
{
    return (float)BMS.getBatteryVoltage()/1000;
}

