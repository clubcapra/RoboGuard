#ifndef BMS_BQ769X0_H
#define BMS_BQ769X0_H

#include <bq769x0.h>
#include <Wire.h>

#define b2I2CAddress 0x08


class bms_bq769x0 : private bq769x0 {
    public:
        bms_bq769x0(TwoWire *theWire);
        ~bms_bq769x0();
        void init_bq769x0(uint8_t alert_pin, uint8_t boot_pin, int16_t beta_therm_batterie, int16_t min_temp, 
                        int16_t max_temp, int16_t volt_under, int16_t volt_over);
        void update();
        float get_voltages_cell(uint8_t cell_id);
        float get_batterie_voltage();
        float get_temperatures();
        void shutdown();
    private: 
        bq769x0 BMS;

};

#endif