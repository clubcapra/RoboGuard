#ifndef ADS7828_H
#define ADS7828_H

#include <stdint.h>
#include <Wire.h>

#define ADS7828_SINGLE_ENDED 1
#define ADS7828_DIFFERENTIAL 0

class ADS7828{
    uint16_t read(uint8_t channel);
    uint8_t send_command(uint8_t channel);
    ADS7828(TwoWire *wire, uint8_t address, uint8_t mode, uint8_t ADC_PWR, uint8_t REFERENCE_PWR);
    
    uint8_t address = 72U;
    uint8_t mode = ADS7828_SINGLE_ENDED;
    uint8_t ADC_PWR = 1;
    uint8_t REFERENCE_PWR = 1;
    TwoWire *wire;
};

#endif