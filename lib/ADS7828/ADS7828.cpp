#include "ADS7828.h"
#include "stdint.h"

#define CH_OFFSET 4

#define SD_OFFSET 7
#define SINGLE_ENDED 1
#define DIFFERENTIAL 0

#define REFERENCE_OFFSET 3
#define ADC_OFFSET 2

uint16_t ADS7828::read(uint8_t channel)
{
    this->send_command(channel);
    this->wire->requestFrom(this->address, (uint8_t)2);
    return word(this->wire->read(), this->wire->read());
}

uint8_t ADS7828::send_command(uint8_t channel)
{
    uint8_t csel = (channel >> 1) | ((channel & 0x1) << 2);
    uint8_t command = ((this->mode&1) << SD_OFFSET) | ((this->ADC_PWR&1) << ADC_OFFSET) | ((this->REFERENCE_PWR&1) << REFERENCE_OFFSET) | ((csel&7) << CH_OFFSET);
    this->wire->beginTransmission(this->address);
    this->wire->write(command);
    return this->wire->endTransmission();
}


ADS7828::ADS7828(uint8_t address = 72U, TwoWire *wire = &Wire, uint8_t mode = SINGLE_ENDED, uint8_t ADC_PWR = 1, uint8_t REFERENCE_PWR = 1)
{
    this->wire = wire;
    this->address = address;
    this->mode = mode;
    this->ADC_PWR = ADC_PWR;
    this->REFERENCE_PWR = REFERENCE_PWR;
}
