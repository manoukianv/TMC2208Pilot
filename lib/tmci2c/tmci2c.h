#ifndef TMCI2C_H
#define TMCI2C_H

#include <Arduino.h>

class TmcI2C {

public:
    TmcI2C() {}
    void decode(uint8_t *buff, int length, uint8_t *channel, uint8_t *command, unsigned int *value);
    void encode(uint8_t *buffer, int length, uint8_t channel, uint8_t command, unsigned int value);
};

#endif
