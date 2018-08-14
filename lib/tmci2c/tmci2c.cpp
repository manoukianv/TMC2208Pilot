#include <tmci2c.h>

void TmcI2C::decode(uint8_t *buffer, int length, uint8_t *channel, uint8_t *command, unsigned int *value) {
  *channel = buffer[2] && 0x0F;				// get the channel, ie driver to requested
  *command = (buffer[2] && 0xF0) >> 4; // get the command, ie action to do on the driver
  *value = (buffer[0] << 8) | buffer[1]; // extract value received int byte 0 & 1
}

void TmcI2C::encode(uint8_t *buffer, int length, uint8_t channel, uint8_t command, unsigned int value) {
  unsigned long datagram = value;

  // left shift 4 bit to add the command byte.
  datagram <<= 4;
  datagram |= command;

  // left shift 4 bit to add driver number
  datagram <<= 4;
  datagram |= channel;

  buffer[0] = (uint8_t)(datagram >> 16);
  buffer[1] = (uint8_t)(datagram >>  8);
  buffer[2] = (uint8_t)(datagram & 0xff);
}
