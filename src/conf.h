#include <Arduino.h>
#include <ESP8266WiFi.h>

#define TMC_1 true
#define TMC_2 true
#define TMC_3 true
#define TMC_4 true
#define TMC_5 true

// Set the default current amps, max amp is 1700mA (1.7A)
const float defaults_amps[] = {1000, 1000, 1000, 1000, 1000};

// Set the default microsteps, (2, 4, 8, 16, 32, 64, 128, or 256)
const int defaults_microsteps[] = {32, 32, 32, 32, 32};

// Enable the spreadCycle on driver
const bool defaults_en_spreadCycle[] = {false, false, false, false, false};

// Set the default hold current multiplier (0.5 by default)
const float defaults_hold_amps[] = {0.5, 0.5, 0.5, 0.5, 0.5};

// Set the default resistance (0.11 by default)
const float defaults_r_sense[] = {0.11, 0.11, 0.11, 0.11, 0.11};

// Set the value for your Wifi network
IPAddress    apIP(192, 168, 10, 1);         // Defining a static IP address: AP mode is 192.168.4.1
const char *ssid      = "TMC2208Pilot-AP";  // Acces Point Name
const char *password  = "TMC2208PWD";         // Acces Point password (min 8 char, if wrong, start is Failed)

/****************************************************************************
                    Do not update under this line :-)
****************************************************************************/
#define TMC_1_RX_PIN 3
#define TMC_1_TX_PIN 1

#define TMC_2_RX_PIN 13
#define TMC_2_TX_PIN 15

#define TMC_3_RX_PIN 14
#define TMC_3_TX_PIN 12

#define TMC_4_RX_PIN 0
#define TMC_4_TX_PIN 2

#define TMC_5_RX_PIN 5
#define TMC_5_TX_PIN 4
