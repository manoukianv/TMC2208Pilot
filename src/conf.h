#include <Arduino.h>
#include <ESP8266WiFi.h>

// Setup the start Parameters
const int startup_wait_before_init_driver = 5000; //delay in ms, default 5000

// Define which drivers you used, see pinout to disable unused drivers
// 5 drivers max are available
const bool use_tmc[] = {true, true, true, true, false};

// Set the default current amps, max amp is 1700mA (1.7A)
const float defaults_amps[] = {1000, 1000, 1000, 1000, 1000};

// Set the default microsteps, (2, 4, 8, 16, 32, 64, 128, or 256)
const int defaults_microsteps[] = {64, 64, 64, 64, 64};

// Enable the spreadCycle on driver
const bool defaults_en_spreadCycle[] = {false, false, false, false, false};

// Set the default hold current multiplier (0.5 by default)
const float defaults_hold_amps[] = {0.5, 0.5, 0.5, 0.5, 0.5};

// Set the default resistance (0.11 by default)
const float defaults_r_sense[] = {0.11, 0.11, 0.11, 0.11, 0.11};

// Set the default TOFF (0, driver disabled, more than 2 for StealChop, beetween 2-15 for spreadCycle)
// https://hackaday.com/2016/09/30/3d-printering-trinamic-tmc2130-stepper-motor-drivers-shifting-the-gears/
const float defaults_toff[] = {8, 8, 8, 8, 8};

// Set the value for your Wifi network
IPAddress    apIP(192, 168, 10, 1);         // Defining a static IP address: AP mode is 192.168.4.1
const char *ssid      = "TMC2208Pilot-AP";  // Acces Point Name
const char *password  = "TMC2208PWD";         // Acces Point password (min 8 char, if wrong, start is Failed)

// set the Web page Parameters
const char *timeToRefresh = "1";         // refresh web page every 10s

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
