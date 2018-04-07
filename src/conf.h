#include <Arduino.h>

// Setup the start Parameters
const unsigned long DELAY_BEFORE_STARTUP_CONF = 5000;  //delay in ms, default 5000
const unsigned long DELAY_BEETWEEN_CHECK_CONF = 3000;  // delay beetween 2 checkConfig requested by pin
const bool          AUTO_CONF_ON_ERROR        = true;  //restart conf automaticaly if checkIsFailed

// Define which drivers you used, see pinout to disable unused drivers
// 5 drivers max are available
const bool use_tmc[] = {true, true, true, true, false};

// Set the default current amps, max amp is 1700mA (1.7A)
const float defaults_amps[] = {1000, 1000, 1000, 1000, 1000};

// Set the default microsteps, (2, 4, 8, 16, 32, 64, 128, or 256)
const uint16_t defaults_microsteps[]        = {64, 64, 32, 64, 64};
const bool     defaults_256_step_interpol[] = {false, false, false, false, false};

// Enable the spreadCycle on driver
const bool defaults_en_spreadCycle[] = {false, false, false, false, false};

// Set the default TOFF (0, driver disabled, more than 2 for StealChop, beetween 2-15 for spreadCycle)
// https://hackaday.com/2016/09/30/3d-printering-trinamic-tmc2130-stepper-motor-drivers-shifting-the-gears/
const float defaults_toff[] = {8, 8, 8, 8, 8};

/****************************************************************************
                    Do not update under this line :-)
****************************************************************************/
#define TMC_1_RX_PIN 3
#define TMC_1_TX_PIN 2

#define TMC_2_RX_PIN 5
#define TMC_2_TX_PIN 4

#define TMC_3_RX_PIN 7
#define TMC_3_TX_PIN 6

#define TMC_4_RX_PIN 9
#define TMC_4_TX_PIN 8

#define TMC_5_RX_PIN 10
#define TMC_5_TX_PIN 11

#define ERROR_PIN    13

#define CHECK_PIN    14
