#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TMC2208Stepper.h>
#include <SerialCommand.h>

const char *RELEASE      = "0.5.1";

// based on the TMC2208Stepper_MACRO.h, return a flag from a read status
#define GETSTATUS(VAR, SETTING) ((VAR&SETTING##_bm)	>>SETTING##_bp)

// Local variable used in standard mode
uint16_t min_current[5]     = {9999,9999,9999,9999,9999};
uint16_t max_current[5]     = {0,0,0,0,0};
uint16_t act_current[5]     = {0,0,0,0,0};
uint32_t reg_drv_status[5]  = {0,0,0,0,0};
uint32_t reg_ms_cur_act[5]  = {0,0,0,0,0};
uint32_t reg_chop_conf[5]   = {0,0,0,0,0};
bool     conf_checked[5]    = {false, false, false, false, false};

// Variable used for TMC communication with Driver
SoftwareSerial *tmc_sw[5];
TMC2208Stepper *driver[5];
bool DriversBusy = false;
bool isConfigOK;

// Variable used for Monitoring and serial
SerialCommand sCmd;               // The demo SerialCommand object
bool startedMonitoring = false;
unsigned long lastTime = 0;       // used for timer

// Variable used for i2C communication
uint8_t i2c_channel_req;				// driver to requested
uint8_t i2c_command_req;        // action to do on the driver
unsigned int i2c_value_req;   // value requested int byte 0 & 1
