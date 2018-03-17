#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

#include <SoftwareSerial.h>
#include <TMC2208Stepper.h>
#include <TMC2208Stepper_REGDEFS.h>

#include "conf.h"

const char *version      = "0.2.2";

// based on the TMC2208Stepper_MACRO.h, return a flag from a read status
#define GETSTATUS(VAR, SETTING) ((VAR&SETTING##_bm)	>>SETTING##_bp)

uint16_t min_current[5]     = {9999,9999,9999,9999,9999};
uint16_t max_current[5]     = {0,0,0,0,0};
uint16_t act_current[5]     = {0,0,0,0,0};
uint32_t reg_drv_status[5]  = {0,0,0,0,0};
uint32_t reg_ms_cur_act[5]  = {0,0,0,0,0};
uint32_t reg_chop_conf[5]   = {0,0,0,0,0};

SoftwareSerial *tmc_sw[5];

TMC2208Stepper *driver[5];

// Set web server port number to 80
ESP8266WebServer server(80);

bool DriversBusy = false;
