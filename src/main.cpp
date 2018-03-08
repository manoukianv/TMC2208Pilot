#include <Arduino.h>
#include <TMC2208Stepper.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include "conf.h"
#include <TMC2208Stepper_REGDEFS.h>

// based on the TMC2208Stepper_MACRO.h, return a flag from a read status
#define GETSTATUS(VAR, SETTING) ((VAR&SETTING##_bm)	>>SETTING##_bp)

uint16_t min_current[5] = {0,0,0,0,0};
uint16_t max_current[5] = {0,0,0,0,0};
uint16_t act_current[5] = {0,0,0,0,0};
uint32_t flag_drv[5]    = {0,0,0,0,0};

SoftwareSerial *tmc_sw[5];

TMC2208Stepper *driver[5];

// Set web server port number to 80
ESP8266WebServer server(80);

String getTemperatureThreshold (int driverNumber) {
  if (GETSTATUS(flag_drv[driverNumber],T157)==1) {
    return "<font color='red'> T157 &deg;C</font>";
  }
  else if (GETSTATUS(flag_drv[driverNumber],T150)==1) {
    return "<font color='orange'> =T150 &deg;C</font>";
  }
  else if (GETSTATUS(flag_drv[driverNumber],T143)==1) {
    return "<font color='purple'> T143 &deg;C</font>";
  }
  else if (GETSTATUS(flag_drv[driverNumber],T120)==1) {
    return "<font color='blue'> T120 &deg;C</font>";
  }
  else {
    return "<font color='green'>OK</font>";
  }
}

String getTemperatureOver (int driverNumber) {
  if (GETSTATUS(flag_drv[driverNumber],OT)) {
    return "<font color='red'>ERROR</font>";
  }
  else if (GETSTATUS(flag_drv[driverNumber],OTPW)) {
    return "<font color='orange'>WARN</font>";
  }
  else {
    return "<font color='green'>OK</font>";
  }
}

/**
    | driver | microsteps default | spreadCycle default | spreadCycle actual | hold current | R Sense |

    | driver | current default | current actual | Temperature threshold | over temp | short to gnd | open load |
*/
String getPage() {
  String page = "<html lang=fr-FR><head><meta http-equiv='refresh' content='10'/>";
  page += "<title>TMC2208 Pilot Manager</title>";
  page += "<style> body { background-color: #fffff; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }";
  page += "table { width: 100%; border-collapse : collapse; }";
  //page += "table, th, td { border : 1px solid black; }";
  page += "th { background-color: #000088; color : white; }";
  page += "th, td { padding: 5px; text-align: center; vertical-align: middle; border-bottom: 1px solid #ddd; }";
  page += "</style></head>";
  page += "<body><h1>TMC2208 Monitoring</h1>";
  page += "<h3>Parameters</h3>";
  page += "<table>";
  page += "<tr><th>driver</th><th>microsteps</th><th>current</th><th>hold current</th><th>R Sense</th><th>spreadCycle</th></tr>";
  for (size_t i = 0; i < 5; i++) {
      page += "<tr><td>";
      page += i + 1;
      page += "</td><td>";
      page += defaults_microsteps[i];
      page += " step</td><td>";
      page += defaults_amps[i];
      page += " mA</td><td>";
      page += defaults_hold_amps[i] * defaults_amps[i];
      page += " mA</td><td>";
      page += defaults_r_sense[i];
      page += " ohm</td><td>";
      page += defaults_en_spreadCycle[i]?"true":"false";
      page += "</td></tr>";
  }
  page += "</table>";
  page += "<h3>Monitoring</h3>";
  page += "<table>";
  page += "<tr><th>driver</th><th>spreadCycle actual</th><th>current actual</th><th>temperature threshold</th><th>over temp</th><th>short to GND</th><th>open load</th></tr>";
  for (size_t i = 0; i < 5; i++) {
      page += "<tr><td>";
      page += i + 1;
      page += "</td><td>";
      page += GETSTATUS(flag_drv[i],STEALTH)==0?"true":"false";
      page += "</td><td> actual : ";
      page += act_current[i];
      page += " mA<BR> min : ";
      page += min_current[i];
      page += " mA<BR> max : ";
      page += max_current[i];
      page += " mA</td><td>";
      page += getTemperatureThreshold(i);
      page += "</td><td>";
      page += getTemperatureOver(i);
      page += "</td><td>";
      page += (GETSTATUS(flag_drv[i],S2GA) + GETSTATUS(flag_drv[i],S2GB) + GETSTATUS(flag_drv[i],S2VSA) + GETSTATUS(flag_drv[i],S2VSB))>0?"true":"false";
      page += "</td><td>";
      page += (GETSTATUS(flag_drv[i],OLA) + GETSTATUS(flag_drv[i],OLB))>0?"true":"false";
      page += "</td></tr>";
  }
  page += "</table>";
  page += "</body></html>";
  return page;
}

void handleSubmit() {
  // Actualise le GPIO / Update GPIO
  String LEDValue;
  LEDValue = server.arg("stepper");
  //Serial.println("Set GPIO "); //Serial.print(LEDValue);
  if ( LEDValue == "1" ) {
    //digitalWrite(LEDPIN, 1);
    //etatLed = "On";
    server.send ( 200, "text/html", getPage() );
  } else if ( LEDValue == "0" ) {
    //digitalWrite(LEDPIN, 0);
    //etatLed = "Off";
    server.send ( 200, "text/html", getPage() );
  } else {
    //Serial.println("Err Led Value");
  }
}

void handleRoot(){
  if ( server.hasArg("stepper") ) {
    handleSubmit();
  } else {
    server.send ( 200, "text/html", getPage() );
  }
}

void setup() {
  //Serial.begin(115200);
  //Serial.println();

  // Connect to Wi-Fi network with SSID and password
  //Serial.println("Setup the WIFI Access Point (AP)");
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));   // subnet FF FF FF 00

  /* You can remove the password parameter if you want the AP to be open. */
  //Serial.print("... AP Status : ");
  bool startAP = WiFi.softAP(ssid, password);
  //Serial.println(startAP?"Ready":"Failed!");

  //Serial.printf("...AP Name : %s\n", ssid);
  //Serial.printf("...AP password : %s\n", password);

  IPAddress myIP = WiFi.softAPIP();
  //Serial.print("...AP IPaddress: ");
  //Serial.println(myIP);

  // On branche la fonction qui gère la premiere page / link to the function that manage launch page
  server.on ( "/", handleRoot );

  server.begin();
  //Serial.println ( "HTTP server started" );

  //Serial.println ( "End of setup, ready to connect." );
  //Serial.end();

  tmc_sw[0] = new SoftwareSerial(TMC_1_RX_PIN, TMC_1_TX_PIN, false, 64);
  tmc_sw[1] = new SoftwareSerial(TMC_2_RX_PIN, TMC_2_TX_PIN, false, 64);
  tmc_sw[2] = new SoftwareSerial(TMC_3_RX_PIN, TMC_3_TX_PIN, false, 64);
  tmc_sw[3] = new SoftwareSerial(TMC_4_RX_PIN, TMC_4_TX_PIN, false, 64);
  tmc_sw[4] = new SoftwareSerial(TMC_5_RX_PIN, TMC_5_TX_PIN, false, 64);

  //Serial.println ( "Start init driver" );
  for (size_t i = 0; i < 5; i++) {

    // Initiate the SoftwareSerial
    //tmc_sw[i].begin(9600);                             // Init used serial port
    while(!tmc_sw[i]);                                  // Wait for port to be ready

    driver[i] = new TMC2208Stepper(tmc_sw[i]);
    // Setup de driver
    driver[i]->pdn_disable(1);													  // Use PDN/UART pin for communication
    driver[i]->I_scale_analog(0);												// Adjust current from the registers
    driver[i]->rms_current(defaults_amps[i],
                          defaults_hold_amps[i],
                          defaults_r_sense[i]);					// Set driver current, multiplier for hold current and RSENSE
    driver[i]->microsteps(defaults_microsteps[i]);       // Set the defaults_microsteps
    driver[i]->en_spreadCycle(defaults_en_spreadCycle[i]); // Set the spreadCycle
    driver[i]->toff(0x2);																// Enable driver
    //Serial.printf("...driver %d init\n", i);
  }

}

void loop() {
  server.handleClient();

  for (size_t i = 0; i < 5; i++) {
      // read the actual amps for the driver
      uint16_t amp_a = driver[i]->cur_a();
      uint16_t amp_b = driver[i]->cur_b();

      driver[i]->DRV_STATUS(&flag_drv[i]);

      uint16_t amp_tot = abs(amp_a) + abs(amp_b);
      min_current[i]=(amp_tot < min_current[i])?amp_tot:min_current[i];
      max_current[i]=(amp_tot > min_current[i])?amp_tot:max_current[i];
      act_current[i]=amp_tot;

      delay(200);
  }

}
