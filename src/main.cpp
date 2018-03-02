#include <Arduino.h>
#include <TMC2208Stepper.h>
#include <SoftwareSerial.h>
#include <ESP8266WebServer.h>
#include "conf.h"

uint16_t min_current[5] = {0,0,0,0,0};
uint16_t max_current[5] = {0,0,0,0,0};
uint16_t act_current[5] = {0,0,0,0,0};
uint32_t flag_drv[5]    = {0,0,0,0,0};

SoftwareSerial tmc_sw[5] = {
  SoftwareSerial(TMC_1_RX_PIN, TMC_1_TX_PIN, false, 256),
  SoftwareSerial(TMC_2_RX_PIN, TMC_2_TX_PIN, false, 256),
  SoftwareSerial(TMC_3_RX_PIN, TMC_3_TX_PIN, false, 256),
  SoftwareSerial(TMC_4_RX_PIN, TMC_4_TX_PIN, false, 256),
  SoftwareSerial(TMC_5_RX_PIN, TMC_5_TX_PIN, false, 256)
};

TMC2208Stepper driver[5] = {
  TMC2208Stepper(&tmc_sw[0]),
  TMC2208Stepper(&tmc_sw[1]),
  TMC2208Stepper(&tmc_sw[2]),
  TMC2208Stepper(&tmc_sw[3]),
  TMC2208Stepper(&tmc_sw[4])
};

// Set web server port number to 80
ESP8266WebServer server(80);

/**
    | driver | microsteps default | spreadCycle default | spreadCycle actual | hold current | R Sense |

    | driver | current default | current actual | Temperature threshold | over temp | short to gnd | open load |
*/
String getPage() {
  String page = "<html lang=fr-FR><head><meta http-equiv='refresh' content='10'/>";
  page += "<title>TMC2208 Pilot Manager</title>";
  page += "<style> body { background-color: #fffff; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>";
  page += "</head><body><h1>TMC2208 Monitoring</h1>";
  page += "<style> body { background-color: #fffff; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style>";
  page += "<h3>Parameters</h3>";
  page += "<table border=1>";
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
  page += "<table border=1>";
  page += "<tr><th>driver</th> <th>spreadCycle</th></tr>";
  for (size_t i = 0; i < 5; i++) {
      page += "<tr><td>";
      page += i + 1;
      page += "</td><td>";
      page += tmc_sw[i]?"true":"false";
      page += "</td></tr>";
  }
  page += "</table>";
  page += "<br><br><p><a hrf='https://www.google.com'>www.tmcpilot.fr</p>";
  page += "</body></html>";
  return page;
}

void handleSubmit() {
  // Actualise le GPIO / Update GPIO
  String LEDValue;
  LEDValue = server.arg("stepper");
  Serial.println("Set GPIO "); Serial.print(LEDValue);
  if ( LEDValue == "1" ) {
    //digitalWrite(LEDPIN, 1);
    //etatLed = "On";
    server.send ( 200, "text/html", getPage() );
  } else if ( LEDValue == "0" ) {
    //digitalWrite(LEDPIN, 0);
    //etatLed = "Off";
    server.send ( 200, "text/html", getPage() );
  } else {
    Serial.println("Err Led Value");
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

  for (size_t i = 0; i < 4; i++) {

    // Initiate the SoftwareSerial
    tmc_sw[i].begin(57600);                             // Init used serial port
    while(!tmc_sw[i]);                                  // Wait for port to be ready

    // Setup de driver
    driver[i].pdn_disable(1);													  // Use PDN/UART pin for communication
    driver[i].I_scale_analog(0);												// Adjust current from the registers
    driver[i].rms_current(defaults_amps[i],
                          defaults_hold_amps[i],
                          defaults_r_sense[i]);					// Set driver current, multiplier for hold current and RSENSE
    driver[i].microsteps(defaults_microsteps[i]);       // Set the defaults_microsteps
    driver[i].en_spreadCycle(defaults_en_spreadCycle[i]); // Set the spreadCycle
    driver[i].toff(0x2);																// Enable driver
  }

  // Connect to Wi-Fi network with SSID and password
  Serial.begin(115200);
  Serial.print("Connecting to ");
  Serial.println(SSID_NAME);
  WiFi.begin(SSID_NAME, SSID_PASWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // On branche la fonction qui gère la premiere page / link to the function that manage launch page
  server.on ( "/", handleRoot );

  server.begin();
  Serial.println ( "HTTP server started" );

}

void loop() {
  server.handleClient();

  for (size_t i = 0; i < 5; i++) {
      // read the actual amps for the driver
      uint16_t amp_a = driver[i].cur_a();
      uint16_t amp_b = driver[i].cur_b();

      driver[i].DRV_STATUS(&flag_drv[i]);

      uint16_t amp_tot = abs(amp_a) + abs(amp_b);
      min_current[i]=(amp_tot < min_current[i])?amp_tot:min_current[i];
      max_current[i]=(amp_tot > min_current[i])?amp_tot:max_current[i];
      act_current[i]=amp_tot;
  }

}
