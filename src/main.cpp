#include "main.h"

void applySettings() {

  // before call driver, wait they are not busy
  while (DriversBusy) {}

  DriversBusy = true;
  for (int i = 0; i <= 4; i++) {
    //if driver disables, go to next
    if (!use_tmc[i]) continue;

    // Initiate the SoftwareSerial
    TMC2208Stepper *tmc = driver[i];

    // Setup de driver
    tmc->toff(0x0);																// Disable driver
    tmc->pdn_disable(1);													  // Use PDN/UART pin for communication
    tmc->I_scale_analog(0);												  // Adjust current from the registers
    tmc->rms_current( defaults_amps[i],
                      defaults_hold_amps[i],
                      defaults_r_sense[i] );					// Set driver current, multiplier for hold current and RSENSE
    tmc->microsteps(defaults_microsteps[i]);       // Set the defaults_microsteps
    tmc->en_spreadCycle(defaults_en_spreadCycle[i]); // Set the spreadCycle
    tmc->mstep_reg_select(true);
    tmc->toff(defaults_toff[i]);										// Enable driver or setup the spreadCycle value
  }

  DriversBusy = false;
}

uint16_t getMicrostep (uint8_t mres){
	switch(mres) {
		case 0: return 256;
		case 1: return 128;
		case 2: return  64;
		case 3: return  32;
		case 4: return  16;
		case 5: return   8;
		case 6: return   4;
		case 7: return   2;
		case 8: return   0;
	}
	return 0;
}

String getTemperatureThreshold (int driverNumber) {
  if (GETSTATUS(reg_drv_status[driverNumber],T157)==1) {
    return "<font color='red'> T157 &deg;C</font>";
  }
  else if (GETSTATUS(reg_drv_status[driverNumber],T150)==1) {
    return "<font color='orange'> =T150 &deg;C</font>";
  }
  else if (GETSTATUS(reg_drv_status[driverNumber],T143)==1) {
    return "<font color='purple'> T143 &deg;C</font>";
  }
  else if (GETSTATUS(reg_drv_status[driverNumber],T120)==1) {
    return "<font color='blue'> T120 &deg;C</font>";
  }
  else {
    return "<font color='green'>OK</font>";
  }
}

String getTemperatureOver (int driverNumber) {
  if (GETSTATUS(reg_drv_status[driverNumber],OT)) {
    return "<font color='red'>ERROR</font>";
  }
  else if (GETSTATUS(reg_drv_status[driverNumber],OTPW)) {
    return "<font color='orange'>WARN</font>";
  }
  else {
    return "<font color='green'>OK</font>";
  }
}

String getHeader() {
    String page = "<html lang=fr-FR><head><meta http-equiv='refresh' content='";
    page += timeToRefresh;
    page += "'/>";
    page += "<title>TMC2208 Pilot Manager</title>";
    page += "<style> body { background-color: #fffff; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }";
    page += "table { width: 100%; border-collapse : collapse; }";
    page += "th { background-color: #000088; color : white; }";
    page += "th, td { padding: 5px; text-align: center; vertical-align: middle; border-bottom: 1px solid #ddd; }";
    page += ".submit { background-color: #4CAF50; border: none; color: white; padding: 5px 15px; text-align: center; text-decoration: none; display: inline-block; font-size: 16px; cursor: pointer;}";
    page += "</style></head>";
    page += "<body><h1>TMC2208 Monitoring";
    page += RELEASE;
    page += "</H1>";
    return page;
}

String getPageInfo(String info) {
  String page = getHeader();
  page += "<H3>";
  page += info;
  page += "</H3>";
  page += "<form action='/' method='POST'>";
  page += "<INPUT type='submit' value='Back' class='submit' >";
  page += "</form>";
  page += "</body></html>";
  return page;
}

/**
    | driver | microsteps default | spreadCycle default | spreadCycle actual | hold current | R Sense |

    | driver | current default | current actual | Temperature threshold | over temp | short to gnd | open load |
*/
String getPage() {
  String page = getHeader();
  page += "<h3>Parameters</h3>";
  page += "<form action='/' method='POST'>";
  page += "<INPUT type='hidden' name='reapplysettings' value='true'>";
  page += "<INPUT type='submit' value='Reapply settings now !' class='submit'>";
  page += "</form>";
  page += "<table>";
  page += "<tr><th>driver</th><th>microsteps</th><th>current</th><th>hold current</th><th>R Sense</th><th>spreadCycle</th></tr>";
  for (size_t i = 0; i < 5; i++) {
      //if driver disables, go to next
      if (!use_tmc[i]) continue;

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
  page += "<h3>Monitoring actual values</h3>";
  page += "<table>";
  page += "<tr><th>driver</th><th>spreadCycle</th><th>microsteps</th><th>current</th><th>temperature<br>threshold</th><th>over<br>temperature</th><th>short to GND</th><th>open load</th></tr>";
  for (size_t i = 0; i < 5; i++) {
      //if driver disables, go to next
      if (!use_tmc[i]) continue;

      page += "<tr><td>";
      page += i + 1;
      page += "</td><td>";
      page += GETSTATUS(reg_drv_status[i],STEALTH)==0?"true":"false";
      page += "</td><td>";
      page += getMicrostep(GETSTATUS(reg_chop_conf[i],MRES));
      page += "</td><td>";
      page += act_current[i];
      page += " mA<BR>";
      page += min_current[i];
      page += " mA / ";
      page += max_current[i];
      page += " mA</td><td>";
      page += getTemperatureThreshold(i);
      page += "</td><td>";
      page += getTemperatureOver(i);
      page += "</td><td>";
      page += (GETSTATUS(reg_drv_status[i],S2GA) + GETSTATUS(reg_drv_status[i],S2GB) + GETSTATUS(reg_drv_status[i],S2VSA) + GETSTATUS(reg_drv_status[i],S2VSB))>0?"true":"false";
      page += "</td><td>";
      page += (GETSTATUS(reg_drv_status[i],OLA) + GETSTATUS(reg_drv_status[i],OLB))>0?"true":"false";
      page += "</td></tr>";
  }
  page += "</table>";
  page += "</body></html>";
  return page;
}

void handleSubmit() {
  // Actualise le GPIO / Update GPIO
  String StepperValue;
  StepperValue = server.arg("stepper");
  //Serial.println("Set GPIO "); //Serial.print(LEDValue);
  if ( StepperValue == "update" ) {
    //TODO : add the update value.
    server.send ( 200, "text/html", getPage() );
  } else if ( StepperValue == "0" ) {
    //TODO : setup
    server.send ( 200, "text/html", getPage() );
  } else {
    server.send ( 404, "text/html", "" );
  }
}

void handleRoot(){
  if ( server.hasArg("reapplysettings") ) {
    applySettings();
    server.send ( 200, "text/html", getPageInfo("Settings reapplied !") );
  } else {
    server.send ( 200, "text/html", getPage() );
  }
}

void setup() {

  // Connect to Wi-Fi network with SSID and password
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));   // subnet FF FF FF 00

  /* You can remove the password parameter if you want the AP to be open. */
  bool startAP = WiFi.softAP(ssid, password);

  IPAddress myIP = WiFi.softAPIP();

  // link to the function that manage launch page
  server.on ( "/", handleRoot );

  server.begin();

  delay(100); // wait to reduce uart reuse pin error

  if (use_tmc[1]) tmc_sw[1] = new SoftwareSerial(TMC_2_RX_PIN, TMC_2_TX_PIN, false, 64);
  if (use_tmc[2]) tmc_sw[2] = new SoftwareSerial(TMC_3_RX_PIN, TMC_3_TX_PIN, false, 64);
  if (use_tmc[3]) tmc_sw[3] = new SoftwareSerial(TMC_4_RX_PIN, TMC_4_TX_PIN, false, 64);
  if (use_tmc[4]) tmc_sw[4] = new SoftwareSerial(TMC_5_RX_PIN, TMC_5_TX_PIN, false, 64);

  // Init the driver
  for (int i = 0; i <= 4; i++) {

    //if driver disables, go to next
    if (!use_tmc[i]) continue;

    // Initiate the SoftwareSerial
    TMC2208Stepper *tmc;
    if (i==0) {
      Serial.begin(57600);
      tmc = new TMC2208Stepper(&Serial);
    } else {
      tmc_sw[i]->begin(57600);                             // Init used serial port
      while(!tmc_sw[i]);                                  // Wait for port to be ready
      tmc = new TMC2208Stepper(tmc_sw[i]);
    }

    driver[i] = tmc ;
  }

  // at startup wait driver are online to setup them
  delay(startup_wait_before_init_driver);

  // setup the driver
  applySettings();

}

void loop() {
  server.handleClient();

  if (!DriversBusy) { // if nothing talk with driver, we can read them
    DriversBusy = true;
    for (int i = 0; i <= 4; i++) {

        //if driver disables, go to next
        if (!use_tmc[i]) continue;

        // read the registers for the driver
        uint32_t tempValue;
        TMC2208Stepper *tmc = driver[i];

        tmc->DRV_STATUS(&tempValue);
        if (tempValue !=0 ) reg_drv_status[i] = tempValue;
        tmc->CHOPCONF(&tempValue);
        if (tempValue !=0 ) reg_chop_conf[i] = tempValue;
        tmc->MSCURACT(&tempValue);
        if (tempValue !=0 ) {
          reg_ms_cur_act[i] = tempValue;

          uint16_t amp_a = GETSTATUS(reg_ms_cur_act[i],CUR_A);
          uint16_t amp_b = GETSTATUS(reg_ms_cur_act[i],CUR_B);

          uint16_t amp_tot = abs(amp_a) + abs(amp_b);

          min_current[i] = (amp_tot < min_current[i])?amp_tot:min_current[i];
          max_current[i] = (amp_tot > max_current[i])?amp_tot:max_current[i];
          act_current[i] = amp_tot;
        }
    }
    DriversBusy = false;
  }

  delay(100);

}
