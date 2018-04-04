#include "main.h"

void applySettings() {
  Serial.println("applySettings");

  // before call driver, wait they are not busy
  while (DriversBusy) {}

  DriversBusy = true;
  for (int i = 0; i <= 4; i++) {
    //if driver disables, go to next
    if (!use_tmc[i]) continue;

    // enable serial RX before call
    tmc_sw[i]->listen();

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

    tmc->rms_current()
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
    return "157";
  }
  else if (GETSTATUS(reg_drv_status[driverNumber],T150)==1) {
    return "150";
  }
  else if (GETSTATUS(reg_drv_status[driverNumber],T143)==1) {
    return "143";
  }
  else if (GETSTATUS(reg_drv_status[driverNumber],T120)==1) {
    return "120";
  }
  else {
    return "OK";
  }
}

String getTemperatureOver (int driverNumber) {
  if (GETSTATUS(reg_drv_status[driverNumber],OT)) {
    return "ERROR";
  }
  else if (GETSTATUS(reg_drv_status[driverNumber],OTPW)) {
    return "WARN";
  }
  else {
    return "OK";
  }
}

void getConfig() {
  String spacer = " \t ";
  Serial.println("driver \t microsteps \t current \t hold current \t R Sense \t spreadCycle");
  for (size_t i = 0; i < 5; i++) {
    //if driver disables, go to next
    if (!use_tmc[i]) continue;

    Serial.print(i + 1);
    Serial.print(spacer);
    Serial.print(defaults_microsteps[i]);
    Serial.print(spacer);
    Serial.print(spacer);
    Serial.print(defaults_amps[i]);
    Serial.print(spacer);
    Serial.print(defaults_hold_amps[i] * defaults_amps[i]);
    Serial.print(spacer);
    Serial.print(defaults_r_sense[i]);
    Serial.print(spacer);
    Serial.print(spacer);
    Serial.print(defaults_en_spreadCycle[i]?"true":"false");
    Serial.println();
  }
}

void getMonitoring() {
  String spacer = " \t ";
  Serial.println("driver \t spreadCycle \t microsteps \t current \t T°(threshold) \t T°(over) \t short(GND) \t Openload");
  for (size_t i = 0; i < 5; i++) {
    //if driver disables, go to next
    if (!use_tmc[i]) continue;

    Serial.print(i + 1);
    Serial.print(spacer);
    Serial.print(GETSTATUS(reg_drv_status[i],STEALTH)==0?"true":"false");
    Serial.print(spacer);
    Serial.print(spacer);
    Serial.print(getMicrostep(GETSTATUS(reg_chop_conf[i],MRES)));
    Serial.print(spacer);
    Serial.print(spacer);
    Serial.print(min_current[i]);
    Serial.print("<");
    Serial.print(act_current[i]);
    Serial.print("<");
    Serial.print(max_current[i]);
    Serial.print(spacer);
    Serial.print(getTemperatureThreshold(i));
    Serial.print(spacer);
    Serial.print(spacer);
    Serial.print(getTemperatureOver(i));
    Serial.print(spacer);
    Serial.print(spacer);
    Serial.print((GETSTATUS(reg_drv_status[i],S2GA) + GETSTATUS(reg_drv_status[i],S2GB) + GETSTATUS(reg_drv_status[i],S2VSA) + GETSTATUS(reg_drv_status[i],S2VSB))>0?"true":"false");
    Serial.print(spacer);
    Serial.print(spacer);
    Serial.print((GETSTATUS(reg_drv_status[i],OLA) + GETSTATUS(reg_drv_status[i],OLB))>0?"true":"false");
    Serial.println();
  }
}

void startMon() {
  startedMonitoring = true;
  Serial.println("Started Monitoring, use 'getMon' to see data");
}

void stopMon() {
  startedMonitoring = false;
  Serial.println("Stoped Monitoring");
}

// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
  Serial.println("Commands available :");
  Serial.println("getConf, getMon, startMon, stopMon");
}

void setup() {

  Serial.begin(57600);
  sCmd.addCommand("getConf", getConfig);
  sCmd.addCommand("getMon", getMonitoring);
  sCmd.addCommand("startMon", startMon);
  sCmd.addCommand("stopMon", stopMon);
  sCmd.setDefaultHandler(unrecognized);      // Handler for command that isn't matched  (says "What?")

  Serial.print("TMC2208Pilot v");
  Serial.println(RELEASE);
  Serial.println("Startup...");

  if (use_tmc[0]) tmc_sw[0] = new SoftwareSerial(TMC_1_RX_PIN, TMC_1_TX_PIN);
  if (use_tmc[1]) tmc_sw[1] = new SoftwareSerial(TMC_2_RX_PIN, TMC_2_TX_PIN);
  if (use_tmc[2]) tmc_sw[2] = new SoftwareSerial(TMC_3_RX_PIN, TMC_3_TX_PIN);
  if (use_tmc[3]) tmc_sw[3] = new SoftwareSerial(TMC_4_RX_PIN, TMC_4_TX_PIN);
  if (use_tmc[4]) tmc_sw[4] = new SoftwareSerial(TMC_5_RX_PIN, TMC_5_TX_PIN);

  // Init the driver
  for (int i = 0; i <= 4; i++) {

    //if driver disables, go to next
    if (!use_tmc[i]) continue;

    // Initiate the SoftwareSerial
    tmc_sw[i]->begin(19200);                             // Init used serial port
    while(!tmc_sw[i]);                                  // Wait for port to be ready
    driver[i] = new TMC2208Stepper(tmc_sw[i]);

  }

  // at startup wait driver are online to setup them
  delay(startup_wait_before_init_driver);

  // setup the driver
  applySettings();

  Serial.println("Startup end !");

}

void loop() {

  sCmd.readSerial();

  if (!DriversBusy && startedMonitoring) { // if nothing talk with driver, we can read them
    DriversBusy = true;
    for (int i = 0; i <= 4; i++) {

        //if driver disables, go to next
        if (!use_tmc[i]) continue;

        // enable reception on this serial, else no read value
        tmc_sw[i]->listen();

        // read the registers for the driver
        uint32_t tempValue;
        TMC2208Stepper *tmc = driver[i];

        delay(50);
        tmc->DRV_STATUS(&tempValue);
        if (tempValue !=0 ) reg_drv_status[i] = tempValue;
        delay(50);
        tmc->CHOPCONF(&tempValue);
        if (tempValue !=0 ) reg_chop_conf[i] = tempValue;
        delay(50);
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

}
