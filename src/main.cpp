#include <Arduino.h>

#include <SerialCommand.h>
#include <SoftwareSerial.h>
#include <TMC2208Stepper.h>
#include <TMC2208Stepper_REGDEFS.h>
#include <Wire.h>
#include <tmci2c.h>

#include "conf.h"

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

TmcI2C tmc2ic;

#define RESET_CONF_REG              0
#define SET_MICROSTEP_REG           1
#define SET_CURRENT_REG             2
#define SET_INTERPOL_256_REG        3
#define SET_SPREADCYCLE_EN_REG      4
#define SET_ENABLE_REG              5
#define GET_MICROSTEP_REG           6
#define GET_CURRENT_REG             7
#define GET_INTERPOL_256_REG        8
#define GET_SPREADCYCLE_EN_REG      9
#define GET_ENABLE_REG              10
#define GET_ALARM_REG               11

//*************************************************************************************
//		                               Config section
//*************************************************************************************

void applySettings(bool enable_i2c) {
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
    tmc->toff(0x0);																	// Disable driver
    tmc->pdn_disable(1);													  // Use PDN/UART pin for communication
    tmc->I_scale_analog(0);												  // Adjust current from the registers
		tmc->mstep_reg_select(true);										 // enable the microsteps settings by register

		if (!enable_i2c) {																// only use the configuration section file if i2c is not enable
	    tmc->rms_current(defaults_amps[i]); 						// Set driver current
	    tmc->microsteps(defaults_microsteps[i]);        // Set the defaults_microsteps
			tmc->intpol(defaults_256_step_interpol[i]);			// enable or disable 256 microsteps interpolation
			tmc->en_spreadCycle(defaults_en_spreadCycle[i]); // Set the spreadCycle
    }

    tmc->toff(defaults_toff[i]);										// Enable driver or setup the spreadCycle value
  }

  DriversBusy = false;
}

bool checkConfig(bool enable_i2c) {
	  Serial.println("CheckConfig :");

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

			// Init the internal information
	    conf_checked[i] =
		      tmc->pdn_disable() == 1 &&
		      tmc->I_scale_analog() == 0 &&
					tmc->mstep_reg_select() == 1 &&
		      tmc->toff() == defaults_toff[i];

			if (!enable_i2c) {
					conf_checked[i] = conf_checked[i] &&
							tmc->microsteps() == defaults_microsteps[i] &&
							tmc->intpol() == defaults_256_step_interpol[i] &&
				      tmc->en_spreadCycle() == defaults_en_spreadCycle[i];
			}

	    Serial.print(" driver ");Serial.print(i+1);Serial.print(" check control is : ");
	    Serial.println(conf_checked[i]==1?"true":"false");

	  }

	  DriversBusy = false;

		for (int i = 0; i <= 4; i++) {
			if (!use_tmc[i]) continue;
			if (!conf_checked[i]) return false;
		}

		return true;
}

bool checkConfigAndSendDiag(bool enable_i2c) {
	isConfigOK = checkConfig(enable_i2c);
	Serial.print("config is ");
	Serial.println(isConfigOK?"OK":"on ERROR");
	if (isConfigOK) {
		digitalWrite(ERROR_PIN, LOW);
	} else {
		digitalWrite(ERROR_PIN, HIGH);
	}
	return isConfigOK;
}


//*************************************************************************************
//		                             Standalone section
//*************************************************************************************

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

void readData() {
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
  Serial.println("driver \t microsteps \t current \t spreadCycle \t conf Status");
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
    Serial.print(defaults_en_spreadCycle[i]?"true":"false");
    Serial.print(spacer);
    Serial.print(spacer);
    Serial.print(conf_checked[i]==1?"OK":"ERROR !");
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


//*************************************************************************************
//		                               I2C section
//*************************************************************************************
void doCommandOnTMC(uint8_t channel, uint8_t command, unsigned int value) {

	// sanity check on channel value
	if (channel<0 || channel>4 || !use_tmc[channel]) {
    Serial.print("I2C error, channel requested not available : ");
    Serial.println(channel);
    return;
  }

	// Initiate the SoftwareSerial
	TMC2208Stepper *tmc = driver[channel];

	switch (command) {
		case RESET_CONF_REG :
				//TODO perhaps reinit the register, but not necessary during test phase
        i2c_value_req = 1;          // return 1 to say init is OK
				break;
		case SET_MICROSTEP_REG :
				tmc->microsteps(value);
				break;
		case SET_CURRENT_REG :
				tmc->rms_current(value); 						// Set driver current
				break;
		case SET_INTERPOL_256_REG  :
				tmc->intpol(value);			// enable or disable 256 microsteps interpolation
		 		break;
		case SET_SPREADCYCLE_EN_REG :
				tmc->en_spreadCycle(value); // Set the spreadCycle
				break;
		case SET_ENABLE_REG :
				tmc->toff(value?defaults_toff[channel]:0x00); // if value send, set default_toff, else set 0x00
				break;
		case GET_MICROSTEP_REG :
        i2c_value_req = tmc->microsteps();
				break;
		case GET_CURRENT_REG :
        i2c_value_req = tmc->rms_current();
        break;
		case GET_INTERPOL_256_REG :
        i2c_value_req = tmc->intpol();
        break;
		case GET_SPREADCYCLE_EN_REG :
        i2c_value_req = tmc->en_spreadCycle();
        break;
		case GET_ENABLE_REG :
        i2c_value_req = !(tmc->toff()==0); // If toff=0 the driver is disable, return false
        break;
		case GET_ALARM_REG :
        i2c_value_req = tmc->DRV_STATUS() ;
        break;
		default : break;
	}

  // Store the channel and the command request for the response
  i2c_channel_req = channel;
  i2c_command_req = command;
}

//receive 24 bits : data[24..9]command[8..4]channel[3..0]
void receiveEvent(int howMany) {
	uint8_t buffer[3];
	byte cursor=0;
  while (Wire.available() && cursor<3) {      // loop through all but the last
    buffer[cursor]= Wire.read(); 							// receive byte as a character
    cursor++;              						        // print the character
  }

  uint8_t channel;
  uint8_t command;
  unsigned int value;

  tmc2ic.decode(buffer, 3, &channel, &command, &value);
  #ifdef DEBUG
    Serial.print("I2C receive :");
    Serial.print(buffer[0],HEX);Serial.print(buffer[1],HEX);Serial.print(buffer[2],HEX);
    Serial.print('(');
    Serial.print(buffer[0],BIN);Serial.print(buffer[1],BIN);Serial.print(buffer[2],BIN);
    Serial.print(')');
    Serial.print(" -> channel|command|value :");
    Serial.print(channel); Serial.print('|');
    Serial.print(command); Serial.print('|');
    Serial.println(value);
  #endif

	doCommandOnTMC(channel, command, value);
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {

  uint8_t buffer[3];
  tmc2ic.encode(buffer, 3, i2c_channel_req, i2c_command_req, i2c_value_req);

  #ifdef DEBUG
    Serial.print("I2C respond channel|command|value : ");
    Serial.print(i2c_channel_req);  Serial.print('|');
    Serial.print(i2c_command_req);  Serial.print('|');
    Serial.print(i2c_value_req);    Serial.print(" -> ");
    Serial.print(buffer[0],HEX);Serial.print(buffer[1],HEX);Serial.print(buffer[2],HEX);
    Serial.print('(');
    Serial.print(buffer[0],BIN);Serial.print(buffer[1],BIN);Serial.print(buffer[2],BIN);   
    Serial.println(')');
  #endif

  Wire.write(buffer, 3); // respond with message of 3 bytes
  // as expected by master
}

//*************************************************************************************
//		                               Arduino section
//*************************************************************************************
void setup() {

  Serial.begin(115200);

	// the
	pinMode(ERROR_PIN, OUTPUT);
	digitalWrite(ERROR_PIN, HIGH);

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

	Serial.print("init mode : ");
	// If I2C is enable, start the I2C communication
	if (ENABLE_I2C) {
		// Init the i2c mode
		Serial.println("I2C");

		// Init the I2C component
		Wire.begin(I2C_ADDRESS);                // join i2c bus with address #8
		Wire.onReceive(receiveEvent); 					// register event when this receive from i2c
		Wire.onRequest(requestEvent);						// register event when this sent to i2c

	} else {
		// Init the standalone mode
		Serial.println("Standalone");

	  // at startup wait driver are online to setup them
	  delay(DELAY_BEFORE_STARTUP_CONF);

	}

  // setup the driver
  applySettings(ENABLE_I2C);

	// check the actual config of drivers and if wrong switch HIGH pin ERROR
	checkConfigAndSendDiag(ENABLE_I2C);

  Serial.println("Startup end !");

}

void loop() {

	unsigned long time = millis();

  sCmd.readSerial();

	// if the monitoring is started, read the drivers data
  if (!DriversBusy && isConfigOK && startedMonitoring &&!ENABLE_I2C) { // if nothing talk with driver, we can read them
		readData();
  }

	if (!isConfigOK && AUTO_CONF_ON_ERROR && ((time - lastTime)>DELAY_BEETWEEN_CHECK_CONF)) {
		Serial.println("Auto restart config on error...");
		lastTime = time;
		applySettings(ENABLE_I2C);
		checkConfigAndSendDiag(ENABLE_I2C);
	}

	// if pin "check" is up, check the config, and wait 1 sec before do a new step on
	if (!ENABLE_I2C && digitalRead(CHECK_PIN) && ((time - lastTime)>DELAY_BEETWEEN_CHECK_CONF)) {
		Serial.println("Check pin is enable and it's time to check...");
		lastTime = time;
		checkConfigAndSendDiag(ENABLE_I2C);
	}

	delay(200); // to not stress CPU

}
