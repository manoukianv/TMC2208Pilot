# TMC2208Pilot
## Description
Application running on ESP8266/NodeMCU to setup and monitor TMC2208 by UART. You can :
* Setup the currents, hold currents
* Setup Microsteps from 1 to 256 real
* Switch beetween StealChop and SpreadCycle
* Monitoring temperature, max_current, actual mode (StealChop and SpreadCycle) and actual microstepping.

## How does it works :
The NodeMCU create a Wifi network called "TMC2208Pilot-AP" with the password "TMC2208PWD" (please change the parameters in the setup file).
Connect a computer/phone/tablet to this network and open a web browser on the 192.168.10.1, the application appears.
You can see the theoretical configuration and the real configuration, and reapply config.

WARN : The NodeMCU setup driver at NodeMCU bootup, so if you reset you printer controller, you have to reapply settings !

## BOM & Wiring
Check the wiki for [BOM](https://github.com/manoukianv/TMC2208Pilot/wiki/BOM) and wiring

# Quick starts
## Installation
1. Download ATOM : https://atom.io/
2. Install Platform.io for ATOM : https://platformio.org/get-started/ide?install=atom
3. Download / clone project on the github

## Setup
The inital setup values are in <conf.h> file

## Starting
1. update the sketch on the ESP8266 (Alt + Cmd + U)
2. connect the serial terminal to get the module IP.
3. Start a web browser on the URL http://<WIFI IP> . Data are refresh each 10s.

# Copyright
Using this free lib https://github.com/teemuatlut/TMC2208Stepper
