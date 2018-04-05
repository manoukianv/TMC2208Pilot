# TMC2208Pilot
## Description
Application running on Arduino to setup and monitor TMC2208 by UART. You can :
* Setup the currents, hold currents
* Setup Microsteps from 1 to 256 real
* Switch beetween StealChop and SpreadCycle
* Monitoring temperature, max_current, actual mode (StealChop and SpreadCycle) and actual microstepping.

## How does it works :
Connect a usb connector to power off the

WARN : The arduino setup driver in startup step, so if you reset you printer controller, you have to reapply settings or wiring the reset pin of the arduino board on the reset pin of the main board (check the voltage signal before : all is fine with the arduino board) !

## BOM & Wiring
Check the wiki for [BOM](https://github.com/manoukianv/TMC2208Pilot/wiki/BOM) and [wiring](https://github.com/manoukianv/TMC2208Pilot/wiki/Wiring)

# Quick starts
## Installation
1. Download ATOM : https://atom.io/
2. Install Platform.io for ATOM : https://platformio.org/get-started/ide?install=atom
3. Download / clone project on the github

## Setup
The inital setup values are in <conf.h> file

## Starting
1. update the sketch on the arduino (Alt + Cmd + U)
2. connect the serial terminal to use commands.

## Commands
Command can be send to the arduino by the terminal port (case sensitive)
* getConf : display the settings
* startMon : start to monitoring drivers, start to collect data (this can stress drivers)
* getMon : display the collected data
* stopMon : stop to collect data

Do not use monitoring on a full print, just do it several time to collect and check data

# Copyright
Using this free lib https://github.com/teemuatlut/TMC2208Stepper
