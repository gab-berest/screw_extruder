# Screw Extruder

Screw extruder is a program to control the speed and temperature of the screw extruder. Please refer to the usage manual for further instruction on usage and installation.

## Installation

Use the arduino software [Arduino](https://www.arduino.cc/en/software) to install the program directly via USB to the microcontroller. It is compatible with RAMPS 1.4 (tested). 
For other compatibilities, please chnage pin outputs in the main file defines.

## Usage

Use the encoder to select one of the five top menus: RPM, Winder speed, Nozzle Temperature, Preheat temperature and Fan speed.

RPM selects the motor speed in rotation per minute.
Temperature nozzle sets the control temperature of the nozzle heater band.
Temperature preheat sets the control temperature of the preheat heater band.
Fan speed sets the fan speed (0-10)
Winder speed selects the winder motor speed

Three bottom menus only show the current speed of the motor, the nozzle temperature and the two preheat temperatures.

Data folders are the data used to tune the default PID values.

## PID tuning folders

These forlders incorporate the data, transfer function and PID values of the PID tuning done for the nozzle heater band 3594K971 and the preheat heater bands 3594K118 on McMaster-Carr. 
These were used with Matlab and .csv files.