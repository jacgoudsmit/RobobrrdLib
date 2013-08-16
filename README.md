RobobrrdLib
===========

A library for the Robobrrd project. Robobrrd was designed and created by Erin "RobotGrrl" Kennedy.

The library provides an abstract API to the bird operations (movements of the servos, changing the LED colors, reading the sensors etc) and stores pin assignments and servo calibration settings in separate structs.

Storing the pin assignments and calibration values into separate structs makes it easy to make small variations in the same program, or re-use the same sketch on multiple robobrrds with varying hardware. My EEPROM_mgr library (http://github.com/jacgoudsmit/EEPROM_mgr) makes it possible to store settings in the on-board EEPROM; I provided a demo program to do this.

The code is licensed under the same license as the original Robobrrd code.
