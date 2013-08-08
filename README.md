RobobrrdLib
===========

A library for the Robobrrd project. Robobrrd was designed and created by Erin "RobotGrrl" Kennedy.

The library provides an abstract API to the bird operations (movements of the sensors, changing the LED colors etc) and stores calibration values in EEPROM.

Applications can implement different behavior but can reuse the library. Storing the calibration values in EEPROM makes it possible to use the exact same sketch on multiple Robobrrds without modifications in the code.

The code is licensed under the same license as the original Robobrrd code.
