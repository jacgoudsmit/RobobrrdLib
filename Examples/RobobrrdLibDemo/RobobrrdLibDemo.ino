/*
  Robobrrd Library Demo
  ---------------------
  By Jac Goudsmit
  Based on RobotGrrl's RoboBrrd Kit Demo, (C) 2011-2013 Erin Kennedy
  Licensed under the BSD (3-clause) license, see end of file.
 
  First demo program for RoboBrrd! Try it out & make sure everything works
  robobrrd.com/kit
*/


#include <Arduino.h>
#include <EEPROM_mgr.h>
#include <Servo.h>
#include <Streaming.h>
#include "RobobrrdLib.h"


EEPROM_item<RoboBrrd::Values> settings(RoboBrrd::DefaultValues);
RoboBrrd robobrrd(RoboBrrd::DefaultPins, settings);


// -- LET'S GO! :) -- //
void setup()
{
  // -- serial
  Serial.begin(9600);
  Serial << "Hello from RoboBrrd! SQUAWK! CHIRP! "
    "Beginning initialization... beep beep" << endl;

  // At this point, the settings contain the default values.
  // If you wish, you can retrieve the calibrated values from the
  // EEPROM inside the Arduino by using EEPROM_mgr::RetrieveAll, but
  // only if you stored them there at least once, of course!
  //
  // See the calibration example program for more information.
  //EEPROM_mgr::RetrieveAll();
  
  // Set up all the pins but don't attach the servos yet
  // otherwise they may move suddenly. They're also noisy while they're
  // attached, so this keeps the brrd quiet until we start moving
  // things!
  robobrrd.Attach(false);
  
  // -- servos
  robobrrd.Move(RoboBrrd::Servos_All, RoboBrrd::Lower);
  
  // -- leds
  blinkLed(5, 100);
  
  robobrrd.Led(RoboBrrd::Left, 128, 128, 128);
  
  // -- misc
  for (int i = 0; i < 5; i++)
  {
    robobrrd.Tone(260, 70);
    robobrrd.Tone(280, 70);
    robobrrd.Tone(300, 70);
    delay(100);
  }

  Serial << "LDR L: " << robobrrd.GetLDR(RoboBrrd::Left)  << " ";
  Serial << "LDR R: " << robobrrd.GetLDR(RoboBrrd::Right) << " ";
  Serial << "TMP36: " << (float)(robobrrd.GetCelsius())/10.0 << "C/" 
                      << (float)(robobrrd.GetFahrenheit())/10.0 << "F" << endl;
  	
  // -- go!
  Serial << "Finished initializing! Beep bloop boop whirrrr" << endl;
}

void loop()
{
  robobrrd.AttachServos();
  for (int which = 0; which < RoboBrrd::NumServos; which++)
  { 
    switch(which)
    {
    case RoboBrrd::RWing: Serial << "--rwing--" << endl; break;
    case RoboBrrd::LWing: Serial << "--lwing--" << endl; break;
    case RoboBrrd::Beak:  Serial << "--beak--"  << endl; break;
    }

    blinkLed(1, 500);
	
    for (int i = -256; i < 255; i += 5)
    {
      robobrrd.MoveExact((RoboBrrd::ServoSelect)which, i);
      delay(5);
    }
	
    delay(500);
	
    for (int i = 255; i >= -256; i -= 5)
    {
      robobrrd.MoveExact((RoboBrrd::ServoSelect)which, i);
      delay(5);
    }
  }
  robobrrd.DetachServos();
  
  // -- heartbeat -- //
  Serial << "--heartbeat--" << endl;
  blinkLed(5, 100);

  // -- eyes -- //
  Serial << "--eyes--" << endl;
  blinkLed(4, 500);
	
  for (int i = 0; i < 8; i++)
  {
    robobrrd.LedBits(RoboBrrd::SidesBoth, i);
    delay(500);
  }
	
  for (int j = 0; j < RoboBrrd::NumRGB; j++) 
  {
    robobrrd.LedOnOff(RoboBrrd::SidesBoth, false, false, false);
    delay(100);

    // Note, with the default pin assignments, the following doesn't have
    // the intended effect because the Arduino doesn't support PWM on all
    // the pins.
    // - Red:   Only the right eye (default: pin 11) supports PWM
    // - Green: Only the left  eye (default: pin 5 ) supports PWM
    // - Blue:  Only the left  eye (default: pin 6 ) supports PWM
    // To fix this (and save 3 pins), connect both eyes to pins 5, 6 and 11.
    for (int i = 0; i < 256; i++) 
    {
      robobrrd.Led(RoboBrrd::SidesBoth, (RoboBrrd::RGB)j, i);
      delay(10);
    }
  }

  // -- sensors -- //
  Serial << "--sensors--" << endl;
  blinkLed(5, 500);

  for (int i = 0; i < 20; i++) 
  {
    Serial << "LDR L: " << robobrrd.GetLDR(RoboBrrd::Left)  << " ";
    Serial << "LDR R: " << robobrrd.GetLDR(RoboBrrd::Right) << " ";
    Serial << "TMP36: " << (float)(robobrrd.GetCelsius())/10.0 << "C/" 
                        << (float)(robobrrd.GetFahrenheit())/10.0 << "F" << endl;
    delay(50);
  }	
}

// -- BLINK -- //
void blinkLed(int rep, int del)
{
  for (int i = 0; i < rep; i++)
  {
    robobrrd.Led(RoboBrrd::Right, RoboBrrd::Blue, true);
    delay(del);
    robobrrd.Led(RoboBrrd::Right, RoboBrrd::Blue, false);
    delay(del);
  }
}


/*
Copyright (c) 2013, Jac Goudsmit
Based on code by RobotGrrl, (C) 2011-2013 Erin Kennedy
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.

  Neither the name of the {organization} nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

