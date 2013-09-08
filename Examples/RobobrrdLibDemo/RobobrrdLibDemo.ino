/*
  Robobrrd Library Demo
 ---------------------
 By Jac Goudsmit
 Based on RobotGrrl's RoboBrrd Kit Demo, (C) 2011-2013 Erin Kennedy
 Licensed under the BSD (3-clause) license, see end of file.
 
 First demo program for RoboBrrd! Try it out & make sure everything works
 robobrrd.com/kit
 */


#include <Streaming.h>
#include <RobobrrdLib.h>

// Copied from RobobrrdLib.h -->
#include <Servo.h>

#ifdef ROBOBRRD_HAS_GPS
#include <SoftwareSerial.h>
#include <TinyGPS.h> // http://arduiniana.org/libraries/tinygps/
#include <Time.h> // http://www.pjrc.com/teensy/td_libs_Time.html
#endif

#ifdef ROBOBRRD_HAS_LCD
#include <Wire.h>
#include <Adafruit_MCP23017.h> // https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library
#include <Adafruit_RGBLCDShield.h> // https://github.com/adafruit/Adafruit-RGB-LCD-Shield-Library
#endif
// <-- End


RoboBrrd robobrrd;


// -- LET'S GO! :) -- //
void setup()
{
  // -- serial
  Serial.begin(9600);
  Serial << "Hello from RoboBrrd! SQUAWK! CHIRP! "
    "Beginning initialization... beep beep" << endl;

  // Set up all the pins but don't attach the servos yet
  // otherwise they may move suddenly. They're also noisy while they're
  // attached, so this keeps the brrd quiet until we start moving
  // things!
#ifdef ROBOBRRD_HAS_GPS
  robobrrd.Attach(false, true);
#else
  robobrrd.Attach(false);
#endif

  // -- servos
  robobrrd.Move(RoboBrrd::ServosAll, RoboBrrd::Lower);

  // -- leds
  //blinkLed(5, 100);

  robobrrd.Led(RoboBrrd::Left, 128, 128, 128);

  // -- misc
#if 0  
  for (int i = 0; i < 5; i++)
  {
    robobrrd.Tone(260, 70);
    robobrrd.Tone(280, 70);
    robobrrd.Tone(300, 70);
    delay(100);
  }
#endif


  Serial << "LDR L: " << robobrrd.GetLDR(RoboBrrd::Left)  << " ";
  Serial << "LDR R: " << robobrrd.GetLDR(RoboBrrd::Right) << " ";
  Serial << "TMP36: " << (float)(robobrrd.GetCelsius())/10.0 << "C/" 
    << (float)(robobrrd.GetFahrenheit())/10.0 << "F" << endl;

  // -- go!
  Serial << "Finished initializing! Beep bloop boop whirrrr" << endl;
}

void Announce(bool line2, const char *fmt, ...)
{
  va_list args;
  char s[80];

  va_start(args, fmt);
  vsprintf(s, fmt, args);
  va_end(args);

  Serial << s << endl;
#ifdef ROBOBRRD_HAS_LCD
  if (line2)
  {
    robobrrd.m_lcd.setCursor(0,1);
  }
  else
  {
    robobrrd.m_lcd.clear();
  }
  strcpy(s + strlen(s), "                ");
  robobrrd.m_lcd.print(s);
#endif
}

#ifdef ROBOBRRD_HAS_LCD
boolean demoLCD()
{
  static boolean useservos = false;

  // -- LCD --
  robobrrd.m_lcd.setBacklight(7);
  Announce(false, "-- LCD --");
  //blinkLed(5, 500);

  Announce(true, "Hello Robobrrd!");

  for (int i = 0; i < 1000; i += 20)
  {
    byte b = robobrrd.m_lcd.readButtons();
    if (!b)
    {
      delay(20);
    }
    else
    {
      if (b & BUTTON_UP)
      {
        useservos = true;
        Announce(true, "Servo tests ON");
        break;
      }
      else if (b & BUTTON_DOWN)
      {
        useservos = false;
        Announce(true, "Servo tests OFF");
        break;
      }
      delay(1000);
    }
  }
  
  return useservos;
}
#endif

void demoServos()
{
  // -- Servos --
  Announce(false, "-- Servos --");
  robobrrd.AttachServos();
  for (int which = 0; which < RoboBrrd::NumServos; which++)
  { 
    switch(which)
    {
      case RoboBrrd::RWing: 
      Announce(true, "--rwing--"); 
      break;
      case RoboBrrd::LWing: 
      Announce(true, "--lwing--"); 
      break;
      case RoboBrrd::Beak:  
      Announce(true, "--beak--" ); 
      break;
    }

    //blinkLed(1, 500);
    for (int j = 0; j < 3; j++)
    {
      Announce(true, "Up");

      for (int i = -256; i < 255; i += 5)
      {
        robobrrd.MoveExact((RoboBrrd::ServoSelect)which, i);
        delay(5);
      }

      //        delay(500);

      Announce(true, "Down");

      for (int i = 255; i >= -256; i -= 5)
      {
        robobrrd.MoveExact((RoboBrrd::ServoSelect)which, i);
        delay(5);
      }
    }
  }
  robobrrd.DetachServos();
}

void demoEyes()
{
  // -- eyes -- //
  Announce(false, "--eyes--");
  //blinkLed(4, 500);

  for (int i = 0; i < 8; i++)
  {
    Announce(true, "Color %u", i);
    robobrrd.LedBits(RoboBrrd::SidesBoth, i);
    delay(500);
  }

  Announce(true, "fading...");

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
}

void demoSensors()
{
  // -- sensors -- //
  Announce(false, "--sensors--");
  //blinkLed(5, 500);

  for (int i = 0; i < 20; i++) 
  {
    Announce(true, "Light: %u/%u", 
    robobrrd.GetLDR(RoboBrrd::Left), 
    robobrrd.GetLDR(RoboBrrd::Right));
    delay(200);
  }

  for (int i = 0; i < 20; i++)
  {
    int c10 = robobrrd.GetCelsius();
    Announce(true, "Temp: %dC/%dF",
    c10 / 10, robobrrd.C10toF10(c10) / 10);
    delay(200);
  }
}

#ifdef ROBOBRRD_HAS_LCD
void demoBacklight()
{
  // -- LCD --
  Announce(false, "-- lcd backlight --");
  for (int i = 0; i < 8; i++)
  {
    Announce(true, "color %u", i);
    robobrrd.m_lcd.setBacklight(i);
    delay(500);
  }
}
#endif

#ifdef ROBOBRRD_HAS_GPS
void demoGPS()
{
  unsigned long lasttime = 0;
  
  // -- GPS --
  Announce(false, "--GPS--");

  tmElements_t tm;

  for (int i = 0; i < 10; i++)
  {
    if (timeStatus() != timeSet)
    {
      Announce(true, "Time not set (%u)", robobrrd.m_gps_state);
    }
    else
    {
      unsigned long curtime;

      for (;;)
      {
        if ((curtime = now()) != lasttime)
        {
          lasttime = curtime;
          break;
        }
      }
      
      breakTime(curtime, tm);

      Announce(true, "%2u:%02u:%02u", tm.Hour, tm.Minute, tm.Second);
    }
      
    delay(500);
  }

  if (timeStatus() == timeSet)
  {
    Announce(true, "%04u-%02u-%02u", tmYearToCalendar(tm.Year), tm.Month, tm.Day);
    delay(2000);
  }
}
#endif

void loop()
{
#ifdef ROBOBRRD_HAS_LCD
  if (demoLCD())
#endif
  {
    demoServos();
  }

  demoEyes();

  demoSensors();

#ifdef ROBOBRRD_HAS_LCD
  demoBacklight();  
#endif

#ifdef ROBOBRRD_HAS_GPS
  demoGPS();
#endif
}

// -- BLINK -- //
/*
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
 */

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



