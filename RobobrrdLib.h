/*
  Robobrrd Library
  ----------------
  By Jac Goudsmit
  Licensed under the BSD (3-clause) license, see end of file.
  
  This library provides an API for the Robobrrd servos and sensors, to make
  it easier to write many applications without knowing the details of the
  Robobrrd hardware.
  
  The pin assignments and servo calibration values can be supplied by the
  sketch (e.g. by using the EEPROM_mgr library). This makes it easier to
  use the same sketch on multiple Robobrrds that may be wired differently
  and/or different calibration settings.
*/

#ifndef ROBOBRRDLIB_H
#define ROBOBRRDLIB_H

// Definitions for extra features
#define ROBOBRRD_HAS_GPS
#define ROBOBRRD_HAS_LCD

// Uncomment this to generate GPS logging
//#define DEBUG_GPS

#include <Arduino.h>

/////////////////////////////////////////////////////////////////////////////
// LIBRARY DEPENDENCIES
//
// To use this library, copy the following lines to your sketch, under the
// #include <RobobrrdLib.h> directive.
/////////////////////////////////////////////////////////////////////////////

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


/////////////////////////////////////////////////////////////////////////////
// Robobrrd class
class RoboBrrd
{
  //=========================================================================
  // Subtypes
  //=========================================================================

  
  //-------------------------------------------------------------------------
  // Servo index enum
public:
  enum ServoSelect
  {
    LWing,
    RWing,
    Beak,
   
    NumServos 
  };
  

  //-------------------------------------------------------------------------
  // Servo multi-selection enum
  enum Servos
  {
    ServosLeft  = (1 << LWing),
    ServosRight = (1 << RWing),
    ServosBeak  = (1 << Beak ),
    
    ServosWings = (ServosLeft  | ServosRight),
    ServosAll   = (ServosWings | ServosBeak),
  };

  
  //-------------------------------------------------------------------------
  // LED and LDR selection enum
public:
  enum Side
  {
    Left,
    Right,
    
    NumSides
  };
  
  
  //-------------------------------------------------------------------------
  // Multiple side selection enum
public:
  enum Sides
  {
    SidesLeft  = (1 << Left),
    SidesRight = (1 << Right),
    SidesBoth  = (SidesLeft | SidesRight),
  };
  
  
  //-------------------------------------------------------------------------
  // LED color index enum
public:
  enum RGB
  {
    Red,
    Green,
    Blue,
    
    NumRGB
  };
  
  
  //-------------------------------------------------------------------------
  // Wing and beak control constants
public:
  enum Pos
  {
    Middle,
    Upper,
    Open = Upper,
    Lower,
    Closed = Lower,
    
    NumPos
  };


#ifdef ROBOBRRD_HAS_GPS
  //-------------------------------------------------------------------------
  // GPS state
public:
  enum GPSState { 
    GPSStateUnknown,
    GPSStateOnline,
    GPSStateData,
    GPSStateValidData,
    GPSStateGotTime,
    GPSStateGotDate
    // To be expanded...
    
  };
#endif


#ifdef ROBOBRRD_HAS_LCD
  //-------------------------------------------------------------------------
  // Single button
public:
  enum Button
  {
    BtnSelect  = 0,
    BtnRight   = 1,
    BtnDown    = 2,
    BtnUp      = 3,
    BtnLeft    = 4,
    
    NumButtons
  };
  
  
  //-------------------------------------------------------------------------
  // Multiple buttons
public:
  enum Buttons
  {
    BtnsNone   = 0,
    
    BtnsLeft   = (1 << BtnLeft),
    BtnsRight  = (1 << BtnRight),
    BtnsUp     = (1 << BtnUp),
    BtnsDown   = (1 << BtnDown),
    BtnsSelect = (1 << BtnSelect),

    // Some combinations I care about
    // The ones that aren't mentioned here are still legal in functions that
    // take an argument of this type; feel free to combine values with
    // bitwise OR, or expand the enum type as required.
    BtnsLeftRight = BtnsLeft | BtnsRight,
    BtnsUpDown    = BtnsUp   | BtnsDown,
    
    BtnsAny    = BtnsLeft | BtnsRight | BtnsUp | BtnsDown | BtnsSelect,
  };    
#endif

  
  //-------------------------------------------------------------------------
  // Pin assignments struct
public:
  struct Pins
  {
    byte            m_ledpin[NumSides][NumRGB];
    byte            m_speakerpin;
    byte            m_servopin[NumServos];
    byte            m_ldrpin[NumSides];
    byte            m_thermometerpin;
#ifdef ROBOBRRD_HAS_GPS
    byte            m_gps_outpin;
    byte            m_gps_inpin;
#endif

  } static const DefaultPins;


  //-------------------------------------------------------------------------
  // Setup values
public:
  struct Values
  {
    byte            m_servopos[NumServos][NumPos];
    unsigned        m_supply_mv;
#ifdef ROBOBRRD_HAS_GPS
    int8_t          m_timezone;
#endif
#ifdef ROBOBRRD_HAS_LCD
    byte            m_lcdrows;
    byte            m_lcdcolumns;
#endif
    
  } static DefaultValues;


  //=========================================================================
  // Members
  //=========================================================================


public:
  const Pins       &m_pins;
  Values           &m_values;
  Servo             m_servo[NumServos];
#ifdef ROBOBRRD_HAS_GPS
  GPSState          m_gps_state;
  SoftwareSerial    m_gps_serial;
  TinyGPS           m_gps;
#endif
#ifdef ROBOBRRD_HAS_LCD
  Adafruit_RGBLCDShield m_lcd;
#endif


  //=========================================================================
  // Constructor/Destructor
  //=========================================================================
  

public:
  RoboBrrd(
    Values &values = DefaultValues, // Calibration values, must remain valid, may be changed
    const Pins &pins = DefaultPins) // Pin assignments, must remain valid at all times
  : m_pins(pins)
  , m_values(values)
  , m_servo()
#ifdef ROBOBRRD_HAS_GPS
  , m_gps_state(GPSStateUnknown)
  , m_gps_serial(m_pins.m_gps_inpin, m_pins.m_gps_outpin)
  , m_gps()
#endif
#ifdef ROBOBRRD_HAS_LCD
  , m_lcd()
#endif
  {
  }
  
  
  //=========================================================================
  // Servo functions
  //=========================================================================
public:
  // Attach servos to their pins
  void AttachServos()
  {
    for (int i = 0; i < NumServos; i++)
    {
      m_servo[i].attach(m_pins.m_servopin[i]);
    }
  }


  // Detach servos from their pins
  void DetachServos()
  {
    for (int i = 0; i < NumServos; i++)
    {
      m_servo[i].detach();
    }
  }
  

  // Servo control, one servo at a time
  void Move(
    ServoSelect which,
    Pos pos)
  {
    m_servo[which].write(m_values.m_servopos[which][pos]);
  }
  

  // Servo control, multiple servos
  void Move(
    Servos which,
    Pos pos)
  {
    for (int i = 0; i < NumServos; i++)
    {
      if (0 != (which & (1 << i)))
      {
        Move((ServoSelect)i, pos);
      }
    }
  }
  
  
  // Servo control, precision control
  // Use 0 to move to the middle, a positive value to move up (or open)
  // or a negative value to move down (close).
  // Values can be between -256 and 255.
  void MoveExact(
    ServoSelect which,
    int pos)
  {
    int v;

    if (pos >= 0)
    {
      v = map(pos, 0, 255, 
        m_values.m_servopos[which][Middle], 
        m_values.m_servopos[which][Upper]);
    }
    else
    {
      v = map(pos, -256, 0,
        m_values.m_servopos[which][Lower],
        m_values.m_servopos[which][Middle]);
    }
    m_servo[which].write(v);
  }
  
  
  // Move exact, multiple servos
  void MoveExact(
    Servos which,
    int pos)
  {
    for (int i = 0; i < NumServos; i++)
    {
      if (0 != (which & (1 << i)))
      {
        MoveExact((ServoSelect)i, pos);
      }
    }
  }
  
  
  //=========================================================================
  // LEDs
  //=========================================================================
public:
  // Attach the LEDs
  void AttachLeds()
  {
    for (int side = 0; side < NumSides; side++)
    {
      for (int rgb = 0; rgb < NumRGB; rgb++)
      {
        pinMode(m_pins.m_ledpin[side][rgb], OUTPUT);
      }
    }
  }


  // Turn an LED on or off     
  void LedOnOff(
    Side side,
    RGB rgb,
    boolean on)
  {
    digitalWrite(m_pins.m_ledpin[side][rgb], on ? HIGH : LOW);
  }


  // Turn multiple LEDs on or off
  void LedOnOff(
    Sides sides,
    RGB rgb,
    boolean on)
  {
    for (int i = 0; i < NumSides; i++)
    {
      if (0 != (sides & (1 << i)))
      {
        LedOnOff((Side)i, rgb, on);
      }
    }
  }
  
  
  // Turn given RGB LEDs on or off
  void LedOnOff(
    Side side,
    boolean r,
    boolean g,
    boolean b)
  {
    Led(side, Red,   r);
    Led(side, Green, g);
    Led(side, Blue,  b);
  }
  
  
  // Turn multiple RGB LEDs on or off
  void LedOnOff(
    Sides sides,
    boolean r,
    boolean g,
    boolean b)
  {
    for (int i = 0; i < NumSides; i++)
    {
      if (0 != (sides & (1 << i)))
      {
        LedOnOff((Side)i, r, g, b);
      }
    }
  }
  

  // Turn LEDs on or off based on bit pattern
  void LedBits(
    Side side,
    int rgb)
  {
    for (int i = 0; i < NumRGB; i++)
    {
      LedOnOff(side, (RGB)i, (0 != (rgb & (1 << i))));
    }
  }
  
  
  // Turn LEDs on or off based on bit pattern
  void LedBits(
    Sides sides,
    int rgb)
  {
    for (int i = 0; i < NumSides; i++)
    {
      LedBits((Side)i, rgb);
    }
  }
  
  
  // Set analog (PWM) brightness for an LED
  void Led(
    Side side,
    RGB rgb,
    byte brightness)
  {
    analogWrite(m_pins.m_ledpin[side][rgb], brightness);
  }
  
  
  // Set analog (PWM) brightness for multiple LEDs
  void Led(
    Sides sides,
    RGB rgb,
    byte brightness)
  {
    for (int i = 0; i < NumSides; i++)
    {
      if (0 != (sides & (1 << i)))
      {
        Led((Side)i, rgb, brightness);
      }
    }
  }
    
    
  // Set analog (PWM) values for given RGB LEDs  
  void Led(
    Side side,
    byte r,
    byte g,
    byte b)
  {
    Led(side, Red,   r);
    Led(side, Green, g);
    Led(side, Blue,  b);
  }    
  
  
  // Set analog (PWM) values for given RGB LEDs on multiple sides
  void Led(
    Sides sides,
    byte r,
    byte g,
    byte b)
  {
    for (int i = 0; i < NumSides; i++)
    {
      if (0 != (sides & (1 << i)))
      {
        Led((Side)i, r, g, b);
      }
    }
  }
  
  
  //=========================================================================
  // Speaker
  //=========================================================================
public:
  // Attach the speaker
  void AttachSpeaker()
  {
    pinMode(m_pins.m_speakerpin, OUTPUT);
  }
  
  
  // Play a tone
  void Tone(
    unsigned halfperiod, // microseconds
    unsigned long duration) // ms
  {
    for (unsigned long i = 0; i < duration * 1000L; i += halfperiod * 2)
    {
      digitalWrite(m_pins.m_speakerpin, HIGH);
      delayMicroseconds(halfperiod);
      digitalWrite(m_pins.m_speakerpin, LOW);
      delayMicroseconds(halfperiod);
    }
  }
  
  
  //=========================================================================
  // LDR's
  //=========================================================================
public:
  // Attach LDR's
  void AttachLDRs()
  {
    for (int i = 0; i < NumSides; i++)
    {
      pinMode(m_pins.m_ldrpin[i], INPUT);
    }
  }
  
  
  // Get LDR value
  int GetLDR(
    Side side)
  {
    return analogRead(m_pins.m_ldrpin[side]);
  }
  
  
  //=========================================================================
  // Thermometer
  //=========================================================================
public:
  // Attach thermometer
  void AttachThermometer()
  {
    pinMode(m_pins.m_thermometerpin, INPUT);
  }
  
  
  // Get thermometer value in tenths of degrees Celsius
  // The library uses integer arithmetic only to minimize the size of the
  // code. Your sketch can either handle the result as it is, or it can
  // convert the value to a float and divide by 10.
  // Note: because of various issues (such as noise on the power supply),
  // the best way to get an accurate reading is to read the value a 
  // couple of times with a short interval, and average the results.
  int GetCelsius()
  {
    // The thermometer generates a voltage on its output that represents the
    // temperature in Celsius, where each 0.1 degree equals 10mV and 0C is
    // 500mV. The A/D converter is 10 bits and the analog read result is
    // based on the supply voltage.
    return (((unsigned long)analogRead(m_pins.m_thermometerpin) 
      * (unsigned long)m_values.m_supply_mv) >> 10) - 500;
  }

  
  // Convert tenths of Celsius to tenths of Fahrenheit
  // Provided as a separate function so that you don't have to take an 
  // extra reading to the Fahrenheit value once you have the Celsius
  // value
  static int C10toF10(
    int celsius)
  {
    // 32 F is 0 C, and each degree F is 5/9 of a degree C.
    return 320 + ((celsius * 9) / 5);
  }

  
  // Get thermometer value in tenths of degrees Fahrenheit
  // The library uses integer arithmetic only to minimize the size of the
  // code. Your sketch can either handle the result as it is, or it can
  // convert the value to a float and divide by 10.
  // Note: because of various issues (such as noise on the power supply),
  // the best way to get an accurate reading is to read the value a 
  // couple of times with a short interval, and average the results.
  int GetFahrenheit()
  {
    return C10toF10(GetCelsius());
  }
  
  
#ifdef ROBOBRRD_HAS_GPS

#ifdef DEBUG_GPS
#define GPSLOG(x) Serial.print(x)
#define GPSLOGLN(x) Serial.println(x)
#else
#define GPSLOG(x)
#define GPSLOGLN(x)
#endif

  //=========================================================================
  // GPS
  //=========================================================================

public:
  // Attach GPS and turn it on
  void AttachGPS()
  {
    // Nothing for now
    // (In the future, commands may be sent to the GPS module here)
    
    // Factory reset
    //m_gps_serial.println("$PMTK104*37");
    
    // send GPRMC and GPGGA
    m_gps_serial.println(F("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"));
    
    // send updates once per second
    m_gps_serial.println(F("$PMTK220,1000*1F"));
    
    // send updates 5x per second
    //m_gps_serial.println(F("$PMTK220,200*2C"));
    
    m_gps_state = GPSStateOnline;
  }
  

  // Disconnect GPS (turn it off)
  void DetachGPS()
  {
    // Nothing for now
    // (In the future, commands may be sent to the GPS module here)
    m_gps_state = GPSStateUnknown;
  }

  
  // Get data from GPS until an NMEA sentence is received.
  // Returns true if an update was received, false on timeout
  // NOTE: The servos should be detached while doing this.
  boolean GetGPS(unsigned long timeout)
  {
    m_gps_serial.begin(9600);
    
    boolean result = false;
    unsigned long ts = millis();
    
    for(;;)
    {
      // Get characters from GPS serial port and feed them to the GPS parser.
      // If there's nothing to read, test for timeout.
      unsigned numavailable = m_gps_serial.available();
      if (numavailable)
      {
        if (m_gps_state < GPSStateData)
        {
          m_gps_state = GPSStateData;
        }

        bool gotsentence = false;

        // Stuff as much data into the GPS parser as possible to avoid losing
        // data from the serial port
        do
        {
          GPSLOG("."); // Inidicate that we're processing the receive buffer
          for (int i = 0; i < numavailable; i++)
          {
            char c = m_gps_serial.read();
            //GPSLOG(c); // Dump incoming data
          
            if (m_gps.encode(c))
            {
              GPSLOG("*"); // Indicate that we got the end of a sentence
              gotsentence = true;
              
              // Note, we're not stopping here; keep reading until the GPS
              // is "quiet" for a while.
              // Experiments have shown that if we break out here, we often
              // still don't have valid data because the GPS sends so much
              // stuff that TinyGPS doesn't use. It's much more reliable to
              // simply keep going until we detect the interval between
              // updates instead.
            }
          }
        
          delay(1); // This seems to prevent framing errors
          
          // Check if there's more data available now
          numavailable = m_gps_serial.available();
          
          // Quit the loop between updates from the GPS
        } while (numavailable);
        
        GPSLOGLN("Parsing...");
        
        if (gotsentence)
        {
          if (m_gps_state < GPSStateValidData)
          {
            m_gps_state = GPSStateValidData;
          }
          
          unsigned long date;
          unsigned long time;
          
          m_gps.get_datetime(&date, &time, NULL);
          
          if ((time != TinyGPS::GPS_INVALID_TIME) && (m_gps_state < GPSStateGotTime))
          {
            m_gps_state = GPSStateGotTime;
            
            if ((date != TinyGPS::GPS_INVALID_DATE) && (m_gps_state < GPSStateGotDate))
            {
              m_gps_state = GPSStateGotDate;
            }
          }
            
          if (m_gps_state >= GPSStateGotDate)
          {
            GPSLOGLN("Got date and time");          
            result = true;
            break;
          }
        }
      }
      else
      {
        // If we received nothing, check for timeout
        if (millis() - ts >= timeout)
        {
          GPSLOGLN("GPS timeout");
          break;
        }
      }
    }
    
    m_gps_serial.end();
    
    return result;
  }
  
  
  // Get UTC time from the GPS module if possible
  boolean GetUTCTime(
    int *pyear,
    byte *pmonth,
    byte *pday,
    byte *phour,
    byte *pminute,
    byte *psecond,
    unsigned long timeout = 1500,
    unsigned long max_age = 500)
  {
    boolean result = false;
    
    if (GetGPS(timeout))
    {
      unsigned long age;

      m_gps.crack_datetime(pyear, pmonth, pday, phour, pminute, psecond, NULL, &age);

      GPSLOG("Got UTC. Age=");
      GPSLOGLN(age);
      
      if (age < max_age)
      {
        GPSLOGLN("Success UTC");      
        result = true;
      }
      else
      {
        GPSLOGLN("GPS lost");
      }
    }
    else
    {
      GPSLOGLN("GPS not received yet");
    }
    
    return result;
  }
  

  // Get UTC time from GPS as time_t type.
  // If something went wrong, the function returns 0.
  time_t GetUTCTime(
    unsigned long timeout = 1500,
    unsigned long max_age = 500)
  {
    time_t result = 0;
    tmElements_t te;
    int year;
    
    if (GetUTCTime(
      &year,
      &te.Month,
      &te.Day,
      &te.Hour,
      &te.Minute,
      &te.Second,
      timeout,
      max_age))
    {
      te.Year = CalendarYrToTm(year);
      result = makeTime(te);
    }

    GPSLOG("Current time is: ");
    GPSLOG(te.Year);
    GPSLOG("/");
    GPSLOG(te.Month);
    GPSLOG("/");
    GPSLOG(te.Day);
    GPSLOG(" ");
    GPSLOG(te.Hour);
    GPSLOG(":");
    GPSLOG(te.Minute);
    GPSLOG(":");
    GPSLOG(te.Second);   
    GPSLOG(" Got UTC=");
    GPSLOGLN(result);
    
    return result;
  }

  
  // Get local time from GPS
  // If something went wrong, the function returns 0.
  time_t GetLocalTime(
    unsigned long timeout = 1000,
    unsigned long max_age = 500)
  {
    time_t result = GetUTCTime(timeout, max_age);
    
    if (result)
    {
      result += m_values.m_timezone * SECS_PER_HOUR;
    }
    
    GPSLOG("Local time=");
    GPSLOGLN(result);
    
    return result;
  }
  
  
  // Static version of the above, needed by automatic time sync below
  // Unfortunately the Time library doesn't use a callback parameter so we
  // have to store the instance statically.
  static RoboBrrd *timeprovider;
  static time_t StaticLocalTime(void)
  {
    time_t result = 0;
    
    if (timeprovider)
    {
      // If the time has not been set yet, wait longer to give up
      // We can't call timeStatus() here, it would cause infinite recursion
      // so we remember our previous result
      static unsigned long timeout = 3000;

      result = timeprovider->GetLocalTime(timeout);
      
      // If GPS could not be synced, try again soon
      if (!result)
      {
        // If that didn't work, try again after a second
        setSyncInterval(1);
        
        // Wait extra long to get data
        timeout = 3000;
      }
      else
      {
        // If it worked, ask again after 5 minutes
        setSyncInterval(300);
        
        // No need to try very hard
        timeout = 1000;
      }
    }
    
    GPSLOGLN("Auto update called");
    
    return result;
  }
  
  
  // Call this function to enable synchronization of the system time in the
  // Time library with the GPS module.
  // After calling this, you should make sure that the servos are always
  // detached whenever you call a Time function; if not, the incoming
  // serial traffic from the GPS module will interfere with the operation
  // of the servos.
  void EnableGPSTimeSync(void)
  {
    timeprovider = this;
    setSyncProvider(StaticLocalTime);
  }
  
  
  // Disable automatic time synchronization
  void DisableGPSTimeSync(void)
  {
    timeprovider = NULL;
  }
#endif
  

#ifdef ROBOBRRD_HAS_LCD  
  //=========================================================================
  // LCD
  //=========================================================================
public:
  // Attach LCD
  // The LCD is cleared and the backlight is turned off.
  void AttachLCD()
  {
    m_lcd.begin(m_values.m_lcdcolumns, m_values.m_lcdrows);
    m_lcd.setBacklight(0);
    m_lcd.clear();
  }

  // Set backlight according to pattern
  void Backlight(
    boolean r,
    boolean g,
    boolean b)
  {
    m_lcd.setBacklight(
      (r ? 1 : 0) | (g ? 2 : 0) | (b ? 4 : 0));
  }
#endif

  //=========================================================================
  // Miscellaneous
  //=========================================================================
public:
  // Attach everything
  // This should be called in the setup() routine of the sketch.
  // To avoid moving the servos, use false as parameter
  virtual void Attach(
    boolean attachservos = true
#ifdef ROBOBRRD_HAS_GPS
    ,boolean autogpssync = false // Don't make TRUE if attaching servos!
#endif
    )
  {
    if (attachservos)
    {
      AttachServos();
    }
    AttachLeds();
    AttachSpeaker();
    AttachLDRs();
    AttachThermometer();
#ifdef ROBOBRRD_HAS_GPS
    AttachGPS();
#endif
#ifdef ROBOBRRD_HAS_LCD
    AttachLCD();
#endif

#ifdef ROBOBRRD_HAS_GPS
    if (autogpssync)
    {
      EnableGPSTimeSync();
    }
#endif
  }
};

#endif // ROBOBRRDLIB_H


/*
Copyright (c) 2013, Jac Goudsmit
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

