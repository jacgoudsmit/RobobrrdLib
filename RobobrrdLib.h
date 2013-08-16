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

#include <Servo.h>

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
    Servos_Left  = (1 << LWing),
    Servos_Right = (1 << RWing),
    Servos_Beak  = (1 << Beak ),
    
    Servos_Wings = (Servos_Left  | Servos_Right),
    Servos_All   = (Servos_Wings | Servos_Beak),
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


  //-------------------------------------------------------------------------
  // Pin assignments struct
public:
  struct Pins
  {
    byte m_ledpin[NumSides][NumRGB];
    byte m_speakerpin;
    byte m_servopin[NumServos];
    byte m_ldrpin[NumSides];
    byte m_thermometerpin;
  } static const DefaultPins;


  //-------------------------------------------------------------------------
  // Setup values
public:
  struct Values
  {
    byte     m_servopos[NumServos][NumPos];
    unsigned m_supply_mv;
    
  } static DefaultValues;


  //=========================================================================
  // Members
  //=========================================================================


public:
  const Pins &m_pins;
  Values &m_values;
  Servo m_servo[NumServos];


  //=========================================================================
  // Constructor/Destructor
  //=========================================================================
  

public:
  RoboBrrd(
    const Pins &pins, // Pin assignments, must remain valid at all times
    Values &values) // Servo values, must remain valid, may be changed
  : m_pins(pins)
  , m_values(values)
  , m_servo()
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
  
  
  //=========================================================================
  // Miscellaneous
  //=========================================================================
public:
  // Attach everything
  // This should be called in the setup() routine of the sketch.
  // To avoid moving the servos, use false as parameter
  virtual void Attach(
    boolean attachservos = true)
  {
    if (attachservos)
    {
      AttachServos();
    }
    AttachLeds();
    AttachSpeaker();
    AttachLDRs();
    AttachThermometer();
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

