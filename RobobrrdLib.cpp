/*
  Robobrrd Library
  ----------------
  By Jac Goudsmit
  Licensed under the BSD (3-clause) license, see end of file.
*/

#include <Arduino.h>
#include <Servo.h>

#include "RobobrrdLib.h"


//---------------------------------------------------------------------------
// Default pin assignments
//
// Unless you modified your Robobrrd, you probably want to use this as
// parameter to the constructor of the RoboBrrd class.
const RoboBrrd::Pins RoboBrrd::DefaultPins =
{
  //                Left R/G/B     Right R/G/B
  m_ledpin:         { { 4, 5, 6, }, { 11, 12, 13 } },
  
  m_speakerpin:     7,
  
  //                Left/Right/Beak
  m_servopin:       { 8, 9, 10, },
  
  //                Left/Right
  m_ldrpin:         { A0, A1 },
  
  m_thermometerpin: A2,
};


//---------------------------------------------------------------------------
// Default values
//
// These are values for the servos that can be used before calibration
RoboBrrd::Values RoboBrrd::DefaultValues =
{
  m_servopos:
  {
    { 90, 40, 140 },
    { 90, 140, 40 },
    { 90, 40, 140 },
  },
  
  m_supply_mv: 5000,
};


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

