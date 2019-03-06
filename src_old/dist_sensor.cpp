/***************************************************************************
 *   Copyright (C) 2015 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 * Line sensor functions
 *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include "main.h"
#include "dist_sensor.h"
#include "eeconfig.h"
#include "pins.h"
/** sensor 0 (called 1 in interface) is normally looking to the left (pin 1(gnd),2(+), and 3(analog value) in plug)
 *  sensor 1 (called 2 in interface) is normally looking forward (pin 4,5,6 in plug) */
uint32_t irRaw[2]; 
uint16_t irRawAD[2]; 
bool useDistSensor = false;
bool distSensorInstalled = true;
bool initIrFilter = true;
//
float irDistance[2];
uint32_t irCal20cm[2] = {72300, 72300}; 
uint32_t irCal80cm[2] = {12500, 12500};

float irA[2] = {1e-4, 1};
float irB[2] = {1e-4, 1};

void sendStatusDistIR()
{
  const int MRL = 64;
  char reply[MRL];
  snprintf(reply, MRL, "irc %.3f %.3f %lu %lu %ld %ld %ld %ld %d %d\r\n" ,
           irDistance[0], irDistance[1],
           irRaw[0], irRaw[1],
           irCal20cm[0], irCal80cm[0],
           irCal20cm[1], irCal80cm[1],
           useDistSensor,
           distSensorInstalled
  );
  usb_send_str(reply);
}

///////////////////////////////////////////////////////

void calibrateIr()
{
  // 13cm - 50cm calibration
  // inverse of distance is linear, if measured from GP2Y0A21 base
  // 2 measurements ad d1=13cm and d2=50cm - (scaled by 10000 in c-code)
  // corresponding values v1=irCal20cm, v2 = irCal50cm
  // as inclination:
  // dri = (1/d1 - 1/d2)/(v1 - v2)
  // any value r(x) for an AD measurement x is then
  // 1/r(x) = dri * x - dri * v2 + 1/d2, reduced to
  // 1/r(x) = irA * x - irB, or 
  // r(x ) = 1/(irA * x - irB)
  //
  // calibration fixed distances - inverted and scaled
  const int32_t d1 = 10000/13; // cm
  const int32_t d2 = 10000/50; // cm
  for (int i = 0; i < 2; i++)
  { // calculate constants for both sensors
    const int32_t v1 = irCal20cm[i];
    const int32_t v2 = irCal80cm[i];
    float dri = float(d1-d2)/float(v1-v2);
    // and scaled back to meters^-1
    irA[i] = dri / 100.0; // inclination
    irB[i] = (dri * v2 - d2) / 100.0; // offset
    // debug
    //         if (hbTimerCnt %200 == 0)
    {
      const int MSL = 90;
      char s[MSL];
      snprintf(s, MSL, "# ir%d, irA=%g, irB=%g, dri=%g\n", i, irA[i], irB[i], dri);
      usb_send_str(s);
    }
    // debug 2
  }
}

////////////////////////////////////////////////

bool setIrCalibrate(const char * buf)
{
  bool used = false; 
  { // is for the line sensor
    if (strncmp(buf, "irc", 3) == 0)
    { // 
      char * p1 = (char *)&buf[4];
      used = true;
      irCal20cm[0] = strtol(p1, &p1, 10);
      irCal80cm[0] = strtol(p1, &p1, 10);
      irCal20cm[1] = strtol(p1, &p1, 10);
      irCal80cm[1] = strtol(p1, &p1, 10);
      setIRpower(strtol(p1, &p1, 10));
      distSensorInstalled = strtol(p1, &p1, 10);
      //usb_send_str("# got an irc\n");      
      //
      calibrateIr();
    }
  }
  return used;  
}

/////////////////////////////////////////////////////////

void estimateIrDistance()
{
  if (useDistSensor)
  { // dist sensor has power, so estimate
    // is updated every approx 32ms or slower, so average over 32 samples 
    if (initIrFilter)
    { // when IR is first turned on
      irRaw[0] = irRawAD[0] * 32;
      irRaw[1] = irRawAD[1] * 32;
      initIrFilter = false;
    }
    else
    {
      irRaw[0] = (irRaw[0]*31)/32 + irRawAD[0];
      irRaw[1] = (irRaw[1]*31)/32 + irRawAD[1];
    }
    irDistance[0] = 1.0/(float(irRaw[0]) * irA[0] - irB[0]);
    irDistance[1] = 1.0/(float(irRaw[1]) * irA[1] - irB[1]);
    // debug
//     if (hbTimerCnt %200 == 0)
//     {
//       const int MSL = 70;
//       char s[MSL];
//       snprintf(s, MSL, "# irRaw=%lu, irA=%g, irB=%g, d=%g\n", irRaw[1], irA[1], irB[1], irDistance[1]);
//       usb_send_str(s);
//     }
    // debug 2
    if (irDistance[0] > 1.5 or irDistance[0] < 0.05)
      irDistance[0] = 1.5;
    if (irDistance[1] > 1.5 or irDistance[1] < 0.05)
      irDistance[1] = 1.5;
  }
  else
  { // not installed or not on (set to far away 10m)
    irDistance[0] = 10.0;
    irDistance[1] = 10.0;
  }
}

/////////////////////////////////////

void eePromSaveIr()
{
  uint8_t f = 0;
  if (useDistSensor) f = 1 << 0;
  if (distSensorInstalled) f += 1 << 1;
  eeConfig.pushByte(f);
  eeConfig.pushWord(irCal20cm[0]/8);
  eeConfig.pushWord(irCal20cm[1]/8);
  eeConfig.pushWord(irCal80cm[0]/8);
  eeConfig.pushWord(irCal80cm[1]/8);
}

/////////////////////////////////////

void eePromLoadIr()
{
  int f = eeConfig.readByte();
  distSensorInstalled = f & (1 << 1);
  setIRpower(f & (1 << 0));
  int skipCount = 2 + 2 + 2 + 2;
  //
  if (not eeConfig.isStringConfig())
  { // load from flash
    irCal20cm[0] = eeConfig.readWord()*8;
    irCal20cm[1] = eeConfig.readWord()*8;
    irCal80cm[0] = eeConfig.readWord()*8;
    irCal80cm[1] = eeConfig.readWord()*8;
    if (irCal20cm[0] < 50000)
    { // old calibration values - set default
      irCal20cm[0] = 80000;
      irCal80cm[0] = 24000;
      irCal20cm[1] = irCal20cm[0];
      irCal80cm[1] = irCal80cm[0];
    }
    calibrateIr();
  }
  else
    // load from hard-coded mission
    eeConfig.skipAddr(skipCount);
}

void setIRpower(bool power)
{
  if (distSensorInstalled)
  {
    if (power and not useDistSensor)
    { // initialize average fileter
      initIrFilter = true;
    }
    useDistSensor = power;
  }
  else
    useDistSensor = false;
  //
  digitalWriteFast(PIN_POWER_IR, useDistSensor);
}
