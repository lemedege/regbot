/***************************************************************************
 *   Copyright (C) 2016 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 *   Main function for small regulation control object (regbot)
 *   build on a small 72MHz ARM processor MK20DX256,
 *   intended for 31300 Linear control
 *   has an IMU and a dual motor controller with current feedback.
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
 ***************************************************************************//*
  This file contains all the functions used for calculating
  the frequency, real Power, Vrms and Irms.
*/
#include <stdlib.h>
#include "main.h"
#include "servo.h"
#include "mission.h"
#include "control.h"
#include "robot.h"
#include "motor_controller.h"
#include "eeconfig.h"
#include "pins.h"

UServo::UServo()
{
//   initServo();
}


void UServo::initServo()
{
  // resolution set by motor controller
  //analogWriteRes(10); /// resolution (10 bit)
  if (robotHWversion >= 3 and robotHWversion != 5)
  { // frequency is common for a motor-pin too - on HW version 3
    pinMode(PIN_SERVO1, OUTPUT);
    pinMode(PIN_SERVO2, OUTPUT);
    pinMode(PIN_SERVO3, OUTPUT);
//     analogWriteFrequency(PIN_SERVO1, PWMfrq); /// frequency (Hz)
    analogWriteFrequency(PIN_SERVO2, PWMfrq); /// frequency (Hz)
//     analogWriteFrequency(PIN_SERVO3, PWMfrq); /// frequency (Hz)
    analogWrite(PIN_SERVO1, 0);
    analogWrite(PIN_SERVO2, 0);
    analogWrite(PIN_SERVO3, 0);
    // used by DAConverter - but may influence motor controller
  }
  // disable servo (analog value = 0)
  for (int i = 0; i < 3; i++)
  {
    servoEnabled[i] = false;
    servoValue[i] = 0;
    servoVel[i] = 2;
    servoRef[i] = 0;
    servoEnaRef[i] = false;
  }
  servo1isSteering = true;
  /// distance to front wheel (from drive wheels) - positive is front
  distToSteerWheel = 0.135;
  /// value for straight forward (nominal 0, range +/-512, positive is left
  forwardOffset = 0;
  /// angle turned for 1ms change (from 1 to 2 ms) in degrees
  anglePer_ms = 90;
}


/** 
 * \param pin allowed pin is 0,1. 
 * \param value input true is 1
 * \param enable if false, then port in set as input
 * */
void UServo::setServoPin(int8_t pin, int16_t value, bool enable)
{
  switch (pin)
  {
    case 0:
      if (enable)
      { // output
        pinMode(SERVO_PIN_0, OUTPUT);
        pinIsOutput[0] = true;
        analogWrite(SERVO_PIN_0, value);
        pin4Value = value;
      }
      else
      {
        pinMode(SERVO_PIN_0, INPUT);
        pinIsOutput[0] = false;
        pin4Value = analogRead(SERVO_PIN_0);
//           usb_send_str("# pin 4 (A14) set\r\n");
      }
//         usb_send_str("# pin 4 (A14) set\r\n");
      break;
    case 1:
      if (enable)
      {
        pinMode(SERVO_PIN_1, OUTPUT);
        pinIsOutput[1] = true;
        digitalWriteFast(SERVO_PIN_1, value != 0);
        pin5Value = value != 0; 
//         usb_send_str("# pin 5 (24) set\r\n");
      }
      else
      {
        pinMode(SERVO_PIN_1, INPUT);
        pinIsOutput[1] = false;
        pin5Value = digitalRead(SERVO_PIN_1);
      }
      break;
    default:
      usb_send_str("# unknown pin not set\r\n");
      break;
  }
}


void UServo::setServo ( int8_t idx, int16_t value, bool enable, int8_t vel )
{
  switch (idx)
  {
    case 1: setServo1PWM(value, enable, vel); break;
    case 2: setServo2PWM(value, enable, vel); break;
    case 3: setServo3PWM(value, enable, vel); break;
    case 4: // set analog value to DAC (A14)
    case 5: // set digital value to pin (24)
      setServoPin(idx - 4, value, enable);
      break;
    default:
      break;
  }
}

void UServo::releaseServos()
{
//   usb_send_str("# releasing servos\r\n");
  for (int i = 0; i < 3; i++)
  {
//     if (servoEnabled[i])
      // disable servo
      setServo(i+1, 0, false, 1);
  }
  // servo 4 pin (analog)
  setServoPin(0, 0, false);
  // servo 5 pin (digital)
  setServoPin(1, 0, false);
}

/**
 * Active castor wheel control
 * Known values: 
 * f = Distance to castor wheel [m] positive is forward
 * B = distance between differential wheels
 * v1 = left wheel velocity
 * v2 = right wheel velocity
 * Turn radius (positive to the right (CV)):
 * d = V2/((v1 - v2)/B) + B/2
 * d = V2 B / (v1 - v2) + B/2
 * d = (2 V2 B + B (V1 - V2)) / (2 (V1 - V2))
 * d = B(2 V2 + V1 - V2) / (2 (V1 - V2))
 * d = B(V1 + V2) / (2 (V1 - V2))
 * Turn angle (positive is CV)
 * a = atan(f/d)
 * a = atan(2f(v1 - v2) / (B(v1 + v2)))
 * a = atan2(2f(v1 - v2), B(v1 + v2))
 * */
float UServo::setServoSteering()
{
  float a = 0.111;
  if ((servo1isSteering and not balance_active and (fabs(mission_vel_ref) > 0.005)) or remoteControl)
  {
    const float f = distToSteerWheel; // distance to front wheel
    const float B = odoWheelBase; // probably 0.17 m
    // base angle on differential velocity reference
    float v1 = control.vel_ref[0];
    float v2 = control.vel_ref[1];
     int where = 1; //, lim=0;
    //if (fabs(v1) + fabs(v2) > 0.01)
    if (remoteControlNewValue or not remoteControl)
    { // we are driving
      float dh = 0.11111;
      if (mission_turn_do)
      { // convert turn radius to steer angle in radians
        if (mission_tr_turn > 0)
          a = atan2(f, mission_turn_radius);
        else
          a = atan2(f,-mission_turn_radius);
        where = 2;
      }
      else if (not (regul_line_use or (mission_irSensor_use > 0 and mission_wall_turn) or remoteControl))
      { // heading control active
        dh = mission_turn_ref - pose[2];
        if (dh < -M_PI)
        {
          dh += 2 * M_PI;
           where = 31;
        }
        else if (dh > M_PI)
        {
          dh -= 2 * M_PI;
           where = 32;
        }
        if (fabs(dh) < (5 * M_PI / 180.0))
        { // less than 10 deg - go straight
          a = 0;
           where = 33;
        }
        else
        {  // same as default
          a = atan2(2*f*(v2 - v1), B*(v1 + v2));
           where = 34;
        }
      }
      else 
      { // use velocity ref for heading
         where = 4;
        a = atan2(2*f*(v2 - v1), B*(v1 + v2));
      }
      //
      if (a < -M_PI/2)
      {
        a += M_PI;
//         lim = 1;
      }
      else if (a > M_PI/2)
      {
        a -= M_PI;
//         lim = -1;
      }
      // convert to servo msPulse
      const int scent = forwardOffset; // servo centre value (0=1.5ms, +512=2ms (as is +45 deg nominal))
      const float vpr = 1024/(anglePer_ms * M_PI/180);
      // positive value is CV
      int p = int(a * vpr) - scent;
      //
      // changed sign for savox servo (different from HK)
      servoVel[0] = 0; // full speed
      servoEnaRef[0] = true;
      if (p != servoRef[0])
      {
        servoRef[0] = p;
      }
      remoteControlNewValue = false;
      if (false and mission_turn_do)
      { // debug
        const int MSL = 100;
        char s[MSL];
        snprintf(s, MSL, "# new steer %d (a=%g, v1=%g, v2=%g, where=%d, tr=%g, dh=%g)\n", p, a, v1, v2, where, mission_turn_radius, dh);
        usb_send_str(s);
      }
    }
  }
//   else
//   { // debug
//     const int MSL = 100;
//     char s[MSL];
//     snprintf(s, MSL, "# no steer servo1isSteering=%d, balance_active=%d, mission_vel_ref=%f, remoteControl=%d)\n", 
//              servo1isSteering, balance_active, mission_vel_ref, remoteControl);
//     usb_send_str(s);
//   }
  
  return a;
}


void UServo::sendServoStatus()
{ // return servo status
  const int MSL = 100;
  char s[MSL];
  snprintf(s, MSL, "svo %d %d %d %d %d %d %d %d %d %d "
                   " %d %d %g %g\r\n", 
          servo.servoEnabled[0], servo.servoValue[0],
          servo.servoEnabled[1], servo.servoValue[1],
          servo.servoEnabled[2], servo.servoValue[2],
          servo.pinIsOutput[0], servo.pin4Value,
          servo.pinIsOutput[1], servo.pin5Value,
           servo1isSteering, forwardOffset, distToSteerWheel, anglePer_ms
  );
  usb_send_str(s);
}

void UServo::sendServo1Status()
{ // return servo status
  const int MSL = 100;
  char s[MSL];
  snprintf(s, MSL, "sv1 %d %d %g %g\r\n", 
          servo1isSteering, forwardOffset, 
           distToSteerWheel, anglePer_ms
          );
  usb_send_str(s);
}

void UServo::setServoConfig(const char* line)
{
  int16_t v, e;
  uint8_t a;
  const char * p1 = line;
  for (int i = 1; i <= 5; i++)
  { // get all set of values
    e = strtol(p1, (char**)&p1, 10);
    v = strtol(p1, (char**)&p1, 10);
    a = strtol(p1, (char**)&p1, 10);
    servo.setServo(i, v, e != 0, a);
  }
}

void UServo::setServo1Config(const char * line)
{
  int16_t v, e;
  const char * p1 = line;
  e = strtol(p1, (char**)&p1, 10);
  servo1isSteering = e != 0;
  v = strtol(p1, (char**)&p1, 10);
  forwardOffset = v;
  float f = strtof(p1, (char**)&p1);
  distToSteerWheel = f;
  f = strtof(p1, (char**)&p1);
  anglePer_ms = f;
  {
    const int MSL = 100;
    char s[MSL];
    snprintf(s, MSL, "# steer UServo::setServo1Config ofs=%d, dist=%g, angle=%g\r\n", forwardOffset, distToSteerWheel, anglePer_ms);
  }
}

void UServo::setOneServo(const char* line)
{
  if (robotHWversion >= 3 and robotHWversion != 5)
  {
    const char * p1 = line;
    int8_t idx = strtol(p1, (char**)&p1, 10);
    if ((idx >= 1 and idx < 6) or idx == -1)
    {
      int us = strtol(p1, (char**)&p1, 10);
      int vel = strtol(p1, (char**)&p1, 10);
      bool enable;
      if (idx < 4)
        enable = us >= -1024 and us <= 1024;
      else
        enable = us > -4096 and us < 4096;
      switch (idx)
      {
        case -1:  analogWriteFrequency(M2PWM1, (float)us); break; // timer used for servo PWM - set frequency in Hz
        case 1: 
          // servo 1 is steering, so may be active, if remote control is used - this ia a conflict
          // set back to true, if rc=1, ... is used
          remoteControl = false;
          servo.setServo1PWM(us, enable, vel); 
          break;
        case 2: servo.setServo2PWM(us, enable, vel); break;
        case 3: servo.setServo3PWM(us, enable, vel); break;
        case 4: servo.setServoPin(0, us, enable); break; // us=-1 means input (analog read)
        case 5: servo.setServoPin(1, us != 0, enable); break; // us=-1 means input
        default: break;
      }
      // debug
//       const int MSL = 75;
//       char s[MSL];
//       snprintf(s, MSL, "# %g servo %d (en=%d), '%s'\n", time,idx, enable, line);
//       usb_send_str(s);
      // debug end
    }
    else
      usb_send_str("# unknown servo: 1-3 is servo 4,5 is pins\r\n");
  }
  else
    usb_send_str("# supported on hardware version 3\r\n");
}


void UServo::servoTick()
{ // speed limit on servo
  // pt. servo 2 (index 1) handled only 
  if (servoEnabled[0] != servoEnaRef[0])
  { 
    if (not servoEnaRef[0])
    { // closing
      if (not digitalReadFast(PIN_SERVO1))
      { // output is zero, time to disable PWM for servo (konstant 0)
        analogWrite(PIN_SERVO1, 0);
        servoEnabled[0] = false;
//         usb_send_str("# servo 1 disabled\n");
      }
//       else
//         // waiting for PWM pulse to go low
//         usb_send_str("# servo 1 disable waiting\r\n");
    }
    else
      servoEnabled[0] = true;
  }
  if (servoEnabled[1] != servoEnaRef[1])
  { 
    if (not servoEnaRef[1])
    { // closing
      if (not digitalReadFast(PIN_SERVO2))
      { // output is zero, time to disable PWM for servo (konstant 0)
        analogWrite(PIN_SERVO2, 0);
        servoEnabled[1] = false;
//         usb_send_str("# servo 2 disabled\n");
      }
//       else
//         // waiting for PWM pulse to go low
//         usb_send_str("# servo 2 disable waiting\r\n");
    }
    else
      servoEnabled[1] = true;
  }
  if (servoEnabled[2] != servoEnaRef[2])
  { 
    if (not servoEnaRef[2])
    { // closing
      if (not digitalReadFast(PIN_SERVO3))
      { // output is zero, time to disable PWM for servo (konstant 0)
        analogWrite(PIN_SERVO3, 0);
        servoEnabled[2] = false;
//         usb_send_str("# servo 3 disabled\n");
      }
//       else
//         // waiting for PWM pulse to go low
//         usb_send_str("# servo 3 disable waiting\r\n");
    }
    else
      servoEnabled[2] = true;
  }
  //
  // set servo position - if enabled
  if (servoEnabled[0] and (hbTimerCnt % 4 == 0))
  { // often, this is a fast servo
    int dw = servoRef[0] - servoValue[0];
    if (abs(dw) > servoVel[0] and servoVel[0] > 0)
    {
      if (dw > 0)
        dw = servoVel[0];
      else
        dw = -servoVel[0];
    }
    servoValue[0] += dw;
    if (dw != 0)
    { // implement new value
      // midt v= 2040, min=1240, max= 2840
      // valid for HiTec HS7235-MH in high angle mode
      int v = (servoValue[0] * msPulse/2) / 512 + midt;
      if (v < 1240)
        v = 1240;
      else if (v > 2840)
        v = 2840;
      analogWrite(PIN_SERVO1, v);
      //
      if (false)
      { // debug
        const int MSL = 100;
        char s[MSL];
        snprintf(s, MSL, "# servo 1 to %d (%d)\n", v, servoValue[0]);
        usb_send_str(s);
      }
    }
  }
  if (servoEnabled[1] and (hbTimerCnt % 8 == 1))
  {
    int dw = servoRef[1] - servoValue[1];
    if (abs(dw) > servoVel[1] and servoVel[1] > 0)
    {
      if (dw > 0)
        dw = servoVel[1];
      else
        dw = -servoVel[1];
    }
    servoValue[1] += dw;
    int v = (servoValue[1] * msPulse/2) / 512 + midt;
    analogWrite(PIN_SERVO2, v);
  }
  if (servoEnabled[2] and (hbTimerCnt % 8 == 2))
  {
    int dw = servoRef[2] - servoValue[2];
    if (abs(dw) > servoVel[2] and servoVel[2] > 0)
    {
      if (dw > 0)
        dw = servoVel[2];
      else
        dw = -servoVel[2];
    }
    servoValue[2] += dw;
    int v = (servoValue[2] * msPulse/2) / 512 + midt;
    analogWrite(PIN_SERVO3, v);
  }
}

///////////////////////////////////////////////////////

void UServo::eePromSave()
{
  uint8_t flag = 0;
  // flags
  if (servo1isSteering)
    flag +=  1 << 0;
  eeConfig.pushByte(flag);
  eeConfig.pushWord(forwardOffset);
  eeConfig.pushFloat(distToSteerWheel);
  eeConfig.pushFloat(anglePer_ms);
}

void UServo::eePromLoad()
{
  if (eeConfig.isStringConfig())
  {
    eeConfig.skipAddr(1 + 2 + 4 + 4);
  }
  else
  {
    uint8_t flag;
    flag = eeConfig.readByte();
    // enabeled
    servo1isSteering = (flag & (1 << 0)) != 0;
    // offset
    forwardOffset = eeConfig.readWord();
    // distance to from wheel
    distToSteerWheel = eeConfig.readFloat();
    // angle servo turns when PW change from 1 to 2 ms
    anglePer_ms = eeConfig.readFloat();
  }
}


