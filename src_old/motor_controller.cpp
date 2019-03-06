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
#include "motor_controller.h"
#include <math.h>
#include <stdlib.h>
#include "robot.h"
#include "main.h"
#include "control.h"
#include "eeconfig.h"
#include "pins.h"
#include "data_logger.h"

#define MOTOR1   0
#define MOTOR2   1

// Variables
static volatile int   rotations[2] = {0};
float motorAnkerVoltage[2];
// float max_motor_voltage = 19;
int16_t motorAnkerPWM[2] = {0,0};
int8_t  motorEnable[2] = {0,0};
bool motorPreEnabled = true;
bool motorPreEnabledRestart = true;
/**
 * if motor gets full power in more than 1 second, then stop */
int overloadCount = 0;

/// encoder
uint32_t encoder[2];
uint32_t encoderLast[2] = {0,0};
// uint32_t encStartTime[2];
// uint32_t  encPeriod10us[2];
uint32_t  encPeriod_cpu[2];
bool     encCCV[2];
// bool     encTimeOverload[2];
bool     encTimeOverload_cpu[2];

// encoder calibration varables
uint32_t encStartTime_cpu[2]; /// last encoder tick CPU count (used during collect data for calibration)
uint32_t encTime_cpu[2][MAX_ENC_VALS * 2]; /// collected timing data (2 revolutions)
uint32_t encTimeScale[2][MAX_ENC_VALS]; /// calibration values * 1000
bool encTimeScaleCalibrated = false; /// calibrated values are usable (if use flag is true)
bool encTimeTryCalibrate = true; /// try calibrate or retry calibrate when speed is right
bool encTimeStopCollect = false; /// save a bit of time and values used for calibration
bool encTimeCalibrateUse = true; /// ability to not use a calibrated system, even if calibrated
bool encTimeTryReindex = false;  /// used after loading configuration from flash
int encReindexBestIndex = -1;

// test better encoder value end

float motorCurrentA[2];  // in amps
uint16_t motorCurrentM[2]; // external current sensor
int32_t motorCurrentMOffset[2];
uint8_t pinMotor2Dir = M2DIR1;
uint8_t pinMotor2Pwm = M2PWM1;

/**
 * Read motor current
 * \param motor 0 or 1 for motor 0 or 1.
 * \returns motor current in amps */

void motorInit(void)
{
  // Input
  pinMode(M1ENC_A,INPUT);
  pinMode(M1ENC_B,INPUT);
  pinMode(M2ENC_A,INPUT);
  pinMode(M2ENC_B,INPUT);
  // set PWM output mode and frequency
  analogWriteRes(10); /// resolution (10 bit)
  analogWriteFrequency(M1PWM, 20000); /// frequency (20kHz) motor 1 (and 2 for hardware 3)
  analogWriteFrequency(M2PWM1, 20000); // motor 2 - hardware < 3 only
  // find offset for motor current
  motorPreEnabled = true;
  // initialize encoder calibration values
  for (int m=0; m < 2; m++)
    for (int i=0; i< MAX_ENC_VALS; i++)
      encTimeScale[m][i] = 1000;
}


void motorSetAnchorVoltage()
{
  float batteryNominalVoltage = batVoltInt * batVoltIntToFloat;
  if (batteryNominalVoltage < 5.0)
    // not on battery - justfor test and avoid divide by zero
    batteryNominalVoltage = 11.1;
  const float max_pwm = 4096;
  float scaleFactor = max_pwm / batteryNominalVoltage;
  // limit motor voltage
//   // left motor
//   if (motorAnkerVoltage[0] > max_motor_voltage)
//   {
//     motorAnkerVoltage[0] = max_motor_voltage;
//     overloadCount++;
//   }
//   else if (motorAnkerVoltage[0] < -max_motor_voltage)
//   {
//     motorAnkerVoltage[0] = -max_motor_voltage;
//     overloadCount++;
//   }
//   else
//     overloadCount = 0;
//   // right motor
//   if (motorAnkerVoltage[1] > max_motor_voltage)
//   {
//     motorAnkerVoltage[1] = max_motor_voltage;
//     overloadCount++;
//   }
//   else if (motorAnkerVoltage[1] < -max_motor_voltage)
//   {
//     motorAnkerVoltage[1] = -max_motor_voltage;
//     overloadCount++;
//   }
//   else
//     overloadCount = 0;
  // overload check
  if (overloadCount > 500)
  { // disable motor (after 0.5 second)
    motorSetEnable(0, 0);
  }
  // debug
  if (true)
  { // log en extra logger set as item 1
    dataloggerExtra[0] = overloadCount;
  }
  // debug end
  // convert to PWM values (at 20khz)
  int w1, w2;
  w1 = int16_t(motorAnkerVoltage[0] * scaleFactor);
  // the right motor must run the other way
  w2 = int16_t(-motorAnkerVoltage[1] * scaleFactor);
  // implement
  motorSetAnkerPWM(w1, w2);
}

/**
 * e2 used on hardware < 3 only */
void motorSetEnable(uint8_t e1, uint8_t e2)
{
  if (motorPreEnabled and (e1 or e2) and not (motorEnable[0] or motorEnable[1]))
  { // switch off current zero offset calculation
//     const char MSL = 100;
//     char s[MSL];
    motorPreEnabled = false;
    motorPreEnabledRestart = true;
    // not needed logIntervalChanged();
//     snprintf(s, MSL, "# current A/D*300 offset  %ld %ld fac %d raw %d %d.\r\n", 
//              motorCurrentMOffset[0], motorCurrentMOffset[1], 
//              lowPassFactor,
//              motorCurrentM[0], motorCurrentM[1]
//             );
//     usb_send_str(s);
  }
  // reset overload
  if (e1 and not motorEnable[0])
    overloadCount = 0;
  // enable motors (or disable)
  motorEnable[0] = e1;
  motorEnable[1] = e2;
  //
  if (robotHWversion < 3 or robotHWversion == 5)
  {
    pinMode(SLEW,OUTPUT); // slewrate hight
    digitalWrite(SLEW,HIGH); // slewrate - always high
    pinMode(M1DIS1,OUTPUT); // disable motor 1
    pinMode(M1PWM,OUTPUT); // motor 1 PWM 
    pinMode(M1DIR,OUTPUT); // motor 1 direction
    pinMode(M2DIS1,OUTPUT); // disable motor 2
    pinMode(M2DIR1,OUTPUT); // direction (hardware < 3 only)
    // slow motor controller
    analogWriteFrequency(M2PWM1, 17000); // motor 2
    analogWriteFrequency(M1PWM, 17000); // motor 1
    pinMotor2Dir = M2DIR1;
    motorSetAnkerPWM(0,0);
    // write to motor controller pins
    digitalWrite(M1DIS1, motorEnable[0]); 
    digitalWrite(M2DIS1, motorEnable[1]);
  }
  else
  {
    pinMode(M1PWM,OUTPUT); // motor 1 PWM 
    pinMode(M1DIR,OUTPUT); // motor 1 direction
    pinMode(M12DIS,OUTPUT); // disable both motors pin
    pinMode(M2DIR3,OUTPUT); // direction (hardware 3 only) pin
    pinMode(M2PWM3,1); //OUTPUT); // PWM signal right motor (hardware 3 only) pin
    if (robotHWversion == 4)
      analogWriteFrequency(M2PWM3, 25000); // small motor controller (both)
    else
      analogWriteFrequency(M2PWM3, 17000); // big motor controller (both)
    pinMotor2Dir = M2DIR3;
    pinMotor2Pwm = M2PWM3;
    motorSetAnkerPWM(0,0);
    // write to motor controller pins
    digitalWrite(M12DIS, motorEnable[0]);
//     usb_send_str("# enabled motor HW 3\r\n");
  }
  
}

/** 
 * allowed input is +/- 2048, where 2048 is full battery voltage
 * */
void motorSetAnkerPWM(int m1PWM, int m2PWM)
{ // too small PWM will not implement
//   if (m1PWM > -55 && m1PWM < 55)
//     m1PWM = -0;
//   if (m2PWM > -55 && m2PWM < 55)
//     m2PWM = 0;
  const int max_pwm = 4095;
  const int pwmOffsetFwd = 50; // compensation for slow rise time
  const int pwmOffsetRev = 100; // motor controller around 12 V is a bit slower
  motorAnkerPWM[0] = m1PWM;
  motorAnkerPWM[1] = m2PWM;
  switch (robotHWversion)
  {
    case 1:
    case 2:
    case 3:
    case 5:
      // pololu 33926 dual controller (big controller with FB and overload detect)
      // motor 1 (left)
      if (m1PWM >= 0)
      { // put H-bridge side 2 to high
        digitalWrite(M1DIR, HIGH);
        if (m1PWM == 0)
          // make side 1 switch high
          analogWrite(M1PWM, max_pwm);
        else
          // make side 1 switch with low pulses down to fully low 
          analogWrite(M1PWM, max_pwm - m1PWM - pwmOffsetFwd);
      }
      else
      { // put H-bridge side 2 to low
        digitalWrite(M1DIR, LOW);
        // make side 1 switch with high pulses up to fully high 
        analogWrite(M1PWM, -m1PWM + pwmOffsetRev);
      }
      // motor 2 (rignt)
      if (m2PWM >= 0)
      { // put H-bridge side 2 to high
        digitalWrite(pinMotor2Dir, HIGH);
        if (m2PWM == 0)
          // make side 1 switch high
          analogWrite(pinMotor2Pwm, max_pwm);
        else
          // make side 1 switch with low pulses down to fully low 
          analogWrite(pinMotor2Pwm, max_pwm - m2PWM - pwmOffsetFwd);
      }
      else
      { // put H-bridge side 2 to low
        digitalWrite(pinMotor2Dir, LOW);
        // make side 1 switch with high pulses up to fully high 
        analogWrite(pinMotor2Pwm, -m2PWM + pwmOffsetRev);
      }
      break;
      //
    case 4:
    case 6:
      // pololu TB6612 dual motor controller (small controller)
      if (m1PWM >= 0)
      { // put H-bridge side 2 to high
        digitalWrite(M1DIR, LOW);
        // make side 1 switch with low pulses down to fully low 
        analogWrite(M1PWM, max_pwm - m1PWM);
      }
      else
      { // put H-bridge side 2 to low
        digitalWrite(M1DIR, HIGH);
        // make side 1 switch with high pulses up to fully high 
        analogWrite(M1PWM, max_pwm + m1PWM);
      }
      if (m2PWM >= 0)
      { // put H-bridge side 2 to high
        digitalWrite(pinMotor2Dir, LOW);
        // make side 1 switch with low pulses down to fully low 
        analogWrite(pinMotor2Pwm, max_pwm - m2PWM);
      }
      else
      { // put H-bridge side 2 to low
        digitalWrite(pinMotor2Dir, HIGH);
        // make side 1 switch with high pulses up to fully high 
        analogWrite(pinMotor2Pwm, max_pwm + m2PWM);
      }
      break;
    default:
      break;
  }
}


////////////////////////////////////////////////////////////

void m1EncoderA()
{ // motor 1 encoder A change
  // get timestamp now
  uint32_t ecpu = ARM_DWT_CYCCNT;
  uint8_t  a = digitalRead(M1ENC_A);
  uint32_t te;
  int idx;
  // read other channel for direction
  if (a == 0)
    encCCV[0] = digitalRead(M1ENC_B);
  else
    encCCV[0] = digitalRead(M1ENC_B) == 0;
  // ns method
  idx = encoder[0] % MAX_ENC_VALS;
  // get encoder tick time in centi-us - should allow up to ~40 sec values
  // when cpu clock is about 100 MHz.
  // Time since last tick in cpu clocks
  te = ecpu - encStartTime_cpu[0];
  // save to allow encoder calibration
  if (not encTimeStopCollect)
  {
    int idx2 = encoder[0] % (MAX_ENC_VALS * 2);
    encTime_cpu[0][idx2] = te;
  }
  // save start of next encoder period
  encStartTime_cpu[0] = ecpu;
//   if (encTimeScaleCalibrated)
  { // using CPU clock and magnet calibration
    if (encTimeOverload_cpu[0])
    {
      encPeriod_cpu[0] = 0;
      encTimeOverload_cpu[0] = false;
    }
    else
    { // valid timing
      if (encTimeCalibrateUse)
      { // adjust time using calibration values
        // scale is multiplied by 1000
        if (te > (F_CPU / 100))
          // long time > ~10ms
          // integer operation with divide first to avoid overflow
          te = (te / 1000) * encTimeScale[0][idx];
        else
          // divide last to avoid loosing accuracy
          te = (te * encTimeScale[0][idx]) / 1000;
      }
      if (te < (F_CPU / 1000000 * 250))
        // shorter than 250 us between ticks
        // average over 4 samples
        encPeriod_cpu[0] = (encPeriod_cpu[0] * 3 + te) / 4;
      else if (te < (F_CPU / 1000000 * 500))
        // shorter than 0.5 ms average over 2 samples
        encPeriod_cpu[0] = (encPeriod_cpu[0] + te) / 2;
      else
        // longer than 0.5ms, use timing as is
        encPeriod_cpu[0] = te;
    }
  }
//   else
//   { // old method - uncalibrated based on 10us clock
//     uint32_t e = hb03us;
//     uint32_t dt = e - encStartTime[0];
//     encStartTime[0] = e;
//     if (encTimeOverload[0])
//     { // too long time to count
//       encPeriod10us[0] = 0;
//       encTimeOverload[0] = false;
//     }
//     else if (dt < CONTROL_PERIOD_10us/2)
//     { // We are running fast, more than two enc pulse per 
//       // sample-time
//       encPeriod10us[0] = (encPeriod10us[0]*3 + dt)/4 ;
//     }
//     else if (dt < CONTROL_PERIOD_10us)
//     { // We are running fast, more than one enc pulse per 
//       // sample-time
//       encPeriod10us[0] = (encPeriod10us[0] + dt) / 2 ;
//     }
//     else
//     { // period acceptable - longer than sample time and shorter than 2.5 seconds
//       encPeriod10us[0] = dt;
//     }
//   }
  if (encCCV[0])
    encoder[0]--;
  else
    encoder[0]++;
}

//////////////////////////////////////////////////////////////

void m2EncoderA()
{ // motor 2
  // get timestamp now
  uint32_t ecpu = ARM_DWT_CYCCNT;
  uint8_t  a = digitalRead(M2ENC_A);
  uint32_t te;
  int idx;
  // read other channel for direction
  if (a)
    encCCV[1] = digitalRead(M2ENC_B);
  else
    encCCV[1] = digitalRead(M2ENC_B) == 0;
  // velocity baset on encoder period
  // ns method
  idx = encoder[1] % MAX_ENC_VALS;
  // get encoder tick time in centi-us - should allow up to ~40 sec values
  // when cpu clock is about 100 MHz.
  // Time since last tick in cpu clocks
  te = ecpu - encStartTime_cpu[1];
  // save to allow encoder calibration
  if (not encTimeStopCollect)
  {
    int idx2 = encoder[1] % (MAX_ENC_VALS * 2);
    encTime_cpu[1][idx2] = te;
  }
  // save start of next encoder period
  encStartTime_cpu[1] = ecpu;
//   if (encTimeScaleCalibrated)
  { // using CPU clock and magnet calibration
    if (encTimeOverload_cpu[1])
    {
      encPeriod_cpu[1] = 0;
      encTimeOverload_cpu[0] = false;
    }
    else
    { // valid timing
      if (encTimeCalibrateUse)
      { // adjust time using calibration values
        // scale is multiplied by 1000
        if (te > (F_CPU / 100))
          // long time > ~10ms
          // integer operation with divide first to avoid overflow
          te = (te / 1000) * encTimeScale[1][idx];
        else
          // divide last to avoid loosing accuracy
          te = (te * encTimeScale[1][idx]) / 1000;
      }
      if (te < (F_CPU / 1000000 * 250))
        // shorter than 250 us between ticks
        // average over 4 samples
        encPeriod_cpu[1] = (encPeriod_cpu[1] * 3 + te) / 4;
      else if (te < (F_CPU / 1000000 * 500))
        // shorter than 0.5 ms average over 2 samples
        encPeriod_cpu[1] = (encPeriod_cpu[1] + te) / 2;
      else
        // longer than 0.5ms, use timing as is
        encPeriod_cpu[1] = te;
    }
  }
//   else
//   { // old method
//     uint32_t e = hb03us;
//     uint32_t dt = e - encStartTime[1];
//     encStartTime[1] = e;
//     if (encTimeOverload[1])
//     { // too long time to count
//       encPeriod10us[1] = 0;
//       encTimeOverload[1] = false;
//     }
//     else if (dt < CONTROL_PERIOD_10us/2)
//     { // We are running fast, more than two enc pulse per 
//       // control period, so average over about 4 samples
//       encPeriod10us[1] = (encPeriod10us[1]*3 + dt)/4 ;
//     }
//     else if (dt < CONTROL_PERIOD_10us)
//     { // We are running fast, more than one enc pulse per 
//       // control period, so average over about 2 samples
//       encPeriod10us[1] = (encPeriod10us[1] + dt) / 2 ;
//     }
//     else
//     { // period acceptable - less than 2.5 seconds and more than 
//       // one control period
//       encPeriod10us[1] = dt;
//     }
//   }
  if (encCCV[1])
    encoder[1]--;
  else
    encoder[1]++;
}

void m1EncoderB()
{ // motor 1 encoder pin B
  // get timestamp now
  uint32_t ecpu = ARM_DWT_CYCCNT;
  uint8_t  b = digitalRead(M1ENC_B);
  uint32_t te;
  int idx;
  // read other channel for direction
  if (b)
    encCCV[0] = digitalRead(M1ENC_A);
  else
    encCCV[0] = digitalRead(M1ENC_A) == 0;
  // velocity baset on encoder period
  // ns method
  idx = encoder[0] % MAX_ENC_VALS;
  // get encoder tick time in centi-us - should allow up to ~40 sec values
  // when cpu clock is about 100 MHz.
  // Time since last tick in cpu clocks
  te = ecpu - encStartTime_cpu[0];
  // save to allow encoder calibration
  if (not encTimeStopCollect)
  {
    int idx2 = encoder[0] % (MAX_ENC_VALS * 2);
    encTime_cpu[0][idx2] = te;
  }
  // save start of next encoder period
  encStartTime_cpu[0] = ecpu;
  //
//   if (encTimeScaleCalibrated)
  { // using CPU clock and magnet calibration
    if (encTimeOverload_cpu[0])
    {
      encPeriod_cpu[0] = 0;
      encTimeOverload_cpu[0] = false;
    }
    else
    { // valid timing
      if (encTimeCalibrateUse)
      { // adjust time using calibration values
        // scale is multiplied by 1000
        if (te > (F_CPU / 100))
          // long time > ~10ms
          // integer operation with divide first to avoid overflow
          te = (te / 1000) * encTimeScale[0][idx];
        else
          // divide last to avoid loosing accuracy
          te = (te * encTimeScale[0][idx]) / 1000;
      }
      if (te < (F_CPU / 1000000 * 250))
        // shorter than 250 us between ticks
        // average over 4 samples
        encPeriod_cpu[0] = (encPeriod_cpu[0] * 3 + te) / 4;
      else if (te < (F_CPU / 1000000 * 500))
        // shorter than 0.5 ms average over 2 samples
        encPeriod_cpu[0] = (encPeriod_cpu[0] + te) / 2;
      else
        // longer than 0.5ms, use timing as is
        encPeriod_cpu[0] = te;
    }
  }
//   else
//   { // old method
//     uint32_t e = hb03us;
//     uint32_t dt = e - encStartTime[0];
//     encStartTime[0] = e;
//     if (encTimeOverload[0])
//     { // too long time to count
//       encPeriod10us[0] = 0;
//       encTimeOverload[0] = false;
//     }
//     else if (dt < CONTROL_PERIOD_10us/2)
//     { // We are running fast, more than two enc pulse per 
//       // control period, so average over about 4 samples
//       encPeriod10us[0] = (encPeriod10us[0]*3 + dt) / 4 ;
//     }
//     else if (dt < CONTROL_PERIOD_10us)
//     { // We are running fast, more than one enc pulse per 
//       // control period, so average over about 2 samples
//       encPeriod10us[0] = (encPeriod10us[0] + dt) / 2 ;
//     }
//     else
//     { // period acceptable - less than 2.5 seconds and more than 
//       // one control period
//       encPeriod10us[0] = dt;
//     }
//   }
  if (encCCV[0])
    encoder[0]--;
  else
    encoder[0]++;
}

void m2EncoderB()
{ // motor 2 encoder pin B
  // get timestamp now
  uint32_t ecpu = ARM_DWT_CYCCNT;
  uint8_t  b = digitalRead(M2ENC_B);
  uint32_t te;
  // read other channel for direction
  if (b == 0)
    encCCV[1] = digitalRead(M2ENC_A);
  else
    encCCV[1] = digitalRead(M2ENC_A) == 0;
  // velocity baset on encoder period
  // encoder magnet index
  int idx = encoder[1] % MAX_ENC_VALS;
  // get encoder tick time in centi-us - should allow up to ~40 sec values
  // when cpu clock is about 100 MHz.
  // Time since last tick in cpu clocks
  te = ecpu - encStartTime_cpu[1];
  // save to allow encoder calibration
  if (not encTimeStopCollect)
  {
    int idx2 = encoder[1] % (MAX_ENC_VALS * 2);
    encTime_cpu[1][idx2] = te;
  }
  // save start of next encoder period
  encStartTime_cpu[1] = ecpu;
//   if (encTimeScaleCalibrated)
  { // using CPU clock and magnet calibration
    if (encTimeOverload_cpu[1])
    {
      encPeriod_cpu[1] = 0;
      encTimeOverload_cpu[1] = false;
    }
    else
    { // valid timing
      if (encTimeCalibrateUse)
      { // adjust time using calibration values
        // scale is multiplied by 1000
        if (te > (F_CPU / 100))
          // long time > ~10ms
          // integer operation with divide first to avoid overflow
          te = (te / 1000) * encTimeScale[1][idx];
        else
          // divide last to avoid loosing accuracy
          te = (te * encTimeScale[1][idx]) / 1000;
      }
      if (te < (F_CPU / 1000000 * 250))
        // shorter than 250 us between ticks
        // average over 4 samples
        encPeriod_cpu[1] = (encPeriod_cpu[1] * 3 + te) / 4;
      else if (te < (F_CPU / 1000000 * 500))
        // shorter than 0.5 ms average over 2 samples
        encPeriod_cpu[1] = (encPeriod_cpu[1] + te) / 2;
      else
        // longer than 0.5ms, use timing as is
        encPeriod_cpu[1] = te;
    }
  }
//   else
//   { // old method
//     uint32_t e = hb03us;
//     uint32_t dt = e - encStartTime[1];
//     encStartTime[1] = e;
//     if (encTimeOverload[1])
//     { // too long time to count
//       encPeriod10us[1] = 0;
//       encTimeOverload[1] = false;
//     }
//     else if (dt < CONTROL_PERIOD_10us/2)
//     { // We are running fast, more than two enc pulse per 
//       // control period, so average over about 4 samples
//       encPeriod10us[1] = (encPeriod10us[1]*3 + dt)/4 ;
//     }
//     else if (dt < CONTROL_PERIOD_10us)
//     { // We are running fast, more than one enc pulse per 
//       // control period, so average over about 2 samples
//       encPeriod10us[1] = (encPeriod10us[1] + dt) / 2 ;
//     }
//     else
//     { // period acceptable - less than 2.5 seconds and more than 
//       // one control period
//       encPeriod10us[1] = dt;
//     }
//   }
  // 
  if (encCCV[1])
    encoder[1]--;
  else
    encoder[1]++;
}

/////////////////////////////////////////////

uint32_t encoderTimeAverage(int motoridx)
{
  uint32_t avg = 0;
  for (int i = 0; i < MAX_ENC_VALS; i++)
    avg += encTime_cpu[motoridx][i];
  avg /= MAX_ENC_VALS;
  return avg;
}

///////////////////////////////////////////////

/** velocity history for stability test */
float velHist[2] = {0,0};
int velHistWait = 100;


bool calibrateEncoderTest()
{
  velHist[0] = velHist[0] * 0.95 + wheelVelocityEst[0] * 0.05;
  velHist[1] = velHist[1] * 0.95 + wheelVelocityEst[1] * 0.05;
  //
  if (velHistWait == 0)
  {
    if (fabsf(wheelPosition[0]) > 0.05 and
        fabsf(wheelPosition[1]) > 0.05 and 
      fabsf(wheelVelocityEst[0]) > 0.2 and
      fabsf(wheelVelocityEst[1]) > 0.2 and
      not balance_active and
      fabsf(velHist[0] - wheelVelocityEst[0]) < 0.1 and
      fabsf(velHist[1] - wheelVelocityEst[1]) < 0.1
      )
    { // velocity high and stable
      bool isOK = true;
      for (int m = 0; m < 2; m++)
      {
        uint32_t vsum = 0;  
        uint32_t difSum = 0;
        for (int i = 0; i < MAX_ENC_VALS; i++)
        {
          uint32_t ti = encTime_cpu[m][i];
          vsum += ti;
          difSum += abs(int(ti) - int(encTime_cpu[m][i + MAX_ENC_VALS]));
        }
        isOK = difSum < (vsum / (5 * MAX_ENC_VALS));
        const int MSL = 100;
        char s[MSL];
        if (false)
        {
          snprintf(s, MSL, "#enc test ok=%d, vsum=%lu, difsum=%lu\n", 
                  isOK, vsum, difSum);
          usb_send_str(s);
        }
        if (not isOK)
          break;
      }
      if (isOK)
      {
        if (encTimeTryReindex)
          calibrateEncoderIndex();
        else
          calibrateEncoder();
        usb_send_str("# motor_controller::calibrateEncoderTest: calibrate encoder succeeded\n");
      }
//       else
//         usb_send_str("# motor_controller::calibrateEncoderTest: failed\n");
    }
    // wait for new values
    velHistWait = 20;
  }
  else
    velHistWait--;
  return encTimeScaleCalibrated;
}


void calibrateEncoder()
{ // assuming velocity is constant and encoder is running forward at constant velocity
  // - calculate average across 2 x 48 timing measurements (two revolutons)
  // - calculate factor (* 1000) for each cell to reach this average, i.e.
  // - normalized over the average over the dataset (2x48 measurements)
  // stop collecting data
  encTimeStopCollect = true;
  // calculate compensation factors
  for (int m = 0; m < 2; m++)
  {
    uint32_t avg = encoderTimeAverage(m);
    for (int i = 0; i < MAX_ENC_VALS; i++)
    { // factor values are scaled a factor 1000 (over 2x48 periods, so 2000)
      encTimeScale[m][i] = (avg * 2000) /
             (encTime_cpu[m][i] + encTime_cpu[m][i + MAX_ENC_VALS]);
    }
  }
  encTimeScaleCalibrated = true;
  // restart collecting data
  encTimeStopCollect = false;
}

////////////////////////////////////////////////////


bool calibrateEncoderIndex()
{
  bool result = false;
  if (encTimeScaleCalibrated)
  { // find variation
    // stop collecting data
    encTimeStopCollect = true;
    for (int m = 0; m < 2; m++)
    { // for both motors
      float bestVar = 2000000000;
      float va[MAX_ENC_VALS] = {0}; // should be changed to just one value
      float va1, va2;
      bzero(va, sizeof(va));
      for (int i = 0; i < MAX_ENC_VALS; i++)
      { // for this calibration index i, 
        // find first encoder tick timing value, when compensated using calibration index i
        va1 = (encTimeScale[m][i] * encTime_cpu[m][0]) / 1000.0;
        // initialize previous value
        va2 = va1;
        for (int j = 1; j < MAX_ENC_VALS/5; j++)
        { // // find encoder tick j timing value, when compensated using calibration index i
          float v = (encTimeScale[m][(j + i) % MAX_ENC_VALS] * encTime_cpu[m][j]) / 1000.0;
          // sum the difference in (calibrated) timing from one encoder tick to the next 
          va[i] += fabsf(v - va2);
          // save the calibrated timing from this encoder tick
          va2 = v;
        }
        // add also the difference from last tick to first tick
        va[i] += fabsf(va1 - va2);
        // the timing sum from all encoder ticks should be best when calibration is right
        if (va[i] < bestVar)
        { // this calibration index is better
          encReindexBestIndex = i;
          bestVar = va[i];
        }
      }
      if (true)
      { // debug
        const int MSL = 680;
        char s[MSL];
        uint32_t avgtimu = encoderTimeAverage(m);
        float avgErr = float(bestVar) / MAX_ENC_VALS * 1000000.0 / float(F_CPU);
        float avgtim = float(avgtimu) * 1000000.0/ float(F_CPU); // to us
        snprintf(s, MSL, "#for motor %d is index %d avg %g, avg err %g, best var %g, va1 %g, va2 %g\n#", 
                 m, encReindexBestIndex, avgtim, avgErr, bestVar, va1, va2);
        int n = strlen(s);
        char * p1 = &s[n];
        for (int i = 0; i < MAX_ENC_VALS; i++)
        { // get all summs
          snprintf(p1, MSL-n, " %g", va[i]);
          n += strlen(p1);
          p1 = &s[n];
        }
        snprintf(p1, MSL-n, "\n");
        usb_send_str(s);
      }
      if (encReindexBestIndex > 0 and va[encReindexBestIndex] > 10)
      { // a new index is better - and value is greater than nothing
        result = true;
        uint32_t d[MAX_ENC_VALS];
        // save old order of calibration values
        memmove(d, encTimeScale[m], sizeof(uint32_t)*MAX_ENC_VALS);
        for (int i = 0; i < MAX_ENC_VALS; i++)
        { // move to new position
          encTimeScale[m][i] = d[(i + encReindexBestIndex) % MAX_ENC_VALS];
        }
      }
      else if (encReindexBestIndex == 0)
      {
        usb_send_str("# best idx is still 0\n");
        result = true;
        // debug
//         const int MSL = 180;
//         char s[MSL];
//         snprintf(s, MSL, "#for motor %d is index %d with sd=%g (idx=1 sd=%g)\n", m, encReindexBestIndex, sqrt(va[encReindexBestIndex]/MAX_ENC_VALS), sqrt(va[1]/MAX_ENC_VALS));
//         usb_send_str(s);
        // debug end
      }
      else
      { // at least one of the motors failed
        usb_send_str("# calibrateEncoderIndex failed\n");
        result = false;
        break;
      }
    }
    // restart data capture
    encTimeStopCollect = false;
    // one reindex should be enough
    encTimeTryReindex = false;
  }
  return result;
}

void eePromSaveEncoderCalibrateInfo()
{
  char v = 0x00;
  if (encTimeTryCalibrate)
    v |= 0x01;
  if (encTimeCalibrateUse)
    v |= 0x02;
  if (encTimeScaleCalibrated)
    v |= 0x04;
  eeConfig.pushByte(v);
  if (encTimeScaleCalibrated)
  { // save number of factors
    eeConfig.pushByte(MAX_ENC_VALS);
    // save all factors
    for (int m = 0; m < 2; m++)
    { // both motors
      for (int i = 0; i < MAX_ENC_VALS; i++)
      {
        eeConfig.pushWord(uint16_t(encTimeScale[m][i]));
      }
    }
  }
}


void eePromLoadEncoderCalibrateInfo()
{
  char v = eeConfig.readByte();
//  char v = eeprom_read_byte((uint8_t*)eePushAdr++);
  bool calibrated = (v & 0x04) == 0x04;
  // number of bytes to skip if not robot-specific configuration
  int skipCount = 0;
  int encTickCnt = 0;
  if (calibrated)
  {
    encTickCnt = eeConfig.readByte();
    // calibration values are not relevant if string configuration
    // 2 motors, 2 byte each value
    skipCount = 2 * encTickCnt * 2;
  }
  if (not eeConfig.isStringConfig())
  { // load from flash
    encTimeTryCalibrate = (v & 0x01) == 0x01;
    // NB!
    // do not use calibrated values at boot, as it is worse than nothing when not in sync with encoder
    encTimeCalibrateUse = false; // (v & 0x02) == 0x02;
    //
    encTimeTryReindex = calibrated;
    encTimeScaleCalibrated = calibrated;
    if (calibrated)
    { // read calibration values for both motors
      for (int m = 0; m < 2; m++)
      { // both motors
        for (int i = 0; i < encTickCnt; i++)
        { // read all calibration values
          uint16_t v = eeConfig.readWord();
          if (i < MAX_ENC_VALS)
            // do not write outside array - if size has changed
            encTimeScale[m][i] = v;
          else
          { // data size changet, not usable values
            encTimeTryReindex = false;
          }
        }
      }
    }
  }
  else
    // load from hard-coded mission
    eeConfig.skipAddr(skipCount);
}
