/***************************************************************************
 *   Copyright (C) 2016 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 * Motor controller functions - controlling Pololu1213 dual motor controller
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
 
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <stdint.h>
#include "main.h"
#include "command.h"
#include "pins.h"

/**
 * Motor ankor voltage - assuming battery voltage is 12V */
//extern float batVoltFloat;
extern float motorAnkerVoltage[2];
// extern float max_motor_voltage;
extern int16_t motorAnkerPWM[2];
/**
 * Motor enable flages - not enabled means high impedanze for the motor. */
extern int8_t  motorEnable[2];
/** flag used to set calibrate motor current to zero */
extern bool motorPreEnabled;
extern bool motorPreEnabledRestart;
/**
 * Motor direction flag. */
extern bool    directionFWD[2];
/**
 * if motor gets full power in more than 1 second, then stop */
extern int overloadCount;
/// encoder
extern uint32_t encoder[2];
extern uint32_t encoderLast[2];
extern uint32_t encStartTime[2];
// test with better encoder values
extern uint32_t encStartTime_cpu[2];
const int MAX_ENC_VALS = 48;
extern uint32_t encTime_cpu[2][MAX_ENC_VALS * 2]; /// actual measured values from encoder (in ns)
extern uint32_t encTimeScale[2][MAX_ENC_VALS]; /// scale to compensate for uneven magnets in encoder (*1000)
extern bool encTimeScaleCalibrated; /// scale to compensate for uneven magnets in encoder is valid
extern bool encTimeTryCalibrate;
extern bool encTimeStopCollect;
extern bool encTimeCalibrateUse;
extern int encReindexBestIndex;
// test better encoder value end
// extern uint32_t  encPeriod10us[2];
extern uint32_t  encPeriod_cpu[2];
extern bool     encCCV[2];
// extern bool     encTimeOverload[2];
extern bool     encTimeOverload_cpu[2];

extern float motorCurrentA[2];
extern uint16_t motorCurrentM[2];
extern int32_t motorCurrentMOffset[2];
extern int32_t motorCurrentMLowPass[2];

/**
 * initialize motor port directions and sets PWM mode */
void motorInit();
/**
 * Reads motor status flag (overheat) */
bool motorError();
/**
 * Set motor speed - NB! one of the speed should change sign to run same way
 * \param m1PWM motor 1 speed in range +/- 1024
 * \param m2PWM motor 2 speed in range +/- 1024
 */
void motorSetAnkerPWM(int m1PWM, int m2PWM);
/**
 * Set motor anker voltage
 * The voltage is valid only if battery voltage is 12V
 * Else the voltage scales acordingly */
void motorSetAnchorVoltage();

/**
 * Set motor driver voltage
 * NB! can be set higher than 6V and will be implemented - up to supply (battVoltFloat) voltage.
 * \param left is new anchor voltage for left wheel
 * \param right is new anchor voltage for left wheel */
// inline void addMotorVoltage(float left, float right)
// {
//   motorAnkerVoltage[0] += left;
//   motorAnkerVoltage[1] += right;
// }

/**
 * Set motor driver voltage
 * NB! can be set higher than 6V and will be implemented - up to supply (battVoltFloat) voltage.
 * \param turnrate approximately turnrage in rad/sec positive is CCV
 */
// inline void addMotorTurnrate(float turnrate)
// {
//   float av = turnrate * 0.25;
//   motorAnkerVoltage[0] -= av;
//   motorAnkerVoltage[1] += av;
// }

/**
 * Enable one, two or no motors 
 * \param e1 is enable of motor 1 (0 is disable) - both motors for hardware 3
 * \param e2 is enable of motor 2 (0 is disable) - not used for hardware 3
 * */
void motorSetEnable(uint8_t e1, uint8_t e2);

/**
 * get motor current for motor 0 or 1 in amps.
 * NB! no test for valid index.
 * \returns current in amps */
inline float getMotorCurrentM(int m, int32_t value)
{ // sensor: 2.5V (5V/2) is zero and 185mV/A
  // offset to zero about 0.7V and still 185mV/A
  // A/D max=1.2V 10bit
  // measured value is upscaled with factor 300 to
  // improve accuracy with low-pass filter
  const float scale = 1.2 / lpFilteredMaxADC / 0.185 / 300.0 ; // 185 mV/A
  if (m == 0)
    return float(value - motorCurrentMOffset[0]) * scale;
  else
    // right motor runs backwards, so current is negative,
    // change sign so that forward requires positive current
    return -float(value - motorCurrentMOffset[1]) * scale;
}

/**
 * interrupt for motor encoder */
void m1EncoderA();
void m2EncoderA();
void m1EncoderB();
void m2EncoderB();

/**
 * Test if velocity is stable and good for encoder calibration 
 * If conditions are OK, then do the calibration.
 * \returns true if calibrated */
bool calibrateEncoderTest();

/**
 * Assumes velocity is constant and encoder is running forward at constant velocity
 * calculate average across 48 timing measurements (one revolution)
 * calculate factor (* 1000) for each cell to reach this average.
 * result is saved in factor array encTimeScale[2][48].
 * Motor must run at least 0.5 rotation per second (30 RPM or approx 1cm/sec) to avoid integer overload.
 * A good speed is anout 0.5m/s.
 * Calibration is not saved to ee-prom (yet) - must be set manually */
void calibrateEncoder();

/**
 * Attempts to correlate most recent measurement with calibration scale, by
 * trying all 48 offsets to find the one with the lowest resulting variance,
 * then the calibration values are offset to match the current encoder modulo.
 * \returns true if passed */
bool calibrateEncoderIndex();

/**
 * save encoder calibration values to flash (NB! takes about 200 bytes configuration space) */
void eePromSaveEncoderCalibrateInfo();
/**
 * load encoder calibration values - and schedule reindex of calibration values */
void eePromLoadEncoderCalibrateInfo();


#endif // MOTOR_CONTROLLER_H
