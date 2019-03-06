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
 
#ifndef SERVO_ON_REGBOT_H
#define SERVO_ON_REGBOT_H

#include <stdint.h>
#include "main.h"
#include "command.h"
#include "pins.h"

class UServo
{
public:
  /** 
   * constructor (niit) */
  UServo();
  /** else disabled (no pulse) */
  bool servoEnabled[3];
  /** last commanded value */
  int16_t servoValue[3];
  int16_t servoVel[3];
  int16_t servoRef[3];
  bool    servoEnaRef[3];
  /** servoboard pin - is output */
  bool pinIsOutput[2];
  /** servoboard pin - set value if output and read value if input
   * pin 4 is analog, pin 5 is digital */
  int16_t pin4Value;
  bool    pin5Value;
  /** PWM frequency (400 Hz is probably maximum) */
  static const int PWMfrq = 333; // Hz - 3ms update rate
  /// is servo 1 steered front wheel
  bool servo1isSteering;
  /**
  * set PWM port of frekvens */
  void initServo();
  /**
  * Set servo 1 PWM value
  * \param pwm allowed input is +/- 512, where
  * 0 is center (1.5ms)
  *  \param enable if false, then PWM pulse is stopped (servo disables),
  * but port is still an output port
  * \param vel is max velocity for servo 0=no limit 1 is slow 2 faster, ~10 is full speed.
  * */  
  inline void setServo1PWM(int16_t pwm, bool enable, int8_t vel)
  {
    servoRef[0] = pwm;
    servoEnaRef[0] = enable;
//     if (vel >= 0)
      servoVel[0] = vel;
//     if (not enable)
//       usb_send_str("# setServo1PWM off\n");
  }
  /**
   * Set servo 2 PWM value
   * \param pwm allowed input is +/- 512, where
   * 0 is center (1.5ms)
   *  \param enable if false, then PWM pulse is stopped (servo disables),
   * but port is still an output port
   * \param vel is max velocity for servo 0=no limit 1 is slow 2 faster, ~5 is full speed.
   * */  
  inline void setServo2PWM(int16_t pwm, bool enable, int8_t vel)
  {
    servoRef[1] = pwm;
    servoEnaRef[1] = enable;
//     if (vel >= 0)
      servoVel[1] = vel;
//     if (not enable)
//       usb_send_str("# setServo2PWM off\n");
  }
  /**
   * Set servo 3 PWM value
   * \param pwm allowed input is +/- 512, where
   * 0 is center (1.5ms)
   *  \param enable if false, then PWM pulse is stopped (servo disables),
   * but port is still an output port
   * \param vel is max velocity for servo 0=no limit 1 is slow 2 faster, ~5 is full speed.
   * */  
  inline void setServo3PWM(int16_t pwm, bool enable, uint8_t vel)
  {
    servoRef[2] = pwm;
    servoEnaRef[2] = enable;
//     if (vel >= 0)
    servoVel[2] = vel;
//     if (not enable)
//       usb_send_str("# setServo3PWM off\n");
  }
  /** 
   * \param pin allowed pin is 0,1. 
   * \param value input true is 1
   * \param enable if false, then port in set as input
   * */
  void setServoPin(int8_t pin, int16_t value, bool enable);
  /**
   * set any servo or IO pin/value */
  void setServo(int8_t idx, int16_t value, bool enable, int8_t vel);
  /**
   * stop PWM to servos and set servo pins to input */
  void releaseServos();
  /**
   * make servo 1 act as steering */
  float setServoSteering();
  /**
   * send servo status to client */
  void sendServoStatus();
  /**
   * send status for steering using servo 1 
   *  servo 1 as steering wheel
   *  format: 'sv1' use offset dist angle
   *  e.g.: sv1 1 0 0.135 90.0
   *  use is a boolean - activate steering
   *  offset is value +/- 512 for straight forward (close to 0)
   *  dist is distance to steering wheel
   *  angle is servo angle from 1ms to 2ms
   * */
  void sendServo1Status();
  /**
   * set servo values from string */
  void setServoConfig(const char * line);
  /**
   *  servo 1 as steering wheel - settings from command line
   *  format: 'sv1' use offset dist angle
   *  e.g.: sv1 1 0 0.135 90.0
   *  use is a boolean - activate steering
   *  offset is value +/- 512 for straight forward (close to 0)
   *  dist is distance to steering wheel
   *  angle is servo angle from 1ms to 2ms
   * */
  void setServo1Config(const char * line);
  /**
   * set one servo or pin (mostly for debug) */
  void setOneServo(const char * line);
  /**
   * 5ms update of servo */
  void servoTick();
  
  /**
   * save steering configuration to (EE)disk */
  void eePromSave();
  /**
   * load configuration from EE-prom */
  void eePromLoad();
  
  
private:
  // 1ms = frq/12bit
  static const int max_pwm = 4096;
  /// pwm value to give 1ms
  static const int msPulse = (max_pwm * PWMfrq) / 1000;
  /// center position (1.5ms)
  static const int midt = (msPulse * 3) / 2;
  /// distance to front wheel (from drive wheels) - positive is front
  float distToSteerWheel;
  /// value for straight forward (nominal 0, range +/-512, positive is left
  int16_t forwardOffset;
  /// angle turned for 1ms change (from 1 to 2 ms) in degrees
  float anglePer_ms;
};

#endif // MOTOR_CONTROLLER_H
