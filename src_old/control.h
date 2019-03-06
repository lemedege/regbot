 /***************************************************************************
 * definition file for the regbot.
 * The main function is the controlTick, that is called 
 * at a constant time interval (of probably 1.25 ms, see the intervalT variable)
 * 
 * The main control functions are in control.cpp file
 * 
 *   Copyright (C) 2014 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
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

#ifndef REGBOT_CONTROL_H
#define REGBOT_CONTROL_H

#include <stdint.h>
#include "main.h"
#include "mission.h" 
#include "controlbase.h"
#include "controlturn.h"
 
extern const int missionMax;
extern int8_t missionLineNum;
extern float mission_vel_ref;
extern float mission_turn_ref;
extern bool  mission_line_LeftEdge;
extern bool mission_wall_turn;
extern int8_t mission_irSensor_use;
extern float mission_line_ref;
extern float mission_turn_radius;
extern float mission_wall_ref;
extern float mission_wall_vel_ref;
extern bool mission_turn_do;
extern float mission_tr_turn;
extern UMissionLine * misLine;
extern uint8_t misThread;
extern bool mission_pos_use;    // should position regulator be used
extern float mission_pos_ref;    // should position regulator be used
extern float misStartDist;         // start distance for this line
extern bool balance_active;
extern bool regul_line_use;  // use heading control from line sensor
// debug values for controller
extern float regVelELeft[], regVelUILeft[], regVelUDLeft[];
/// remote control variables
extern bool remoteControl;
extern float remoteControlVel; /// in m/s
extern float remoteControlVelDif; /// in m/s
extern bool remoteControlNewValue;


class UControl
{
public:
  /**
   * constructor */
  UControl();
  /**
   * send status about regulators 
   * \param line is a line received from client, that may be 
   * setting of a controller.
   * \returns true if line is used */  
  bool setRegulator(const char * line);
  /**
   * Initialize input and output of controllers */
  void setRegulatorInOut();
  /**
   * Send control status for one controller */
  bool sendStatusControl(const char * line);
  /**
   * save controller configuration to EE Prom */
  void eePromSaveCtrl();
  /**
   * load controller configuration from EE Prom
   * same order as when saved
   */
  void eePromLoadCtrl();
  /**
   * Reset all controllers, i.e. set all passed values to zero. */
  void resetControl();
  /**
   * Stop turning */
  void resetTurn();
  /**
   * Set remote control parameters 
   * \param enable shift to RC control if true
   * \param vel is velocity reference in m/s
   * \param velDif is difference in wheel speed when turning
   * \param balance should robot try to acheive balance
   * */
  void setRemoteControl(bool enable, float vel, float velDif, bool balance);
  
  /**
   * This function is called every time it is time to do a new control calculation
   * */
  void controlTick(void); 
  /**
   * set rate limit i.e. acceleration limit (m/s^2) */
  inline void setRateLimit(float limit)
  {
    rateLimitUse = limit < 100.0;
    rateLimit = fabs(limit);
  }
  /**
   * mission start state for hard coded missions */
  bool mission_hard_coded();
  /**
   * state update for user mission */
  void user_mission();
  /**
   * send mission status if any of the data has changed
   * \param forced send status anyhow - changed or not. */
  void sendMissionStatusChanged(bool forced);
  /**
   * is rate limit used - e.g. to stop integrator windup */
  inline bool isRateLimitUsed()
  { // eg for 
    return rateLimitUsed;
  }
  /**
   * Zeroset chirp variables, and set number of
   * logged values every full circle (360 degrees) 
   * \param logInterval is set from normal log interval,
   * but with chirp active is changes to samples per chirp period (of 360 deg) */
  void chirpReset(int logInterval);
  /**
   * Start chirp now, from highest frequency, as set with log interval.
   * This is called from mission line (first time chirp is found)
   * \param chirp is the amplitude of the chirp in cm/s or deci-radians (both 1..255) (0 is off) 
   * \param useHeading if false, then control velocity */
  void chirpStart(uint8_t chirp, bool useHeading);

public:  
  UControlBase * ctrlVelLeft;
  UControlBase * ctrlVelRight;
  UControlTurn * ctrlTurn;
  UControlBase * ctrlWallVel;
  UControlWallTurn * ctrlWallTurn;
  UControlPos * ctrlPos;
  UControlEdge * ctrlEdge;
  UControlBase * ctrlBal;
  UControlBalVel * ctrlBalVel;  
  UControlPos * ctrlBalPos;  
private:
  /// 
  /// adjusted and interim reference values
  /// other than those set by mission
//   float vel_ref_add_wall; // 0=left 1=right 
  float vel_ref_red_turn[2]; // 0=left 1=right
  float ctrl_vel_ref; // velocity ref for position control
  float ctrl_turn_ref; // turn ref used in for control input
  float balanceTiltRef; // reference tilt to balance regulator (from balance velocity)
  float balanceVelRef; // reference velocity as determined by balance controller
  /** rate limited wheel velocity - except for balance */
  bool rateLimitUse;
  bool rateLimitUsed;
  // mission control variables
  int16_t missionStateLast;
  uint8_t misThreadLast;
  //
  int tick;
  /// mission end time (theEnd got true)
  float endTime;
  ///
  bool savedBalanceActive = false;
  bool savedControlActive = false;
  /// frequency scale each sec (or ms?)
  double chirpFrqRate = 0; // scale factor per ms
  /// current chirp quadrant
  uint8_t chirpLogQuadrant;
  /// how many "quadrants" should be logged every revolution
  uint8_t chirpLogInterval;
  /// chirp destination
  /// 0 is velocity, 1 is heading
  uint8_t chirpDestination;

public:
  // used by data logger
  float vel_ref_pre_bal[2]; // reference to wheel velocity before ballance adjust
  float vel_ref[2]; // reference to wheel velocity controller
  float rateLimit; // current acceleration limit
  /// mission to start (with start or button)
  int mission;
  int16_t missionState;
  bool controlActive;
  // chirp settings - 
  bool  chirpRun = false;
  // amplitude in m/s (velocity) or radians (turn)
  float chirpAmplitude = 0;
  /// log flag - set by chirp, reset by log function
  bool chirpLog;
  /// current chirp frequency
  double chirpFrq = 0;
  /// current phase angle
  double chirpAngle = 0;
  /// current chirp value (amplitude at this time)
  float chirpValue;
  // do not go lower than this frequency
  float chirpMinimumFrequency;
  // chirp amplitude limitation
  /**
   * for every cycle find max motor voltage.
   * should limit chirp amplitude to reduce motor voltage
   * limit is hard coded to 6V */
  float chirpMotorVoltMax = 0;
  /**
   * for every cycle find max motor velocity.
   * should limit chirp amplitude to reduce velocity
   * limit is hard coded to 1m/s */
  float chirpVelMax = 0;
  /**
   * for every cycle find max gyro turnrate amplitude.
   * assumed to increase with falling frequency, and
   * therefore used to decrease chirp amplitude accordingly.
   * limit is hard-coded */
//   float chirpGyroTurnrateMax = 0;
//   float chirpGyroTurnrateMax2 = 0;
};

#endif
