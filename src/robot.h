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
 ***************************************************************************/

#ifndef ROBOT_H
#define ROBOT_H

#include <stdlib.h>

const int MAX_ROBOT_NAMES = 90;

extern const char * robotname[MAX_ROBOT_NAMES];
/// distance traveled in m
extern float distance;
/// normal pose x,y,h,tilt in meter and radians
extern float pose[4];
/// wheel velocity in radians/s for each of the two wheels
/// the calculation is based on time between encoder pulses.
extern float wheelVelocity[2];
/// supplemented velocity with model bast estimator for control
/// in rad/s
extern float wheelVelocityEst[2];
/// should velocity at low speed be estimated (observer)
extern bool regul_vel_est;
/// wheel position in radians (since start of mission)
extern float wheelPosition[2];
/// turnrate in rad/s
extern float turnRate;
/// wheel base in m
extern float odoWheelBase;
/// radius of each wheel in m
extern float odoWheelRadius[2];
/// ballance offset for COG just above support (in radians as tilt value)
extern float balanceOffset;
/// use of LiPo battery requires an off (idle) voltage limit.
extern bool batteryUse;
/// voltage limit in Volts (go idle if below this limit) - converted to integer
extern uint16_t batteryIdleVoltageInt;
extern bool batteryOff;
extern uint16_t lowPassFactor;
// debug
extern float tiltu1;   // old value for complementary filter
extern float accAng;   // for debug
extern float gyroTiltRate; // radianer pr sekund
// end debug
/**
 * Read all sensors - should be called every 1 ms */
void readSensors();
/**
 * Update pose and wheel position and velocity 
 * \param loop is for debug printout only
 */
void updatePose(uint32_t loop);
/**
 * Update tilt estimate with complementary filter */
void estimateTilt();
/**
 * Clear odometry pose */
void clearPose();
/**
 * send status for (fixed) robot parameters and ID */
void sendStatusRobotID();
/// set robot ID and other robot specific parameters 
void setRobotID(const char * buf);
/**
 * clear historic part of vel estimator */
// void zeroVelEstimator();
/// save robot settings to ee prom
void eePromSaveRobotId();
/// load ID settings from ee-prom
void eePromLoadRobotId();

void batteryMonitoring();

void logIntervalChanged();

#endif
