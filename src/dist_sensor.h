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

//#define IR13_50CM 1

/**
 * Raw sensor data from 2 IR sensors (Sharp 2Y0A21 F) */
extern uint32_t irRaw[2];
extern uint16_t irRawAD[2];
extern float irA[2];
extern float irB[2];

/**
 * Calculated sensor distance in meters */
extern float irDistance[2];
/**
 * should time be spend on dist sensor calculation */
extern bool useDistSensor;
/**
 * Is dist sensor installed on this robot */
extern bool distSensorInstalled;

/**
 * Send a "dip" message */
void sendStatusDistIR();

/**
 * Sets the two calibrate values for each IR sensor
 * "irc 20cm ir1, 80cm ir1, 20cm ir2, 80cm ir2", like
 * "irc 3011 480 2990 480"
 * \param buf line with calibrate values
 * \returns true if buffer was used
 *  */
bool setIrCalibrate(const char * buf);

/**
 * Estimate distance in meters
 * uses straight hyperbolic estimate
 * source irRaw 
 * irDistance = irA + irB/irRaw;
 * where irA and irB is calibration values:
 * irA = 0.2 (irCal20cm - 4 * irCal80cm) / (irCal20cm - irCal80cm)
 * irB = 0.6 * irCal20cm * irCal80cm / (irCal20cm - irCal80cm)
 * \returns value in irDistance */
void estimateIrDistance();
/**
 * set power on/off to IR sensor */
void setIRpower(bool power);

void eePromSaveIr();

void eePromLoadIr();
