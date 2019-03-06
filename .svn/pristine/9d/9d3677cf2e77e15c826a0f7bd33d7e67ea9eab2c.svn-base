/***************************************************************************
 *   Copyright (C) 2017 by DTU                             *
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

extern int16_t adcValue[8];
extern int16_t adcLSL[8];
extern int16_t adcLSH[8];
extern bool lsIsWhite;
extern bool lsPowerHigh;
extern bool lsTiltCompensate;

/**
 * Send line sensor difference values,
 * either directly from source, or kompensated for calibration (and tilt if in balance)
 */
void sendStatusLineSensor(bool normalized);
/**
 * send normalize gain values */
void sendLineSensorGain();

/**
 * Estimate line edge position - if relevant.
 * Called at every control interval */
void estimteLineEdgePosition();
/**
 * Send status for aAD converter values directly
 * \param idx = 1 : ADC, 2: limits, 3:values and edges, 4: gain */
void sendADLineSensor(int8_t idx);
/**
 * set linesensor parameters from GUI.
 * \param buf is the command string
 * \returns true if used */
bool setLineSensor(const char * buf);
/**
 * estimate edges of line */
void findLineEdge(void);
/**
 * estimate line crossing and edge - designed for balance use,
 * slightly more complicated */
void findEdgeV2(void);

/** save line sensor calibration */
void eePromSaveLinesensor();
/** load line sensor calibration */
void eePromLoadLinesensor();
/**
 * reset filters and stored values
 * as a follow line mission line is finished */
void lsPostProcess();
/**
 * Send linesensor findings */
void sendLineSensorPosition();

/**
 * Use line sensor */
extern bool lineSensorOn;
/**
 * difference between illuminated and not, normalized between calibration values */
extern float lineSensorValue[8];
extern int16_t whiteLevel[8];
extern int16_t blackLevel[8];
/**
 * Line sensor result */
extern float lsLeftSide;
extern float lsRightSide;
extern bool lsEdgeValid;
// extern bool crossingWhiteLine;
// extern bool crossingBlackLine;
extern int8_t crossingLineCnt;
//extern int8_t crossingLineCnt;
extern int8_t lsEdgeValidCnt;


extern float whiteVal;    // white value when line is white
// extern float notLineVal; // background value estimate when line is white or black
// extern float notLineVal; // background value estimate when line is black
extern float blackVal;    // black value when line is black
// extern bool xingW, xingB; // crossing detect (based on one measurement)
extern float findCrossingLineVal;
extern float edgeAngle;

// debug
extern int lsIdxLow, lsIdxHi;
extern int lsIdxxl, lsIdxxr;
extern float lsTl, lsTr; // position index with decimals
extern uint8_t whiteQ, blackQ; // quality of white line and black line detect
// debug end
