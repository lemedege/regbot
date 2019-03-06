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
#include "control.h"
#include "linesensor.h"
#include "robot.h"
#include "eeconfig.h"
#include "data_logger.h"
#include "mpu9150.h"

bool lineSensorOn = true;

int16_t adcValue[8] = {0,  111,221,331,441,551,661,771};
int16_t adcLSL[8] =   {0,  111,222,332,442,552,666,772};
int16_t adcLSH[8] =   {0,  110,221,331,441,551,665,771};
int16_t adcLSD[8] =   {600,611,622,633,644,655,666,677};

bool lsIsWhite = false;
int16_t blackLevel[8] = {100, 101, 102, 103, 104, 105, 106, 107};
int16_t whiteLevel[8] = {100, 101, 102, 103, 104, 105, 106, 107};
float lsGain[8] =          {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float lineSensorValue[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

float lsLeftSide = 0.0;
float lsRightSide = 0.0;
bool lsEdgeValid = false;
bool swapLeftRight = false;
// bool crossingWhiteLine = false;
// bool crossingBlackLine = false;
int8_t crossingLineCnt = 0;
// int8_t crossingLineCnt = 0;
int8_t crossingCntLimit = 20;
int8_t lsEdgeValidCnt = 0;
bool lsPowerHigh;
bool lsTiltCompensate;
float whiteLimit = 0.8; // normalized value - white part of line must exceed this value (crossing white and white edge)
float blackLimit = 0.3; // normalized value - black part of line must be below this value (crossing black and black edge)
// actual sensor value limits
float lineValMin = 1.0, lineValMax = 0.0;
// version 2 edge estimate values
float whiteVal;    /// white value when line is white
// float notLineVal; /// background value estimate when line is white or black
//float notLineVal; /// background value estimate when line is black
float blackVal;    /// black value when line is black
const int edgeDetectCntLimit = 20; /// number of times a line has to be detected to be valid
static const float lsMidtIndex = 3.5; /// led 0..3 left and LED 4..7 right, so center is 3.5
static const float lsLEDdistance = 0.76; /// distance between LED sensors in cm.
int lsIdxLow, lsIdxHi;
int lsIdxxl, lsIdxxr;
uint8_t whiteQ, blackQ;
float lsTl, lsTr;
//bool xingW = false, xingB = false; // crossing detect (one crossing)
float crossingDetect = 6.5; // criteria for sensor count (as float) estimated as line
bool wideSensor = false; // either 6cm (false) or 10 cm wide (true)
int invalidCnt;
float oldPos2mm = 0, oldPos10mm = 0;
float edgeAngle = 0.0;
float findCrossingLineVal;
float edgePos2mm[2];
float edgePos10mm[2];


void sendLSfind();

//////////////////////////////////////////////

void sendLineSensorPosition()
{
  const int MRL = 100;
  char reply[MRL];
  snprintf(reply, MRL, "lip %d %d %.4f %d %.4f %d %d %d %d %d %d %d %d %g %d %d\r\n" ,
           lineSensorOn, 
           lsIsWhite,
           lsLeftSide, lsEdgeValidCnt,
           lsRightSide, lsEdgeValidCnt, 
           mission_line_LeftEdge,  // not visible in client
           44, 44, crossingLineCnt, crossingLineCnt,
           lsPowerHigh, lsTiltCompensate, crossingDetect,
           wideSensor, swapLeftRight
  );
  usb_send_str(reply);
}

//////////////////////////////////////////////

void sendStatusLineSensorLimitsWhite()
{
  const int MRL = 70;
  char reply[MRL];
  snprintf(reply, MRL, "liw %d %d %d %d %d %d %d %d\r\n" ,
           whiteLevel[0],
           whiteLevel[1],
           whiteLevel[2],
           whiteLevel[3],
           whiteLevel[4],
           whiteLevel[5],
           whiteLevel[6],
           whiteLevel[7]
  );
  usb_send_str(reply);
}

void sendStatusLineSensorLimitsBlack()
{
  const int MRL = 70;
  char reply[MRL];
  snprintf(reply, MRL, "lib %d %d %d %d %d %d %d %d\r\n" ,
           blackLevel[0],
           blackLevel[1],
           blackLevel[2],
           blackLevel[3],
           blackLevel[4],
           blackLevel[5],
           blackLevel[6],
           blackLevel[7]
  );
  usb_send_str(reply);
}

//////////////////////////////////////////////

void sendStatusLineSensor(bool normalized)
{
  const int MRL = 70;
  char reply[MRL];
//   sendLineSensorPosition();
  if (normalized)
  { // compensated for calibration and tilt,
    // value in range 0..1
    snprintf(reply, MRL, "liv %d %d %d %d %d %d %d %d\r\n" ,
             int(lineSensorValue[0] * 3000),
             int(lineSensorValue[1] * 3000),
             int(lineSensorValue[2] * 3000),
             int(lineSensorValue[3] * 3000),
             int(lineSensorValue[4] * 3000),
             int(lineSensorValue[5] * 3000),
             int(lineSensorValue[6] * 3000),
             int(lineSensorValue[7] * 3000)
    );
  }
  else
  { // raw value from AD converter
    snprintf(reply, MRL, "liv %d %d %d %d %d %d %d %d\r\n" ,
             adcLSD[0],
             adcLSD[1],
             adcLSD[2],
             adcLSD[3],
             adcLSD[4],
             adcLSD[5],
             adcLSD[6],
             adcLSD[7]
    );
  }
  usb_send_str(reply);
    // send also position
}

void sendLineSensorADC()
{
  const int MRL = 150;
  char reply[MRL];
  //   sendLineSensorPosition();
  //if (useLineSensor)
  snprintf(reply, MRL, "#adc %d %d  %d %d  %d %d  %d %d  %d %d  %d %d  %d %d  %d %d\r\n", //    %d %d %d %d %d %d %d %d\r\n" ,
           adcLSH[0], adcLSL[0],
           adcLSH[1], adcLSL[1],
           adcLSH[2], adcLSL[2],
           adcLSH[3], adcLSL[3],
           adcLSH[4], adcLSL[4],
           adcLSH[5], adcLSL[5],
           adcLSH[6], adcLSL[6],
           adcLSH[7], adcLSL[7] //,
           //adcErr0[3], adcErr0[4], adcErr0[5], adcErr0[6], adcErr0[7], adcErr0[8], adcErr0[9], adcErr0[10] 
  );
  usb_send_str(reply);
    // send also position
    //     sendStatusLineSensorLimits();
}

void sendLineSensorGain()
{
  const int MRL = 120;
  char reply[MRL];
  //   sendLineSensorPosition();
  //if (useLineSensor)
  snprintf(reply, MRL, "#gain %g %g %g %g %g %g %g %g\r\n" ,
           lsGain[0],
           lsGain[1],
           lsGain[2],
           lsGain[3],
           lsGain[4],
           lsGain[5],
           lsGain[6],
           lsGain[7]
  );
  usb_send_str(reply);
  // send also position
  //     sendStatusLineSensorLimits();
}

//////////////////////////////////////////////


void sendADLineSensor(int8_t idx)
{
  switch (idx)
  {
    case 1: // u9 -> liw
      sendStatusLineSensorLimitsWhite();
      break;
    case 2: // u10 -> lib
      sendStatusLineSensorLimitsBlack();
      break;
    case 3: // u11 -> #gain
      //sendLineSensorGain();
      sendStatusLineSensor(true);
      break;
    case 4: // u12 -> liv
      sendStatusLineSensor(false);
      break;
    case 5: // u13 -> lip
      sendLineSensorPosition();
      break;
    case 6: // u14 -> #adc
      sendLineSensorADC();
      break;
    default:
      break;
  }
}

//////////////////////////////////////////////

bool setLineSensor(const char * buf)
{
  bool used = false;
  { // is for the line sensor
    if (strncmp(buf, "lip", 3) == 0)
    { // assumed white line
      char * p1 = (char *)&buf[4];
      used = true;
      lineSensorOn = strtol(p1, &p1, 10);
      lsIsWhite = strtol(p1, &p1, 10);
      lsPowerHigh = strtol(p1, &p1, 10);
      lsTiltCompensate = strtol(p1, &p1, 10);
      crossingDetect = strtof(p1, &p1);
      wideSensor = strtol(p1, &p1, 10);
      if (strlen(p1) > 0)
        swapLeftRight = strtol(p1, &p1, 10);
      //usb_send_str("# got a lip\n");
    }
    else if (strncmp(buf, "licw", 4) == 0)
    { // calibrate white
//      int16_t v =  adcLSH[0] - adcLSL[0];
      used = true;
//       if (v < 256)
//         usb_send_str("#surface is NOT white!\n");
//       else
      {
        for (int i = 0; i < 8; i++)
        {
          whiteLevel[i] = adcLSH[i] - adcLSL[i];
          lsGain[i] = 1.0/(whiteLevel[i] - blackLevel[i]);
        }
      }
    } 
    else if (strncmp(buf, "licb", 4) == 0)
    { // calibrate black
      int16_t v =  adcLSH[0] - adcLSL[0];
      used = true;
//       if (v > 1024)
//         usb_send_str("#surface is NOT black!\n");
//       else
      {
        for (int i = 0; i < 8; i++)
        {
          v = adcLSH[i] - adcLSL[i];
          if (v < 0)
            v = 0;
          blackLevel[i] = v;
          lsGain[i] = 1.0/(whiteLevel[i] - blackLevel[i]);
        }
      }
    } 
  }
  return used;
}


//
void findCrossingLine();
void findEdgeV3();
void findEdgeV4();
void normalize(void);


void estimteLineEdgePosition()
{
  if (lineSensorOn)
  { // find edge position ..
    normalize();
    // findEdgeV3();
    findEdgeV4();
    // and estimate if we met a crossing line
    findCrossingLine();
    if (swapLeftRight)
    {
      float a = lsLeftSide;
      lsLeftSide = lsRightSide;
      lsRightSide = a;
    }
  }
  else
  { // nothing is valid if sensor is off
    lsEdgeValid = false;
    crossingLineCnt = 0;
    findCrossingLineVal = 0.0;
    lsLeftSide = 0;
    lsRightSide = 0;
  }
}

/////////////////////////////////////////////

void normalize(void)
{
  lineValMin = 1.0;
  lineValMax = 0.0;
  float lsv;
  for (int i = 0; i < 8; i++)
  {
    //       int16_t d = whiteLevel[i] - blackLevel[i];
    int16_t v = adcLSH[i] - adcLSL[i];
    adcLSD[i] = v;
    v -= blackLevel[i];
    lsv = v * lsGain[i];
    if (lsTiltCompensate)
    { // assumed to be in balance within +/- 18 degrees
      // compensate intensity as the robot tilts.
      // assumes the intensity is calibrated at tilt angle 0
      // leaning forward (pose[3] positive) then decrease value
      if (pose[3] >= 0)
      { // leaning forward - tilt is positive
        if (i == 0 or i == 7)
          lsv /= (1.0 + 2.5 * pose[3]);
        else
          lsv /= (1.0 + 1.25 * pose[3]);
      }
      else //if (pose[3] > -0.3)
      {  // leaning away from line - tilt is negative
        if (i == 0 or i == 7)
          lsv /= (1.0 + 2.5 * pose[3]);
        else if (i == 1 or i == 6)
          lsv /= (1.0 + 1.8 * pose[3]);
        else
          lsv /= (1.0 + 1.53 * pose[3]);
      }
    }
    // limit
//     if (lsv > 1)
//       lsv = 1;
//     else if (lsv < 0)
//       lsv = 0;
    // remove a bit of noice
    lineSensorValue[i] = (lineSensorValue[i] + lsv) / 2.0;
    // get max and min values for all sensors
    if (lineSensorValue[i] > lineValMax)
      lineValMax = lineSensorValue[i];
    if (lineSensorValue[i] < lineValMin)
      lineValMin = lineSensorValue[i];
  }
}


void lsPostProcess()
{
  blackVal = 0;
  whiteVal = 1;
  lsEdgeValid = false;
  whiteQ = 0;
  blackQ = 0;
}


///////////////////////////////////////////////////

const bool logLineSensorExtra = false;

// flyttet til data_logger.h
// const int dataloggerExtraSize = 19;
// float dataloggerExtra[dataloggerExtraSize];
// findEdgeV3
// 0..6 is lensor gradient
// 7,8 max,min gradient
// 9,10,11 left,cent,right - left side
// 12 edge position  - left side
// 13,14,15 left,cent,right - rignt side
// 16 edge position  - right side
// 17, 18 max index, min idx (of int values)

// findEdgeV4
// 0..7 is lensor normalized
// 8 - crossing line vr value
// 9,10,11 left,r-l,right - left side
// 12 edge position  - left side
// 13,14,15 left,r.l,right - rignt side
// 16 edge position  - right side
// 17, 18 max index, min idx (of int values)

const float kd = 10;
bool crossing = false;

void findCrossingLine()
{ // metode: for højre side af linje
  //   line width + udvikling sidste mm * kd - gyroZ * (-50)
  // skulle nok være dynamisk, nu da vi har en variabel
  float crossingTreshold = crossingDetect;
  float vl, vr;
  float dist = distance - oldPos2mm;
  if (crossing)
    // add a bit of hysteresis
    crossingTreshold = crossingDetect - 1;
  else
    crossingTreshold = crossingDetect;
  //
  if ((dist > 0.01) or (oldPos2mm > distance))
  { // do not trust crossing line until control is setteled,
    // or if line sensor has been inactive for some time
    invalidCnt = 10; // in sample time
    oldPos2mm = distance;
    crossingLineCnt = 0;
    edgePos2mm[0] = lsLeftSide;
    edgePos2mm[1] = lsRightSide;
  }
  // debug log
  if (logLineSensorExtra)
  {
    dataloggerExtra[0] = edgePos2mm[0];
    dataloggerExtra[1] = edgePos2mm[1];
    dataloggerExtra[2] = invalidCnt;
    dataloggerExtra[3] = distance;
    dataloggerExtra[4] = oldPos2mm;
    dataloggerExtra[5] = dist;
  }
  // debug log end
  if (lsEdgeValid)
  { // edges are valid
    if (invalidCnt == 0)
    { // add a bit rate of change per mm with a gain of kd
      vl = lsLeftSide + (lsLeftSide - edgePos2mm[0]) * kd;
      vr = lsRightSide + (lsRightSide - edgePos2mm[1]) * kd;
    }
    else
    { // history (rate of change) is not valid
      vl = lsLeftSide;
      vr = lsRightSide;
    }
    // save rate of change (per mm)
    if (distance - oldPos2mm > 0.002)
    { // distance is 1mm (or more), so we save a value
      // (to avoid turning at a crossing line)
      oldPos2mm = distance;
      edgePos2mm[0] = lsLeftSide;
      edgePos2mm[1] = lsRightSide;
    }
    const float kg = 0.0005;
    // on robobot z-axis is gyro x, on regbot it is z, so add the two
    findCrossingLineVal = vr - vl - fabsf(imuGyro[2] + imuGyro[0]) * kg;
    // debug log
    if (logLineSensorExtra)
    {
      dataloggerExtra[7] = vl;
      dataloggerExtra[8] = vr;
    }
    // debug log end
    // simpel threshold to detect
    crossing =  findCrossingLineVal > crossingTreshold;
    if (invalidCnt > 0)
      invalidCnt--;
  }
  else
  { // no valid input
    if (invalidCnt < 4)
      invalidCnt++;
    findCrossingLineVal = 0.0;
    crossingLineCnt = 0;
    edgeAngle = 0.0;
  }
  // make a quality count to be used as mission parameter
  if (crossing)
  { // increase up to maximum (20)
    if (crossingLineCnt < crossingCntLimit)
      crossingLineCnt++;
  }
  else
  { // decrease until 0
    if (crossingLineCnt > 0)
      crossingLineCnt--;
  }
}




void findEdgeV4()
{
  float * nv = dataloggerExtra; // normalized value - for debug logging only;
  if (logLineSensorExtra)
    memset(dataloggerExtra, 0, sizeof(dataloggerExtra));
  const float minOnWhiteLine = 0.8; // hard limit - part of span from white to black
  const float maxOnBlackLine = 0.3; // hard limit - part of span from white to black
  lsEdgeValid = false;
  for (int i = 0; i < 8; i++)
  {
    if (lsIsWhite)
    {
      if (lineSensorValue[i] > minOnWhiteLine)
      {
        lsEdgeValid = true;
        break;
      }
    }
    else
    {
      if (lineSensorValue[i] < maxOnBlackLine)
      {
        lsEdgeValid = true;
        break;
      }
    }
  }
  //   if (not lsIsWhite)
  //     usb_send_str("# NB! code not valid for black line yet\n");
  //
  if (lsEdgeValid)
  {
    int dimax = 0, dimin = 0;
    float dvmin, dvmax;
    // find gratest gradient (dv) - positive and negative
    dvmin = lineSensorValue[0];
    dvmax = dvmin;
    nv[0] = lineSensorValue[0];
    // find midt line
    for (int i = 1; i < 8; i++)
    {
      // debug log
      nv[i] = lineSensorValue[i];
      // debug log end
      if (lineSensorValue[i] > dvmax)
      {
        dvmax = lineSensorValue[i];
        dimax = i;
      }
      else if (lineSensorValue[i] < dvmin)
      { // for black line
        dvmin = lineSensorValue[i];
        dimin = i;
      }
    }
    // debug log
    if (logLineSensorExtra)
    {
      dataloggerExtra[17] = dimax; // white
      dataloggerExtra[18] = dimin; // black
    }
    // debug log end
    int leftIdx = 0, rightIdx = 7;
    for (int i = dimax - 1; i > 0; i--)
    {
      if (lineSensorValue[i] < minOnWhiteLine)
      {
        leftIdx = i;
        break;
      }
    }
    for (int i = dimax + 1; i < 7; i++)
    {
      if (lineSensorValue[i] < minOnWhiteLine)
      {
        rightIdx = i;
        break;
      }
    }
    // debug log
    if (logLineSensorExtra)
    {
      dataloggerExtra[19] = leftIdx * 100 + rightIdx;
    }
    // debug log end
    // left side (positive gradient if line is white)
    float mleft, mright; // gradient values left and right of maximum
    float edgePos;
    // left edge of line
    mleft = lineSensorValue[leftIdx];
    mright = lineSensorValue[leftIdx + 1];
    // find edge position in LED index
    float s = mright-mleft;
    if (s > 0.001)
      edgePos = (minOnWhiteLine - mleft)/s + leftIdx;
    else
      // left edge increasing - no good
      edgePos = leftIdx - 0.5;
    // debug log
    if (logLineSensorExtra)
    {
      dataloggerExtra[9] = mleft;
      dataloggerExtra[11] = mright;
      dataloggerExtra[10] = s;
      dataloggerExtra[12] = edgePos;
    }
    // debug log end
    // convert to cm.
    lsLeftSide = (edgePos - lsMidtIndex) * lsLEDdistance;
    if (lsLeftSide < -3.0)
      lsLeftSide = -3.0;
    //
    mleft = lineSensorValue[rightIdx - 1];
    mright = lineSensorValue[rightIdx];
    // find edge position in LED index
    s = mright-mleft;
    if (s < -0.001)
      edgePos = (minOnWhiteLine - mleft)/s + rightIdx - 1;
    else
      // right edge is increasing - no good
      edgePos = rightIdx + 0.5;
    // debug log
    if (logLineSensorExtra)
    {
      dataloggerExtra[13] = mleft;
      dataloggerExtra[15] = mright;
      dataloggerExtra[14] = s;
      dataloggerExtra[16] = edgePos;
    }
    // debug log end
    // convert to cm.
    lsRightSide = (edgePos - lsMidtIndex) * lsLEDdistance;
    if (lsRightSide > 3.0)
      lsRightSide = 3.0;
    //
    if (lsEdgeValidCnt < edgeDetectCntLimit)
      lsEdgeValidCnt++;
  }
  else
  {
    lsRightSide = 0.0;
    lsLeftSide = 0.0;
    if (lsEdgeValidCnt > 0)
      lsEdgeValidCnt--;
  }  
}


/////////////////////////////////////////////////////////////

// void findEdgeV3()
// {
//   float * dv = dataloggerExtra; //dv[7];
//   bool onLine[8];
//   memset(dataloggerExtra, 0, sizeof(dataloggerExtra));
//   const float minOnWhiteLine = 0.65; // hard limit - part of span from white to black
//   const float maxOnBlackLine = 0.3; // hard limit - part of span from white to black
//   lsEdgeValid = false;
//   for (int i = 0; i < 8; i++)
//   {
//     if (lsIsWhite)
//     {
//       if (lineSensorValue[i] > minOnWhiteLine)
//       {
//         lsEdgeValid = true;
//         onLine[i] = true;
//       }
//       else
//         onLine[i] = false;
//     }
//     else
//     {
//       if (lineSensorValue[i] < maxOnBlackLine)
//       {
//         lsEdgeValid = true;
//         onLine[i] = true;
//       }
//       else
//         onLine[i] = false;
//     }
//   }
// //   if (not lsIsWhite)
// //     usb_send_str("# NB! code not valid for black line yet\n");
//   //
//   if (lsEdgeValid)
//   {
//     int dimax = -1, dimin = -11;
//     float dvmin = 1, dvmax = -1;
//     // find gratest gradient (dv) - positive and negative
//     for (int i = 0; i < 7; i++)
//     {
//       dv[i] = lineSensorValue[i + 1] - lineSensorValue[i];
//       if (onLine[i] or onLine[i+1])
//       {
//         if (dv[i] > dvmax)
//         {
//           dvmax = dv[i];
//           dimax = i;
//         }
//         else if (dv[i] < dvmin)
//         {
//           dvmin = dv[i];
//           dimin = i;
//         }
//       }
//     }
//     if (not lsIsWhite)
//     { // for black line invert gradient
//       // invert gradient and svap min and max
//       int i = dimin;
//       float v = dvmin;
//       dvmin = -dvmax;
//       dimin = dimax;
//       dvmax = - v;
//       dimax = i;
//     }
//     dataloggerExtra[7] = dvmax;
//     dataloggerExtra[8] = dvmin;
//     dataloggerExtra[17] = dimax;
//     dataloggerExtra[18] = dimin;
//     // left side (positive gradient if line is white)
//     float mleft, mcent, mright; // gradient values left and right of maximum
//     float edgePos;
//     if (dimax == 0)
//       // use minimum as reference
//       mleft = -dvmin;
//     else
//       mleft = dv[dimax - 1];
//     if (dimax == 7)
//       // unlikely, but possible, that maximum is right-most
//       // then use maximum gradient on other side
//       mright = -dvmin;
//     else
//       mright = dv[dimax + 1];
//     dataloggerExtra[9] = mleft;
//     dataloggerExtra[11] = mright;
//     if (mleft > mright)
//     { // use interpolation to the left
//       mcent = dvmax - mright;
//       mleft = mleft - mright;
//       // find edge position in LED index
//       float s = mcent + mleft;
//       if (fabs(s) < 1e-4)
//         s = copysign(1e-4, mcent);
//       edgePos = (mcent - mleft)/s * 0.5 - 0.5 + dimax;
//     }
//     else
//     {
//       mcent = dvmax - mleft;
//       mleft = mright - mleft;
//       float s = mcent + mright;
//       if (fabs(s) < 1e-4)
//         s = copysign(1e-4, mcent);
//       // find edge position in LED index
//       edgePos = 0.5 - (mcent - mright)/s * 0.5 + dimax;
//     }
//     dataloggerExtra[10] = mcent;
//     dataloggerExtra[12] = edgePos;
//     // convert to cm.
//     lsLeftSide = (edgePos - lsMidtIndex) * lsLEDdistance;
//     if (lsLeftSide < -3.0)
//       lsLeftSide = -3.0;
//     //
//     // right side (negative gradient if line is white)
//     if (dimin == 6)
//       // use maximum as reference
//       mright = -dvmax;
//     else
//       mright = dv[dimin + 1];
//     if (dimin == 0)
//       mleft = -dvmax;
//     else
//       mleft = dv[dimin - 1];
//     dataloggerExtra[13] = mleft;
//     dataloggerExtra[15] = mright;
//     if (mleft > mright)
//     { // use interpolation to the left - left is most positive
//       // subtract right value
//       mcent = dvmin - mright;
//       mleft = mleft - mright;
//       // find edge position in LED index
//       float s = mcent + mleft;
//       if (fabs(s) < 1e-4)
//         s = copysign(1e-4, mcent);
//       edgePos = (mcent - mleft)/s * 0.5 - 0.5 + dimin;
//     }
//     else
//     {
//       mcent = dvmin - mleft;
//       mleft = mright - mleft;
//       float s = mcent + mright;
//       if (fabs(s) < 1e-4)
//         s = copysign(1e-4, mcent);
//       // find edge position in LED index
//       edgePos = 0.5 - (mcent - mright)/s * 0.5 + dimin;
//     }
//     dataloggerExtra[14] = mcent;
//     dataloggerExtra[16] = edgePos;
//     // convert to cm.
//     lsRightSide = (edgePos - lsMidtIndex) * lsLEDdistance;
//     if (lsRightSide > 3.0)
//       lsRightSide = 3.0;
//     //
//     if (lsEdgeValidCnt < edgeDetectCntLimit)
//       lsEdgeValidCnt++;
//   }
//   else
//   {
//     lsRightSide = 0.0;
//     lsLeftSide = 0.0;
//     if (lsEdgeValidCnt > 0)
//       lsEdgeValidCnt--;
//   }  
// }
// 
// 
// ////////////////////////////////////////////////////////
// 
// void findEdgeV2()
// {
//   const uint8_t lineQualMax = 50; // max edge quality
//   int onLine1 = -1, onLine2 = -1 ; // first and last LED looking at line tape
//   const float minOnWhiteLine = 0.85; // hard limit - part of span from white to black
//   const float maxOnBlackLine = 0.3; // hard limit - part of span from white to black
//   float maxWhiteDeviation = 0.15 / (1.0 + float(whiteQ)/10.0); // deviation from last line value (whiteVal or blackVal)
//   float maxBlackDeviation = 0.15 / (1.0 + float(blackQ)/10.0); // deviation from last line value (whiteVal or blackVal)
//   if (lineSensorOn)
//   { // normalize to range 0..1
//     normalize();
//     if (lsIsWhite)
//     { // 
//       for (int i = 0; i < 8; i++)
//       { // looking for on-white-line sensors
//         if (onLine1 == -1)
//         { // looking for line start
//           if (lineSensorValue[i] > minOnWhiteLine and
//             lineSensorValue[i] - whiteVal > -maxWhiteDeviation)
//           { // white line detected
//             onLine1 = i;
//             onLine2 = i;
//           }
//         }
//         else
//         { // looking for line end
//           if (lineSensorValue[i] < minOnWhiteLine or
//             whiteVal - lineSensorValue[i] > maxWhiteDeviation)
//             // not on line any more, and only one line expected
//             break;
//           // this LED is also on line
//           onLine2 = i;
//         }
//       }
//       // debug
//       lsIdxLow = onLine1;
//       lsIdxHi  = onLine2;
//       // debug end
//       // a white edge is valid if at least one sensor is white
//       if (onLine1 >= 0)
//       { // a white line is valid
//         if (whiteQ < lineQualMax)
//           whiteQ++;
//         //
//         // increase edge count, as edge is set valid if line is detected
//         if (lsEdgeValidCnt < edgeDetectCntLimit)
//           lsEdgeValidCnt++;
//         else
//           lsEdgeValid = true;
//         //
//         // calculate new mean white value - for next detection sample
//         float mean = 0;
//         for (int i = onLine1; i <= onLine2; i++)
//           mean += lineSensorValue[i];
//         mean /= onLine2 - onLine1 + 1;
//         // average estimated white intensity over 8 samples (8ms)
//         whiteVal = mean; //(whiteVal * 7 + mean) / 8;
//         // 
//         // find edge as interpolation
//         // between sensors that has value on either side of
//         // value halfWhite between white and not white
//         // 
//         int i;
//         float edgePos;
//         float edgePositionThreshold = minOnWhiteLine;// lineValMax - 0.03; // (whiteVal * 3.0 + notLineVal) / 4.0;
//         // detectable left edge
//         for (i = onLine1 /*- 1*/; i >= 0; i--)
//         { // find first non-white sensor to the left
//           if (lineSensorValue[i] < edgePositionThreshold)
//             break;
//         }
//         // debug
//         lsIdxxl = i;
//         // debug end
//         int i2 = 0;
//         if (i > 0)
//           // not too close to 0, take 2 values
//           i2=1;
//         // sendor i is lower than edge value
//         float dw = lineSensorValue[i + 1] - lineSensorValue[i-i2];
//         if (dw > 0.001)
//         {
//           float de = edgePositionThreshold - lineSensorValue[i-i2];
//           float t = de/dw;
//           edgePos = i + t*(1+i2);
//         }
//         else
//           // unlikely situation - equal intensity
//           edgePos = 0.5 + i;
//         if (edgePos < -1)
//           edgePos = -1;
//         // debug
//         lsTl = edgePos;
//         // debug end
//         // calculate position in cm from center line
//         edgePos = (edgePos - lsMidtIndex) * lsLEDdistance;
//         // make a bit of averaging
//         lsLeftSide = edgePos; // (lsLeftSide * 3 + edgePos)/4;
//         //
//         // find right edge too
//         // is detectable
//         for (i = onLine2 /*+ 1*/; i < 8; i++)
//         { // find first non-white sensor to the right
//           if (lineSensorValue[i] < edgePositionThreshold)
//             break;
//         }
//         // debug
//         lsIdxxr = i;
//         // debug end
//         i2 = 0;
//         if (i < 7)
//           // not too close to max, take 2 values
//           i2 = 1;
//         // sensor i is lower than edge value
//         dw = lineSensorValue[i - 1] - lineSensorValue[i + i2];
//         if (dw > 0.001)
//         { // non zero difference
//           float de = edgePositionThreshold - lineSensorValue[i + i2];
//           float t = de/dw;
//           if (t > 1)
//             t = 1;
//           else if (t < 0)
//             t = 0;
//           edgePos = i - t * (1 + i2);
//         }
//         else
//           // zero difference (unlikely), or other way (also unlikely), so use half way
//           edgePos = i - 0.5;
//         if (edgePos > 8)
//           edgePos = 8;
//         // debug
//         lsTr = edgePos;
//         // debug end
//         // calculate position in cm from center line
//         edgePos = (edgePos - lsMidtIndex) * lsLEDdistance;
//         // make a bit of averaging
//         lsRightSide = edgePos; //(lsRightSide * 3 + edgePos)/4;
//         //
//       }
//       else
//       { // no white line detected
//         if (whiteQ > 0)
//           whiteQ--;
//         // return to calibrated white value
//         whiteVal = 1.0;
//         // no line detected, so both edges are becoming invalid fast
//         if (lsEdgeValidCnt > 0)
//           lsEdgeValidCnt -= 3;
//         else
//         {
//           lsEdgeValidCnt = 0;
//           lsEdgeValid = false;
//         }
//         lsRightSide = 0.0;
//         lsLeftSide = 0.0;
//       }
//     }
//     else
//     { // looking for black line
//       for (int i = 0; i < 8; i++)
//       { // looking for on-black-line sensors
//         if (onLine1 == -1)
//         { // looking for line start
//           if (lineSensorValue[i] < maxOnBlackLine and
//             lineSensorValue[i] - blackVal < maxBlackDeviation)
//           { // white line detected
//             onLine1 = i;
//             onLine2 = i;
//           }
//         }
//         else
//         { // looking for line end
//           if (lineSensorValue[i] > maxOnBlackLine or
//             lineSensorValue[i] - blackVal > maxBlackDeviation)
//             // not on line any more, and only one line expected
//             break;
//           // this LED is also on line
//           onLine2 = i;
//         }
//       }
//       // debug
//       lsIdxLow = onLine1;
//       lsIdxHi  = onLine2;
//       // debug end
//       // a black edge is valid if at least one sensor is black
//       if (onLine1 >= 0)
//       { // a black line is valid
//         if (blackQ < lineQualMax)
//           blackQ++;
//         //
//         // increase edge count, as edge is set valid if line is detected
//         if (lsEdgeValidCnt < edgeDetectCntLimit)
//           lsEdgeValidCnt++;
//         else
//           lsEdgeValid = true;
//         //
//         // calculate new mean black value - for next detection sample
//         float mean = 0;
//         for (int i = onLine1; i <= onLine2; i++)
//           mean += lineSensorValue[i];
//         mean /= onLine2 - onLine1 + 1;
//         // average estimated black intensity over 8 samples (8ms)
//         blackVal = mean; //(blackVal * 7 + mean) / 8;
//         // 
//         // find edge as interpolation
//         // between sensors that has value on either side of
//         // value halfBlack between black and not black
//         // 
//         int i;
//         float edgePos;
//         float edgePositionThreshold = lineValMin + 0.03; // (whiteVal * 3.0 + notLineVal) / 4.0;
//         // detectable left edge
//         for (i = onLine1 /*- 1*/; i >= 0; i--)
//         { // find first non-white sensor to the left
//           if (lineSensorValue[i] > edgePositionThreshold)
//             break;
//         }
//         // debug
//         lsIdxxl = i;
//         // debug end
//         int i2 = 0;
//         if (i > 0)
//           // not too close to 0, take 2 values
//           i2=1;
//         // sendor i is lower than edge value
//         float dw = lineSensorValue[i-i2] - lineSensorValue[i + 1];
//         if (dw > 0.001)
//         {
//           float de = lineSensorValue[i-i2] - edgePositionThreshold;
//           float t = de/dw;
//           edgePos = i + t * (1+i2);
//         }
//         else
//           // unlikely situation - equal intensity
//           edgePos = 0.5 + i;
//         if (edgePos < 0)
//           edgePos = 0;
//         // debug
//         lsTl = edgePos;
//         // debug end
//         // calculate position in cm from center line
//         edgePos = (edgePos - lsMidtIndex) * lsLEDdistance;
//         // make a bit of averaging
//         lsLeftSide = edgePos; // (lsLeftSide * 3 + edgePos)/4;
//         //
//         for (i = onLine2 /*+ 1*/; i < 8; i++)
//         { // find first non-white sensor to the right
//           if (lineSensorValue[i] > edgePositionThreshold)
//             break;
//         }
//         // debug
//         lsIdxxr = i;
//         // debug end
//         i2 = 0;
//         if (i < 7)
//           // not too close to max, take 2 values
//           i2 = 1;
//         // sensor i is lower than edge value
//         dw = lineSensorValue[i + i2] - lineSensorValue[i - 1];
//         if (dw > 0.001)
//         { // non zero difference
//           float de = lineSensorValue[i + i2] - edgePositionThreshold;
//           float t = de/dw;
//           edgePos = i - t * (1 + i2);
//         }
//         else
//           // zero difference (unlikely), or other way (also unlikely), so use half way
//           edgePos = i - 0.5;
//         if (edgePos > 7)
//           edgePos = 7;
//         
//         
//         // debug
//         lsTr = edgePos;
//         // debug end
//         // calculate position in cm from center line
//         edgePos = (edgePos - lsMidtIndex) * lsLEDdistance;
//         // make a bit of averaging
//         lsRightSide = edgePos; // (lsRightSide * 3 + edgePos)/4;
//       }
//       else
//       { // no black line detected
//         if (blackQ > 0)
//           blackQ--;
//         // return to calibrated white value
//         blackQ = 0.0;
// //         xingB = false;
//         // no line detected, so both edges are becoming invalid fast
//         if (lsEdgeValidCnt > 0)
//           lsEdgeValidCnt -= 3;
//         else
//         {
//           lsEdgeValidCnt = 0;
//           lsEdgeValid = false;
//         }
//         lsRightSide = 0.0;
//         lsLeftSide = 0.0;
//       }
//     }
//   }
// }


/////////////////////////////////////////////////////

void eePromSaveLinesensor()
{
  char v = 0x00;
  if (lineSensorOn)
    v |= 0x01;
  if (lsIsWhite)
    v |= 0x02;
  if (lsPowerHigh)
    v |= 0x04;
  if (lsTiltCompensate)
    v |= 0x08;
  if (wideSensor)
    v |= 0x10;
  if (swapLeftRight)
    v |= 0x20;
  eeConfig.pushByte(v);
  eeConfig.pushByte(int(crossingDetect*10.0));
  for (int i = 0; i < 8; i++)
  {
    eeConfig.pushWord(blackLevel[i]);
    eeConfig.pushWord(whiteLevel[i]);
  }
}

/////////////////////////////////////////////////////

void eePromLoadLinesensor()
{
  char v = eeConfig.readByte();
//  char v = eeprom_read_byte((uint8_t*)eePushAdr++);
  lineSensorOn = (v & 0x01) == 0x01;
  lsIsWhite = (v & 0x02) == 0x02;
  lsPowerHigh = (v & 0x04) == 0x04;
  lsTiltCompensate = (v & 0x08) == 0x08;
  wideSensor = (v & 0x10) == 0x10;
  swapLeftRight = (v & 0x20) == 0x20;
  // limit 4 crossing detect
  crossingDetect = float(eeConfig.readByte()) / 10.0;
  // number of bytes to skip if not robot-specific configuration
  int skipCount = 8*(2 + 2);
  if (not eeConfig.isStringConfig())
  { // load from flash
    for (int i = 0; i < 8; i++)
    {
      blackLevel[i] = eeConfig.readWord();
      whiteLevel[i] = eeConfig.readWord();
    }
    for (int i = 0; i < 8; i++)
    { // set gains from new values
      lsGain[i] = 1.0/(whiteLevel[i] - blackLevel[i]);
    }
  }
  else
    // load from hard-coded mission
    eeConfig.skipAddr(skipCount);
}
