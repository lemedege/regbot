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
#include <string.h>
#include <stdio.h>
#include "math.h"
#include "mission.h"
//#include "serial_com.h"
#include "main.h"
#include "eeconfig.h"
#include "robot.h"
#include "control.h"
#include "linesensor.h"
#include "dist_sensor.h"
#include "data_logger.h"
#include "motor_controller.h"
#include "servo.h"

UMissionLine miLines[miLinesCntMax];
int miLinesCnt = 0;

char missionErrStr[missionErrStrMaxCnt];

////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////

void UMissionThread::clear(uint8_t idx)
{ // give thread a number and clear the rest
  threadNr = idx;
  lineCnt = 0;
  lineStartIndex = 0;
  misLineNum = -1;
  misStartTime = 0;
  linStartAngle = 0;
  turnAngSum = 0;
  linStartDist = 0;
  turnSumLast = 0;
  theEnd = false;
  activateFlag = 0;
  deactivateFlag = 0;
  threadActive = true;
  continueReason = 0;
}


bool UMissionThread::advanceLine ( int16_t toLabel )
{
  if (misLineNum < lineCnt)
    // default is goto next line
    misLineNum++;
  if (toLabel > 0)
  { // there is a label number, find line
    // if label is not found in this thread, then
    // continue to next line (default action)
    for (int i = 0; i < lineCnt; i++)
      if (miLines[i + lineStartIndex].label == toLabel)
      {
        misLineNum = i;
        break;
      }
  }
  // debug
//   const int MSL = 50;
//   char s[MSL];
//   snprintf(s, MSL, "# advance line to %d of %d (label=%d)\n", misLineNum, lineCnt, toLabel);
//   usb_send_str(s);
  // debug end
  if (misLineNum < lineCnt)
  { // new line is available
    currentLine = &miLines[misLineNum + lineStartIndex];
    return true;
  }
  else
    return false;
}

/////////////////////////////////////////////////////////////////

bool UMissionThread::testFinished() 
{
  bool LineEnded = false;
  //usb_send_str("# Thread::testFinished -> start\n");
  // we are not finished 
  // are thread active or deactivated
  if (threadActive and ((deactivateFlag & userMission.eventFlags) > 0))
  {
    // debug
//     const int MSL=80;
//     char s[MSL];
//     snprintf(s, MSL, "# thread %d de-activated\n", threadNr);
//     usb_send_str(s);
    // debug end
    threadActive = false;
    theEnd = true;
  }
  else if (not threadActive and ((activateFlag & userMission.eventFlags) > 0))
  {
    // debug
//     const int MSL=80;
//     char s[MSL];
//     snprintf(s, MSL, "# thread %d activated\n", threadNr);
//     usb_send_str(s);
    // debug end
    //     usb_send_str("# UMissionThread::testFinished : thread activated\n");
    // restart lines in this thread
    // - and implement first line (and clears "theEnd" (flag for this thread))
    threadActive = true;
    startMission();
    // if no lines, then the end is reached already
    LineEnded = theEnd;
  }
  else if (theEnd)
  { // line (and thread) has ended (or inactive)
    // debug
//     const int MSL=80;
//     char s[MSL];
//     snprintf(s, MSL, "# UMissionThread::testFinished : inactive thread %d ended\n", threadNr);
//     usb_send_str(s);
    // debug end
    LineEnded = true;
  }
  else if (misLineNum < lineCnt)
  {
    // debug
//     const int MSL=100;
//     char s[MSL];
//     snprintf(s, MSL, "# UMissionThread::testFinished : thread %d has %d lines, now %d, thread active=%d\n", threadNr, lineCnt, misLineNum, threadActive);
//     usb_send_str(s);
    // debug end
    if (threadActive)
    {
      //uint16_t labelNum = 0; // 0 is a flag for no goto
      //UMissionLine * line = &miLines[misLineNum + lineStartIndex];
      gotoLabel = 0;
      char cr = ' ';
      if (currentLine->finished(this, &gotoLabel, &turnEndedAtEndAngle, &cr, continueReason))
      { // stop any special drive scheme set by this line
  //       line->postProcess(misLastAng);
  //       theEnd = not advanceLine(labelNum);
  //       if (not theEnd)
  //       {
  //         implementNewLine();
  //       }
        if (not currentLine->gotoUse)
          // save new reason to continue
          continueReason = cr;
        //usb_send_str("#Line ended!");;
        LineEnded = true;
      }
  //     if (gotoLabel > 0)
  //     {
  //       const int MSL = 50;
  //       char s[MSL];
  //       snprintf(s, MSL, "# in thread goto=%d\n", gotoLabel);
  //       usb_send_str(s);
  //     }
    }
  }
  else
    LineEnded = true;
  return LineEnded;
}


//////////////////////////////////////////////////////////////

bool UMissionThread::moveToNextLine()
{
  if (not theEnd)
  { // finish last line
    currentLine->postProcess(linStartAngle, turnEndedAtEndAngle);
    // advance line (maybe a goto)
    theEnd = not advanceLine(gotoLabel);
    if (not theEnd)
    { // implement all parameter settings
      implementNewLine();
    }
  }
  return theEnd;
}

//////////////////////////////////////////////////////////////

void UMissionThread::resetVisits()
{
  for (int i = 0; i < lineCnt; i++)
    miLines[i + lineStartIndex].visits = 0;
}

//////////////////////////////////////////////////////////////

void UMissionThread::startMission()
{
  resetVisits();
  misLineNum = 0;
  theEnd = false;
  if (misLineNum < lineCnt and threadActive)
    implementNewLine();
//   else
//     theEnd = true;
//   const int MSL = 100;
//   char s[MSL];
//   snprintf(s, MSL, "# thread=%d initiated is active=%d\n", threadNr, threadActive);
//   usb_send_str(s);
}

void UMissionThread::stopMission()
{
  theEnd = true;
}
  
//////////////////////////////////////////////////////////////

void UMissionThread::implementNewLine()
{
//   // debug
//   const int MSL = 120;
//   char s[MSL];
//   // debug end
 // initialize new mission line
  currentLine = &miLines[misLineNum + lineStartIndex];
  currentLine->visits++;
  // if old line controlled angle, then
  // use end angle as new reference
  if (mission_wall_turn or regul_line_use)
  { // Wall follow or line follow
    // just use current heading as new reference
    mission_turn_ref = pose[2];
//     regTurnM[1] = pose[2];
  }
  if (misLineNum < miLinesCnt)
  { // prepare next line
    turnAngSum = 0; 
    turnSumLast = mission_turn_ref;
    linStartAngle = mission_turn_ref;
    turnEndedAtEndAngle = false;
    linStartDist = distance;
    misStartTime = hbTimerCnt;
    // all other settings
    currentLine->implementLine();
  }
  // every time a thread implements a new line 
  // this data is set - to allow user state reporting
  misLine = currentLine;
  misThread = threadNr;
  missionLineNum = misLineNum;
//   // debug
//   snprintf(s, MSL, "# implemented line %d in thread %d\r\n", misLineNum, threadNr);
//   usb_send_str(s);
//   // debug end
}

////////////////////////////////////////////////////////

// bool UMissionThread::addThreadOptions(const char * lineToAdd, int16_t * newThreadNumber)
// { // look for start and end events
//   UMissionLine tmp;
//   bool isOK = tmp.decodeLine(lineToAdd, newThreadNumber);
//   if (isOK)
//   {
//     activateFlag = tmp.eventSet;
//     deactivateFlag = tmp.eventMask;
//   }
//   return isOK;
// }


bool UMissionThread::addLine(const char * lineToAdd)
{
  int idx = lineCnt + lineStartIndex;
  bool isOK = idx < miLinesCntMax;
  const int MSL = 150;
  char s[MSL];
  if (isOK)
  {  
//     // debug
//      snprintf(s, MSL, "# UMissionThread::addLine thread=%d, lineCnt %d is: %s\n\r", threadNr, lineCnt, lineToAdd);
//      usb_send_str(s);
//     // debug end
    isOK = miLines[idx].decodeLine(lineToAdd, NULL);
    if (not isOK)
    { // report error
      snprintf(s, MSL, "#syntax error thread %d line %d: %s\n", threadNr, lineCnt, missionErrStr);
      usb_send_str(s);
    }
//     // debug
//      snprintf(s, MSL, "# thread %d newThread %d isOK=%d\n\r", threadNr, *newThreadNumber, isOK);
//      usb_send_str(s);
//     // debug end
    // changing thread number is handled at UMission level
//     if (*newThreadNumber != threadNr)
//     { // there is a new thread number
//       if (lineCnt == 0)
//       {  // but OK, as there is no lines
//         threadNr = *newThreadNumber;
//         // get the two other valid options in a thread line
//         activateFlag = miLines[idx].eventSet;
//         deactivateFlag = miLines[idx].eventMask;
//         // debug
//         snprintf(s, MSL, "# added thread %d with act=%lx, deac=%lx\n", *newThreadNumber, activateFlag, deactivateFlag);
//         usb_send_str(s);
//         // debug end
//       }
//       // line should not be added as a separate line
//       isOK = false;
//     }
    if (isOK)
    { // OK to add line
      isOK = miLines[idx].valid;
      if (isOK)
      {
        int n = miLines[idx].toString(s, MSL, false);
        if (n > 2)
        { // not an empty line
          lineCnt++;
          if (idx >= miLinesCnt)
            miLinesCnt++;
//           // debug
//            snprintf(s, MSL, "# thread %d line increased to %d\n\r", threadNr, lineCnt);
//            usb_send_str(s);
//           // debug end
        }
      }
      else
      {
        snprintf(s, MSL, "\r\n# add line %d failed: %s\r\n", idx, missionErrStr);
        usb_send_str(s);
      }
    }
//     // debug
//      snprintf(s, MSL, "# thread %d line %d added OK =%d\n\r", threadNr, lineCnt, isOK);
//      usb_send_str(s);
//     // debug end
  }
  return isOK;
}


bool UMissionThread::modLine(int16_t line, const char * p2)
{
  bool isOK = line <= lineCnt  and line > 0;
  if (isOK)
  { // decode the line twice first for syntax check, then for real
    UMissionLine tmp;
    int16_t thnr = threadNr;
    isOK = tmp.decodeLine(p2, &thnr);
    if (thnr != threadNr)
    { // there must not be a line with new thread number
      isOK = false;
      usb_send_str("# not legal to modify thread number\n");
    }
    // debug
//     const int MSL = 50;
//     char s[MSL];
//     snprintf(s, MSL, "# line %d found isOK=%d\n", line, isOK);
//     usb_send_str(s);
    // debug end
    
    if (isOK)
    { // OK, modify the line
      int idx = line + lineStartIndex - 1;
      isOK = miLines[idx].decodeLine(p2, &thnr);
    }
  }
  else
    usb_send_str("# modify failed - no such line\n");
  return isOK;
}

/////////////////////////////////////

bool UMissionThread::getLines(bool more)
{ // get lines in thread as string
  const int MRL = 100;
  char reply[MRL];
  bool result = true;
  if (moreLine == -1)
  {
//     usb_send_str("# UMissionThread::getLines: first line in thread\n");
    snprintf(reply, MRL, "<m thread=%d", threadNr);
    int n = strlen(reply);
    char * p1 = &reply[n];
    if (activateFlag or deactivateFlag)
    { // add event part(s)
//       const int MSL = 100;
//       char s[MSL];
//       snprintf(s, MSL, "# thread-line acti=%lu, deac=%lu\n", activateFlag, deactivateFlag);
//       usb_send_str(s);
//       usb_send_str("# UMissionThread::getLines: has event flags\n");
      const char sep = ',';
      if (activateFlag)
      {
        for (int i = 0; i < 32; i++)
        { // test all possible events
          if (activateFlag & (1 << i))
          { // event i is set, add to line
            *p1++=sep; 
            n++;
            snprintf(p1, MRL - n - 1, "event=%d", i);
            n += strlen(p1);
            p1 = &reply[n];
          }
        }
      }
      if (deactivateFlag)
      {
        *p1++ = ':';
        n++;
        for (int i = 0; i < 32; i++)
        { // test all possible events
          if (deactivateFlag & (1 << i))
          { // event i is set, add to line
//             usb_send_str("# found deactivate flag\n");
            if (p1[-1] != ':')
            { // first stop event should not be preceded by a separator
              *p1++=sep; 
              n++;
//               usb_send_str("# found deactivate flag and :\n");
            }
//             else
//               usb_send_str("# found deactivate flag no :\n");
            snprintf(p1, MRL - n - 1, "event=%d", i);
            n += strlen(p1);
            p1 = &reply[n];
          }
        }
      }
    }
    reply[n++] = '\r';
    reply[n++] = '\n';
    reply[n] = '\0';
    if (usb_send_str(reply))
    {
      result = lineCnt > 0;
      moreLine = 0;
//       usb_send_str("#thread start\n");
    }
  }
  else
  {
    int n = miLines[moreLine + lineStartIndex].toString(reply, MRL);
    if (n > 2)
    { // a thread line may have no other attributes
      if (usb_send_str(reply))
      {
        moreLine++;
        result = moreLine < lineCnt;
      }
    }
    else
    {
      moreLine++;
      result = moreLine < lineCnt;
    }
//     usb_send_str("#thread line\n");
  }
  return result;
}

/////////////////////////////////////////////////////


/**
 * serach for use of IR sensor and line sensor */
void UMissionThread::getPowerUse ( bool* lineSensor, bool* irSensor, bool * chirpLog )
{
  for (int i = 0; i < lineCnt; i++)
  {
    UMissionLine * ml = &miLines[i + lineStartIndex];
    if (ml->edgeLUse or ml->edgeRUse or ml->lineValidUse /*or ml->lineValidUse*/ or ml->edgeWhiteUse)
      *lineSensor = true;
    if (ml->irDist1Use or ml->irDist2Use or ml->irSensorUse or ml->irDistRefUse)
      *irSensor = true;
    if (ml->chirp > 0)
      *chirpLog = true;
  }
}



/////////////////////////////////////////////////////

void UMissionThread::getToken()
{ // get token lines to console for debug
  const int MRL = 30;
  char reply[MRL];
  const int MSL = 100;
  char s[MSL];
  // first the thread number
  toTokenString(reply, MRL);
  snprintf(s, MSL, "#thread %d (%d) %s\r", threadNr, strlen(reply), reply);
  usb_send_str(s);
  // then the lines in the thread
  for (int i = 0; i < lineCnt; i++)
  {
    //usb_write("# line\n");
    miLines[i + lineStartIndex].toTokenString(reply, MRL);
    //usb_send_str(reply);
    snprintf(s, MSL, "#line %d (%d) %s\r", i, strlen(reply), reply);
    usb_send_str(s);
    //         m.decodeToken(&reply[1]);
    //         m.toString(&reply[1], MRL-1);
    //         usb_write(reply);
  }
  usb_send_str("<done tokens>\n");
}

///////////////////////////////////////////

int UMissionThread::toTokenString(char * bf, int bfCnt)
{
  int n = snprintf(bf, bfCnt, "%c%d", UMissionLine::MP_THREAD, threadNr);
  if (activateFlag or deactivateFlag)
  {
    char * p1 = &bf[n];
    if (activateFlag)
    {
      snprintf(p1, bfCnt - n - 1, "%c%lu", UMissionLine::MP_EVENT, activateFlag);
      n += strlen(p1);
      p1 = &bf[n];
    }
    if (deactivateFlag)
    { // add both ':' and stop-event(s)
      snprintf(p1, bfCnt - n - 1, ":%c%lu", UMissionLine::MC_EVENT, deactivateFlag);
      n += strlen(p1);
    }
  }
  // terminate with a new-line
  bf[n++] = '\n';
  bf[n] = '\0';
  return n;
}

///////////////////////////////////////////

bool UMissionThread::decodeToken(char * line, uint16_t * tn)
{ // decode tiken (from EE-flash)
//   // debug
//     const int MSL=120;
//     char s[MSL];
//   // debug end
  bool isOK = true;
  if (line[0] == UMissionLine::MP_THREAD)
  { // get thread number
    // tokens thread number is on a separate line
    char * p1 = &line[1];
    *tn = strtol(p1, &p1, 10);
    isOK = lineCnt == 0;
    if (isOK)
    {
      threadNr = *tn;
      //      debug
//       snprintf(s, MSL, "# got new thread %d isOK %d, tok:%s\r\n", threadNr, isOK, p1);
//       usb_send_str(s);
      //      debug end
      if (*p1 > ' ')
      { // there is more
        UMissionLine tmp;
        tmp.decodeToken(p1);
        // set start - stop flags directly - is 0 if unused
        activateFlag = tmp.eventSet;
        deactivateFlag = tmp.eventMask;
      }
    }
//     // debug
//      snprintf(s, MSL, "# got new thread %d isOK %d\r\n", threadNr, isOK);
//      usb_send_str(s);
//     // debug end
  }
  else
  { // not a thread number
    int idx = lineStartIndex + lineCnt;
    //     // debug
//      snprintf(s, MSL, "# pre  decode thread %d line %d idx=%d isOK %d, %s\r\n", threadNr, lineCnt, idx, isOK, line);
//      usb_send_str(s);
    //     // debug end
    isOK = miLines[idx].decodeToken(line);
//     // debug
//      snprintf(s, MSL, "# post decode thread %d line %d idx=%d isOK %d\r\n", threadNr, lineCnt, idx, isOK);
//      usb_send_str(s);
//     // debug end
    if (isOK)
    {
      lineCnt++;
      if (idx >= miLinesCnt)
        miLinesCnt++;
    }
  }
  return isOK;
}

////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////


void UMissionLine::clear()
{
  accUse = false;
  logUse = false;
  trUse = false;
  velUse = false;
  edgeLUse = false;
  edgeRUse = false;
  edgeWhiteUse = false;
  balUse = false;
  gotoUse = false;
  drivePosUse = false;
  irSensorUse = false;
  irDistRefUse = false;
  label = 0;
  eventSet = 0;
  servoID = 0; // not valid (1..5 is valid)
  servoPosition = 0;
  headUse = false;
  //
  distUse = false;
  timeUse = false;
  turnUse = false;
  countUse = false;
  xingUse = false;
//  xingUse = false;
  lineValidUse = '\0';
//   lineValidUse = '\0';
  irDist1Use = '\0';
  irDist2Use = '\0';
  tiltUse = false;
  eventMask = 0;
  logFullUse = false;
  headEndUse = '\0';
  servoVel = 0;
  velTestUse = false;
  reasonUse = '\0'; 
  // 
  valid = false;
  visits = 0;
  chirp = 0;
}

///////////////////////////////////////////////////

bool UMissionLine::finished(UMissionThread * state, 
                            uint16_t * labelNum, 
                            bool * endedAtEndAngle, 
                            char * continueReason,
                            char lastReason
                           )
{
  bool finished;
  bool condition = turnUse or distUse or timeUse or
      xingUse or xingUse or lineValidUse or //lineValidUse or
      irDist1Use or irDist2Use or eventMask or tiltUse or logFullUse;
  // test for finished with this line
  //usb_send_str("# Line::testFinished -> start\n");
  if (condition or gotoUse)
  { // there is a continue condition
    //usb_send_str("# Line::testFinished -> condition or goto\n");
    finished = (userMission.eventFlags & eventMask) > 0;
    if (finished)
      *continueReason = MC_EVENT;
//     if (finished)
//       usb_send_str("# Line::testFinished -> 2 finished\n");
//     else
//       usb_send_str("# Line::testFinished -> 2 not finished\n");
    if (not finished and turnUse)
    { // avoid angle folding
      float ta = pose[2] - state->turnSumLast;
      if (ta > M_PI)
        ta -= 2* M_PI;
      else if (ta < -M_PI)
        ta += 2 * M_PI; 
      state->turnAngSum += ta;
      state->turnSumLast = pose[2];
      //finished = misAngSum * 180/M_PI > fabs(misLine->turn);
    }
    if (not finished and distUse)
    { 
      if (distUse == '<')
        finished = fabsf(distance - state->linStartDist) < dist;
      else
        finished = fabsf(distance - state->linStartDist) >= dist;
      if (finished)
        *continueReason = MC_DIST;
    }
    if (not finished and velTestUse)
    { 
      if (velTestUse == '<')
        finished = (wheelVelocityEst[0] + wheelVelocityEst[1]) / 2  < velTest;
      else
        finished = (wheelVelocityEst[0] + wheelVelocityEst[1]) / 2 >= velTest;
      if (finished)
        *continueReason = MC_VEL;
    }
    if (not finished and timeUse)
    {
      finished = time < float(hbTimerCnt - state->misStartTime)/1000.0;
      if (finished)
        *continueReason = MC_TIME;
    }
    if (turnUse)
    {
      if (not finished)
      { // acc limit in turns
        float accAng = 0;
        float turnrad = fabsf(turn/180*M_PI);
        // acc limit in turns
        if (control.rateLimit < 100.0 and control.rateLimit > 0.1)
        { // use acceleration limit to reduce vel ref in final end of turn
          float velredmax = mission_vel_ref * odoWheelBase / (mission_turn_radius + odoWheelBase / 2.0);
          float accTime = velredmax / control.rateLimit;
          accAng = 0.5 * control.rateLimit * accTime * accTime / odoWheelBase;
          if (accAng > turnrad/2.0)
            accAng = turnrad/2.0;
        }
        finished = fabsf(state->turnAngSum) > (fabsf(turn)/180*M_PI - accAng);
        if (finished)
        {
          *endedAtEndAngle = true;
          *continueReason = MC_TURN;
        }
      }
      //             else
      //               mission_turn_ref = pose[2];
    }
    if (not finished and xingUse)
    {
      if (xingUse == '<')
        finished = crossingLineCnt < xingVal;
      else if (xingUse == '>')
        finished = crossingLineCnt > xingVal;
      else 
        finished = crossingLineCnt == xingVal;
      if (finished)
        *continueReason = MC_XING;
      //if (finished)
//         usb_send_str("# xing black\n");
    }
//     if (not finished and xingUse)
//     {
//       if (xingUse == '<')
//         finished = crossingLineCnt < xingVal;
//       if (xingUse == '>')
//         finished = crossingLineCnt > xingVal;
//       else
//         finished = crossingLineCnt == xingVal;
//       if (finished)
//         *continueReason = MC_XINGW;
      // debug
      //             if (crossingWhiteCnt > 0)
      //             {
      //               snprintf(s, MSL, "# xing white %d %d\n", crossingWhiteCnt, finished);
      //               usb_send_str(s);
      //             }
      // debug end
//    }
    if (not finished and lineValidUse)
    { // left valid test
      if (lineValidUse == '<')
        finished = lsEdgeValidCnt < lineValidVal;
      else if (lineValidUse == '>')
        finished = lsEdgeValidCnt > lineValidVal;
      else
        finished = lsEdgeValidCnt == lineValidVal;
      if (finished)
        *continueReason = MC_LINE_VALID;
    }
//     if (not finished and lineValidUse)
//     { // right edge valid
//       if (lineValidUse == '<')
//         finished = lsEdgeValidCnt < lineValidVal;
//       else if (lineValidUse == '>')
//         finished = lsEdgeValidCnt > lineValidVal;
//       else
//         finished = lsEdgeValidCnt == lineValidVal;
//       if (finished)
//         *continueReason = MC_LINE_VALID_R;
//     }
    if (not finished and irDist1Use)
    {
      if (irDist1Use == '<')
        finished = irDistance[0] < irDist1;
      else
        finished = irDistance[0] >= irDist1;
      if (finished)
        *continueReason = MC_IR_DIST1;
    }
    if (not finished and irDist2Use)
    {
      if (irDist2Use == '<')
        finished = irDistance[1] < irDist2;
      else
        finished = irDistance[1] >= irDist2;
      if (finished)
        *continueReason = MC_IR_DIST2;
    }
    if (not finished and logFullUse)
    {
      finished = logFull;
      if (finished)
        *continueReason = MC_LOG;
    }
    if (not finished and tiltUse)
    {
      if (tiltUse == '<')
        finished = pose[3] < tilt;
      else if (tiltUse == '>')
        finished = pose[3] >= tilt;
      else
        finished = fabs(tilt - pose[3]) < 1/180*M_PI;
      if (finished)
        *continueReason = MC_TILT;
    }
    if (not finished and headEndUse)
    {
      if (headEndUse == '<')
        finished = pose[2] < headEndValue;
      else if (headEndUse == '>')
        finished = pose[2] > headEndValue;
      else
      { // this heading reached
        float d = headEndValue - pose[2];
        if (d > M_PI)
          d -= 2*M_PI;
        if (d < -M_PI)
          d += 2*M_PI;
        finished = fabs(d) < (3 * M_PI / 180.0);
      }
      if (finished)
        *continueReason = MC_HEADING;
    }
    if (not finished and reasonUse)
    { // intended for use with '!' and goto, where goto is executed only if reason match,
      // in other lines a '=' is more likely to skip this line too
      if (reasonUse == '!')
        finished = reasonValue != lastReason;
      else if (reasonUse == '<')
        finished = reasonValue < lastReason;
      else if (reasonUse == '>')
        finished = reasonValue > lastReason;
      else
        finished = reasonValue == lastReason;
      if (finished)
        *continueReason = MC_REASON;
    }
    // with goto lines as the exception
    if (gotoUse)
    { // this is a goto-line
      if (true)
      { // count is incremented already
        if ((visits <= count or not countUse) and not finished)
        { // we need to jump, return the label number
          *labelNum = gotoDest;
        }
        else
        { // reset count
          visits = 0;
        }
      }
      // debug
//       const int MSL = 90;
//       char s[MSL];
//       snprintf(s, MSL, "#Line::finished visits=%d, count=%d (%d), *label=%d\n",
//                visits, count, countUse, *labelNum);
//       usb_send_str(s);
      // debug end
      finished = true;
    }
//     if (finished)
//       usb_write("# mission line - finished\n");
//     else
//       usb_write("# mission line - not finished\n");
  }
  else
  { // no condition (or just count), so continue right away
    finished = true;
    //usb_write("# mission line - no condition\n");
  }
  return finished;
}

///////////////////////////////////////////////////////

void UMissionLine::postProcess(float lineStartAngle, bool endAtAngle)
{
  // turn off one-liner controls
  // edge follow, wall follow and turn
  if (turnUse or edgeLUse or edgeRUse or trUse or irSensorUse)
  {
    if (headEndUse and headEndUse == '=')
    {
        mission_turn_ref = headEndValue * M_PI / 180;
    }
    else if (turnUse and endAtAngle)
    { // ended at an angle condition, so we know
      // that new angle ref should be exactly this angle
      mission_turn_ref = lineStartAngle + turn * M_PI / 180.0;
      while (mission_turn_ref > M_PI)
        mission_turn_ref -= 2 * M_PI;
      while (mission_turn_ref < -M_PI)
        mission_turn_ref += 2 * M_PI;
    }
    else 
    { // not a specific turn angle, but turning allowed, so use 
      // current heading as new reference
      mission_turn_ref = pose[2];
    }
    if (edgeLUse or edgeRUse)
    {
      regul_line_use = false;
      lsPostProcess();
    }
    else if (trUse)
      mission_turn_do = false;
    if (irSensorUse)
    { // ir-sensor drive control ended
      // both wall follow and velocity follow
      mission_irSensor_use = false;
      mission_wall_turn = false;
    }    
    // make sure old turn reference values are gone
    control.resetTurn();
  }
  // use of position controller is a one-liner also
  if (drivePosUse)
  {
    mission_pos_use = false;
    mission_vel_ref = 0;
  }
}



///////////////////////////////////////////////////////

void UMissionLine::implementLine()
{ // implement all line parameters
  // and note start state.
  //
  // tell control that control type may have changed
//   historyInvalid = false;
  // logging may change
  if (logUse and not logFull)
  { // logging may have changed
    if (log > 0)
    { // (re)start logging
      startLogging(log, false);
    }
    else
    { // pause logging, when set to log=0
      stopLogging();
    }
  }
  if (chirp > 0 and not control.chirpRun)
  { // 
    control.chirpStart(chirp, headUse);
    // debug
    const int MSL = 50;
    char s[MSL];
    snprintf(s, MSL, "#chirp start chirp=%d, head=%d, interval=%d, frq=%g\n", chirp, headUse, logInterval, control.chirpFrq);
    usb_send_str(s);
    // debug end
  }
  if (balUse)
    balance_active = bal;
  //
  if (irSensorUse)
  { // there is new 
    setIRpower(true);
    if (irSensor == 1 and not mission_wall_turn)
    { // sensor 1 is for wall follow
      mission_wall_turn = true; // turn is sensor 1 (side looking)
      // remove old historic values
      control.ctrlWallTurn->resetControl();
    }
    else if (mission_wall_turn)// sensor 2
    { // we are dooing - follow the leader - i.e. speed control
      mission_wall_turn = false; // is sensor 2 (fwd looking)
      control.ctrlWallVel->resetControl();
    }
    if (irDistRefUse)
    {
      mission_wall_ref = irDistRef;
      mission_wall_vel_ref = irDistRef;
    }
    // use ir sensor value as bolean 
    // irsensor=1: turn (follow wall), 
    // irsensor=2: distance based on sensor 2 only, 
    // irsensor=3: distance based on both irsensor 1 and 2 (minimum)
    mission_irSensor_use = irSensor;
  }
//   else
//     // save power (may be this is a bad idea?)
//     // yes - not thread safe
//     setIRpower(false);
//   else
//     mission_wall_use = false;
  //
  if (velUse)
  {  // change velocity
    mission_vel_ref = vel;
    //usb_send_str("# set mission_vel_ref\n");
  }
  if (drivePosUse)
  {
    mission_pos_ref = drivePos;
    misStartDist = distance;
    if (not mission_pos_use)
    {
      mission_pos_use = true;
      control.ctrlPos->resetControl();
    }
  }
  // edge/line sensor
  if (xingUse or lineValidUse or edgeLUse or edgeRUse)
  {  // turn on sensor for crossing detect
    lineSensorOn = true;
    if (edgeLUse or edgeRUse)
    { // Remove potentially old history values - inclusive integrators
      // as edge, color or offset may have changed
      //
      // resetControl seems to make things worse, when driving at differet speed, but on same line
      // control.ctrlEdge->resetControl();
      //
      // activate control
      regul_line_use = true;
      // set edge to follow
      mission_line_LeftEdge = edgeLUse;
      // set offset to edge
      mission_line_ref = edgeRef;
      // line color
      if (edgeWhiteUse)
        lsIsWhite = edgeWhite;
    }
  }
//   else
// not a good idea, as other threads may use the line sensor
//     lineSensorOn = false;
  
//   else if (regul_line_use)
//   { // set heading ref to as is now
//     regul_line_use = false;
//   }
  // turn control
  if (trUse)
  { // turn radius in meters positive i left
    mission_turn_radius = tr;
    mission_turn_do = true;
    // set turn angle
    mission_tr_turn = turn;
    // reduce reference velocity
    // mission_vel_ref_preturn = mission_vel_ref;
    // reduce reference (mid-point) velocity and allow outher wheel to keep old velocity
    // especially when turn radius is very small -> 0
    // ¤¤¤              mission_vel_ref = mission_vel_ref_preturn * mission_turn_radius / 
    //                 (mission_turn_radius + odoWheelBase/2.0);
  }
//   else
//   { // no turn this time - keep current turn ref
//     mission_turn_do = false;
//     //               regul_turn_vel_reduc[0] = 0;
//     //               regul_turn_vel_reduc[1] = 0;
//   }
  if (accUse)
    control.setRateLimit(acc);
  if (eventSet > 0)
  { // there is at least one user event, activate
    for (int i = 0; i < 32; i++)
    { // test all 32 possible user events
      if ((eventSet & (1 << i)) > 0)
        userMission.setEvent(i);
    }
  }
  if (servoID > 0)
  {
    servo.setServo(servoID, servoPosition, true, servoVel);
  }
  if (headUse)
  { // set reference heading
    mission_turn_ref = headValue * M_PI / 180.0;
    while (mission_turn_ref >= M_PI)
      mission_turn_ref -= 2 * M_PI;
    while (mission_turn_ref < -M_PI)
      mission_turn_ref += 2 * M_PI;
  }
}


///////////////////////////////////////////////////////

int UMissionLine::toString(char* bf, int bfCnt, bool frame)
{
  char * ms = bf;
  char * mc;
  int n = 0;
  const char sep = ',';
  if (not valid)
  {
    strncpy(ms, "\r# not a valid line", bfCnt - n);
    n += strlen(ms);
    ms = &bf[n];
  }
  else if (frame)
  {
    strcpy(ms, "<m ");
    n+=3;
    ms = &bf[n];
  }
  if (velUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n, "vel=%g", vel);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (accUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n, "acc=%g", acc);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (trUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "tr=%g", tr);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (edgeLUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "edgeL=%g", edgeRef);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (edgeRUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "edgeR=%g", edgeRef);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (edgeWhiteUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "white=%d", edgeWhite);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (logUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "log=%g", log);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (balUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "bal=%d", bal);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (irSensorUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "irsensor=%d", irSensor);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (irDistRefUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "irdist=%g", irDistRef);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (drivePosUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "topos=%g", drivePos);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (headUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "head=%g", headValue);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (chirp > 0)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "chirp=%g", chirp/100.0);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (servoID > 0)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "servo=%d", servoID);
    n += strlen(ms);
    ms = &bf[n];
    *ms++=sep; n++;
    snprintf(ms, bfCnt - n - 1, "pservo=%d", servoPosition);
    n += strlen(ms);
    ms = &bf[n];
    *ms++=sep; n++;
    snprintf(ms, bfCnt - n - 1, "aservo=%d", servoVel);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (label > 0)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "label=%d", label);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (gotoUse)
  {
    if (n > 3) {*ms++=sep; n++;}
    snprintf(ms, bfCnt - n - 1, "goto=%d", gotoDest);
    n += strlen(ms);
    ms = &bf[n];
  }
  if (eventSet)
  { // there  is at least one event set on this line
    // 
//     const int MSL = 100;
//     char s[MSL];
//     snprintf(s, MSL,"# eventSet (%lu) to string\n", eventSet);
//     usb_send_str(s);
    // debug end
    for (int i = 0; i < 32; i++)
    { // test all possible events
      if (eventSet & (1 << i))
      { // event i is set, add to line
        if (n > 3) {*ms++=sep; n++;}
        snprintf(ms, bfCnt - n - 1, "event=%d", i);
        n += strlen(ms);
        ms = &bf[n];
      }
    }
  }
  // now the condition part - if it exist
  mc = ms;
  *mc++ = ':';
  n++;
  if (distUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "dist%c%g", distUse, dist);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (velTestUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "vel%c%g", velTestUse, velTest);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (timeUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "time%c%g", timeUse, time);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (turnUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "turn%c%g", turnUse, turn);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (countUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "count%c%d", countUse, count);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (xingUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "xl%c%d", xingUse, xingVal);
    n += strlen(mc);
    mc = &bf[n];
  }
//   if (xingUse)
//   {
//     if (mc - ms > 1) {*mc++=sep; n++;}
//     snprintf(mc, bfCnt - n - 1, "xw%c%d", xingUse, xingVal);
//     n += strlen(mc);
//     mc = &bf[n];
//   }
  if (lineValidUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "lv%c%d", lineValidUse, lineValidVal);
    n += strlen(mc);
    mc = &bf[n];
  }
//   if (lineValidUse)
//   {
//     if (mc - ms > 1) {*mc++=sep; n++;}
//     snprintf(mc, bfCnt - n - 1, "lv%c%d", lineValidUse, lineValidVal);
//     n += strlen(mc);
//     mc = &bf[n];
//   }
  if (irDist1Use)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "ir1%c%g", irDist1Use, irDist1);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (irDist2Use)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "ir2%c%g", irDist2Use, irDist2);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (tiltUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "tilt%c%g", tiltUse, tilt);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (headEndUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "head%c%g", headEndUse, headEndValue);
    n += strlen(mc);
    mc = &bf[n];
  }
  if (eventMask)
  {
    for (int i = 0; i < 32; i++)
    {
      if (eventMask & (1 << i))
      {
        if (n > 3) {*mc++=sep; n++;}
        snprintf(mc, bfCnt - n - 1, "event=%d", i);
        n += strlen(mc);
        mc = &bf[n];
      }
    }
  }
  if (logFullUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "log=0");
    n += strlen(mc);
    mc = &bf[n];
  }
  if (reasonUse)
  {
    if (mc - ms > 1) {*mc++=sep; n++;}
    snprintf(mc, bfCnt - n - 1, "last%c%d", reasonUse, reasonValue - MC_DIST);
    n += strlen(mc);
    mc = &bf[n];
//     usb_send_str("#send a reason line\n");
  }
  if (frame)
  {
    strncpy(mc, "\n\r", bfCnt - n - 1);
    n += strlen(mc);
  }
  return n;
}



/////////////////////////////////////////////////

int UMissionLine::toTokenString(char* bf, int bfCnt)
{
  char * ms = bf;
  char * mc;
  int n = 0;
//   const char sep = ',';
  if (not valid)
  {
    ms[0] = '\0';
  }
  else
  {
    if (velUse)
    {
      snprintf(ms, bfCnt - n, "%c%g", MP_VEL, vel);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (accUse)
    {
      snprintf(ms, bfCnt - n, "%c%g", MP_ACC, acc);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (trUse)
    {
      snprintf(ms, bfCnt - n - 1, "%c%g", MP_TR, tr);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (edgeWhiteUse)
    {
      snprintf(ms, bfCnt - n - 1, "%c%d", MP_EDGE_WHITE, edgeWhite);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (edgeLUse)
    {
      snprintf(ms, bfCnt - n - 1, "%c%g", MP_EDGE_L, edgeRef);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (edgeRUse)
    {
      snprintf(ms, bfCnt - n - 1, "%c%g", MP_EDGE_R, edgeRef);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (logUse)
    {
      snprintf(ms, bfCnt - n - 1, "%c%g", MP_LOG, log);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (balUse)
    {
      snprintf(ms, bfCnt - n - 1, "%c%d", MP_BAL, bal);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (irSensorUse)
    {
      snprintf(ms, bfCnt - n - 1, "%c%d", MP_IR_SENSOR, irSensor);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (irDistRefUse)
    {
      snprintf(ms, bfCnt - n - 1, "%c%g", MP_IR_DIST, irDistRef);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (drivePosUse)
    {
      snprintf(ms, bfCnt - n - 1, "%c%g", MP_DRIVE_DIST, drivePos);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (label > 0)
    {
      snprintf(ms, bfCnt - n - 1, "%c%d", MP_LABEL, label);
      n += strlen(ms);
      ms = &bf[n];
    }
    if (gotoUse)
    {
      snprintf(ms, bfCnt - n - 1, "%c%d", MP_GOTO, gotoDest);
      n += strlen(ms);
      ms = &bf[n]; 
//       usb_send_str("#gotoline\n");
//       usb_send_str(bf);
//       usb_send_str("\n");
    }
    if (eventSet)
    {
      snprintf(ms, bfCnt - n - 1, "%c%lu", MP_EVENT, eventSet);
      n += strlen(ms);
      ms = &bf[n];
    }      
    if (headUse)
    {
      snprintf(ms, bfCnt - n - 1, "%c%g", MP_HEADING, headValue);
      n += strlen(ms);
      ms = &bf[n];
    }      
    if (chirp > 0)
    {
      snprintf(ms, bfCnt - n - 1, "%c%d", MP_CHIRP, chirp);
      n += strlen(ms);
      ms = &bf[n];
    }      
    if (servoID > 0)
    {
      snprintf(ms, bfCnt - n - 1, "%c%d", MP_SERVO, servoID);
      n += strlen(ms);
      ms = &bf[n];
      snprintf(ms, bfCnt - n - 1, "%c%d", MP_SERVO_POS, servoPosition);
      n += strlen(ms);
      ms = &bf[n];
      snprintf(ms, bfCnt - n - 1, "%c%d", MP_SERVO_VEL, servoVel);
      n += strlen(ms);
      ms = &bf[n];
    }      
    // now the condition part - if it exist
    mc = ms;
    *mc++ = ':';
    n++;
    if (distUse)
    {
      snprintf(mc, bfCnt - n - 1, "%c%c%g", MC_DIST, distUse, dist);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (velTestUse)
    {
      snprintf(mc, bfCnt - n - 1, "%c%c%g", MC_VEL, velTestUse, velTest);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (timeUse)
    {
      snprintf(mc, bfCnt - n - 1, "%c%c%g", MC_TIME, timeUse, time);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (turnUse)
    {
      snprintf(mc, bfCnt - n - 1, "%c%c%g", MC_TURN, turnUse, turn);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (countUse)
    {
      snprintf(mc, bfCnt - n - 1, "%c%c%d", MC_COUNT, countUse, count);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (xingUse)
    {
      snprintf(mc, bfCnt - n - 1, "%c%c%d", MC_XING, xingUse, xingVal);
      n += strlen(mc);
      mc = &bf[n];
    }
//     if (xingUse)
//     {
//       snprintf(mc, bfCnt - n - 1, "%c%c%d", MC_XINGW, xingUse, xingVal);
//       n += strlen(mc);
//       mc = &bf[n];
//     }
    if (lineValidUse)
    {
      snprintf(mc, bfCnt - n - 1, "%c%c%d", MC_LINE_VALID, lineValidUse, lineValidVal);
      n += strlen(mc);
      mc = &bf[n];
    }
//     if (lineValidUse)
//     {
//       snprintf(mc, bfCnt - n - 1, "%c%c%d", MC_LINE_VALID_R, lineValidUse, lineValidVal);
//       n += strlen(mc);
//       mc = &bf[n];
//     }
    if (irDist1Use)
    {
      snprintf(mc, bfCnt - n - 1, "%c%c%g", MC_IR_DIST1, irDist1Use, irDist1);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (irDist2Use)
    {
      snprintf(mc, bfCnt - n - 1, "%c%c%g", MC_IR_DIST2, irDist2Use, irDist2);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (tiltUse)
    {
      snprintf(mc, bfCnt - n - 1, "%c%c%g", MC_TILT, tiltUse, tilt);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (logFullUse)
    {
      snprintf(mc, bfCnt - n - 1, "%c0", MC_LOG);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (eventMask)
    {
      snprintf(mc, bfCnt - n - 1, "%c%lu", MC_EVENT, eventMask);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (headEndUse)
    {
      snprintf(mc, bfCnt - n - 1, "%c%c%g", MC_HEADING, headEndUse, headEndValue);
      n += strlen(mc);
      mc = &bf[n];
    }
    if (reasonUse)
    {
      snprintf(mc, bfCnt - n - 1, "%c%c%d", MC_REASON, reasonUse, reasonValue - MC_DIST);
      n += strlen(mc);
      mc = &bf[n];
//       usb_send_str("#Mission send a reason line\n");
    }
    strncpy(mc, "\n", bfCnt - n - 1);
    n += strlen(mc);
  }
  return n;
}




////////////////////////////////////////////////////////////////

bool UMissionLine::decodeLine(const char* buffer, int16_t * threadNumber)
{
  char * p1 = (char *)buffer;
  char * p2 = strchr(p1, ':');
  char * p3 = strchr(p1, '=');
  bool err = false;
  // reset use flags
  clear();
  missionErrStr[0] = '\0';
  // strip white space
  while (*p1 <= ' ' and *p1 > '\0') p1++;
  // find all parameters until ':'
  while ((p1 < p2 or p2 == NULL) and p3 != NULL and not err)
  {
    // debug
//     const int MSL = 100;
//     char s[MSL];
//     snprintf(s, MSL, "#Decoding line p1='%s'\n # p3 = '%s'\n", p1, p3);
//     usb_send_str(s);
    // debug end
    if (strncmp (p1, "acc", 3) == 0)
    {
      accUse = true;
      acc = strtof(++p3, &p1);
    }
    else if (strncmp (p1, "vel", 3) == 0)
    {
      velUse = true;
      vel = strtof(++p3, &p1);
    }
    else if (strncmp(p1, "tr", 2) == 0)
    {
      trUse = true;
      tr = fabsf(strtof(++p3, &p1));
    }
    else if (strncmp(p1, "edgel", 5) == 0 or strncmp(p1, "liner", 5) == 0)
    {
      edgeLUse = true;
      edgeRef = strtof(++p3, &p1);
      if (edgeRef > 2.0)
        edgeRef = 2.0;
      if (edgeRef < -2.0)
        edgeRef = -2.0;
    }
    else if (strncmp(p1, "edger", 5) == 0 or strncmp(p1, "liner", 5) == 0)
    {
      edgeRUse = true;
      edgeRef = strtof(++p3, &p1);
      if (edgeRef > 2.0)
        edgeRef = 2.0;
      if (edgeRef < -2.0)
        edgeRef = -2.0;
    }
    else if (strncmp(p1, "white", 5) == 0)
    {
      edgeWhiteUse = true;
      edgeWhite = strtol(++p3, &p1, 10);
      if (edgeWhite)
        edgeWhite = true;
    }
    else if (strncmp (p1, "log", 3) == 0)
    {
      logUse = true;
      log = strtof(++p3, &p1);
    }
    else if (strncmp (p1, "bal", 3) == 0)
    {
      balUse = true;
      bal = strtof(++p3, &p1) > 0.5;
    }
    else if (strncmp (p1, "irsensor", 8) == 0)
    {
      irSensorUse = true;
      irSensor = strtol(++p3, &p1, 10);
      if (irSensor < 1)
        // default is follow wall
        irSensor = 1;
    }
    else if (strncmp (p1, "irdist", 6) == 0)
    {
      irDistRefUse = true;
      irDistRef = strtof(++p3, &p1);
    }
    else if (strncmp (p1, "topos", 5) == 0)
    {
      drivePosUse = true;
      drivePos = strtof(++p3, &p1);
    }
    else if (strncmp (p1, "label", 5) == 0)
    {
      label = strtol(++p3, &p1, 10);
    }
    else if (strncmp (p1, "goto", 4) == 0)
    {
      // debug
//       const int MSL = 100;
//       char s[MSL];
//       snprintf(s, MSL, "#Decoding goto p1='%s', p3 = '%s'\n", p1, p3);
//       usb_send_str(s);
      // debug end
      gotoUse = true;
      gotoDest = strtol(++p3, &p1, 0);
//       snprintf(s, MSL, "#Decoding goto use=%d, dest = %d\n", gotoUse, gotoDest);
//       usb_send_str(s);
    }
    else if (strncmp (p1, "thread", 6) == 0)
    {
      if (threadNumber != NULL)
        *threadNumber = strtol(++p3, &p1, 0);
    }
    else if (strncmp (p1, "event", 5) == 0)
    {
      int s = strtol(++p3, &p1, 0);
      if (s >= 0 and s < 32)
        eventSet |= 1 << s;
    }
    else if (strncmp (p1, "head", 4) == 0)
    {
      headUse = true;
      headValue = strtof(++p3, &p1);
    }
    else if (strncmp (p1, "chirp", 5) == 0)
    { // convert to deci values (to save line space)
      chirp = int(strtof(++p3, &p1) * 100);
    }
    else if (strncmp (p1, "servo", 5) == 0)
    {
      int s = strtol(++p3, &p1, 0);
      if (s <= 5 and s > 0)
      {
        servoID = s;
//         usb_send_str("# servo line added");
      }
    }
    else if (strncmp (p1, "pservo", 6) == 0)
    {
      int s = strtol(++p3, &p1, 0);
      servoPosition = s;
//       usb_send_str("# servo position added");
    }
    else if (strncmp (p1, "vservo", 6) == 0)
    {
      int s = strtol(++p3, &p1, 0);
      servoVel = s;
//       usb_send_str("# servo velocity added");
    }
    else
    { // error, just skip
      snprintf(missionErrStr, missionErrStrMaxCnt, "failed parameter at %s", p1);
      p1 = ++p3;
      err = true;
    }
    // remove white space
    while ((*p1 <= ' ' or *p1 == ',') and *p1 > '\0') p1++;
    p3 = strchr(p1, '=');
    
  }
  // now the part after the ':', where p2 is pointing.
  // - if there is a ':'
  if (p2 != NULL)
  { // there might be a condition part
    p1 = p2 + 1;
    while (*p1 <= ' ' and *p1 > '\0') p1++;
    p3 = p1; // start of token
    // skip token until operator or reached end or next parameter (error)
    while (strchr("=<>!", *p3) == NULL and *p3 >= ' ' and *p3 != ',') p3++;
    if (strchr("=<>!", *p3) == NULL)
    { // not a valid operator
      snprintf(missionErrStr, missionErrStrMaxCnt, "invalid operator %s (pos %d)", p3, p3 - buffer);
      err = true;
    }
//     p3 = strchr(p1, '=');
//     if (p3 == NULL)
//       p3 = strchr(p1, '<');
//     if (p3 == NULL)
//       p3 = strchr(p1, '!');    
//     if (p3 == NULL)
//       p3 = strchr(p1, '>');    
    while (*p1 > '\0' and p3 != NULL and not err)
    {
      // debug
//        const int MSL = 100;
//        char s[MSL];
//        snprintf(s, MSL, "#Decoding condition p1='%s', p3 = '%s'\n", p1, p3);
//        usb_send_str(s);
      // debug end
      if (strncmp (p1, "dist", 4) == 0)
      { // distance is always positive (even if reversing)
        distUse = *p3;
        dist = fabsf(strtof(++p3, &p1));
      }
      else if (strncmp (p1, "vel", 3) == 0)
      { // average velocity of robot center (including sign)
        velTestUse = *p3;
        velTest = strtof(++p3, &p1);
      }
      else if (strncmp (p1, "turn", 4) == 0)
      {
        turnUse = *p3;
        turn = strtof(++p3, &p1);
      }
      else if (strncmp (p1, "time", 4) == 0)
      {
        timeUse = *p3;
        time = strtof(++p3, &p1);
      }
      else if (strncmp (p1, "count", 4) == 0)
      {
        countUse = *p3;
        count = strtol(++p3, &p1, 10);
      }
      else if (strncmp (p1, "xl", 2) == 0)
      {
//         snprintf(s, MSL, "# XB p1='%s' p3 = '%s'\n", p1, p3);
//         usb_send_str(s);        
        xingUse = *p3;
        xingVal = strtol(++p3, &p1, 10);
      }
//       else if (strncmp (p1, "xw", 2) == 0)
//       {
// //         snprintf(s, MSL, "# XW p1='%s' p3 = '%s'\n", p1, p3);
// //         usb_send_str(s);        
//         xingUse = *p3;
//         xingVal = strtol(++p3, &p1, 10);
//       }
      else if (strncmp (p1, "lv", 2) == 0)
      {
//         snprintf(s, MSL, "# XB p1='%s' p3 = '%s'\n", p1, p3);
//         usb_send_str(s);        
        lineValidUse = *p3;
        lineValidVal = strtol(++p3, &p1, 10);
      }
//       else if (strncmp (p1, "rv", 2) == 0)
//       { // NB! line valid is for both edges, so "rv" is not needed.
//         //         snprintf(s, MSL, "# XB p1='%s' p3 = '%s'\n", p1, p3);
//         //         usb_send_str(s);        
//         lineValidUse = *p3;
//         lineValidVal = strtol(++p3, &p1, 10);
//       }
      else if (strncmp (p1, "ir1", 3) == 0)
      {
        irDist1Use = *p3;
        irDist1 = strtof(++p3, &p1);
        //usb_send_str("#ir1\n");
      }
      else if (strncmp (p1, "ir2", 3) == 0)
      {
        irDist2Use = *p3;
        irDist2 = strtof(++p3, &p1);
        //usb_send_str("#ir2\n");
      }
      else if (strncmp (p1, "tilt", 4) == 0)
      {
        tiltUse = *p3;
        tilt = strtof(++p3, &p1);
      }
      else if (strncmp (p1, "log", 3) == 0)
      {
        logFullUse = true;
        strtol(++p3,&p1, 10);//skip zero (or any number)
        //p1 = p3 + 2; // skip zero
      }
      else if (strncmp (p1, "event", 5) == 0)
      {
        int s = strtol(++p3, &p1, 0);
        if (s < 32 and s >= 0)
          eventMask |= 1 << s;
      }
      else if (strncmp (p1, "head", 4) == 0)
      {
        headEndUse = *p3;
        headEndValue = strtof(++p3, &p1);
      }
      else if (strncmp (p1, "last", 4) == 0)
      {
        if (*p3 == '!' or *p3 == '<' or *p3 == '>')
          reasonUse = *p3;
        else
          reasonUse = '=';
        reasonValue = strtol(++p3, &p1, 10) + MC_DIST;
//         usb_send_str("#found a last\n");
      }
      else
      { // error, just skip
        snprintf(missionErrStr, missionErrStrMaxCnt, "failed condition at %s (pos %d)", p1, p1 - buffer);
        p1 = ++p3;
        err = true;
      }
      // remove white space
      while ((*p1 <= ' ' or *p1 == ',') and *p1 > '\0') p1++;
      p3 = p1; // start of token
      // skip token until operator or reached end or next parameter (error)
      while (strchr("=<>!", *p3) == NULL and *p3 >= ' ' and *p3 != ',') p3++;
      if (strchr("=<>!", *p3) == NULL)
      { // not a valid operator
        snprintf(missionErrStr, missionErrStrMaxCnt, "invalid operator 2 %s (pos %d)", p3, p3 - buffer);
        err = true;
      }
//       p3 = strchr(p1, '=');
//       if (p3 == NULL)
//         p3 = strchr(p1, '<');
//       if (p3 == NULL)
//         p3 = strchr(p1, '>');    
//       if (p3 == NULL)
//         p3 = strchr(p1, '!');    
    }
  }
  valid = not err;
  
//   if (true)
//   {
//     const int MSL = 180;
//     char s[MSL];
//     snprintf(s, MSL, "# decoded ir1Use=%c, ir2use=%c from %s\n", irDist1Use, irDist2Use, buffer);
//     usb_send_str(s);
//   }
  
  return valid;
}

bool UMissionLine::decodeToken(const char* buffer)
{
  char * p1 = (char *)buffer;
  char * p2 = strchr(p1, ':');
  bool err = false;
  uint8_t n = 255;
  // reset use flags
  clear();
//     const int MSL = 120;
//     char  s[MSL];
//   missionErrStr[0]='\0';
  // find all parameters until ':'
  while ((p1 < p2 or p2==NULL) and *p1 >=' ')
  {
    // debug
//      snprintf(s, MSL, "#pi1=%s ¤ p2=%s\r\n", p1, p2);
//      usb_send_str(s);
    // denug end
    if (*p1 == MP_ACC) 
    {
      accUse = true;
      acc = strtof(++p1, &p1);
    }
    else if (*p1 == MP_VEL)
    {
      velUse = true;
      vel = strtof(++p1, &p1);
    }
    else if (*p1 == MP_TR)
    {
      trUse = true;
      tr = fabsf(strtof(++p1, &p1));
    }
    else if (*p1 == MP_EDGE_L)
    {
      edgeLUse = true;
      edgeRef = strtof(++p1, &p1);
    }
    else if (*p1 == MP_EDGE_R)
    {
      edgeRUse = true;
      edgeRef = strtof(++p1, &p1);
    }
    else if (*p1 == MP_EDGE_WHITE)
    {
      edgeWhiteUse = true;
      edgeWhite = strtol(++p1, &p1, 10);
    }
    else if (*p1 == MP_LOG)
    {
      logUse = true;
      log = strtof(++p1, &p1);
    }
    else if (*p1 == MP_BAL)
    {
      balUse = true;
      bal = strtol(++p1, &p1, 10);
    }
    else if (*p1 == MP_IR_SENSOR)
    {
      irSensorUse = true;
      irSensor = strtol(++p1, &p1, 10);
    }
    else if (*p1 == MP_IR_DIST)
    {
      irDistRefUse = true;
      irDistRef = strtof(++p1, &p1);
    }
    else if (*p1 == MP_DRIVE_DIST)
    {
      drivePosUse = true;
      drivePos = strtof(++p1, &p1);
    }
    else if (*p1 == MP_LABEL)
    {
      label = strtol(++p1, &p1, 10);
    }
    else if (*p1 == MP_GOTO)
    {
      gotoUse = true;
      gotoDest = strtol(++p1, &p1, 10);
    }
    else if (*p1 == MP_EVENT)
    {
      eventSet = strtol(++p1, &p1, 10);
    }
    else if (*p1 == MP_HEADING)
    {
      headUse = true;
      headValue = strtof(++p1, &p1);
    }
    else if (*p1 == MP_CHIRP)
    {
      chirp = strtol(++p1, &p1, 10);
    }
    else if (*p1 == MP_SERVO)
    {
      servoID = strtof(++p1, &p1);
    }
    else if (*p1 == MP_SERVO_POS)
    {
      servoPosition = strtof(++p1, &p1);
    }
    else if (*p1 == MP_SERVO_VEL)
    {
      servoVel = strtof(++p1, &p1);
    }
    else if (*p1 > ' ')
    { // error, just skip
      err = true;
      snprintf(missionErrStr, missionErrStrMaxCnt, "failed line P at %s\n", p1);
      break;
    }
    // remove seperator
//     if (*p1 == ',')
//       p1++;
  }
  if (p2 != NULL and not err)
  {
    p1 = p2 + 1;
    while (*p1 > ' ')
    {
      // debug
//       snprintf(s, MSL, "#pi1=%s\n", p1);
//       usb_send_str(s);
      // debug end
      if (*p1 == MC_DIST)
      { // distance is always positive (even if reversing)
        distUse = *(++p1);
        dist = fabsf(strtof(++p1, &p1));
      }
      else if (*p1 == MC_VEL)
      { // distance is always positive (even if reversing)
        velTestUse = *(++p1);
        velTest = strtof(++p1, &p1);
      }
      else if (*p1 == MC_TURN)
      {
        turnUse = *(++p1);
        turn = strtof(++p1, &p1);
      }
      else if (*p1 == MC_TIME)
      {
        timeUse = *(++p1);
        time = strtof(++p1, &p1);
      }
      else if (*p1 == MC_COUNT)
      {
        countUse = *(++p1);;
        count = strtol(++p1, &p1, 10);
      }
      else if (*p1 == MC_XING)
      {
        xingUse = *(++p1);
        xingVal = strtol(++p1, &p1, 10);
      }
//       else if (*p1 == MC_XINGW)
//       {
//         xingUse = *(++p1);
//         xingVal = strtol(++p1, &p1, 10);
//       }
      else if (*p1 == MC_LINE_VALID)
      {
        lineValidUse = *(++p1);
        lineValidVal = strtol(++p1, &p1, 10);
      }
      else if (*p1 == MC_LINE_VALID)
      {
        lineValidUse = *(++p1);
        lineValidVal = strtol(++p1, &p1, 10);
      }
      else if (*p1 == MC_IR_DIST1)
      {
        irDist1Use = *(++p1);
        irDist1 = strtof(++p1, &p1);
      }
      else if (*p1 == MC_IR_DIST2)
      {
        irDist2Use = *(++p1);
        irDist2 = strtof(++p1, &p1);
      }
      else if (*p1 == MC_TILT)
      {
        tiltUse = *(++p1);
        tilt = strtof(++p1, &p1);
      }
      else if (*p1 == MC_LOG)
      {
        logFullUse = true;
        p1++;
//         p1++;
        // ignore value
        while (isdigit(*p1))
          p1++;
      }
      else if (*p1 == MC_EVENT)
      {
        eventMask = strtol(++p1, &p1, 10);
      }
      else if (*p1 == MC_HEADING)
      {
        headEndUse = *(++p1);
        headEndValue = strtof(++p1, &p1);
      }
      else if (*p1 == MC_REASON)
      {
        reasonUse = *(++p1);
        reasonValue = strtol(++p1, &p1, 10) + MC_DIST;
      }
      else
      { // error, just skip
        err = true;
        snprintf(missionErrStr, missionErrStrMaxCnt, "failed line C at %s (n=%d)\n", p1, n);
        break;
      }
//       if (*p1 > ' ' and *p1 == ',')
//         p1++;
    }
  }
  valid = not err;
  return valid;
}


/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////
/////////////////////////////////////////////////

bool UMission::testFinished()
{
  bool theEnd = true;
  bool lineFinished[threadsMax];
//  bool debugOut = false;
  for (int i = 0; i < threadsCnt; i++)
  {
    lineFinished[i] = threads[i].testFinished();
    // debug
//     if (lineFinished[i])
//     {
//       const int MSL=50;
//       char s[MSL];
//       snprintf(s, MSL, "# testFinished -> finished thread %d (%d)\n", threads[i].threadNr, i);
//       usb_send_str(s);
//     }
    // debug end
//    debugOut |= lineFinished[i];
  }
//   if (debugOut)
//   {
//     const int MSL = 50;
//     char s[MSL];
//     for (int i=0; i < threadsCnt; i++)
//     {
//       snprintf(s, MSL, "#line end th:%d line:%d finished:%d\n",
//                i, threads[i].misLineNum, lineFinished[i]);
//       usb_send_str(s);
//     }
//   }
  // save ebvent for log (reset by stateToLog())
  eventFlagsSavedLog = eventFlagsSavedLog | eventFlags;
  // reset events before implementing next line
//   eventFlagsSaved1 = eventFlags;
  eventFlags = 0;
//  usb_send_str("# testFinished -> events cleared\n");
  // implement next line (if relevant)
  for (int i = 0; i < threadsCnt; i++)
  {
    if (lineFinished[i])
    {
      theEnd &= threads[i].moveToNextLine();
      // debug
//       if (theEnd)
//       {
//         const int MSL=90;
//         char s[MSL];
//         snprintf(s, MSL, "# thread %d finished all lines\n", i);
//         usb_send_str(s);
//       }
      // debug end
    }
    else
      theEnd = false;
  }
  // event flag 0 has special meening : stop mission NOW!
  if ((eventFlags & 0x01) > 0)
  {
    theEnd = true;
    usb_send_str("# the end - event=0\n");
  }
  return theEnd;
}


bool UMission::startMission()
{
  // debug
//   const int MSL = 120;
//   char s[MSL];
  // debug end
  bool isOK = threadsCnt > 0;
  if (isOK)
  {
    time = 0.0;
    clearPose();
    balance_active = false;
    mission_vel_ref = 0;
    mission_turn_ref = 0;
//     // reset log buffer (but do not start logging)
//     initLogStructure(50);
//     // clear extra logger data (may not be used)
//     memset(dataloggerExtra, 0, sizeof(dataloggerExtra));
    logAllow = true;
    toLog = false;
    lineSensorOn = false;
    mission_wall_turn = false;
    mission_irSensor_use = false;
    regul_line_use = false;
    control.setRateLimit(100);
    motorSetEnable(true, true);
    //
    for (int i = 0; i < threadsCnt; i++)
    {
      threads[i].startMission();
    }
    usb_send_str("#** Mission started\n");
  }
  return isOK;
}

void UMission::stopMission()
{
  for (int i = 0; i < threadsCnt; i++)
  {
    threads[i].stopMission();
  }
}


///////////////////////////////////////////////////

// bool UMission::eeAddBlock(char * data, int dataCnt)
// {
//   if (eeConfig.getAddr() + dataCnt < 2048 - 2)
//   {
//     eeConfig.busy_wait();
//     eeConfig.write_block(data, dataCnt);
//     return true;
//   }
//   else
//     return false;
// }

///////////////////////////////////////////////////////

void UMission::eePromSaveMission()
{
  const int MRL = 100;
  char reply[MRL];
//   const int MSL = 100;
//   char s[MSL];
  int n, i = 0, t = 0;
  uint32_t adr = eeConfig.getAddr();
  bool isOK = true;
  // reserve space for size
  eeConfig.setAddr(adr + 2);
  //eePushAdr += 2;
  for (t = 0; t < threadsCnt; t++)
  {
    UMissionThread * mt = &threads[t];
    n = mt->toTokenString(reply, MRL);
    isOK = eeConfig.pushBlock(reply, n);
//     // debug
//      snprintf(reply, MRL, "# saving thread %d (%d), %d line(s) ...\r\n", t, mt->threadNr, mt->lineCnt);
//      usb_send_str(reply);
//     // debug end
    for (i = 0; i < mt->lineCnt and isOK; i++)
    {
      n = miLines[mt->lineStartIndex + i].toTokenString(reply, MRL);
      isOK = eeConfig.pushBlock(reply, n);
//       // debug
//        snprintf(s, MSL, "# line %d OK=%d (%d bytes) %s\r\n", i, isOK, n, reply);
//        usb_send_str(s);
//       // debug end
    }
//     // debug
//     usb_send_str("# end\r\n");
//     // debug end
    if (not isOK) 
      break;
  }
  if (not isOK)
  { // not enough space
    snprintf(reply, MRL, "# failed to save mission thread %d line %d (of %d)\n", t, i, miLinesCnt);
    usb_send_str(reply);
  }
  // write size of (saved part of) misson in characters
  eeConfig.write_word(adr, eeConfig.getAddr() - adr);
}

////////////////////////////////////

void UMission::eePromLoadMission()
{
  const int MRL = 100;
  char reply[MRL];
//   const int MSL = 120;
//   char s[MSL];
//   const int MSL = 100;
//   char s[MSL];
  int n = 0, m;
  uint16_t t = -1;
  miLinesCnt = 0;
  bool isOK = true;
  UMissionThread * mt;
  // read number of bytes in mission
  clear();
  threadsCnt = 1;
  mt = threads; // get first thread
  m = eeConfig.readWord() - 1;
  // debug
//   snprintf(s, MSL, "# loading mission %d bytes (config addr = %d)\n", m, eeConfig.getAddr());
//   usb_send_str(s);
  // debug end
  while (m > 1 and n < MRL and isOK)
  {
    snprintf(missionErrStr, 10, "#none\n"); // no error
    // read one byte at a time
    reply[n] = eeConfig.readByte();
    // debug
//     reply[n+1] = '\n';
//     reply[n+2] = '\0';
//     usb_send_str(reply);
    // debug end
    m--;
    if (reply[n] <= ' ')
    { // newline marks end of line
      reply[n] = '\n';
      reply[n+1] = '\0';
      // debug
//       snprintf(s, MSL, "# line: %s", reply);
//       usb_send_str(s);
      // debug end
      if (n > 1)
      { // this is a line
        isOK = mt->decodeToken(reply, &t);
        // debug
//         usb_send_str("#");
//         mt->getLines(true);
        // debug end
        if (not isOK and (t != mt->threadNr))
        { // line belongs to next thread
          mt++;
          threadsCnt++;
          mt->lineStartIndex = miLinesCnt;
          isOK = mt->decodeToken(reply, &t);
          // debug
//          usb_send_str("# new thread\r\n");
          // debug end
        }
      }
//       // debug
//       snprintf(s, MSL, "# m=%d reading to thread %d - %d chars %c%c%c #... - isOK %d\r\n", 
//                m, mt->threadNr, n, reply[0], reply[1], reply[2], isOK);
//       usb_send_str(s);
//       // debug end
      n = 0;
    }
    else
      n++;
  }
  if (not isOK)
    usb_send_str(missionErrStr);
}

/////////////////////////////////////////////////

bool UMission::addLine(const char * lineToAdd)
{
  int16_t tn;
  int16_t ti;
  bool isOK;
  if (threadsCnt == 0)
    threadsCnt = 1;
  UMissionThread * mt = &threads[threadsCnt - 1];
//   tn = mt->threadNr;
//   isOK = mt->addLine(lineToAdd, &tn);
//   // debug
//   const int MSL = 110;
//   char s[MSL];
//   snprintf(s, MSL, "# adding line OK=%d, tn=%d, '%s'\n", isOK, tn, lineToAdd);
//   usb_send_str(s);
  // debug end
  
  // is it a thread line
  char * p1 = strstr(lineToAdd, "thread");
  if (p1 != NULL)
  { // yes, decode thread line
    UMissionLine tmp;
    isOK = tmp.decodeLine(lineToAdd, &tn);
//     if (not isOK and tn != mt->threadNr and threadsCnt < threadsMax - 1)
    if (isOK)
    { // new thread - find or create
      if (tn <= 0)
      { // threads should have a number >= 1
        tn = 1;
        usb_send_str("# UMission::addLine: thread number should be > 0\n");
      }
      else if (tn != mt->threadNr and mt->lineCnt == 0)
      { // thread has no lines yet, so renumber thread
        mt->threadNr = tn;
      }
      ti = getThreadIndex(tn);
      // returns -1 if not known, so add a new thread
      isOK = ti == -1;
      if (isOK)
      { // thread number not seen before
        if (threadsCnt <= threadsMax)
        { // space for more, so add thread
          threadsCnt++;
          // advance to next thread
          mt++;
          // assign the number
          mt->threadNr = tn;
        }
        else
          usb_send_str("# too many threads, max is 5 threads! (number from 1 to 31000) - reused last thread\n");
        // set start line for this thread
        mt->lineStartIndex = miLinesCnt;
        //mt->threadNr = tn;
      }
      // add event part of thread (if any)
      mt->activateFlag = tmp.eventSet;
      // if it is an event activated thread, then start inactive
      mt->threadActive = mt->activateFlag == 0;
      // set also deactivate event flags
      mt->deactivateFlag = tmp.eventMask;
      //       isOK = mt->addThreadOptions(lineToAdd, &tn);
      // debug
//       snprintf(s, MSL, "# adding thread optinos tn=%d, activateEvents=0x%lx, stopEvents=0x%lx, isactive=%d\n", 
//                tn, mt->activateFlag, mt->deactivateFlag, mt->threadActive);
//       usb_send_str(s);
      // debug end
    }
    else
    {
      isOK = false;
      usb_send_str("# UMission::addLine: error in thread line\n");
    }
  }
  else
  {
    isOK = mt->addLine(lineToAdd);
  }
  return isOK;
}

////////////////////////////////////////////////

bool UMission::modLine(int16_t thread, int16_t line, const char * p2)
{
  int idx = getThreadIndex(thread);
  bool isOK = idx >= 0;
  // debug
//   const int MSL = 50;
//   char s[MSL];
//   snprintf(s, MSL, "# found thread %d, isOK=%d (idx=%d)\n", thread, isOK, idx);
//   usb_send_str(s);
  // debug end
  if (isOK)
  {
    UMissionThread * mt = &threads[idx];
    isOK = mt->modLine(line, p2);
  }  
  return isOK;
}



////////////////////////////////////////////////

bool UMission::getLines(bool more)
{
  bool result = true;
  if (not more)
  { // start from first
    moreThread = 0;
    // and with thread number
    threads[moreThread].moreLine = -1;
  }
//   usb_send_str("# UMission::getLines\n");
  // Get one line from this thread
  if (not threads[moreThread].getLines(more))
  { // no more lines in thread, advance
    result = ++moreThread < threadsCnt;
//     usb_send_str("# UMission::getLines advance to next thread\n");
    if (result)
      // mark that first line is thread number
      threads[moreThread].moreLine = -1;
  }
  return result;
}

//////////////////////////////////////

void UMission::getToken()
{
  for (int t = 0; t < threadsCnt; t++)
    threads[t].getToken();
}

///////////////////////////////////////

void UMission::getPowerUse(bool * lineSensor, bool *  irSensor, bool * chirpLog)
{ // power should be on if log is enabled
//   * lineSensor = logRowFlags[LOG_LINE];
//   * irSensor = logRowFlags[LOG_DIST];
//   * chirpLog = logRowFlags[LOG_CHIRP];
//   const int MSL = 120;
//   char s[MSL];
//   snprintf(s, MSL, "# log power use 1, thread %d, Chirp=%d, LS=%d, ir=%d\n", 0, *chirpLog, *lineSensor, *irSensor);
//   usb_send_str(s);
  // look for interface use in current mission line for each thread
  for (int t = 0; t < threadsCnt; t++)
  {
    threads[t].getPowerUse(lineSensor, irSensor, chirpLog);
    // debug
//     if (chirpLog)
    {
//       const int MSL = 50;
//       char s[MSL];
//       snprintf(s, MSL, "# log power use 2, thread %d, Chirp=%d, LS=%d, ir=%d\n", t, *chirpLog, *lineSensor, *irSensor);
//       usb_send_str(s);
    }
    // debug end
    if (*lineSensor and *irSensor and *chirpLog)
      break;
  }
}

//////////////////////////////////////
/**
 * Get number of threads */
int UMission::getLinesCnt()
{
  return miLinesCnt;
}

//////////////////////////////////////

void UMission::clear()
{
  miLinesCnt = 0;
  for (int i = 0; i < miLinesCntMax; i++)
    miLines[i].clear();
  threadsCnt = 0;
  for (int i = 0; i < threadsMax; i++)
    threads[i].clear(i + 1);
  eventFlags = 0;
}

//////////////////////////////////////////

int16_t UMission::getThreadIndex ( int16_t thread )
{
  int16_t idx = -1;
  for (int i = 0; i < threadsCnt; i++)
  {
    if (threads[i].threadNr == thread)
    {
      idx = i;
      break;
    }
  }
  return idx;
}

/////////////////////////////////////////

void UMission::decodeEvent(const char* eventNumber)
{
  const char * p1 = eventNumber;
  int e;
  while (*p1 == ' ' or *p1 == '=')
    p1++;
  e = strtol(p1, NULL, 10);
  if (e >= 0 and e < 32)
    setEvent(e);
  if (e==33 and control.missionState == 0)
  {
    missionStop = false;
    missionStart = true;
  }
}

void UMission::setEvent(int number)
{
  uint32_t f = 1 << number;
  eventFlags |= f;
//   if (number < 30)
  { // send message back to client (except 30 and 31 used to swap 
    // mission lines in Robobot" configuration (from raspberry pi)
    const int MSL = 20;
    char s[MSL];
    snprintf(s, MSL, "event %d\n", number);
    usb_send_str(s);
  }
}
