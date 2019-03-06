/***************************************************************************
 *   Copyright (C) 2019 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 *   Main function for small regulation control object (regbot)
 *   build on a teensy 3.1
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
 

#define REV_ID "$Id: command.cpp 1006 2019-02-03 08:51:52Z jcan $" 

#include <malloc.h>
#include <ADC.h>
#include "IntervalTimer.h"
#include "mpu9150.h"
#include "motor_controller.h"
#include "data_logger.h"
#include "control.h"
#include "robot.h"
#include "main.h"
#include "mission.h"
#include "linesensor.h"
#include "dist_sensor.h"
#include "eeconfig.h"
#include "wifi8266.h"
#include "command.h"
#include "servo.h"
#include "subscribe.h"
#include "control.h"
//#include <../Snooze/Snooze.h>
//#define __MK20DX256__

float time = 0.0; // system time in seconds
uint32_t timeAtMissionStart = 0;
const char * lfcr = "\n\r";
// bool pushToUSB = true;
bool logToUSB = false;
bool silentUSB = false;
bool silenceUSBauto = true; // manuel inhibit of USB silent timeout
// should teensy echo commands to usb
int8_t localEcho = 0;
int32_t motorCurrentMLowPass[2];
//
// int pushInterval = 0;
// int pushTimeLast = 0;
// int pushStatus = 0;
// line sensor full or reduced power (output is high power)
bool pinModeLed = OUTPUT;
//
// mission stop and start
bool button;
bool missionStart = false;
bool missionStop = false;
// bool sendStatusWhileRunning = false;
/**
 * usb command buffer space */
const int RX_BUF_SIZE = 200;
char usbRxBuf[RX_BUF_SIZE];
int usbRxBufCnt = 0;
bool usbRxBufOverflow = false;
bool moreMissionLines = false;
uint32_t usbTimeoutGotData = 0;

int * m1;
/**
 * write to I2C sensor (IMU) */
int writeSensor(uint8_t i2caddr, int reg, int data);
/**
 * send state as text to USB (direct readable) */
void stateToUsb();
void sendStatusSensor(int8_t type);
//void sendStatusLogging();
void sendStatusVersion();
/// eeprom functions
void eePromSaveStatus(uint8_t * configBuffer);
void eePromLoadStatus(const uint8_t * configBuffer);
///
///
/** Who is requesting data
 * -2 = none (push), -1=USB, 0..4 = wifi client */
int8_t requestingClient = -2;
// 
/// serial 3 variables
/// debug copy everyting to USB (to be used with putty)
bool echoSerial3ToUsb = false;
/// size of serial3 command buffer
const int MAX_SER3_BUFFER_CNT = 100;
/// buffer for commands from serial 3
char ser3buff[MAX_SER3_BUFFER_CNT];
/// number of valid characters in serial 3 buffer
int16_t ser3buffCnt = 0;

/**
 * Get SVN revision number */
uint16_t getRevisionNumber()
{
  const char * p1 = strstr(REV_ID, ".cpp");
  return strtol(&p1[4], NULL, 10) * 10 + REV_MINOR;
}

////////////////////////////////////////////

void sendStatusLS()
{
  const int MRL = 250;
  char reply[MRL];
  //                       #1 #2   LS0    LS1    LS2    LS3    LS4    LS5    LS6    LS7    timing (us)
  snprintf(reply, MRL, "ls %d %d  %d %d  %d %d  %d %d  %d %d  %d %d  %d %d  %d %d  %d %d  %ld %ld\r\n",
           adcStartCnt, adcHalfCnt, 
           adcLSH[0], adcLSL[0], adcLSH[1], adcLSL[1], adcLSH[2], adcLSL[2], adcLSH[3], adcLSL[3], 
           adcLSH[4], adcLSL[4], adcLSH[5], adcLSL[5], adcLSH[6], adcLSL[6], adcLSH[7], adcLSL[7], 
           adcConvertTime * 10, adcHalfConvertTime * 10);
  usb_send_str(reply);
}

////////////////////////////////////////////

void sendStatusCurrentVolt()
{
  const int MRL = 250;
  char reply[MRL];
  snprintf(reply, MRL, "VA %d %.3f  %d %ld %.3f  %d %ld %.3f\r\n",
           batVoltInt, batVoltInt * batVoltIntToFloat, 
           motorCurrentM[0], motorCurrentMOffset[0], getMotorCurrentM(0, motorCurrentM[0]),
           motorCurrentM[1], motorCurrentMOffset[1], getMotorCurrentM(1, motorCurrentM[1])
  );
  usb_send_str(reply);
}

//////////////////////////////////////////////////////

/**
 * Test all I2C adresses and print reply. */
void testAddr(void)
{
  int ak;
  const int MRL = 100;
  char reply[MRL];
  for (int i= 0; i < 0x7f; i++)
  {
    Wire.beginTransmission(i);
    Wire.write(0);
    ak = Wire.endTransmission(I2C_STOP,1000);
    snprintf(reply, MRL, "addr test addr %d (%x) gave %d", i, i, ak);
    usb_send_str(reply);
  }
}

// ////////////////////////////////////////

/**
 * Got a new character from USB channel
 * Put it into buffer, and if a full line, then intrepid the result.
 * \param n is the new character */
void receivedCharFromUSB(uint8_t n)
{ // got another character from usb host (command)
  if (n >= ' ')
  {
    usbRxBuf[usbRxBufCnt] = n;
    if (usbRxBufCnt < RX_BUF_SIZE - 1)
      usbRxBuf[++usbRxBufCnt] = '\0';
    else
    {
      usbRxBufOverflow = true;
      usbRxBufCnt = 0;
      usbRxBuf[usbRxBufCnt] = '\0';
    }
  }
  if (localEcho == 1 and not silentUSB)
    // echo characters back to terminal
    usb_serial_putchar(n);
  if (n == '\n' or n=='\r')
  { // zero terminate
    if (usbRxBufOverflow)
    {
      usbRxBufOverflow = false;
      usb_send_str("# USB rx-buffer overflow\n");
    }
    else
    {
      if (usbRxBufCnt > 0)
      {
        usbRxBuf[usbRxBufCnt] = '\0';
        if (strncmp(usbRxBuf, "alive", 5) == 0 or strncmp(usbRxBuf, "<alive", 6) == 0)
        { // just an alive signal - reply with mobotware message
          const int MSL = 50;
          char s[MSL];
          snprintf(s, MSL, "<alive last=\"%.5f\"/>\r\n", float(controlUsedTime[1]) / F_CPU);
          usb_send_str(s);
        }
        else
          parse_and_execute_command(usbRxBuf, -1);
      }
      if (localEcho == 1)
      {
        usb_send_str("\r\n>>");
        //usb_serial_flush_output();
      }
    }
    // flush remaining input
    usbRxBufCnt = 0;
  }
  else if (usbRxBufCnt >= RX_BUF_SIZE - 1)
  { // garbage in buffer, just discard
    usbRxBuf[usbRxBufCnt] = 0;
    const char * msg = "** Discarded (missing \\n)\n";
    usb_send_str(msg);
    usbRxBufCnt = 0;
  }
}

/**
 * Got a new character from serial 3 connection - channel
 * Put it into buffer, and if a full line, then intrepid the result.
 * \param n is the new character */
void receivedCharFromSer3(uint8_t n)
{ // got another character from usb host (command)
  if (n >= ' ')
  {
    ser3buff[ser3buffCnt] = n;
    if (ser3buffCnt < MAX_SER3_BUFFER_CNT - 1)
      ser3buff[++ser3buffCnt] = '\0';
  }
  if (localEcho == 1  and not silentUSB)
    // echo characters back to terminal
    usb_serial_putchar(n);
  if (n == '\n' or n=='\r')
  { // zero terminate
    {
      if (ser3buffCnt > 0)
      {
        ser3buff[ser3buffCnt] = '\0';
        if (strncmp(ser3buff, "alive", 5) == 0 or strncmp(ser3buff, "<alive", 6) == 0)
        { // just an alive signal - reply with mobotware message
          const int MSL = 50;
          char s[MSL];
          snprintf(s, MSL, "<alive last=\"%.5f\"/>\r\n", float(controlUsedTime[1]) / F_CPU);
          usb_send_str(s);
        }
        else
          // set source (alternative serial - source after wifi)
          parse_and_execute_command(ser3buff, wifi.WIFI_MAX_CLIENTS);
      }
      if (localEcho == 1)
      {
        usb_send_str("\r\n>>");
        //usb_serial_flush_output();
      }
    }
    // flush remaining input
    ser3buffCnt = 0;
  }
  else if (ser3buffCnt >= MAX_SER3_BUFFER_CNT - 1)
  { // garbage in buffer, just discard
    ser3buff[ser3buffCnt] = 0;
    const char * msg = "** Discarded (missing \\n)\n";
    usb_send_str(msg);
    ser3buffCnt = 0;
  }
}


void handleIncoming(uint32_t mainLoop)
{
  int n = 0, m;
  // get number of available chars in USB buffer
  m = usb_serial_available();
  if (m > 20)
    // limit to no more than 20 chars in one 1ms cycle
    m = 20;
  else if (m == 0 and (not silentUSB) and silenceUSBauto and hbTimerCnt - usbTimeoutGotData > 4000)
  { // no data for a long time - stop sending data to USB
//     usb_send_str("# USB silent - no activity\r\n");
    usbTimeoutGotData = hbTimerCnt;
    silentUSB = true;
  }
  // 
  if (m > 0)
  { // get characters
    for (int i = 0; i < m; i++)
    { // get pending characters
      n = usb_serial_getchar();
      if (n < 0)
        break;
      if (n >= '\n' and n < 0x80)
      { // there is data from USB, so it is active
        if (silentUSB and (n == '\n' or n == '\r'))
        { // open for data
          usb_send_str("# USB activated\r\n");
        }
        silentUSB = false;
        usbTimeoutGotData = hbTimerCnt;
        // command arriving from USB
        //usb_send_str("#got a char from USB\r\n");
        receivedCharFromUSB(n) ;
        break;
      }
    }
  }
  // handling wifi output data
  // long messages are send in parts - handle any pending tx message
  wifi.sendPendingToWifi();
  // sending (many) mission lines, try to send the next
  if (moreMissionLines)
    // we are in a transmission - drop any other commands til we are finished
    moreMissionLines = userMission.getLines(true);
  // get characters form wifi
  // up to 10 at a time
  m = Serial1.available();
  if (m > 10)
    m = 10;
  for (int i = 0; i < m; i++)
  { // read up to 10 chars from wifi connections
    n = Serial1.read(); // read 1 char
    if (n < 0)
      break;
    if (n > '\0' and n < 0x80)
    { // potentially usable character
      // debug relay wifi input to USB
      if (wifi.echo8266toUSB  and not silentUSB)
        usb_serial_putchar(n);
      // debug end relay wifi input to USB
      if (wifi.receivedCharFromSer(n))
      {
        //         const int MSL = 50;
        //         char s[MSL];
        //         // debug
        //           snprintf(s, MSL, "# about to parse ch=%d msg:\"%s\"", channel, cmd);
        //           usb_send_str(s);
        // debug end
        if (not moreMissionLines)
        {  // if in a transmission - drop all commands til we are finished,
          const int MSL = 70;
          char s[MSL];
          // a message to handle 
          if (wifi.wifiCommand != NULL)
          {
            if (strncmp(wifi.wifiCommand, "alive", 5) == 0 or strncmp(wifi.wifiCommand, "<alive", 6) == 0)
            {  // just an alive signal - note
              wifi.clientAlive[wifi.lastChannel] = hbTimerCnt;
              snprintf(s, MSL, "<alive last=\"%.5f\"/>\r\n", float(controlUsedTime[1]) / F_CPU);
              wifi.wifiSend(wifi.lastChannel, s, strlen(s));
            }
          }
          // so now, ready to parse
          if (wifi.wifiCommand[0] > ' ')
          {
            // debug
            if (wifi.wifiCommand[0] == '+')
            { // should never happen
              snprintf(s, MSL, "# wifi cmd='%s' from channel %d\r\n", wifi.wifiCommand, wifi.lastChannel);
              usb_send_str(s);
            }
            // debug end
            parse_and_execute_command(wifi.wifiCommand, wifi.lastChannel);
          }
          //         usb_send_str("# parsed wifi msg\r\n");
        }
        wifi.clearRxBuffer();
      }      
      //     const int MSL = 70;
      //     char s[MSL];
      //     snprintf(s, MSL, "# handleIncoming got %d chars, wait=%d\r\n", serRxBufCnt, wifiWaitForSendOK);
      //     usb_send_str(s);    
    }
  }
  for (int i = 0; i < 10; i++)
  { // read up to 10 chars from alternative serial connections
    n = Serial3.read(); // read 1 char
    if (n < 0)
      break;
    if (n > '\0' and n < 0x80)
    { // potentially usable character
      // debug relay wifi input to USB
      if (echoSerial3ToUsb  and not silentUSB)
        usb_serial_putchar(n);
      // buffer new character and execute, when complete
      receivedCharFromSer3(n);
    }
  }
  if (wifi.sendingState == UWifi8266::WFD_SEND_REQ or wifi.sendingState == UWifi8266::WFD_SENDING)
  {
    if (hbTimerCnt - wifi.waitForSendOKtime > 1000)
    { // debug/error?
            const int MSL = 100;
            char s[MSL];
            snprintf(s, MSL, "# %lu ms wifi timeout - not send %s... msg\r\n", hbTimerCnt, wifi.serTxBuf);
            usb_send_str(s);
      // debug/err end
      wifi.sendingState = UWifi8266::WFD_SEND_FAILED;
    }
  }  
}

//////////////////////////////////////////


void sendHartBeat()
{
  const int MRL = 35;
  char reply[MRL];
  snprintf(reply, MRL,  "hbt %g %g %d %d %d %lu\r\n", time, batVoltInt * batVoltIntToFloat, 
           control.controlActive, control.missionState, remoteControl, controlUsedTime[2] / (F_CPU/1000000));
  usb_send_str(reply, false);
}

void sendPose()
{
  const int MRL = 64;
  char reply[MRL];
  snprintf(reply, MRL,"pse %g %g %g %g %g\r\n",
           pose[0], pose[1], pose[2], distance, pose[3]);
  usb_send_str(reply, false);
}

void sendAcc()
{
  const int MRL = 64;
  char reply[MRL];
  snprintf(reply, MRL,"acw %g %g %g\r\n",
           imuAcc[0] * accScaleFac, imuAcc[1] * accScaleFac, imuAcc[2] * accScaleFac);
  usb_send_str(reply);
}

void sendGyro()
{
  const int MRL = 64;
  char reply[MRL];
  snprintf(reply, MRL,"gyw %g %g %g\r\n",
           imuGyro[0] * gyroScaleFac, imuGyro[1] * gyroScaleFac, imuGyro[2] * gyroScaleFac);
  usb_send_str(reply);
}

void sendMotorCurrent()
{
  const int MRL = 64;
  char reply[MRL];
  snprintf(reply, MRL,"mca %g %g\r\n", motorCurrentA[0], motorCurrentA[1]);
  usb_send_str(reply);
}

void sendWheelVelocity()
{
  const int MRL = 64;
  char reply[MRL];
  snprintf(reply, MRL,"wve %g %g\r\n", wheelVelocityEst[0], wheelVelocityEst[1]);
  usb_send_str(reply);
}



/** send sensor status
 * \param type gives the message type
 * -1 = all 
 *  0 = accelerometer
 *  1 = gyro
 *  2 = gyro offset
 *  3 = motor current
 *  4 = encoder tick count
 *  5 = wheel velocity (m/s)
 *  6 = wheel position (m)
 *  7 = pose (x,y,heading,tilt)
 *  8 = button (start button pressed)
 *  9 = battery voltage (V)
 * 10-11 encoder time values (in cpu clock count)
 * 12-13 encoder calibrate factors
 * */
void sendStatusSensor(int8_t type)
{
  //int switch2 = digitalRead(11);
  const int MRL = 450;
  char reply[MRL];
  reply[0] = '\0';
  //   float mc0 = getMotorCurrent(0);
  //   float mc1 = getMotorCurrent(1);
  switch (type)
  { 
    case -1:
      // send all
      snprintf(reply, MRL,"acw %g %g %g\n"
      "gyw %g %g %g\n"
      "gyo %g %g %g %d\n"
      "mca %g %g\n" //%d %d %d %d %ld %ld %g %g\n"
      "enc 0x%lx 0x%lx\n"
      "wve %g %g\n"
      "wpo %g %g\n"
      "pse %g %g %g %g %g\n"
      "swv %d\n"
      "bat %g\r\n" ,
      imuAcc[0] * accScaleFac, imuAcc[1] * accScaleFac, imuAcc[2] * accScaleFac,
      imuGyro[0] * gyroScaleFac, imuGyro[1] * gyroScaleFac, imuGyro[2] * gyroScaleFac,
      offsetGyro[0] * gyroScaleFac, offsetGyro[1] * gyroScaleFac, offsetGyro[2]* gyroScaleFac, gyroOffsetDone,
      getMotorCurrentM(0, motorCurrentM[0]), getMotorCurrentM(1, motorCurrentM[1]),
               encoder[0], encoder[1], 
               wheelVelocityEst[0], wheelVelocityEst[1],
               wheelPosition[0], wheelPosition[1],
               pose[0], pose[1], pose[2], distance, pose[3],
               button, 
               batVoltInt * batVoltIntToFloat
      );
      break;
    case 0:
      sendAcc();
//       snprintf(reply, MRL,"acw %g %g %g\r\n",
//                imuAcc[0] * accScaleFac, imuAcc[1] * accScaleFac, imuAcc[2] * accScaleFac);
      break;
    case 1:
      sendGyro();
//       snprintf(reply, MRL,"gyw %g %g %g\r\n",
//                imuGyro[0] * gyroScaleFac, imuGyro[1] * gyroScaleFac, imuGyro[2] * gyroScaleFac);
      break;
    case 2:
      snprintf(reply, MRL,"gyo %g %g %g %d\r\n",
               offsetGyro[0] * gyroScaleFac, offsetGyro[1] * gyroScaleFac, offsetGyro[2]* gyroScaleFac, gyroOffsetDone);
      break;
    case 3:
      sendMotorCurrent();
//       snprintf(reply, MRL,"mca %g %g\r\n", motorCurrentA[0], motorCurrentA[1]);
      //getMotorCurrentM(0, motorCurrentM[0]), getMotorCurrentM(1, motorCurrentM[1]));
      break;
    case 4:
      snprintf(reply, MRL,"enc 0x%lx 0x%lx\r\n", encoder[0], encoder[1]);
      break;
    case 5:
      sendWheelVelocity();
//       snprintf(reply, MRL,"wve %g %g\r\n", wheelVelocityEst[0], wheelVelocityEst[1]);
      break;
    case 6:
      snprintf(reply, MRL,"wpo %g %g\r\n",
               wheelPosition[0], wheelPosition[1]);
      break;
    case 7:
      sendPose();
//       snprintf(reply, MRL,"pse %g %g %g %g %g\r\n",
//                pose[0], pose[1], pose[2], distance, pose[3]);
      break;
    case 8:
      snprintf(reply, MRL, "swv %d\r\n", button);
      break;
    case 9:
      snprintf(reply, MRL, "bat %g\r\n", batVoltInt * batVoltIntToFloat);
      break;
    case 10: // motor 1 (0)
    case 11: // motor 2 (1)
      // encoder magnet size offset - motor 1
      { // allow local variables
        char * p1;
        int n;
        int m = type - 10; // motor
        snprintf(reply, MRL, "en%d %d %u", m, MAX_ENC_VALS, F_CPU);
        n = strlen(reply);
        p1 = &reply[n];
        for (int i = 0; i < MAX_ENC_VALS; i++)
        {
          snprintf(p1, MRL - n, " %lu", encTime_cpu[m][i]);
          n += strlen(p1);
          p1 = &reply[n];
        }
        snprintf(p1, MRL - n, "\r\n");
      }
      break;
    case 12:
    case 13: // encoder calibrate factors
    { // allow local variables
      char * p1;
      int n;
      int m = type - 12; // motor
      snprintf(reply, MRL, "ef%d %d", m, MAX_ENC_VALS);
      n = strlen(reply);
      p1 = &reply[n];
      for (int i = 0; i < MAX_ENC_VALS; i++)
      {
        snprintf(p1, MRL - n, " %g", float(encTimeScale[m][i])/1000.0);
        n += strlen(p1);
        p1 = &reply[n];
      }
      snprintf(p1, MRL - n, "\r\n");
    }
    break;
    case 14: // encoder calibrate status
      snprintf(reply, MRL, "ecs %d %d %d %d %d\n", encTimeScaleCalibrated, encTimeTryCalibrate, not encTimeStopCollect, encTimeCalibrateUse, encReindexBestIndex);
      break;
    case 15: // (white) line sensor debug
      snprintf(reply, MRL, "# white line low=%d, hi=%d, whiteVal=%g, whiteQ=%d\n"
                           "#            0=%g, 1=%g, 2=%g, 3=%g, 4=%g, 5=%g, 6=%g, 7=%g\n"
                           "#            leftIdx=%d, RightIdx=%d, edgePosL=%g, edgePosr=%g, validCnt=%d\n",
               lsIdxLow, lsIdxHi, whiteVal,  whiteQ,
               lineSensorValue[0],lineSensorValue[1],lineSensorValue[2],lineSensorValue[3],
               lineSensorValue[4],lineSensorValue[5],lineSensorValue[6],lineSensorValue[7],
               lsIdxxl, lsIdxxr, lsTl, lsTr, lsEdgeValidCnt);
      break;
    case 16: // Line sensor normalize gain
      sendLineSensorGain();
      break;
    default:
      reply[0] = '\0';
  }
  if (reply[0] != '\0')
    usb_send_str(reply);
  // debug
  //   snprintf("# current pree %d first %d \n", motorPreEnabled, motorPreEnabledRestart);           
  // debug end
}



void sendStatusVersion()
{
  const int MRL = 100;
  char reply[MRL];
  snprintf(reply, MRL, "version %.1f %d\r\n", (float)getRevisionNumber() / 10.0, imuAvailable);
  usb_send_str(reply);
//   snprintf(reply, MRL, "# version %d (%s)\r\n", getRevisionNumber(), REV_ID);
//   usb_send_str(reply);
  
}


// parse a user command and execute it, or print an error message
//
void parse_and_execute_command(char *buf, int8_t serChannel)
{ // first character is the port letter
  const int MSL = 100;
  char s[MSL];
  char * p2;
  bool commandHandled = false;
  //
  if (strncmp(buf, "stop", 4) == 0)
  {
    missionStop = true;
    remoteControlVel = 0;
    remoteControlVelDif = 0;
    logToUSB = false;
    commandHandled = true;
  }
  else if (strncmp(buf, "halt", 4) == 0)
  { // stop all 12V power (or turn on if 12 V power is off (and switch is on))
    if (buf[4] >= ' ')
      batteryHalt = strtol(&buf[5], NULL, 10);
    else
      batteryHalt = true;
    commandHandled = true;
  }
  else if (logToUSB)
    // while sending log to client, no other cammands are valid
    return;
  // the rest is
  // commands with a simple reply, so
  // requesting client is needed
  requestingClient = serChannel;
  if (not commandHandled)
  { // all other commands
    if (buf[1] == '=')
    { // one character assignment command
      bool isOK;
      switch (buf[0])
      {
        case 'i':  // interactive mode -1=no info, 0=normal GUI, 1=telnet session (local echo of all)
          localEcho = strtol(&buf[2], NULL, 10);
          break;
        case 'M':  // mission to run
          if (control.missionState == 0)
          { // mission can not be changed while another mission is running
            control.mission = strtol(&buf[2], NULL, 10); 
            if (control.mission >= missionMax)
              control.mission = missionMax - 1;
          }
          else
            usb_send_str("#* sorry, a mission is in progress.\r\n");
          break;
        case 's':  // log interval
          logInterval = strtol(&buf[2], &p2, 10);
          logIntervalChanged();
          if (*p2 >= ' ')
            logAllow = strtol(p2, &p2, 10); 
          break;
//         case 'S':  pushInterval = strtol(&buf[2], NULL, 10); break;
//         case 'R':  sendStatusWhileRunning = strtol(&buf[2], NULL, 10); break;
        case 'a':  
          usb_send_str("# trying to access IMU: sending request\r\n");
          isOK = mpuRequestData();
          if (isOK)
            usb_send_str("# sending request was OK\r\n");
          else
            usb_send_str("# sending request failed\r\n");
          mpuReadData();        
          break;
        default: 
          usb_send_str("#** unknown command!\r\n");
          break;
      }
    }
    else if (buf[0] == 'x')
    { // reply on user data request for controller status
      // debug
      //     hertil - hvorfor forsvinder xcvel kommando???
      //     const int MSL = 50;
      //     char s[MSL];
      //     snprintf(s, MSL, "# passed x (client=%d)\r\n", requestingClient);
      //     usb_send_str(s);
      // debug end
      if (not control.sendStatusControl(&buf[1]))
        usb_send_str("# unknown controller request\r\n");
    }
    else if (strncmp(&buf[0], "rc=", 3) == 0)
    { // remote control 
      char * p1 = &buf[3];
      char * p2 = &buf[3];
      bool  e =  strtol(p1, &p1, 10); // enable
      float v = strtof(p1, &p1); // velocity
      float t = strtof(p1, &p2); // wheel difference
      if (p2 > p1)
      { // check also for balance parameter
        p1 = p2;
        bool b = strtol(p1, &p2, 10);
        if (p1 == p2)
          b = false;
        control.setRemoteControl(e, v, t, b);
      }
      if (false)
      { // debug
        const int MSL = 100;
        char s[MSL];
        snprintf(s, MSL, "# rc=: '%s'\n", buf);
        usb_send_str(s);
      }
    }
    else if (control.setRegulator(buf))
    { // regulator parameters
    }
    else if (setLineSensor(buf))
    { // line sensor settings
    }
    else if (setIrCalibrate(buf))
    { // infrared distance sensor data
    }
    else if (buf[0] == '<')
    { // mission line
      if (strncmp(&buf[1], "clear", 5) == 0)
        userMission.clear();
      else if (strncmp(&buf[1], "add", 3) == 0)
      {
        userMission.addLine(&buf[4]);
        // debug
        //       snprintf(s, MSL, "# add %s\r\n", &buf[4]);
        //       usb_send_str(s);
        // debug end
      }
      else if (strncmp(&buf[1], "event", 5) == 0)
      {
        userMission.decodeEvent(&buf[6]);
      }
      else if (strncmp(&buf[1], "mod", 3) == 0)
      { // modify line
        int16_t thread = strtol(&buf[4], &p2, 10);
        bool isOK = p2 != &buf[4] and p2 != NULL;
        int16_t line   = strtol(p2, &p2, 10);
        isOK &= p2 != &buf[4] and p2 != NULL;
        const int MSL = 100;
        char s[MSL];
        // debug
//         const int MSL = 100;
//         char s[MSL];
//         snprintf(s, MSL, "# parse_and_execute_command: thread=%d, line=%d, isOK=%d\r\n", thread, line, isOK);
//         usb_send_str(s);
        // debug end
        if (isOK and thread > 0 and line > 0)
        { // has got valid thread and line numbers
          isOK = userMission.modLine(thread, line, p2);
          if (not isOK)
          {
            snprintf(s, MSL, "# parse_and_execute_command: %s\r\n", missionErrStr);
            usb_send_str(s);
          }
        }
        else
        {
          snprintf(s, MSL, "# parse_and_execute_command: error thread=%d (< 1!), line=%d (<1!)\r\n", thread, line);
          usb_send_str(s);
        }
      }
      else if (strncmp(&buf[1], "get", 3) == 0)
      {
        moreMissionLines = userMission.getLines(false);
        //usb_send_str("# send all lines?\n");
      }
      else if (strncmp(&buf[1], "token", 5) == 0)
      {
        userMission.getToken();
      }
      else
        usb_send_str("# no such user mission command\r\n");
    }
    else if (buf[0] == 'u')
    { // reply on user data request
      int8_t r = strtol(&buf[1], NULL, 10);
      switch (r)
      {
        case 0: sendStatusVersion(); break;
        case 1: 
          //sendHartBeat();
          sendStatusSensor(-1);  break;
        case 2: 
          control.sendMissionStatusChanged(true); break;
        case 3: sendStatusLogging();  break;
        case 4: sendStatusRobotID(); break;
        case 5: sendHartBeat(); break;
        case 6: sendStatusMag(); break;
        case 7: sendStatusLS(); break;
        case 8: sendStatusCurrentVolt(); break;
        case 9:  // 1 limit white (liw)
        case 10: // 2 limit black (lib)
        case 11: // 3 normalized (liv)
        case 12: // 4 raw line sensor values  (liv)
        case 13: // 5 edge position and crossings (lip)
        case 14: // 6 raw ADC values (as comment)
          sendADLineSensor(r - 8); 
          break;
        case 15: //  *  0 = accelerometer
        case 16: //  *  1 = gyro
        case 17: //  *  2 = gyro offset
        case 18: //  *  3 = motor current
        case 19: //  *  4 = encoder tick count
        case 20: //  *  5 = wheel velocity (m/s)
        case 21: //  *  6 = wheel position (m)
        case 22: //  *  7 = pose (x,y,heading,tilt)
        case 23: //  *  8 = button (start button pressed)
        case 24: //  *  9 = battery voltage (V)
        case 25: //  * 10 = encoder offset factor values (motor 1)
        case 26: //  * 11 = encoder offset factor values (motor 2)
        case 27: //  * 12 = encoder calibrate factors (motor 1)
        case 28: //  * 13 = encoder calibrate factors (motor 2)
        case 29: //  * 14 = encoder calibrate status
        case 30: //  * 15 line sensor debug
        case 31: //  * 16 line sensor normalize gain
          sendStatusSensor(r - 15);
          break;
        default:
          usb_send_str("#** unknown U status request!\r\n");
          break;
      }
    }
    else if (buf[0] == 'v')
    { // reply on user data request
      switch (buf[1])
      {
        case '0': sendStatusDistIR(); break;
        case '1': wifi.sendStatusWiFi(); break;
        case '2': wifi.sendStatusWiFiClients(); break;
        default:
          usb_send_str("#** unknown V status request!\r\n");
          break;
      }
    }
    else if (buf[0] == 'h' || buf[0] == 'H')
    {
      const int MRL = 220;
      char reply[MRL];
      snprintf(reply, MRL, "# RegBot commands (" REV_ID ")\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   M=N         Select mission 0=user mission, 1.. = hard coded mission (M=%d)\r\n", control.mission);
      usb_send_str(reply);
      snprintf(reply, MRL, "#   i=1         Interactive: 0: GUI info, 1: use local echo of all commands, -1:no info  (i=%d)\r\n", localEcho);
      usb_send_str(reply);
      snprintf(reply, MRL, "#   s=N A       Log interval in milliseconds (s=%d) and allow A=1 (is %d)\r\n", logInterval, logAllow);
      usb_send_str(reply);
      snprintf(reply, MRL, "#   log+/-item  Log item (mis %d, acc %d, gyro %d, " /*"mag %d, " */ "motR %d, "
      "motV %d, motA %d, enc %d, mvel %d, tr %d, pose %d, line %d, dist %d, bat %d, ct %d)\r\n",
      logRowFlags[LOG_MISSION], logRowFlags[LOG_ACC], logRowFlags[LOG_GYRO], //logRowFlags[LOG_MAG],
      logRowFlags[LOG_MOTV_REF], logRowFlags[LOG_MOTV], 
      logRowFlags[LOG_MOTA], logRowFlags[LOG_ENC], logRowFlags[LOG_WHEELVEL], logRowFlags[LOG_TURNRATE], logRowFlags[LOG_POSE], 
      logRowFlags[LOG_LINE], logRowFlags[LOG_DIST],
      logRowFlags[LOG_BATT], /*logRowFlags[LOG_BAL_CTRL],*/ logRowFlags[LOG_CTRLTIME]);
      usb_send_str(reply);
      snprintf(reply, MRL, "#   log start    Start logging to %dkB RAM (is=%d, logged %d/%d lines)\r\n", LOG_BUFFER_MAX/1000, loggerLogging(), logRowCnt, logRowsCntMax);
      usb_send_str(reply);
      snprintf(reply, MRL, "#   log get      Transfer log to USB (active=%d)\r\n", logToUSB);
      usb_send_str(reply);
      snprintf(reply, MRL, "#   motw m1 m2   Set motor PWM -1024..1024 (is=%d %d)\r\n", motorAnkerPWM[0], motorAnkerPWM[1]);
      usb_send_str(reply);
      snprintf(reply, MRL, "#   motv m1 m2   Set motor voltage -6.0 .. 6.0 (is=%.2f %.2f)\r\n", motorAnkerVoltage[0], motorAnkerVoltage[1]);
      usb_send_str(reply);
      snprintf(reply, MRL, "#   mote m1 m2   Set motor enable (left right) (is=%d %d)\r\n", motorEnable[0], motorEnable[1]);
      usb_send_str(reply);
      snprintf(reply, MRL, "#   u0..u8       Status: u0:ver,u1:measure,u2:mission, u3:log,u4:robot,u5:heartbeat,u6:mag,u8:motorVA\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   u9..u14      Status: Line sensor u9=limit w, u10=limits B, u11=value, u12=raw, u13=pos and x, u14=ADC\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   u15..u24     Status:  15=acc, 16=gyro, 17=gyro offs, 18=motor (A), 19=enc, 20=vel(m/s), 21=pos(m), 22=pose, 23=button, 24=voltage (V)\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   u25..u31     Status:  25,26 encoder calibrate raw values (cpu clock units), u27,28 calibrate factors, u29 enc status, u30 line sensor, u31 line sensor gain \r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   v0..2        Status: v0:IR sensor data, v1:wifi status, v2 wifi clients\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   xID          Get Control params, ID: cvel=wheel vel, ctrn=turn, cwve=wall vel, cwth=wall turn, cpos=position, \r\n"
                           "#                cedg=edge, cbal=balance, cbav=bal vel, ctrl=rate limit\r\n");
      usb_send_str(reply);
      usb_send_str(        "#   ID x x ...   Set controler parameters (ID use Kp ... see format below)\r\n");
      usb_send_str(        "#         format: ID use kp iuse itau ilim Lfuse LfNum LfDen Lbuse LbNum LbDen preUse preNum preDen\r\n");
      usb_send_str(        "#                 preIuse preItau preIlim ffUse ffKp ffFuse ffFnum ffFden LimUse Lim\r\n");
      //     usb_send_str(        "#   rX=d d d ... Settingsfor regulator X - use GUI or see code for details\r\n");
      snprintf(reply, MRL, "#   rid=d d d d d d d d d d  Robot ID set: ID wheelBase gear PPR RadLeft RadRight balanceOffset batt_use batt_low HW_version\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   eew          Save configuration to EE-Prom\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   eeW          Get configuration as string\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   eer          Read configuration from EE-Prom\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   eeR=X        Read config and mission from hard coded set X=0: empty, X=1 follow wall\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   sub s p m    Subscribe s=1, unsubscribe s=0, p=priority 0 (high) .. 4(low), "
                                            "m=msg:0=hbt, 1=pose,2=IR,3=edge,4=acc,5=gyro,6=current,7=vel\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   sut t p      msg interval for priority, t=time in ms 1.., p=priority 0..4 (is 0=%d 1=%d 2=%d 3=%d 4=%d)\r\n", 
               subscribe.sendInterval[0], subscribe.sendInterval[1], subscribe.sendInterval[2],
               subscribe.sendInterval[3], subscribe.sendInterval[4]);
      usb_send_str(reply);
      snprintf(reply, MRL, "#   posec        Reset pose and position\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   gyroo        Make gyro offset calibration\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   mem          Some memory usage info\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   start        start mission now\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   stop         terminate mission now\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   rc=A V T [B] Remote control A=0/1 (is %d), V=m/s (is %g), T=vel difference (is %g), B=1 try keep balance\r\n", remoteControl, remoteControlVel, remoteControlVelDif);
      usb_send_str(reply);
      snprintf(reply, MRL, "#   <add user-mission-line>     add a user mission line (%d lines loaded in %d threads)\r\n", 
               userMission.getLinesCnt(), userMission.getThreadsCnt());
      usb_send_str(reply);
      snprintf(reply, MRL, "#   <mod T L user-mission-line  Modify line L in thread T to new user-mission-line\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   <clear>      Clear all user mission lines\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   <get>        Get all user mission lines\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   <event=X>    Make an event number X (from 0 to 31)\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   <token>      Get all user mission lines as tokens\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   :xxxx        Send data (AT commands) to wifi board (all except the ':') \\r\\n is added\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   link=L,data  Send data to socket link L\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   wifi use port SSID PW   Wifi setup (e.g. 'wifi 1 24001 \"device\" \"\"')\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   wifi e/n     Echo all received from 8266 to USB link\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   halt         Turn 12V power off (on by halt=0) (is %d)\r\n", batteryHalt);
      usb_send_str(reply);
      snprintf(reply, MRL, "#   alive        Alive command - reply: <alive last=\"0.0xxx\"/>\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   iron 1       IR sensor on (1 = on, 0 = off) is=%d\r\n", useDistSensor);
      usb_send_str(reply);
      snprintf(reply, MRL, "#   irc 2 8 2 8 u i  IR set 20cm 80cm 20cm 80cm on installed\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   servo N P V  Set servo N=1..3 (4,5) P (position):-512..+512 (>1024 = disable), V (velocity): 0=full, 1..10 P-values/per ms\r\n"
                           "#                (status: (servo 1,2,3) enabled %d,%d,%d, P= %d, %d, %d, V= %d, %d, %d)\r\n", 
               servo.servoEnabled[0], servo.servoEnabled[1], servo.servoEnabled[2],
               servo.servoValue[0], servo.servoValue[1], servo.servoValue[2],
               servo.servoVel[0], servo.servoVel[2], servo.servoVel[2] 
      );
      usb_send_str(reply);
      snprintf(reply, MRL, "#   servo -1 frw Set PWM frequency\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   svo          Get servo status (same format as below)\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   svo s1 p1 v1 s2 p2 v2 s3 p3 v3 s4 p4 v4 s5 p5 v5  Set servo status sx=enable px=position (-1000..1000), vx velocity 0 (full) 1..10 (slower)\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   encc, enci     Encoder calibrate: enc: calibrate now, eni: Find calibration index now\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   eneX=1       enec=1/0 enable timing collect, eneu=1/0 use calibration, enea=1/0 auto calibrate\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   silent=1     Should USB be silent, if no communication (1=auto silent)\r\n");
      usb_send_str(reply);
      snprintf(reply, MRL, "#   help         This help text\r\n");
      usb_send_str(reply);
    }
    else if (strncmp(buf, "eew", 3) == 0)
    { // initalize ee-prom
      eeConfig.setStringBuffer(NULL, true);
      // save to ee-prom
      usb_send_str("# saving to flash\r\n");
      eeConfig.eePromSaveStatus(false);
    }
    else if (strncmp(buf, "eeW", 3) == 0)
    { // save config to string buffer (and do not touch ee-prom)
      uint8_t buffer2k[2048];
      eeConfig.setStringBuffer(buffer2k, false);
      eeConfig.eePromSaveStatus(true);
      // send string-buffer to client
      eeConfig.stringConfigToUSB(NULL, 0);
      eeConfig.clearStringBuffer();
    }
    else if (strncmp(buf, "eer", 3) == 0)
      // load from flash to ccurrent configuration
      eeConfig.eePromLoadStatus(NULL);
    else if (strncmp(buf, "eeR", 3) == 0)
    { // load from hard coded configuration to current configuration
      // get config number
      char * p1 = strchr(buf,'=');
      int m = 1;
      if (p1 != NULL)
      { // skip the equal sign
        p1++;
        // get the configuration number
        m = strtol(p1, NULL, 10);
        if (m < 0)
          m = 0; // factory reset
      }
      // 2. parameter is a debug flag to send 
      // the newly loaded configuration to USB
      eeConfig.hardConfigLoad(m, true);
    }
    //   else if (strncmp(buf, "test", 4) == 0)
    //     processingTimeTest();
    else if (strncmp(buf, "start", 5) == 0)
    {
      missionStop = false;
      missionStart = true;
    }
    else if (strncmp(buf, "posec", 5) == 0)
      clearPose();
    else if (strncmp(buf, "log", 3) == 0)
    { // add or remove
      if (strstr(&buf[3], "start") != NULL)
      { // if not mission timing, then zero here
        if (control.missionState == 0)
          time = 0.0;
        startLogging(0, true);
        logAllow = true;
      }
      if (strstr(&buf[3], "stop") != NULL)
      { // 
        stopLogging();
      }
      else if (strstr(&buf[3], "get") != NULL)
      {
        if (logRowCnt == 0)
          usb_send_str("% log is empty\r\n");
        else
        { // stop mission first (if running)
          missionStop = true;
          logToUSB = true;
        }
      }
      else
      {
        int plus = buf[3] == '+';
        char * p1 = &buf[4];
        while (*p1 <= ' ' and *p1 > '\0')
          // skip any whitespace
          p1++;
        if (strncasecmp(p1, "mis", 3) == 0)
          logRowFlags[LOG_MISSION] = plus;
        else if (strncasecmp(p1, "acc", 3) == 0)
          logRowFlags[LOG_ACC] = plus;
        else if (strncasecmp(p1, "gyro", 4) == 0)
          logRowFlags[LOG_GYRO] = plus;
        else if (strncasecmp(p1, "motV", 4) == 0)
          logRowFlags[LOG_MOTV] = plus;
        else if (strncasecmp(p1, "motR", 4) == 0)
          logRowFlags[LOG_MOTV_REF] = plus;
        else if (strncasecmp(p1, "motA", 4) == 0)
          logRowFlags[LOG_MOTA] = plus;
        else if (strncasecmp(p1, "enc", 3) == 0)
          logRowFlags[LOG_ENC] = plus;
        else if (strncasecmp(p1, "mvel", 4) == 0)
          logRowFlags[LOG_WHEELVEL] = plus;
        else if (strncasecmp(p1, "tr", 2) == 0)
          logRowFlags[LOG_TURNRATE] = plus;
        else if (strncasecmp(p1, "pose", 4) == 0)
          logRowFlags[LOG_POSE] = plus;
        else if (strncasecmp(p1, "line", 4) == 0)
          logRowFlags[LOG_LINE] = plus;
        else if (strncasecmp(p1, "dist", 4) == 0)
          logRowFlags[LOG_DIST] = plus;
        else if (strncasecmp(p1, "bat", 3) == 0)
          logRowFlags[LOG_BATT] = plus;
        else if (strncasecmp(p1, "ct", 2) == 0)
          logRowFlags[LOG_CTRLTIME] = plus;
        //       else if (strncasecmp(p1, "bcl", 3) == 0)
        //         logRowFlags[LOG_BAL_CTRL] = plus;
        else if (strncasecmp(p1, "extra", 5) == 0)
          logRowFlags[LOG_EXTRA] = plus;
        else if (strncasecmp(p1, "chirp", 5) == 0)
          logRowFlags[LOG_CHIRP] = plus;
        //
        initLogStructure(100000 / CONTROL_PERIOD_10us);
        
      }
    } 
    else if (strncmp(buf, "lcl ", 4) == 0)
    { // log controller values
      // format, e.g. lcl 0 1 1 1 0 0 0 1 0 0
      p2 = &buf[4];
      logRowFlags[LOG_CTRL_VELL] = strtol(p2, &p2, 10);
      logRowFlags[LOG_CTRL_VELR] = logRowFlags[LOG_CTRL_VELL];
      logRowFlags[LOG_CTRL_TURN] = strtol(p2, &p2, 10);
      logRowFlags[LOG_CTRL_POS] = strtol(p2, &p2, 10);
      logRowFlags[LOG_CTRL_EDGE] = strtol(p2, &p2, 10);
      logRowFlags[LOG_CTRL_WALL] = strtol(p2, &p2, 10);
      logRowFlags[LOG_CTRL_FWD_DIST] = strtol(p2, &p2, 10);
      logRowFlags[LOG_CTRL_BAL] = strtol(p2, &p2, 10);
      logRowFlags[LOG_CTRL_BAL_VEL] = strtol(p2, &p2, 10);
      logRowFlags[LOG_CTRL_BAL_POS] = strtol(p2, &p2, 10);
      initLogStructure(100000 / CONTROL_PERIOD_10us);
      // debug
//       snprintf(s, MSL, "# lcl %d %d %d %d %d %d %d %d %d\r\n",
//                logRowFlags[LOG_CTRL_VELL],
//                //  logRowFlags[LOG_CTRL_VELR], (both controlled by left wheel controller)
//                logRowFlags[LOG_CTRL_TURN],
//                logRowFlags[LOG_CTRL_POS],
//                logRowFlags[LOG_CTRL_EDGE],
//                logRowFlags[LOG_CTRL_WALL],
//                logRowFlags[LOG_CTRL_FWD_DIST],
//                logRowFlags[LOG_CTRL_BAL],
//                logRowFlags[LOG_CTRL_BAL_VEL],
//                logRowFlags[LOG_CTRL_BAL_POS]
//       );
//       usb_send_str(s);
    }
    else if (strncmp(buf, "mote", 4) == 0)
    { // motor enable
      uint8_t m1, m2;
      const char * p1 = &buf[4];
      // get two values - if no value, then 0 is returned
      m1 = strtol(p1, (char**)&p1, 10);
      m2 = strtol(p1, (char**)&p1, 10);
      motorSetEnable(m1, m2);
    }
    else if (strncmp(buf, "motw", 4) == 0)
    {
      int m1, m2;
      const char * p1 = &buf[4];
      // get two values - if no value, then 0 is returned
      m1 = strtol(p1, (char**)&p1, 10);
      m2 = strtol(p1, (char**)&p1, 10);
      motorSetAnkerPWM(m1, m2);
    }
    else if (strncmp(buf, "motv", 4) == 0)
    {
      float m1, m2;
      const char * p1 = &buf[4];
      // get two values - if no value, then 0 is returned
      m1 = strtof(p1, (char**)&p1);
      m2 = strtof(p1, (char**)&p1);
      motorAnkerVoltage[0] = m1;
      motorAnkerVoltage[1] = m2;
      //addMotorVoltage(m1, m2);
      // debug
      snprintf(s, MSL, "# command motv = %g %g\r\n", m1, m2);
      usb_send_str(s);
      // debug end
      // transfer to PWM
      if (fabs(m1) > 0.01 or fabs(m2) > 0.01)
        // this will take precedence to ant mission
        control.controlActive = false;
      else //if (control.missionState > 0)
        // back to mission control
        control.controlActive = true;
      motorSetAnchorVoltage();
    }
    else if (strncmp(buf, "rid", 3) == 0)
    { // robot ID and other permanent stuff
      setRobotID(&buf[4]);
    }
    else if (strncmp(buf, "gyroo", 5) == 0)
    {
      gyroOffsetDone = false;
    }
    else if (strncmp(buf,"mem", 3) == 0)
    { // memory usage
      snprintf(s, MSL, "# Main 3 areana=%d (m1=%x &time=%x s=%x)\n", mallinfo().arena, 
               (unsigned int)m1, (unsigned int)&time, (unsigned int)s);
      usb_send_str(s);      
      snprintf(s, MSL, "# Main log buffer from %x to %x\n", (unsigned int)logBuffer, (unsigned int)&logBuffer[LOG_BUFFER_MAX-1]);
      usb_send_str(s);      
      snprintf(s, MSL, "# Mission line takes %d bytes (bool is %d bytes)\r\n", 
               sizeof(UMissionLine), sizeof(bool));
      usb_send_str(s);      
    }
    else if (buf[0] == ':')
    { // send to wifi serial connection
      int n = strlen(buf);
      // remove newline
      while (buf[n-1] < ' ')
        n--;
      // add fresh carriage return and newline
      buf[n++] = '\r';
      buf[n++] = '\n';
      buf[n] = '\0';
      // send to 8266
      Serial1.write(&buf[1]);
      //
      wifi.replyType = UWifi8266::WFI_FINISHED;
    }
    else if (strncmp(buf,"wifi", 4) == 0)
    { // setup new wifi connection
      if (buf[4] == ' ' or buf[4] == '=')
        wifi.decodeWifi(&buf[5]);
      else if (wifi.wifiActive)
        // just close connections and restart wifi
        wifi.setup = 10;
    }
    else if (strncmp(buf,"link", 4) == 0)
    { // setup new wifi connection
      if (buf[4] == ' ' or buf[4] == '=')
      {
        char * p1 = &buf[5];
        int v = strtol(p1, &p1, 10);
        if (*p1 == ' ' or *p1 == ',')
        {
          p1++;
          wifi.wifiSend(v, p1, true);
          // debug
          snprintf(s, MSL, "# parse link : send to link %d :'%s'\r\n", v, buf);
          usb_send_str(s);
          // debug end
        }
        // debug
        else
        {
          snprintf(s, MSL, "# parse link : send to link %d failed:'%s'\r\n", v, buf);
          usb_send_str(s);
        }
        // debug end
      }
      // debug
      else
      {
        snprintf(s, MSL, "# parse link failed :'%s'\r\n", buf);
        usb_send_str(s);
      }
      // debug end
    }
    else if (strncmp(buf, "iron ", 5) == 0)
    {
      const char * p1 = &buf[5];
      uint8_t v = strtol(p1, (char**)&p1, 10);
      setIRpower(v != 0);
      usb_send_str("# ir set\n");
    }
    else if (strncmp(buf, "svo", 3) == 0)
    {
      const char * p1 = &buf[3];
      /*int8_t e =*/ strtol(p1, (char**)&p1, 10);
      if (p1 == &buf[3])
        // no valid number - so report
        servo.sendServoStatus();
      else
        // new values
        servo.setServoConfig(&buf[3]);
    }
    else if (strncmp(buf, "sv1", 3) == 0)
    { // set servo 1 as steering (or not)
      const char * p1 = &buf[3];
      /*int8_t e =*/ strtol(p1, (char**)&p1, 10);
      if (p1 == &buf[3])
        // no valid number - so report status instead
        servo.sendServo1Status();
      else
        // new values
        servo.setServo1Config(&buf[3]);
    }
    else if (strncmp(buf, "servo ", 6) == 0)
    {
      servo.setOneServo(&buf[6]);
    }
    else if (strncmp(buf, "ene", 3) == 0)
    { // encoder calibrate flags
      char * p2 = strchr(&buf[4],'=');
      bool isOK = p2 != NULL;
      if (isOK)
      {
        char * p1 = p2 + 1;
        int v = strtol(p1, &p2, 10);
        switch (buf[3])
        {
          case 'c':
            encTimeStopCollect = v == 0;
            break;
          case 'u':
            encTimeCalibrateUse = v != 0;
            break;
          case 'a':
            encTimeTryCalibrate = v != 0;
            if (v != 0)
              encTimeScaleCalibrated = false;
            break;
          default:
            isOK=false;
            break;
        }
      }
      if (not isOK)
      {
        usb_send_str("# encoder calibration flag set failed, expected a value (eneX=1 or eneX=0), X=c,u or a\n");
      }
    }
    else if (strncmp(buf, "enc", 3) == 0)
    {
      if (strncmp(buf, "encc", 4) == 0)
      { // not a disable, so run calibrate
        calibrateEncoder();
        usb_send_str("# motors were assumed to run, calbrated to RAM, use u25..u29 to get values\r\n");
      }
      else if (strncmp(buf, "enci", 4) == 0)
      {
        bool isOK = false;
        if (encTimeStopCollect)
          usb_send_str("# no encoder timing is collected, start that first be e.g. enec=1\n");
        else
          isOK = calibrateEncoderIndex();
        if (isOK)
          usb_send_str("# calibration index found\r\n");
        else
          usb_send_str("# calibration index NOT found - missing calibration? has never run?\r\n");
      }
      else
      { // send current encoder values
        sendStatusSensor(4);
      }
    }
    else if (strncmp(buf, "silent", 6) == 0)
    {
      char * p2 = strchr(&buf[4],'=');
      int v = 1;
      if (p2 != NULL)
      {
        p2++;
        v = strtol(p2, NULL, 10);
      }
      silenceUSBauto = v != 0;
      silentUSB = false;
    }
    else if (strncmp(buf, "sub", 3) == 0)
    { //
      char * p1 = &buf[3];
      char * p2 = p1;
      int s = strtol(p1, &p2, 10);
      bool isOK = p1 != p2;     
      int p = strtol(p2, &p1, 10);
      isOK &= p1 != p2;
      int m = strtol(p1, &p2, 10);
      isOK &= p1 != p2;
      if (isOK)
      { // all 3 parameters precent
//         usb_send_str("# Got sub s p m\n");
        if (s)
          subscribe.subscribe(USubscribe::MESSAGE_TYPES(m), p);
        else
          subscribe.unsubscribe(USubscribe::MESSAGE_TYPES(m));
      }
      else
      { // no or missing parameters
        subscribe.sendSubscribeStatus();
      }
    }
    else if (strncmp(buf, "sut", 3) == 0)
    {
      char * p1 = &buf[3];
      char * p2 = p1;
      int t = strtol(p1, &p2, 10);
      bool isOK = p1 != p2;     
      int p = strtol(p2, &p1, 10);
      isOK &= p1 != p2;
      if (isOK and p >=0 and p < subscribe.MAX_PRIORITIES)
      { // got parameters
        subscribe.sendInterval[p] = t;
      }
      else
      { // missing parameters - send status
        usb_send_str("sut is missing parameters: 'sut t p' t > 0 [ms], p = 0..4\n");
        subscribe.sendSubscribeStatus();
      }
    }
    else if (strlen(buf) > 1)
    {
      snprintf(s, MSL, "#** unknown %dc from %d command '%s'\r\n", strlen(buf), serChannel,  buf);
      usb_send_str(s);
      usb_serial_flush_input();
    }
  }
}

/////////////////////////////////////////////////

bool client_send_str(const char * str, bool blocking);

bool usb_send_str(const char* str, bool blocking)
{
  bool send;
  if (str[0] == '#')
  { // messages with a # is send to USB only (debug message)
    int8_t client = requestingClient;
    // do not use client as destination, use USB
    requestingClient = -1; // -1 is USB
    send = client_send_str(str, blocking);
    requestingClient = client;
  }
  else
    send = client_send_str(str, blocking);
  return send;
}

//////////////////////////////////////////////////

bool client_send_str(const char * str, bool blocking) //, bool toUSB, bool toWifi)
{
  int n = strlen(str);
  bool okSend = true;
  // a human client, so send all to USB and other clients
  if (not silentUSB)
  { // sending may give a timeout, and then the rest is not send!
    // this happends especially often on lenovo PCs, but they are especially slow in data transfer.
    if (not blocking)
      // surplus characters will be skipped
      usb_serial_write(str, n);
    else
    { // ensure all is send (like log data)
      int k = n, m;
      const char * p1 = str;
      uint32_t t0 = hbTimerCnt;
      // debug
//       usb_serial_write("#blocking\r\n", 11);
      // debug end
      // send as much as possible at a time
      while (k > 0 /*and usbWriteFullCnt < usbWriteFullLimit*/)
      {
        m = usb_serial_write_buffer_free();
        if (m > 0)
        {
          if (m > k)
            m = k;
          usb_serial_write(p1, m);
          k = k - m;
          if (k <= 0)
            break;
          p1 += m;
        }
        if (hbTimerCnt - t0 > 1500)
          // wait no more
          break;
      }
    }
  }
  // serial connection (serial3) has the next number after wifi
  /*else*/ 
  if (requestingClient == UWifi8266::WIFI_MAX_CLIENTS)
  { // this may give a timeout, and the rest is not send!
    // this happends especially often on lenovo PCs, but they are especially slow in data transfer.
    // surplus characters will be skipped
    Serial3.write((const uint8_t *)str, n);
  }
  //     if (requestingClient == -2)
  //     { // a push message - send to first wifi channel only
  //       if (str[n - 1] == '\n' and n > 4)
  //       { 
  //         for (int i = 0; i < UWifi8266::WIFI_MAX_CLIENTS; i++)
  //         {
  //           if (wifi.clientActive[i] != UWifi8266::WIFI_NOT)
  //           {
  //             if (blocking or wifi.clientActive[i] == UWifi8266::WIFI_ALL or 
  //               (wifi.clientActive[i] == UWifi8266::WIFI_NO_HASH and str[0] != '#' and not blocking))
  //             { // send if available time, or wait if blocking is requested
  //               okSend = wifi.wifiSend(i, str, n);
  //               // can send to one client at a time only
  //               break;
  //             }
  // //             else
  // //             {
  // //               const int MSL = 50;
  // //               char s[MSL];
  // //               requestingClient = -1;
  // //               snprintf(s, MSL, "# client %d has state %d\r\n", i, wifi.clientActive[i]);
  // //               usb_send_str(s);
  // //             }
  //           }
  //         }
  //       }
  //     }
  else if (requestingClient >= 0 and requestingClient < UWifi8266::WIFI_MAX_CLIENTS)
  {
    // debug
    //     const int MSL = 100;
    //     char s[MSL];
    //     snprintf(s, MSL, "# wifi try to send %d (strlen=%d) bytes %s\r\n", n, strlen(str), str);
    //     usb_send_str(s);
    // debug end
    okSend = wifi.wifiSend(requestingClient, str, n);
  }
  return okSend;
}

////////////////////////////////////////////////////////////////

// void sendStatus()
// {
//   if ((control.missionState == 0 or sendStatusWhileRunning) and gyroOffsetDone)
//   { // send status
//     pushStatus++;
//     if (pushStatus % 2 == 0)
//       // sendHartBeat();
//       sendStatusSensor(-1);
//     else
//     { // else one of the slow changing data types
//       switch(pushStatus / 2)
//       {
//         case 0 : 
//           //           if (sendStatusControl(-1) != 0)
//           //             // stay here until all is send
//           //             pushStatus -= 2;
//           break;
//         case 1 : 
//           control.sendMissionStatusChanged(false); 
//           break;
//         case 2 : 
//           sendStatusLogging(); 
//           break;
//         case 3 : 
//           sendStatusRobotID(); 
//           break;
//         case 4:
//           sendStatusLineSensor(false);
//           break;
//         case 5:
//           sendStatusDistIR();
//           break;
//         case 6:
//           sendStatusVersion();
//           break;
//         case 7:
//           wifi.sendStatusWiFi();
//           break;
//         case 8:
//           wifi.sendStatusWiFiClients();
//           break;
//         default:
//           pushStatus = 0;
//       }
//     }
//   }
//   else if (not gyroOffsetDone and hbTimerCnt % 1000 == 0)
//   {
//     usb_send_str("# IMU: gyro not calibrated\n");
//   }
// }
////////////////////////////////////////////////////////////////

