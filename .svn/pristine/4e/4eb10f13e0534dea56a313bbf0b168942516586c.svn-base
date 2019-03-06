/***************************************************************************
 *   Copyright (C) 2017 by DTU                             *
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

#include "subscribe.h"
#include "dist_sensor.h"
#include "linesensor.h"


USubscribe::USubscribe()
{ // set pointers to functions
  sender[MSG_HART_BEAT] = sendHartBeat;
  sender[MSG_POSE] = sendPose;
  sender[MSG_IR] = sendStatusDistIR;
  sender[MSG_EDGE] = sendLineSensorPosition;
  sender[MSG_ACC] = sendAcc;
  sender[MSG_GYRO] = sendGyro;
  sender[MSG_MOTOR_CURRENT] = sendMotorCurrent;
  sender[MSG_WHEEL_VEL] = sendWheelVelocity;
  // set default timing
  sendInterval[0] = 5; // unit in ms, highest priority
  for (int i = 1; i < MAX_PRIORITIES; i++)
  {
    sendInterval[i] = sendInterval[i-1] * 5; // unit in ms
  }
  // clear last-send value and clear subscriptions flags
  for (int i = 0; i < MAX_PRIORITIES; i++)
  {
    lastSendTime[i] = 0;
    subsc[i] = 0;
    lastSendMsg[i] = 0;
  }
};

/////////////////////////////////////////////////////

USubscribe::~USubscribe()
{
}

//////////////////////////////////////////////////

bool USubscribe::subscribe(USubscribe::MESSAGE_TYPES msg, uint8_t priority)
{
  bool isOK = msg < MSG_MAX and msg >= 0 and priority < MAX_PRIORITIES and priority >= 0;
  if (isOK)
  { // remove old priority
    for (int i = 0; i < MAX_PRIORITIES; i++)
      subsc[i] &= ~(1 << msg);
    // add new
    subsc[priority] |= 1 << msg;
  }
  return isOK;
}

///////////////////////////////////////////////

void USubscribe::unsubscribe(USubscribe::MESSAGE_TYPES msg)
{
  bool isOK = msg < MSG_MAX and msg >= 0;
  if (isOK)
  { // remove from all priorities
    for (int i = 0; i < MAX_PRIORITIES; i++)
      subsc[i] &= ~(1 << msg);
  }
}

/////////////////////////////////////////

void USubscribe::sendToSubscriber()
{ // check for space in USB buffer
  bool space4message;
  if (requestingClient >= 0)
  { // client is on wifi
    int m = Serial1.availableForWrite();
    space4message = m > 20;
  }
  else
  { // client is on USB
    int m = usb_serial_write_buffer_free();
    space4message = m > 60;
  }
  if (space4message)
  { // there is space for at least 60 bytes
    bool msgSend = false;
    for (int p = 0; p < MAX_PRIORITIES and not msgSend; p++)
    { // try all priorities, starting with 0
      if (subsc[p] > 0)
      { // there is messages in priority
        if (hbTimerCnt - lastSendTime[p] > sendInterval[p])
        { // send next message
          int f = lastSendMsg[p];
          for (int m = 0; m < MSG_MAX; m++)
          { // try the message next after last
            f = (f+1) % MSG_MAX;
//             { // debug
//               const int MSL = 50;
//               char s[MSL];
//               snprintf(s, MSL, "#subs f=%d, mask=%d OK=%d\n", f, 1 << f, subsc[p] & (1 << f));
//               usb_send_str(s);
//             }
            if (subsc[p] & (1 << f))
            { // send to USB only
              // requestingClient = -1;
              // send requested message
              sender[f]();
              // save this as last message send
              lastSendMsg[p] = f;
              // save time send
              lastSendTime[p] = hbTimerCnt;
              msgSend = true;
              break;
            }
          }
        }
      }
    }
  }
}

//////////////////////////////////////

void USubscribe::sendSubscribeStatus()
{
  const int MSL = 100;
  char s[MSL];
  for (int p=0; p < MAX_PRIORITIES; p++)
  {
    snprintf(s, MSL, "sup %d %d  %d %d %d %d %d %d %d %d\r\n", p, sendInterval[p],
             (subsc[p] & (1 << MSG_HART_BEAT)) != 0,
             (subsc[p] & (1 << MSG_POSE)) != 0,
             (subsc[p] & (1 << MSG_IR)) != 0,
             (subsc[p] & (1 << MSG_EDGE)) != 0,
             (subsc[p] & (1 << MSG_ACC)) != 0,
             (subsc[p] & (1 << MSG_GYRO)) != 0,
             (subsc[p] & (1 << MSG_MOTOR_CURRENT)) != 0,
             (subsc[p] & (1 << MSG_WHEEL_VEL)) != 0
    );
    usb_send_str(s);
  }
}
