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

#ifndef SUBSCRIBE_H
#define SUBSCRIBE_H

#include <stdint.h>
#include "main.h"
#include "command.h"


/**
 * This class if for subscription of data messages over USB only 
 * */
class USubscribe
{
public:
  /** constructor */
  USubscribe();
  /** destructor */
  ~USubscribe();
  typedef enum  {
    MSG_HART_BEAT,
    MSG_POSE,
    MSG_IR,
    MSG_EDGE,
    MSG_ACC,
    MSG_GYRO,
    MSG_MOTOR_CURRENT,
    MSG_WHEEL_VEL,
    MSG_MAX} MESSAGE_TYPES;
    /**
   * subscribe to a message */
  bool subscribe(MESSAGE_TYPES msg, uint8_t priority);
  /**
   * remove subscription */
  void unsubscribe(MESSAGE_TYPES msg);
  /**
   * Send to subscriber if needed */
  void sendToSubscriber();
  /**
   * send current subscribe status */
  void sendSubscribeStatus();
  
public:
  /// number of priorities (0 is highest)
  static const int MAX_PRIORITIES = 5;
  /// minimum interval between messages in ms
  /// if too low, only high priority will be send
  uint16_t sendInterval[MAX_PRIORITIES];
  
private:
  int subsc[MAX_PRIORITIES];
  /// function list
  void (*sender[MSG_MAX])(void);
  /// last time a message was send (in ms)
  uint32_t lastSendTime[MAX_PRIORITIES];
  /// number of last send message in each priority
  uint8_t lastSendMsg[MAX_PRIORITIES];
};

#endif