/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2018 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef CANHARDWARE_H
#define CANHARDWARE_H

#include <stdint.h>

#ifndef MAX_USER_MESSAGES
#define MAX_USER_MESSAGES 30
#endif

#ifndef MAX_RECV_CALLBACKS
#define MAX_RECV_CALLBACKS 5
#endif

class CanCallback
{
public:
   virtual bool HandleRx(uint32_t canId, uint32_t data[2]) = 0;
   virtual void HandleClear() = 0;
};

class FunctionPointerCallback: public CanCallback
{
public:
   FunctionPointerCallback(bool (*r)(uint32_t, uint32_t*), void (*c)()) : recv(r), clear(c) { };
   bool HandleRx(uint32_t canId, uint32_t data[2]) { return recv(canId, data); }
   void HandleClear() { clear(); }

private:
   bool (*recv)(uint32_t, uint32_t*);
   void (*clear)();
};

class CanHardware
{
   public:
      enum baudrates
      {
         Baud125, Baud250, Baud500, Baud800, Baud1000, BaudLast
      };

      CanHardware();
      virtual void SetBaudrate(enum baudrates baudrate) = 0;
      void Send(uint32_t canId, uint32_t data[2]) { Send(canId, data, 8); }
      void Send(uint32_t canId, uint8_t data[8], uint8_t len) { Send(canId, (uint32_t*)data, len); }
      virtual void Send(uint32_t canId, uint32_t data[2], uint8_t len) = 0;
      void HandleRx(uint32_t canId, uint32_t data[2]);
      bool AddCallback(CanCallback* cb);
      bool RegisterUserMessage(uint32_t canId);
      void ClearUserMessages();
      /** \brief Get RTC time when last message was received
       *
       * \return uint32_t RTC time
       *
       */
      uint32_t GetLastRxTimestamp() { return lastRxTimestamp; }

   protected:
      uint16_t userIds[MAX_USER_MESSAGES];
      int nextUserMessageIndex;
      uint32_t lastRxTimestamp;

   private:
      int nextCallbackIndex;
      CanCallback* recvCallback[MAX_RECV_CALLBACKS];

      virtual void ConfigureFilters() = 0;
};

#endif // CANHARDWARE_H
