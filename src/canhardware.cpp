/*
 * This file is part of the stm32-... project.
 *
 * Copyright (C) 2021 Johannes Huebner <dev@johanneshuebner.com>
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
#include "canhardware.h"

class NullCallback: public CanCallback
{
public:
   void HandleRx(uint32_t, uint32_t*, uint8_t) override {}
   void HandleClear() override { }
};


static NullCallback nullCallback;

CanHardware::CanHardware()
   : nextUserMessageIndex(0), nextCallbackIndex(0)
{
   for (int i = 0; i < MAX_RECV_CALLBACKS; i++)
   {
      recvCallback[i] = &nullCallback;
   }
}

/** \brief Add interface to be called for user handled CAN messages
 *
 * \param recv CanCallback interface
 */
bool CanHardware::AddCallback(CanCallback* recv)
{
   if (nextCallbackIndex < MAX_RECV_CALLBACKS)
   {
      recvCallback[nextCallbackIndex] = recv;
      nextCallbackIndex++;
      return true;
   }
   return false;
}

/** \brief Add CAN Id to user message list
 * canId can be 0x20000000 + std id to force registering a filter for an extended ID
 * even if the Id is < 0x7ff
 * \post Receive callback will be called when a message with this Id id received
 * \param canId CAN identifier of message to be user handled
 * \return true: success, false: already maximum messages registered
 *
 */
bool CanHardware::RegisterUserMessage(uint32_t canId, uint32_t mask)
{
   if (nextUserMessageIndex < MAX_USER_MESSAGES)
   {
      for (int i = 0; i < nextUserMessageIndex; i++)
      {
         if (canId == userIds[i]) //already exists
            return false; //do not add again
      }

      userIds[nextUserMessageIndex] = canId;
      userMasks[nextUserMessageIndex] = mask;
      nextUserMessageIndex++;
      ConfigureFilters();
      return true;
   }
   return false;
}

/** \brief Remove all CAN Id from user message list
 */
void CanHardware::ClearUserMessages()
{
   nextUserMessageIndex = 0;
   ConfigureFilters();

   for (int i = 0; i < nextCallbackIndex; i++)
   {
      recvCallback[i]->HandleClear();
   }
}

void CanHardware::HandleRx(uint32_t canId, uint32_t data[2], uint8_t dlc)
{
   for (int i = 0; i < nextCallbackIndex; i++)
   {
      recvCallback[i]->HandleRx(canId, data, dlc);
   }
}
