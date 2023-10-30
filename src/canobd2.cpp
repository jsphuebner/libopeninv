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
#include "canobd2.h"


/* Details from http://en.wikipedia.org/wiki/OBD-II_PIDs */
#define OBD2_MODE1               0x01        //Show current data
#define OBD2_MODE2               0x02        //Show freeze frame data
#define OBD2_MODE3               0x03        //Show stored Diagnostic Trouble Codes
#define OBD2_MODE4               0x04        //Clear Diagnostic Trouble Codes and stored values
#define OBD2_MODE42              0x2A

#define OBD2_PID_SUPPORTED       0x00
#define OBD2_MONITOR_STATUS      0x01

#define OBD2_MODE1_RESPONSE      0x41
#define OBD2_MODE3_RESPONSE      0x43
#define OBD2_MODE4_RESPONSE      0x44
#define OBD2_MODE42_RESPONSE     0x6A
#define OBD2_PID_REQUEST         0x7DFU
#define OBD2_PID_REPLY           0x7E8U

/** \brief
 *
 * \param hw CanHardware*
 * \param cm CanMap*
 *
 */
CanObd2::CanObd2(CanHardware* hw)
 : canHardware(hw), nodeId(0)
{
   canHardware->AddCallback(this);
   HandleClear();
}

//Somebody (perhaps us) has cleared all user messages. Register them again
void CanObd2::HandleClear()
{
   canHardware->RegisterUserMessage(OBD2_PID_REQUEST); // Broadcast address
   canHardware->RegisterUserMessage(OBD2_PID_REQUEST + nodeId); // ECU specific address
}

bool CanObd2::HandleRx(uint32_t canId, uint32_t data[2], uint8_t)
{
   if ((canId == OBD2_PID_REQUEST) || (canId == (OBD2_PID_REQUEST + nodeId))) //OBD2 request
   {
      ProcessOBD2(data);
      return true;
   }
   return false;
}

void CanObd2::SetNodeId(uint8_t id)
{
   nodeId = id;
   canHardware->ClearUserMessages();
}

void CanObd2::ProcessOBD2(uint32_t data[2])
{
   uint8_t *bytes = (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
   uint8_t response[8] = {0x00};

   switch (bytes[1])
   {
   case OBD2_MODE3:
      response[0] = 0x02;
      response[1] = OBD2_MODE3_RESPONSE;
      break;
   case OBD2_MODE4:
      response[1] = OBD2_MODE4_RESPONSE;
      break;
   case OBD2_MODE1:
      response[1] = OBD2_MODE1_RESPONSE;
      switch (bytes[2])
      {
      case OBD2_PID_SUPPORTED:
         response[0] = 0x06;
         response[2] = OBD2_PID_SUPPORTED;
         response[3] = 0x00;
         response[4] = 0x00;
         response[5] = 0x00;
         response[6] = 0x00;
         response[7] = 0x00;
         break;
      case OBD2_MONITOR_STATUS:
         response[0] = 0x05;
         response[2] = OBD2_MONITOR_STATUS;
         response[3] = 0x00;
         response[4] = 0x07;
         response[5] = 0xFF;
         break;
      }
      break;
   case OBD2_MODE42:
      response[0] = 7;
      uint16_t pid = bytes[2] * 256 + bytes[3];
      if (Param::NumFromId(pid) != Param::PARAM_INVALID)
      {
         response[1] = OBD2_MODE42_RESPONSE;
         s32fp val = Param::Get(Param::NumFromId(pid));
         response[2] = bytes[2];
         response[3] = bytes[3];
         response[4] = (val >> 24) & 0xFF;
         response[5] = (val >> 16) & 0xFF;
         response[6] = (val >> 8) & 0xFF;
         response[7] = val & 0xFF;
      }
      else
      {
         response[0] = 3;
         response[1] = 0x7F;
         response[2] = 0x22;
         response[3] = 0x31;
      }
      break;
   }
   canHardware->Send((OBD2_PID_REPLY + nodeId), response, 8);
}
