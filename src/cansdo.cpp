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
#include <libopencm3/stm32/desig.h>
#include <libopencm3/cm3/scb.h>
#include "cansdo.h"
#include "param_save.h"

#define SDO_REQUEST_DOWNLOAD  (1 << 5)
#define SDO_REQUEST_UPLOAD    (2 << 5)
#define SDO_REQUEST_SEGMENT   (3 << 5)
#define SDO_TOGGLE_BIT        (1 << 4)
#define SDO_RESPONSE_UPLOAD   (2 << 5)
#define SDO_RESPONSE_DOWNLOAD (3 << 5)
#define SDO_EXPEDITED         (1 << 1)
#define SDO_SIZE_SPECIFIED    (1)
#define SDO_WRITE             (SDO_REQUEST_DOWNLOAD | SDO_EXPEDITED | SDO_SIZE_SPECIFIED)
#define SDO_READ              SDO_REQUEST_UPLOAD
#define SDO_ABORT             0x80
#define SDO_WRITE_REPLY       SDO_RESPONSE_DOWNLOAD
#define SDO_READ_REPLY        (SDO_RESPONSE_UPLOAD | SDO_EXPEDITED | SDO_SIZE_SPECIFIED)
#define SDO_ERR_INVIDX        0x06020000
#define SDO_ERR_RANGE         0x06090030
#define SDO_ERR_GENERAL       0x08000000
#define SDO_REQ_ID_BASE       0x600U
#define SDO_REP_ID_BASE       0x580U

#define SDO_INDEX_PARAMS      0x2000
#define SDO_INDEX_PARAM_UID   0x2100
#define SDO_INDEX_MAP         0x3000
#define SDO_INDEX_SERIAL      0x5000
#define SDO_INDEX_STRINGS     0x5001
#define SDO_INDEX_COMMANDS    0x5002
#define SDO_CMD_SAVE          0
#define SDO_CMD_LOAD          1
#define SDO_CMD_RESET         2
#define SDO_CMD_DEFAULTS      3

/** \brief
 *
 * \param hw CanHardware*
 * \param cm CanMap*
 *
 */
CanSdo::CanSdo(CanHardware* hw, CanMap* cm)
 : canHardware(hw), canMap(cm), nodeId(1), remoteNodeId(255), printRequest(-1), printComplete(true)
{
   canHardware->AddReceiveCallback(this);
   HandleClear();
}

//Somebody (perhaps us) has cleared all user messages. Register them again
void CanSdo::HandleClear()
{
   canHardware->RegisterUserMessage(SDO_REQ_ID_BASE + nodeId);

   if (remoteNodeId < 64)
      canHardware->RegisterUserMessage(SDO_REP_ID_BASE + remoteNodeId);
}

bool CanSdo::HandleRx(uint32_t canId, uint32_t data[2])
{
   if (canId == (SDO_REQ_ID_BASE + nodeId)) //SDO request
   {
      ProcessSDO(data);
      return true;
   }
   else if (canId == (SDO_REP_ID_BASE + nodeId))
   {
      sdoReplyValid = (data[0] & 0xFF) != SDO_ABORT;
      sdoReplyData = data[1];
      return true;
   }
   return false;
}

void CanSdo::SDOWrite(uint8_t nodeId, uint16_t index, uint8_t subIndex, uint32_t data)
{
   InitiateSDOTransfer(SDO_WRITE, nodeId, index, subIndex, data);
}

void CanSdo::SDORead(uint8_t nodeId, uint16_t index, uint8_t subIndex)
{
   InitiateSDOTransfer(SDO_READ, nodeId, index, subIndex, 0);
}

bool CanSdo::SDOReadReply(uint32_t& data)
{
   data = sdoReplyData;
   return sdoReplyValid;
}

void CanSdo::SetNodeId(uint8_t id)
{
   nodeId = id;
   canHardware->ClearUserMessages();
}

void CanSdo::InitiateSDOTransfer(uint8_t req, uint8_t nodeId, uint16_t index, uint8_t subIndex, uint32_t data)
{
   uint32_t d[2];
   CAN_SDO *sdo = (CAN_SDO*)d;

   sdo->cmd = req;
   sdo->index = index;
   sdo->subIndex = subIndex;
   sdo->data = data;

   if (nodeId != remoteNodeId)
   {
      remoteNodeId = nodeId;
      //This registers the reply message
      canHardware->ClearUserMessages();
   }

   sdoReplyValid = false;
   canHardware->Send(SDO_REQ_ID_BASE + remoteNodeId, d);
}

//http://www.byteme.org.uk/canopenparent/canopen/sdo-service-data-objects-canopen/
void CanSdo::ProcessSDO(uint32_t data[2])
{
   CAN_SDO *sdo = (CAN_SDO*)data;

   if ((sdo->cmd & SDO_REQUEST_SEGMENT) == SDO_REQUEST_SEGMENT)
   {
      sdo->cmd = sdo->cmd & SDO_TOGGLE_BIT;
      sdo->index = printBuffer[0] | (printBuffer[1] << 8);
      sdo->subIndex = printBuffer[2];
      sdo->data = *(uint32_t*)&printBuffer[3];

      if (printComplete)
      {
         sdo->cmd |= SDO_SIZE_SPECIFIED;
         sdo->cmd |= (7 - printByte) << 1; //specify how many bytes do NOT contain data
      }

      //Clear buffer
      for (uint32_t i = 0; i < sizeof(printBuffer); i++)
         printBuffer[i] = 0;

      printByte = 0; //reset buffer index to allow printing
   }
   else if (sdo->index == SDO_INDEX_PARAMS || (sdo->index & 0xFF00) == SDO_INDEX_PARAM_UID)
   {
      Param::PARAM_NUM paramIdx = (Param::PARAM_NUM)sdo->subIndex;

      //SDO index 0x21xx will look up the parameter by its unique ID
      //using subIndex as low byte and xx as high byte of ID
      if ((sdo->index & 0xFF00) == SDO_INDEX_PARAM_UID)
         paramIdx = Param::NumFromId(sdo->subIndex + ((sdo->index & 0xFF) << 8));

      if (paramIdx < Param::PARAM_LAST)
      {
         if (sdo->cmd == SDO_WRITE)
         {
            if (Param::Set(paramIdx, sdo->data) == 0)
            {
               sdo->cmd = SDO_WRITE_REPLY;
            }
            else
            {
               sdo->cmd = SDO_ABORT;
               sdo->data = SDO_ERR_RANGE;
            }
         }
         else if (sdo->cmd == SDO_READ)
         {
            sdo->data = Param::Get(paramIdx);
            sdo->cmd = SDO_READ_REPLY;
         }
      }
      else
      {
         sdo->cmd = SDO_ABORT;
         sdo->data = SDO_ERR_INVIDX;
      }
   }
   else if (0 != canMap && (sdo->index & 0xF000) == SDO_INDEX_MAP)
   {
      if (sdo->cmd == SDO_WRITE)
      {
         int result;

         if (sdo->subIndex == 0)
         {
            //Now we receive UID of value to be mapped along with bit start and length
            mapParam = Param::NumFromId(sdo->data & 0xFFFF);
            mapBit = (sdo->data >> 16) & 0x3F;
            mapLen = (sdo->data >> 24) & 0x1F;
            result = mapParam < Param::PARAM_LAST ? 0 : -1;
         }
         else if (sdo->subIndex == 3)
         {
            //Delete an item
            mapParam = Param::NumFromId(sdo->data & 0xFFFF);
            result = canMap->Remove(mapParam) - 1; //if no items are removed return error
         }
         else if (mapLen > 0) //This sort of verifies that we received subindex 0
         {
            //Now we receive gain and offset and add the map
            float gain = (sdo->data & 0xFFFFFF) / 1000.0f;
            int8_t offset = sdo->data >> 24;
            uint16_t id = sdo->index & 0x7FF;

            if (sdo->subIndex == 1) //TX map
               result = canMap->AddSend(mapParam, id, mapBit, mapLen, gain, offset);
            else if (sdo->subIndex == 2) //RX map
               result = canMap->AddRecv(mapParam, id, mapBit, mapLen, gain, offset);
            else
               result = -1;

            mapLen = 0;
         }

         if (result >= 0)
         {
            sdo->cmd = SDO_WRITE_REPLY;
         }
         else
         {
            sdo->cmd = SDO_ABORT;
            sdo->data = SDO_ERR_RANGE;
         }
      }
   }
   else
   {
      ProcessSpecialSDOObjects(sdo);
   }
   canHardware->Send(0x580 + nodeId, data);
}

void CanSdo::PutChar(char c)
{
   //When print buffer is full, wait
   while (printByte >= sizeof(printBuffer));
   printBuffer[printByte++] = c;
   printRequest = -1; //We can clear the print start trigger as we've obviously started printing
}

void CanSdo::ProcessSpecialSDOObjects(CAN_SDO* sdo)
{
   if (sdo->index == SDO_INDEX_SERIAL && sdo->cmd == SDO_READ)
   {
      sdo->cmd = SDO_READ_REPLY;
      switch (sdo->subIndex)
      {
      case 0:
         sdo->data = DESIG_UNIQUE_ID0;
         break;
      case 1:
         sdo->data = DESIG_UNIQUE_ID1;
         break;
      case 2:
         sdo->data = DESIG_UNIQUE_ID2;
         break;
      case 3:
         sdo->data = Param::GetIdSum();
         break;
      default:
         sdo->cmd = SDO_ABORT;
         sdo->data = SDO_ERR_INVIDX;
      }
   }
   else if (sdo->index == SDO_INDEX_STRINGS)
   {
      if (sdo->cmd == SDO_READ)
      {
         sdo->data = 65535; //this should be the size of JSON but we don't know this in advance. Hmm.
         sdo->cmd = SDO_RESPONSE_UPLOAD | SDO_SIZE_SPECIFIED;
         printByte = 0; //reset buffer index to allow printing
         printComplete = false;
         printRequest = sdo->subIndex;
      }
   }
   else if (sdo->index == SDO_INDEX_COMMANDS && sdo->cmd == SDO_WRITE)
   {
      sdo->cmd = SDO_WRITE_REPLY;

      switch (sdo->subIndex)
      {
      case SDO_CMD_SAVE:
         if (0 != canMap) canMap->Save();
         parm_save();
         break;
      case SDO_CMD_LOAD:
         parm_load();
         Param::Change(Param::PARAM_LAST);
         break;
      case SDO_CMD_RESET:
         scb_reset_system();
         break;
      case SDO_CMD_DEFAULTS:
         Param::LoadDefaults();
         Param::Change(Param::PARAM_LAST);
         break;
      default:
         sdo->cmd = SDO_ABORT;
         sdo->data = SDO_ERR_INVIDX;
      }
   }
}
