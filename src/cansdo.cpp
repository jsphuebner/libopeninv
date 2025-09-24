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
#include "cansdo.h"
#include "my_math.h"
#include "errormessage.h"

#define SDO_REQ_ID_BASE       0x600U
#define SDO_REP_ID_BASE       0x580U

#define SDO_INDEX_PARAMS      0x2000
#define SDO_INDEX_PARAM_UID   0x2100
#define SDO_INDEX_MAP_TX      0x3000
#define SDO_INDEX_MAP_RX      0x3001
#define SDO_INDEX_MAP_RD      0x3100
#define SDO_INDEX_STRINGS     0x5001
#define SDO_INDEX_ERROR_NUM   0x5002
#define SDO_INDEX_ERROR_TIME  0x5003


#define PRINT_BUF_ENQUEUE(c)  printBuffer[(printByteIn++) & (sizeof(printBuffer) - 1)] = c
#define PRINT_BUF_DEQUEUE()   printBuffer[(printByteOut++) & (sizeof(printBuffer) - 1)]
#define PRINT_BUF_EMPTY()     ((printByteOut - printByteIn) == sizeof(printBuffer))

/** \brief
 *
 * \param hw CanHardware*
 * \param cm CanMap*
 *
 */
CanSdo::CanSdo(CanHardware* hw, CanMap* cm)
 : canHardware(hw), canMap(cm), nodeId(1), remoteNodeId(255), printRequest(-1)
{
   canHardware->AddCallback(this);
   HandleClear();
}

//Somebody (perhaps us) has cleared all user messages. Register them again
void CanSdo::HandleClear()
{
   canHardware->RegisterUserMessage(SDO_REQ_ID_BASE + nodeId);

   if (remoteNodeId < 64)
      canHardware->RegisterUserMessage(SDO_REP_ID_BASE + remoteNodeId);
}

void CanSdo::HandleRx(uint32_t canId, uint32_t data[2], uint8_t)
{
   if (canId == (SDO_REQ_ID_BASE + nodeId)) //SDO request
   {
      ProcessSDO(data);
   }
   else if (canId == (SDO_REP_ID_BASE + remoteNodeId))
   {
      SdoFrame* sdoFrame = (SdoFrame*)data;
      if (sdoFrame->index == SDO_INDEX_MAP_RX || sdoFrame->index == SDO_INDEX_MAP_TX)
      {
         if (sdoFrame->subIndex == 0)
            InitiateSDOTransfer(SDO_WRITE, remoteNodeId, sdoFrame->index, 1, mapInfo.mapParam | (mapInfo.offsetBits << 16) | (mapInfo.numBits << 24));
         else if (sdoFrame->subIndex == 1)
            InitiateSDOTransfer(SDO_WRITE, remoteNodeId, sdoFrame->index, 2, (int32_t)(mapInfo.gain * 1000.0f) | (mapInfo.offset << 24));
      }
      sdoReplyValid = sdoFrame->cmd != SDO_ABORT;
      sdoReplyData = sdoFrame->data;
      //TODO: check indexes against request in case there are stray SDO replies
   }
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

void CanSdo::RemoteMap(uint8_t nodeId, bool rx, uint32_t cobId, CanMap::CANPOS mapping)
{
   mapInfo = mapping;

   InitiateSDOTransfer(SDO_WRITE, nodeId, rx ? SDO_INDEX_MAP_RX : SDO_INDEX_MAP_TX, 0, cobId);
}

void CanSdo::SetNodeId(uint8_t id)
{
   nodeId = id;
   canHardware->ClearUserMessages();
}

void CanSdo::InitiateSDOTransfer(uint8_t req, uint8_t nodeId, uint16_t index, uint8_t subIndex, uint32_t data)
{
   uint32_t d[2];
   SdoFrame *sdo = (SdoFrame*)d;

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
   SdoFrame *sdo = (SdoFrame*)data;

   if ((sdo->cmd & SDO_REQUEST_SEGMENT) == SDO_REQUEST_SEGMENT)
   {
      const int bytesPerMessage = 7;
      uint8_t *bytes = (uint8_t*)data;
      int i = 1;

      sdo->cmd = sdo->cmd & SDO_TOGGLE_BIT;

      for (; i <= bytesPerMessage && !PRINT_BUF_EMPTY(); i++)
         bytes[i] = PRINT_BUF_DEQUEUE();

      if (PRINT_BUF_EMPTY())
      {
         sdo->cmd |= SDO_SIZE_SPECIFIED;
         sdo->cmd |= (bytesPerMessage - i + 1) << 1; //specify how many bytes do NOT contain data
      }
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
   else if (0 != canMap && sdo->index == SDO_INDEX_MAP_TX)
   {
      AddCanMap(sdo, false);
   }
   else if (0 != canMap && sdo->index == SDO_INDEX_MAP_RX)
   {
      AddCanMap(sdo, true);
   }
   else if (0 != canMap && (sdo->index & 0xFF00) == SDO_INDEX_MAP_RD)
   {
      ReadOrDeleteCanMap(sdo);
   }
   else if (sdo->index == SDO_INDEX_ERROR_NUM)
   {
      if (sdo->cmd == SDO_READ)
      {
         sdo->data = ErrorMessage::GetErrorNum(sdo->subIndex);
         sdo->cmd = SDO_READ_REPLY;
      }
      else
      {
         sdo->cmd = SDO_ABORT;
         sdo->data = SDO_ERR_INVIDX;
      }
   }
   else if (sdo->index == SDO_INDEX_ERROR_TIME)
   {
      if (sdo->cmd == SDO_READ)
      {
         sdo->data = ErrorMessage::GetErrorTime(sdo->subIndex);
         sdo->cmd = SDO_READ_REPLY;
      }
      else
      {
         sdo->cmd = SDO_ABORT;
         sdo->data = SDO_ERR_INVIDX;
      }
   }
   else
   {
      if (!ProcessSpecialSDOObjects(sdo))
         return; //Don't send reply when handled by user space
   }
   canHardware->Send(0x580 + nodeId, data);
}

void CanSdo::PutChar(char c)
{
   //When print buffer is full, wait
   while (printByteIn == printByteOut);

   PRINT_BUF_ENQUEUE(c);
   printRequest = -1; //We can clear the print start trigger as we've obviously started printing
}

void CanSdo::SendSdoReply(SdoFrame* sdoFrame)
{
   canHardware->Send(0x580 + nodeId, (uint32_t*)sdoFrame);
   pendingUserSpaceSdo = false;
}

bool CanSdo::ProcessSpecialSDOObjects(SdoFrame* sdo)
{
   if (sdo->index == SDO_INDEX_STRINGS)
   {
      if (sdo->cmd == SDO_READ)
      {
         sdo->data = 65535; //this should be the size of JSON but we don't know this in advance. Hmm.
         sdo->cmd = SDO_RESPONSE_UPLOAD | SDO_SIZE_SPECIFIED;
         printByteIn = 0;
         printByteOut = sizeof(printBuffer); //both point to the beginning of the physical buffer but virtually they are 64 bytes apart
         printRequest = sdo->subIndex;
         return true;
      }
   }
   else
   {
      pendingUserSpaceSdo = true;
      pendingUserSpaceSdoFrame.cmd = sdo->cmd;
      pendingUserSpaceSdoFrame.index = sdo->index;
      pendingUserSpaceSdoFrame.subIndex = sdo->subIndex;
      pendingUserSpaceSdoFrame.data = sdo->data;
   }
   return false;
}

void CanSdo::ReadOrDeleteCanMap(SdoFrame* sdo)
{
   bool rx = (sdo->index & 0x80) != 0;
   uint32_t canId;
   uint8_t itemIdx = MAX(0, sdo->subIndex - 1) / 2;
   const CanMap::CANPOS* canPos = canMap->GetMap(rx, sdo->index & 0x3f, itemIdx, canId);

   if (sdo->cmd == SDO_READ)
   {
      if (canPos != 0)
      {
         uint16_t id = Param::GetAttrib((Param::PARAM_NUM)canPos->mapParam)->id;

         if (sdo->subIndex == 0) //0 contains COB Id
            sdo->data = canId;
         else if (sdo->subIndex & 1) //odd sub indexes have data id, position and length
            sdo->data = id | (canPos->offsetBits << 16) | (canPos->numBits << 24);
         else //even sub indexes except 0 have gain and offset
            sdo->data = (uint32_t)(((int32_t)(canPos->gain * 1000)) & 0xFFFFFF) | (canPos->offset << 24);
         sdo->cmd = SDO_READ_REPLY;
      }
      else
      {
         sdo->cmd = SDO_ABORT;
         sdo->data = SDO_ERR_INVIDX;
      }
   }
   else if (sdo->cmd == SDO_WRITE && canPos != 0 && sdo->data == 0)
   {
      canMap->Remove(rx, sdo->index & 0x3f, itemIdx);
      sdo->cmd = SDO_WRITE_REPLY;
   }
   else
   {
      sdo->cmd = SDO_ABORT;
      sdo->data = SDO_ERR_INVIDX;
   }
}

void CanSdo::AddCanMap(SdoFrame* sdo, bool rx)
{
   if (sdo->cmd == SDO_WRITE)
   {
      int result = -1;

      if (sdo->subIndex == 0)
      {
         if (sdo->data < 0x20000000 || (sdo->data & ~CAN_FORCE_EXTENDED) < 0x800)
         {
            mapId = sdo->data;
            result = 0;
         }
         else
         {
            mapId = 0xFFFFFFFF;
         }
      }
      else if (mapId != 0xFFFFFFFF && sdo->subIndex == 1)
      {
         //Now we receive UID of value to be mapped along with bit start and length
         mapInfo.mapParam = Param::NumFromId(sdo->data & 0xFFFF);
         mapInfo.offsetBits = (sdo->data >> 16) & 0x3F;
         mapInfo.numBits = ((int32_t)sdo->data >> 24);
         result = mapInfo.mapParam < Param::PARAM_LAST ? 0 : -1;
      }
      else if (mapInfo.numBits != 0 && sdo->subIndex == 2) //This sort of verifies that we received subindex 1
      {
         //Now we receive gain and offset and add the map

         // sign extend the 24-bit integer to a 32-bit integer
         int32_t gainFixedPoint = (sdo->data & 0xFFFFFF) << (32-24);
         gainFixedPoint >>= (32-24);
         mapInfo.gain = gainFixedPoint / 1000.0f;
         mapInfo.offset = sdo->data >> 24;

         if (rx) //RX map
            result = canMap->AddRecv((Param::PARAM_NUM)mapInfo.mapParam, mapId, mapInfo.offsetBits, mapInfo.numBits, mapInfo.gain, mapInfo.offset);
         else
            result = canMap->AddSend((Param::PARAM_NUM)mapInfo.mapParam, mapId, mapInfo.offsetBits, mapInfo.numBits, mapInfo.gain, mapInfo.offset);

         mapInfo.numBits = 0;
         mapId = 0xFFFFFFFF;
      }

      if (result >= 0)
      {
         sdo->cmd = SDO_WRITE_REPLY;
      }
      else
      {
         sdo->cmd = SDO_ABORT;
         sdo->data = SDO_ERR_INVIDX;
      }
   }
}
