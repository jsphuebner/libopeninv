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
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/cm3/scb.h>
#include "canmap.h"
#include "hwdefs.h"
#include "my_string.h"
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

#define SDO_INDEX_PARAMS      0x2000
#define SDO_INDEX_PARAM_UID   0x2100
#define SDO_INDEX_MAP_START   0x3000
#define SDO_INDEX_MAP_END     0x4800
#define SDO_INDEX_MAP_RX      0x4000
#define SDO_INDEX_SERIAL      0x5000
#define SDO_INDEX_PJSON       0x5001
#define SDO_INDEX_COMMANDS    0x5002
#define SDO_CMD_SAVE          0
#define SDO_CMD_LOAD          1
#define SDO_CMD_RESET         2

#define SENDMAP_ADDRESS(b)    b
#define RECVMAP_ADDRESS(b)    (b + sizeof(canSendMap))
#define POSMAP_ADDRESS(b)     (b + sizeof(canSendMap) + sizeof(canRecvMap))
#define CRC_ADDRESS(b)        (b + sizeof(canSendMap) + sizeof(canRecvMap) + sizeof(canPosMap))
#define SENDMAP_WORDS         (sizeof(canSendMap) / (sizeof(uint32_t)))
#define RECVMAP_WORDS         (sizeof(canRecvMap) / (sizeof(uint32_t)))
#define POSMAP_WORDS          ((sizeof(CANPOS) * MAX_ITEMS) / (sizeof(uint32_t)))
#define ITEM_UNSET            0xff
#define forEachCanMap(c,m) for (CANIDMAP *c = m; (c - m) < MAX_MESSAGES && c->first != MAX_ITEMS; c++)
#define forEachPosMap(c,m) for (CANPOS *c = &canPosMap[m->first]; c->next != ITEM_UNSET; c = &canPosMap[c->next])

#ifdef CAN_EXT
#define IDMAPSIZE 8
#else
#define IDMAPSIZE 4
#endif // CAN_EXT
#if (MAX_ITEMS * 12 + 2 * MAX_MESSAGES * IDMAPSIZE + 4) > FLASH_PAGE_SIZE
#error CANMAP will not fit in one flash page
#endif

volatile bool CanMap::isSaving = false;
volatile char printBuffer[7];
volatile uint8_t printByte = 0;

CanMap::CanMap(CanHardware* hw)
 : canHardware(hw), nodeId(1), printRequest(-1), printComplete(true)
{
   canHardware->AddReceiveCallback(this);

   ClearMap(canSendMap);
   ClearMap(canRecvMap);
   LoadFromFlash();
   HandleClear();
}

//Somebody (perhaps us) has cleared all user messages. Register them again
void CanMap::HandleClear()
{
   canHardware->RegisterUserMessage(0x600 + nodeId);

   forEachCanMap(curMap, canRecvMap)
   {
      canHardware->RegisterUserMessage(curMap->canId);
   }
}

bool CanMap::HandleRx(uint32_t canId, uint32_t data[2])
{
   if (canId == (0x600U + nodeId)) //SDO request
   {
      ProcessSDO(data);
      return true;
   }
   else
   {
      if (isSaving) return false; //Only handle mapped messages when not currently saving to flash

      CANIDMAP *recvMap = FindById(canRecvMap, canId);

      if (0 != recvMap)
      {
         forEachPosMap(curPos, recvMap)
         {
            float val;

            if (curPos->offsetBits > 31)
            {
               val = (data[1] >> (curPos->offsetBits - 32)) & ((1 << curPos->numBits) - 1);
            }
            else
            {
               val = (data[0] >> curPos->offsetBits) & ((1 << curPos->numBits) - 1);
            }
            val += curPos->offset;
            val *= curPos->gain;

            if (Param::IsParam((Param::PARAM_NUM)curPos->mapParam))
               Param::Set((Param::PARAM_NUM)curPos->mapParam, FP_FROMFLT(val));
            else
               Param::SetFloat((Param::PARAM_NUM)curPos->mapParam, val);
         }
         return true;
      }
   }
   return false;
}

/** \brief Clear all defined messages
 */
void CanMap::Clear()
{
   ClearMap(canSendMap);
   ClearMap(canRecvMap);
   canHardware->ClearUserMessages();
}

/** \brief Send all defined messages
 */
void CanMap::SendAll()
{
   forEachCanMap(curMap, canSendMap)
   {
      uint32_t data[2] = { 0 }; //Had an issue with uint64_t, otherwise would have used that

      forEachPosMap(curPos, curMap)
      {
         if (isSaving) return; //Only send mapped messages when not currently saving to flash

         float val = Param::GetFloat((Param::PARAM_NUM)curPos->mapParam);

         val *= curPos->gain;
         val += curPos->offset;
         int ival = val;
         ival &= ((1 << curPos->numBits) - 1);

         if (curPos->offsetBits > 31)
         {
            data[1] |= ival << (curPos->offsetBits - 32);
         }
         else
         {
            data[0] |= ival << curPos->offsetBits;
         }
      }

      canHardware->Send(curMap->canId, data);
   }
}

void CanMap::SDOWrite(uint8_t remoteNodeId, uint16_t index, uint8_t subIndex, uint32_t data)
{
   uint32_t d[2];
   CAN_SDO *sdo = (CAN_SDO*)d;

   sdo->cmd = SDO_WRITE;
   sdo->index = index;
   sdo->subIndex = subIndex;
   sdo->data = data;

   canHardware->Send(0x600 + remoteNodeId, d);
}

/** \brief Add periodic CAN message
 *
 * \param param Parameter index of parameter to be sent
 * \param canId CAN identifier of generated message
 * \param offset bit offset within the 64 message bits
 * \param length number of bits
 * \param gain Fixed point gain to be multiplied before sending
 * \return success: number of active messages
 * Fault:
 * - CAN_ERR_INVALID_ID ID was > 0x1fffffff
 * - CAN_ERR_INVALID_OFS Offset > 63
 * - CAN_ERR_INVALID_LEN Length > 32
 * - CAN_ERR_MAXMESSAGES Already 10 send messages defined
 * - CAN_ERR_MAXITEMS Already than MAX_ITEMS items total defined
 */
int CanMap::AddSend(Param::PARAM_NUM param, uint32_t canId, uint8_t offsetBits, uint8_t length, float gain, int8_t offset)
{
   return Add(canSendMap, param, canId, offsetBits, length, gain, offset);
}

int CanMap::AddSend(Param::PARAM_NUM param, uint32_t canId, uint8_t offsetBits, uint8_t length, float gain)
{
   return Add(canSendMap, param, canId, offsetBits, length, gain, 0);
}

/** \brief Map data from CAN bus to parameter
 *
 * \param param Parameter index of parameter to be received
 * \param canId CAN identifier of consumed message
 * \param offset bit offset within the 64 message bits
 * \param length number of bits
 * \param gain Fixed point gain to be multiplied after receiving
 * \return success: number of active messages
 * Fault:
 * - CAN_ERR_INVALID_ID ID was > 0x1fffffff
 * - CAN_ERR_INVALID_OFS Offset > 63
 * - CAN_ERR_INVALID_LEN Length > 32
 * - CAN_ERR_MAXMESSAGES Already 10 receive messages defined
 * - CAN_ERR_MAXITEMS Already than MAX_ITEMS items total defined
 */
int CanMap::AddRecv(Param::PARAM_NUM param, uint32_t canId, uint8_t offsetBits, uint8_t length, float gain, int8_t offset)
{
   int res = Add(canRecvMap, param, canId, offsetBits, length, gain, offset);
   canHardware->RegisterUserMessage(canId);
   return res;
}

int CanMap::AddRecv(Param::PARAM_NUM param, uint32_t canId, uint8_t offsetBits, uint8_t length, float gain)
{
   return AddRecv(param, canId, offsetBits, length, gain, 0);
}

/** \brief Remove all occurences of given parameter from CAN map
 *
 * \param param Parameter index to be removed
 * \return int number of removed items
 *
 */
int CanMap::Remove(Param::PARAM_NUM param)
{
   int removed = RemoveFromMap(canSendMap, param);
   removed += RemoveFromMap(canRecvMap, param);

   return removed;
}

/** \brief Save CAN mapping to flash
 */
void CanMap::Save()
{
   uint32_t crc;
   uint32_t check = 0xFFFFFFFF;
   uint32_t baseAddress = GetFlashAddress();
   uint32_t *checkAddress = (uint32_t*)baseAddress;

   isSaving = true;

   for (int i = 0; i < FLASH_PAGE_SIZE / 4; i++, checkAddress++)
      check &= *checkAddress;

   crc_reset();

   flash_unlock();
   flash_set_ws(2);

   if (check != 0xFFFFFFFF) //Only erase when needed
      flash_erase_page(baseAddress);

   ReplaceParamEnumByUid(canSendMap);
   ReplaceParamEnumByUid(canRecvMap);

   SaveToFlash(SENDMAP_ADDRESS(baseAddress), (uint32_t *)canSendMap, SENDMAP_WORDS);
   crc = SaveToFlash(RECVMAP_ADDRESS(baseAddress), (uint32_t *)canRecvMap, RECVMAP_WORDS);
   crc = SaveToFlash(POSMAP_ADDRESS(baseAddress), (uint32_t *)canPosMap, POSMAP_WORDS);
   SaveToFlash(CRC_ADDRESS(baseAddress), &crc, 1);
   flash_lock();

   ReplaceParamUidByEnum(canSendMap);
   ReplaceParamUidByEnum(canRecvMap);

   isSaving = false;
}


/** \brief Find first occurence of parameter in CAN map and output its mapping info
 *
 * Memory layout: SendMap, RecvMap, PosMap, CRC
 * Send/Recv maps point to items in PosMap. PosMap may point to next item
 *
 * \param[in] param Index of parameter to be looked up
 * \param[out] canId CAN identifier that the parameter is mapped to
 * \param[out] offset bit offset that the parameter is mapped to
 * \param[out] length number of bits that the parameter is mapped to
 * \param[out] gain Parameter gain
 * \param[out] rx true: Parameter is received via CAN, false: sent via CAN
 * \return true: parameter is mapped, false: not mapped
 */
bool CanMap::FindMap(Param::PARAM_NUM param, uint32_t& canId, uint8_t& start, uint8_t& length, float& gain, int8_t& offset, bool& rx)
{
   rx = false;
   bool done = false;

   for (CANIDMAP *map = canSendMap; !done; map = canRecvMap)
   {
      forEachCanMap(curMap, map)
      {
         forEachPosMap(curPos, curMap)
         {
            if (curPos->mapParam == param)
            {
               canId = curMap->canId;
               start = curPos->offsetBits;
               length = curPos->numBits;
               gain = curPos->gain;
               offset = curPos->offset;
               return true;
            }
         }
      }
      done = rx;
      rx = true;
   }
   return false;
}

void CanMap::IterateCanMap(void (*callback)(Param::PARAM_NUM, uint32_t, uint8_t, uint8_t, float, int8_t, bool))
{
   bool done = false, rx = false;

   for (CANIDMAP *map = canSendMap; !done; map = canRecvMap)
   {
      forEachCanMap(curMap, map)
      {
         forEachPosMap(curPos, curMap)
         {
            callback((Param::PARAM_NUM)curPos->mapParam, curMap->canId, curPos->offsetBits, curPos->numBits, curPos->gain, curPos->offset, rx);
         }
      }
      done = rx;
      rx = true;
   }
}

/****************** Private methods and ISRs ********************/

//http://www.byteme.org.uk/canopenparent/canopen/sdo-service-data-objects-canopen/
void CanMap::ProcessSDO(uint32_t data[2])
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
   else if (sdo->index >= SDO_INDEX_MAP_START && sdo->index < SDO_INDEX_MAP_END && sdo->subIndex < Param::PARAM_LAST)
   {
      if (sdo->cmd == SDO_WRITE)
      {
         int result;
         int offset = sdo->data & 0xFF;
         int len = (sdo->data >> 8) & 0xFF;
         s32fp gain = sdo->data >> 16;

         if ((sdo->index & SDO_INDEX_MAP_RX) == SDO_INDEX_MAP_RX)
         {
            result = AddRecv((Param::PARAM_NUM)sdo->subIndex, sdo->index & 0x7FF, offset, len, gain);
         }
         else
         {
            result = AddSend((Param::PARAM_NUM)sdo->subIndex, sdo->index & 0x7FF, offset, len, gain);
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

void CanMap::PutChar(char c)
{
   //When print buffer is full, wait
   while (printByte >= sizeof(printBuffer));
   printBuffer[printByte++] = c;
   printRequest = -1; //We can clear the print start trigger as we've obviously started printing
}

void CanMap::ProcessSpecialSDOObjects(CAN_SDO* sdo)
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
      default:
         sdo->cmd = SDO_ABORT;
         sdo->data = SDO_ERR_INVIDX;
      }
   }
   else if (sdo->index == SDO_INDEX_PJSON)
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
         Save();
         parm_save();
         break;
      case SDO_CMD_LOAD:
         parm_load();
         Param::Change(Param::PARAM_LAST);
         break;
      case SDO_CMD_RESET:
         scb_reset_system();
         break;
      default:
         sdo->cmd = SDO_ABORT;
         sdo->data = SDO_ERR_INVIDX;
      }
   }
}

void CanMap::ClearMap(CANIDMAP *canMap)
{
   for (int i = 0; i < MAX_MESSAGES; i++)
   {
      canMap[i].first = MAX_ITEMS;
   }

   //Initialize also tail to ITEM_UNSET
   for (int i = 0; i < (MAX_ITEMS + 1); i++)
   {
      canPosMap[i].next = ITEM_UNSET;
   }
}

int CanMap::RemoveFromMap(CANIDMAP *canMap, Param::PARAM_NUM param)
{
   int removed = 0;
   CANPOS *lastPosMap;

   forEachCanMap(curMap, canMap)
   {
      lastPosMap = 0;

      forEachPosMap(curPos, curMap)
      {
         if (curPos->mapParam == param)
         {
            if (lastPosMap != 0)
            {
               lastPosMap->next = curPos->next;
            }
            else
            {
               curMap->first = curPos->next;
            }
         }
         lastPosMap = curPos;
      }
   }

   return removed;
}

int CanMap::Add(CANIDMAP *canMap, Param::PARAM_NUM param, uint32_t canId, uint8_t offsetBits, uint8_t length, float gain, int8_t offset)
{
   if (canId > 0x1fffffff) return CAN_ERR_INVALID_ID;
   if (offsetBits > 63) return CAN_ERR_INVALID_OFS;
   if (length > 32) return CAN_ERR_INVALID_LEN;

   CANIDMAP *existingMap = FindById(canMap, canId);

   if (0 == existingMap)
   {
      for (int i = 0; i < MAX_MESSAGES; i++)
      {
         if (canMap[i].first == MAX_ITEMS)
         {
            existingMap = &canMap[i];
            break;
         }
      }

      if (0 == existingMap)
         return CAN_ERR_MAXMESSAGES;

      existingMap->canId = canId;
   }

   int freeIndex;

   for (freeIndex = 0; freeIndex < MAX_ITEMS && canPosMap[freeIndex].next != ITEM_UNSET; freeIndex++);

   if (freeIndex == MAX_ITEMS)
      return CAN_ERR_MAXITEMS;

   CANPOS* precedingItem = 0;

   for (int precedingIndex = existingMap->first; precedingIndex != MAX_ITEMS; precedingIndex = canPosMap[precedingIndex].next)
      precedingItem = &canPosMap[precedingIndex];

   CANPOS* freeItem = &canPosMap[freeIndex];
   freeItem->mapParam = param;
   freeItem->gain = gain;
   freeItem->offset = offset;
   freeItem->offsetBits = offsetBits;
   freeItem->numBits = length;
   freeItem->next = MAX_ITEMS;

   if (precedingItem == 0) //first item for this can ID
   {
      existingMap->first = freeIndex;
   }
   else
   {
      precedingItem->next = freeIndex;
   }

   int count = 0;

   forEachCanMap(curMap, canMap)
      count++;

   return count;
}

uint32_t CanMap::SaveToFlash(uint32_t baseAddress, uint32_t* data, int len)
{
   uint32_t crc = 0;

   for (int idx = 0; idx < len; idx++)
   {
      crc = crc_calculate(*data);
      flash_program_word(baseAddress + idx * sizeof(uint32_t), *data);
      data++;
   }

   return crc;
}

int CanMap::LoadFromFlash()
{
   uint32_t baseAddress = GetFlashAddress();
   uint32_t storedCrc = *(uint32_t*)CRC_ADDRESS(baseAddress);
   uint32_t crc;

   crc_reset();
   crc = crc_calculate_block((uint32_t*)baseAddress, SENDMAP_WORDS + RECVMAP_WORDS + POSMAP_WORDS);

   if (storedCrc == crc)
   {
      memcpy32((int*)canSendMap, (int*)SENDMAP_ADDRESS(baseAddress), SENDMAP_WORDS);
      memcpy32((int*)canRecvMap, (int*)RECVMAP_ADDRESS(baseAddress), RECVMAP_WORDS);
      memcpy32((int*)canPosMap, (int*)POSMAP_ADDRESS(baseAddress), POSMAP_WORDS);
      ReplaceParamUidByEnum(canSendMap);
      ReplaceParamUidByEnum(canRecvMap);
      return 1;
   }
   return 0;
}

CanMap::CANIDMAP* CanMap::FindById(CANIDMAP *canMap, uint32_t canId)
{
   forEachCanMap(curMap, canMap)
   {
      if (curMap->canId == canId)
         return curMap;
   }
   return 0;
}

uint32_t CanMap::GetFlashAddress()
{
   uint32_t flashSize = desig_get_flash_size();

   return FLASH_BASE + flashSize * 1024 - FLASH_PAGE_SIZE * CAN1_BLKNUM;
}

void CanMap::ReplaceParamEnumByUid(CANIDMAP *canMap)
{
   forEachCanMap(curMap, canMap)
   {
      forEachPosMap(curPos, curMap)
      {
         const Param::Attributes* attr = Param::GetAttrib((Param::PARAM_NUM)curPos->mapParam);
         curPos->mapParam = (uint16_t)attr->id;
      }
   }
}

void CanMap::ReplaceParamUidByEnum(CANIDMAP *canMap)
{
   forEachCanMap(curMap, canMap)
   {
      forEachPosMap(curPos, curMap)
      {
         Param::PARAM_NUM param = Param::NumFromId(curPos->mapParam);
         curPos->mapParam = param;
      }
   }
}
