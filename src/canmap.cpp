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
#include "canmap.h"
#include "hwdefs.h"
#include "my_string.h"

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

CanMap::CanMap(CanHardware* hw)
 : canHardware(hw)
{
   canHardware->AddCallback(this);

   ClearMap(canSendMap);
   ClearMap(canRecvMap);
   LoadFromFlash();
   HandleClear();
}

//Somebody (perhaps us) has cleared all user messages. Register them again
void CanMap::HandleClear()
{
   forEachCanMap(curMap, canRecvMap)
   {
      canHardware->RegisterUserMessage(curMap->canId);
   }
}

bool CanMap::HandleRx(uint32_t canId, uint32_t data[2], uint8_t)
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

         if (Param::GetType((Param::PARAM_NUM)curPos->mapParam) == Param::TYPE_PARAM || Param::GetType((Param::PARAM_NUM)curPos->mapParam) == Param::TYPE_TESTPARAM)
            Param::Set((Param::PARAM_NUM)curPos->mapParam, FP_FROMFLT(val));
         else
            Param::SetFloat((Param::PARAM_NUM)curPos->mapParam, val);
      }
      return true;
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

/** \brief Remove first occurrence of given parameter from CAN map
 *
 * \param param Parameter index to be removed
 * \return int number of removed items
 *
 */
int CanMap::Remove(Param::PARAM_NUM param)
{
   bool rx = false;
   bool done = false;
   uint8_t messageIdx = 0, itemIdx = 0;

   for (CANIDMAP *map = canSendMap; !done; map = canRecvMap)
   {
      messageIdx = 0;
      forEachCanMap(curMap, map)
      {
         itemIdx = 0;
         forEachPosMap(curPos, curMap)
         {
            if (curPos->mapParam == param)
               goto itemfound; //ugly but the only way without extra function

            itemIdx++;
         }
         messageIdx++;
      }
      done = rx; //done iterating RX map
      rx = true; //done iterating TX map, now we iterate RX map
   }

itemfound:

   if (!done) //loop didn't run to end
      return Remove(rx, messageIdx, itemIdx);

   return 0;
}

int CanMap::Remove(bool rx, uint8_t messageIdx, uint8_t itemidx)
{
   CANPOS *lastPosMap = 0;
   CANIDMAP *map = rx ? &canRecvMap[messageIdx] : &canSendMap[messageIdx];

   if (messageIdx > MAX_MESSAGES || map->first == MAX_ITEMS) return 0;

   forEachPosMap(curPos, map)
   {
      if (itemidx == 0)
      {
         if (lastPosMap != 0)
         {
            lastPosMap->next = curPos->next;
         }
         else if (curPos->next != MAX_ITEMS)
         {
            //We deleted the first item of the message -> move second item to first
            map->first = curPos->next;
         }
         else
         {
            //If curPos was the last mapped item, we have to fill in the blank
            //by moving the last mapped item here.
            uint8_t lastIdx = 0;
            //find last item
            for (; (lastIdx + messageIdx) < MAX_MESSAGES && map[lastIdx].first != MAX_ITEMS; lastIdx++);
            //lastidx is now the first unused item, go back one for the last used one
            lastIdx--;

            //move last message to our deleted message
            //we might move the message to itself but that's ok
            map->first = map[lastIdx].first;
            map->canId = map[lastIdx].canId;
            //mark last message ununsed
            map[lastIdx].first = MAX_ITEMS;
         }
         return 1;
      }
      itemidx--;
      lastPosMap = curPos;
   }
   return 0;
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

const CanMap::CANPOS* CanMap::GetMap(bool rx, uint8_t ididx, uint8_t itemidx, uint32_t& canId)
{
   CANIDMAP *map = rx ? &canRecvMap[ididx] : &canSendMap[ididx];

   if (ididx > MAX_MESSAGES || map->first == MAX_ITEMS) return 0;

   forEachPosMap(curPos, map)
   {
      if (itemidx == 0)
      {
         canId = map->canId;
         return curPos;
      }
      itemidx--;
   }
   return 0;
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

int CanMap::Add(CANIDMAP *canMap, Param::PARAM_NUM param, uint32_t canId, uint8_t offsetBits, uint8_t length, float gain, int8_t offset)
{
   if (canId > MAX_COB_ID) return CAN_ERR_INVALID_ID;
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


/** \brief Loads message definitions from flash
 *
 * \return 1 for success, 0 for CRC error
 *
 */
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
   else
   {
      return LegacyLoadFromFlash();
   }
}

/** \brief Loads the old-style message definitions from flash
 * \return 1 for success, 0 for CRC error
 */
int CanMap::LegacyLoadFromFlash()
{
   const int MAX_ITEMS_PER_MESSAGE = 8;
   const int LEGACY_MAX_MESSAGES   = 10;
   const int CANID_UNSET           = 0xffff;

   struct LEGACY_CANPOS
   {
      uint16_t mapParam;
      s16fp gain;
      uint8_t offsetBits;
      int8_t numBits;
   };

   struct LEGACY_CANIDMAP
   {
      uint16_t canId;
      LEGACY_CANPOS items[MAX_ITEMS_PER_MESSAGE];
   };

   auto convert = [this](LEGACY_CANIDMAP* m, CANIDMAP* newIdMap, bool gainToFloat)
   {
      for (LEGACY_CANIDMAP *c = m; (c - m) < LEGACY_MAX_MESSAGES && c->canId < CANID_UNSET; c++)
      {
         for (LEGACY_CANPOS *cp = c->items; (cp - c->items) < MAX_ITEMS_PER_MESSAGE && cp->numBits > 0; cp++)
         {
            Param::PARAM_NUM param = Param::NumFromId(cp->mapParam);
            Add(newIdMap, param, c->canId, cp->offsetBits, cp->numBits, gainToFloat ? FP_TOFLOAT(cp->gain) : cp->gain, 0);
         }
      }
   };

   uint32_t data = GetFlashAddress();
   const int size = sizeof(LEGACY_CANIDMAP) * LEGACY_MAX_MESSAGES * 2;
   uint32_t storedCrc = *(uint32_t*)(data + size);

   crc_reset();
   uint32_t crc = crc_calculate_block((uint32_t*)data, size / 4);

   if (storedCrc == crc)
   {
      convert((LEGACY_CANIDMAP*)data, canSendMap, false);
      convert((LEGACY_CANIDMAP*)(data + sizeof(LEGACY_CANIDMAP) * LEGACY_MAX_MESSAGES), canRecvMap, true);

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
