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
#ifndef CANMAP_H
#define CANMAP_H
#include "params.h"
#include "printf.h"
#include "canhardware.h"

#define CAN_ERR_INVALID_ID -1
#define CAN_ERR_INVALID_OFS -2
#define CAN_ERR_INVALID_LEN -3
#define CAN_ERR_MAXMESSAGES -4
#define CAN_ERR_MAXITEMS -5

#ifndef MAX_ITEMS
#define MAX_ITEMS 50
#endif

#ifndef MAX_MESSAGES
#define MAX_MESSAGES 10
#endif

class CanMap: CanCallback, public IPutChar
{
   public:
      /** Default constructor */
      CanMap(CanHardware* hw);
      void HandleClear();
      bool HandleRx(uint32_t canId, uint32_t data[2]);
      void Clear();
      void SendAll();
      void SDOWrite(uint8_t remoteNodeId, uint16_t index, uint8_t subIndex, uint32_t data);
      void SetNodeId(uint8_t id);
      int AddSend(Param::PARAM_NUM param, uint32_t canId, uint8_t offsetBits, uint8_t length, float gain);
      int AddRecv(Param::PARAM_NUM param, uint32_t canId, uint8_t offsetBits, uint8_t length, float gain);
      int AddSend(Param::PARAM_NUM param, uint32_t canId, uint8_t offsetBits, uint8_t length, float gain, int8_t offset);
      int AddRecv(Param::PARAM_NUM param, uint32_t canId, uint8_t offsetBits, uint8_t length, float gain, int8_t offset);
      int Remove(Param::PARAM_NUM param);
      void Save();
      bool FindMap(Param::PARAM_NUM param, uint32_t& canId, uint8_t& start, uint8_t& length, float& gain, int8_t& offset, bool& rx);
      void IterateCanMap(void (*callback)(Param::PARAM_NUM, uint32_t, uint8_t, uint8_t, float, int8_t, bool));
      int GetPrintRequest() { return printRequest; }
      void SignalPrintComplete() { printComplete = true; }
      void PutChar(char c);

   protected:

   private:
      static volatile bool isSaving;

      struct CANPOS
      {
         float gain;
         uint16_t mapParam;
         int8_t offset;
         uint8_t offsetBits;
         uint8_t numBits;
         uint8_t next;
      };

      struct CANIDMAP
      {
         #ifdef CAN_EXT
         uint32_t canId;
         #else
         uint16_t canId;
         #endif // CAN_EXT
         uint8_t first;
      };

      struct CAN_SDO
      {
         uint8_t cmd;
         uint16_t index;
         uint8_t subIndex;
         uint32_t data;
      } __attribute__((packed));

      CanHardware* canHardware;
      CANIDMAP canSendMap[MAX_MESSAGES];
      CANIDMAP canRecvMap[MAX_MESSAGES];
      CANPOS canPosMap[MAX_ITEMS + 1]; //Last item is a "tail"
      uint32_t lastRxTimestamp;
      uint8_t nodeId;
      int printRequest;
      bool printComplete;

      void ProcessSDO(uint32_t data[2]);
      void ProcessSpecialSDOObjects(CAN_SDO *sdo);
      void ClearMap(CANIDMAP *canMap);
      int RemoveFromMap(CANIDMAP *canMap, Param::PARAM_NUM param);
      int Add(CANIDMAP *canMap, Param::PARAM_NUM param, uint32_t canId, uint8_t offsetBits, uint8_t length, float gain, int8_t offset);
      uint32_t SaveToFlash(uint32_t baseAddress, uint32_t* data, int len);
      int LoadFromFlash();
      CANIDMAP *FindById(CANIDMAP *canMap, uint32_t canId);
      int CopyIdMapExcept(CANIDMAP *source, CANIDMAP *dest, Param::PARAM_NUM param);
      void ReplaceParamEnumByUid(CANIDMAP *canMap);
      void ReplaceParamUidByEnum(CANIDMAP *canMap);
      uint32_t GetFlashAddress();
};

#endif // CANMAP_H
