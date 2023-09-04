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
#ifndef CANSDO_H
#define CANSDO_H
#include "params.h"
#include "printf.h"
#include "canhardware.h"
#include "canmap.h"

class CanSdo: CanCallback, public IPutChar
{
   public:
      /** Default constructor */
      CanSdo(CanHardware* hw, CanMap* cm = 0);
      void HandleClear();
      bool HandleRx(uint32_t canId, uint32_t data[2]);
      void SDOWrite(uint8_t nodeId, uint16_t index, uint8_t subIndex, uint32_t data);
      void SDORead(uint8_t nodeId, uint16_t index, uint8_t subIndex);
      bool SDOReadReply(uint32_t& data);
      void SetNodeId(uint8_t id);
      int GetPrintRequest() { return printRequest; }
      void SignalPrintComplete() { printComplete = true; }
      void PutChar(char c);

   private:
      struct CAN_SDO
      {
         uint8_t cmd;
         uint16_t index;
         uint8_t subIndex;
         uint32_t data;
      } __attribute__((packed));

      CanHardware* canHardware;
      CanMap* canMap;
      uint8_t nodeId;
      uint8_t remoteNodeId;
      int printRequest;
      bool printComplete;
      volatile char printBuffer[7];
      volatile uint8_t printByte = 0;
      Param::PARAM_NUM mapParam;
      uint32_t mapId;
      CanMap::CANPOS mapInfo;
      bool sdoReplyValid;
      uint32_t sdoReplyData;

      void ProcessSDO(uint32_t data[2]);
      void ProcessSpecialSDOObjects(CAN_SDO *sdo);
      void ReadOrDeleteCanMap(CAN_SDO *sdo);
      void AddCanMap(CAN_SDO *sdo, bool rx);
      void InitiateSDOTransfer(uint8_t req, uint8_t nodeId, uint16_t index, uint8_t subIndex, uint32_t data);
};

#endif // CANSDO_H
