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

class CanSdo: CanCallback, public IPutChar
{
   public:
      struct SdoFrame
      {
         uint8_t cmd;
         uint16_t index;
         uint8_t subIndex;
         uint32_t data;
      } __attribute__((packed));

      /** Default constructor */
      CanSdo(CanHardware* hw, CanMap* cm = 0);
      CanHardware* GetHardware() { return canHardware; }
      void HandleClear();
      void HandleRx(uint32_t canId, uint32_t data[2], uint8_t dlc) override;
      void SDOWrite(uint8_t nodeId, uint16_t index, uint8_t subIndex, uint32_t data);
      void SDORead(uint8_t nodeId, uint16_t index, uint8_t subIndex);
      bool SDOReadReply(uint32_t& data);
      void RemoteMap(uint8_t nodeId, bool rx, uint32_t cobId, CanMap::CANPOS mapping);
      void SetNodeId(uint8_t id);
      int GetPrintRequest() { return printRequest; }
      SdoFrame* GetPendingUserspaceSdo() { return pendingUserSpaceSdo ? &pendingUserSpaceSdoFrame : 0; }
      void SendSdoReply(SdoFrame* sdoFrame);
      void PutChar(char c);
      void TriggerTimeout(int callingFrequency);

   private:
      CanHardware* canHardware;
      CanMap* canMap;
      uint8_t nodeId;
      uint8_t remoteNodeId;
      int printRequest;
      //We use a ring buffer with non-wrapping index. This limits us to 4 GB, huh!
      //In the beginning printBufIn starts at 0 and printBufOut at sizeof(printBuffer) (e.g. 64)
      //All addressing of printBuffer is modulo buffer size
      volatile char printBuffer[64]; //Must be a power of 2 for efficient modulo calculation
      volatile uint32_t printByteIn;
      volatile uint32_t printByteOut;
      volatile int printTimeout; //remaining time to wait
      Param::PARAM_NUM mapParam;
      uint32_t mapId;
      CanMap::CANPOS mapInfo;
      bool sdoReplyValid;
      uint32_t sdoReplyData;
      SdoFrame pendingUserSpaceSdoFrame;
      bool pendingUserSpaceSdo;

      void ProcessSDO(uint32_t data[2]);
      bool ProcessSpecialSDOObjects(SdoFrame *sdo);
      void ReadOrDeleteCanMap(SdoFrame *sdo);
      void AddCanMap(SdoFrame *sdo, bool rx);
      void InitiateSDOTransfer(uint8_t req, uint8_t nodeId, uint16_t index, uint8_t subIndex, uint32_t data);
};

#endif // CANSDO_H
