/*
 * This file is part of the libopeninv project.
 *
 * Copyright (C) 2016 Nail Güzel
 * Johannes Huebner <dev@johanneshuebner.com>
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
#ifndef STM32_CAN_H_INCLUDED
#define STM32_CAN_H_INCLUDED
#include "canhardware.h"

#ifndef SENDBUFFER_LEN
#define SENDBUFFER_LEN 20
#endif // SENDBUFFER_LEN

class Stm32Can: public CanHardware
{
public:
   Stm32Can(uint32_t baseAddr, enum baudrates baudrate, bool remap = false);
   void SetBaudrate(enum baudrates baudrate);
   void Send(uint32_t canId, uint32_t data[2], uint8_t len);
   void HandleTx();
   void HandleMessage(int fifo);
   static Stm32Can* GetInterface(int index);

private:
   struct SENDBUFFER
   {
      uint32_t id;
      uint32_t len;
      uint32_t data[2];
   };

   SENDBUFFER sendBuffer[SENDBUFFER_LEN];
   int sendCnt;
   uint32_t canDev;

   void ConfigureFilters();
   void SetFilterBank(int& idIndex, int& filterId, uint16_t* idList);
   void SetFilterBank29(int& idIndex, int& filterId, uint32_t* idList);
   uint32_t GetFlashAddress();

   static Stm32Can* interfaces[];
};


#endif
