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
#ifndef CANOBD2_H
#define CANOBD2_H
#include "params.h"
#include "canhardware.h"

class CanObd2: CanCallback
{
   public:
      /** Default constructor */
      CanObd2(CanHardware* hw);
      void HandleClear();
      bool HandleRx(uint32_t canId, uint32_t data[2], uint8_t dlc);
      void SetNodeId(uint8_t id);

   private:
      CanHardware* canHardware;
      uint8_t nodeId;

      void ProcessOBD2(uint32_t data[2]);
};

#endif // CANOBD2_H
