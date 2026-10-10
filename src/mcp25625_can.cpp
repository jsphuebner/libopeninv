/*
 * This file is part of the libopeninv project.
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
#include <stdint.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/cm3/common.h>
#include "mcp25625_can.h"
#include "cortex.h"

//Some functions use the "register" keyword which C++ doesn't like
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wregister"
#include <libopencm3/cm3/cortex.h>
#pragma GCC diagnostic pop

#ifndef MCP25625_OSC_HZ
#define MCP25625_OSC_HZ 16000000
#endif

// See stm32_can.cpp for the rationale of these macros
#ifdef CAN_MAX_IRQ_PRIORITY
#define DISABLE_CAN_USER_INTERRUPTS()  cm_set_basepriority(CAN_MAX_IRQ_PRIORITY);
#define ENABLE_CAN_USER_INTERRUPTS()   cm_set_basepriority(CM_BASEPRI_ENABLE_INTERRUPTS);
#else
#define DISABLE_CAN_USER_INTERRUPTS()  cm_disable_interrupts()
#define ENABLE_CAN_USER_INTERRUPTS()   cm_enable_interrupts()
#endif // CAN_MAX_IRQ_PRIORITY

// SPI instructions
#define CMD_RESET       0xC0
#define CMD_READ        0x03
#define CMD_WRITE       0x02
#define CMD_BITMODIFY   0x05
#define CMD_LOAD_TX     0x40
#define CMD_RTS         0x80
#define CMD_READ_RX     0x90
#define CMD_READ_STATUS 0xA0

// Registers
#define REG_CANSTAT     0x0E
#define REG_CANCTRL     0x0F
#define REG_CNF3        0x28
#define REG_CNF2        0x29
#define REG_CNF1        0x2A
#define REG_CANINTE     0x2B
#define REG_CANINTF     0x2C
#define REG_RXB0CTRL    0x60
#define REG_RXB1CTRL    0x70

#define MODE_NORMAL     0x00
#define MODE_LISTEN     0x60
#define MODE_CONFIG     0x80
#define MODE_MASK       0xE0

#define STATUS_RX0IF    0x01
#define STATUS_RX1IF    0x02
static const uint8_t txReqMask[3] = { 0x04, 0x10, 0x40 };

static const uint32_t baudrateValues[CanHardware::BaudLast] =
   { 125000, 250000, 500000, 800000, 1000000, 33333, 100000 };

Mcp25625Can::Mcp25625Can(uint32_t spi, DigIo& cs, enum baudrates baudrate)
   : spiDev(spi), csPin(cs), runMode(MODE_NORMAL)
{
   csPin.Set();
   Reset();

   //receive any message, roll over RXB0 to RXB1, filtering is done in software
   WriteReg(REG_RXB0CTRL, 0x64);
   WriteReg(REG_RXB1CTRL, 0x60);
   //interrupts: RX0, RX1, TX0, TX1, TX2
   WriteReg(REG_CANINTE, 0x1F);

   SetBaudrate(baudrate);
}

/** \brief Set baud rate to given value, depends on MCP25625_OSC_HZ
 */
void Mcp25625Can::SetBaudrate(enum baudrates baudrate)
{
   uint32_t baud = baudrateValues[baudrate];
   uint32_t brp = 1, tq = 16;

   //find a time quanta count per bit between 16 and 8 that fits best
   uint32_t bestErr = 0xFFFFFFFF;
   for (uint32_t n = 16; n >= 8; n--)
   {
      uint32_t div = 2 * n * baud;
      uint32_t b = (MCP25625_OSC_HZ + div / 2) / div;
      if (b < 1 || b > 64) continue;
      uint32_t actual = MCP25625_OSC_HZ / (2 * n * b);
      uint32_t err = actual > baud ? actual - baud : baud - actual;
      if (err < bestErr)
      {
         bestErr = err;
         brp = b;
         tq = n;
      }
   }

   //sample point at about 87.5%
   uint32_t ps2 = tq / 8 < 2 ? 2 : tq / 8;
   uint32_t seg = tq - 1 - ps2;
   uint32_t ps1 = seg / 2;
   uint32_t prop = seg - ps1;

   SetMode(MODE_CONFIG);
   WriteReg(REG_CNF1, brp - 1);
   WriteReg(REG_CNF2, 0x80 | ((ps1 - 1) << 3) | (prop - 1));
   WriteReg(REG_CNF3, ps2 - 1);
   SetMode(runMode);
}

void Mcp25625Can::SetListenOnly(bool enable)
{
   runMode = enable ? MODE_LISTEN : MODE_NORMAL;
   SetMode(runMode);
}

/** \brief Send a user defined CAN message
 *
 * \param canId uint32_t
 * \param data[2] uint32_t
 * \param len message length
 * \return void
 *
 */
void Mcp25625Can::Send(uint32_t canId, uint32_t data[2], uint8_t len, bool forceExt)
{
   DISABLE_CAN_USER_INTERRUPTS();

   SendOrQueue(canId, data, len, forceExt);

   ENABLE_CAN_USER_INTERRUPTS();
}

void Mcp25625Can::HandleInterrupt()
{
   //TX flags (bits 2..4) and error flags are only cleared, RX flags are cleared by reading the buffer
   BitModify(REG_CANINTF, 0xFC, 0);
   HandleMessage();
   HandleTx();
}

void Mcp25625Can::HandleMessage()
{
   uint8_t status;

   while (((status = ReadStatus()) & (STATUS_RX0IF | STATUS_RX1IF)) != 0)
   {
      if (status & STATUS_RX0IF)
         ReceiveBuffer(0);
      if (status & STATUS_RX1IF)
         ReceiveBuffer(1);
   }
}

void Mcp25625Can::HandleTx()
{
   DISABLE_CAN_USER_INTERRUPTS();

   FlushSendBuffer();

   ENABLE_CAN_USER_INTERRUPTS();
}

/****************** Private methods ********************/

uint8_t Mcp25625Can::Xfer(uint8_t b)
{
   return spi_xfer(spiDev, b);
}

void Mcp25625Can::Reset()
{
   csPin.Clear();
   Xfer(CMD_RESET);
   csPin.Set();
   //oscillator start-up time after reset
   for (volatile int i = 0; i < 100000; i++);
}

uint8_t Mcp25625Can::ReadReg(uint8_t addr)
{
   csPin.Clear();
   Xfer(CMD_READ);
   Xfer(addr);
   uint8_t val = Xfer(0);
   csPin.Set();
   return val;
}

void Mcp25625Can::WriteReg(uint8_t addr, uint8_t val)
{
   csPin.Clear();
   Xfer(CMD_WRITE);
   Xfer(addr);
   Xfer(val);
   csPin.Set();
}

void Mcp25625Can::BitModify(uint8_t addr, uint8_t mask, uint8_t val)
{
   csPin.Clear();
   Xfer(CMD_BITMODIFY);
   Xfer(addr);
   Xfer(mask);
   Xfer(val);
   csPin.Set();
}

uint8_t Mcp25625Can::ReadStatus()
{
   csPin.Clear();
   Xfer(CMD_READ_STATUS);
   uint8_t val = Xfer(0);
   csPin.Set();
   return val;
}

void Mcp25625Can::SetMode(uint8_t mode)
{
   BitModify(REG_CANCTRL, MODE_MASK, mode);

   for (int i = 0; i < 10000 && (ReadReg(REG_CANSTAT) & MODE_MASK) != mode; i++);
}

/** \return true if message was loaded into a free transmit buffer */
bool Mcp25625Can::Transmit(uint32_t id, uint32_t canData[2], uint8_t len, bool forceExt)
{
   const uint8_t* data = (const uint8_t*)canData;
   uint8_t status = ReadStatus();
   bool ext = (id > 0x7FF) | forceExt;
   uint8_t buf[5];

   if (len > 8) len = 8;

   for (int i = 0; i < 3; i++)
   {
      if (status & txReqMask[i])
         continue;

      if (ext)
      {
         id &= 0x1FFFFFFF;
         buf[0] = id >> 21;
         buf[1] = (((id >> 18) & 7) << 5) | 0x08 | ((id >> 16) & 3);
         buf[2] = id >> 8;
         buf[3] = id;
      }
      else
      {
         buf[0] = id >> 3;
         buf[1] = (id & 7) << 5;
         buf[2] = 0;
         buf[3] = 0;
      }
      buf[4] = len;

      csPin.Clear();
      Xfer(CMD_LOAD_TX | (i << 1)); //load starting at TXBnSIDH
      for (int j = 0; j < 5; j++) Xfer(buf[j]);
      for (int j = 0; j < len; j++) Xfer(data[j]);
      csPin.Set();

      csPin.Clear();
      Xfer(CMD_RTS | (1 << i));
      csPin.Set();
      return true;
   }
   return false;
}

bool Mcp25625Can::IsUserMessage(uint32_t id, bool ext)
{
   for (int i = 0; i < nextUserMessageIndex; i++)
   {
      bool userExt = userIds[i] > 0x7FF;
      uint32_t userId = userIds[i] & 0x1FFFFFFF;
      uint32_t mask = userMasks[i] != 0 ? userMasks[i] : 0x1FFFFFFF;

      if (userExt == ext && (id & mask) == (userId & mask))
         return true;
   }
   return false;
}

/** \brief Read one receive buffer, this clears its interrupt flag */
bool Mcp25625Can::ReceiveBuffer(int index)
{
   uint8_t hdr[5];
   uint32_t data[2] = { 0, 0 };
   uint8_t* d = (uint8_t*)data;

   csPin.Clear();
   Xfer(CMD_READ_RX | (index << 2));
   for (int i = 0; i < 5; i++) hdr[i] = Xfer(0);
   uint8_t len = hdr[4] & 0xF;
   if (len > 8) len = 8;
   for (int i = 0; i < len; i++) d[i] = Xfer(0);
   csPin.Set();

   bool ext = (hdr[1] & 0x08) != 0;
   uint32_t id;

   if (ext)
      id = ((uint32_t)hdr[0] << 21) | ((uint32_t)(hdr[1] >> 5) << 18) | ((uint32_t)(hdr[1] & 3) << 16) | ((uint32_t)hdr[2] << 8) | hdr[3];
   else
      id = ((uint32_t)hdr[0] << 3) | (hdr[1] >> 5);

   if (IsUserMessage(id, ext))
   {
      HandleRx(id, data, len);
      lastRxTimestamp = rtc_get_counter_val();
   }
   return true;
}

void Mcp25625Can::ConfigureFilters()
{
   //Filtering is done in software against userIds and userMasks, nothing to do in hardware
}
