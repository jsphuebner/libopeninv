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
#include "mcp251863_can.h"
#include "cortex.h"

//Some functions use the "register" keyword which C++ doesn't like
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wregister"
#include <libopencm3/cm3/cortex.h>
#pragma GCC diagnostic pop

#ifndef MCP251863_SYSCLK_HZ
#define MCP251863_SYSCLK_HZ 40000000
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
#define CMD_RESET       0x0
#define CMD_WRITE       0x2
#define CMD_READ        0x3

// Registers
#define REG_C1CON       0x000
#define REG_C1NBTCFG    0x004
#define REG_C1DBTCFG    0x008
#define REG_C1TDC       0x00C
#define REG_C1INT       0x01C
#define REG_C1TXQCON    0x050
#define REG_C1TXQSTA    0x054
#define REG_C1TXQUA     0x058
#define REG_C1FIFOCON1  0x05C
#define REG_C1FIFOSTA1  0x060
#define REG_C1FIFOUA1   0x064
#define REG_C1FLTCON    0x1D0
#define REG_C1FLTOBJ    0x1F0
#define REG_C1MASK      0x1F4
#define REG_OSC         0xE00

#define RAM_START       0x400
#define NUM_FILTERS     32

// Operation modes
#define MODE_FD         0
#define MODE_LISTEN     3
#define MODE_CONFIG     4
#define MODE_CAN20      6

#define OSC_OSCRDY      (1 << 10)

// Payload size of all FIFOs is 64 bytes: PLSIZE = 7
#define PLSIZE_64       7
#define TXQ_SIZE        8
#define RXFIFO_SIZE     16
#define ELEMENT_SIZE    (8 + 64)
#define TXQ_RAM         RAM_START
#define RXFIFO_RAM      (RAM_START + TXQ_SIZE * ELEMENT_SIZE)

static const uint32_t baudrateValues[CanHardware::BaudLast] =
   { 125000, 250000, 500000, 800000, 1000000, 33333, 100000 };

static const uint8_t dlcToLen[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };

static uint8_t LenToDlc(uint8_t len)
{
   for (uint8_t i = 0; i < 16; i++)
   {
      if (dlcToLen[i] >= len)
         return i;
   }
   return 15;
}

/** Convert CAN id to the format used in filter, mask and message object registers */
static uint32_t IdToReg(uint32_t id, bool ext)
{
   if (ext)
      return ((id >> 18) & 0x7FF) | ((id & 0x3FFFF) << 11);
   return id & 0x7FF;
}

/** Find bit timing for a bit rate.
 * \param maxTq maximum number of time quanta per bit
 * \return total time quanta per bit, brp is the prescaler */
static uint32_t FindBitTiming(uint32_t baud, uint32_t maxTq, uint32_t maxBrp, uint32_t& brp)
{
   uint32_t tq = 1;
   brp = 1;
   for (; brp <= maxBrp; brp++)
   {
      uint32_t div = baud * brp;
      tq = (MCP251863_SYSCLK_HZ + div / 2) / div;
      if (tq <= maxTq)
         break;
   }
   if (brp > maxBrp) brp = maxBrp;
   return tq < 5 ? 5 : tq;
}

Mcp251863Can::Mcp251863Can(uint32_t spi, DigIo& cs, enum baudrates baudrate)
   : spiDev(spi), csPin(cs), runMode(MODE_CAN20), fdEnabled(false),
     dataBaud(2000000), nominalBaud(baudrateValues[baudrate]), fdRxCallback(0)
{
   csPin.Set();
   Reset(); //chip is in configuration mode afterwards

   //Transmit queue, 8 messages, 64 byte payload
   WriteReg(REG_C1TXQCON, ((TXQ_SIZE - 1) << 24) | (PLSIZE_64 << 29));
   //Receive FIFO 1, 16 messages, 64 byte payload, RXEN
   WriteReg(REG_C1FIFOCON1, ((RXFIFO_SIZE - 1) << 24) | (PLSIZE_64 << 29));
   //enable transmit queue
   WriteReg(REG_C1CON, ReadReg(REG_C1CON) | (1 << 20));

   ConfigureFilters();
   SetBaudrate(baudrate);
}

/** \brief Set nominal baud rate to given value, depends on MCP251863_SYSCLK_HZ
 */
void Mcp251863Can::SetBaudrate(enum baudrates baudrate)
{
   nominalBaud = baudrateValues[baudrate];
   ConfigureTimings();
}

void Mcp251863Can::SetDataBaudrate(uint32_t dataBaudrate)
{
   dataBaud = dataBaudrate;
   ConfigureTimings();
}

void Mcp251863Can::EnableFd(bool enable, uint32_t dataBaudrate)
{
   fdEnabled = enable;
   dataBaud = dataBaudrate;
   runMode = enable ? MODE_FD : MODE_CAN20;
   ConfigureTimings();
}

void Mcp251863Can::SetListenOnly(bool enable)
{
   runMode = enable ? MODE_LISTEN : (fdEnabled ? MODE_FD : MODE_CAN20);
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
void Mcp251863Can::Send(uint32_t canId, uint32_t data[2], uint8_t len, bool forceExt)
{
   DISABLE_CAN_USER_INTERRUPTS();

   SendOrQueue(canId, data, len, forceExt);

   ENABLE_CAN_USER_INTERRUPTS();
}

bool Mcp251863Can::SendFd(uint32_t canId, const uint8_t* data, uint8_t len, bool brs, bool forceExt)
{
   if (!fdEnabled || len > 64)
      return false;

   DISABLE_CAN_USER_INTERRUPTS();
   bool ok = TransmitFrame(canId & 0x1FFFFFFF, (canId > 0x7FF) | forceExt, true, brs, len, data);
   ENABLE_CAN_USER_INTERRUPTS();
   return ok;
}

void Mcp251863Can::HandleInterrupt()
{
   HandleMessage();
   HandleTx();
}

void Mcp251863Can::HandleMessage()
{
   while (ReceiveMessage());
}

void Mcp251863Can::HandleTx()
{
   DISABLE_CAN_USER_INTERRUPTS();

   FlushSendBuffer();

   ENABLE_CAN_USER_INTERRUPTS();
}

/****************** Private methods ********************/

uint8_t Mcp251863Can::Xfer(uint8_t b)
{
   return spi_xfer(spiDev, b);
}

void Mcp251863Can::Header(uint8_t cmd, uint16_t addr)
{
   Xfer((cmd << 4) | ((addr >> 8) & 0xF));
   Xfer(addr & 0xFF);
}

void Mcp251863Can::Reset()
{
   csPin.Clear();
   Header(CMD_RESET, 0);
   csPin.Set();

   //wait for oscillator to become ready
   for (int i = 0; i < 100000 && !(ReadReg(REG_OSC) & OSC_OSCRDY); i++);
}

uint32_t Mcp251863Can::ReadReg(uint16_t addr)
{
   uint32_t val = 0;
   csPin.Clear();
   Header(CMD_READ, addr);
   for (int i = 0; i < 4; i++) val |= (uint32_t)Xfer(0) << (8 * i);
   csPin.Set();
   return val;
}

void Mcp251863Can::WriteReg(uint16_t addr, uint32_t val)
{
   csPin.Clear();
   Header(CMD_WRITE, addr);
   for (int i = 0; i < 4; i++) Xfer(val >> (8 * i));
   csPin.Set();
}

void Mcp251863Can::WriteByte(uint16_t addr, uint8_t val)
{
   csPin.Clear();
   Header(CMD_WRITE, addr);
   Xfer(val);
   csPin.Set();
}

void Mcp251863Can::ReadRam(uint16_t addr, uint8_t* buf, uint16_t len)
{
   csPin.Clear();
   Header(CMD_READ, addr);
   for (uint16_t i = 0; i < len; i++) buf[i] = Xfer(0);
   csPin.Set();
}

void Mcp251863Can::WriteRam(uint16_t addr, const uint8_t* buf, uint16_t len)
{
   csPin.Clear();
   Header(CMD_WRITE, addr);
   for (uint16_t i = 0; i < len; i++) Xfer(buf[i]);
   csPin.Set();
}

void Mcp251863Can::SetMode(uint8_t mode)
{
   uint32_t con = ReadReg(REG_C1CON);
   con = (con & ~(7u << 24)) | ((uint32_t)mode << 24);
   WriteReg(REG_C1CON, con);

   for (int i = 0; i < 10000 && ((ReadReg(REG_C1CON) >> 21) & 7) != mode; i++);
}

void Mcp251863Can::ConfigureTimings()
{
   uint32_t brp, tq;

   SetMode(MODE_CONFIG);

   //nominal bit rate, sample point at 80%
   tq = FindBitTiming(nominalBaud, 80, 256, brp);
   uint32_t tseg2 = tq / 5;
   uint32_t tseg1 = tq - 1 - tseg2;
   WriteReg(REG_C1NBTCFG, ((brp - 1) << 24) | ((tseg1 - 1) << 16) | ((tseg2 - 1) << 8) | (tseg2 - 1));

   if (fdEnabled)
   {
      //data bit rate, sample point at 80%
      tq = FindBitTiming(dataBaud, 48, 256, brp);
      tseg2 = tq / 5;
      tseg1 = tq - 1 - tseg2;
      if (tseg1 > 32) tseg1 = 32;
      if (tseg2 > 16) tseg2 = 16;
      WriteReg(REG_C1DBTCFG, ((brp - 1) << 24) | ((tseg1 - 1) << 16) | ((tseg2 - 1) << 8) | (tseg2 - 1));

      //automatic transmitter delay compensation
      uint32_t tdco = brp * tseg1;
      if (tdco > 63) tdco = 63;
      WriteReg(REG_C1TDC, (2 << 16) | (tdco << 8));
   }

   SetMode(runMode);
}

bool Mcp251863Can::Transmit(uint32_t canId, uint32_t data[2], uint8_t len, bool forceExt)
{
   return TransmitFrame(canId & 0x1FFFFFFF, (canId > 0x7FF) | forceExt, false, false, len, (const uint8_t*)data);
}

/** \return true if message was put into the transmit queue */
bool Mcp251863Can::TransmitFrame(uint32_t id, bool ext, bool fd, bool brs, uint8_t len, const uint8_t* data)
{
   if (!(ReadReg(REG_C1TXQSTA) & 1)) //TXQNIF: queue not full
      return false;

   if (!fd && len > 8) len = 8;

   uint8_t dlc = LenToDlc(len);
   uint8_t padded = dlcToLen[dlc];
   uint8_t buf[8 + 64];
   uint32_t t0 = IdToReg(id, ext);
   uint32_t t1 = dlc | (ext ? 0x10 : 0) | (fd ? 0x80 : 0) | (fd && brs ? 0x40 : 0);

   for (int i = 0; i < 4; i++)
   {
      buf[i] = t0 >> (8 * i);
      buf[4 + i] = t1 >> (8 * i);
   }
   for (int i = 0; i < padded; i++)
      buf[8 + i] = i < len ? data[i] : 0;

   //RAM is written in multiples of 4 bytes
   uint16_t total = (8 + padded + 3) & ~3;
   for (uint16_t i = 8 + padded; i < total; i++) buf[i] = 0;

   uint16_t addr = (ReadReg(REG_C1TXQUA) + RAM_START) & 0xFFF;
   WriteRam(addr, buf, total);
   WriteByte(REG_C1TXQCON + 1, 0x03); //UINC and TXREQ
   return true;
}

/** \brief Read one message from receive FIFO and pass it on
 * \return true if a message was available */
bool Mcp251863Can::ReceiveMessage()
{
   if (!(ReadReg(REG_C1FIFOSTA1) & 1)) //TFNRFNIF: FIFO not empty
      return false;

   uint8_t buf[8 + 64];
   uint16_t addr = (ReadReg(REG_C1FIFOUA1) + RAM_START) & 0xFFF;

   ReadRam(addr, buf, 8);
   uint32_t r0 = buf[0] | (buf[1] << 8) | (buf[2] << 16) | ((uint32_t)buf[3] << 24);
   uint8_t r1 = buf[4];
   uint8_t len = dlcToLen[r1 & 0xF];
   bool ext = (r1 & 0x10) != 0;
   bool brs = (r1 & 0x40) != 0;

   if (len > 0)
      ReadRam(addr + 8, buf + 8, (len + 3) & ~3);

   WriteByte(REG_C1FIFOCON1 + 1, 0x01); //UINC

   uint32_t id = r0 & 0x7FF;
   if (ext)
      id = (id << 18) | ((r0 >> 11) & 0x3FFFF);

   if (len > 8 && fdRxCallback != 0)
   {
      fdRxCallback(id, buf + 8, len, brs);
   }
   else
   {
      uint32_t data[2] = { 0, 0 };
      if (len > 8) len = 8;
      for (int i = 0; i < len; i++) ((uint8_t*)data)[i] = buf[8 + i];
      HandleRx(id, data, len);
   }
   lastRxTimestamp = rtc_get_counter_val();
   return true;
}

void Mcp251863Can::ConfigureFilters()
{
   //disable all filters, they must not be enabled while being modified
   for (int i = 0; i < NUM_FILTERS; i++)
      WriteByte(REG_C1FLTCON + i, 0);

   for (int i = 0; i < nextUserMessageIndex && i < NUM_FILTERS; i++)
   {
      bool ext = userIds[i] > 0x7FF;
      uint32_t id = userIds[i] & 0x1FFFFFFF;
      uint32_t mask = userMasks[i] != 0 ? userMasks[i] : (ext ? 0x1FFFFFFF : 0x7FF);

      WriteReg(REG_C1FLTOBJ + 8 * i, IdToReg(id, ext) | (ext ? (1u << 30) : 0)); //EXIDE
      WriteReg(REG_C1MASK + 8 * i, IdToReg(mask, ext) | (1u << 30)); //MIDE: match type
      WriteByte(REG_C1FLTCON + i, 0x80 | 1); //enable, point to FIFO 1
   }
}
