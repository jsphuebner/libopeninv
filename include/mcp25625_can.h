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
#ifndef MCP25625_CAN_H_INCLUDED
#define MCP25625_CAN_H_INCLUDED
#include "canhardware.h"
#include "digio.h"

/** CAN controller MCP25625(T) (MCP2515 compatible) connected via SPI.
 * Call HandleInterrupt() when the INT pin of the chip is asserted or poll it regularly.
 */
class Mcp25625Can: public CanHardware
{
public:
   /** \param spi libopencm3 SPI peripheral base address, must be configured for mode 0, MSB first, 8 bit
    *  \param cs chip select pin, must be configured as output, active low
    *  \param baudrate initial baud rate
    *  The oscillator frequency can be set via MCP25625_OSC_HZ (default 16 MHz)
    */
   Mcp25625Can(uint32_t spi, DigIo& cs, enum baudrates baudrate);
   void SetBaudrate(enum baudrates baudrate);
   void Send(uint32_t canId, uint32_t data[2], uint8_t len, bool forceExt = false);
   /** Service chip: receive pending messages and transmit buffered ones */
   void HandleInterrupt();
   /** Transmit buffered messages if a transmit buffer became free */
   void HandleTx();
   /** Receive all pending messages */
   void HandleMessage();
   /** Enable/disable listen only mode (no ACK, no transmission) */
   void SetListenOnly(bool enable);

private:
   uint32_t spiDev;
   DigIo& csPin;
   uint8_t runMode;

   void ConfigureFilters();
   uint8_t Xfer(uint8_t b);
   void Reset();
   uint8_t ReadReg(uint8_t addr);
   void WriteReg(uint8_t addr, uint8_t val);
   void BitModify(uint8_t addr, uint8_t mask, uint8_t val);
   uint8_t ReadStatus();
   void SetMode(uint8_t mode);
   bool Transmit(uint32_t canId, uint32_t data[2], uint8_t len, bool forceExt) override;
   bool ReceiveBuffer(int index);
   bool IsUserMessage(uint32_t id, bool ext);
};

#endif
