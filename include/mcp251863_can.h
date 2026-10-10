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
#ifndef MCP251863_CAN_H_INCLUDED
#define MCP251863_CAN_H_INCLUDED
#include "canhardware.h"
#include "digio.h"

#ifndef SENDBUFFER_LEN
#define SENDBUFFER_LEN 20
#endif // SENDBUFFER_LEN

/** CAN FD controller MCP251863(T) (MCP2518FD compatible) connected via SPI.
 * Call HandleInterrupt() when the INT pin of the chip is asserted or poll it regularly.
 * By default the chip operates in classic CAN 2.0 mode. CAN FD is enabled via EnableFd().
 */
class Mcp251863Can: public CanHardware
{
public:
   /** Callback for received frames longer than 8 bytes */
   typedef void (*FdRxCallback)(uint32_t canId, uint8_t* data, uint8_t len, bool brs);

   /** \param spi libopencm3 SPI peripheral base address, must be configured for mode 0, MSB first, 8 bit
    *  \param cs chip select pin, must be configured as output, active low
    *  \param baudrate initial nominal baud rate
    *  The system clock (oscillator) frequency can be set via MCP251863_SYSCLK_HZ (default 40 MHz)
    */
   Mcp251863Can(uint32_t spi, DigIo& cs, enum baudrates baudrate);
   void SetBaudrate(enum baudrates baudrate);
   void Send(uint32_t canId, uint32_t data[2], uint8_t len, bool forceExt = false);
   /** Service chip: receive pending messages and transmit buffered ones */
   void HandleInterrupt();
   /** Transmit buffered messages if there is room in the transmit queue */
   void HandleTx();
   /** Receive all pending messages */
   void HandleMessage();

   /** Enable or disable CAN FD operation. With FD disabled the chip works in CAN 2.0 mode.
    * \param dataBaudrate data phase bit rate in bit/s, only used when enabling */
   void EnableFd(bool enable, uint32_t dataBaudrate = 2000000);
   /** Change data phase bit rate in bit/s */
   void SetDataBaudrate(uint32_t dataBaudrate);
   /** Send a CAN FD frame with up to 64 bytes. Needs EnableFd(true).
    * \param len 0-64, lengths not matching a valid DLC are padded with zeros
    * \param brs use bit rate switching for data phase
    * \return false if there is no room in the transmit queue */
   bool SendFd(uint32_t canId, const uint8_t* data, uint8_t len, bool brs = true, bool forceExt = false);
   /** Set handler for received frames longer than 8 bytes. Without it they are truncated to 8 bytes */
   void SetFdRxCallback(FdRxCallback cb) { fdRxCallback = cb; }
   /** Enable/disable listen only mode (no ACK, no transmission) */
   void SetListenOnly(bool enable);

private:
   struct SENDBUFFER
   {
      uint32_t id;
      bool forceExt;
      uint8_t len;
      uint32_t data[2];
   };

   SENDBUFFER sendBuffer[SENDBUFFER_LEN];
   int sendCnt;
   uint32_t spiDev;
   DigIo& csPin;
   uint8_t runMode;
   bool fdEnabled;
   uint32_t dataBaud;
   uint32_t nominalBaud;
   FdRxCallback fdRxCallback;

   void ConfigureFilters();
   uint8_t Xfer(uint8_t b);
   void Header(uint8_t cmd, uint16_t addr);
   void Reset();
   uint32_t ReadReg(uint16_t addr);
   void WriteReg(uint16_t addr, uint32_t val);
   void WriteByte(uint16_t addr, uint8_t val);
   void ReadRam(uint16_t addr, uint8_t* buf, uint16_t len);
   void WriteRam(uint16_t addr, const uint8_t* buf, uint16_t len);
   void SetMode(uint8_t mode);
   void ConfigureTimings();
   bool Transmit(uint32_t id, bool ext, bool fd, bool brs, uint8_t len, const uint8_t* data);
   bool ReceiveMessage();
};

#endif
