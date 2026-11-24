/*
 * This file is part of the libopeninv project.
 *
 * Copyright (C) 2011 Johannes Huebner <dev@johanneshuebner.com>
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
#ifndef TERMINAL_H
#define TERMINAL_H
#include <stdint.h>
#include "printf.h"

#ifndef TERM_BUF_DEPTH
#define TERM_BUF_DEPTH 256 //must be a power of 2
#endif

class Terminal;

typedef struct
{
   char const *cmd;
   void (*CmdFunc)(Terminal*, char*);
} TERM_CMD;

class Terminal: public IPutChar
{
public:
   enum OverrunBehaviour { OVERRUN_DROP, OVERRUN_BLOCK };

   Terminal(uint32_t usart, const TERM_CMD* commands, bool remap = false, bool echo = true, bool allowFastUart = true);
   void SetNodeId(uint8_t id);
   void Run();
   void PutChar(char c);
   void SendBinary(const uint8_t* data, uint32_t length);
   void SendBinary(const uint32_t* data, uint32_t length);
   bool KeyPressed();
   void FlushInput();
   void DisableTxDMA();
   /** Choose what happens when the transmit FIFO is full: drop new characters or block until space is free */
   void SetOverrunBehaviour(OverrunBehaviour behaviour);
   static Terminal* defaultTerminal;

private:
   struct HwInfo
   {
      uint32_t usart;
      uint32_t dmactl;
      uint8_t dmatx;
      uint8_t dmarx;
      uint32_t port;
      uint16_t pin;
      uint32_t port_re;
      uint16_t pin_re;
   };

   void ResetDMA();
   const TERM_CMD *CmdLookup(char *buf);
   void EnableUart(char* arg);
   void FastUart(char* arg);
   void Echo(char* arg);
   void Send(const char *str);
   bool Enqueue(char c);
   void ReapDMA();
   void StartDMA();
   void Flush();

   static const int bufSize = 128;
   static const HwInfo hwInfo[];
   const HwInfo* hw;
   uint32_t usart;
   bool remap;
   const TERM_CMD* termCmds;
   uint8_t nodeId;
   bool enabled;
   bool txDmaEnabled;
   const TERM_CMD *pCurCmd;
   int lastIdx;
   bool echo;
   char inBuf[bufSize];
   char outBuf[TERM_BUF_DEPTH]; //tx fifo
   char args[bufSize];
   bool allowFastUart;
   uint32_t txTail;     //oldest byte not yet fully sent
   uint32_t txHead;     //next write position
   uint32_t txUsed;     //bytes in fifo, including the chunk currently in DMA
   uint32_t txInFlight; //size of chunk currently handed to DMA
   bool flushRequested;
   OverrunBehaviour overrun;
};

#endif // TERMINAL_H
