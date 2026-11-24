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

#include "my_string.h"
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include "terminal.h"
#include "printf.h"

#define TERM_BUF_MASK (TERM_BUF_DEPTH - 1)
static_assert(TERM_BUF_DEPTH > 1 && (TERM_BUF_DEPTH & (TERM_BUF_DEPTH - 1)) == 0, "TERM_BUF_DEPTH must be a power of 2");

#define HWINFO_ENTRIES (sizeof(hwInfo) / sizeof(struct HwInfo))

#ifndef USART_BAUDRATE
#define USART_BAUDRATE 115200
#endif // USART_BAUDRATE

const Terminal::HwInfo Terminal::hwInfo[] =
{
   { USART1, DMA1, DMA_CHANNEL4, DMA_CHANNEL5, GPIOA, GPIO_USART1_TX, GPIOB, GPIO_USART1_RE_TX },
   { USART2, DMA1, DMA_CHANNEL7, DMA_CHANNEL6, GPIOA, GPIO_USART2_TX, GPIOD, GPIO_USART2_RE_TX },
   { USART3, DMA1, DMA_CHANNEL2, DMA_CHANNEL3, GPIOB, GPIO_USART3_TX, GPIOC, GPIO_USART3_PR_TX },
   { UART4,  DMA2, DMA_CHANNEL5, DMA_CHANNEL3, GPIOC, GPIO_UART4_TX,  GPIOC, GPIO_UART4_TX },
};

Terminal* Terminal::defaultTerminal;

Terminal::Terminal(uint32_t usart, const TERM_CMD* commands, bool remap, bool echo, bool allowFastUart)
:  usart(usart),
   remap(remap),
   termCmds(commands),
   nodeId(1),
   enabled(true),
   txDmaEnabled(true),
   pCurCmd(NULL),
   lastIdx(0),
   echo(echo),
   allowFastUart(allowFastUart),
   txTail(0),
   txHead(0),
   txUsed(0),
   txInFlight(0),
   flushRequested(false),
   overrun(OVERRUN_BLOCK)
{
   //Search info entry
   hw = hwInfo;
   for (uint32_t i = 0; i < HWINFO_ENTRIES; i++)
   {
      if (hw->usart == usart) break;
      hw++;
   }

   defaultTerminal = this;

   gpio_set_mode(remap ? hw->port_re : hw->port, GPIO_MODE_OUTPUT_50_MHZ,
               GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, remap ? hw->pin_re : hw->pin);

   usart_set_baudrate(usart, USART_BAUDRATE);
   usart_set_databits(usart, 8);
   usart_set_stopbits(usart, USART_STOPBITS_2);
   usart_set_mode(usart, USART_MODE_TX_RX);
   usart_set_parity(usart, USART_PARITY_NONE);
   usart_set_flow_control(usart, USART_FLOWCONTROL_NONE);
   usart_enable_rx_dma(usart);
   usart_enable_tx_dma(usart);

   dma_channel_reset(hw->dmactl, hw->dmatx);
   dma_set_read_from_memory(hw->dmactl, hw->dmatx);
   dma_set_peripheral_address(hw->dmactl, hw->dmatx, (uint32_t)&USART_DR(usart));
   dma_set_peripheral_size(hw->dmactl, hw->dmatx, DMA_CCR_PSIZE_8BIT);
   dma_set_memory_size(hw->dmactl, hw->dmatx, DMA_CCR_MSIZE_8BIT);
   dma_enable_memory_increment_mode(hw->dmactl, hw->dmatx);

   dma_channel_reset(hw->dmactl, hw->dmarx);
   dma_set_peripheral_address(hw->dmactl, hw->dmarx, (uint32_t)&USART_DR(usart));
   dma_set_peripheral_size(hw->dmactl, hw->dmarx, DMA_CCR_PSIZE_8BIT);
   dma_set_memory_size(hw->dmactl, hw->dmarx, DMA_CCR_MSIZE_8BIT);
   dma_enable_memory_increment_mode(hw->dmactl, hw->dmarx);
   dma_enable_channel(hw->dmactl, hw->dmarx);

   ResetDMA();

   usart_enable(usart);
}

/** Run the terminal */
void Terminal::Run()
{
   ReapDMA();
   if (flushRequested)
      StartDMA();

   int unusedBytes = dma_get_number_of_data(hw->dmactl, hw->dmarx);
   int currentIdx = bufSize - unusedBytes;

   if (0 == unusedBytes)
      ResetDMA();

   while (echo && lastIdx < currentIdx) //echo
      usart_send_blocking(usart, inBuf[lastIdx++]);

   if (usart_get_flag(usart, USART_SR_ORE))
      usart_recv(usart); //Clear possible overrun

   if (currentIdx > 0)
   {
      if (inBuf[currentIdx - 1] == '\n' || inBuf[currentIdx - 1] == '\r')
      {
         //Do not accept a new command while processing the current one
         dma_disable_channel(hw->dmactl, hw->dmarx);
         if (currentIdx > 1) //handle just \n quicker
         {
            inBuf[currentIdx] = 0;
            lastIdx = 0;
            char *space = (char*)my_strchr(inBuf, ' ');

            if (0 == *space) //No args after command, look for end of line
            {
               space = (char*)my_strchr(inBuf, '\n');
               args[0] = 0;
            }
            else //There are arguments, copy everything behind the space
            {
               my_strcpy(args, space + 1);
            }

            if (0 == *space) //No \n found? try \r
               space = (char*)my_strchr(inBuf, '\r');

            *space = 0;
            pCurCmd = NULL;

            if (my_strcmp(inBuf, "enableuart") == 0)
            {
               EnableUart(args);
               currentIdx = 0; //Prevent unknown command message
            }
            else if (my_strcmp(inBuf, "fastuart") == 0)
            {
               if (allowFastUart)
                  FastUart(args);
               else
                  Send("fastuart not available\r\n");

               currentIdx = 0;
            }
            else if (my_strcmp(inBuf, "echo") == 0)
            {
               Echo(args);
               currentIdx = 0;
            }
            else
            {
               pCurCmd = CmdLookup(inBuf);
            }

            if (NULL != pCurCmd)
            {
               usart_wait_send_ready(usart);
               pCurCmd->CmdFunc(this, args);
            }
            else if (currentIdx > 1 && enabled)
            {
               Send("Unknown command sequence\r\n");
            }
         }
         ResetDMA();
      }
      else if (inBuf[0] == '!' && NULL != pCurCmd)
      {
         ResetDMA();
         lastIdx = 0;
         pCurCmd->CmdFunc(this, args);
      }
   }
}

void Terminal::SetNodeId(uint8_t id)
{
   char one[] = { '1', 0 };
   char buf[4];
   nodeId = id;

   if (nodeId != 1)
   {
      Send("Disabling terminal, type 'enableuart ");
      my_ltoa(buf, id, 10);
      Send(buf);
      Send("' to re-enable\r\n");
   }

   EnableUart(one);
}

/*
 * Revision 1 hardware can only use synchronous sending as the DMA channel is
 * occupied by the encoder timer (TIM3, channel 3).
 * All other hardware can use DMA for seamless sending of data. Characters are
 * collected in a FIFO. The oldest contiguous chunk is handed to DMA when the
 * FIFO is 70% full or a newline is sent.
*/
void Terminal::PutChar(char c)
{
   if (!txDmaEnabled)
   {
      usart_send_blocking(usart, c);
      return;
   }

   Enqueue(c);

   if (c == '\n' || txUsed * 10 >= TERM_BUF_DEPTH * 7)
      Flush();
}

void Terminal::SetOverrunBehaviour(OverrunBehaviour behaviour)
{
   overrun = behaviour;
}

void Terminal::SendBinary(const uint8_t* data, uint32_t len)
{
   if (!txDmaEnabled)
   {
      for (uint32_t i = 0; i < len; i++)
         usart_send_blocking(usart, data[i]);
      return;
   }

   for (uint32_t i = 0; i < len; i++)
      Enqueue(data[i]);
   Flush();
}

void Terminal::SendBinary(const uint32_t* data, uint32_t len)
{
   SendBinary((const uint8_t*)data, len * sizeof(uint32_t));
}

//Release the DMA chunk if it has been sent completely
void Terminal::ReapDMA()
{
   if (txInFlight > 0 && dma_get_interrupt_flag(hw->dmactl, hw->dmatx, DMA_TCIF))
   {
      txTail = (txTail + txInFlight) & TERM_BUF_MASK;
      txUsed -= txInFlight;
      txInFlight = 0;
   }
}

//Hand the oldest contiguous unsent data to DMA, if DMA is idle
void Terminal::StartDMA()
{
   ReapDMA();

   if (txInFlight > 0 || txUsed == 0)
      return;

   uint32_t len = TERM_BUF_DEPTH - txTail;
   if (txUsed < len) len = txUsed;

   dma_disable_channel(hw->dmactl, hw->dmatx);
   dma_set_number_of_data(hw->dmactl, hw->dmatx, len);
   dma_set_memory_address(hw->dmactl, hw->dmatx, (uint32_t)&outBuf[txTail]);
   dma_clear_interrupt_flags(hw->dmactl, hw->dmatx, DMA_TCIF);
   dma_enable_channel(hw->dmactl, hw->dmatx);
   txInFlight = len;
   flushRequested = false;
}

//Request sending of all queued data, as soon as DMA is available
void Terminal::Flush()
{
   flushRequested = true;
   StartDMA();
}

bool Terminal::Enqueue(char c)
{
   ReapDMA();

   while (txUsed >= TERM_BUF_DEPTH)
   {
      if (overrun == OVERRUN_DROP)
         return false;
      StartDMA(); //make sure something is in flight that will free up space
      ReapDMA();
   }

   outBuf[txHead] = c;
   txHead = (txHead + 1) & TERM_BUF_MASK;
   txUsed++;
   return true;
}

bool Terminal::KeyPressed()
{
   return usart_get_flag(usart, USART_SR_RXNE);
}

void Terminal::FlushInput()
{
   usart_recv(usart);
}

void Terminal::DisableTxDMA()
{
   txDmaEnabled = false;
   dma_disable_channel(hw->dmactl, hw->dmatx);
   usart_disable_tx_dma(usart);
}

void Terminal::ResetDMA()
{
   dma_disable_channel(hw->dmactl, hw->dmarx);
   dma_set_memory_address(hw->dmactl, hw->dmarx, (uint32_t)inBuf);
   dma_set_number_of_data(hw->dmactl, hw->dmarx, bufSize);
   dma_enable_channel(hw->dmactl, hw->dmarx);
}

void Terminal::EnableUart(char* arg)
{
   arg = my_trim(arg);
   int val = my_atoi(arg);

   if (val == nodeId)
   {
      enabled = true;
      gpio_set_mode(remap ? hw->port_re : hw->port, GPIO_MODE_OUTPUT_50_MHZ,
               GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, remap ? hw->pin_re : hw->pin);
      Send("OK\r\n");
   }
   else
   {
      enabled = false;
      gpio_set_mode(remap ? hw->port_re : hw->port, GPIO_MODE_INPUT,
               GPIO_CNF_INPUT_FLOAT, remap ? hw->pin_re : hw->pin);
   }
}

void Terminal::FastUart(char *arg)
{
   arg = my_trim(arg);
   int baud = arg[0] == '0' ? USART_BAUDRATE : (arg[0] == '2' ? 2250000 : 921600);
   if (enabled)
   {
      char buf[10];
      my_ltoa(buf, baud, 10);
      Send("\nOK\r\n");
      Send("Baud rate now ");
      Send(buf);
      Send("\r\n");
   }
   usart_set_baudrate(usart, baud);
   usart_set_stopbits(usart, USART_STOPBITS_1);
}

void Terminal::Echo(char* arg)
{
   arg = my_trim(arg);

   if (arg[0] != 0)
      echo = arg[0] == '0' ? false : true;

   if (echo)
      Send("Echo on\r\n");
   else
      Send("Echo off\r\n");
}

const TERM_CMD* Terminal::CmdLookup(char *buf)
{
   const TERM_CMD *pCmd = termCmds;

   if (!enabled) return NULL;

   for (; NULL != pCmd->cmd; pCmd++)
   {
      if (0 == my_strcmp(buf, pCmd->cmd))
      {
         break;
      }
   }
   if (NULL == pCmd->cmd)
   {
      pCmd = NULL;
   }
   return pCmd;
}

void Terminal::Send(const char *str)
{
   SendBinary((const uint8_t*)str, my_strlen(str));
}

//Backward compatibility for printf
extern "C" void libopeninv_putchar(int c)
{
   if (Terminal::defaultTerminal != nullptr)
      Terminal::defaultTerminal->PutChar(c);
}
