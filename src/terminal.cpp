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

Terminal::Terminal(uint32_t usart, const TERM_CMD* commands, bool remap, bool echo)
:  usart(usart),
   remap(remap),
   termCmds(commands),
   nodeId(1),
   enabled(true),
   txDmaEnabled(true),
   pCurCmd(NULL),
   lastIdx(0),
   curBuf(0),
   curIdx(0),
   firstSend(true),
   echo(echo)
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
   int unusedBytes = dma_get_number_of_data(hw->dmactl, hw->dmarx);
   int currentIdx = bufSize - unusedBytes;

   if (usart_get_flag(usart, USART_SR_ORE))
      usart_recv(usart); //Clear possible overrun

   if (0 == unusedBytes)
      ResetDMA();

   while (echo && lastIdx < currentIdx) //echo
      usart_send_blocking(usart, inBuf[lastIdx++]);

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
               FastUart(args);
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
 * All other hardware can use DMA for seamless sending of data. We use double
 * buffering, so while one buffer is sent by DMA we can prepare the other
 * buffer to go next.
*/
void Terminal::PutChar(char c)
{
   if (!txDmaEnabled)
   {
      usart_send_blocking(usart, c);
   }
   else if (c == '\n' || curIdx == (bufSize - 1))
   {
      outBuf[curBuf][curIdx] = c;
      SendCurrentBuffer(curIdx + 1);
      curIdx = 0;
   }
   else
   {
      outBuf[curBuf][curIdx] = c;
      curIdx++;
   }
}

void Terminal::SendBinary(const uint8_t* data, uint32_t len)
{
   uint32_t limitedLen = len < bufSize ? len : bufSize;

   for (uint32_t i = 0; i < limitedLen; i++)
      outBuf[curBuf][i] = data[i];
   SendCurrentBuffer(limitedLen);
}

void Terminal::SendBinary(const uint32_t* data, uint32_t len)
{
   uint32_t limitedLen = len < (bufSize / sizeof(uint32_t)) ? len : bufSize / sizeof(uint32_t);
   memcpy32((int*)outBuf[curBuf], (int*)data, limitedLen);
   SendCurrentBuffer(limitedLen * sizeof(uint32_t));
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
      Send("OK\r\n");
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

void Terminal::SendCurrentBuffer(uint32_t len)
{
   while (!dma_get_interrupt_flag(hw->dmactl, hw->dmatx, DMA_TCIF) && !firstSend);

   dma_disable_channel(hw->dmactl, hw->dmatx);
   dma_set_number_of_data(hw->dmactl, hw->dmatx, len);
   dma_set_memory_address(hw->dmactl, hw->dmatx, (uint32_t)outBuf[curBuf]);
   dma_clear_interrupt_flags(hw->dmactl, hw->dmatx, DMA_TCIF);
   dma_enable_channel(hw->dmactl, hw->dmatx);

   curBuf = !curBuf; //switch buffers
   firstSend = false; //only needed once so we don't get stuck in the while loop above
}

//Backward compatibility for printf
extern "C" void putchar(int c)
{
   Terminal::defaultTerminal->PutChar(c);
}
