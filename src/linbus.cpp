/*
 * This file is part of the stm32-... project.
 *
 * Copyright (C) 2021 Johannes Huebner <dev@johanneshuebner.com>
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
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include "linbus.h"

#define HWINFO_ENTRIES (sizeof(hwInfo) / sizeof(struct HwInfo))

const LinBus::HwInfo LinBus::hwInfo[] =
{
   { USART1, DMA1, DMA_CHANNEL4, DMA_CHANNEL5, GPIOA, GPIO_USART1_TX, GPIO_USART1_RX },
   { USART2, DMA1, DMA_CHANNEL7, DMA_CHANNEL6, GPIOA, GPIO_USART2_TX, GPIO_USART2_RX },
   { USART3, DMA1, DMA_CHANNEL2, DMA_CHANNEL3, GPIOB, GPIO_USART3_TX, GPIO_USART3_RX },
   { UART4,  DMA2, DMA_CHANNEL5, DMA_CHANNEL3, GPIOC, GPIO_UART4_TX, GPIO_UART4_RX },
};

/** \brief Create a new LIN bus object but DO NOT configure hardware
 *
 */
LinBus::LinBus()
   : hw(hwInfo), sendBuffer{}, recvBuffer{}
{
}

/** \brief Create a new LIN bus object and initialize USART and DMA
 * \pre Associated USART, GPIO and DMA clocks must be enabled.
 * \param usart USART base address
 * \param baudrate 9600 or 19200
 *
 */
LinBus::LinBus(uint32_t usart, int baudrate)
   : hw(hwInfo), sendBuffer{}, recvBuffer{}
{
   Init(usart, baudrate);
}

/** \brief Initialise the hardware associated with a previously created LIN bus
 * object
 * \pre Associated USART, GPIO and DMA clocks must be enabled.
 * \param usart USART base address
 * \param baudrate 9600 or 19200
 *
 */
void LinBus::Init( uint32_t usart, int baudrate)
{
   for (uint32_t i = 0; i < HWINFO_ENTRIES; i++)
   {
      if (hw->usart == usart) break;
      hw++;
   }

   gpio_set_mode(hw->port, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, hw->pintx);
   gpio_set_mode(hw->port, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, hw->pinrx);

   usart_set_baudrate(usart, baudrate);
   usart_set_databits(usart, 8);
   usart_set_stopbits(usart, USART_STOPBITS_1);
   usart_set_mode(usart, USART_MODE_TX_RX);
   usart_set_parity(usart, USART_PARITY_NONE);
   usart_set_flow_control(usart, USART_FLOWCONTROL_NONE);
   USART_CR2(usart) |= USART_CR2_LINEN;
   usart_enable_tx_dma(usart);
   usart_enable_rx_dma(usart);

   dma_channel_reset(hw->dma, hw->dmatx);
   dma_set_read_from_memory(hw->dma, hw->dmatx);
   dma_set_peripheral_address(hw->dma, hw->dmatx, (uint32_t)&USART_DR(usart));
   dma_set_memory_address(hw->dma, hw->dmatx, (uint32_t)sendBuffer);
   dma_set_peripheral_size(hw->dma, hw->dmatx, DMA_CCR_PSIZE_8BIT);
   dma_set_memory_size(hw->dma, hw->dmatx, DMA_CCR_MSIZE_8BIT);
   dma_enable_memory_increment_mode(hw->dma, hw->dmatx);

   dma_channel_reset(hw->dma, hw->dmarx);
   dma_set_peripheral_address(hw->dma, hw->dmarx, (uint32_t)&USART_DR(usart));
   dma_set_peripheral_size(hw->dma, hw->dmarx, DMA_CCR_PSIZE_8BIT);
   dma_set_memory_size(hw->dma, hw->dmarx, DMA_CCR_MSIZE_8BIT);
   dma_enable_memory_increment_mode(hw->dma, hw->dmarx);

   usart_enable(usart);
}

/** \brief Send data on LIN bus
 *
 * \param id feature ID
 * \param data payload data, if any
 * \param len length of payload, if any
 *
 */
void LinBus::Request(uint8_t id, uint8_t* data, uint8_t len)
{
   int sendLen = len == 0 ? 2 : len + 3;

   if (len > 8) return;

   dma_disable_channel(hw->dma, hw->dmatx);
   dma_set_number_of_data(hw->dma, hw->dmatx, sendLen);
   dma_disable_channel(hw->dma, hw->dmarx);
   dma_set_memory_address(hw->dma, hw->dmarx, (uint32_t)recvBuffer);
   dma_set_number_of_data(hw->dma, hw->dmarx, sizeof(recvBuffer));

   sendBuffer[0] = 0x55; //Sync
   sendBuffer[1] = Parity(id);

   for (uint8_t i = 0; i < len; i++)
      sendBuffer[i + 2] = data[i];

   sendBuffer[len + 2] = Checksum(sendBuffer[1], data, len);

   dma_clear_interrupt_flags(hw->dma, hw->dmatx, DMA_TCIF);

   USART_CR1(hw->usart) |= USART_CR1_SBK;
   dma_enable_channel(hw->dma, hw->dmatx);
   dma_enable_channel(hw->dma, hw->dmarx);
}

/** \brief Check whether we received valid data with given PID and length
 *
 * \param pid Feature ID to check for
 * \param requiredLen Length of data we expect
 * \return true if data with given properties was received
 *
 */
bool LinBus::HasReceived(uint8_t id, uint8_t requiredLen)
{
   int numRcvd = dma_get_number_of_data(hw->dma, hw->dmarx);
   int receiveIdx = sizeof(recvBuffer) - numRcvd;

   if (requiredLen > 8) return false;

   uint8_t pid = Parity(id);

   if (receiveIdx == (requiredLen + payloadIndex + 1) && recvBuffer[pidIndex] == pid)
   {
      uint8_t checksum = Checksum(recvBuffer[pidIndex], &recvBuffer[payloadIndex], requiredLen);

      return checksum == recvBuffer[requiredLen + payloadIndex];
   }

   return false;
}

/** \brief Calculate LIN checksum
 *
 * \param pid ID with parity
 * \param data uint8_t*
 * \param len int
 * \return checksum
 *
 */
uint8_t LinBus::Checksum(uint8_t pid, uint8_t* data, int len)
{
   uint8_t checksum = pid;

   for (int i = 0; i < len; i++)
   {
      uint16_t tmp = (uint16_t)checksum + (uint16_t)data[i];
      if (tmp >= 256) tmp -= 255;
      checksum = tmp;
   }
   return checksum ^ 0xff;
}

uint8_t LinBus::Parity(uint8_t id)
{
   bool p1 = !(((id & 0x2) > 0) ^ ((id & 0x8) > 0) ^ ((id & 0x10) > 0) ^ ((id & 0x20) > 0));
   bool p0 = ((id & 0x1) > 0) ^ ((id & 0x2) > 0) ^ ((id & 0x4) > 0) ^ ((id & 0x10) > 0);

   return id | p1 << 7 | p0 << 6;
}
