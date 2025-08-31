/*
 * This file is part of the stm32-sine project.
 *
 * Copyright (C) 2024 David J. Fiddes <D.J@fiddes.net>
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
#include "stdint.h"

void flash_unlock(void)
{
}

void flash_lock(void)
{
}

void flash_set_ws(uint32_t ws)
{
}

void flash_program_word(uint32_t address, uint32_t data)
{
}

void flash_erase_page(uint32_t page_address)
{
}

uint16_t desig_get_flash_size(void)
{
    return 8;
}

uint32_t crc_calculate(uint32_t data)
{
    return 0xaa55;
}

uint32_t crc_calculate_block(uint32_t *datap, int size)
{
    return 0xaaaa5555;
}

void crc_reset(void)
{
}

void gpio_set_mode(uint32_t gpioport, uint8_t mode, uint8_t cnf, uint16_t gpios)
{
}

void usart_set_baudrate(uint32_t usart, uint32_t baud)
{
}

void usart_set_databits(uint32_t usart, uint32_t bits)
{
}

void usart_set_stopbits(uint32_t usart, uint32_t stopbits)
{
}

void usart_set_mode(uint32_t usart, uint32_t mode)
{
}

void usart_set_parity(uint32_t usart, uint32_t parity)
{
}

void usart_set_flow_control(uint32_t usart, uint32_t flowcontrol)
{
}

void usart_enable_tx_dma(uint32_t usart)
{
}

void usart_enable_rx_dma(uint32_t usart)
{
}

void usart_enable(uint32_t usart)
{
}

void dma_channel_reset(uint32_t dma, uint8_t channel)
{
}

void dma_set_read_from_memory(uint32_t dma, uint8_t channel)
{
}

void dma_set_peripheral_address(uint32_t dma, uint8_t channel, uint32_t address)
{
}

void dma_set_memory_address(uint32_t dma, uint8_t channel, uint32_t address)
{
}

void dma_set_peripheral_size(uint32_t dma, uint8_t channel,
			     uint32_t peripheral_size)
{
}

void dma_set_memory_size(uint32_t dma, uint8_t channel, uint32_t mem_size)
{
}

void dma_enable_memory_increment_mode(uint32_t dma, uint8_t channel)
{
}

void dma_set_number_of_data(uint32_t dma, uint8_t channel, uint16_t number)
{
}

uint16_t dma_get_number_of_data(uint32_t dma, uint8_t channel)
{
    return 0x5a;
}

void dma_clear_interrupt_flags(uint32_t dma, uint8_t channel,
			       uint32_t interrupts)
{
}

void dma_enable_channel(uint32_t dma, uint8_t channel)
{
}

void dma_disable_channel(uint32_t dma, uint8_t channel)
{
}
