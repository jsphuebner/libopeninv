/*
 * This file is part of the libopeninv project.
 *
 * Copyright (C) 2016 Nail Güzel
 * Johannes Huebner <dev@johanneshuebner.com>
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
#include "hwdefs.h"
#include "my_math.h"
#include "printf.h"
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/nvic.h>
#include "stm32_can.h"

#define MAX_INTERFACES        2
#define IDS_PER_BANK          4
#define EXT_IDS_PER_BANK      2

struct CANSPEED
{
   uint32_t ts1;
   uint32_t ts2;
   uint32_t prescaler;
};

Stm32Can* Stm32Can::interfaces[MAX_INTERFACES];

static const CANSPEED canSpeed[CanHardware::BaudLast] =
{
   { CAN_BTR_TS1_9TQ, CAN_BTR_TS2_6TQ, 18}, //125kbps
   { CAN_BTR_TS1_9TQ, CAN_BTR_TS2_6TQ, 9 }, //250kbps
   { CAN_BTR_TS1_4TQ, CAN_BTR_TS2_3TQ, 9 }, //500kbps
   { CAN_BTR_TS1_5TQ, CAN_BTR_TS2_3TQ, 5 }, //800kbps
   { CAN_BTR_TS1_6TQ, CAN_BTR_TS2_5TQ, 3 }, //1000kbps
};



/** \brief Init can hardware with given baud rate
 * Initializes the following sub systems:
 * - CAN hardware itself
 * - Appropriate GPIO pins
 * - Enables appropriate interrupts in NVIC
 *
 * \param baseAddr base address of CAN peripheral, CAN1 or CAN2
 * \param baudrate enum baudrates
 * \param remap use remapped IO pins
 * \return void
 *
 */
Stm32Can::Stm32Can(uint32_t baseAddr, enum baudrates baudrate, bool remap)
   : sendCnt(0), canDev(baseAddr)
{
   switch (baseAddr)
   {
      case CAN1:
         if (remap)
         {
            // Configure CAN pin: RX (input pull-up).
            gpio_set_mode(GPIO_BANK_CAN1_PB_RX, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_CAN1_PB_RX);
            gpio_set(GPIO_BANK_CAN1_PB_RX, GPIO_CAN1_PB_RX);
            // Configure CAN pin: TX.-
            gpio_set_mode(GPIO_BANK_CAN1_PB_TX, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_CAN1_PB_TX);
         }
         else
         {
            // Configure CAN pin: RX (input pull-up).
            gpio_set_mode(GPIO_BANK_CAN1_RX, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_CAN1_RX);
            gpio_set(GPIO_BANK_CAN1_RX, GPIO_CAN1_RX);
            // Configure CAN pin: TX.-
            gpio_set_mode(GPIO_BANK_CAN1_TX, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_CAN1_TX);
         }

         //CAN1 RX and TX IRQs
         nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ); //CAN RX
         nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, 0xf << 4); //lowest priority
         nvic_enable_irq(NVIC_CAN_RX1_IRQ); //CAN RX
         nvic_set_priority(NVIC_CAN_RX1_IRQ, 0xf << 4); //lowest priority
         nvic_enable_irq(NVIC_USB_HP_CAN_TX_IRQ); //CAN TX
         nvic_set_priority(NVIC_USB_HP_CAN_TX_IRQ, 0xf << 4); //lowest priority
         interfaces[0] = this;
         break;
      case CAN2:
         if (remap)
         {
            // Configure CAN pin: RX (input pull-up).
            gpio_set_mode(GPIO_BANK_CAN2_RE_RX, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_CAN2_RE_RX);
            gpio_set(GPIO_BANK_CAN2_RE_RX, GPIO_CAN2_RE_RX);
            // Configure CAN pin: TX.-
            gpio_set_mode(GPIO_BANK_CAN2_RE_TX, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_CAN2_RE_TX);
         }
         else
         {
            // Configure CAN pin: RX (input pull-up).
            gpio_set_mode(GPIO_BANK_CAN2_RX, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO_CAN2_RX);
            gpio_set(GPIO_BANK_CAN2_RX, GPIO_CAN2_RX);
            // Configure CAN pin: TX.-
            gpio_set_mode(GPIO_BANK_CAN2_TX, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_CAN2_TX);
         }

         //CAN2 RX and TX IRQs
         nvic_enable_irq(NVIC_CAN2_RX0_IRQ); //CAN RX
         nvic_set_priority(NVIC_CAN2_RX0_IRQ, 0xf << 4); //lowest priority
         nvic_enable_irq(NVIC_CAN2_RX1_IRQ); //CAN RX
         nvic_set_priority(NVIC_CAN2_RX1_IRQ, 0xf << 4); //lowest priority
         nvic_enable_irq(NVIC_CAN2_TX_IRQ); //CAN RX
         nvic_set_priority(NVIC_CAN2_TX_IRQ, 0xf << 4); //lowest priority
         interfaces[1] = this;
         break;
   }

	// Reset CAN
	can_reset(canDev);

	SetBaudrate(baudrate);
   ConfigureFilters();
	// Enable CAN RX interrupts.
	can_enable_irq(canDev, CAN_IER_FMPIE0);
	can_enable_irq(canDev, CAN_IER_FMPIE1);
}

/** \brief Set baud rate to given value
 *
 * \param baudrate enum baudrates
 * \return void
 *
 */
void Stm32Can::SetBaudrate(enum baudrates baudrate)
{
	// CAN cell init.
	 // Setting the bitrate to 250KBit. APB1 = 36MHz,
	 // prescaler = 9 -> 4MHz time quanta frequency.
	 // 1tq sync + 9tq bit segment1 (TS1) + 6tq bit segment2 (TS2) =
	 // 16time quanto per bit period, therefor 4MHz/16 = 250kHz
	 //
	can_init(canDev,
		     false,          // TTCM: Time triggered comm mode?
		     true,           // ABOM: Automatic bus-off management?
		     false,          // AWUM: Automatic wakeup mode?
		     false,          // NART: No automatic retransmission?
		     false,          // RFLM: Receive FIFO locked mode?
		     false,          // TXFP: Transmit FIFO priority?
		     CAN_BTR_SJW_1TQ,
		     canSpeed[baudrate].ts1,
		     canSpeed[baudrate].ts2,
		     canSpeed[baudrate].prescaler,				// BRP+1: Baud rate prescaler
		     false,
		     false);
}

/** \brief Send a user defined CAN message
 *
 * \param canId uint32_t
 * \param data[2] uint32_t
 * \param len message length
 * \return void
 *
 */
void Stm32Can::Send(uint32_t canId, uint32_t data[2], uint8_t len)
{
   can_disable_irq(canDev, CAN_IER_TMEIE);

   if (can_transmit(canDev, canId, canId > 0x7FF, false, len, (uint8_t*)data) < 0 && sendCnt < SENDBUFFER_LEN)
   {
      /* enqueue in send buffer if all TX mailboxes are full */
      sendBuffer[sendCnt].id = canId;
      sendBuffer[sendCnt].len = len;
      sendBuffer[sendCnt].data[0] = data[0];
      sendBuffer[sendCnt].data[1] = data[1];
      sendCnt++;
   }

   if (sendCnt > 0)
   {
      can_enable_irq(canDev, CAN_IER_TMEIE);
   }
}


Stm32Can* Stm32Can::GetInterface(int index)
{
   if (index < MAX_INTERFACES)
   {
      return interfaces[index];
   }
   return 0;
}

void Stm32Can::HandleMessage(int fifo)
{
   uint32_t id;
	bool ext, rtr;
	uint8_t length, fmi;
	uint32_t data[2];

   while (can_receive(canDev, fifo, true, &id, &ext, &rtr, &fmi, &length, (uint8_t*)data, 0) > 0)
   {
      HandleRx(id, data);
      lastRxTimestamp = rtc_get_counter_val();
   }
}

void Stm32Can::HandleTx()
{
   SENDBUFFER* b = sendBuffer; //alias

   while (sendCnt > 0 && can_transmit(canDev, b[sendCnt - 1].id, b[sendCnt - 1].id > 0x7FF, false, b[sendCnt - 1].len, (uint8_t*)b[sendCnt - 1].data) >= 0)
      sendCnt--;

   if (sendCnt == 0)
   {
      can_disable_irq(canDev, CAN_IER_TMEIE);
   }
}

/****************** Private methods and ISRs ********************/

void Stm32Can::SetFilterBank(int& idIndex, int& filterId, uint16_t* idList)
{
   can_filter_id_list_16bit_init(
         filterId,
         idList[0] << 5, //left align
         idList[1] << 5,
         idList[2] << 5,
         idList[3] << 5,
         filterId & 1,
         true);
   idIndex = 0;
   filterId++;
   idList[0] = idList[1] = idList[2] = idList[3] = 0;
}

void Stm32Can::SetFilterBank29(int& idIndex, int& filterId, uint32_t* idList)
{
   can_filter_id_list_32bit_init(
         filterId,
         (idList[0] << 3) | 0x4, //filter extended
         (idList[1] << 3) | 0x4,
         filterId & 1,
         true);
   idIndex = 0;
   filterId++;
   idList[0] = idList[1] = 0;
}

void Stm32Can::ConfigureFilters()
{
   uint16_t idList[IDS_PER_BANK] = { 0, 0, 0, 0 };
   uint32_t extIdList[EXT_IDS_PER_BANK] = { 0, 0 };
   int idIndex = 0, extIdIndex = 0;
   int filterId = canDev == CAN1 ? 0 : ((CAN_FMR(CAN2) >> 8) & 0x3F);

   for (int i = 0; i < nextUserMessageIndex; i++)
   {
      if (userIds[i] > 0x7ff)
      {
         extIdList[extIdIndex] = userIds[i];
         extIdIndex++;
      }
      else
      {
         idList[idIndex] = userIds[i];
         idIndex++;
      }

      if (idIndex == IDS_PER_BANK)
      {
         SetFilterBank(idIndex, filterId, idList);
      }
      if (extIdIndex == EXT_IDS_PER_BANK)
      {
         SetFilterBank29(extIdIndex, filterId, extIdList);
      }
   }

   //loop terminates before adding last set of filters
   if (idIndex > 0)
   {
      SetFilterBank(idIndex, filterId, idList);
   }
   if (extIdIndex > 0)
   {
      SetFilterBank29(extIdIndex, filterId, extIdList);
   }
}

/* Interrupt service routines */
extern "C" void usb_lp_can_rx0_isr(void)
{
   Stm32Can::GetInterface(0)->HandleMessage(0);
}

extern "C" void can_rx1_isr()
{
   Stm32Can::GetInterface(0)->HandleMessage(1);
}

extern "C" void usb_hp_can_tx_isr()
{
   Stm32Can::GetInterface(0)->HandleTx();
}

extern "C" void can2_rx0_isr()
{
   Stm32Can::GetInterface(1)->HandleMessage(0);
}

extern "C" void can2_rx1_isr()
{
   Stm32Can::GetInterface(1)->HandleMessage(1);
}

extern "C" void can2_tx_isr()
{
   Stm32Can::GetInterface(1)->HandleTx();
}
