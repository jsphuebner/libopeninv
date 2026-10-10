# libopeninv

[![Build status](../../actions/workflows/CI-build.yml/badge.svg)](../../actions/workflows/CI-build.yml)

Generic modules that can be used in many projects

## Usage example: SPI CAN controller (MCP25625T / MCP251863T)

The SPI CAN drivers `Mcp25625Can` (`mcp25625_can.h`) and `Mcp251863Can` (`mcp251863_can.h`) are
`CanHardware` implementations. Unlike the STM32 peripheral they have no interrupt vector of their own.
The chip signals events on its active low INT pin, so the project must configure a pin change interrupt
and, in the handler, receive pending messages (`HandleMessage()`, which calls `HandleRx()` of the
registered callbacks, e.g. `CanMap`/`CanSdo`) and send buffered ones (`HandleTx()`).
The following sketch is based on a typical [stm32-template](https://github.com/jsphuebner/stm32-template) project
on STM32F1. Adapt pins and clocks to your board.

`digio_prj.h` - declare the chip select and interrupt pins via the DigIo module:

```cpp
#define DIG_IO_LIST \
    DIG_IO_ENTRY(can_cs,  GPIOA, GPIO4, PinMode::OUTPUT) \
    DIG_IO_ENTRY(can_int, GPIOB, GPIO0, PinMode::INPUT_PU) \
```

`main.cpp`:

```cpp
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>
#include "digio.h"
#include "mcp25625_can.h" // or mcp251863_can.h with Mcp251863Can

static Mcp25625Can* can;

static void SpiSetup()
{
   rcc_periph_clock_enable(RCC_SPI1);
   // SCK = PA5, MOSI = PA7 (alternate function), MISO = PA6 (input)
   gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO5 | GPIO7);
   gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO6);
   // Mode 0, MSB first, 8 bit, software controlled CS (done by the driver via DigIo)
   spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_8, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                   SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
   spi_enable_software_slave_management(SPI1);
   spi_set_nss_high(SPI1);
   spi_enable(SPI1);
}

static void CanIntSetup()
{
   rcc_periph_clock_enable(RCC_AFIO); // required for EXTI pin routing
   // can_int (PB0) was already configured as pulled up input by DigIo
   exti_select_source(EXTI0, GPIOB);
   exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING); // INT is active low
   exti_enable_request(EXTI0);
   nvic_set_priority(NVIC_EXTI0_IRQ, 0xf << 4); // lowest priority
   nvic_enable_irq(NVIC_EXTI0_IRQ);
}

extern "C" void exti0_isr()
{
   exti_reset_request(EXTI0);
   // INT stays low until the chip has been serviced, so handle all pending events
   can->HandleMessage(); // receive, passes messages to HandleRx() of registered callbacks
   can->HandleTx();      // send messages from the software send buffer
}

int main()
{
   rcc_clock_setup_in_hse_8mhz_out_72mhz();
   rcc_periph_clock_enable(RCC_GPIOA);
   rcc_periph_clock_enable(RCC_GPIOB);

   DIG_IO_CONFIGURE(DIG_IO_LIST); // configures can_cs and can_int
   DigIo::can_cs.Set();           // deselect chip
   SpiSetup();

   static Mcp25625Can canDev(SPI1, DigIo::can_cs, CanHardware::Baud500);
   can = &canDev;

   // Register callbacks and user messages as with any CanHardware, e.g.
   // can->AddCallback(&canMap); can->RegisterUserMessage(0x123);

   CanIntSetup();

   while (1)
   {
      uint32_t data[2] = { 0, 0 };
      can->Send(0x100, data); // goes to the chip directly or into the send buffer
   }
}
```

Notes:
- `Send()` falls back to the send buffer only when all chip transmit buffers are busy; `HandleTx()` in the
  interrupt handler drains it once the chip signals a free buffer (the INT pin is also asserted on TX complete).
- The handler performs SPI transfers, so give the EXTI interrupt the lowest priority and do not call `Send()`
  from interrupts of lower priority than it without masking (see `CAN_MAX_IRQ_PRIORITY` handling in the drivers).
- If an edge was missed while the line is already low, call `can->HandleMessage()` and `can->HandleTx()` once after setup.
