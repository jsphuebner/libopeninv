# I/O and communication modules

Real-world usage: [stm32-sine](https://github.com/jsphuebner/stm32-sine) (`include/digio_prj.h`, `include/anain_prj.h`, `src/stm32_sine.cpp`) and [stm32-vcu](https://github.com/jsphuebner/stm32-vcu).

## digio (`digio.h`)

Declarative digital I/O. `digio_prj.h` lists pins with `DIG_IO_ENTRY(name, port, pin, mode)`; each becomes a static `DigIo::name`. Modes: `INPUT_PD/PU/FLT` (+`_INV` for inverted logic), `INPUT_AIN`, `OUTPUT`, `OUTPUT_OD`, `OUTPUT_ALT`. Methods: `Get()`, `Set()`, `Clear()`, `Toggle()`. If a `busio.h` exists, `BUS_IO_ENTRY` adds external expander pins.

```cpp
// digio_prj.h
#define DIG_IO_LIST \
    DIG_IO_ENTRY(led_out,  GPIOC, GPIO13, PinMode::OUTPUT) \
    DIG_IO_ENTRY(start_in, GPIOB, GPIO5,  PinMode::INPUT_PD)

// init (after enabling GPIO clocks)
DIG_IO_CONFIGURE(DIG_IO_LIST)

// use
if (DigIo::start_in.Get()) DigIo::led_out.Set();
```

## anain (`anain.h`)

DMA-driven ADC sampling with median-of-3 filtering. `anain_prj.h` defines `ANA_IN_LIST` of `ANA_IN_ENTRY(name, port, pin)` and optionally `ADC_COUNT`. `ANA_IN_CONFIGURE(list)` maps channels, `AnaIn::Start()` starts conversion, `Get()` returns the 12 bit value.

```cpp
#define ANA_IN_LIST \
    ANA_IN_ENTRY(throttle, GPIOA, 0) \
    ANA_IN_ENTRY(uaux,     GPIOA, 1)

ANA_IN_CONFIGURE(ANA_IN_LIST)
AnaIn::Start();
...
uint16_t raw = AnaIn::throttle.Get();
```

## terminal (`terminal.h`)

DMA-driven USART command line. Provide a `TERM_CMD` table (`{name, handler}`, terminated by `{NULL, NULL}`) and call `Run()` from the main loop. Supports RS-485 direction pins, node-id based enable, optional fast UART, `SendBinary`. The first terminal becomes `Terminal::defaultTerminal`, the sink of `printf`.

```cpp
static void Hello(Terminal *t, char *arg) { fprintf(t, "hello %s\r\n", arg); }

static const TERM_CMD termCmds[] = {
    { "set",   TerminalCommands::ParamSet },
    { "get",   TerminalCommands::ParamGet },
    { "save",  TerminalCommands::SaveParameters },
    { "json",  TerminalCommands::PrintParamsJson },
    { "hello", Hello },
    { NULL, NULL }
};

Terminal term(USART3, termCmds);
while (1) term.Run();
```

## terminalcommands (`terminalcommands.h`)

Ready-made handlers: `ParamSet` (`set name value`), `ParamGet`, `ParamFlag`, `ParamStream` / `ParamStreamBinary` (selected values streamed), `PrintParamsJson` (full parameter database for UIs), `MapCan` (`can tx|rx name id bit len gain`), `SaveParameters`, `LoadParameters`, `Reset`. Use `SetCanMap()` for CAN commands and `Enable/DisableSaving()` to guard flash.

```cpp
TerminalCommands::SetCanMap(&cm);
// terminal: can tx speed 0x180 0 16 1
```

## linbus (`linbus.h`)

LIN master on a USART with DMA. `Request(id, data, len)` sends a header (plus payload if `len` > 0); `HasReceived(pid, len)` checks for a reply; `GetReceivedBytes()` returns it; static `Checksum` and `Parity` helpers.

```cpp
LinBus lin(USART1, 19200);
uint8_t data[8] = {0};
lin.Request(0x20, data, 0);
...
if (lin.HasReceived(LinBus::Parity(0x20), 8))
    uint8_t *rx = lin.GetReceivedBytes();
```

## stm32_loader (`stm32_loader.h`)

Data structures (`pindef`, `pincommands`) for the pin-state block stored in flash that the bootloader applies at start-up, e.g. to put power-stage pins in a safe state.
