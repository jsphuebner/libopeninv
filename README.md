# libopeninv

[![Build status](../../actions/workflows/CI-build.yml/badge.svg)](../../actions/workflows/CI-build.yml)

Generic modules for STM32 (libopencm3) firmware that can be used in many projects. Real-world applications:
[stm32-sine](https://github.com/jsphuebner/stm32-sine) (inverter) and [stm32-vcu](https://github.com/jsphuebner/stm32-vcu) (vehicle control unit).

Include as a git submodule and add `include/` and `src/` to your build. Project-specific configuration headers
(`param_prj.h`, `digio_prj.h`, `anain_prj.h`, `errormessage_prj.h`) are supplied by your project.

## Documentation

| File | Modules |
|---|---|
| [doc/core.md](doc/core.md) | `params`, `param_save`, `stm32scheduler`, `errormessage`, `my_fp`, `my_math`, `my_string`, `crc8`, `delay`, `cortex`, `printf` – parameter database with flash storage, periodic tasks, error log, fixed point and helpers |
| [doc/can.md](doc/can.md) | `canhardware`, `stm32_can`, `canmap`, `cansdo`, `sdocommands`, `canobd2` – CAN driver, user-configurable signal mapping, CANopen SDO, OBD-II |
| [doc/io.md](doc/io.md) | `digio`, `anain`, `terminal`, `terminalcommands`, `linbus`, `stm32_loader` – GPIO, ADC, serial console, LIN |
| [doc/motor.md](doc/motor.md) | `sine_core`, `fu`, `foc`, `picontroller` – sine/SVPWM generation, V/f curve, field oriented control, PI controller |

## Minimal example

```cpp
#include "params.h"
#include "param_save.h"
#include "stm32scheduler.h"
#include "stm32_can.h"
#include "canmap.h"
#include "terminal.h"
#include "terminalcommands.h"

static Stm32Scheduler *scheduler;
static CanMap *canMap;

void Param::Change(Param::PARAM_NUM) { }
extern "C" void tim4_isr() { scheduler->Run(); }

static void Ms100Task() { canMap->SendAll(); }

static const TERM_CMD termCmds[] = {
    { "set",  TerminalCommands::ParamSet },
    { "get",  TerminalCommands::ParamGet },
    { "save", TerminalCommands::SaveParameters },
    { NULL, NULL }
};

int main()
{
    // clock, GPIO and timer setup omitted
    if (parm_load() != 0) Param::LoadDefaults();

    Stm32Scheduler s(TIM4);   scheduler = &s;
    Stm32Can can(CAN1, CanHardware::Baud500);
    CanMap cm(&can);          canMap = &cm;
    Terminal term(USART3, termCmds);
    TerminalCommands::SetCanMap(&cm);

    s.AddTask(Ms100Task, 100);
    while (1) term.Run();
}
```

Host unit tests: `make -C test`.
