# Core modules: parameters, storage, scheduler, errors, utilities

Real-world usage: [stm32-sine](https://github.com/jsphuebner/stm32-sine) (`src/stm32_sine.cpp`, `include/param_prj.h`) and [stm32-vcu](https://github.com/jsphuebner/stm32-vcu).

## params (`params.h`, `params.cpp`)

Central parameter / spot-value database. The project supplies `param_prj.h` defining `PARAM_LIST` with three entry kinds:

| Macro | Meaning |
|---|---|
| `PARAM_ENTRY(category, name, unit, min, max, def, id)` | Persistent, user-editable parameter. `id` must be unique and stable (used in flash and CAN maps) |
| `TESTP_ENTRY(...)` | Test parameter, same fields, not saved |
| `VALUE_ENTRY(name, unit, id)` | Read-only spot value |

Values are fixed point (`s32fp`, 5 fractional bits, see `my_fp`). Accessors: `Set/Get`, `GetInt/SetInt`, `GetFloat/SetFloat`, `GetBool`, `SetFixed`, `NumFromString`, `NumFromId`, `GetAttrib`, `LoadDefaults`, flags (`FLAG_HIDDEN`), `GetIdSum`. The project must implement `void Param::Change(Param::PARAM_NUM)`, called after a parameter is set.

```cpp
// param_prj.h
#define CAT_MOTOR "Motor"
#define PARAM_LIST \
    PARAM_ENTRY(CAT_MOTOR, polepairs, "",   1,   16,   4,   1) \
    PARAM_ENTRY(CAT_MOTOR, fmax,      "Hz", 0, 1000, 200,   2) \
    VALUE_ENTRY(speed,                "rpm",               100)

// main.cpp
#include "params.h"
void Param::Change(Param::PARAM_NUM p)
{
    if (p == Param::polepairs) recalcSpeedFactor();
}
...
Param::SetInt(Param::speed, rpm);
float fmax = Param::GetFloat(Param::fmax);
```

## param_save (`param_save.h`)

`parm_save()` writes all `PARAM_ENTRY` values plus a CRC to flash and returns the CRC; `parm_load()` restores them (0 on success, non-zero on error/CRC mismatch). Values are stored by id so adding parameters stays compatible.

```cpp
if (parm_load() != 0)
    Param::LoadDefaults();
...
parm_save();
```

## stm32scheduler (`stm32scheduler.h`)

Periodic tasks driven by up to 4 output-compare channels of one timer (period 1-655 ms). Timer clock and NVIC must be set up first.

```cpp
static Stm32Scheduler *scheduler;
static void Ms100Task() { /* ... */ }
static void Ms10Task()  { /* ... */ }

extern "C" void tim4_isr() { scheduler->Run(); }

Stm32Scheduler s(TIM4);
scheduler = &s;
s.AddTask(Ms10Task, 10);
s.AddTask(Ms100Task, 100);
Param::SetInt(Param::cpuload, s.GetCpuLoad());   // 0.1 %
```

## errormessage (`errormessage.h`)

Timestamped error log. The project supplies `errormessage_prj.h` with `ERROR_MESSAGE_LIST` of `ERROR_MESSAGE_ENTRY(id, type)`; `type` is `ERROR_STOP`, `ERROR_DERATE` or `ERROR_DISPLAY`. Each id yields `ERR_<id>`.

```cpp
#define ERROR_MESSAGE_LIST \
    ERROR_MESSAGE_ENTRY(OVERCURRENT, ERROR_STOP) \
    ERROR_MESSAGE_ENTRY(TEMPDERATE,  ERROR_DERATE)

ErrorMessage::SetTime(rtc_get_counter_val());
ErrorMessage::Post(ERR_OVERCURRENT);   // logged once until UnpostAll()
ErrorMessage::PrintNewErrors();        // to default terminal
```

## my_fp (`my_fp.h`)

Fixed point helpers: `FP_FROMINT`, `FP_TOINT`, `FP_FROMFLT`, `FP_TOFLOAT`, `FP_MUL`, `FP_DIV`; functions `fp_itoa`, `fp_atoi`, `fp_sqrt`, `fp_ln`, `fp_hypot2/3`. `FRAC_DIGITS` is 5.

```cpp
s32fp a = FP_FROMFLT(1.5), b = FP_FROMINT(3);
s32fp c = FP_MUL(a, b);        // 4.5
char buf[16]; fp_itoa(buf, c); // "4.50"
```

## my_math, my_string, crc8, delay, cortex, printf

* `my_math.h`: macros `ABS SIGN MIN MAX RAMPUP RAMPDOWN IIRFILTER IIRFILTERF MEDIAN3`.
  `speed = IIRFILTER(speed, newSpeed, 3);`
* `my_string.h`: libc-free `my_strcmp/strcat/strlen/strchr/strcpy/atoi/ltoa/trim`, `memcpy32/memset32`, `STRINGIFY`.
* `crc8.h`: table CRC-8, `crc8(data, len, crc)` or byte-wise `crc8(byte, crc)`.
  `uint8_t c = crc8(buf, 7, 0);`
* `delay.h`: `uDelay(us)` busy-wait calibrated for 72 MHz STM32F1.
* `cortex.h`: `cm_set_basepriority(prio)` masks interrupts at or below a priority; `CM_BASEPRI_ENABLE_INTERRUPTS` (0) unmasks.
* `printf.h`: minimal `printf`, `sprintf`, `fprintf(IPutChar*, ...)`. `printf` writes to `Terminal::defaultTerminal`. Implement `IPutChar::PutChar` for custom sinks. `debugf` is active with `T_DEBUG=1`.
  `fprintf(&canSdo, "speed=%d\r\n", rpm);`
