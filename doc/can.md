# CAN modules

Real-world usage: [stm32-sine](https://github.com/jsphuebner/stm32-sine) (`src/stm32_sine.cpp`) and [stm32-vcu](https://github.com/jsphuebner/stm32-vcu) (`src/main.cpp`).

Layering: `Stm32Can` (hardware) → callbacks `CanMap`, `CanSdo`, `CanObd2` (and your own) → `Param`.

## canhardware (`canhardware.h`)

Abstract CAN interface. `Send(id, data, len)`, `SetBaudrate()`, `AddCallback(CanCallback*)` (up to `MAX_RECV_CALLBACKS`=5), `RegisterUserMessage(id, mask)` to receive IDs for your own handler (up to `MAX_USER_MESSAGES`=30), `ClearUserMessages()`, `GetLastRxTimestamp()`. Implement `CanCallback::HandleRx` / `HandleClear` or wrap plain functions with `FunctionPointerCallback`.

`HandleClear` is called when the filter set is rebuilt; re-register user messages there.

## stm32_can (`stm32_can.h`)

libopencm3 implementation with a software send buffer (`SENDBUFFER_LEN`) and hardware filters generated from the registered IDs. Call `HandleTx()` / `HandleMessage(fifo)` from the ISRs.

```cpp
extern "C" void can1_rx0_isr() { Stm32Can::GetInterface(0)->HandleMessage(0); }
extern "C" void can1_tx_isr()  { Stm32Can::GetInterface(0)->HandleTx(); }

static bool OnRx(uint32_t id, uint32_t data[2], uint8_t dlc)
{
    if (id == 0x123) Param::SetInt(Param::torque, data[0] & 0xffff);
    return true;
}
static void OnClear() { Stm32Can::GetInterface(0)->RegisterUserMessage(0x123); }

Stm32Can can(CAN1, CanHardware::Baud500);
FunctionPointerCallback cb(OnRx, OnClear);
can.AddCallback(&cb);
can.RegisterUserMessage(0x123);

uint32_t d[2] = { 1, 2 };
can.Send(0x200, d);
```

## canmap (`canmap.h`)

User-configurable mapping of parameters/spot values to CAN signals, persisted in flash. A signal is described by CAN id, start bit, length (negative = signed), gain and optional offset. `SendAll()` transmits all mapped messages (call from a periodic task); received frames update parameters automatically. Capacity: `MAX_MESSAGES`=10, `MAX_ITEMS`=50. Define `CAN_EXT` for 29-bit ids, or OR an id with `CAN_FORCE_EXTENDED`. Errors are returned as `CAN_ERR_*`.

```cpp
CanMap cm(&can);                                    // loads saved map from flash
cm.AddSend(Param::speed,  0x180, 0,  16, 1);        // unsigned 16 bit at bit 0
cm.AddRecv(Param::torque, 0x181, 16, -16, 0.1f);    // signed 16 bit
cm.Save();

void Ms100Task() { cm.SendAll(); }
```

## cansdo (`cansdo.h`)

CANopen SDO server (plus client helpers) giving access to parameters, spot values and CAN-map configuration, as used by the openinverter web UI. It is also an `IPutChar`, so `fprintf(&sdo, ...)` streams text over SDO. Override `ProcessUserSpaceSdo()` for custom indexes; `SDOWrite/SDORead/SDOReadReply/RemoteMap` talk to other nodes; call `TriggerTimeout(freq)` periodically to flush print buffers.

```cpp
class MySdo : public CanSdo {
  public:
    using CanSdo::CanSdo;
    bool ProcessUserSpaceSdo(SdoFrame *f) override {
        if (f->index == 0x5100) { f->data = 42; SendSdoReply(f); return true; }
        return false;
    }
};
MySdo sdo(&can, &cm);
sdo.SetNodeId(1);

void Ms10Task() { sdo.TriggerTimeout(100); }
```

## sdocommands (`sdocommands.h`)

Standard command set at SDO index `0x5002` (save, load, defaults, CAN-map management). Forward to `ProcessStandardCommands(frame)` from `ProcessUserSpaceSdo`; `EnableSaving()/DisableSaving()` guard flash writes (e.g. disable while the inverter runs).

```cpp
SdoCommands::SetCanMap(&cm);
bool ProcessUserSpaceSdo(SdoFrame *f) override {
    if (f->index == SDO_INDEX_COMMANDS) { SdoCommands::ProcessStandardCommands(f); return true; }
    return false;
}
```

## canobd2 (`canobd2.h`)

Answers OBD-II requests from parameter values.

```cpp
CanObd2 obd(&can);
obd.SetNodeId(0);
```

## Tests

`test/` holds host unit tests (`make -C test`) with `stub_canhardware`, a template for mocking CAN in your own project.
