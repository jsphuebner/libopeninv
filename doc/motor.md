# Motor control modules

Real-world usage: [stm32-sine](https://github.com/jsphuebner/stm32-sine) (`src/pwmgeneration-*.cpp`, `src/throttle.cpp`).

## sine_core (`sine_core.h`)

Lookup-table sine generation and space-vector PWM. Angles are 16 bit (0..65535 = 0..360°).

* `Sine(angle)`, `Cosine(angle)`, `Atan2(cos, sin)`
* `SetAmp(amp)` / `GetAmp()`: amplitude in digits (max `MAXAMP`)
* `Calc(angle)` fills `DutyCycles[3]` (with SVPWM offset); `CalcSVPWMOffset(a,b,c)`

```cpp
SineCore::SetAmp(amp);
SineCore::Calc(angle);
timer_set_oc_value(PWM_TIMER, TIM_OC1, SineCore::DutyCycles[0]);
timer_set_oc_value(PWM_TIMER, TIM_OC2, SineCore::DutyCycles[1]);
timer_set_oc_value(PWM_TIMER, TIM_OC3, SineCore::DutyCycles[2]);
```

## fu (`fu.h`): V/f curve

`MotorVoltage` computes the voltage amplitude for a frequency: boost at 0 Hz, linear rise up to the field weakening frequency, then constant.

```cpp
MotorVoltage::SetBoost(Param::GetInt(Param::boost));
MotorVoltage::SetWeakeningFrq(Param::GetFloat(Param::fweak));
MotorVoltage::SetMaxAmp(SineCore::MAXAMP);
uint32_t amp = MotorVoltage::GetAmp(frq);                      // frq: fixed point Hz
uint32_t half = MotorVoltage::GetAmpPerc(frq, FP_FROMINT(50)); // 50 %
```

## foc (`foc.h`)

Field oriented control: Clarke/Park transforms, inverse transform to duty cycles, MTPA, voltage limiting.

* `SetAngle(angle)`, then `ParkClarke(il1, il2)` → `FOC::id`, `FOC::iq`
* `GetQLimit(maxVd)`, `GetTotalVoltage(ud, uq)`
* `InvParkClarke(ud, uq)` → `FOC::DutyCycles[3]`
* `Mtpa(is, idref, iqref)`, `SetMotorParameters(lqMinusLd, fluxLinkage)`
* `Get/SetMaximumModulationIndex()`

```cpp
static PiController idCtrl, iqCtrl;
idCtrl.SetGains(kp, ki);
idCtrl.SetCallingFrequency(PWM_FREQ);
idCtrl.SetMinMaxY(-maxVd, maxVd);

// in the PWM interrupt
FOC::SetAngle(angle);
FOC::ParkClarke(il1, il2);
float idref, iqref;
FOC::Mtpa(isRef, idref, iqref);
idCtrl.SetRef(FP_FROMFLT(idref));
iqCtrl.SetRef(FP_FROMFLT(iqref));
int32_t ud = idCtrl.Run(FOC::id);
int32_t uq = iqCtrl.Run(FOC::iq);
FOC::InvParkClarke(ud, uq);
timer_set_oc_value(PWM_TIMER, TIM_OC1, FOC::DutyCycles[0]);
```

## picontroller (`picontroller.h`)

Template PI controller with anti-windup and feed-forward; typedefs `PiController` (fixed point) and `PiControllerFloat`. Configure with `SetGains`, `SetCallingFrequency`, `SetMinMaxY`, `SetRef`, then call `Run(measured, feedForward)` at the configured rate. Also `RunProportionalOnly`, `ResetIntegrator`, `PreloadIntegrator`.

```cpp
PiControllerFloat speedCtrl;
speedCtrl.SetGains(0.5f, 0.1f);
speedCtrl.SetCallingFrequency(100);
speedCtrl.SetMinMaxY(0, 100);
speedCtrl.SetRef(targetRpm);
float torque = speedCtrl.Run(measuredRpm);
```
