# On-Charger Coordination

Embedded firmware for EV charging optimization on STM32G4. Runs a time-step simulation over multiple days, calling an optimization cycle each step (base load, LMP price, PV, and EV data from `data.c`). Communication and state handling are done via USART and a state machine.

**Target:** STM32G474CE (Keil MDK-ARM, HAL).

---

## Project structure

```
On-Charger_Coordination/
├── HAL_lib/          STM32G4 HAL drivers (Inc/, Src/)
├── MDK/              CMSIS headers, startup, core_cm4.h
├── mycode/            Application and drivers
│   ├── ADC, Delay, HRTIM, RGB, clock_config   Hardware / timing
│   ├── Optimization, data, state_machine      Optimization and state
│   ├── oled, oledfont                         OLED display
│   ├── type3, usart                           Type3 / serial
│   ├── common.h, Config.h                     Shared config
│   └── Int.c, int.h                           Peripheral init
├── User_code/        Entry and HAL integration
│   ├── main.c, main.h
│   ├── G4.uvprojx, G4.uvoptx                  Keil project
│   ├── stm32g4xx_*.c/h                        HAL config, IRQ, system
│   └── stm32g4xx.h, stm32g474xx.h, ...       Device headers
```

Build: open `User_code/G4.uvprojx` in Keil and compile.
