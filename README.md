# Lumon Bot Motor Driver Firmware

Firmware for an STM32G0-based closed-loop stepper motor controller. The board
receives buffered motion commands over Classic CAN, measures shaft position
with an AS5600 magnetic encoder, and generates STEP/DIR pulses for a TMC2209
stepper driver.

The project is generated from STM32CubeMX and built with CMake, Ninja, and the
GNU Arm Embedded toolchain.

## Hardware and firmware summary

| Item | Configuration |
|---|---|
| MCU | STM32G0B1CBT6, Arm Cortex-M0+, LQFP48 |
| System clock | 64 MHz from the internal HSI and PLL |
| Motor driver | TMC2209 over UART, with STEP/DIR motion control |
| Encoder | AS5600, 12-bit (4096 counts/revolution), read over I2C using DMA |
| Network | Classic CAN on FDCAN1, 500 kbit/s, standard 11-bit identifiers |
| Control loop | 1 kHz, triggered by TIM6 |
| Step generation | TIM2 channel 1 output compare, 1 MHz timer tick |
| Debug console | USART1, 115200 baud, 8-N-1 |
| TMC2209 UART | USART2, 115200 baud, 8-N-1 |

## How it works

```text
CAN COMMAND frames                         AS5600 encoder
        |                                      |
        v                                      v
position/velocity/tension queues       absolute multi-turn count
        |                                      |
        +---- consumed by CAN SYNC ------------+
                         |
                         v
              timed absolute position target
                         |
                         v
              1 kHz position controller
            feed-forward + PID correction
                         |
                         v
            acceleration and speed limits
                         |
                         v
             TIM2 STEP pulses + DIR pin
                         |
                         v
                      TMC2209
```

At startup, the firmware:

1. Initializes GPIO, DMA, ADC, FDCAN, I2C, timers, and both UARTs.
2. Reads the four board-address pins (`S1` through `S4`) to form the controller
   ID.
3. Starts the 1 kHz control timer and interrupt-driven debug UART reception.
4. Initializes the STEP/DIR generator and begins the first asynchronous AS5600
   read.
5. Configures CAN filters for the local controller ID and broadcast ID `0xF`.
6. Initializes the TMC2209, selects 1/32 microstepping, configures StallGuard,
   and checks the UART connection.
7. Enables the closed-loop motor controller at the encoder's current position.

The main loop handles debug commands, requested encoder replies, deferred fault
messages, controller telemetry, and queue statistics. Time-critical motor
control and STEP pulse scheduling run in timer interrupts.

## CAN protocol

The firmware uses Classic CAN data frames with standard 11-bit identifiers:

```text
Bits 10..8       Bits 7..4        Bits 3..0
+------------+----------------+-----------------+
| Priority   | Message type   | Destination ID  |
+------------+----------------+-----------------+
```

```c
can_id = ((priority & 0x7) << 8)
       | ((message_type & 0xF) << 4)
       |  (destination_id & 0xF);
```

Lower numeric priority values win CAN arbitration. The implemented priorities
are `VERY_HIGH` (`0`) through `VERY_LOW` (`4`). A destination of `0xF` is
broadcast; the host/controller node is ID `0`.

Multi-byte payload values are little-endian.

### Implemented messages

| Type | Value | Direction | Payload and behavior |
|---|---:|---|---|
| `EMERGENCY` | `0x0` | — | Defined but not handled in `main.c`. |
| `HEARTBEAT` | `0x1` | — | Defined but not handled in `main.c`. |
| `STATUS` | `0x2` | — | Defined but not handled in `main.c`. |
| `COMMAND` | `0x3` | Host → motor | A packed 32-bit command is decoded and appended to three queues. |
| `SYNC` | `0x4` | Host → motor | Dequeues one buffered command and applies its position over 100 ms. |
| `INIT` | `0x5` | Host → motor | Little-endian signed 16-bit absolute target in encoder counts. |
| `DEBUG` | `0x6` | Host → motor | One byte: `0x01` resets the MCU; `0x02` requests an encoder reply. |
| `ENCODER` | `0x7` | Motor → host | A 16-bit encoder response sent to host ID `0`. |

The `COMMAND` payload is packed as follows:

| Bits | Field | Decode performed by firmware |
|---|---|---|
| 11..0 | Position | Signed 12-bit value divided by `1000`; interpreted as incremental millimetres |
| 23..12 | Velocity | Signed 12-bit value divided by `100` |
| 31..24 | Tension | Unsigned 8-bit value divided by `100` |

`COMMAND` only buffers data. A later `SYNC` frame removes one item from each
queue. The position increment is converted to encoder counts using a 5 mm drum
diameter and accumulated into an absolute target:

```text
encoder counts/mm = 4096 / (pi × 5 mm) ≈ 260.76
```

The resulting target is scheduled over `0.100 s`. Velocity and tension remain
in lockstep with position in the queues, but the current `SYNC` implementation
does not yet use them for control.

An `ENCODER` reply packs the controller ID into bits 15..12 and the raw AS5600
angle into bits 11..0:

```text
encoder_reply = ((controller_id & 0xF) << 12) | (raw_angle & 0xFFF)
```

The queues hold 1024 floating-point values each. See
[`Libs/can_bus/can_bus_message_format.md`](Libs/can_bus/can_bus_message_format.md)
for the general identifier layout; the enum in
[`Libs/can_bus/can_bus.h`](Libs/can_bus/can_bus.h) is authoritative for the
message types currently compiled into the firmware.

## Closed-loop motor control

TIM6 calls `MotorController_Service_1kHz()` every millisecond. The controller:

- reads the AS5600 asynchronously through I2C DMA;
- unwraps the 12-bit angle into a signed multi-turn position;
- calculates position and speed in encoder counts;
- combines target-velocity feed-forward with a moving-average PID correction;
- converts encoder counts/s to motor steps/s;
- limits commanded acceleration and velocity; and
- sends the result to the TIM2 output-compare STEP generator.

The current application configuration in `Core/Src/main.c` is:

| Setting | Value |
|---|---:|
| Position PID (`Kp`, `Ki`, `Kd`) | `1.4`, `0`, `0` |
| Speed PID (`Kp`, `Ki`, `Kd`) | `0`, `0`, `0` |
| Feed-forward / feedback gain | `1.0` / `1.0` |
| Motor steps per encoder count | `1.5625` |
| Maximum commanded velocity | `20,000 steps/s` |
| Maximum commanded acceleration | `200,000 steps/s²` |
| Following-error limit | `8192 encoder counts` |
| PID moving-average window | `100 samples` |
| Telemetry interval | `50 ms` |

The STEP generator itself accepts `1` to `50,000` steps/s. Direction changes
stop the output-compare channel before changing the DIR pin.

The motor controller declares faults for following error, encoder jump, command
limit, encoder-not-ready, and StallGuard conditions. A fault stops step
generation and disables control until it is explicitly cleared or the board is
reset.

## Pin map

| Function | Peripheral / pin |
|---|---|
| AS5600 I2C clock / data | PB6 / PB7 (`I2C1_SCL`, `I2C1_SDA`) |
| TMC2209 UART TX / RX | PA2 / PA3 (`USART2_TX`, `USART2_RX`) |
| Debug UART TX / RX | PA9 / PA10 (`USART1_TX`, `USART1_RX`) |
| CAN RX / TX | PA11 / PA12 (`FDCAN1_RX`, `FDCAN1_TX`) |
| STEP | PA15 (`TIM2_CH1`) |
| DIR | PD1 |
| TMC2209 DIAG | PD2, rising-edge EXTI |
| TMC2209 MS2 / MS1 | PD3 / PB3 |
| Driver enable (`ENN`, active low) | PB4 |
| Controller ID S1 / S2 / S3 / S4 | PA7 / PB0 / PB2 / PB10 |
| Status LED | PA5 |
| Optional encoder ADC input | PA0 (`ADC1_IN0`) |
| SWDIO / SWCLK | PA13 / PA14 |

The controller ID is sampled once during startup:

```text
ID = (S1 << 3) | (S2 << 2) | (S3 << 1) | S4
```

All four inputs have internal pull-down resistors. Ensure the selected ID is
unique on the CAN bus. ID `0xF` should be avoided because it is reserved for
broadcast addressing.

## Debug console

Connect to USART1 at `115200 8-N-1`. `printf()` is redirected to this port.
Startup diagnostics report clocks and peripheral initialization results.
Periodic motor/encoder telemetry is enabled by default.

Single-character commands are:

| Character | Action |
|---|---|
| `R` | Reset the MCU |
| `D` | Stop pulses and disable the motor driver |
| `E` | Enable the motor driver |
| `S` | Toggle periodic motor/encoder monitor output |

The status LED toggles once per second. The queue occupancy is also printed once
per second.

## Repository layout

```text
Core/
  Inc/main.h                 Peripheral aliases and pin definitions
  Src/main.c                 Startup, callbacks, CAN command handling, main loop
Libs/
  as5600/                    DMA-based encoder driver and multi-turn unwrapping
  can_bus/                   CAN framing, filters, serialization, and protocol notes
  debug_helper/              Clock and timer diagnostic printing
  motor_controller/          1 kHz closed-loop controller and motion profile
  queue/                     Fixed-capacity float ring buffer
  stepper_oc/                TIM2 output-compare STEP/DIR generator
  tmc2209/                   TMC2209 UART register driver
  tmc_stall/                 DIAG/StallGuard integration
cmake/stm32cubemx/           CubeMX-generated build target
Lumon_Bot_MotorDriver_Firmware.ioc
                              STM32CubeMX hardware configuration
```

## Build

### Requirements

- CMake 3.22 or newer
- Ninja
- GNU Arm Embedded toolchain (`arm-none-eabi-gcc` available on `PATH`)
- An SWD programmer/debugger, such as ST-LINK, for flashing

Configure and build the Debug image from the repository root:

```sh
cmake --preset Debug
cmake --build --preset Debug
```

For an optimized image:

```sh
cmake --preset Release
cmake --build --preset Release
```

The main build artifact is:

```text
build/<preset>/Lumon_Bot_MotorDriver_Firmware.elf
```

Flash the ELF with an SWD-capable debugger or STM32CubeProgrammer. The target
and memory layout are defined by `STM32G0B1XX_FLASH.ld`.

After flashing, open the debug UART before powering the motor stage and confirm
that CAN, the AS5600, and the TMC2209 report successful initialization.

## Configuration

Application-level tuning constants are near the top of
[`Core/Src/main.c`](Core/Src/main.c). Timer, communication, and pin
configuration is maintained in
[`Lumon_Bot_MotorDriver_Firmware.ioc`](Lumon_Bot_MotorDriver_Firmware.ioc).

When regenerating code with STM32CubeMX, keep custom application code inside
the generated `USER CODE` sections. If the timer frequency, microstep setting,
motor step angle, encoder resolution, or drum diameter changes, update the
related conversion constants together.

## Current implementation notes

- `COMMAND` velocity and tension values are buffered but not applied by
  `SYNC`.
- Queue overflow return values are currently ignored; a producer that gets
  more than 1024 commands ahead of `SYNC` will lose new samples.
- The TMC2209 UART slave address is currently forced to `0`; the board's CAN
  controller ID does not select the TMC UART address.
- The DIAG interrupt marks a StallGuard event as pending, but
  `TMC_Stall_Service()` is not currently called from the main loop. Add that
  service call before relying on StallGuard shutdown.
- Some debug `printf()` calls execute from interrupt callbacks. Heavy debug
  output can increase interrupt latency and disturb motion timing.
- There is no command-timeout failsafe in the current application. Validate
  stop behavior and mechanical limits before operating unattended.
