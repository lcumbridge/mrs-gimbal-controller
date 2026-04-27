# mrs-gimbal-controller

Dual-axis BLDC gimbal firmware for the MicroRobo Systems GPS beacon tracking system.
Runs on a Teensy 4.1, drives two GBM5208 brushless gimbal motors via SimpleFOC v2.4.0,
and receives angle commands from the ground station over USB Serial at up to 200 Hz.

> **Part of a three-component system.**
> See also:
> [`mrs-airborne-tracker`](https://github.com/lcumbridge/mrs-airborne-tracker) —
> airborne EKF and telemetry firmware |
> [`mrs-ground-station`](https://github.com/lcumbridge/mrs-ground-station) —
> Python ground station, EKF tracker, and VISCA camera control

---

## Table of Contents

1. [System Context](#system-context)
2. [Hardware Requirements](#hardware-requirements)
3. [Wiring Summary](#wiring-summary)
4. [Dependencies](#dependencies)
5. [Building and Flashing](#building-and-flashing)
6. [First-Install Calibration](#first-install-calibration)
7. [Configuration](#configuration)
8. [Command and Telemetry Interface](#command-and-telemetry-interface)
9. [Architecture and Key Design Decisions](#architecture-and-key-design-decisions)
10. [Known Limitations](#known-limitations)
11. [Performance Characterization](#performance-characterization)
12. [License](#license)
13. [Citation](#citation)

---

## System Context

The ground station computes azimuth and elevation angles to the tracked target,
applies lead/prediction compensation, and sends angle commands to this firmware
over a USB Serial connection. This firmware executes field-oriented control on
both gimbal axes and streams feedback telemetry back to the ground station at
200 Hz. VISCA zoom camera control is handled entirely in the ground station —
this firmware is motor control only.

```
Ground Station PC
  │
  │  USB Serial (115200 baud)
  │  E<rad>\n  A<rad>\n commands → (200 Hz)
  │  ← 17-byte binary feedback frames (200 Hz)
  │
[Teensy 4.1]
  ├── SPI1 → AS5048A encoder (elevation)
  ├── SPI0 → AS5048A encoder (azimuth)
  ├── PWM  → SimpleFOC shield → GBM5208 (elevation)
  └── PWM  → SimpleFOC shield → GBM5208 (azimuth)
```

---

## Hardware Requirements

| Component | Part | Qty | Notes |
|---|---|---|---|
| Flight computer | PJRC Teensy 4.1 | 1 | 600 MHz Cortex-M7 |
| Motor driver (elevation) | SimpleFOC Shield v2.0.4 (L6234) | 1 | Wired to Teensy headers |
| Motor driver (azimuth) | SimpleFOC Shield v2.0.4 (L6234) | 1 | Wired to Teensy headers |
| Gimbal motor (elevation) | iPower GBM5208 | 1 | 11 pole pairs, 12 V, R > 10 Ω |
| Gimbal motor (azimuth) | iPower GBM5208 | 1 | 11 pole pairs, 12 V, R > 10 Ω |
| Encoder (elevation) | AMS AS5048A | 1 | 14-bit SPI, SPI1 bus |
| Encoder (azimuth) | AMS AS5048A | 1 | 14-bit SPI, SPI0 bus |
| Power supply | 12 V DC, ≥ 3 A | 1 | Shared between both shields |

> **Motor compatibility note:** The SimpleFOC Shield v2.0.4 is rated for gimbal
> motors with phase resistance R > 10 Ω and absolute maximum current of 5 A.
> The GBM5208 (R ≈ 14 Ω) is within spec. Do not substitute low-resistance BLDC
> motors without reducing `voltage_limit` accordingly.

---

## Wiring Summary

> Full schematic available in [`/docs/Gimbal_motor_driver.pdf`](docs/Gimbal_motor_driver.pdf).

### SPI1 — Elevation Encoder (AS5048A)

| Signal | Teensy 4.1 Pin | Notes |
|---|---|---|
| MOSI | 26 | Configured explicitly via `SPI1.setMOSI(26)` |
| MISO | 1  | Configured explicitly via `SPI1.setMISO(1)` |
| SCK  | 27 | Configured explicitly via `SPI1.setSCK(27)` |
| CS   | 0  | Elevation encoder chip select |
| GND  | GND | |
| 3.3 V | 3.3 V | AS5048A supply |

### SPI0 — Azimuth Encoder (AS5048A)

| Signal | Teensy 4.1 Pin | Notes |
|---|---|---|
| MOSI | 11 | Teensy 4.1 default SPI (SPI0) |
| MISO | 12 | |
| SCK  | 13 | |
| CS   | 10 | Azimuth encoder chip select |
| GND  | GND | |
| 3.3 V | 3.3 V | AS5048A supply |

### PWM — Elevation Driver (SimpleFOC Shield, L6234)

| Signal | Teensy 4.1 Pin | SimpleFOC Shield Terminal |
|---|---|---|
| INA | 9  | Phase A PWM |
| INB | 5  | Phase B PWM |
| INC | 6  | Phase C PWM |
| EN  | 8  | Driver enable |

### PWM — Azimuth Driver (SimpleFOC Shield, L6234)

| Signal | Teensy 4.1 Pin | SimpleFOC Shield Terminal |
|---|---|---|
| INA | 33 | Phase A PWM |
| INB | 36 | Phase B PWM |
| INC | 37 | Phase C PWM |
| EN  | 7  | Driver enable |

### USB — Ground Station Connection

The Teensy 4.1 USB port connects directly to the ground station PC.
`Serial` on Teensy maps to USB CDC — no USB-to-Serial adapter is needed.
The ground station opens this port at 115200 baud.

---

## Dependencies

| Library | Version | Source |
|---|---|---|
| SimpleFOC | **2.4.0** | Arduino Library Manager / [GitHub](https://github.com/simplefoc/Arduino-FOC) |
| Teensyduino | 1.59 | [pjrc.com](https://www.pjrc.com/teensy/td_download.html) |

> **Version requirement:** `angle_nocascade` control mode (used for the azimuth
> axis) was introduced in SimpleFOC v2.3.x. Do not use v2.2.x — the firmware
> will not compile.

---

## Building and Flashing

### PlatformIO (recommended)

```bash
git clone https://github.com/lcumbridge/mrs-gimbal-controller.git
cd mrs-gimbal-controller
pio run --target upload
```

### Arduino IDE

1. Install Teensyduino 1.59 from [pjrc.com](https://www.pjrc.com/teensy/td_download.html).
2. Install **SimpleFOC v2.4.0** via **Tools → Manage Libraries**.
3. Open `GBM5208_GIMBAL_DualAxis.ino`.
4. Select **Teensy 4.1** under **Tools → Board**.
5. Set **Tools → CPU Speed** to **600 MHz**.
6. Click **Upload**.

---

## First-Install Calibration

**Both axes require calibration before first use.** The values currently in
`setup()` are specific to the original hardware build. After any motor or encoder
replacement, recalibrate both affected axes.

### What needs calibrating

Each call to `initAxis()` takes two hardware-specific values:

| Parameter | What it is | How to obtain |
|---|---|---|
| `zero_electric_angle` | Electrical zero of the motor at encoder position 0 | Run SimpleFOC `find_pole_pairs_number()` example; electrical zero is printed to Serial during `initFOC()` |
| `sensor_offset` | Mechanical offset so the motor homes to the desired physical position | Set `sensor_offset = 0`, power up, observe `shaft_angle` at the desired home position, negate that value |

### Pole pair verification

Both GBM5208 motors are datasheet-specified and bench-verified at 11 pole pairs.
If you substitute a different motor, run SimpleFOC's `find_pole_pairs_number()`
utility before setting the `BLDCMotor(N)` constructor argument. An incorrect pole
pair count produces a 180° position error at certain commanded angles and prevents
the motor from holding position stably.

### Calibration procedure (per axis)

1. Set `zero_electric_angle = 0` and `sensor_offset = 0` in `setup()`.
2. Upload and open Serial Monitor at 115200 baud.
3. During `initFOC()`, SimpleFOC prints the detected electrical zero. Note this value.
4. Update `zero_electric_angle` with the printed value.
5. Upload again. Observe `shaft_angle` printed to Serial at the desired home position.
6. Set `sensor_offset = -shaft_angle` (negate).
7. Upload and verify the motor homes correctly.

---

## Configuration

The following constants at the top of `GBM5208_GIMBAL_DualAxis.ino` are the
primary tuning points. PID parameters are set inside `initAxis()` with per-axis
overrides applied in `setup()`.

### Fault thresholds

| Constant | Default | Description |
|---|---|---|
| `MAX_SAFE_VELOCITY` | `15.0` rad/s | Velocity above which the runaway watchdog arms |
| `INSTANT_CLAMP_VEL` | `40.0` rad/s | Velocity above which an instant glitch hold triggers |
| `SAFE_RESUME_VEL` | `3.0` rad/s | Velocity below which a glitch hold clears |
| `MIN_RUNAWAY_TRAVEL` | `1.0` rad | Minimum travel to classify a fault as genuine runaway vs. sensor glitch |
| `RUNAWAY_TIMEOUT_MS` | `500` ms | Time above `MAX_SAFE_VELOCITY` before fault action |
| `GLITCH_HOLD_MS` | `2000` ms | Duration of each glitch hold extension |
| `ABSOLUTE_HALT_MS` | `30000` ms | Maximum time in glitch hold before permanent halt |

### PID parameters (working values)

Both axes share the `initAxis()` defaults; per-axis overrides are applied in `setup()`.

| Parameter | Elevation | Azimuth | Notes |
|---|---|---|---|
| Control mode | `angle` (cascade) | `angle_nocascade` (direct V) | See architecture section |
| `PID_velocity.P` | 0.5 | — | Not active in `angle_nocascade` |
| `PID_velocity.I` | 8.0 | — | Holds camera weight against gravity |
| `PID_velocity.D` | 0.0 | 0.0 | Zeroed — AS5048A quantization noise |
| `PID_velocity.output_ramp` | 1000 | — | rad/s² |
| `P_angle.P` | 15.0 | 30.0 | V/rad (azimuth); rad/s/rad (elevation) |
| `P_angle.I` | — | 0.5 | V·s/rad — steady-state offset correction |
| `P_angle.D` | — | 2.5 | V·s/rad — arrival damping |
| `P_angle.output_ramp` | 300 | 150 | Limits demand rate |
| `LPF_velocity.Tf` | 0.01 s | — | Velocity LPF time constant |
| `LPF_angle.Tf` | — | 0.010 s | Angle LPF time constant |
| `voltage_limit` | 8.0 V | 11.0 V | Azimuth raised for direct-V headroom |
| `velocity_limit` | 8.0 rad/s | — | |

### Notch filters

| Instance | Axis | Notch freq | Bandwidth | Notes |
|---|---|---|---|---|
| `notchE` | Elevation shaft_angle | 4.8 Hz | 1.0 Hz | Gimbal structural resonance |
| `notchE_vel` | Elevation shaft_velocity | 4.9 Hz | 1.0 Hz | |
| `notchA` | Azimuth shaft_angle | 4.9 Hz | 1.0 Hz | |

---

## Command and Telemetry Interface

### Commands (Ground Station → Gimbal)

USB Serial, 115200 baud, ASCII line protocol:

| Format | Example | Description |
|---|---|---|
| `E<radians>\n` | `E1.5708\n` | Set elevation target angle |
| `A<radians>\n` | `A-0.3490\n` | Set azimuth target angle |

The parser (`handleSerial()`) is non-blocking. Commands arriving between FOC ticks
are buffered and applied on the next tick. The ground station sends at up to 200 Hz;
at 115200 baud a 10-byte command takes ~0.87 ms so the link is not the bottleneck.

### Feedback Telemetry (Gimbal → Ground Station)

Binary frame, 17 bytes, transmitted every 10 FOC ticks (~200 Hz):

| Offset | Size | Type | Field | Description |
|---|---|---|---|---|
| 0 | 1 | `uint8_t` | `header` | `0x46` (`'F'`) — frame sync byte |
| 1 | 4 | `uint32_t` | `timestamp_us` | `micros()` at frame assembly |
| 5 | 4 | `float` | `angle_E` | Elevation shaft angle (rad) |
| 9 | 4 | `float` | `angle_A` | Azimuth shaft angle (rad) |
| 13 | 4 | `float` | `vq_E` | Elevation quadrature voltage Vq (V) |
| 17 | 4 | `float` | `vq_A` | Azimuth quadrature voltage Vq (V) |

All multi-byte fields are little-endian. Floats are IEEE 754 single precision.
`Vq` is the torque-producing voltage component from the FOC output — useful for
diagnosing motor load, cogging, and saturation in the ground station.

---

## Architecture and Key Design Decisions

### Two different control modes

The elevation and azimuth axes use different SimpleFOC control modes because their
mechanical loads are fundamentally different.

**Elevation** uses `MotionControlType::angle` (cascaded). The angle loop outputs a
velocity demand to an inner velocity PID, which outputs voltage. The integrator
(`PID_velocity.I = 8.0`) winds up over approximately one second to hold the camera
weight against gravity without a continuous position error. This is the standard
SimpleFOC angle control path.

**Azimuth** uses `MotionControlType::angle_nocascade` (added in SimpleFOC v2.3.x).
The angle PID outputs voltage directly, bypassing the inner velocity loop. This was
chosen for the azimuth axis because the bearing friction and cogging torque are
low and roughly constant — the inner velocity loop added unnecessary phase lag for
this load profile. The full PID on `P_angle` (P + I + D) compensates for
steady-state offset and damps arrival.

### PWM frequency — 25 kHz

Both drivers are set to 25 kHz via `d.pwm_frequency = 25000` in `initAxis()`.
This moves switching noise above the audible range. The default SimpleFOC PWM
frequency for Teensy is lower and produces an audible whine through the gimbal
structure at the frequencies relevant to camera audio.

### Center-aligned 3PWM mode excluded

`SIMPLEFOC_TEENSY4_FORCE_CENTER_ALIGNED_3PWM` is deliberately not defined. Both
drivers share FlexPWM2 (elevation on submodules 1 and 2, azimuth on submodules 0
and 3). The center-aligned initialisation sequence writes to module-level MCTRL
and SYNC registers that are shared across submodules, causing the second driver
init to corrupt the phase alignment already established on the first. Edge-aligned
fast PWM operates independently per submodule and avoids this collision.

### PID_velocity.D = 0 on both axes

The AS5048A produces 14-bit CORDIC output with approximately 0.02° quantization.
At the 2 kHz FOC loop rate, consecutive angle samples frequently differ by exactly
one LSB, producing a rectangular velocity signal. The D term differentiates this
signal, producing high-frequency torque commands that cause audible torque reversals.
D is zeroed on both axes; the velocity LPF (`LPF_velocity.Tf`) smooths the velocity
estimate instead.

### Loop rate cap — 2 kHz

`loop()` gates FOC execution to one call per 500 µs. The AS5048A's internal CORDIC
pipeline updates at approximately 1 kHz. Running the FOC loop faster causes repeated
identical angle readings; the velocity LPF interprets these as a burst of zero
velocity followed by a velocity spike, producing oscillatory torque output. The 2 kHz
cap ensures at most one repeated reading per two FOC ticks, which the LPF handles
cleanly.

### Notch filters as post-loopFOC() processing

The second-order IIR notch filters write directly to `motorE.shaft_angle`,
`motorE.shaft_velocity`, and `motorA.shaft_angle` after `loopFOC()` returns.
This is an intentional deviation from standard SimpleFOC usage — these fields
are normally read-only outputs of the sensor update inside `loopFOC()`. It is
valid in this context because the notch output is numerically close to the input
and SimpleFOC re-reads the sensor at the start of the next `loopFOC()` call,
overwriting these fields. The practical effect is that the PID controllers see
notch-filtered feedback while the sensor state remains unaffected.

### sensor_offset applied after initFOC()

`motor.sensor_offset` is assigned after `motor.initFOC()` in `initAxis()`.
`initFOC()` uses the raw sensor reading to determine the electrical zero of the
motor. Applying `sensor_offset` before `initFOC()` would shift the angle
reference used for electrical zero calibration, producing incorrect commutation.

---

## Known Limitations

**These are documented honestly. This is a prototype system in active development.**

1. **Calibration required on hardware replacement.** `zero_electric_angle` and
   `sensor_offset` are specific to the current motor and encoder assembly. Any
   motor or encoder swap requires recalibration of that axis before operation.
   See [First-Install Calibration](#first-install-calibration).

2. **Fault halt stops both axes.** The `while(1)` inside `runFaultLogic()` halts
   the entire Teensy, disabling both axes regardless of which one faulted. For
   applications requiring independent axis fault isolation, replace `while(1)` with
   a per-axis `m.disable()` call and a permanent `disabled` flag checked in `loop()`.

3. **`rampTarget()` is not active.** The loop calls `wrapNearest()` and relies on
   `motor.velocity_limit` for slew limiting. `rampTarget()` provides finer per-tick
   slew control and is retained in the source for reference.

4. **200 Hz command rate — lurching risk.** At 200 Hz, each command step is
   approximately 5 ms × target angular velocity. At high tracking rates the step
   size can exceed `SLEW_RATE` (0.02 rad), causing visible step motion rather than
   smooth tracking. The ground station's critically damped filter reduces this in
   practice but the interaction between command rate, step size, and velocity limit
   requires field tuning. This is an active area.

5. **Single-frequency gimbal resonance.** The ~4.8–4.9 Hz notch frequency was
   tuned for the current gimbal mechanical assembly. Changes to camera weight,
   mounting stiffness, or gimbal geometry will shift this frequency and may require
   retuning the notch filter parameters.

6. **No current sensing.** The SimpleFOC Shield v2.0.4 does not expose current
   sense shunts in this configuration. Torque control and motor protection rely
   entirely on voltage limiting. Overcurrent protection depends on the power supply's
   current limiting behaviour.

---

## Performance Characterization

**Environment:** Bench testing, gimbal fully assembled with camera payload, 12 V supply.
Step response data from plant characterization runs on 2026-04-24 (elevation) and
2026-04-23 (azimuth). Steady-state metrics computed over flat-hold intervals,
excluding 500 ms windows around each step transition.

### Step Response

| Metric | Elevation | Azimuth | Notes |
|---|---|---|---|
| Step size | 8.6° | 11.5° | Commanded step amplitude |
| Rise time (10–90%) | 0.120 s | 0.160 s | |
| Overshoot | 5.6% | 0.0% | |
| Damping ratio ζ | 0.655 | ~0.757 | From Bode fit |
| Natural frequency ωn | 3.81 rad/s | ~10.64 rad/s | |
| Transient peak error | 9.4° | 11.8° | At step edge |

### Steady-State Hold (between steps, camera loaded)

| Metric | Elevation | Azimuth | Notes |
|---|---|---|---|
| RMS error | **0.46°** | **0.47°** | Primary position hold metric |
| Mean error | −0.08° | −0.08° | Near-zero: integrator working |
| 95th percentile error | 1.03° | 0.91° | |
| Peak steady-state error | 2.26° | 0.98° | Outlier spikes |
| Mean Vq (torque) | 0.46 V | −0.01 V | Elevation non-zero: gravity load |

### Frequency Domain

| Metric | Elevation | Azimuth |
|---|---|---|
| Closed-loop bandwidth (−3 dB) | 0.53 Hz | 1.65 Hz |
| Phase crossover frequency | 0.40 Hz | 0.05 Hz |
| Structural resonance | 4.9 Hz, 34 dB | — |

### System Timing

| Metric | Value | Notes |
|---|---|---|
| FOC loop rate | 2 kHz | 500 µs gate in `loop()` |
| Feedback telemetry rate | 200 Hz | Every 10 FOC ticks |
| Fault detection latency | ≤ 500 µs | Tier 2 instant clamp |

Performance under dynamic tracking loads (moving target, ground station commanding
at 200 Hz) has not been systematically characterised.

---

## License

This project is released under the
[PolyForm Noncommercial License 1.0.0](LICENSE).

You are free to use, study, and modify this software for **noncommercial purposes**
with attribution. The required attribution notice is:

> Copyright (c) 2025 MicroRobo Systems LLC (https://microrobosys.com)

**Commercial use requires a separate license from MicroRobo Systems LLC.**
Contact us via [microrobosys.com](https://microrobosys.com).

---

## Citation

```
Cumbridge, L. (2025). mrs-gimbal-controller: Dual-axis BLDC gimbal firmware
for GPS beacon tracking. MicroRobo Systems LLC.
https://github.com/lcumbridge/mrs-gimbal-controller
```

```bibtex
@software{cumbridge2025gimbal,
  author  = {Cumbridge, Leonard},
  title   = {mrs-gimbal-controller: Dual-axis {BLDC} gimbal firmware for {GPS} beacon tracking},
  year    = {2025},
  url     = {https://github.com/lcumbridge/mrs-gimbal-controller},
  note    = {MicroRobo Systems LLC}
}
```
