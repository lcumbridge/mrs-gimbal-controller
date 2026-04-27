// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Copyright (c) 2025 MicroRobo Systems LLC (https://microrobosys.com)
//
// =============================================================================
// GBM5208_GIMBAL_DualAxis.ino — Dual-axis BLDC gimbal controller
// =============================================================================
//
// Drives elevation (E) and azimuth (A) axes of a two-axis camera gimbal.
// Each axis uses a GBM5208 brushless gimbal motor (11 pole pairs, 12 V),
// an AS5048A magnetic encoder, and an L6234-based SimpleFOC shield.
//
// HARDWARE:
//   Flight computer : PJRC Teensy 4.1
//   Library         : SimpleFOC v2.4.0
//
// ELEVATION AXIS (E):
//   Driver  : BLDCDriver3PWM(9, 5, 6, 8)   — INA=9  INB=5  INC=6  EN=8
//   Sensor  : AS5048A on SPI1, CS=0
//             SPI1 pins: MOSI=26, MISO=1, SCK=27
//   Control : Cascaded angle → velocity → voltage (MotionControlType::angle)
//   Command : E<radians>\n  e.g. "E1.57\n"
//
// AZIMUTH AXIS (A):
//   Driver  : BLDCDriver3PWM(33, 36, 37, 7) — INA=33 INB=36 INC=37 EN=7
//   Sensor  : AS5048A on SPI0, CS=10
//             SPI0 pins: MOSI=11, MISO=12, SCK=13 (Teensy 4.1 default SPI)
//   Control : Direct-voltage angle control (MotionControlType::angle_nocascade)
//             P_angle outputs voltage directly; no inner velocity PID loop.
//   Command : A<radians>\n  e.g. "A0.00\n"
//
// COMMAND INTERFACE:
//   USB Serial (115200 baud).  Ground station sends E and A commands at up to
//   200 Hz.  The serial parser in handleSerial() is non-blocking.
//
// FEEDBACK TELEMETRY:
//   A 17-byte binary frame is written to Serial every 10 FOC ticks (~200 Hz):
//     Byte 0     : 0x46 ('F') header
//     Bytes 1–4  : timestamp_us (uint32, µs since boot)
//     Bytes 5–8  : angle_E     (float, rad)
//     Bytes 9–12 : angle_A     (float, rad)
//     Bytes 13–16: vq_E        (float, V)  — quadrature voltage, elevation
//     Bytes 17–20: vq_A        (float, V)  — quadrature voltage, azimuth
//
// PWM FREQUENCY:
//   Both drivers run at 25 kHz to eliminate audible switching noise from the
//   gimbal structure.  The center-aligned 3PWM mode
//   (SIMPLEFOC_TEENSY4_FORCE_CENTER_ALIGNED_3PWM) is intentionally NOT used —
//   both drivers occupy FlexPWM2 submodules (E: 1,2 / A: 0,3) and the
//   center-aligned init sequence collides on module-level MCTRL/SYNC registers,
//   corrupting phase alignment on motorE.  Edge-aligned fast PWM avoids this.
//
// PID_velocity.D = 0 ON BOTH AXES:
//   The AS5048A's 14-bit CORDIC output has ~0.02° quantization. At the 2 kHz
//   FOC loop rate, this quantization appears as high-frequency velocity noise
//   that D amplifies directly into torque reversals.  D is zeroed on both axes.
//
// LOOP RATE:
//   Capped at 2 kHz (500 µs gate in loop()).  The AS5048A's internal CORDIC
//   pipeline refreshes at ~1 kHz; running the FOC loop faster causes repeated
//   identical angle readings which the velocity LPF misinterprets as burst
//   noise, driving unwanted torque oscillation.
//
// FAULT / RUNAWAY RECOVERY:
//   runFaultLogic() implements a three-tier fault state machine per axis:
//     Tier 1 — Instant clamp:  |velocity| > INSTANT_CLAMP_VEL → glitch hold
//     Tier 2 — Runaway watchdog: |velocity| > MAX_SAFE_VELOCITY for
//              RUNAWAY_TIMEOUT_MS, with position travel > MIN_RUNAWAY_TRAVEL
//              → genuine runaway → disable, 2 s pause, resume at current angle
//     Tier 3 — Repeated faults: > 3 runaways or > 10 spikes → permanent halt
//   Both axes halt on a per-axis while(1).  See FAULT BEHAVIOUR NOTE below.
//
// FAULT BEHAVIOUR NOTE:
//   while(1) in runFaultLogic() halts the entire Teensy, stopping both axes.
//   If independent per-axis fault isolation is required, replace while(1)
//   with per-axis m.disable() and a permanent disabled flag checked in loop().
//
// NOTCH FILTERS:
//   Second-order IIR notch filters suppress a ~4.8–4.9 Hz mechanical resonance
//   in the gimbal structure.  The filters post-process SimpleFOC's shaft_angle
//   and shaft_velocity fields directly after loopFOC().  This is an intentional
//   deviation from standard SimpleFOC usage — shaft_angle and shaft_velocity
//   are normally read-only outputs of loopFOC().  It is valid here because the
//   notch output is numerically close to the input and the FOC state machine
//   re-reads sensor position at the start of the next loopFOC() call.
//
// SENSOR_OFFSET SEQUENCING:
//   motor.sensor_offset is assigned AFTER motor.initFOC() inside initAxis().
//   This is required — initFOC() uses the raw sensor reading to determine
//   electrical zero, and applying sensor_offset before that call would bias
//   the electrical angle calibration.
//
// KNOWN LIMITATIONS:
//   - Azimuth pole pair count (11) is based on GBM5208 datasheet and confirmed
//     at bench; full find_pole_pairs_number() sweep should be run on hardware
//     replacement to verify.
//   - rampTarget() is retained but not called — see note on that function.
//   - Motor A calibration values (zero_electric_angle=4.13, sensor_offset=-0.50)
//     are hardware-specific.  Recalibrate after any motor or encoder replacement.
// =============================================================================

#define SIMPLEFOC_DISABLE_DEBUG
#include <SimpleFOC.h>

// ---------------------------------------------------------------------------
// Watchdog and fault threshold constants
// ---------------------------------------------------------------------------
const float          MAX_SAFE_VELOCITY  = 15.0f;   // rad/s — runaway watchdog threshold
const float          INSTANT_CLAMP_VEL = 40.0f;   // rad/s — instant spike clamp
const float          SAFE_RESUME_VEL   = 3.0f;    // rad/s — velocity below which glitch hold clears
const float          SLEW_RATE         = 0.02f;   // rad/FOC-tick — rampTarget() step size (unused in loop)
const float          MIN_RUNAWAY_TRAVEL = 1.0f;   // rad — minimum travel to distinguish runaway from glitch
const float          NORMAL_VOLT_LIMIT = 8.0f;    // V — normal operating voltage limit
const unsigned long  RUNAWAY_TIMEOUT_MS = 500UL;  // ms — time above MAX_SAFE_VELOCITY before action
const unsigned long  GLITCH_HOLD_MS    = 2000UL;  // ms — per-extension duration of glitch hold
const unsigned long  ABSOLUTE_HALT_MS  = 30000UL; // ms — maximum time in glitch hold before permanent halt

// ---------------------------------------------------------------------------
// Per-axis fault state
// ---------------------------------------------------------------------------
#define FAULT_BUF_SIZE 8

struct FaultSample {
    float vel;
    float angle;
    float vq;
};

struct AxisState {
    const char*   name;               // "E" or "A" — used in fault messages
    float         commanded_angle;    // most recent angle command from ground station (rad)
    unsigned long last_safe_time;     // millis() of last loop iteration below MAX_SAFE_VELOCITY
    float         runaway_start_angle;// shaft_angle when runaway watchdog armed
    bool          runaway_tracking;   // true while above MAX_SAFE_VELOCITY, watchdog running
    bool          glitch_hold_active; // true while motor is disabled in glitch hold
    unsigned long glitch_hold_start;  // millis() of last hold extension start
    unsigned long glitch_total_start; // millis() of first hold entry (absolute timeout reference)
    uint8_t       hold_extensions;    // number of times GLITCH_HOLD_MS has been extended
    uint8_t       spike_count;        // total instant-clamp events since boot
    uint8_t       glitch_count;       // total glitch-hold entries since boot
    uint8_t       fault_count;        // total genuine runaway events since boot
    FaultSample   buf[FAULT_BUF_SIZE];// circular pre-fault telemetry buffer
    uint8_t       buf_head;           // write index into buf[]
};

// ---------------------------------------------------------------------------
// Second-order IIR notch filter
//
// Suppresses the ~4.8–4.9 Hz mechanical resonance observed in the gimbal
// structure under 2 kHz FOC loop operation.  Coefficients are computed once
// in init() from the notch frequency, sample rate, and bandwidth.
//
// Applied post-loopFOC() to shaft_angle and shaft_velocity — see file header.
// ---------------------------------------------------------------------------
struct NotchFilter {
    float f0, fs, bw;
    float x1, x2, y1, y2;
    float b0, b1, b2, a1, a2;

    void init(float f0_hz, float fs_hz, float bw_hz) {
        f0 = f0_hz; fs = fs_hz; bw = bw_hz;
        float w0  = 2.0f * M_PI * f0 / fs;
        float bww = 2.0f * M_PI * bw / fs;
        float r   = 1.0f - bww / 2.0f;
        b0 =  1.0f;
        b1 = -2.0f * cosf(w0);
        b2 =  1.0f;
        a1 = -2.0f * r * cosf(w0);
        a2 =  r * r;
        x1 = x2 = y1 = y2 = 0.0f;
    }

    float process(float x) {
        float y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2;
        x2 = x1; x1 = x;
        y2 = y1; y1 = y;
        return y;
    }
};

// Three notch filter instances:
//   notchE     — elevation shaft_angle
//   notchE_vel — elevation shaft_velocity
//   notchA     — azimuth shaft_angle
NotchFilter notchE;
NotchFilter notchE_vel;
NotchFilter notchA;

// ---------------------------------------------------------------------------
// Axis state instances
// ---------------------------------------------------------------------------
AxisState stateE = { "E", 0, 0, 0, false, false, 0, 0, 0, 0, 0, 0, {}, 0 };
AxisState stateA = { "A", 0, 0, 0, false, false, 0, 0, 0, 0, 0, 0, {}, 0 };

// ---------------------------------------------------------------------------
// Sensors — separate SPI buses, separate CS pins
//
//   sensorE : AS5048A on SPI1 (MOSI=26, MISO=1, SCK=27), CS=0
//             init(&SPI1) called explicitly in setup()
//   sensorA : AS5048A on SPI0 (MOSI=11, MISO=12, SCK=13), CS=10
//             init() called with no argument — uses Teensy default SPI bus
// ---------------------------------------------------------------------------
MagneticSensorSPI sensorE = MagneticSensorSPI(AS5048_SPI, 0);   // SPI1, CS=0
MagneticSensorSPI sensorA = MagneticSensorSPI(AS5048_SPI, 10);  // SPI0, CS=10

// ---------------------------------------------------------------------------
// Drivers
// ---------------------------------------------------------------------------
BLDCDriver3PWM driverE = BLDCDriver3PWM(9,  5,  6,  8);   // INA INB INC EN
BLDCDriver3PWM driverA = BLDCDriver3PWM(33, 36, 37, 7);   // INA INB INC EN

// ---------------------------------------------------------------------------
// Motors — GBM5208, 11 pole pairs (verified on both axes)
// ---------------------------------------------------------------------------
BLDCMotor motorE = BLDCMotor(11);
BLDCMotor motorA = BLDCMotor(11);

// ---------------------------------------------------------------------------
// Fault helper functions
// ---------------------------------------------------------------------------

// Append current motor state to the circular pre-fault telemetry buffer.
// Called every FOC tick so the buffer always holds the last FAULT_BUF_SIZE
// samples immediately before any fault event.
void recordSample(AxisState& s, BLDCMotor& m) {
    s.buf[s.buf_head] = { m.shaft_velocity, m.shaft_angle, m.voltage.q };
    s.buf_head = (s.buf_head + 1) % FAULT_BUF_SIZE;
}

// Print the circular fault buffer to Serial in chronological order.
// Called immediately before entering any fault state.
void dumpFaultBuffer(AxisState& s) {
    Serial.print(F("=FAULT DUMP ["));
    Serial.print(s.name);
    Serial.println(F("]: vel\tangle\tVq"));
    uint8_t i = s.buf_head;
    for (uint8_t n = 0; n < FAULT_BUF_SIZE; n++) {
        Serial.print(s.buf[i].vel);   Serial.print('\t');
        Serial.print(s.buf[i].angle); Serial.print('\t');
        Serial.println(s.buf[i].vq);
        i = (i + 1) % FAULT_BUF_SIZE;
    }
    Serial.println(F("=END DUMP"));
}

// Disable motor and enter glitch hold state.
// Hold clears after GLITCH_HOLD_MS once velocity drops below SAFE_RESUME_VEL.
void enterGlitchHold(AxisState& s, BLDCMotor& m) {
    m.disable();
    unsigned long now    = millis();
    s.glitch_hold_active = true;
    s.glitch_hold_start  = now;
    s.glitch_total_start = now;
    s.hold_extensions    = 0;
    s.runaway_tracking   = false;
    s.last_safe_time     = now;
}

// ---------------------------------------------------------------------------
// runFaultLogic()
//
// Runs one iteration of the fault state machine for one axis.
// Must be called every FOC tick from loop().
//
// Returns true while the axis is in glitch hold — caller does not run FOC
// on a held axis.  Returns false during normal operation.
//
// Fault tiers (in priority order):
//   1. Glitch hold management: if hold active, check for recovery or extend.
//   2. Instant clamp: |velocity| > INSTANT_CLAMP_VEL → enterGlitchHold().
//   3. Runaway watchdog: |velocity| > MAX_SAFE_VELOCITY for RUNAWAY_TIMEOUT_MS
//      with travel > MIN_RUNAWAY_TRAVEL → genuine runaway → disable + recover.
//
// HALT BEHAVIOUR: while(1) halts the entire Teensy (both axes).
// See file header for per-axis isolation upgrade path.
// ---------------------------------------------------------------------------
bool runFaultLogic(AxisState& s, BLDCMotor& m) {

    // ── Tier 1: Glitch hold management ─────────────────────────────────────
    if (s.glitch_hold_active) {

        if (millis() - s.glitch_total_start >= ABSOLUTE_HALT_MS) {
            Serial.print(F("GLITCH ["));  Serial.print(s.name);
            Serial.print(F("]: no recovery in "));
            Serial.print(ABSOLUTE_HALT_MS / 1000);
            Serial.print(F("s, vel="));   Serial.print(m.shaft_velocity);
            Serial.println(F(" — physical sensor fault suspected. Halted."));
            while (1);
        }

        if (millis() - s.glitch_hold_start >= GLITCH_HOLD_MS) {
            float vel = m.shaft_velocity;
            if (fabsf(vel) < SAFE_RESUME_VEL) {
                // Velocity LPF has decayed — safe to re-enable
                m.voltage_limit   = NORMAL_VOLT_LIMIT;
                m.target          = m.shaft_angle;
                s.commanded_angle = m.target;
                m.enable();
                s.glitch_hold_active = false;
                s.last_safe_time     = millis();
                Serial.print(F("GLITCH [")); Serial.print(s.name);
                Serial.print(F("]: recovered after "));
                Serial.print(millis() - s.glitch_total_start);
                Serial.print(F("ms, "));
                Serial.print(s.hold_extensions);
                Serial.println(F(" extensions"));
            } else {
                // Still contaminated — extend hold
                s.hold_extensions++;
                s.glitch_hold_start = millis();
                Serial.print(F("GLITCH [")); Serial.print(s.name);
                Serial.print(F("]: extending hold #"));
                Serial.print(s.hold_extensions);
                Serial.print(F(" vel="));    Serial.print(vel);
                Serial.print(F(" total="));  Serial.print(millis() - s.glitch_total_start);
                Serial.println(F("ms"));
            }
        }
        return true;  // axis is in hold — caller skips FOC
    }

    recordSample(s, m);

    // ── Tier 2: Instant clamp ───────────────────────────────────────────────
    if (fabsf(m.shaft_velocity) > INSTANT_CLAMP_VEL) {
        dumpFaultBuffer(s);
        Serial.print(F("SPIKE [")); Serial.print(s.name);
        Serial.print(F("] #"));    Serial.println(++s.spike_count);
        if (s.spike_count > 10) {
            Serial.print(F("SPIKE [")); Serial.print(s.name);
            Serial.println(F("]: too many, halted — check SPI wiring/EMI"));
            while (1);
        }
        enterGlitchHold(s, m);
        return true;
    }

    // ── Tier 3: Runaway watchdog ────────────────────────────────────────────
    if (fabsf(m.shaft_velocity) > MAX_SAFE_VELOCITY) {
        if (!s.runaway_tracking) {
            s.runaway_start_angle = m.shaft_angle;
            s.runaway_tracking    = true;
        }
        if (millis() - s.last_safe_time > RUNAWAY_TIMEOUT_MS) {
            float travel = fabsf(m.shaft_angle - s.runaway_start_angle);
            if (travel < MIN_RUNAWAY_TRAVEL) {
                // High velocity, low travel → sensor glitch (LPF contamination)
                dumpFaultBuffer(s);
                Serial.print(F("GLITCH [")); Serial.print(s.name);
                Serial.print(F("] #"));      Serial.print(++s.glitch_count);
                Serial.print(F(" travel=")); Serial.println(travel);
                if (s.glitch_count > 10) {
                    Serial.print(F("GLITCH [")); Serial.print(s.name);
                    Serial.println(F("]: too many, halted — check SPI wiring/EMI"));
                    while (1);
                }
                enterGlitchHold(s, m);
            } else {
                // High velocity, real travel → genuine runaway
                m.disable();
                dumpFaultBuffer(s);
                Serial.print(F("RUNAWAY [")); Serial.print(s.name);
                Serial.print(F("] #"));       Serial.println(++s.fault_count);
                if (s.fault_count > 3) {
                    Serial.print(F("RUNAWAY [")); Serial.print(s.name);
                    Serial.println(F("]: halted"));
                    while (1);
                }
                Serial.print(F("RUNAWAY [")); Serial.print(s.name);
                Serial.println(F("]: recovering"));
                delay(2000);
                m.target          = m.shaft_angle;
                s.commanded_angle = m.target;
                m.enable();
                s.last_safe_time   = millis();
                s.runaway_tracking = false;
            }
        }
    } else {
        s.last_safe_time   = millis();
        s.runaway_tracking = false;
    }

    return false;
}

// ---------------------------------------------------------------------------
// handleSerial()
//
// Non-blocking line parser for angle commands from the ground station.
// Command format:
//   E<radians>\n  — set elevation target angle
//   A<radians>\n  — set azimuth target angle
//
// The parsed angle is written to stateE/stateA.commanded_angle.
// The FOC move() call in loop() picks it up on the next tick via wrapNearest().
// ---------------------------------------------------------------------------
void handleSerial() {
    static char    buf[12];
    static uint8_t idx = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (idx > 1) {
                if      (buf[0] == 'E') stateE.commanded_angle = atof(&buf[1]);
                else if (buf[0] == 'A') stateA.commanded_angle = atof(&buf[1]);
            }
            idx = 0;
        } else if (idx < sizeof(buf) - 1) {
            buf[idx++] = c;
            buf[idx]   = '\0';
        }
    }
}

// ---------------------------------------------------------------------------
// rampTarget()
//
// Slew-rate limiter — steps motor.target toward the commanded angle by at most
// SLEW_RATE radians per call, taking the shortest angular path.
//
// NOTE: This function is retained for reference but is not called in the
// current loop().  wrapNearest() is used instead, which passes the commanded
// angle directly to motor.move() and relies on SimpleFOC's internal velocity
// limit (motor.velocity_limit) for slew limiting.  rampTarget() provides
// finer-grained per-tick control if velocity_limit proves insufficient.
// ---------------------------------------------------------------------------
void rampTarget(BLDCMotor& m, float commanded) {
    float delta = commanded - m.shaft_angle;
    while (delta >  M_PI) delta -= 2.0f * M_PI;
    while (delta < -M_PI) delta += 2.0f * M_PI;
    float destination = m.shaft_angle + delta;

    float error = destination - m.target;
    if (fabsf(error) > SLEW_RATE)
        m.target += (error > 0) ? SLEW_RATE : -SLEW_RATE;
    else
        m.target = destination;
}

// ---------------------------------------------------------------------------
// wrapNearest()
//
// Converts a ground-station absolute angle command (radians) to SimpleFOC's
// unwrapped shaft_angle space, taking the shortest angular path from the
// current shaft position.
//
// Required because SimpleFOC accumulates shaft_angle without wrapping to
// [0, 2π] — passing an unwrapped commanded angle that matches the current
// position avoids unnecessary full-rotation travel on each command tick.
// ---------------------------------------------------------------------------
float wrapNearest(const BLDCMotor& m, float commanded) {
    float delta = commanded - m.shaft_angle;
    while (delta >  M_PI) delta -= 2.0f * M_PI;
    while (delta < -M_PI) delta += 2.0f * M_PI;
    return m.shaft_angle + delta;
}

// ---------------------------------------------------------------------------
// initAxis()
//
// Common initialisation sequence for one axis.  Called from setup() with
// axis-specific calibration values.
//
// Parameters:
//   m          — BLDCMotor instance
//   d          — BLDCDriver3PWM instance
//   s          — MagneticSensorSPI instance (already init()'d before call)
//   zero_elec  — electrical zero angle (rad), from initFOC() calibration sweep
//   dir        — sensor direction (Direction::CW or Direction::CCW)
//   sensor_off — mechanical offset applied after initFOC() (rad)
//   home_angle — target angle at startup (rad); shortest-path move on enable
//
// SENSOR_OFFSET ORDERING:
//   sensor_offset is assigned after initFOC(). initFOC() uses the raw sensor
//   reading to align electrical zero; applying sensor_offset before that call
//   would bias the electrical angle calibration.
// ---------------------------------------------------------------------------
void initAxis(BLDCMotor& m, BLDCDriver3PWM& d, MagneticSensorSPI& s,
              float zero_elec, Direction dir, float sensor_off, float home_angle) {
    m.linkSensor(&s);

    d.voltage_power_supply = 12.0f;
    d.pwm_frequency        = 25000;  // 25 kHz — eliminates audible switching noise
    d.init();
    m.linkDriver(&d);

    m.foc_modulation           = FOCModulationType::SpaceVectorPWM;
    m.controller               = MotionControlType::angle;
    m.PID_velocity.P           = 0.5f;
    m.PID_velocity.I           = 8.0f;   // may be overridden per-axis in setup()
    m.PID_velocity.D           = 0.0f;   // zeroed — AS5048A quantization noise at 2 kHz
    m.PID_velocity.output_ramp = 300.0f; // may be overridden per-axis in setup()
    m.voltage_limit            = NORMAL_VOLT_LIMIT;
    m.LPF_velocity.Tf          = 0.01f;  // velocity LPF time constant (s)
    m.P_angle.P                = 15.0f;
    m.P_angle.output_ramp      = 300.0f; // rad/s² — velocity demand rate limit
    m.velocity_limit           = 8.0f;   // rad/s

    m.init();
    m.zero_electric_angle = zero_elec;
    m.sensor_direction    = dir;
    m.initFOC();
    m.sensor_offset       = sensor_off;  // applied AFTER initFOC() — see note above

    // Move to home using shortest angular path to avoid a full-rotation startup sweep
    s.update();
    m.shaft_angle = m.shaftAngle();
    float delta   = home_angle - m.shaft_angle;
    while (delta >  M_PI) delta -= 2.0f * M_PI;
    while (delta < -M_PI) delta += 2.0f * M_PI;
    m.target = m.shaft_angle + delta;
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);

    // Initialise SPI1 for the elevation encoder (sensorE)
    SPI1.setMOSI(26);
    SPI1.setMISO(1);
    SPI1.setSCK(27);
    SPI1.begin();

    // sensorE on SPI1 — explicit bus argument required
    sensorE.spi_mode = SPI_MODE1;
    sensorE.init(&SPI1);

    // sensorA on SPI0 — no argument uses Teensy default SPI (pins 11/12/13)
    sensorA.spi_mode = SPI_MODE1;
    sensorA.init();

    // Elevation axis — calibration values from v5 hardware build
    initAxis(motorE, driverE, sensorE, 3.73f, Direction::CW, -0.30f, 0.0f);

    // Azimuth axis — calibration values from current hardware build
    // NOTE: zero_electric_angle and sensor_offset are hardware-specific.
    // Re-run initFOC() calibration sweep after any motor or encoder replacement.
    initAxis(motorA, driverA, sensorA, 4.13f, Direction::CW, -0.50f, 0.0f);

    // Notch filters — tuned to suppress ~4.8–4.9 Hz gimbal resonance at 2 kHz
    notchE.init(4.8f,     2000.0f, 1.0f);
    notchE_vel.init(4.9f, 2000.0f, 1.0f);
    notchA.init(4.9f,     2000.0f, 1.0f);

    // Elevation axis overrides — integrator required to hold camera weight against gravity.
    // I=8.0 winds up over ~1 s under static load; output_ramp prevents position overshoot.
    motorE.PID_velocity.I           = 8.0f;
    motorE.PID_velocity.output_ramp = 1000.0f;

    // Azimuth axis overrides — direct-voltage angle control (no inner velocity loop).
    // P_angle outputs voltage (V/rad) directly to the motor; I and D are added for
    // steady-state offset and arrival damping respectively.
    motorA.controller          = MotionControlType::angle_nocascade;
    motorA.P_angle.P           = 30.0f;   // V/rad
    motorA.P_angle.I           = 0.5f;    // V·s/rad — steady-state offset correction
    motorA.P_angle.D           = 2.5f;    // V·s/rad — arrival damping
    motorA.P_angle.output_ramp = 150.0f;  // V/s — voltage slew rate limit
    motorA.LPF_angle.Tf        = 0.010f;  // angle LPF time constant (s)
    motorA.voltage_limit       = 11.0f;   // V — full supply headroom for direct-V mode

    // Seed state machines from current motor targets
    stateE.commanded_angle = motorE.target;
    stateA.commanded_angle = motorA.target;
    stateE.last_safe_time  = millis();
    stateA.last_safe_time  = millis();

    delay(1000);
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
    // Gate FOC to ~2 kHz. See loop rate note in file header.
    static unsigned long last_us = 0;
    unsigned long now_us = micros();
    if (now_us - last_us < 500UL) return;
    last_us = now_us;

    // Elevation axis — cascaded angle → velocity → voltage
    motorE.loopFOC();
    motorE.shaft_angle    = notchE.process(motorE.shaft_angle);      // see notch filter note
    motorE.shaft_velocity = notchE_vel.process(motorE.shaft_velocity);
    motorE.move(wrapNearest(motorE, stateE.commanded_angle));

    // Azimuth axis — direct-voltage angle control
    motorA.loopFOC();
    motorA.shaft_angle = notchA.process(motorA.shaft_angle);
    motorA.move(wrapNearest(motorA, stateA.commanded_angle));

    handleSerial();
    runFaultLogic(stateE, motorE);
    runFaultLogic(stateA, motorA);

    // Binary feedback telemetry → ground station (every 10 FOC ticks ≈ 200 Hz)
    static uint8_t fb_div = 0;
    if (++fb_div >= 10) {
        fb_div = 0;
        struct __attribute__((packed)) FeedbackFrame {
            uint8_t  header;        // 0x46 'F'
            uint32_t timestamp_us;  // micros() at frame assembly
            float    angle_E;       // motorE.shaft_angle (rad)
            float    angle_A;       // motorA.shaft_angle (rad)
            float    vq_E;          // motorE.voltage.q (V) — quadrature voltage
            float    vq_A;          // motorA.voltage.q (V)
        } fb;
        fb.header       = 0x46;
        fb.timestamp_us = micros();
        fb.angle_E      = motorE.shaft_angle;
        fb.angle_A      = motorA.shaft_angle;
        fb.vq_E         = motorE.voltage.q;
        fb.vq_A         = motorA.voltage.q;
        Serial.write((const uint8_t*)&fb, sizeof(fb));
    }
}
