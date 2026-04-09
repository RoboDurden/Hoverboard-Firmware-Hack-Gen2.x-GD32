# FOC Implementation Notes

## Hardware

- MCU: GD32F130C8 (Cortex-M3, 72MHz, 8KB RAM, 64KB Flash, no FPU)
- Motor: BLDC hoverboard motor, 3-phase
- Hall sensors: 3 (6 positions per electrical revolution, 60° resolution)
- PWM: 16kHz, center-aligned, complementary outputs with 60-count dead time
- Phase current shunts: 2 low-side, amplified by dual op-amp
  - Shunt resistors: 2x R004 (4mΩ) SMD
  - Op-amp: HM8632 (C32X marking), dual RRIO CMOS, 6.5MHz, 520µA
  - Feedback: 3x 10KΩ (1002 marking) resistors
  - Op-amp gain: ~20x (estimated from 1A → ~100 ADC counts)
  - Biased at ~VCC/2 (~2000 ADC counts at zero current)
  - ~0.01A per ADC count (10mA resolution)
  - PB0 = TIMER_BLDC_CHANNEL_Y (CH0/PA8 low-side)
  - PB1 = TIMER_BLDC_CHANNEL_B (CH1/PA9 low-side)
  - No shunt on TIMER_BLDC_CHANNEL_G (CH2/PA10)
  - Third phase derived via Kirchhoff: Ig = -(Iy + Ib)
  - NOTE: firmware channel names (Y/B/G) may not match physical motor wire
    colors. Robo Durden's photo of the same board labels the shunts as
    "Blue shunt" and "Green shunt" by wire color. The firmware-to-ADC
    mapping is correct regardless of wire color naming.

## Architecture

All FOC code lives in `foc.c` / `foc.h`. The control loop runs inside
`CalculateBLDC()` at ~16kHz (triggered by DMA completion after ADC scan).

### Signal flow

```
ADC (DMA) → phase_current_y, phase_current_b
                    ↓
         foc_current_update()     ← auto-calibrated offsets
                    ↓
            Iy, Ib, Ig (int16_t, ADC counts)
                    ↓
           foc_clarke()           ← Clarke transform
                    ↓
            Iα, Iβ (stationary frame)
                    ↓
           foc_park()             ← Park transform (needs electrical angle)
                    ↓
            Id, Iq (rotating frame)
                    ↓
      [Currently open-loop: Vd=0, Vq=speed_input]
      [PI controllers exist but sign convention issue]
                    ↓
        foc_inverse_park()
                    ↓
            Vα, Vβ
                    ↓
       foc_inverse_clarke()
                    ↓
         Vy, Vb, Vg → PWM timer channels
```

### Angle estimation

Hall sensors give 6 discrete positions per electrical revolution. Between
transitions, the angle is linearly interpolated using the time between the
last two transitions (speed estimate). The electrical angle is stored as
uint16_t (0..65535 = 0..360°) for natural wraparound.

**Direction-aware**: The interpolation detects rotation direction from hall
transition sequence (1→2→3 = forward, 3→2→1 = reverse) and interpolates
forward or backward accordingly. This was critical — without it, one
direction was silent and the other rumbled.

**Angle offset**: 150° (27307 uint16), hardcoded. Determined by rotor
alignment test (`foc_align_rotor()`) and confirmed by Id_rms sweep.

### Math

All transforms use integer-only Q15 fixed-point arithmetic:
- 256-entry sin lookup table (Q15: -32768..32767 = -1.0..+1.0)
- cos via sin table with 90° offset
- Clarke: 1/√3 ≈ 18919/32768 (Q15)
- Inverse Clarke: √3/2 ≈ 28378/32768 (Q15)

Total overhead: ~660 bytes Flash for sin table, ~8 bytes RAM for FOC state.

## Current status

### What works
- **Open-loop voltage FOC**: Vd=0, Vq=bldc_outputFilterPwm. Motor runs
  silently in both directions, self-starts from standstill.
- Synchronized ADC sampling of phase currents (DMA triggered from timer update)
- Automatic zero-current offset calibration at startup (~200ms, motor off)
- Hall-based angle estimation with direction-aware interpolation
- Clarke and Park transforms producing Id/Iq in the rotating frame
- 150° angle offset (lowest Id_rms = 28.1)
- Lower pitch vibration than block commutation (more sinusoidal output)

### What doesn't work yet
- **PI current controllers**: When enabled, the PI opposes motor motion.
  The motor starts fast (PI output small), then slows as iq_ref ramps up.
  Inverting current sign made it worse. Root cause: sign convention mismatch
  in the feedback chain (shunt polarity → Clarke → Park → PI direction).
  
  **To debug**: Log Id/Iq via RTT while open-loop runs at constant speed.
  Check: does Iq read positive or negative when motor spins forward with
  positive Vq? The PI reference must match this sign.

## Angle offset calibration

### Alignment test
`foc_align_rotor()` applies 500 PWM counts at 0° electrical for 800ms, reads
halls, computes offset. Found **150° offset**. Currently hardcoded to avoid
the DC overcurrent from the alignment hold.

### Sweep results (block commutation, observing FOC math)

| Offset | Id_rms | Iq_rms | Ratio | Note |
|--------|--------|--------|-------|------|
| 0°     | 40.3   | 32.4   | 0.80  | default |
| 55°    | 40.0   | 50.8   | 1.27  | manual guess |
| 90°    | 40.4   | 45.7   | 1.13  | |
| 120°   | 39.3   | 33.2   | 0.85  | |
| **150°** | **28.1** | **33.4** | **1.19** | **alignment result, lowest Id** |
| 180°   | 35.2   | 39.3   | 1.12  | |

### Direction tuning
At 150°, one direction was nearly silent, the other rumbled. Adding
direction-aware interpolation (forward/reverse angle within sector)
fixed this — both directions now sound the same.

135° and 165° were both worse than 150° in both directions.

## FOC driving attempts (chronological)

### Attempt 1: PI closed-loop (Kp=5, limit=1000, no calibration, 0° offset)
- Immediate overcurrent, PSU tripped

### Attempt 2: PI closed-loop (Kp=2, limit=500, auto calibration, 150° offset)
- Motor moved! Continued spinning when pushed by hand
- Horrible noise, voltage dropped 25→24V

### Attempt 3: PI closed-loop (Kp=1, limit=250)
- Too weak, just clicking

### Attempt 4: PI + BLC startup + warmup + alignment
- Alignment DC hold drew too much current
- BLC didn't start properly after alignment

### Attempt 5: Open-loop voltage, BLC startup, 150° offset, /4 scaling
- First working FOC! Silent one way, rumble other way
- Direction-aware interpolation fixed the rumble

### Attempt 6: Open-loop, no BLC, Vq=bldc_outputFilterPwm
- Self-starts from standstill
- Lower pitch than BLC, both directions similar
- Current best configuration

### Attempt 7: PI closed-loop with soft ramp (Kp=3/2, limit=400, iq_ref/8)
- Motor started fast then slowed — PI opposing motion
- Sign convention issue in current feedback
- Inverting current sign made it worse

## Configuration

In `config.h`:
```c
#define BLDC_BC            // block commutation (used as fallback)
#define FOC_ENABLED        // enable open-loop voltage FOC
```

In `remoteDummy.c`: `speed = 300` for constant speed testing.

## Current sensing verification

ISR-rate averaging (1000 samples at 16kHz, reported every ~62ms) confirms
sensing and transforms are working correctly:

| Signal | Avg | Variance | Interpretation |
|--------|-----|----------|----------------|
| iIy | -2 | 3 | Near-zero average (AC signal) ✓ |
| iIb | -22 | 4 | DC offset — calibration issue |
| iId | -2 | 7 | Near-zero flux ✓ |
| iIq | 40 | 6 | Stable torque signal ✓ |

Key insight: snapshot values (200Hz RTT rate) show variance ~1000, but
ISR-averaged values show variance ~5. The 16kHz control loop sees stable
values — the apparent noise was just aliasing from the slow RTT sample rate.

**Open-loop FOC does not use current measurements** — the motor spins purely
from hall-based angle estimation and voltage transforms. Current sensing is
read-only until PI controllers are enabled.

## Live trim test results (confirmed angle and sign convention)

Used joystick axis 4 → `iSteer` → `foc_angle.angle_offset` for live tuning.
Sweep range: 90°-210° around 150° center.

| Offset | iId | iIq | Notes |
|--------|-----|-----|-------|
| 90° | -4 | +9 | Edge of range |
| 150° | +0.4 | **-32** | **Motor fastest here** (user observation) |
| 175° | -22 | +26 | (default startup value) |
| 190° | +0.4 | +64 | Second iId zero crossing (180° flipped) |
| 205° | +48 | +55 | Diverging |

**Confirmations:**
- 150° is the correct angle offset (matches motor max speed)
- iIq is **negative** for forward motion at 150°
- The PI controller failure was a **sign convention mismatch** — our `iq_ref`
  was positive (matching forward speed input) but measured `iIq` was negative
- Fix: invert current measurement (`current = offset - adc`) like the reference
  project, OR negate `iq_ref` before passing to PI

The 190° point is the same physical alignment rotated 180° — both are
"perpendicular to flux" but with opposite torque sign. Motor was slower
at 190° because the voltage was applied in the wrong direction relative
to the rotor.

## PI controller (not yet working)

Values tried:
- Kp_d=2, Ki_d=1, limit=400 (flux)
- Kp_q=3, Ki_q=1, limit=400 (torque)
- iq_ref = bldc_outputFilterPwm / 8 with soft ramp

Attempt with ramp: motor started fast (PI output small) then slowed as
iq_ref ramped up — PI was opposing motion. Inverting current sign was worse.

Now that ISR-rate analysis shows iIq=+40 for positive speed, the PI reference
sign should match. The iIb offset of -22 may still cause issues.

## Reference implementation analysis

Studied `/Users/alex/dev/c/hoverboard-firmware-hack-FOC` — a production-grade
FOC for STM32F103 hoverboard mainboard (different from our GD32 sensorboard).
Matlab/Simulink generated core. Key learnings:

### Current sign convention
Reference uses `current = offset - adc_reading` (inverted from our
`adc_reading - offset`). However, this depends on the op-amp topology:
- Their board has different shunt amps (possibly inverting)
- Our HM8632 might be non-inverting — need to verify from circuit
- We tested both signs and neither worked, suggesting the sign alone
  isn't the problem

### PI gains — ours were far too aggressive
Reference gains (converted from Q11 fixed-point):

| Loop | Kp | Ki | Anti-windup |
|------|----|----|-------------|
| Speed | 2.36 | 0.12 | Kb=0.12 |
| Iq (torque) | **0.60** | **0.60** | Kb=0.36 |
| Id (flux) | **0.40** | **0.36** | Kb=0.38 |

Our Iq Kp was 2-3, theirs is **0.60** — we were 3-5x too aggressive.
This is likely the primary reason the PI failed, not the sign convention.
The motor was responding correctly but the overshoot was so severe it
looked like the PI was opposing motion.

### Anti-windup
Reference uses **back-calculation anti-windup** (Kb coefficient) which is
more sophisticated than our simple integrator clamp. When the output
saturates, the integrator is actively driven back by `Kb * (saturated -
unsaturated)`. This prevents the integrator from building up during
saturation and overshooting on release.

### Startup strategy
- Commutation mode below 240 RPM (speed-based, not time-based)
- FOC above 480 RPM
- Hysteresis between thresholds prevents mode oscillation
- Open-loop voltage ramp at `dV_openRate` per control cycle

### SVPWM
Reference doesn't use true SVPWM either — same inverse Clarke (SPWM)
but subtracts common-mode `(Va+Vb+Vc)/3` to center waveforms. This is
equivalent to the min-max method:
```c
offset = -(max(Va,Vb,Vc) + min(Va,Vb,Vc)) / 2;
Va += offset; Vb += offset; Vc += offset;
```
Gives ~15% more DC bus voltage utilization with no downside.

### Motor parameters
- `n_polePairs = 15` (30 poles — standard hoverboard hub motor)
- `cf_speedCoef = 10667` for RPM calculation
- `I_DC_MAX = 100A` DC-link current limit
- `i_max = 12000` units (~60A) phase current limit

### Waveform
Uses sinusoidal modulation in FOC mode, not trapezoidal. The motor
back-EMF shape (sinusoidal vs trapezoidal) can be measured with a scope
by spinning the wheel by hand with the controller off.

### Field weakening
Implemented but disabled by default. Activates above 4800 RPM by
injecting negative Id current. Uses speed-dependent Vq_max lookup table
to constrain torque at high speed.

## SVPWM vs SPWM

Our current implementation uses **SPWM** (sinusoidal PWM) — the inverse
Clarke produces three pure sinusoidal phase voltages. This is simple and
correct but only uses ~87% of the available DC bus voltage.

**SVPWM** (space vector PWM) adds a common-mode offset that centers the
waveforms, giving ~15% more voltage utilization. Two equivalent methods:

1. **Min-max centering** (simplest, one line of code):
   ```c
   int16_t offset = -(MAX(y, MAX(b, g)) + MIN(y, MIN(b, g))) / 2;
   y += offset; b += offset; g += offset;
   ```

2. **Common-mode subtraction** (reference project uses this):
   ```c
   int16_t offset = (y + b + g) / 3;
   y -= offset; b -= offset; g -= offset;
   ```

3. **t0/t1/t2 sector-based switching** (the old hhworking branch SVM code):
   More complex, mathematically equivalent to min-max for centered vectors.

All three produce identical motor behavior. Method 1 is recommended for
our implementation — add after `foc_inverse_clarke()`.

The old SVM code on `hoverboardhavoc/hhworking` was a legitimate SVPWM
implementation based on Jantzen Lee's tutorial, using sector-based t0/t1/t2
timing with phase advance. It was open-loop (hall sector only, no angle
interpolation) — essentially "better shaped block commutation".

## Motor waveform: sinusoidal vs trapezoidal

Hoverboard motors typically have **trapezoidal back-EMF** (designed for
cheap 6-step controllers). This affects the optimal control strategy:

| Approach | Trapezoidal motor | Sinusoidal motor |
|----------|-------------------|------------------|
| 6-step block commutation | Good torque, noisy | Poor — torque ripple |
| Sinusoidal FOC (our impl) | Smooth, ~5% less peak torque | Optimal |
| Trapezoidal FOC | Optimal — max torque + smooth | Poor — mismatch |

**Trapezoidal FOC** uses the same control loop (Clarke, Park, PI, inverse
Park) but with a trapezoidal lookup table instead of sine. This matches
the back-EMF shape for maximum torque while keeping the benefits of current
control. To implement: replace the sine table in `foc.c` with a trapezoidal
waveform (flat tops for 120°, linear ramps for 60°).

**To determine which waveform this motor uses**: disconnect from controller,
spin wheel by hand, scope any two motor wires. Trapezoidal = flat tops with
steep ramps. Sinusoidal = smooth rounded peaks.

The actual back-EMF shape should be measured before deciding. For now,
sinusoidal FOC is a good default — it works on both motor types and the
smoothness improvement over 6-step is significant regardless.

## Next steps (priority order)

1. **Reduce PI gains dramatically**: Match reference — Iq Kp=0.6 Ki=0.6,
   Id Kp=0.4 Ki=0.36. Our values of 2-3 were far too aggressive.
   This is the most likely fix for the PI current loop.
2. **Add anti-windup back-calculation**: Replace simple integrator clamp
   with Kb-based back-calculation for smoother saturation recovery.
3. **Fix Ib calibration**: The -22 offset on iIb suggests the auto-cal
   runs too early. Try longer delay or running average filter like
   the reference (which averages over 2000 cycles).
4. **Verify current sign**: With correct (lower) gains, re-test PI.
   If iIq=+40 for positive speed with `adc-offset`, the sign is correct.
   If PI still opposes, try `offset-adc`.
5. **Add SVPWM centering**: Simple min-max offset after inverse Clarke.
   Free ~15% voltage headroom.

## Ideas and suggestions

### Speed measurement and comparison

The firmware has built-in speed measurement via `realSpeed` (km/h, legacy)
and `revs32` (revs/s × 1024, fixed-point). Both are computed from hall
transition timing in `CalculateBLDC()`.

RPM can also be derived from `foc_angle.sector_ticks`:
```
RPM = 60 * PWM_FREQ / (sector_ticks * 6 * pole_pairs)
```
With 15 pole pairs and sector_ticks=35: RPM ≈ 305.

**Open-loop FOC was slower than block commutation at the same speed input.**
This is expected — SPWM only uses ~87% of the DC bus voltage vs block
commutation's ~100%. Adding SVPWM centering (see above) would recover
~15% and nearly match block commutation speed. An external RPM counter
(optical/magnetic) could verify the exact difference.

The `revs32` value was visible in RTT output (the `revs:` field from
remoteDummy). To add RPM to the FOC RTT output, compute from sector_ticks.

### Live angle tuning via joystick trim

Instead of reflashing to test each angle offset, use REMOTE_UART with
the joystick controller and map the steer axis to the angle offset.
This allows real-time tuning while monitoring Id/Iq on RTT (RTT uses
SWD, UART uses PA2/PA3 — both can run simultaneously).

In `bldc.c`:
```c
// Map steer input (-1000..+1000) to angle offset
// Center = 150° (27307), ±60° trim range (±10922)
foc_angle.angle_offset = 27307 + (steer * 10922 / 1000);
```

Sweep the trim while watching iId on RTT — find the angle where iId
is closest to zero. Then hardcode the result.

The joystick controller already sends `iSteer` in the serial protocol.
A trim axis on the physical joystick could be mapped to this via
the `-a` axis parameter, or by modifying the joystick controller to
send a separate trim axis as the steer value.

### Trapezoidal FOC waveform

If the motor has trapezoidal back-EMF (measure with scope — spin wheel
by hand with controller off, observe waveform on motor wires), a
trapezoidal lookup table would give better torque than sine.

Implementation: replace the 256-entry `sin_table[]` in `foc.c` with a
trapezoidal waveform:
```
Sine:        0 → peak → 0 → -peak → 0  (smooth)
Trapezoidal: 0 → ramp → flat → ramp → 0 → ramp → -flat → ramp → 0
             |60°| 120° |60°|  60°  |120°  |60°|
```

Could be a compile-time option:
```c
#ifdef FOC_WAVEFORM_TRAP
  static const int16_t waveform_table[256] = { ... };  // trapezoidal
#else
  static const int16_t waveform_table[256] = { ... };  // sinusoidal
#endif
```

Then `foc_sin()` and `foc_cos()` use `waveform_table` instead of
`sin_table`. Everything else (Park, Clarke, PI) stays identical.

A/B test sinusoidal vs trapezoidal by switching the define and
comparing sound, vibration, and torque.

### Speed-based FOC/BLC mode switching

Instead of time-based warmup, use speed (sector_ticks) with hysteresis
like the reference project:
- BLC below 240 RPM equivalent (~sector_ticks > 500)
- FOC above 480 RPM equivalent (~sector_ticks < 250)
- Hysteresis between thresholds prevents oscillation

Previous attempts at mode switching caused grinding because the voltage
scaling didn't match between BLC and FOC. The fix is to match the
effective voltage: BLC uses `bldc_outputFilterPwm` directly, so FOC
should use a similar magnitude (currently `bldc_outputFilterPwm / 1`
in open-loop mode, which matches).

### Measure back-EMF with scope

Disconnect motor from controller (or power off with FETs disabled).
Spin wheel by hand, measure between any two motor wires.
- Trapezoidal = flat tops and bottoms with steep ramps
- Sinusoidal = smooth rounded peaks

Also reveals:
- Pole pairs (count electrical cycles per mechanical revolution)
- Back-EMF constant (V/RPM) — useful for PI tuning and field weakening

### Verify physical wire-to-channel mapping

Firmware names Y/B/G correspond to timer channels CH0/CH1/CH2. The
actual motor wire colors may not match. To verify:
1. Disconnect motor from wheel (free spinning)
2. Energize one timer channel at a time with a small DC voltage
3. Observe which physical wire / motor direction responds
4. Map: wire color → timer channel → firmware name

### RPM from odometry (no firmware change needed)

The firmware already sends `iOdomL` (signed int32, hall step count) via
UART. The joystick controller can compute RPM from the delta:

```c
// In joystick controller, every 10ms:
static int32_t lastOdom = 0;
int32_t delta = oData.iOdomL - lastOdom;
lastOdom = oData.iOdomL;
// 90 hall steps per mechanical revolution (6 steps × 15 pole pairs)
float rpm = (delta * 100.0 * 60.0) / 90.0;
```

15 pole pairs confirmed by VESC parameter detection screenshot in the
reference FOC project (`docs/pictures/motor_VESC_params.png`).

The existing `realSpeed` in the firmware is in km/h using a magic constant
(`1991.81f / speedCounter`) that assumes a specific wheel diameter — not
useful for bench testing or different wheel setups.

Could also expose `sector_ticks` and FOC debug data via the unused UART
feedback fields (`iSpeedR`, `iAmpR`, `iOdomR` are all zero in single
board mode):
```c
#ifdef FOC_ENABLED
    oData.iSpeedR = (int16_t) foc_angle.sector_ticks;
    oData.iAmpR = (int16_t) foc_dq.q;    // Iq
    oData.iOdomR = (int32_t) foc_angle.electrical_angle;
#endif
```

### Current scaling calibration

With 4mΩ shunts and ~20x op-amp gain:
- Estimated ~0.01A per ADC count (10mA resolution)
- Verify by comparing PSU ammeter reading with ADC count at known load
- This converts `iq_ref` from arbitrary ADC units to actual amps
- Needed for meaningful torque control and current limiting

### Field weakening

At high speed, back-EMF approaches supply voltage and torque drops.
Inject negative Id to weaken the permanent magnet field:
- Extends speed range at the cost of efficiency
- Reference project activates above 4800 RPM with max Id of -4000 units
- Requires a Vq_max lookup table (speed → max available voltage)
- Only enable once basic FOC + PI is stable

The reference project has a diagram (`docs/pictures/FieldWeakening.png`)
showing three blending modes based on Lo/Hi speed thresholds:
- **Fully Blended**: FW ramps 0→FW_max within the input range
- **Partially Blended**: FW starts but input range ends before FW_max
- **Outside**: FW thresholds above input range — never activates

### Matlab/Simulink vs handwritten C

The reference FOC project uses Simulink-generated code. Pros: visual model,
automatic fixed-point scaling, simulation before flashing. Cons: unreadable
generated code (`rtDW->Switch2_e`), expensive toolchain (~$10K), hard to
debug on target, inflexible for custom features.

Our handwritten approach is ~200-300 lines of C for the complete FOC core.
The Simulink model is useful as a **design reference** (gains, thresholds,
structure) without needing to use the generated code. For a single developer
iterating on hardware with RTT debugging, handwritten C is faster to develop
and much easier to debug.

## Files

| File | Purpose |
|------|---------|
| `Inc/foc.h` | FOC types, function declarations |
| `Src/foc.c` | Transforms, PI controller, sin table, alignment, control loop |
| `Src/bldc.c` | Hooks FOC into CalculateBLDC ISR |
| `Src/bldcBC.c` | InitBldc: FOC init + offset calibration |
| `Inc/defines/defines_2-1-20.h` | PHASE_CURRENT_Y=PB0, PHASE_CURRENT_B=PB1 |
| `Inc/config.h` | FOC_ENABLED flag |

## RTT debugging

### What is RTT?

SEGGER Real Time Transfer (RTT) is a debug logging mechanism that works
over the ST-LINK SWD connection — no UART needed. The firmware writes to
a RAM buffer, and the debugger (OpenOCD) reads it out over SWD in the
background without stopping the CPU.


**How it works:**
1. The firmware includes `SEGGER_RTT.c` (already in the project)
2. A control block (`"SEGGER RTT"`) is placed in RAM — OpenOCD searches
   for this magic string to find the buffer
3. `SEGGER_RTT_WriteString(0, "hello\r\n")` writes to channel 0's ring buffer
4. OpenOCD polls the buffer via SWD and forwards data to a TCP port
5. You connect to that TCP port to read the output

**Advantages over UART:**
- No extra wires — uses the same ST-LINK already connected for flashing
- Doesn't interfere with motor control UART (REMOTE_UART can stay active)
- Non-blocking — if the host isn't reading, data is silently dropped
  (mode NO_BLOCK_SKIP), so it never stalls the firmware
- Much faster than UART for burst data

**Limitations:**
- Buffer is only 1024 bytes — high-frequency logging can overflow
- `SEGGER_RTT_MODE_NO_BLOCK_SKIP` (default) drops writes when buffer is full
- OpenOCD must find the RTT control block in RAM — if the firmware restarts,
  you may need to re-issue `rtt start` to re-scan
- Small CPU overhead for the `sprintf` + `WriteString` calls

### How to use RTT

**Enable in firmware:**
```c
// In config.h, enable REMOTE_DUMMY with RTT:
#define REMOTE_DUMMY
#define RTT_REMOTE
#define WINDOWS_RN     // adds \r before \n for terminal compatibility
```

**Write from firmware:**
```c
#include "../Src/SEGGER_RTT.h"
SEGGER_RTT_WriteString(0, "Hello RTT\r\n");

// Or with formatting:
char s[64];
sprintf(s, "Id:%d Iq:%d\r\n", foc_dq.d, foc_dq.q);
SEGGER_RTT_WriteString(0, s);
```

**Read from host (start OpenOCD + connect):**
```bash
# Start OpenOCD with RTT server on port 9090
/Users/alex/.platformio/packages/tool-openocd-gd32/bin/openocd \
  -f interface/stlink.cfg \
  -c "transport select hla_swd" \
  -c "set CPUTAPID 0" \
  -f target/stm32f1x.cfg \
  -c "rtt setup 0x20000000 0x2000 \"SEGGER RTT\"" \
  -c "init" -c "rtt start" \
  -c "rtt server start 9090 0"

# Then in another terminal, read the output.
# Note: nc on macOS is unreliable for RTT — use python instead:
python3 -c "
import socket, time
s = socket.socket()
s.connect(('localhost', 9090))
s.settimeout(1.0)
while True:
    try:
        data = s.recv(4096)
        if data: print(data.decode(), end='')
    except socket.timeout: pass
"
```

**Reset and capture startup output:**
```bash
# If RTT is already running, reset via telnet to OpenOCD:
python3 -c "
import socket, time
# Connect RTT reader first
s = socket.socket(); s.connect(('localhost', 9090))
# Reset target via OpenOCD telnet
t = socket.socket(); t.connect(('localhost', 4444))
t.recv(1024); t.sendall(b'reset run\n')
time.sleep(1); t.sendall(b'rtt start\n'); t.close()
# Now read fresh output
while True:
    try: print(s.recv(4096).decode(), end='')
    except: pass
"
```

**Note:** OpenOCD must be running to flash firmware too, so you can't have
both `pio run --target upload` and RTT active simultaneously. Kill OpenOCD
before flashing, then restart it for RTT.

### Current FOC RTT output format

When REMOTE_DUMMY + RTT_REMOTE + PHASE_CURRENT are enabled:
```
pos:3  ang:159  Id: -19  Iq:   3  Iy:  16  Ib: -16  iId:  -2  iIq:  40  iIy:  -2  iIb: -22
```
- pos: hall sector (1-6)
- ang: electrical angle in degrees (0-359)
- Id/Iq: snapshot rotating frame currents (noisy at 200Hz sample rate)
- Iy/Ib: snapshot phase currents
- iId/iIq/iIy/iIb: ISR-averaged values (1000 cycles, stable)
