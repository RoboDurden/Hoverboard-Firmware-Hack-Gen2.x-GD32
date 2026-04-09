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

## Next steps

1. **Reduce PI gains dramatically**: Match reference — Iq Kp=0.6 Ki=0.6,
   Id Kp=0.4 Ki=0.36. Our values of 2-3 were far too aggressive.
2. **Add anti-windup back-calculation**: Replace simple integrator clamp
   with Kb-based back-calculation for smoother saturation recovery.
3. **Fix Ib calibration**: The -22 offset on iIb suggests the auto-cal
   runs too early. Try longer delay or running average filter like
   the reference (which averages over 2000 cycles).
4. **Add SVPWM centering**: Simple min-max offset after inverse Clarke.
   Free ~15% voltage headroom.
5. **Verify current sign**: With correct (lower) gains, re-test PI.
   If iIq=+40 for positive speed with `adc-offset`, the sign is correct.
   If PI still opposes, try `offset-adc`.
6. **Measure back-EMF**: Scope the motor wires while spinning by hand.
   Determines if sinusoidal or trapezoidal waveform is optimal.
7. **Verify physical wire mapping**: Firmware Y/B/G names may not match
   motor wire colors. Energize each channel and observe which wire twitches.
8. **UART control**: Switch from REMOTE_DUMMY to REMOTE_UART + joystick
   for real-world testing.

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

FOC state is logged via SEGGER RTT when REMOTE_DUMMY + RTT_REMOTE are enabled.
Output format:
```
26.72 V  FOC  pos:3  ang:159  Id: -19  Iq:   3  off:150  st:37
```
- FOC/BLC: current control mode
- pos: hall sector (1-6)
- ang: electrical angle in degrees (0-359)
- Id/Iq: rotating frame currents
- off: angle offset in degrees
- st: sector_ticks (ISR ticks per hall sector, lower = faster)

See `howto.md` for RTT connection commands.
