# FOC Implementation Notes

## Hardware

- MCU: GD32F130C8 (Cortex-M3, 72MHz, 8KB RAM, 64KB Flash, no FPU)
- Motor: BLDC hoverboard motor, 3-phase
- Hall sensors: 3 (6 positions per electrical revolution, 60° resolution)
- PWM: 16kHz, center-aligned, complementary outputs with 60-count dead time
- Phase current shunts: 2 low-side (PB0=Yellow, PB1=Blue, no Green)
  - Op-amp biased at ~VCC/2 (~2000 ADC counts at zero current)
  - Third phase derived via Kirchhoff: Ig = -(Iy + Ib)

## Architecture

All FOC code lives in `foc.c` / `foc.h`. The control loop runs inside
`CalculateBLDC()` at ~16kHz (triggered by DMA completion after ADC scan).

### Signal flow

```
ADC (DMA) → phase_current_y, phase_current_b
                    ↓
         foc_current_update()     ← offset calibration
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
         PI controllers           ← Id→0 (flux), Iq→ref (torque)
                    ↓
            Vd, Vq
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

An `angle_offset` parameter compensates for the physical alignment between
hall sensors and motor phases. Currently set to ~55° (10000 uint16) based on
minimizing Id RMS while running with block commutation.

### Math

All transforms use integer-only Q15 fixed-point arithmetic:
- 256-entry sin lookup table (Q15: -32768..32767 = -1.0..+1.0)
- cos via sin table with 90° offset
- Clarke: 1/√3 ≈ 18919/32768 (Q15)
- Inverse Clarke: √3/2 ≈ 28378/32768 (Q15)

Total overhead: ~660 bytes Flash for sin table, ~8 bytes RAM for FOC state.

## Current status

### What works
- Synchronized ADC sampling of phase currents (DMA triggered from timer update)
- Automatic zero-current offset calibration at startup (~200ms, motor off)
- Hall-based angle estimation with interpolation between transitions
- Clarke and Park transforms producing Id/Iq in the rotating frame
- PI controllers and inverse transforms producing 3-phase voltage output
- FOC control loop compiles and runs at 16kHz
- Motor produces torque under FOC — drew 1A, continued spinning when
  given a push by hand

### What doesn't work yet
- **Startup from standstill**: Motor cannot self-start under FOC. The angle
  interpolation needs at least one hall transition to estimate speed, so at
  zero RPM the angle is stuck and the voltage vector doesn't rotate.
  Need a startup strategy (e.g. block commutation until speed > threshold,
  then switch to FOC).
- **Angle offset tuning**: The ~55° offset was determined by sweeping offsets
  while running block commutation and comparing Id_rms vs Iq_rms. Results:

  | Offset | Id_rms | Iq_rms | Iq/Id ratio |
  |--------|--------|--------|-------------|
  | 0°     | 40.3   | 32.4   | 0.80        |
  | 30°    | 38.0   | 43.5   | 1.14        |
  | 55°    | 40.0   | 50.8   | 1.27 (best) |
  | 90°    | 40.4   | 45.7   | 1.13        |

  The remaining Id_rms (~40) doesn't change much across offsets because block
  commutation produces inherently non-sinusoidal currents. The offset may need
  further tuning once FOC is driving the motor (sinusoidal currents).
- **Strange noises under FOC**: Audible noise when FOC is active, likely from
  the angle offset being imperfect and/or the low hall sensor resolution (6
  steps per revolution) causing jerky torque output.
- **Overcurrent on first attempt**: With Kp=5, Ki=1, limit=1000 and no offset
  calibration, the controller drove full voltage and tripped the 1A PSU limit.
  Fixed by reducing gains to Kp=2, Ki=1, limit=500 and adding auto-calibration.

## Configuration

In `config.h`:
```c
#define BLDC_BC            // base commutation mode (always needed for startup)
#define FOC_ENABLED        // enable FOC control loop (comment out for block commutation)
```

## PI controller tuning

Current conservative values:
- Kp = 2 (PWM counts per ADC count of current error)
- Ki = 1 (integral gain, accumulated with >>10 shift = 1/1024 rate)
- Output limit = ±500 (max ±1125 = full PWM range)

These need increasing once startup and angle alignment are resolved. The motor
draws current but barely moves with these gains — they were chosen for safety
during initial testing.

## Angle offset calibration results

### Alignment test
`foc_align_rotor()` applies 500 PWM counts at 0° electrical for 800ms, reads
halls, computes offset. Found sector that gives **150° offset**.

### Sweep results (block commutation, observing FOC math)

| Offset | Id_rms | Iq_rms | Ratio | Note |
|--------|--------|--------|-------|------|
| 0°     | 40.3   | 32.4   | 0.80  | default |
| 55°    | 40.0   | 50.8   | 1.27  | manual guess |
| 90°    | 40.4   | 45.7   | 1.13  | |
| 120°   | 39.3   | 33.2   | 0.85  | |
| **150°** | **28.1** | **33.4** | **1.19** | **alignment result, lowest Id** |
| 180°   | 35.2   | 39.3   | 1.12  | |

150° gives clearly the best Id (28.1 vs ~40 for all others).

## FOC driving attempts

### Attempt 1: FOC only (Kp=5, limit=1000, no calibration)
- Immediate overcurrent, PSU tripped at 1A
- No offset calibration, hardcoded offsets wrong

### Attempt 2: FOC only (Kp=2, limit=500, auto calibration, 150° offset)
- Motor moved! Drew 1A, continued spinning when pushed by hand
- Horrible noise, voltage dropped 25→24V
- Id still large (up to -133), angle may need more tuning

### Attempt 3: Kp=1, limit=250
- Too weak, just clicking, can't overcome cogging torque

### Attempt 4: Kp=2, limit=350, with block commutation startup + warmup
- Alignment DC hold itself drew too much current
- Block commutation didn't start properly after alignment
- Motor ticked but didn't spin

## Current issues

1. **Alignment overcurrent**: The DC voltage hold during alignment draws too
   much current through the winding. Need either:
   - Much lower alignment voltage
   - PWM-based alignment (pulsed, not DC)
   - Skip alignment and hardcode 150° offset
2. **FOC transition**: Even with warmup timer, the transition from block
   commutation to FOC causes issues. The FOC controller immediately tries
   to correct current errors and overshoots.
3. **PI tuning**: Kp=2, limit=500 worked but was noisy. Kp=1, limit=250 was
   too weak. Need to find the sweet spot, possibly with different Id/Iq gains.

## Next steps

1. **Hardcode 150° offset** and skip alignment to avoid overcurrent at startup
2. **Reduce FOC iq_ref scaling** — currently `bldc_outputFilterPwm / 2` which
   may be too aggressive. Try `/4` or `/8`.
3. **Add current limiting in FOC** — if |Id| or |Iq| exceed a threshold,
   reduce voltage output regardless of PI
4. **PI gain tuning**: Try Kp=2 Ki=0 (P-only) first to avoid integral windup
5. **Consider open-loop voltage FOC** instead of current-loop FOC as a first
   step — set Vd=0, Vq=reference directly, no PI controllers needed
6. **Field weakening**: At high speeds, inject negative Id to extend speed range
   beyond the voltage limit. Requires knowing the motor's electrical parameters.

## Files

| File | Purpose |
|------|---------|
| `Inc/foc.h` | FOC types, function declarations |
| `Src/foc.c` | Transforms, PI controller, sin table, control loop |
| `Src/bldc.c` | Hooks FOC into CalculateBLDC ISR |
| `Src/bldcBC.c` | InitBldc: FOC init + offset calibration |
| `Inc/defines/defines_2-1-20.h` | PHASE_CURRENT_Y=PB0, PHASE_CURRENT_B=PB1 |
| `Inc/config.h` | FOC_ENABLED flag |

## RTT debugging

FOC state is logged via SEGGER RTT when REMOTE_DUMMY + RTT_REMOTE are enabled.
Output format:
```
26.72 V  pos:3  ang:159  Id: -19  Iq:   3  Iy:  16  Ib: -16  Oy:1993  Ob:2001
```
- pos: hall sector (1-6)
- ang: electrical angle in degrees (0-359)
- Id/Iq: rotating frame currents (should be: Id≈0, Iq=torque)
- Iy/Ib: raw phase currents (offset-corrected ADC counts)
- Oy/Ob: calibrated zero-current offsets

See `howto.md` for RTT connection commands.
