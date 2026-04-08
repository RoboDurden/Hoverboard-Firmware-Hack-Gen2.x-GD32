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

## Next steps

1. **Startup strategy**: Use block commutation from standstill until speed
   exceeds a threshold (~50 RPM?), then transition to FOC. Need smooth
   handover to avoid torque glitches.
2. **Angle offset fine-tuning**: Once motor runs under FOC, adjust offset
   to minimize Id under load. Could implement auto-tuning: slowly sweep offset
   and find the angle that maximizes Iq for a given voltage.
3. **PI gain tuning**: Increase Kp and Ki once the motor runs stably. May need
   different gains for Id (fast response, drives to 0) vs Iq (tracks torque ref).
4. **Current scaling/calibration**: Convert ADC counts to actual amps using
   known shunt resistance and op-amp gain. Currently everything is in raw
   ADC counts.
5. **Field weakening**: At high speeds, inject negative Id to extend speed range
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
