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

## PI controller (not yet working)

Values tried:
- Kp_d=2, Ki_d=1, limit=400 (flux)
- Kp_q=3, Ki_q=1, limit=400 (torque)
- iq_ref = bldc_outputFilterPwm / 8 with soft ramp

Issue: PI opposes motion. Need to verify Iq sign convention before re-enabling.

## Next steps

1. **Fix PI sign convention**: Log Id/Iq via RTT during open-loop at constant
   speed. Determine if Iq is positive or negative for positive torque direction.
   Then set iq_ref sign to match.
2. **P-only control first**: Try Kp only (Ki=0) to avoid integral windup while
   debugging the sign issue.
3. **Current scaling**: Determine ADC counts per amp so iq_ref has physical
   meaning. Use known PSU current + ADC reading.
4. **Reduce hall stepping noise**: The remaining vibration is from 60° hall
   resolution. Could add a low-pass on the angle estimate, or increase the
   sin table to 512 entries.
5. **Field weakening**: Inject negative Id at high speed.
6. **UART control**: Switch from REMOTE_DUMMY to REMOTE_UART + joystick for
   real-world testing.

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
