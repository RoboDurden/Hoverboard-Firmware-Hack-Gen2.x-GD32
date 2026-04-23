# bldcFOC greenfield brief

Write `Src/bldcFOC.c` from scratch. This brief gives you the facts, the
empirical constants that took a while to find, the interface contract, and a
target architecture. It deliberately does **not** hand you the implementation
line-by-line — the code choices are yours.

**Don't** modify `Src/bldc.c`, `Src/main.c`, `Src/setup.c`, `Src/it.c`,
`Inc/defines.h`, `Inc/config.h`, or the various `defines_*.h` — they're
project-wide and already wired for `BLDC_FOC`.

---

## Interface contract

`bldc.c` calls two virtual methods that you must implement:

```c
void InitBldc(void);                                         // once from main()
void bldc_get_pwm(int pwm, int pos, int *y, int *b, int *g); // each DMA ISR, 16 kHz
```

- `pwm` is the low-pass-filtered throttle (range `±BLDC_TIMER_MID_VALUE ≈ ±1125`).
- `pos` is the hall-decoded 6-step position (1..6, invalid filtered to 0 by `bldc.c`).
- `*y`, `*b`, `*g` are PWM compare **deltas** from the timer midpoint, range `±BLDC_TIMER_MID_VALUE`. `bldc.c` adds the midpoint and clamps.

`Inc/bldcFOC.h` exposes nothing beyond `#include "bldc.h"`. Everything else
lives as file-local statics inside `bldcFOC.c`. In particular, do **not**
introduce a `foc_sensor_update`-style hook that `bldc.c` has to call — that's
a leak. Do the sensing at the top of `bldc_get_pwm`.

---

## Hardware facts (verified empirically on layout 2-1-20)

- **MCU**: GD32F130C8T6 — 72 MHz Cortex-M3, no FPU, 8 KB RAM, 64 KB flash.
- **Motor**: 15 pole pairs, 3-phase BLDC, hoverboard hub motor.
- **PWM**: 16 kHz center-aligned on TIMER0. Period 2250, `BLDC_TIMER_MID_VALUE = 1125`, dead time 60 ticks.
- **Phase-current shunts**: 2× R004 (4 mΩ) low-side, dual op-amp ≈ 26× effective gain. Only two phases shunted; derive the third via Kirchhoff.
- **ADC-to-wire mapping** (confirmed by forced-commutation `pos_force` experiment, see `phase_mapping_experiment.py`):
  - `adc_buffer.phase_current_a` (pin **PB0**) → low-side shunt on the **BLUE** wire (phase driven by `BLDC_BH/BL`).
  - `adc_buffer.phase_current_b` (pin **PA0**) → low-side shunt on the **YELLOW** wire (phase driven by `BLDC_YH/BL`).
  - **GREEN** wire has no shunt; reconstruct as `i_green = −(i_yellow + i_blue)`.
- **ADC bias**: ≈ 2000 counts (op-amp midrail). Calibrate offsets at boot by averaging N samples while the PWM output is still disabled.
- **Current sensitivity**: ≈ 128 LSB/A (4 mΩ × 26× / (3.3 V / 4096)).
- **ADC trigger**: hardware via TIMER2 slave of TIMER0 → `T2_TRGO`. Samples near the PWM valley (all low-side FETs on → phase inductor currents equal shunt currents).
- **Hall sensors**: 3, 6 sectors per electrical rev.
- **Bench PSU**: typically 27 V.

Globals that already exist in the project and `bldcFOC.c` may read:

```c
extern uint8_t          wState;       // remote state; bits 0-1 = requested mode
extern volatile uint8_t hall;         // 3-bit hall state, updated each ISR in bldc.c
extern int32_t          steer;        // remote ±1000 — free for FOC tuning (tank-mix bypassed)
extern adc_buf_t        adc_buffer;   // DMA-filled; fields .phase_current_a, .phase_current_b
```

---

## Mode codes (from `wState & 0x03`)

Ordering matches the reference `hoverboard-firmware-hack-FOC` project:

```
0 = BC   (block commutation, FOC off)
1 = VLT  (voltage FOC)
2 = SPD  (speed FOC — stub, fall through to BC)
3 = TRQ  (torque FOC)
```

Bit 5 of `wState` (value 32) is `STATE_LedBattLevel` — don't touch it. The
joystick controller already sets bit 5 and fills bits 0-1 with the mode.

---

## Architecture — suggested types

You're free to pick names, but I'd recommend this shape:

```c
typedef struct { int16_t alpha, beta; } ab_t;   // stator frame: currents or voltages
typedef struct { int16_t d,     q;    } dq_t;   // rotor frame

// Per-ISR snapshot. Lives on the stack inside bldc_get_pwm.
typedef struct {
    int16_t iy, ib;        // measured phase currents, offset-subbed
    ab_t    i_stator;      // after Clarke
    dq_t    i_rotor;       // after Park
    int16_t theta;         // electrical angle used (deg, 0..359) — one θ per ISR
} foc_state_t;
```

Transform primitives as **pure functions** (inputs → output, no hidden writes):
`clarke(iy, ib) → ab_t`, `park(ab, θ) → dq_t`, `ipark(dq, θ) → ab_t`,
`iclarke_svpwm(ab, *y, *b, *g)`. Mode handlers take `const foc_state_t *` and
write the PWM deltas. A file-local `foc_sample(pos) → foc_state_t` does the
sense half and updates persistent statics; `bldc_get_pwm` calls it then
dispatches on `mode_req`. Keep `get_pwm_bc` self-contained (don't depend on
`bldcBC.c`).

---

## Persistent state boundary

File-local statics, kept small and deliberate:

- **PLL**: current angle, Q8 accumulator, Q8 velocity, prev hall, prev pos, ISRs-since-edge counter.
- **Calibration**: offsets for channels A/B, running sums, sample count, done flag.
- **EMAs** of Id/Iq for display (see averaging below).
- **Mode tracking**: `mode_req`, `prev_mode` (for integrator reset on transitions).
- **TRQ PI**: integrators and `iq_target`. `volatile int16_t foc_trq_v_max` for runtime poking.
- **Diagnostics**: raw ADC latches and `vq_cmd` (for RTT log), `volatile uint8_t pos_force` (bench override, forces block commutation to a chosen pos 1..6).

Per-ISR transients — `iy`/`ib`, `i_alpha`/`i_beta`, `i_d`/`i_q`, `theta` —
live in the snapshot on the stack, not in statics.

---

## Q15 fixed-point

No floats (no FPU). Q15 = 16-bit signed integer scaled by 2¹⁵ = 32768.

- Represent fractional constants as `round(k × 32768)`.
- Multiply Q15 × int16 with a widening `int32_t` cast, then `>> 15` to rebase.
- Addition of Q15 values needs no shift.

Constants you'll need:

```
Q15_INV_SQRT3      = 18919    (1/√3  × 2^15)
Q15_TWO_OVER_SQRT3 = 37837    (2/√3  × 2^15)  ── doesn't fit in int16; keep in int32 multiplies
Q15_HALF           = 16384    (0.5   × 2^15)
Q15_SQRT3_OVER_2   = 28378    (√3/2  × 2^15)
```

---

## Clarke (2-shunt)

Standard Clarke with α-axis aligned to phase "a" (yellow, in our mapping):

```
Iα = Ia
Iβ = (Ib − Ic) / √3
```

With star-connected windings `Ia + Ib + Ic = 0`, so `Ic = −(Ia + Ib)`, giving:

```
Iβ = (Ia + 2·Ib) / √3
```

Plug in `Ia = Iy` (yellow), `Ib = Ib_wire` (blue). Use the two Q15 constants
above; keep intermediates in `int32_t`.

---

## Park — sign conventions and the 2× oscillation test

There are **two self-consistent Park families** in the literature:

- **Forward Park = R(−θ), inverse Park = R(+θ)** — common in motor-drive textbooks (Krause, Leonhard)
- **Forward Park = R(+θ), inverse Park = R(−θ)** — some control-systems texts

Either is correct if you pick compatible signs for everything else (hall-to-angle direction, α-axis choice, Clarke β sign, and the direction PLL reports as "forward"). Get the **combination** wrong and Id/Iq oscillate sinusoidally at 2× electrical frequency instead of settling to DC.

**The one invariant you must preserve**: the Park+Inverse-Park chain is the
identity on (d, q) — i.e. whichever matrix you pick for Park, inverse Park is
the **transpose** of it, using the *same θ value*.

**Bring-up test**: spin the motor in VLT at constant throttle. In the RTT
plot, `Iq avg` (green) and `Id avg` (red) should be nearly DC (flat
horizontal lines) at steady state. If instead you see them drawing sinusoids,
your Park matrix is the wrong member of the pair — transpose it (swap the sin
signs) and retest. Nothing else in the stack needs to change: forward/reverse
symmetry, magnitude, steady-state — all driven by angle offset and scaling,
not Park sign.

`FOC_ANGLE_OFFSET_BASE = 125°` is the empirical value for this motor under
the conventions we settled on (α = yellow, `electrical_angle` increases with
forward rotation, 15 pole pairs, standard Clarke). Pick compatible
conventions and expect a similar-order offset; pick opposite ones and you'll
get a different offset but still land on a usable one after VLT bring-up.

---

## Inverse Clarke

α-axis is yellow, so:

```
Vy = Vα
Vb = −Vα/2 + (√3/2)·Vβ
Vg = −Vα/2 − (√3/2)·Vβ
```

Bundle this with SVPWM centering and the PWM scale into one call
(`iclarke_svpwm`).

---

## SVPWM — min-max common-mode injection

After inverse Clarke, subtract `(max(Vy, Vb, Vg) + min(Vy, Vb, Vg)) / 2`
from each phase. Extends linear modulation range ~15% over raw sinusoidal
PWM — same waveform as sector-based SVM, simpler arithmetic. The reference
hoverboard FOC project does this too. Not 3rd-harmonic injection; min-max
injection *is* a form of zero-sequence that's effectively equivalent.

---

## Internal-voltage → PWM scaling

Peak phase-to-centre voltage after inverse Park + inverse Clarke + SVPWM
centering is about **0.866 × |Vq|** (when Vd ≈ 0). With your Vq clamp set at
`FOC_VQ_MAX = 20000`, peak internal is ~17 300. To map that to roughly the
full ±1125 PWM range:

```
peak_pwm = 0.866 × |Vq| / 2^FOC_OUT_SHIFT
         = 0.866 × 20000 / 16
         ≈ 1083
```

So `FOC_OUT_SHIFT = 4`. (Earlier we tried shift 5 → peak ~540; VLT was
visibly slower than BC. Shift 4 is the right setting; clamp catches extremes.)

---

## Sine table

Use a Q15 sine lookup, generated once by `generate_sine_table.py` and pasted
inline. 1° resolution is enough — at 16 kHz and ~250 Hz max electrical, you
advance ~5.6° per ISR; sub-degree resolution gains nothing.

### Full table (simpler, 720 B flash)

```c
static const int16_t sine_q15[360] = { /* from the script */ };

static inline int16_t wrap_angle(int16_t a) {
    while (a >= 360) a -= 360;
    while (a < 0)    a += 360;
    return a;
}

static inline int16_t sin_q15(int16_t a) { return sine_q15[wrap_angle(a)]; }
static inline int16_t cos_q15(int16_t a) { return sine_q15[wrap_angle(a + 90)]; }
```

### Half table (360 B flash)

Exploit `sin(θ + 180°) = −sin(θ)`. Store 180 entries (0..179°) and flip sign
for the upper half:

```c
static const int16_t sine_q15_half[180] = { /* first half of the full table */ };

static inline int16_t sin_q15(int16_t a) {
    a = wrap_angle(a);                           // 0..359
    return (a < 180) ? sine_q15_half[a]
                     : -sine_q15_half[a - 180];
}
static inline int16_t cos_q15(int16_t a) { return sin_q15(a + 90); }
```

Saves 360 B flash, adds one branch per call (trivial on M3, predictable).

### Quarter table (182 B flash, more complex)

`sin(180° − θ) = sin(θ)` and `sin(360° − θ) = −sin(θ)` let you fold further
to 0..90°. Needs two branches; marginal gain (~180 B) for more complexity.
Only worth it if flash is tight. Ours isn't (~35 KB free).

I'd use the half table for the rewrite unless you have a reason to burn the
extra flash. Update `generate_sine_table.py` to emit 180 entries (or just
paste the first 180 of the 360 output).

---

## Hall PLL

First-order observer. `bldc.c` already decodes `hall` → `pos` (1..6) via its
`hall_to_pos[]`. Your job is to turn that into a continuous
`electrical_angle` (degrees, wraps at 360).

Sector centres (degrees), arbitrary absolute base — the true rotor alignment
is absorbed into `FOC_ANGLE_OFFSET_BASE`:

```
pos 1 → 180°
pos 2 → 240°
pos 3 → 300°
pos 4 →   0°
pos 5 →  60°
pos 6 → 120°
```

Direction from the `pos` sequence: forward is `1 → 2 → 3 → 4 → 5 → 6 → 1`. A
delta of ±1 (or ∓5 due to wrap) tells you direction; any other delta is
noise — leave velocity alone for that edge.

On each hall edge (when `hall` differs from last seen):

1. Count ISRs spent in the just-finished sector (`sector_dwell`).
2. If direction ≠ 0 and dwell > 0:
   `velocity_q8 = (dir × 60 × 256) / sector_dwell` — clamp to `±FOC_PLL_VEL_CLAMP = 3000` (Q8 deg/ISR).
3. Snap angle to the sector **boundary** the rotor just crossed, **not the
   centre**. Forward → lower boundary (`center − 30°`); reverse →
   upper (`center + 30°`). Wrap. *Without* the direction-aware snap, forward
   and reverse ideal-offset values differed by ~60° (one sector width) — a
   subtle bug that cost bring-up time.
4. Reset `sector_dwell = 0`, remember new hall / pos.

Between hall edges:

- `sector_dwell += 1`
- `angle_accum_q8 += pll_velocity_q8`, wrap into `[0, 360 << 8)`
- `electrical_angle = angle_accum_q8 >> 8`

First-call seeding (`prev_hall == 0xFF`): set angle to the sector centre,
velocity zero, and record initial `pos` / `hall`.

---

## Sense half — `foc_sample(pos) → foc_state_t`

Responsibilities, in order:

1. Latch `mode_req = wState & 0x03`.
2. Read raw ADCs; mirror into `adc_raw_a/b` for RTT.
3. **Calibration**: sum ADCs for the first 1024 samples and compute the means as offsets, then flip a `calib_done` flag. While calibrating, return a zero-filled snapshot early so downstream doesn't see garbage.
4. Subtract offsets to get `iy` (from channel B — yellow) and `ib` (from channel A — blue). Derive `ig = −(iy + ib)` if you want it for the log.
5. Update the PLL (on `pos` 1..6). Hall edges + interpolation.
6. Compute `theta = wrap_angle(electrical_angle + FOC_ANGLE_OFFSET_BASE)`. Use this same `theta` for Park *and* for the mode handler's inverse Park.
7. Clarke (`iy`, `ib`) → `i_stator`. Park (`i_stator`, `theta`) → `i_rotor`.
8. Update Id/Iq EMAs for display: `reg += (sample − (reg >> AVG_SHIFT))` or equivalent; time constant ≈ 2^10 samples at 16 kHz ≈ 64 ms. Read as `reg >> AVG_SHIFT`.
9. Emit RTT log every `FOC_RTT_LOG_DIV = 160` samples (= 100 Hz). Format below.
10. Fill and return the snapshot.

Why PLL first, then θ, then Park: gives Park and inverse Park one consistent
angle. Previously the old code did Park with stale `electrical_angle` and
inverse Park with the freshly-updated angle — not a bug for VLT but avoidable
in a clean implementation.

---

## BC mode

Self-contained, same 6-step table as `bldcBC.c`. Don't depend on
`bldcBC.c` — FOC build excludes it. The table (signed PWM delta applied to
each phase):

```
pos 1: y =    0, b = +pwm, g = −pwm
pos 2: y = −pwm, b = +pwm, g =    0
pos 3: y = −pwm, b =    0, g = +pwm
pos 4: y =    0, b = −pwm, g = +pwm
pos 5: y = +pwm, b = −pwm, g =    0
pos 6: y = +pwm, b =    0, g = −pwm
```

`pos_force` (bench override) routes to BC at the forced pos, regardless of
requested mode.

---

## VLT mode

Open-loop voltage. Throttle linearly maps to Vq with Vq clamped to
`±FOC_VQ_MAX`. Vd is hardcoded to 0. Invariants:

- The measured Id/Iq from the snapshot are **not fed back**; current sensing
  is diagnostic only.
- Motor behaviour depends entirely on `theta` tracking the rotor correctly —
  tune `FOC_ANGLE_OFFSET_BASE` (via `steer` axis) until `Iq avg` dominates
  and `Id avg` stays near zero on the plot.

Output path: `ipark(Vd=0, Vq, theta)` → inverse Clarke → SVPWM centre → PWM compares.

VLT should work even without current sensors defined (`#if defined(PHASE_CURRENT_*)` guards in the sense half): skip ADC/Clarke/Park, still compute θ, still drive PWM. The plot will just show the raw ADC channels dormant.

---

## TRQ mode

Closed-loop Iq with an Iq PI and an Id PI. Throttle linearly maps to
`iq_target`; Id target is 0 (maximise torque per amp). Each PI produces its
axis voltage; chain the same way as VLT (inverse Park with the same θ).

Constants (empirical starting points; bench-tunable):

- `FOC_IQ_MAX = 500` (≈ 3.9 A target at full throttle).
- `volatile int16_t foc_trq_v_max = 3000` — PI output clamp for Vd/Vq, **tighter than `FOC_VQ_MAX`**. If angle is off, the PI will wind the output all the way to this clamp trying to coerce measured Iq → target, so this — not `FOC_IQ_MAX` — is what caps worst-case stall current. Keep `volatile` so you can poke it via OpenOCD `mwh` on the bench without re-flashing.
- PI gains as right-shifts: Kp shift = 0 (Kp=1), Ki shift = 10 (integrator += err >> 10 per ISR).
- Anti-windup shift = 2: when output saturates to the clamp, subtract `excess >> 2` from the integrator (back-calculation).
- Reset both integrators whenever `mode_req != prev_mode` — stops a prior session's wind-up from biasing the first tick's output.

The rest of the output chain is identical to VLT (inverse Park → iclarke_svpwm).

---

## Empirical constants summary

```
FOC_CALIB_SAMPLES     1024    # ADC offset average N samples at boot
FOC_VQ_MAX            20000   # VLT Vq clamp (internal Q scale)
FOC_ANGLE_OFFSET_BASE 125     # degrees — empirical for this motor
FOC_OUT_SHIFT         4       # internal voltage → PWM delta
FOC_PLL_VEL_CLAMP     3000    # Q8 deg/ISR sanity clamp
FOC_RTT_LOG_DIV       160     # 100 Hz log rate
FOC_AVG_SHIFT         10      # EMA time constant ≈ 2^10 samples
FOC_IQ_MAX            500     # TRQ iq_target max (~3.9 A)
FOC_IQ_KP_SHIFT       0
FOC_IQ_KI_SHIFT       10
FOC_ID_KP_SHIFT       0
FOC_ID_KI_SHIFT       10
FOC_AW_SHIFT          2
foc_trq_v_max         3000 (volatile)
pos_force             0    (volatile)
```

---

## RTT log format (must match `rtt_plot.py` regex — don't reorder)

One line at `FOC_RTT_LOG_DIV` rate:

```
m:<MODE> adc_a=<raw_a> adc_b=<raw_b> Ia:<iy> Ib:<ib> Ialpha:<α> Ibeta:<β> Id:<d> Iq:<q> Idavg:<d̄> Iqavg:<q̄> ang:<θ> rpm:<rpm> Vq:<vq_cmd> p:<pos> f:<pos_force>
```

`<MODE>` is "BC" / "VLT" / "SPD" / "TRQ". RPM from PLL velocity:
`rpm ≈ pll_velocity_q8 / 1.44 ≈ (pll_velocity_q8 × 25) / 36` (integer form).

`#ifdef RTT_REMOTE` guard the whole log block. `sprintf` into a ~220-byte
char buffer; `SEGGER_RTT_WriteString(0, buf)`.

---

## Verification

What "working" means:

| Test | Expected |
|------|----------|
| Build | Clean compile, ~45% flash, ~33% RAM |
| BC mode (btn 24 off) | Same behaviour as pre-FOC block commutation |
| VLT, low throttle | RPM within a few % of BC at the same throttle |
| VLT, high throttle | ~15% more RPM than BC at same throttle |
| VLT Id/Iq plot at steady RPM | Flat `Id avg` / `Iq avg` lines, **not** sinusoidal |
| VLT forward ↔ reverse | Ideal steer trim similar magnitude both ways (thanks to direction-aware snap) |
| TRQ at throttle=0 | Quiet, near-zero current draw |
| TRQ mode-change | No kick when entering/leaving (integrators reset) |
| TRQ with bad angle offset | Current capped at ~`foc_trq_v_max`-driven stall; won't brown out the supply |

Failure fingerprints:

- Id/Iq oscillating sinusoidally → Park matrix is the wrong sign variant (transpose it).
- VLT slower than BC at same throttle → scaling shift is too large; check math around 0.866·|Vq| / 2^FOC_OUT_SHIFT reaching ~±1125.
- Forward/reverse asymmetry on ideal trim → direction-aware hall snap missing.
- Motor brown-outs on TRQ engage → `foc_trq_v_max` too high, or angle offset so wrong the Id PI can't compensate. Lower `foc_trq_v_max` first, then fix alignment.

---

## Non-goals

Defer (don't implement now, but leave the structure amenable):

- Mode hysteresis (OPEN↔FOC thresholds at 240/480 RPM in the reference)
- Iq protection limiter with speed-dependent `Vq_max_M1[]` LUT
- SPD mode (outer speed PI on top of TRQ)
- Fast-capture RAM buffer
- Adaptive angle-offset learning
- Saving calibrated offsets to flash
- Field weakening

SPD mode: stub — fall through to BC if requested.
