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

One additional export — **not** called from `bldc.c`, only from `main.c`'s
main loop:

```c
void FocRttPoll(void);                                       // main-loop tick; formats+emits deferred RTT log
```

`Inc/bldcFOC.h` exposes these three symbols and nothing else. All internals
live as file-local statics inside `bldcFOC.c`. In particular, do **not**
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
Other bits (2–4, 6–7) are joystick-managed status / LED bits — not FOC's
concern. **FOC only reads bits 0-1, and only reads — never writes `wState`.**

**Latching discipline**: `wState` is written by the UART ISR
(`remoteUart.c:179` writes it on each incoming joystick packet) and read by
the FOC DMA ISR. Since `wState` is `uint8_t`, loads and stores are atomic
on M3 (single LDRB/STRB) — no torn reads possible. But within a single
`bldc_get_pwm` call, a UART-ISR preempt between two reads would split
behaviour across the sense and dispatch halves of the same tick. **Latch
exactly once per FOC ISR**, at the top of `foc_sample` (step 1), and use
the latched `mode_req` everywhere downstream. Do not re-read `wState`
inside mode handlers or dispatch.

---

## Architecture — suggested types

You're free to pick names, but I'd recommend this shape. Every FOC stage has
a typed output, so the pipeline reads as a sequence of frame conversions
(phase → αβ → dq) and the type system catches a rotor-frame value being
passed where a stator-frame value was expected.

```c
typedef struct { int16_t iy,    ib;   } iph_t;  // phase frame: measured currents (yellow, blue)
typedef struct { int16_t alpha, beta; } ab_t;   // stator frame (αβ): currents or voltages
typedef struct { int16_t d,     q;    } dq_t;   // rotor  frame (dq):  currents or voltages

// Per-ISR snapshot. Lives on the stack inside bldc_get_pwm.
typedef struct {
    iph_t   i_phase;   // ADC → offset-subbed (phase frame)
    ab_t    i_stator;  // after Clarke
    dq_t    i_rotor;   // after Park
    int16_t theta;     // electrical angle used (deg, 0..359) — one θ per ISR
} foc_state_t;
```

**Units gotcha.** `iph_t` fields are **integer ADC counts** (offset-subtracted,
≈128 LSB/A, range ~±2000 around zero). They are *not* Q15. `ab_t` and `dq_t`
carry the same physical quantity after linear transforms, so they are also in
ADC-count units, not Q15. The Q15 scaling lives in the *constants* inside
Clarke/Park (`Q15_INV_SQRT3`, sine-table entries) — multiply int16 × Q15-const
with an int32 widening and `>> 15` to rebase, and the result is still in
ADC-count units. Only the sine table entries and the named Q15 constants are
Q15-scaled. Mixing the two families (treating a current as Q15, or a Q15
constant as counts) gives results off by 2¹⁵.

Transform primitives as **pure functions** (inputs → output, no hidden writes):
`clarke(iph_t) → ab_t`, `park(ab_t, θ) → dq_t`, `ipark(dq_t, θ) → ab_t`,
`iclarke_svpwm(ab_t, *y, *b, *g)`. Mode handlers take `const foc_state_t *`
and write the PWM deltas. A file-local `foc_sample(pos) → foc_state_t` does
the sense half and updates persistent statics; `bldc_get_pwm` calls it then
dispatches on `mode_req`. Keep `get_pwm_bc` self-contained (don't depend on
`bldcBC.c`).

Two stateful helpers that mutate via pointer — still pure w.r.t. globals,
since state arrives via the pointer and nothing else is touched:

```c
typedef struct {
    int8_t kp_shift;   // Kp term = err >> kp_shift  (0 → Kp·err; negative → left-shift for Kp>1)
    int8_t ki_shift;   // integrator += err >> ki_shift each step
    int8_t aw_shift;   // on saturation: integrator -= excess >> aw_shift  (back-calc anti-windup)
} pi_gains_t;

typedef struct {
    int32_t integrator;    // int32 — err over many ISRs accumulates past int16 range
} pi_state_t;

typedef struct {
    int32_t  angle_accum_q8;   // Q8 degrees; wraps in [0, 360 << 8) = [0, 92160)
    int16_t  velocity_q8;      // Q8 deg/ISR; clamped to ±FOC_PLL_VEL_CLAMP
    int8_t   prev_pos;         // 0 = uninitialised sentinel; else 1..6
    uint16_t sector_dwell;     // ISRs since last hall edge
} pll_state_t;

int16_t pi_step (pi_state_t  *state, int16_t err, int16_t clamp, pi_gains_t gains);
void    pll_step(pll_state_t *state, int8_t pos);
```

Rationale for the shapes — the "compute, don't store" principle drives most of it:

- **`pll_state_t` drops `prev_hall`, `direction`, `electrical_angle`** — all
  derivable. `prev_pos == 0` is the first-call sentinel (pos is already
  sanitised by `bldc.c`); direction computes locally from `pos` delta on each
  edge; electrical angle is `state.angle_accum_q8 >> 8` at the callsite.
- **`pll_step` runs every ISR, including `pos == 0`** — advances the
  accumulator with last-known velocity and increments `sector_dwell`; just
  skips the edge branch. Transient hall glitches don't freeze θ and don't
  break the dwell count.
- **Integrator is `int32_t`** — anti-windup bounds it, but with err up to
  ±2500 counts and multi-ISR accumulation, `int16` would saturate inside
  the bound.
- **Gains passed by value** — same function serves Iq and Id even if their
  gains later diverge; host tests can sweep. Zero M3 cost (AAPCS packs the
  3-byte struct into one register; `-O2` inlines + constant-propagates to
  the same code as a compile-time `#define`).
- **Reset via `state->integrator = 0`** at the callsite — caller handles
  mode-transition resets explicitly. No hidden reset logic inside `pi_step`.
- **Designated-initializer init**: `pll_state_t pll = { .prev_pos = 0 };`,
  `pi_state_t iq_pi = { 0 };`. No separate `*_init` functions.

---

## File split — `bldcFOC.c` + `bldcFOC_core.c/.h`

Split the module into two translation units so the pure helpers compile
clean on a host (Mac, Linux) for unit tests, no GD32 SPL or `#ifdef`
contortions required.

**`Src/bldcFOC_core.c` + `Inc/bldcFOC_core.h`** — pure, no hardware/globals:
- Types: `iph_t`, `ab_t`, `dq_t`, `foc_state_t`, `pi_gains_t`, `pi_state_t`, `pll_state_t`
- Q15 constants, half sine table, `wrap_angle`, `sin_q15`, `cos_q15`
- `clarke`, `park`, `ipark`, `iclarke_svpwm`
- `pi_step(state, err, clamp, gains)` and `pll_step(state, pos)` — mutate
  through the state pointer; signatures and rationale in "Architecture —
  suggested types".

**`Src/bldcFOC.c`** — glue that touches hardware or project globals:
- `InitBldc` (reads `adc_buffer` for offset calibration on first N ISRs)
- `bldc_get_pwm` (reads `wState`, `hall`, `steer`, `adc_buffer`; calls core
  helpers; writes `*y`, `*b`, `*g`). ISR-side log work: populate
  `log_snapshot`, set `log_pending` — **no `sprintf`** (see "Deferred RTT log").
- `FocRttPoll` (main-loop only): `sprintf` the snapshot, `SEGGER_RTT_Write`
- File-local statics holding PLL state, calibration state, PI integrators,
  mode tracking, EMAs, diagnostics, deferred-log snapshot — the actual
  persistent state

Rule of thumb: if the function's only inputs are its arguments and its only
output is its return value, it belongs in `_core`. If it touches a project
global, the ADC buffer, the RTT API, or writes into the ISR's `*y`/`*b`/`*g`
via pointer args, it belongs in the top-level `bldcFOC.c`.

`Inc/bldcFOC.h` stays as-is (just `#include "bldc.h"`). `Inc/bldcFOC_core.h`
declares the pure helpers and the types. `bldcFOC.c` includes both; host
tests include only `bldcFOC_core.h`.

---

## Persistent state boundary

File-local statics, kept small and deliberate:

- **PLL**: a single `pll_state_t` (accumulator, velocity, prev_pos, sector_dwell — see Architecture section).
- **Calibration**: offsets for channels A/B, running sums, sample count (incl. the 32-sample discard prefix), `calib_done` flag.
- **EMAs** of Id/Iq for display (see averaging below).
- **Mode tracking**: `mode_req`, `prev_mode` (for integrator reset on transitions). **Init `prev_mode = 0xFF`** (sentinel) so the first post-calibration dispatch unconditionally triggers an integrator reset, regardless of whether the user is holding BC/VLT/TRQ at boot.
- **TRQ PI**: two `pi_state_t` (Iq and Id integrators), plus `iq_target` and `volatile int16_t foc_trq_v_max` for runtime poking.
- **Diagnostics**: raw ADC latches and `vq_cmd` (for RTT log), `volatile uint8_t pos_force` (bench override, forces block commutation to a chosen pos 1..6).
- **Deferred log**: `volatile log_snapshot_t log_snapshot`, `volatile uint8_t log_pending`, `log_counter` (ISR-side tick counter for `FOC_RTT_LOG_DIV`). See "Deferred RTT log".

Per-ISR transients — `i_phase`, `i_stator`, `i_rotor`, `theta` — live in the
snapshot (`foc_state_t`) on the stack, not in statics.

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

Q15 sine lookup, generated once by `generate_sine_table.py` and pasted
inline. 1° resolution is enough — at 16 kHz and ~250 Hz max electrical, you
advance ~5.6° per ISR; sub-degree resolution gains nothing.

**Use the half table** (180 entries, 360 B flash). Exploits
`sin(θ + 180°) = −sin(θ)`:

```c
static const int16_t sine_q15_half[180] = { /* first 180 entries of the script output */ };

static inline int16_t wrap_angle(int16_t a) {
    while (a >= 360) a -= 360;
    while (a < 0)    a += 360;
    return a;
}

static inline int16_t sin_q15(int16_t a) {
    a = wrap_angle(a);                             // 0..359
    return (a < 180) ? sine_q15_half[a]
                     : -sine_q15_half[a - 180];
}
static inline int16_t cos_q15(int16_t a) { return sin_q15(a + 90); }
```

One predictable branch per call, trivial on M3. Saves 360 B over the full
table for no behavioural cost.

Alternatives (not used, noted for completeness): the full 360-entry table is
one byte simpler (no branch) but burns 720 B; the quarter table folds further
via `sin(180°−θ) = sin(θ)` to 90 entries (182 B) but needs two branches —
not worth the complexity at ~35 KB flash free.

---

## Hall PLL

First-order observer. `bldc.c` already decodes `hall` → `pos` (1..6, or 0
for invalid) via its `hall_to_pos[]`. Your job is to turn that into a
continuous `electrical_angle` (degrees, wraps at 360). `pll_step(&state,
pos)` is called every ISR from the sense half, including on `pos == 0`.

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

Every call (regardless of `pos` validity):

- `sector_dwell += 1`
- `angle_accum_q8 += velocity_q8`, wrap into `[0, 360 << 8)`

So `electrical_angle = state.angle_accum_q8 >> 8` at the callsite keeps
advancing on transient hall glitches (`pos == 0`) with last-known velocity.
Nothing in the edge branch below runs when `pos == 0`.

On each new valid `pos` (`pos != 0 && pos != state.prev_pos`):

1. **First-call seeding** (`state.prev_pos == 0`): set `angle_accum_q8` to
   the sector centre (`<< 8`), `velocity_q8 = 0`, `prev_pos = pos`,
   `sector_dwell = 0`, and return.
2. Otherwise: compute direction from `(prev_pos → pos)`. Delta ±1 (or ∓5 via
   wrap) → ±1; anything else → 0 (glitch, bail out of the rest without
   updating `prev_pos` or `sector_dwell`, so the next valid edge still sees
   an accurate dwell count).
3. If direction ≠ 0 and `sector_dwell > 0`:
   `velocity_q8 = (dir × 60 × 256) / sector_dwell` — clamp to
   `±FOC_PLL_VEL_CLAMP = 3000` (Q8 deg/ISR).
4. Snap `angle_accum_q8` to the sector **boundary** the rotor just crossed,
   **not the centre**. Forward → lower (`center − 30°`); reverse →
   upper (`center + 30°`). Wrap. *Without* the direction-aware snap,
   forward and reverse ideal-offset values differed by ~60° (one sector
   width) — a subtle bug that cost bring-up time.
5. `prev_pos = pos`, `sector_dwell = 0`.

---

## Sense half — `foc_sample(pos) → foc_state_t`

Responsibilities, in order:

1. Latch `mode_req = wState & 0x03`.
2. Read raw ADCs; mirror into `adc_raw_a/b` for RTT.
3. **Calibration** (boot-time, one-shot). Discard the first ~32 ADCs to let the analog front-end settle, then accumulate the next `FOC_CALIB_SAMPLES = 1024` ADCs into running sums. When the count reaches 1024, compute means as offsets and flip `calib_done`. While `!calib_done`: skip steps 4 and 7 (currents/Clarke/Park) — offsets aren't known yet — and return a snapshot with `i_phase`, `i_stator`, `i_rotor` zeroed. **Still run steps 5, 6, 8, 9** — the PLL updates from ISR #1 so θ is warm when FOC engages (handles the case where someone pushes the wheel during boot).
4. Subtract offsets into `i_phase`: `i_phase.iy = channel_B − offset_B` (yellow), `i_phase.ib = channel_A − offset_A` (blue). Derive `ig = −(iy + ib)` on the fly if you want it for the log — it lives in no struct (diagnostic only).
5. Update the PLL (on `pos` 1..6). Hall edges + interpolation.
6. Compute `theta = wrap_angle(electrical_angle + FOC_ANGLE_OFFSET_BASE)`. Use this same `theta` for Park *and* for the mode handler's inverse Park.
7. `clarke(i_phase) → i_stator`. `park(i_stator, theta) → i_rotor`.
8. Update Id/Iq EMAs for display: `reg += (sample − (reg >> AVG_SHIFT))` or equivalent; time constant ≈ 2^10 samples at 16 kHz ≈ 64 ms. Read as `reg >> AVG_SHIFT`.
9. Every `FOC_RTT_LOG_DIV = 160` samples (= 100 Hz): if `!log_pending`, populate the `log_snapshot` struct with this ISR's values and set `log_pending = 1`. **No `sprintf` or RTT write in the ISR** — a main-loop poller formats and emits. See "Deferred RTT log" below.
10. Fill and return the snapshot.

Why PLL first, then θ, then Park: gives Park and inverse Park one consistent
angle. Previously the old code did Park with stale `electrical_angle` and
inverse Park with the freshly-updated angle — not a bug for VLT but avoidable
in a clean implementation.

---

## Mode dispatch & calibration gating

Inside `bldc_get_pwm`, **short-circuit the mode dispatch while `!calib_done`**:
write `*y = *b = *g = 0` and return, regardless of `mode_req`. No PI updates,
no inverse-Park/Clarke, no SVPWM. Consequences:

- User throttle and mode request are ignored for the ~64 ms calibration
  window. Silent and benign — the next ISR after `calib_done` behaves normally.
- PI integrators never tick during calibration, so no wind-up on first engage.
- `bldc.c` already gates PWM outputs off at boot via `bldc_enable == RESET`
  (see bldc.c:214–230), so even the midpoint PWM we emit goes nowhere. If
  `bldc_enable` flipped on mid-calibration (shouldn't happen in practice),
  midpoint PWM = three phases at 50% duty = no net current = still safe for
  offset sampling.

After `calib_done`, dispatch on `mode_req` as documented in the BC / VLT /
TRQ sections below (with `pos_force != 0` routing to BC at the forced pos
regardless of `mode_req`).

**Mode changes during calibration**: `mode_req` is latched every ISR (step 1
of `foc_sample`), so it tracks the user's current request throughout
calibration — only *dispatch* is gated. When `calib_done` flips, the first
real dispatch sees whatever mode is currently requested; the sentinel init
`prev_mode = 0xFF` guarantees the integrator-reset branch fires on that
tick regardless of which mode is selected.

**Held-throttle-at-boot transient**: `pwm` is also latched live. If the user
holds max throttle during calibration, the first post-calib tick engages at
full throttle. In TRQ, `iq_target = FOC_IQ_MAX`, PI with zero integrator +
Kp=1 produces ~500-count output — well under the `foc_trq_v_max = 3000`
clamp, so it ramps in cleanly. In VLT, Vq steps to `FOC_VQ_MAX = 20000`
immediately (no current loop to soften), which is a sharp engage, but
motor impedance limits current rise. Both acceptable — user shouldn't hold
max throttle at boot.

**Bench pre-condition**: ensure the motor is stationary when booting. At
high coast speeds, back-EMF rectification through the FET body diodes can
put current through the shunts during calibration, corrupting offsets. At
realistic bench / riding speeds this isn't an issue (back-EMF stays below
`VBAT`), but noted for completeness — not a firmware check.

---

## BC mode

Self-contained, same 6-step table as `bldcBC.c` (verbatim parity — that file
is excluded from the FOC build, but the lookup is identical). The table
(signed PWM delta applied to each phase):

```
pos 1: y =    0, b = +pwm, g = −pwm
pos 2: y = −pwm, b = +pwm, g =    0
pos 3: y = −pwm, b =    0, g = +pwm
pos 4: y =    0, b = −pwm, g = +pwm
pos 5: y = +pwm, b = −pwm, g =    0
pos 6: y = +pwm, b =    0, g = −pwm
default (pos == 0): y = 0, b = 0, g = 0
```

The `pwm` argument is passed through **unchanged** — no shift, no scale, no
clamp. `bldc.c` has already filtered to `±BLDC_TIMER_MID_VALUE` (~±1125)
upstream and will re-clamp on the way back out. Matches `bldcBC.c:7–47`.

`pos_force` (bench override) routes to BC at the forced pos regardless of
requested mode. Default value is `0` (disabled) — the variable is a
`volatile uint8_t` file-local static, designated-init to zero at boot, poked
at runtime via OpenOCD `mwh` on the bench.

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
- Reset both integrators whenever `mode_req != prev_mode` — stops prior wind-up from biasing the first tick's output. The sentinel init `prev_mode = 0xFF` guarantees this fires on the first post-calibration dispatch (see "Mode dispatch & calibration gating").

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

`<MODE>` is "BC" / "VLT" / "SPD" / "TRQ". RPM from PLL velocity
(`v = state.velocity_q8`):
`rpm ≈ v / 1.44 ≈ (v × 25) / 36` (integer form).

`#ifdef RTT_REMOTE` guards both the ISR snapshot copy and the main-loop
poller. See "Deferred RTT log" for the snapshot struct and concurrency
protocol.

---

## Deferred RTT log

Motivation: `sprintf` with ~15 integer fields is ~15–20 µs on M3 at 72 MHz,
plus `SEGGER_RTT_WriteString` of a 220-byte buffer (~3–5 µs). Doing that
inside the 16 kHz ISR pushes the log-tick duration to ~35 µs out of a 62.5 µs
budget — within bound but fragile, especially since the rewrite adds
Park/Clarke/PI on every ISR (not just log ticks). Moving the format step to
the main loop decouples ISR budget from log verbosity entirely.

**Protocol** — single-producer (ISR) / single-consumer (main loop) via a
snapshot struct + `log_pending` flag, no locks:

```c
typedef struct {
    int8_t  mode;       // 0..3 (BC/VLT/SPD/TRQ)
    int16_t adc_raw_a, adc_raw_b;
    int16_t iy, ib;
    int16_t i_alpha, i_beta;
    int16_t i_d, i_q;
    int16_t i_d_avg, i_q_avg;
    int16_t theta;
    int16_t rpm;
    int16_t vq_cmd;
    int8_t  pos;
    uint8_t pos_force;
} log_snapshot_t;

static volatile log_snapshot_t log_snapshot;
static volatile uint8_t        log_pending;  // 0 = ISR may write; 1 = main must read
```

**ISR side** (inside `foc_sample` step 9):

```c
if ((++log_counter) >= FOC_RTT_LOG_DIV) {
    log_counter = 0;
    if (!log_pending) {                    // only write when the reader is done
        log_snapshot.mode      = mode_req;
        log_snapshot.adc_raw_a = adc_raw_a;
        /* ...populate all fields... */
        log_pending = 1;                   // hand off
    }
    // if log_pending was still 1: skip this tick silently (main loop is slow)
}
```

**Main loop side** (new exported function `FocRttPoll`, called from `main.c`'s
loop):

```c
void FocRttPoll(void) {
#ifdef RTT_REMOTE
    if (!log_pending) return;
    log_snapshot_t local = log_snapshot;   // copy out
    log_pending = 0;                       // release to ISR
    char buf[220];
    int n = sprintf(buf, "m:%s adc_a=%d ...", mode_str(local.mode), local.adc_raw_a, /*...*/);
    SEGGER_RTT_Write(0, buf, n);
#endif
}
```

**Why it's race-free** without locks:
- ISR writes snapshot only when `log_pending == 0`. Main reads only when
  `log_pending == 1`. The flag acts as a one-bit mutex.
- If the main loop is slow (e.g. blocked in another poll), ISRs just skip
  logging silently — dropped samples, never torn data.
- `log_pending` is a single byte; access is atomic on M3. No volatile
  ordering headaches beyond the existing `volatile` qualifier.

**Cost**: ~30 bytes of RAM for the snapshot struct, one byte for the flag,
one new exported function `FocRttPoll(void)`. This is the one permitted
exception to the "no new exports from `bldcFOC.c`" guidance in the
Interface contract — `FocRttPoll` is called from the main loop, not from
`bldc.c`, so it doesn't create a sensor/ISR-side leak.

**Diagnostic caveat**: if the main loop hangs, RTT logs stop — which removes
one failure-mode diagnostic signal. Fair trade given the main loop is
background housekeeping in this firmware; if the main loop's dead, you've
got bigger problems than losing telemetry.

---

## Host unit tests

Everything in `bldcFOC_core.c/.h` compiles on host — that's the point of the
split. Use **plain `assert.h` + a small custom `EXPECT_*` macro** in a common
`test/test_helpers.h`. Rationale: the project has no test framework today and
adding Unity would be the first test-framework dependency; a 5-line macro
like

```c
#define EXPECT_WITHIN(actual, expected, tol) do {                     \
    long _a = (long)(actual), _e = (long)(expected), _t = (long)(tol);\
    if (labs(_a - _e) > _t) {                                         \
        fprintf(stderr, "%s:%d: EXPECT_WITHIN failed: "               \
                "actual=%ld expected=%ld tol=%ld\n",                  \
                __FILE__, __LINE__, _a, _e, _t);                      \
        test_failed = 1;                                              \
    }                                                                 \
} while (0)
```

covers everything the test bodies in this brief need. If the suite ever
grows past a handful of files and Unity's nicer output becomes worth the
dependency, port is mechanical — `EXPECT_WITHIN` ↔ `TEST_ASSERT_INT16_WITHIN`,
etc.

Layout:

```
HoverBoardGigaDevice/
├── Src/bldcFOC_core.c
├── Inc/bldcFOC_core.h
├── test/
│   ├── Makefile              # `make` → build+run all tests
│   ├── test_helpers.h        # EXPECT_* macros, test_failed flag
│   ├── test_sincos.c
│   ├── test_transforms.c
│   ├── test_pll.c
│   └── test_pi.c
```

Host Makefile flags: `-std=c99 -O2 -Wall -Wextra -Werror -fwrapv
-fsanitize=undefined,address -Wdouble-promotion -Werror=float-conversion`.
Signed overflow is UB in C — `-fwrapv` + UBSan catches Q15 overflow during
multiply-and-shift. `-Werror=float-conversion` enforces the no-float rule the
M3 imposes anyway; if a test accidentally uses `double`, host would pass and
target would silently regress.

### Coverage targets

- **`test_sincos.c`** — boundary values (`sin(0)=0`, `sin(90)=32767`,
  `sin(180)=0`, `sin(270)=-32767`, `sin(360)=0`; same shape for `cos`
  shifted 90°); periodicity (`sin(a) == sin(a+360) == sin(a-360)`);
  oddness (`sin(-a) == -sin(a)`); even cosine (`cos(-a) == cos(a)`);
  phase relation (`cos(a) == sin(a+90)`); half-table symmetry (`sin(a)
  == -sin((a+180) mod 360)`); range bound (`|sin| ≤ 32767`, **never**
  `-32768` — so `int16 * int16 → int32` multiplies can't overflow);
  monotonicity on `[0, 90]` (sin increasing, cos decreasing); quarter-wave
  symmetry (`sin(90-a) == cos(a)` for `a ∈ [0, 90]`); Pythagorean identity
  (`sin² + cos² ≈ 32767²` within rounding tolerance); `wrap_angle`
  handles large negative inputs and very large positives without hanging
  or returning out-of-range values.

- **`test_transforms.c`** — Clarke of balanced phase currents (e.g.
  `iy=+k, ib=−k/2` implying `ig=−k/2`) gives `α≈k, β≈0`; Park+iPark
  round-trip is the identity on `(d, q)` at arbitrary θ (this is the
  stated invariant in the Park section); iClarke preserves DC (`α=k, β=0`
  → three phases sum to 0 after SVPWM centering); SVPWM min-max injection
  produces the expected ~15% headroom vs raw sinusoidal (peak amplitude
  of the common-mode-subtracted waveform).

- **`test_pll.c`** — steady forward rotation (pos sequence `1→2→3→4→5→6→1`
  with fixed dwell between edges) produces positive velocity and
  monotonically increasing angle; reverse (`1→6→5→4→3→2→1`) gives negative
  velocity; direction-aware snap lands at the correct sector boundary
  (forward → lower, reverse → upper) — this was explicitly the subtle bug
  that cost bring-up time last round; velocity saturates at
  `±FOC_PLL_VEL_CLAMP`; first-call seeding (`prev_pos == 0`) sets angle to
  sector centre and velocity to zero; glitches (pos delta of ±2, ±3) leave
  velocity and `prev_pos` unchanged; `pos == 0` still advances the
  accumulator and increments `sector_dwell` but doesn't update `prev_pos`.

- **`test_pi.c`** — integrator accumulates `err >> Ki_shift` per step for
  constant error; clamp saturates output at the bound; anti-windup
  back-calc subtracts `excess >> AW_shift` from integrator when saturated
  so it doesn't grow without bound; zero-error holds output constant;
  mode-change integrator reset (caller responsibility, but test the
  reset-to-zero behaviour).

### What host tests do NOT cover

- ISR timing / jitter / budget (bench only)
- DMA ordering, ADC valley sampling (bench only)
- `volatile` interactions and memory ordering (bench only)
- Actual hardware offsets, angle alignment to the physical rotor, mode
  transitions under real load (bench only)

Those belong in **Verification** below.

Fast feedback loop: `make test` in under 1 second; run it before every flash.
If a sine/Park/SVPWM change doesn't flip any assertions, it's at least
dimensionally correct — the bench time then goes to tuning, not hunting
arithmetic bugs.

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
