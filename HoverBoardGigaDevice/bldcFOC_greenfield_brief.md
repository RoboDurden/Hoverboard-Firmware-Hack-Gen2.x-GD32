# bldcFOC greenfield brief

Write `Src/bldcFOC.c` from scratch. This brief gives you the facts, the
empirical constants that took a while to find, the interface contract, and a
target architecture. It deliberately does **not** hand you the implementation
line-by-line — the code choices are yours.

**Don't** modify `Src/bldc.c`, `Src/setup.c`, `Src/it.c`, `Inc/defines.h`,
`Inc/config.h`, or the various `defines_*.h` — they're project-wide and
already wired for `BLDC_FOC`. `Src/main.c` is off-limits *except* for the
single `FocRttPoll()` call already added inside each `while(1)` loop under
`#ifdef BLDC_FOC` — see "Deferred RTT log".

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

**Pointer-type boundary**: `bldc.c` declares the outputs as `int *` (32-bit
on M3). All FOC internals — `iclarke_svpwm`, `dtc_apply`, the BC table — work
in `int16_t` (the values are int16-range and the type discipline relies on
it). Bridge inside `bldc_get_pwm`: declare `int16_t y16, b16, g16;`, run the
output chain on those, then `*y = y16; *b = b16; *g = g16;` once at the end.
Three stores, free; keeps the int16 typing inside FOC and the `int *` API to
`bldc.c` intact.

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

## Implementation phases

Implement in four phases. Each phase lands core helpers + host tests +
glue, flashes to the target, and is verified against a bench gate before
the next phase begins. Earlier phases don't depend on later-phase code;
later phases only add dispatch branches and a handful of helpers.

Within a phase: write `_core` helpers and their host tests first (tests
green before flash), then wire glue in `bldcFOC.c`, then flash and verify.
Do not advance past a phase's bench gate until the listed criteria are
met — later phases are harder to debug if the earlier invariants aren't
solid.

### Phase 1 — Sense + VLT (+ DTC helper, off by default)

End state: full sense pipeline + voltage-FOC output path live in one
flash. BC path remains in dispatch (mode 0) as a no-drive bring-up
diagnostic. DTC code compiled in but `foc_dtc_en = 0` at boot so angle
tuning sees residual Id as pure alignment error. `FOC_ANGLE_OFFSET_BASE`
is empirically tuned during this phase (with DTC first off, then on).
Online Hall boundary learner converges under cruise — BC cruise if
tuning hasn't been done yet, VLT cruise otherwise.

Rationale for the merged phase: BC-only mode can verify ADC calib, RTT
plumbing, gross PLL sanity, and the Hall learner — but *cannot* cleanly
verify Park/Clarke correctness, `lead_q8` sign/scale, DTC scaling, or
direction-aware hall snap. Those only surface under driven current. A
separate BC-only flash therefore buys flash ceremony, not diagnostic
separation; merging keeps BC available as a pre-flight mode without
making it a gating flash.

Core (`Src/bldcFOC_core.c`, `Inc/bldcFOC_core.h`):
- All six frame-struct types (`iph_t`, `ab_i_t`, `ab_v_t`, `ab_pwm_t`,
  `dq_i_t`, `dq_v_t`) plus `foc_state_t`, `pll_state_t`, `pll_edge_t`,
  `pi_gains_t`, `pi_state_t`. Declare the full header up front so later
  phases add implementations, not declarations.
- Q15 constants, sine half-table.
- `wrap_angle`, `sin_q15`, `cos_q15`.
- `signed_shift` (static inline in header; host-testable now even though
  `pi_step` is Phase 2).
- `clarke`, `park`, `ipark`, `ab_scale_for_pwm`, `iclarke_svpwm`.
- `ema_step`.
- `dtc_term`, `dtc_apply`.
- `pll_advance`, `pll_edge`, `hall_widths_update`.
- `get_pwm_bc`.

Glue (`Src/bldcFOC.c`):
- `InitBldc` (prev_mode = 0xFF sentinel; sensorless forces `calib_done
  = 1`; pre-warms `steer_ema` from `steer` at boot; other statics
  zero-init via C runtime).
- `foc_sample` steps 1–9, including the always-on steer LPF update in
  step 2 and the θ composition in step 6 that always adds the scaled
  smoothed steer as a live trim.
- Hall learner window accumulation, steady-state gate,
  `hall_widths_update` call at window close.
- Dispatch in `bldc_get_pwm`: calib gate (short-circuit zero) →
  `pos_force` (BC at forced pos) → mode switch: BC (mode 0), VLT
  (mode 1), TRQ/SPD fall through to BC for now. VLT branch: `vq_cmd =
  pwm × FOC_VQ_MAX / BLDC_TIMER_MID_VALUE` (int32 widen, clamp to
  ±FOC_VQ_MAX), Vd = 0, ipark → ab_scale_for_pwm → iclarke_svpwm →
  optional `dtc_apply` → int16 locals → int* copy.
- Deferred RTT snapshot populate + `FocRttPoll` format/emit.

Host tests (`test/`):
- `test_sincos.c` — full coverage from the Host-tests section.
- `test_transforms.c` — Clarke of balanced phase currents (iy + ib + ig
  = 0) gives `α ≈ k, β ≈ 0`; Park + iPark identity round-trip at
  arbitrary θ; iClarke + SVPWM preserves line-to-line voltages (not
  phase sum); SVPWM min-max injection delivers the ~15% headroom (peak
  phase-to-common at `√3/2 ≈ 0.866` of raw sinusoidal);
  `ab_scale_for_pwm` is a plain right-shift by `FOC_OUT_SHIFT`.
- `test_pll.c` — full coverage (advance + edge, forward/reverse,
  glitches, first-call seed, velocity clamp, direction-aware snap).
- `test_hall_cal.c` — synthetic dwell histograms through
  `hall_widths_update`; convergence, EMA attenuation, Σ drift bound,
  `total == 0` guard.
- `test_pi.c` — `signed_shift` round-trip (positive / negative / zero
  shift) and `ema_step` settling. Full `pi_step` body test deferred to
  Phase 2.
- `test_bc.c` — BC 6-step table parity with `bldcBC.c`, `pos` sanity
  (`0`, out-of-range → zero output), linearity in `pwm`, zero phase sum.
- `test_dtc.c` — `dtc_term` monotonic, zero crossing, saturation at
  ±FOC_DTC_MAX, sign preservation. `dtc_apply` balanced-phase linear-
  region zero-sum + clipped-region non-zero sum (locks in the current
  `FOC_DTC_MAX`).

Bench bring-up:
1. Flash. Boot with `mode_req = 0` (BC, no FOC output driven).
2. **Pre-flight sanity** (quick check, not a ceremonial gate; if any of
   this is wrong, don't proceed to VLT):
   - `adc_raw_a/b` settles near 2000 within ~64 ms; `Ia/Ib` near 0 at
     rest.
   - Hand-spin wheel: RTT `ang:` advances, wraps at 359, direction sign
     matches physical rotation.
   - BC behaviour identical to pre-FOC (throttle response, direction,
     feel).
3. **Pre-warm the Hall learner** (optional but recommended): BC-cruise
   ~10 s above 100 RPM mech and verify `sector_width_q8[]` entries
   diverge from 15360 with Σ held at `360°<<8` within rounding. Letting
   the learner converge before VLT tuning keeps the first angle-tune
   pass from chasing a drifting θ. Alternative: set `foc_hall_learn_en
   = 0` via OpenOCD for the initial VLT tune, re-enable after baking
   `FOC_ANGLE_OFFSET_BASE` in source.
4. Switch to VLT (`mode_req = 1`) and run the bench gate below.

Bench gate (VLT):
- VLT at steady throttle: `Id avg` / `Iq avg` flat, not sinusoidal
  (confirms Park family — if sinusoidal, transpose sin signs and
  retest).
- Tune `FOC_ANGLE_OFFSET_BASE` in source with DTC off until `Id avg ≈
  0` in VLT forward at constant throttle (expect ~125° for this motor
  under brief conventions). This is the phase in which
  `FOC_ANGLE_OFFSET_BASE` is empirically dialled in using the steer
  trim; see "Steer trim" for the bake-in procedure. **Tune at a
  canonical mid-speed cruise** (e.g. ~300 RPM mech, `velocity_q8 ≈
  430`). `Id_meas = 0` is a bench convenience, **not** a physical
  invariant of perfect alignment: under Vd=0, the d-axis steady-state
  equation gives `Id_true = (ω·Lq·Iq)/R` — non-zero and speed-dependent.
  Expect `Id avg` to drift 5–20 counts with throttle / speed once baked;
  that's cross-coupling, not misalignment, and TRQ mode cancels it
  closed-loop in Phase 2.
- `foc_dtc_en = 1` via OpenOCD `mwb`; re-tune `FOC_ANGLE_OFFSET_BASE`
  (expect a few degrees shift as torque-per-amp improves). Leave DTC
  on for Phase 2 and Phase 3.
- VLT RPM tracks BC at low throttle; ~15% higher than BC at full
  throttle (the SVPWM headroom).
- Forward / reverse VLT yield converged angle offsets within ~2–3° of
  each other once DTC is on and the velocity-proportional θ lead is in
  place (step 6). Residual is dead-time bias + ω·Lq·Iq cross-coupling.
  Larger asymmetry (e.g. ≥10°) points at the direction-aware hall snap
  or at the `lead_q8` sign/scale, not at `FOC_ANGLE_OFFSET_BASE`.

### Phase 2 — TRQ mode

End state: closed-loop Iq/Id PI with back-calc anti-windup. Throttle
commands `iq_target`; the PI produces Vd / Vq clamped to
`±foc_trq_v_max`.

Core additions:
- `pi_step` body (accumulate raw err into `integrator_raw`, combine with
  Kp term using `signed_shift(integrator_raw, ki_shift)` on readout,
  clamp to ±clamp, on saturation subtract
  `signed_shift(excess, aw_shift) << ki_shift` from `integrator_raw`).
  See the PI discussion in the Architecture section for the full-
  precision-accumulator rationale.

Glue additions:
- TRQ branch in mode dispatch: map throttle → `iq_target` (int32 widen,
  clamp to ±FOC_IQ_MAX), Iq PI (target `iq_target`, measured
  `i_rotor.q`) → Vq, Id PI (target 0, measured `i_rotor.d`) → Vd.
  Output chain identical to VLT.
- Both PI integrators reset when `mode_req != prev_mode` — covers
  entry from any prior mode. `prev_mode = state.mode_req` at the end
  of the switch path (already written in Phase 1).
- Sensorless build: TRQ short-circuits to zero output (no measurement
  → PI runaway otherwise).

Host test additions (`test_pi.c`):
- Integrator output grows by `err >> Ki_shift` per step at constant
  error (raw accumulator grows by `err`; output shift happens on read).
- **Small-err no-dead-zone**: for `|err| < (1 << ki_shift)`, the output
  still grows — specifically, after `2^ki_shift / err` steps of constant
  sub-shift `err`, output increments by 1. Dead zone only exists in the
  naive "shift-on-write" version the brief used to recommend; this test
  locks in that we're on the full-precision-accumulator side.
- Output saturates to `±clamp`.
- Anti-windup back-calc subtracts `excess >> AW_shift` from the
  integrator's **output contribution** on saturation, integrator bounded.
- Zero error holds output constant.
- Caller-side integrator reset (`integrator_raw = 0`) returns to zero
  output.
- **Sign-symmetric integration**: feeding alternating `+err, -err` for
  equal counts returns the integrator exactly to its starting value
  (locks in the `signed_shift` rounding fix — naive ASR would drift).

Bench gate:
- TRQ at throttle=0: quiet, near-zero current draw.
- TRQ engage / disengage: no kick (integrator reset works).
- TRQ cruise with DTC + Hall learner on: 6×electrical ripple on Id/Iq
  reduced versus the Phase 1 VLT baseline.
- TRQ-as-canary check: if PI output pins at ±`foc_trq_v_max` while
  `Iq` is far from `iq_target`, VLT alignment is insufficient — re-run
  Phase 1 angle tuning (with DTC and Hall learner in their final
  state). Do not tune PI gains against an angle-offset residual.

### Phase 3 — SPD mode

End state: closed-loop speed control via an outer PI on top of the
Phase 2 Iq PI. Throttle commands `speed_target` (Q8 electrical deg/ISR);
the outer PI consumes `velocity_q8` from the Hall PLL and emits
`iq_target`; the inner Iq / Id PIs handle current exactly as in TRQ.

Core additions: none — `pi_step` already landed in Phase 2. Speed
dynamics are ~1000× slower than current dynamics, so the outer loop
reuses the same helper with much larger shifts.

Glue additions:
- A third `pi_state_t` for the speed loop (`speed_pi`), plus constants
  `FOC_SPEED_MAX` (Q8 deg/ISR cap on commanded velocity),
  `FOC_SPEED_KP_SHIFT`, `FOC_SPEED_KI_SHIFT`, `FOC_SPEED_AW_SHIFT`.
- SPD branch in mode dispatch: `speed_target = (int32_t)pwm ×
  FOC_SPEED_MAX / BLDC_TIMER_MID_VALUE` (widen-multiply, clamp), run
  outer PI with `err = speed_target - pll.velocity_q8` and clamp
  `±FOC_IQ_MAX` → this is the Iq target for the inner loop. Then run
  Iq PI (target from outer loop, measured `i_rotor.q`) and Id PI
  (target 0, measured `i_rotor.d`) as in TRQ, same output chain.
- Reset all three integrators (speed, Iq, Id) when `mode_req !=
  prev_mode`.
- Sensorless build: SPD short-circuits to zero output (no current
  measurement → inner loop would run open; same reasoning as TRQ).

Host test additions (`test_pi.c`): none new — outer PI exercises the
same `pi_step` body.

Bench gate:
- SPD at throttle=0: motor freewheels to ~zero velocity.
- SPD engage / disengage: no kick (all three integrators reset).
- Throttle step commands new cruise RPM; PLL velocity converges to
  target. Overshoot <25% on a step is acceptable; settle within a few
  seconds.
- Under hand-braking load on the bench, velocity holds within ~10%
  of target until the inner Iq saturates at ±FOC_IQ_MAX. At
  saturation, anti-windup on the outer PI prevents runaway when the
  brake releases.

Tuning notes:
- Start conservative: `FOC_SPEED_KP_SHIFT = 6`, `FOC_SPEED_KI_SHIFT =
  14`, `FOC_SPEED_AW_SHIFT = 2`. Current loop's Ki shift of 10 is
  wildly too aggressive at the mechanical time constant — the
  integrator would wind out before the motor moves.
- `FOC_SPEED_MAX` is user choice (e.g. `576` Q8 deg/ISR ≈ 400 RPM
  mech for 15 PP at 16 kHz). Full throttle maps there.
- The outer PI's output clamp is `±FOC_IQ_MAX` — same cap the TRQ
  path already enforces, re-used here rather than a new constant.

---

## Hardware facts (verified empirically on layout 2-1-20)

- **MCU**: GD32F130C8T6 — 72 MHz Cortex-M3, no FPU, 8 KB RAM, 64 KB flash.
- **Motor**: 15 pole pairs, 3-phase BLDC, hoverboard hub motor.
- **PWM**: 16 kHz center-aligned on TIMER0. `ARR = 2250`, `BLDC_TIMER_MID_VALUE = 1125`, full PWM period = `2 × ARR = 4500` counter ticks. Dead-time is `DEAD_TIME = 60` ticks (`Inc/defines.h:228`), ≈ 833 ns at 72 MHz — programmed globally in TIMER0's BDTR, so all three half-bridges share the same dead-time.
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
extern uint8_t          wState;       // remote state; bits 0-1 = mode
extern volatile uint8_t hall;         // 3-bit hall state, updated each ISR in bldc.c
extern adc_buf_t        adc_buffer;   // DMA-filled; fields .phase_current_a, .phase_current_b
extern int32_t          steer;        // joystick steer axis, -1000..+1000, written each UART packet
```

**`steer` is a live θ trim, always on.** Every ISR, the raw `steer` is
low-pass-filtered and scaled to a Q8 angular offset that is added into θ
in `foc_sample` step 6 unconditionally. The rider rotates the joystick
stick to dial in the rotor alignment — watch `Id avg` in the RTT plot,
find the stick position where Id sits nearest zero and the wheel
accelerates (not brakes) under positive throttle, leave the stick there.
No button, no mode, no latch — the stick position *is* the trim.

Scaling: full stick deflection (±1000 counts) maps to
`±FOC_STEER_TRIM_RANGE_DEG = ±30°` *nominal* — integer rounding of the
per-count factor actually delivers ±31.25° at full deflection (see
"Steer trim" for the math). That range is enough to cover a ±30° error
in `FOC_ANGLE_OFFSET_BASE` without being so coarse the stick feels
jumpy. See "Steer trim" for the LPF, scaling helper, and bake-in
procedure.

---

## Mode codes (from `wState & 0x03`)

Ordering matches the reference `hoverboard-firmware-hack-FOC` project:

```
0 = BC   (block commutation, FOC off)
1 = VLT  (voltage FOC)
2 = SPD  (speed FOC — outer PI on top of TRQ; see Phase 3)
3 = TRQ  (torque FOC)
```

Bit 5 of `wState` (value 32) is `STATE_LedBattLevel` — don't touch it. The
joystick controller sets bit 5 and fills bits 0-1 with the mode. Other
bits (2–4, 7) are joystick-managed status / LED bits; bit 6 is firmware's
`STATE_Disable` kill switch — not FOC's concern. FOC reads bits 0-1 only.
Bit 2 (`STATE_FocTune`) was the old tune-button flag and is now unused by
FOC — the joystick may still set it from its `-T` CLI arg, but firmware
ignores it.

**Latching discipline**: `wState` is written by the UART ISR
(`remoteUart.c:179` writes it on each incoming joystick packet) and read
by the FOC DMA ISR. Since `wState` is `uint8_t`, a single load is atomic
on M3 (single LDRB). Read once at the top of `foc_sample` (step 1) and
derive `mode_req = wState & 0x03` from the local snapshot. Don't re-read
`wState` inside mode handlers or dispatch.

---

## Architecture — suggested types

You're free to pick names, but I'd recommend this shape. Every FOC stage has
a typed output, so the pipeline reads as a sequence of frame conversions
(phase → αβ → dq) and the type system catches a rotor-frame value being
passed where a stator-frame value was expected.

```c
typedef struct { int16_t iy,    ib;   } iph_t;    // phase-frame currents (yellow, blue) — ADC counts, ~128 LSB/A. Only currents live in this frame; no iph voltage variant exists because iclarke_svpwm emits directly into per-phase PWM deltas.
typedef struct { int16_t alpha, beta; } ab_i_t;   // stator-frame currents — ADC counts
typedef struct { int16_t alpha, beta; } ab_v_t;   // stator-frame voltages in internal units (~±FOC_VQ_MAX)
typedef struct { int16_t alpha, beta; } ab_pwm_t; // stator-frame voltages in PWM-delta units (~±BLDC_TIMER_MID_VALUE) — output-path only
typedef struct { int16_t d,     q;    } dq_i_t;   // rotor-frame currents — ADC counts
typedef struct { int16_t d,     q;    } dq_v_t;   // rotor-frame voltages — internal units (~±FOC_VQ_MAX, clamped to ±foc_trq_v_max in TRQ)

// Per-ISR snapshot. Lives on the stack inside bldc_get_pwm.
typedef struct {
    iph_t   i_phase;   // ADC → offset-subbed (phase frame)
    ab_i_t  i_stator;  // after Clarke
    dq_i_t  i_rotor;   // after Park
    int16_t theta;     // electrical angle used (deg, 0..359) — one θ per ISR
    uint8_t mode_req;  // wState & 0x03, latched once at top of foc_sample
} foc_state_t;
```

**Units gotcha.** Current-carrying types (`iph_t`, `ab_i_t`, `dq_i_t`) hold
**integer ADC counts** (offset-subtracted, ≈128 LSB/A, range ~±2000 around
zero). They are *not* Q15. Clarke and Park are linear transforms, so
ADC-count units pass through unchanged. The Q15 scaling lives in the
*constants* inside Clarke/Park (`Q15_INV_SQRT3`, sine-table entries) —
multiply int16 × Q15-const with an int32 widening and `>> 15` to rebase,
and the result is still in ADC-count units. Voltage-carrying types (`ab_v_t`,
`dq_v_t`) hold **internal-unit signed ints** (range ~±`FOC_VQ_MAX`); PI
integrators produce these from current-error shifts. `ab_pwm_t` holds the
same voltages after `ab_scale_for_pwm` right-shifts by `FOC_OUT_SHIFT`
(range ~±`BLDC_TIMER_MID_VALUE`). Only the sine table entries and the named
Q15 constants are Q15-scaled. Mixing families (treating a current as Q15, or
a Q15 constant as counts) gives results off by 2¹⁵.

Transform primitives as **pure functions** (inputs → output, no hidden writes):
`clarke(iph_t) → ab_i_t`, `park(ab_i_t, θ) → dq_i_t`,
`ipark(dq_v_t, θ) → ab_v_t`,
`ab_scale_for_pwm(ab_v_t) → ab_pwm_t` (right-shifts both components by
`FOC_OUT_SHIFT` to convert internal-voltage units to PWM-delta units),
`iclarke_svpwm(ab_pwm_t v, int16_t *y, int16_t *b, int16_t *g)` (writes int16
phase deltas; see "Pointer-type boundary" in the Interface contract for why
int16 inside FOC and the bridge to `bldc.c`'s `int *` outputs). The six distinct struct types
(`iph_t`, `ab_i_t`, `ab_v_t`, `ab_pwm_t`, `dq_i_t`, `dq_v_t`) all have the
same int16 layout but C treats them as incompatible — the compiler catches
currents handed to `ipark`, voltages fed back into `park`, pre-scale αβ
handed to `iclarke_svpwm`, and the other obvious cross-wires. The
`park`/`ipark` asymmetry becomes visible in the signature: `park` is
current-only (`ab_i_t → dq_i_t`), `ipark` is voltage-only
(`dq_v_t → ab_v_t`) — they are not round-trip partners. Mode handlers take
`const foc_state_t *` and write the PWM deltas. A file-local
`foc_sample(pos) → foc_state_t` does the sense half and updates persistent
statics; `bldc_get_pwm` calls it then dispatches on `mode_req`. Keep
`get_pwm_bc` self-contained (don't depend on `bldcBC.c`).

**pi_step is the current→voltage translator.** Its signature
`int16_t pi_step(pi_state_t *state, int16_t err, int16_t clamp, pi_gains_t gains)`
stays scalar — `err` is a current-unit int16 (target minus measured), the
return value is a voltage-unit int16 (after Kp/Ki shifts and anti-windup).
That one boundary is where current-unit counts become voltage-unit counts;
the shift gains do the dimensional conversion. The struct-type discipline
catches cross-wires everywhere else in the pipeline; `pi_step`'s two
scalars are the sole exception and are documented as such.

Two stateful helpers that mutate via pointer — still pure w.r.t. globals,
since state arrives via the pointer and nothing else is touched:

```c
typedef struct {
    int8_t kp_shift;   // Kp term: positive → err >> kp_shift; negative → err << (-kp_shift) for Kp>1
    int8_t ki_shift;   // integrator += err >> ki_shift each step (same sign convention as kp_shift)
    int8_t aw_shift;   // on saturation: integrator -= excess >> aw_shift  (back-calc anti-windup)
} pi_gains_t;

typedef struct {
    int32_t integrator_raw;    // full-precision err accumulator (NOT the Ki-shifted output);
                               // shift is applied on read inside pi_step, not on write.
                               // int32 — err accumulates past int16 range over many ISRs.
} pi_state_t;

typedef struct {
    int32_t  angle_accum_q8;   // Q8 degrees; wraps in [0, 360 << 8) = [0, 92160)
    int16_t  velocity_q8;      // Q8 deg/ISR; clamped to ±FOC_PLL_VEL_CLAMP
    int8_t   prev_pos;         // 0 = uninitialised sentinel; else 1..6
    uint16_t sector_dwell;     // ISRs since last hall edge
} pll_state_t;

typedef enum {
    PLL_EDGE_VALID  = 0,   // real adjacent-sector edge; consumed_* populated
    PLL_EDGE_SEED   = 1,   // first-call seed; no prior dwell to consume
    PLL_EDGE_GLITCH = 2,   // delta not ±1 (hall noise); caller must invalidate any in-flight learner window
} pll_edge_kind_t;

typedef struct {
    uint16_t        consumed_dwell;   // ISRs spent in the sector that just ended; 0 unless kind == VALID
    int8_t          consumed_sector;  // 0..5 index into sector_width_q8[]; -1 unless kind == VALID
    pll_edge_kind_t kind;             // disambiguates VALID / SEED / GLITCH at the callsite
} pll_edge_t;

int16_t    pi_step    (pi_state_t  *state, int16_t err, int16_t clamp, pi_gains_t gains);
void       pll_advance(pll_state_t *state);
pll_edge_t pll_edge   (pll_state_t *state, int8_t pos, const uint16_t *sector_width_q8);
```

Rationale for the shapes — the "compute, don't store" principle drives most of it:

- **`pll_state_t` drops `prev_hall`, `direction`, `electrical_angle`** — all
  derivable. `prev_pos == 0` is the first-call sentinel (pos is already
  sanitised by `bldc.c`); direction computes locally from `pos` delta on each
  edge; electrical angle is `state.angle_accum_q8 >> 8` at the callsite.
- **`pll_advance` runs every ISR, including `pos == 0`** — advances the
  accumulator with last-known velocity and increments `sector_dwell`.
  `pll_edge` is called conditionally on top. Transient hall glitches don't
  freeze θ and don't break the dwell count.
- **Integrator accumulates `err` at full precision; Ki shift applied on
  read.** Naive "add `err >> ki_shift` per tick" drops the low bits of
  every sample: with `ki_shift = 10` and `|err| < 1024`, the increment
  rounds to 0 and the I-term goes deaf below a ~8 A error threshold
  (far above `FOC_IQ_MAX = 500`) — so TRQ degenerates to P-only. The
  fix is canonical fixed-point PI: `integrator_raw += err;` each tick
  (no shift), then compute `ki_term = signed_shift(integrator_raw,
  ki_shift)` inside the output sum. Anti-windup operates on
  `integrator_raw` with scaled excess: `integrator_raw -=
  signed_shift(excess, aw_shift) << ki_shift` (undo the output-side
  shift so AW subtracts the *raw-units* equivalent of "excess in
  output units"). `int32_t` is mandatory — with err up to ±2500 counts
  and no per-tick shift, the accumulator reaches `±2^31` in ~2.4 hours
  of continuous max-err operation; AW clamps it in practice.
- **Gains passed by value** — same function serves Iq and Id even if their
  gains later diverge; host tests can sweep. Zero M3 cost (AAPCS packs the
  3-byte struct into one register; `-O2` inlines + constant-propagates to
  the same code as a compile-time `#define`).
- **Reset via `state->integrator_raw = 0`** at the callsite — caller handles
  mode-transition resets explicitly. No hidden reset logic inside `pi_step`.
- **Shifts branch on sign inside `pi_step`** — `err >> negative_shift` is
  undefined behavior in C (and the M3's `LSR` masks the shift count). Use:
  ```c
  static inline int32_t signed_shift(int32_t v, int8_t s) {
      if (s <= 0) return v << (-s);
      int32_t bias = (v < 0) ? -((int32_t)1 << (s - 1))
                             :  ((int32_t)1 << (s - 1));
      return (v + bias) >> s;
  }
  int32_t kp_term = signed_shift(err, gains.kp_shift);
  int32_t ki_inc  = signed_shift(err, gains.ki_shift);
  ```
  Rounds half-away-from-zero, matching the `(x + 128) >> 8` pattern used
  for θ composition in `foc_sample` step 6. ARM ASR on signed values
  rounds toward floor, so a raw `err >> ki_shift` accumulates a systematic
  negative bias in the integrator (e.g. `-500 >> 10 = -1`, `+500 >> 10 = 0`
  — symmetric error magnitudes, asymmetric integrator deltas). The bias
  addition kills that drift. Same helper for `kp_shift` and `aw_shift`
  (anti-windup excess). Two predictable branches per call; with constant
  `gains` (the common case — Iq and Id PIs use compile-time literals),
  `-O2` constant-folds both away entirely.
  **Declared `static inline` in `bldcFOC_core.h`** so host tests can
  exercise it directly — a private-in-the-TU version would force tests
  to cover it only through `pi_step`'s observable behaviour, which is a
  weaker guarantee and awkward to debug when a shift edge case breaks.
- **`pi_step` body** — accumulate raw err (`state->integrator_raw += err`),
  then compute the output as `kp_term + signed_shift(integrator_raw,
  gains.ki_shift)`, clamp to `±clamp`, and on saturation back-calc the
  excess in raw-accumulator units:
  `integrator_raw -= signed_shift(excess, gains.aw_shift) << gains.ki_shift;`
  where `excess = out_unclamped − clamp_bound` (signed — positive when
  clipped high, negative when clipped low). The `<< ki_shift` lifts the
  AW correction from output units back to raw-accumulator units so one
  "AW tick" decrements the Ki-scaled output by `excess >> aw_shift`,
  independent of ki_shift. The subtraction always pushes the integrator
  back toward the rail it exceeded. AW fires only on the step where
  saturation actually happened; steady-state-at-clamp still applies AW
  each step, which is the desired behaviour (stops the integrator from
  ratcheting further against the rail).
- **Designated-initializer init**: `pll_state_t pll = { .prev_pos = 0 };`,
  `pi_state_t iq_pi = { 0 };` (zeroes `integrator_raw`). No separate
  `*_init` functions.
- **PLL is split into `pll_advance` + `pll_edge`** — `pll_advance(state)`
  runs every ISR and does the bookkeeping half: `angle_accum_q8 +=
  velocity_q8` (wrap), `sector_dwell += 1`. `pll_edge(state, pos, widths)`
  is called *only when the caller sees a real edge* (`pos != 0 && pos !=
  state->prev_pos`) and handles direction check, velocity update, boundary
  snap, and dwell/prev_pos reset. This split keeps the common path pure
  accumulator math and lets the Hall-width learner in `bldcFOC.c` read
  the just-consumed dwell straight from `pll_edge`'s return value
  (`{consumed_dwell, consumed_sector, kind}` — the `kind` enum lets the
  caller dispatch `VALID` / `SEED` / `GLITCH` without needing to reason
  about `pos` deltas itself). The `pos == 0` glitch case needs no
  special handling — the caller just doesn't call `pll_edge`, the
  accumulator keeps advancing via `pll_advance`, prev_pos stays put.
  Host tests exercise `pll_advance` and `pll_edge` independently: advance
  over many ticks with no edge shouldn't stall θ; edge-only tests sweep
  the pos sequence without needing to spin an accumulator.

---

## File split — `bldcFOC.c` + `bldcFOC_core.c/.h`

Split the module into two translation units so the pure helpers compile
clean on a host (Mac, Linux) for unit tests, no GD32 SPL or `#ifdef`
contortions required.

**`Src/bldcFOC_core.c` + `Inc/bldcFOC_core.h`** — pure, no hardware/globals:
- Types: `iph_t`, `ab_i_t`, `ab_v_t`, `ab_pwm_t`, `dq_i_t`, `dq_v_t`, `foc_state_t`, `pi_gains_t`, `pi_state_t`, `pll_state_t`, `pll_edge_t`
- Q15 constants, half sine table, `wrap_angle`, `sin_q15`, `cos_q15`
- `clarke`, `park`, `ipark`, `ab_scale_for_pwm`, `iclarke_svpwm`, `dtc_term`, `dtc_apply`
- `get_pwm_bc(pwm, pos, *y, *b, *g)` — the BC 6-step table. Pure (arguments
  only, no globals), so it lives here rather than in `bldcFOC.c`; lifted so
  the FOC TU doesn't depend on `bldcBC.c` (which is `#ifdef BLDC_BC` and
  excluded from FOC builds).
- `pi_step(state, err, clamp, gains)`, `pll_advance(state)`,
  `pll_edge(state, pos, sector_width_q8)`,
  `hall_widths_update(widths, learn_sum, ema_shift)`
  — mutate through the state/table pointer; signatures and rationale in
  "Architecture — suggested types" and the dead-time / Hall-learner sections.
- `signed_shift(v, s)` — `static inline` in the header so host tests can
  exercise it directly; a sign-aware shift that avoids the UB of
  `v >> negative` / `v << negative`. Used inside `pi_step` for the Kp/Ki/AW
  paths.
- `ema_step(state, sample, shift)` — `static inline` one-pole IIR; signature
  and body in `foc_sample` step 8.

**`Src/bldcFOC.c`** — glue that touches hardware or project globals:
- `InitBldc` (sets `prev_mode = 0xFF` sentinel; sensorless builds force
  `calib_done = 1`; pre-warms the steer LPF as `steer_ema = (int32_t)steer
  << FOC_STEER_LPF_SHIFT;` so the LPF's 64 ms time constant is honoured
  from ISR #1 regardless of stick-at-boot position; all other statics
  zero-init via C runtime)
- `bldc_get_pwm` (reads `wState`, `hall`, `adc_buffer`; calls core
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

`Inc/bldcFOC.h` exposes just the one extra prototype `void FocRttPoll(void);`
on top of the virtual-method decls pulled in via `bldc.h`. `Inc/bldcFOC_core.h`
declares the pure helpers and the types. `bldcFOC.c` includes both; host
tests include only `bldcFOC_core.h`.

**Build-time gating** — mirror the existing `bldcBC.c` pattern (file always
compiles, body gated by macro):

- `Src/bldcFOC.c` — wrap the entire body in `#ifdef BLDC_FOC … #endif`. When
  `BLDC_BC` (or another impl) is the build's selected BLDC, `bldcFOC.c`
  compiles to nothing and the linker pulls `InitBldc`/`bldc_get_pwm` from
  the other TU.
- `Src/bldcFOC_core.c` — **no `#ifdef BLDC_FOC` guard**. Pure helpers, used
  by the target build (when `BLDC_FOC` is set) *and* by the host test build
  (which never defines `BLDC_FOC`). On a target build with `BLDC_FOC`
  undefined, the helpers compile but are unreferenced; PIO's release flags
  (`-ffunction-sections`, `-fdata-sections`, `--gc-sections`) dead-strip them.
- `Inc/bldcFOC.h` — both the implicit decls from `bldc.h` and the
  `FocRttPoll` prototype are unconditional. Existing `main.c` already
  wraps its `FocRttPoll()` callsites in `#ifdef BLDC_FOC` (lines 220, 471),
  so the symbol only needs to exist for `BLDC_FOC` builds; the unconditional
  declaration costs nothing in non-FOC builds because `main.c` won't
  reference it.

**`#ifdef RTT_REMOTE` for the deferred log** — anything that exists *only*
to support the log is gated; the public `FocRttPoll` symbol stays present
so `main.c` doesn't need a second guard:

- `log_snapshot_t` struct, `log_snapshot` / `log_pending` / `log_counter`
  statics, and the populate-and-flag block in `foc_sample` step 9 → all
  inside `#ifdef RTT_REMOTE` in `bldcFOC.c`. Saves the snapshot's ~30 B of
  RAM in non-RTT builds.
- `FocRttPoll` body → `#ifdef RTT_REMOTE` around the contents (already
  shown in "Deferred RTT log"). When `RTT_REMOTE` is undefined the function
  body is empty — `main.c`'s `FocRttPoll();` call links and executes a
  trivial nothing-return.
- `FocRttPoll` declaration in `bldcFOC.h` → unconditional. The `main.c`
  callsite is already gated on `BLDC_FOC` (not on `RTT_REMOTE`), and we
  don't want the header to drop the prototype based on a different macro
  than the one guarding the call.

---

## Persistent state boundary

File-local statics, kept small and deliberate:

- **PLL**: a single `pll_state_t` (accumulator, velocity, prev_pos, sector_dwell — see Architecture section).
- **Calibration**: offsets for channels A/B, running sums, sample count (incl. the `FOC_CALIB_DISCARD`-sample discard prefix) — all `#if`-gated behind `defined(PHASE_CURRENT_A) && defined(PHASE_CURRENT_B)`. The `calib_done` flag is **not** gated (dispatch reads it every build); sensorless builds force `calib_done = 1` at `InitBldc`. See `foc_sample` step 3.
- **EMAs** of Id/Iq for display (see averaging below).
- **Mode tracking**: `prev_mode` only (for integrator reset on transitions; init `prev_mode = 0xFF` — see "Mode dispatch & calibration gating"). The current `mode_req` lives on the per-ISR `foc_state_t` snapshot, not as a static — it's latched once at the top of `foc_sample` and read from the snapshot downstream.
- **TRQ PI**: two `pi_state_t` (Iq and Id integrators), plus `iq_target` and `volatile int16_t foc_trq_v_max` for runtime poking.
- **Diagnostics**: raw ADC latches and `vq_cmd` (for RTT log), `volatile uint8_t pos_force` (bench override, forces block commutation to a chosen pos 1..6).
- **Hall calibration**: `volatile uint16_t sector_width_q8[6]` (boot-init = 15360 each), `uint32_t learn_sum[6]`, `uint16_t learn_revs`, steady-state detector scratch, `volatile uint8_t foc_hall_learn_en = 1`. See "Online-adaptive Hall boundary learning".
- **DTC gate**: `volatile uint8_t foc_dtc_en = 0`. See "Dead-time compensation".
- **Steer trim**: `int32_t steer_ema = 0` — the LPF accumulator for the live steer-to-θ trim. Ticked every ISR in `foc_sample` step 2; its smoothed output feeds θ composition in step 6 unconditionally. No button, no latch, no mode. See "Steer trim".
- **Deferred log**: `log_snapshot_t log_snapshot` (plain, non-volatile — the flag handshake below orders access), `volatile uint8_t log_pending`, `log_counter` (ISR-side tick counter for `FOC_RTT_LOG_DIV`). See "Deferred RTT log".

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
forward rotation, 15 pole pairs, standard Clarke). Pick the wrong Park
family under these conventions and Id/Iq oscillate sinusoidally at 2×
electrical frequency; transpose the matrix and retest.

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

## Dead-time compensation

All three half-bridges share the single `DEAD_TIME = 60`-tick window
programmed into TIMER0's BDTR. During dead-time both FETs of a leg are off
and the body diode clamps the phase — to 0 V if `i_phase > 0`, to V_bus if
`i_phase < 0`. Average phase voltage ends up offset from commanded by
`−V_bus × (DEAD_TIME / T_pwm) × sign(i_phase)`, where `T_pwm = 2 × ARR = 4500`
counter ticks. At V_bus = 27 V that's `27 × 60/4500 ≈ 0.36` V per phase —
low-order harmonics on Id/Iq, worst near zero-crossings.

Sanity note on the factor: center-aligned complementary PWM has **two** DT
windows per period (one at each commanded edge), but only **one** of them
produces a voltage error for a given `sign(i)` — during the other, the
body-diode-clamped phase voltage coincides with the level the FET would have
produced, so it's a no-op. Count DT once per period, not twice.

In PWM-delta units the correction is
`(DEAD_TIME / T_pwm) × V_bus / (V_bus / ARR)` = `DEAD_TIME × ARR / T_pwm`
= `DEAD_TIME / 2` = **30 ticks × sign(i)** (include `Inc/defines.h` for
`DEAD_TIME`; don't hardcode the value). Hard `sign()` chatters near zero
crossings where the shunt is noise, so use a soft limiter. Pure helpers
in `_core`:

```c
static inline int16_t dtc_term(int16_t i_counts) {
    int32_t t = (int32_t)i_counts >> FOC_DTC_SHIFT;
    if (t >  FOC_DTC_MAX) t =  FOC_DTC_MAX;
    if (t < -FOC_DTC_MAX) t = -FOC_DTC_MAX;
    return (int16_t)t;
}

void dtc_apply(iph_t i, int16_t *y, int16_t *b, int16_t *g);
```

`dtc_apply` derives `ig = −(iy + ib)` locally and adds `dtc_term(ix)` to each
phase delta, called after `iclarke_svpwm`. With `FOC_DTC_MAX = DEAD_TIME / 2`
(= 30 at current `DEAD_TIME`) and `FOC_DTC_SHIFT = 2` the ramp saturates at
`|i| ≈ 120 LSB ≈ 0.9 A` — above the post-calib noise floor on both shunts
and on the derived green channel (~2× noisier than the shunts, absorbed by
the soft limiter).

**Gated by `volatile uint8_t foc_dtc_en = 0`** (file-local, default off). DTC
must stay off during initial `FOC_ANGLE_OFFSET_BASE` tuning so residual Id is
unambiguously an angle-offset error. After VLT alignment is dialled in, flip
via OpenOCD (`mwb`) and expect `FOC_ANGLE_OFFSET_BASE` to shift a few degrees
(torque-per-amp improves → Iq/Id rebalance). Re-tune offset, leave both set.

Calibration-window gating is implicit: while `!calib_done`, `i_phase` is
zeroed so `dtc_term(0) = 0` on every phase — no additional guard needed.

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

**Applied via `ab_scale_for_pwm`** — a one-line pure helper in `_core` that
right-shifts both αβ components by `FOC_OUT_SHIFT` and flips the type from
`ab_v_t` (internal-unit voltage) to `ab_pwm_t` (PWM-delta-unit voltage).
The distinct output type means a caller who accidentally feeds a pre-scale
`ab_v_t` into `iclarke_svpwm` — which only takes `ab_pwm_t` — gets a
compile error, not a runtime surprise. Typical output chain in a mode
handler:

```c
int16_t y16, b16, g16;
ab_v_t   v_ab     = ipark(v_dq, state.theta);
ab_pwm_t v_ab_pwm = ab_scale_for_pwm(v_ab);
iclarke_svpwm(v_ab_pwm, &y16, &b16, &g16);
if (foc_dtc_en) dtc_apply(state.i_phase, &y16, &b16, &g16);
*y = y16; *b = b16; *g = g16;
```

---

## Sine table

Q15 sine lookup, generated once by `generate_sine_table.py` and pasted
inline. 1° resolution is enough — the PLL advances many degrees per ISR
at any usable speed, so sub-degree resolution adds flash with no behavioural
benefit.

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
continuous `electrical_angle` (degrees, wraps at 360). The PLL runs in two
halves: `pll_advance(&state)` is called every ISR from the sense half
(including on `pos == 0`); `pll_edge(&state, pos, sector_width_q8)` is
called only when the caller sees a real edge (`pos != 0 && pos !=
state.prev_pos`).

Sector centres (degrees), arbitrary absolute base — the true rotor alignment
is absorbed into `FOC_ANGLE_OFFSET_BASE`:

```c
static const int16_t sector_centre_deg[6] = { 180, 240, 300, 0, 60, 120 };
// indexed by (pos - 1), i.e. sector_centre_deg[0] is pos 1
```

This table is the single source of truth for the PLL's angle convention.
The boundary cumulative-sum anchor derives from it:

```c
#define FOC_PLL_NOMINAL_WIDTH_Q8  15360  // 60° << 8
#define FOC_PLL_ANCHOR_Q8  \
    (((int32_t)sector_centre_deg[0] << 8) - FOC_PLL_NOMINAL_WIDTH_Q8 / 2)
// = (180 << 8) - 7680 = 38400 = 150° << 8
```

Under nominal widths the anchor reproduces the centre table exactly. Under
adaptive widths, boundaries shift (centres drift with them) but the anchor
stays put. `FOC_ANGLE_OFFSET_BASE = 125°` was tuned under this convention.

Direction from the `pos` sequence: forward is `1 → 2 → 3 → 4 → 5 → 6 → 1`. A
delta of ±1 (or ∓5 due to wrap) tells you direction; any other delta is
noise — leave velocity alone for that edge.

`pll_advance` every call:

- `if (sector_dwell < UINT16_MAX) sector_dwell++;` — saturate at the
  `uint16_t` cap (65535 ISRs ≈ 4.1 s). A rotor sitting stationary with
  power on indefinitely must not wrap the counter to 0; at the next real
  edge, `velocity_q8 = sector_width / sector_dwell` with a small post-
  wrap `sector_dwell` would produce a spurious velocity spike (clamped by
  `FOC_PLL_VEL_CLAMP` but still a one-tick glitch and an RPM-log
  artefact). Saturated, the next edge sees `dwell = 65535`, yielding
  `velocity_q8 = width_q8 / 65535 ≈ 0` — physically correct for a rotor
  that just left standstill.
- `angle_accum_q8 += velocity_q8`, wrap into `[0, 360 << 8)`

So `electrical_angle = state.angle_accum_q8 >> 8` at the callsite keeps
advancing on transient hall glitches (`pos == 0`) with last-known velocity.

`pll_edge` — caller invokes only when `pos != 0 && pos != state.prev_pos`.
Returns `pll_edge_t { consumed_dwell, consumed_sector, kind }` — the
pre-reset dwell, its 0..5 sector index, and a discriminator between the
three distinct outcomes (`VALID`, `SEED`, `GLITCH`). The Hall-width
learner switches on `kind`; it never has to peek at internal state or
reason about `pos` deltas itself. Inside:

1. **First-call seeding** (`state.prev_pos == 0`): set `angle_accum_q8` to
   the sector centre (`<< 8`), `velocity_q8 = 0`, `prev_pos = pos`,
   `sector_dwell = 0`. Return `{0, -1, PLL_EDGE_SEED}` — no real edge was
   consumed, the learner skips this tick.
2. Otherwise: compute direction from `(prev_pos → pos)`. Delta ±1 (or ∓5 via
   wrap) → ±1. Anything else is a hall glitch — **bail out of the rest
   without updating `prev_pos` or `sector_dwell`** (so the next valid
   edge still sees an accurate dwell count for the sector the rotor is
   actually in), and return `{0, -1, PLL_EDGE_GLITCH}`. The
   `GLITCH` kind — distinct from `SEED` — is the caller's signal to
   invalidate any in-flight learner window: dwell that was accruing got
   partly attributed to the wrong sector, so the whole window is no
   longer trustworthy.
3. Snapshot `uint16_t consumed = sector_dwell; int8_t sector_idx = prev_pos - 1;`
   — these are the return payload (with `kind = PLL_EDGE_VALID`).
4. If `sector_dwell > 0`:
   `velocity_q8 = (dir × sector_width_q8[prev_pos − 1]) / sector_dwell` — clamp
   to `±FOC_PLL_VEL_CLAMP = 3000` (Q8 deg/ISR). At boot every entry is
   `60 × 256 = 15360`, so this reduces to the nominal formula; the adaptive
   learner (next section) refines the widths in place.
5. Snap `angle_accum_q8` to the sector **boundary** the rotor just crossed,
   **not the centre**. Boundaries are the cumulative sum of `sector_width_q8[]`
   starting from `FOC_PLL_ANCHOR_Q8` (= 150° << 8, derived from the
   `sector_centre_deg[]` table as shown above):
   ```
   entry_fwd(pos 1) = FOC_PLL_ANCHOR_Q8
   entry_fwd(pos k) = FOC_PLL_ANCHOR_Q8 + sum_{i=0..k-2} sector_width_q8[i]   (k = 2..6)
   ```
   Forward snap (`dir = +1`) → `entry_fwd(pos)`.
   Reverse snap (`dir = −1`) → `entry_fwd(pos) + sector_width_q8[pos-1]` (upper boundary).
   Forward and reverse land on the same physical angle from opposite sides.

   **Wrap the snap target into `[0, 360<<8)` before assigning** —
   `entry_fwd(6) = 38400 + 5×15360 = 115200` exceeds the period (92160), and
   the reverse snap goes one sector further. A single conditional subtract
   suffices (snap can be at most 2× the period, never more):
   ```c
   int32_t snap = FOC_PLL_ANCHOR_Q8 + cumulative_sum;
   if (dir < 0) snap += sector_width_q8[pos - 1];
   if (snap >= (360 << 8)) snap -= (360 << 8);
   state->angle_accum_q8 = snap;
   ```
   Don't rely on `pll_advance` to wrap on the next tick — the `electrical_angle
   = state.angle_accum_q8 >> 8` read at the callsite happens before the next
   `pll_advance`, so an unwrapped value would be visible as ~720° for one ISR.

   At boot, nominal widths reproduce the `centre ± 30°` values in the
   reference implementation. *Without* the direction-aware snap, forward and
   reverse ideal-offset values differed by ~60° (one sector width) — a subtle
   bug that cost bring-up time.
6. `prev_pos = pos`, `sector_dwell = 0`. Return `{consumed, sector_idx, PLL_EDGE_VALID}`.

---

## Online-adaptive Hall boundary learning

Manufacturing variance puts each of the six Hall sectors at a slightly
different physical width — not the nominal 60°. The fixed-width PLL biases
velocity estimates and lurches θ at boundary snaps; visible as 6×electrical
ripple on Id/Iq. Per-motor bench calibration is a non-starter for usability,
so the PLL self-calibrates at runtime from steady-state cruise data —
no user action, survives motor swaps.

**Representation** — a 6-entry Q8 width table, file-local in `bldcFOC.c`:

```c
static volatile uint16_t sector_width_q8[6];   // Q8 degrees per sector; sums to 360<<8
```

Boot-initialised to `15360` (60° × 256) per entry. Passed as `const uint16_t *`
into `pll_edge` (cast from volatile at the call site — both reader and writer
are the FOC ISR, `volatile` is only there so OpenOCD pokes are visible to
compiled code). Boundary absolute angles are the cumulative sum of widths
from a fixed reference — see Hall PLL step 5.

**Learning accumulator** — per-sector ISR counts across a window of
`FOC_HALL_LEARN_WINDOW_REVS = 10` electrical revolutions:

```c
static uint32_t learn_sum[6];   // ISRs spent in each sector this window
static uint16_t learn_revs;     // full-rev count this window
static int16_t  learn_pwm_ref, learn_vel_ref;  // steady-state references
```

Per ISR: always call `pll_advance(&pll)`. If `pos != 0 && pos != pll.prev_pos`,
snapshot `int8_t prev = pll.prev_pos;` then call
`pll_edge_t edge = pll_edge(&pll, pos, widths);`. Dispatch on `edge.kind`:

- `PLL_EDGE_VALID` — real edge; do
  `learn_sum[edge.consumed_sector] += edge.consumed_dwell;`
  and count a rev if the just-consumed edge is the wrap (`prev == 6 &&
  pos == 1` forward, `prev == 1 && pos == 6` reverse). The snapshotted
  `prev` is how the learner detects wrap without peeking at PLL internals.
- `PLL_EDGE_SEED` — first call ever; skip. No partial dwell to attribute.
- `PLL_EDGE_GLITCH` — hall noise was seen inside the window. The dwell
  count spans a stretch where `pos` was briefly wrong, so anything we've
  accumulated this window is suspect. Run the same reset path the
  steady-state gate uses (zero `learn_sum[]`, `learn_revs = 0`) and let
  the next `SEED`-or-`VALID` edge capture fresh refs. Same shape as the
  existing gate-violation reset.

**Gate — three conditions, all required:**

1. **Minimum speed**: `|velocity_q8| > FOC_HALL_LEARN_MIN_VEL = 144`
   (Q8 deg/ISR; derives from `100 RPM mech × 15 PP × 360°/rev / (60 s × 16 kHz)
   × 256 ≈ 144`). Below that, per-sector ISR counts are <100 and quantization
   noise swamps the skew signal.
2. **Steady state**: `|pwm − learn_pwm_ref| < FOC_HALL_LEARN_PWM_TOL` and
   `|velocity_q8 − learn_vel_ref| < FOC_HALL_LEARN_VEL_TOL` across the whole
   window. Matters more than the speed gate — any acceleration distorts the
   dwell ratio. **Ref capture**: on the first edge of a fresh window — detect
   via `learn_revs == 0 && learn_sum[0..5] all zero`, and in that tick
   snapshot `learn_pwm_ref = pwm; learn_vel_ref = velocity_q8;` before the
   accumulator update. Subsequent ticks in the same window check against
   that snapshot; a gate violation resets the accumulators (see below) and
   the next first-edge tick captures fresh refs that match the new cruise.
3. **PLL healthy**: `calib_done == 1`, no hall glitch in the window —
   neither `pos == 0` nor any non-adjacent `pos` jump (the latter
   surfaces as `edge.kind == PLL_EDGE_GLITCH` and triggers the same
   accumulator reset as any gate violation) — and direction constant
   (no reversal).

(The steer trim changes θ live but doesn't touch the hall sensors, the
PLL's velocity estimate, or `pos` — the learner is immune to rider stick
motion and doesn't need a tune-specific gate.)

Any gate violation mid-window resets all accumulators and restarts at the
next valid edge. No partial updates.

**Update** — at window close, the caller invokes a single `_core` helper:

```c
// In bldcFOC_core.h:
void hall_widths_update(uint16_t       widths[6],     // mutated in place
                        const uint32_t learn_sum[6],  // ISR counts per sector
                        uint8_t        ema_shift);    // FOC_HALL_LEARN_EMA_SHIFT
```

Body (in `bldcFOC_core.c`):

```c
uint32_t total = learn_sum[0] + learn_sum[1] + learn_sum[2]
               + learn_sum[3] + learn_sum[4] + learn_sum[5];
if (total == 0) return;
for (int i = 0; i < 6; i++) {
    uint16_t measured = (uint16_t)(((uint64_t)(360u << 8) * learn_sum[i]) / total);
    int16_t  delta    = (int16_t)(measured - widths[i]);
    widths[i] += delta >> ema_shift;
}
```

Caller in `bldcFOC.c` casts the `volatile uint16_t sector_width_q8[6]` to
`uint16_t *` at the call site — same pattern as `pll_edge`'s width-table
argument; both reader and writer are the FOC ISR, the `volatile` only
exists for OpenOCD visibility.

**Reset window state after the update.** `hall_widths_update` is pure (only
mutates `widths`); the caller owns the window-bookkeeping reset. Right after
the call, zero the per-sector accumulators and the rev counter:

```c
if (learn_revs >= FOC_HALL_LEARN_WINDOW_REVS) {
    hall_widths_update((uint16_t *)sector_width_q8, learn_sum,
                       FOC_HALL_LEARN_EMA_SHIFT);
    for (int i = 0; i < 6; i++) learn_sum[i] = 0;
    learn_revs = 0;
    // learn_pwm_ref / learn_vel_ref deliberately not reset — the next first
    // edge of the new window re-captures them via the
    // `learn_revs == 0 && all-zero learn_sum` detector, overwriting cleanly.
}
```

Same zero pattern as the gate-violation reset path, just at a different
trigger. Keeping the two paths parallel means there's only one shape of
"start a fresh window" in the code.

Normalization keeps Σwidths = 360°<<8 up to integer rounding; the EMA
(`FOC_HALL_LEARN_EMA_SHIFT = 3`) attenuates single-window noise and lets
systematic skew converge over a handful of windows. The `total == 0` guard
covers the degenerate case where the gate let a window through with no
edges accumulated — shouldn't happen given the speed gate, but a single
compare beats a divide-by-zero crash.

**Gated by `volatile uint8_t foc_hall_learn_en = 1`** (default on). Flip to 0
via OpenOCD during bring-up or when freezing the table for experiments.
`foc_dtc_en` has no interaction with the learner — both can be on, off, or
mixed; test DTC with learner frozen first to separate effects.

**Boot behaviour**: table at nominal on cold start. ~10 s of steady cruise
above 100 RPM mechanical converges it. Power-cycle re-starts from nominal;
re-convergence is quick. Flash persistence is deferred — see Non-goals.

**Host test** (`test_hall_cal.c`): feed synthetic dwell histograms (uniform,
skewed, noisy, single-outlier) through the update math — widths converge
toward the underlying distribution; Σwidths stays at 360°<<8 within rounding
tolerance; EMA attenuates single-window noise by the expected factor; gate
violations leave the table untouched.

---

## Sense half — `foc_sample(pos) → foc_state_t`

Responsibilities, in order:

1. Latch `wState` into a local `uint8_t ws = wState;` (single atomic LDRB), derive `mode_req = ws & 0x03;`, store on the returned `foc_state_t`. Downstream dispatch reads `state.mode_req`; don't re-read `wState`. See "Latching discipline".
2. Read raw ADCs; mirror into `adc_raw_a/b` for RTT. Update the steer LPF: `steer_ema = ema_step(steer_ema, (int16_t)steer, FOC_STEER_LPF_SHIFT);`. A file-local `int16_t steer_smoothed = (int16_t)(steer_ema >> FOC_STEER_LPF_SHIFT);` derives the current smoothed value for step 6.
3. **Calibration** (boot-time, one-shot). Discard the first `FOC_CALIB_DISCARD = 32` ADCs to let the analog front-end settle, then accumulate the next `FOC_CALIB_SAMPLES = 1024` ADCs into running sums. Count `sample_count` from 0 through `FOC_CALIB_DISCARD + FOC_CALIB_SAMPLES = 1056`, accumulating only when `sample_count >= FOC_CALIB_DISCARD`; when `sample_count` reaches 1056, compute means as offsets and flip `calib_done`. While `!calib_done`: skip steps 4, 7, **and 8** (currents/Clarke/Park/EMA) — offsets aren't known yet, and running the EMA on zeroed `i_rotor` just feeds it meaningless zeros — and return a snapshot with `i_phase`, `i_stator`, `i_rotor` zeroed. **Still run steps 5, 6, 9** — the PLL updates from ISR #1 so θ is warm when FOC engages (handles the case where someone pushes the wheel during boot); step 9 emits an RTT log with zero current fields (visible "calibrating" tell).

   **Sensorless builds** (`!defined(PHASE_CURRENT_A) || !defined(PHASE_CURRENT_B)`): `InitBldc` sets `calib_done = 1` unconditionally (nothing to calibrate), and `foc_sample` `#if`-gates the current-reading halves of steps 2, 3, 4, 7, 8 out — the steer-LPF half of step 2 still runs (steer has nothing to do with current sensors) — and the current-carrying fields of `foc_state_t` stay at zero, EMAs never tick. Steps 1, 5, 6, 9, 10 run unchanged, so VLT still drives PWM and the RTT log still emits (with sensor fields zeroed). TRQ short-circuits to zero output as specified in its section.

   **Calibration constants and state are also `#if`-gated** behind `defined(PHASE_CURRENT_A) && defined(PHASE_CURRENT_B)`. That includes `FOC_CALIB_DISCARD`, `FOC_CALIB_SAMPLES`, and the file-local statics (`offset_A`, `offset_B`, running sums, `sample_count`). They have no callers in a sensorless build, so leaving them in would just sit as dead constants and ~12 B of unused RAM — gating them out makes "this build doesn't have current sensors" structural in the source. `calib_done` itself stays unconditional because the dispatch gate at the top of `bldc_get_pwm` reads it in every build (sensorless: set to 1 once at `InitBldc`, then the dispatch check passes forever).
4. Subtract offsets into `i_phase`: `i_phase.iy = channel_B − offset_B` (yellow), `i_phase.ib = channel_A − offset_A` (blue). The green phase is unshunted — it's reconstructed as `ig = −(iy + ib)` wherever it's needed (inside `dtc_apply`, and in the log snapshot populated at step 9). `ig` lives in no struct.
5. Advance the PLL accumulator: `pll_advance(&pll)` every ISR. Then, if `pos != 0 && pos != pll.prev_pos`, snapshot `int8_t prev = pll.prev_pos` and call `pll_edge_t edge = pll_edge(&pll, pos, widths)` — the Hall-width learner one block down dispatches on `edge.kind` (`VALID` / `SEED` / `GLITCH`). Otherwise `edge = { .kind = PLL_EDGE_SEED }` — no edge was called for, learner does nothing this tick.
6. Compute θ:
   ```c
   int16_t lead_q8 = pll.velocity_q8 >> 1;   // ½ ISR of rotor advance, still Q8 deg
   theta = wrap_angle(electrical_angle
                      + FOC_ANGLE_OFFSET_BASE
                      + ((steer_to_trim_q8(steer_smoothed) + 128) >> 8)
                      + ((lead_q8 + 128) >> 8));
   ```
   Use this same `theta` for Park *and* for the mode handler's inverse Park.

   The `(x + 128) >> 8` pattern rounds half-up rather than floor, avoiding a
   ~½° negative bias from ARM ASR on signed values.

   Each Q8 term is rounded *independently* before being added. Summing the
   Q8 values first and rounding once would give a slightly different θ
   (up to ±1° cumulative rounding drift). Keep them separate — matching
   this literal form makes the per-term contributions easy to reason
   about in isolation.
7. `clarke(i_phase) → i_stator`. `park(i_stator, theta) → i_rotor`.
8. Update Id/Iq EMAs for display via the `_core` helper. Two file-local statics in `bldcFOC.c`:
   ```c
   static int32_t i_d_ema, i_q_ema;   // EMA state, ~Q(FOC_AVG_SHIFT)-scaled
   ```
   Per ISR after Park (`i_rotor` is in scope):
   ```c
   i_d_ema = ema_step(i_d_ema, state.i_rotor.d, FOC_AVG_SHIFT);
   i_q_ema = ema_step(i_q_ema, state.i_rotor.q, FOC_AVG_SHIFT);
   ```
   Read into the log snapshot as `(int16_t)(i_d_ema >> FOC_AVG_SHIFT)`.
   `ema_step` is a one-pole IIR (declared in `bldcFOC_core.h`, see file split):
   ```c
   static inline int32_t ema_step(int32_t state, int16_t sample, uint8_t shift) {
       return state + (int32_t)sample - (state >> shift);
   }
   ```
   `int32_t` state is required: with `sample` ~±2000 counts and `shift = 10`,
   the equilibrium magnitude is `sample << 10` ≈ ±2,048,000 — overflows int16
   well before settling. Time constant ≈ `2^FOC_AVG_SHIFT` samples at 16 kHz
   = 1024 / 16 000 ≈ 64 ms.
9. Every `FOC_RTT_LOG_DIV = 160` samples (= 100 Hz): if `!log_pending`, populate the `log_snapshot` struct with this ISR's values and set `log_pending = 1`. **No `sprintf` or RTT write in the ISR** — a main-loop poller formats and emits. See "Deferred RTT log" below.
10. Fill and return the snapshot.

Why PLL first, then θ, then Park: gives Park and inverse Park one consistent
angle. Previously the old code did Park with stale `electrical_angle` and
inverse Park with the freshly-updated angle — not a bug for VLT but avoidable
in a clean implementation.

**Sampling-to-actuation latency.** The ADC samples at the PWM valley, Park
uses θ at that ISR, and the compare-register writes take effect at the next
PWM update — so sense-θ and apply-θ differ by ~½ PWM period of rotor
advance. That lag is `velocity × ½·T_pwm` radians: **sign-flipping with
direction**, magnitude scaling with speed. A single constant offset cannot
absorb it — forward and reverse would need equal-magnitude, opposite-sign
trims, and the prior implementation's need for different VLT tuning in each
direction traces straight back to this.

Two terms in the θ composition do the two jobs:
- `FOC_ANGLE_OFFSET_BASE` — the *constant* part (rotor-alignment offset).
  Dial once during bring-up.
- `lead_q8 = velocity_q8 >> 1` — the *velocity-proportional* part (the
  ½-PWM lead). Automatically signed through `velocity_q8`; zero at rest;
  at `velocity_q8 = 576` (~400 RPM mech) it contributes ~1.1° forward,
  −1.1° reverse — exactly the asymmetry that previously had to be tuned
  out per direction.

Park and inverse Park share this composed θ (= apply-θ) for simplicity.
Strictly, Park should use sense-θ — using apply-θ rotates the (Id, Iq)
measurement by +lead relative to the true rotor-sense frame. The resulting
Id_meas=0 dial-in point carries a small speed-dependent Id_true offset
(~Iq·tan(lead), a few ADC counts at 400 RPM), direction-symmetric, absorbed
by the tune tolerance. Only the rotor-alignment constant needs bench-dialling;
the speed-dependent piece is analytical.

**Why "direction-symmetric":** both `ω` and `Iq` flip sign with direction
under throttle (reverse throttle → negative Vq → negative ω → negative Iq),
so `Iq · tan(lead)` has the **same** sign fwd and rev — the `Id_meas = 0`
tune point lands at the same θ offset in both directions. Same reasoning
applies to the `ω·Lq·Iq/R` VLT cross-coupling term. Both analytical
residuals preserve direction symmetry; the ≤~3° fwd/rev gap in the bench
gate absorbs them plus dead-time and hall-learner jitter.

---

## Project convention — `SPEED_COEFFICIENT` during bring-up

`SPEED_COEFFICIENT` (defined in `config.h` and per-layout `defines_2-t-l.h`)
is a throttle-direction inversion knob applied at `main.c:309` inside the
default PILOT path:

```c
scaledSpeed = ... * SPEED_COEFFICIENT;
```

It exists so that on boards where the motor is wired such that "motor's
natural rotation for +Vq" = "rider's physical reverse", the firmware can
still deliver "joystick-forward → rider-forward" by flipping the sign in
software. It's a *user-convention* knob, not a motor-physics one.

**For FOC bring-up, set `SPEED_COEFFICIENT = +1`** (at least temporarily).
Reasons:

- Log `Vq` sign, throttle sign, and rotation-direction sign all agree
  under `SPEED_COEFFICIENT = +1`. One convention everywhere. Under `-1`,
  `Vq:-5031` in the log means "rider pushed forward" — which is almost
  impossible to reason about during a bring-up session.
- `SPEED_COEFFICIENT` must *not* be plumbed into the FOC module itself
  (don't multiply `Vq` by it inside `bldc_get_pwm`). That mixes the
  user-convention layer into the motor-frame math and creates an extra
  sign ambiguity on every read. Keep FOC motor-frame-only.

After FOC is dialled in, decide: leave `SPEED_COEFFICIENT = +1` (rider
accepts new joystick convention), or flip back to `-1` (rider keeps old
joystick direction, accepts that log `Vq` reads inverted). The FOC math
doesn't care either way — the only place the constant lives is `main.c`.

---

## Mode dispatch & calibration gating

**Priority, highest first**: `!calib_done` (zero-out + return) → `pos_force != 0`
(route to BC at forced pos) → `switch (mode_req)` (BC / VLT / SPD / TRQ).
The steer trim reshapes θ in every mode but doesn't interact with
dispatch.

**`prev_mode` update policy**: `prev_mode = state.mode_req` is assigned
**inside the `switch (mode_req)` dispatch path**, at the end of dispatch.
Two out-of-band paths touch `prev_mode` differently:
- `!calib_done` leaves `prev_mode` untouched. Dispatch was short-circuited
  before mode handlers ran, so the first post-calib tick still needs the
  `mode_req != prev_mode` reset to fire (the `0xFF` init sentinel
  guarantees this).
- `pos_force != 0` **sets `prev_mode = 0xFF`** before returning. During
  pos_force, BC is forced regardless of requested mode; TRQ/SPD integrators
  are idle but the motor's physical state (position, velocity, current) is
  being driven by BC. Returning to TRQ or SPD with stale integrator state
  from before the pos_force detour would let pre-detour wind-up kick into
  a motor in a different operating point. Poisoning `prev_mode` with the
  init sentinel forces an integrator reset on the first post-pos_force
  dispatch, matching the "reset on mode transition to prevent wind-up
  bias" rule. Cost: one byte-store on a path only used from OpenOCD.

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

Lives in `bldcFOC_core.c` as `get_pwm_bc(pwm, pos, *y, *b, *g)` — pure,
argument-only, no globals. Same 6-step table as `bldcBC.c` (verbatim parity
— that file is excluded from the FOC build, but the lookup is identical).
Putting it in `_core` means the top-level `bldcFOC.c` calls it from two
paths (normal BC dispatch and the `pos_force` bench override) without
introducing a compile-time dependency on `bldcBC.c`, and it can be
unit-tested on host alongside the transforms. The table (signed PWM delta
applied to each phase):

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
`±FOC_VQ_MAX`. Vd is hardcoded to 0.

```
vq_cmd = (int32_t)pwm * FOC_VQ_MAX / BLDC_TIMER_MID_VALUE;
// clamp to ±FOC_VQ_MAX (defensive — pwm is already ±BLDC_TIMER_MID_VALUE upstream)
```

Use a widening `int32_t` for the multiply (`1125 × 20000 = 2.25e7`, well past
int16). Result range matches `±FOC_VQ_MAX` exactly when `pwm` is at the
upstream clamp; the explicit clamp catches transient overshoot from the
upstream low-pass filter.

Invariants:

- The measured Id/Iq from the snapshot are **not fed back**; current sensing
  is diagnostic only.
- Motor behaviour depends entirely on `theta` tracking the rotor correctly —
  find the correct angle offset via the always-on steer trim (see
  "Steer trim"). The rider rotates the joystick stick while watching `Id
  avg` in the RTT plot, parks the stick at the position that zeros Id
  with the wheel accelerating under positive throttle. Bake the winning
  stick-position-in-degrees into `FOC_ANGLE_OFFSET_BASE` in source for
  future builds; runtime stick at rest means zero trim, so baking it in
  is what lets the rider recentre the stick for normal use.

Output path: `ipark(Vd=0, Vq, theta)` → inverse Clarke → SVPWM centre → PWM compares.

VLT should work without current sensors defined — the sense-half gating is specified in `foc_sample` step 3's "Sensorless builds" note: θ still tracks, PWM still drives, and the plot shows the raw ADC channels (and all current-derived fields) at zero.

---

## TRQ mode

Closed-loop Iq with an Iq PI and an Id PI. Throttle linearly maps to
`iq_target`; Id target is 0 (maximise torque per amp).

```
iq_target = (int32_t)pwm * FOC_IQ_MAX / BLDC_TIMER_MID_VALUE;
// clamp to ±FOC_IQ_MAX (defensive — pwm is already ±BLDC_TIMER_MID_VALUE upstream)
```

Same widening-multiply pattern as VLT (`1125 × 500 = 5.6e5`, fits int32);
result range is exactly `±FOC_IQ_MAX` at full throttle.

Each PI produces its axis voltage; chain the same way as VLT (inverse Park
with the same θ).

Constants (empirical starting points; bench-tunable):

- `FOC_IQ_MAX = 500` (≈ 3.9 A target at full throttle).
- `volatile int16_t foc_trq_v_max = 3000` — PI output clamp for Vd/Vq, **tighter than `FOC_VQ_MAX`**. If angle is off, the PI will wind the output all the way to this clamp trying to coerce measured Iq → target, so this — not `FOC_IQ_MAX` — is what caps worst-case stall current. Keep `volatile` so you can poke it via OpenOCD `mwh` on the bench without re-flashing.
- PI gains as right-shifts: Kp shift = 0 (Kp=1), Ki shift = 10 (integrator += err >> 10 per ISR).
- Anti-windup shift = 2: when output saturates to the clamp, subtract `excess >> 2` from the integrator (back-calculation).
- Reset both integrators whenever `mode_req != prev_mode` — stops prior wind-up from biasing the first tick's output.

The rest of the output chain is identical to VLT (inverse Park → iclarke_svpwm).

**Without current sensors** (`#if !defined(PHASE_CURRENT_A) || !defined(PHASE_CURRENT_B)`):
TRQ short-circuits to zero output — write `*y = *b = *g = 0` and return.
Unlike VLT, TRQ has no graceful-fallback semantics without measurement —
`i_rotor.d/q` would both be stuck at zero, so the Iq PI would see constant
error `= iq_target`, wind its output to `±foc_trq_v_max` within a handful of
ISRs, and drive the motor at that clamp regardless of throttle. Silent
runaway. Returning zero instead makes sensorless-build TRQ a visible no-op
the user can diagnose.

---

## Steer trim — always-on θ adjustment

Purpose: give the rider a live analog knob on θ so `FOC_ANGLE_OFFSET_BASE`
can be dialled in empirically on the bench. Every ISR, the joystick's
`steer` axis is low-pass-filtered, scaled to a Q8 angle, and added into
θ unconditionally. No button, no mode — the stick position **is** the
trim. At rest (stick centered), the trim contributes 0 to θ.

This section specifies *what the trim is* and *how it is scaled*. The
full θ composition (including the velocity-proportional lead) lives in
`foc_sample` step 6 — that is the single source of truth for how θ is
assembled each ISR.

**LPF** (in `foc_sample` step 2):

```c
steer_ema = ema_step(steer_ema, (int16_t)steer, FOC_STEER_LPF_SHIFT);
int16_t steer_smoothed = (int16_t)(steer_ema >> FOC_STEER_LPF_SHIFT);
```

`FOC_STEER_LPF_SHIFT = 10` → ≈ 64 ms time constant at 16 kHz. Attenuates
the ±10–30 count UART-packet jitter by ~1000×, keeping the integer-degree
θ output from flickering under packet noise while passing hand-motion
(< 5 Hz) cleanly. (θ is truncated to integer degrees in step 6, so the
LPF's job is stopping the low bit from chattering — not preserving
sub-degree precision.)

**Scaling** — full stick deflection (±1000 counts) → `±FOC_STEER_TRIM_RANGE_DEG
= ±30°` *nominal*; actual full-deflection output is ±31.25° due to
integer rounding of the per-count factor (see note below).

```c
#define FOC_STEER_TRIM_Q8_PER_COUNT  \
    ((FOC_STEER_TRIM_RANGE_DEG * 256 + 500) / 1000)   // rounded, = 8 at 30°
static inline int16_t steer_to_trim_q8(int16_t s) {
    return (int16_t)((int32_t)s * FOC_STEER_TRIM_Q8_PER_COUNT);
}
```

The *exact* per-count gain for 30° is `30 * 256 / 1000 = 7.68` Q8
degrees. Restricting the multiplier to an integer rounds that to `8`,
so `steer = ±1000` yields `±8000` Q8 = ±31.25°. That 1.25° overshoot
is below the 1° resolution of the final integer-degree θ output
(`((trim_q8 + 128) >> 8)` in step 6) and doesn't affect usability —
the RTT `trim:<…>` readback is self-consistent with whatever scaling
is in place, so the baked-in offset is correct regardless. Kept as an
integer multiply to avoid a per-ISR divide.

**State, file-local in `bldcFOC.c`:**

```c
static int32_t steer_ema;     // Q(FOC_STEER_LPF_SHIFT) LPF accumulator
```

That's it. No latch, no button state, no edge detection. The stick
position over time *is* the trim history.

**Why no button / no latch:** an earlier attempt had an Id-driven integrator
on θ gated by a tune button. It hit a two-equilibria trap — when Iq_true
went negative (bad initial offset, transient, or `SPEED_COEFFICIENT = -1`
confusion), the integrator's stable point flipped to the brake trough and
it actively drove the wheel to a halt. A hand on the stick can't do that
(rider's feedback is "wheel brakes = go the other way"). Also: no button
means no mode toggling, no falling-edge latch bug surface, no need to
explain when tune is "on."

**Two-equilibria caveat (as diagnostic, not a hazard).** `Id = 0` is
satisfied at two angles per electrical revolution, 180° apart: the torque
peak (accelerates under forward throttle) and the torque trough (brakes).
The rider recognises the braking basin immediately and doesn't pick that
stick position. If however `FOC_ANGLE_OFFSET_BASE` in source sits in the
brake-trough basin and the ±31° of stick range can't reach the accel
peak, add 180° to `FOC_ANGLE_OFFSET_BASE` in source and re-flash.

**Bench procedure — dial in `FOC_ANGLE_OFFSET_BASE`:**

1. Boot, wait for calib (~64 ms).
2. Enter VLT (mode 1). Apply a small positive throttle; wheel spins up.
3. Rotate the steer stick slowly while watching the RTT `Id avg` (red) line.
4. Find the stick position where `Id avg` sits closest to zero and the
   wheel feels like it's accelerating cleanly (not braking).
5. Read `trim:<…>` from the RTT log at that stick position (value is in
   Q8 degrees — divide by 256 to get degrees).
6. Add that many degrees to `FOC_ANGLE_OFFSET_BASE` in source. Re-flash.
7. Verify: boot, VLT, stick centered (trim = 0), `Id avg` should now sit
   near zero. Any residual is either DTC off / Hall learner not yet
   converged, or speed-dependent cross-coupling (ω·Lq·Iq); dial the stick
   a further few degrees to see the curve.

Stick is free for actual joystick steering only *after* bake-in; until
then the rider lives with "steer is the trim knob, don't expect normal
steering behaviour." On a single-wheel solo build this is fine; on a
dual-wheel build, dial in first and then rely on the baked constant.

**Interactions:**

- **Hall learner**: immune. The learner measures hall-sector dwells from
  physical rotor position via `pos`; the steer trim changes θ used for
  Park/iPark but doesn't touch `pos` or `velocity_q8`. The gate has no
  tune-specific condition.
- **DTC**: independent. Dial in with DTC off first, then enable DTC and
  dial in again — expect a few degrees of shift.
- **TRQ / SPD**: the PI integrators see `i_rotor.d/q` through the trim'd
  θ, same as the output chain. Moving the stick in TRQ is usually
  unhelpful (the Id PI already forces Id = 0 via Vd, so the stick mostly
  shifts Vd's steady-state value). Dial in in VLT; the result carries
  into TRQ and SPD as a θ bias automatically.
- **BC**: doesn't use θ. Steer trim has no visible effect.
- **`pos_force`**: routes to BC, so same — no effect.

---

## Empirical constants summary

```
FOC_CALIB_DISCARD     32      # ADCs to drop at boot before offset accumulation starts (sensored builds only — #if-gated)
FOC_CALIB_SAMPLES     1024    # ADC offset average N samples at boot         (sensored builds only — #if-gated)
FOC_VQ_MAX            20000   # VLT Vq clamp (internal Q scale)
FOC_ANGLE_OFFSET_BASE 125     # degrees — empirical for this motor
FOC_OUT_SHIFT         4       # internal voltage → PWM delta
FOC_PLL_VEL_CLAMP     3000    # Q8 deg/ISR sanity clamp
FOC_PLL_NOMINAL_WIDTH_Q8 15360    # 60° << 8
FOC_PLL_ANCHOR_Q8     38400   # derived: (sector_centre_deg[0]<<8) - FOC_PLL_NOMINAL_WIDTH_Q8/2; see Hall PLL section
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

FOC_DTC_MAX                (DEAD_TIME / 2)  # PWM-delta correction clamp = 30 at DEAD_TIME=60
FOC_DTC_SHIFT              2       # soft-limiter slope; saturates at ~0.9 A
foc_dtc_en                 0       (volatile)  # flip after VLT alignment

FOC_HALL_LEARN_MIN_VEL     144     # Q8 deg/ISR ≈ 100 RPM mech at 15 PP, 16 kHz
FOC_HALL_LEARN_WINDOW_REVS 10      # full electrical revs per learning window
FOC_HALL_LEARN_EMA_SHIFT   3       # ~8-window time constant
FOC_HALL_LEARN_PWM_TOL     50      # |pwm − ref| bound for steady-state gate
FOC_HALL_LEARN_VEL_TOL     64      # Q8 deg/ISR bound for steady-state gate
foc_hall_learn_en          1       (volatile)  # learner default-on

FOC_STEER_LPF_SHIFT         10      # ~64 ms EMA on steer (attenuates ±10-30 count jitter to well below 1° of θ)
FOC_STEER_TRIM_RANGE_DEG    30      # nominal θ trim across full stick deflection (±1000 counts); actual ≈ ±31.25° after integer rounding (see "Steer trim")
FOC_STEER_TRIM_Q8_PER_COUNT ((FOC_STEER_TRIM_RANGE_DEG * 256 + 500) / 1000)  # = 8 at 30°
```

---

## Operational tooling — keeping RTT alive during bring-up

Every MCU reset (explicit reset, flash, brown-out from a current spike)
wipes RAM, which includes the SEGGER_RTT control block. openocd's cached
RTT handle goes stale and the plotter starves silently — the pane looks
alive but no new samples arrive. Over a bring-up session you hit this
dozens of times.

Two tools in the repo paper over this:

- **`rtt_supervisor.sh`** (project root) launches openocd and the plotter,
  then watches `/tmp/rtt.log` mtime. If it goes stale for >3s it kills
  and restarts openocd; the plotter's auto-reconnect thread picks the
  new stream up seamlessly. Start it once at the beginning of a bring-up
  session; it runs until you Ctrl-C or close the plot window.
- **Plotter tees every received line to `/tmp/rtt.log`** (via the
  `RTT_TEE` env var, default `/tmp/rtt.log`). Since openocd's RTT port
  9090 is single-consumer, a `nc localhost 9090` alongside the plotter
  would *steal* bytes from it. The tee lets you `tail -f /tmp/rtt.log`
  from any terminal for a live text view without touching the plot.

**Flash workflow**: kill supervisor → `pio run -t upload` → relaunch
supervisor. The supervisor's own stall-watchdog handles the post-upload
restart of openocd, so the plot window just reconnects — no manual
restart of anything except the flash itself.

---

## RTT log format (must match `rtt_plot.py` regex — don't reorder)

One line at `FOC_RTT_LOG_DIV` rate:

```
m:<MODE> adc_a=<raw_a> adc_b=<raw_b> Ia:<iy> Ib:<ib> Ig:<ig> Ialpha:<α> Ibeta:<β> Id:<d> Iq:<q> Idavg:<d̄> Iqavg:<q̄> ang:<θ> rpm:<rpm> Vq:<vq_cmd> p:<pos> f:<pos_force> trim:<steer_trim_q8>
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
    int16_t iy, ib, ig;   // ig = −(iy + ib), computed at populate time
    int16_t i_alpha, i_beta;
    int16_t i_d, i_q;
    int16_t i_d_avg, i_q_avg;
    int16_t theta;
    int16_t rpm;
    int16_t vq_cmd;
    int8_t  pos;
    uint8_t pos_force;
    int16_t steer_trim_q8;   // current steer_to_trim_q8(steer_smoothed), for the RTT `trim:` field
} log_snapshot_t;

static log_snapshot_t   log_snapshot;              // plain — flag orders access
static volatile uint8_t log_pending;  // 0 = ISR may write; 1 = main must read
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
static const char *mode_str(int8_t m) {
    static const char *names[] = { "BC", "VLT", "SPD", "TRQ" };
    return (m >= 0 && m <= 3) ? names[m] : "??";
}

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

`mode_str` is file-local in `bldcFOC.c` (next to `FocRttPoll`). The `"??"`
fallback covers nothing legitimate — `mode_req` is masked to `0..3` at the
sense step — but is a cheap "this snapshot is wrong" tell if a future change
ever stores a wider value into the snapshot.

**Why it's race-free** without locks:
- ISR writes snapshot fields only when `log_pending == 0`. Main reads
  fields only when `log_pending == 1`. The flag acts as a one-bit mutex.
- If the main loop is slow (e.g. blocked in another poll), ISRs just skip
  logging silently — dropped samples, never torn data.
- `log_pending` is a single byte; access is atomic on M3.
- **Why the struct isn't `volatile`**: the synchronization point is
  `log_pending`, which *is* `volatile`. The ISR's sequence — populate
  fields, then set `log_pending = 1` — is preserved as written because
  ARMv7-M guarantees in-order stores from a single execution context, and
  the `volatile` store to `log_pending` forces the compiler to flush prior
  stores rather than hoist the flag-set above them. On the reader side,
  `FocRttPoll` checks `log_pending` first and only then touches the struct,
  so the read ordering is enforced by control-flow dependency. Making the
  struct itself `volatile` would defeat compiler optimisations (field-by-
  field loads/stores blocking any coalescing, no dead-store elimination on
  fields the log doesn't use) without adding any safety — the flag already
  provides the fence. This is the standard lockless-single-producer-single-
  consumer pattern.

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
│   ├── Makefile              # `make` → build+run ./tests
│   ├── test_helpers.h        # EXPECT_* macros
│   ├── test_main.c           # declares test_failed, calls each run_*_tests
│   ├── test_sincos.c
│   ├── test_transforms.c
│   ├── test_bc.c             # BC 6-step table parity
│   ├── test_pll.c
│   ├── test_pi.c
│   ├── test_dtc.c
│   └── test_hall_cal.c
```

**Single-binary runner.** All `test_*.c` files compile + link with
`bldcFOC_core.c` into one `./tests` executable. Each per-area file exposes
`int run_<area>_tests(void)` and uses the shared `test_failed` flag
(defined once in `test_main.c`, extern-declared in `test_helpers.h`).
`test_main.c` calls every `run_*_tests` unconditionally, so a failure in
the first suite doesn't hide failures in later ones — you see the full
damage in one pass. `./tests` exits 0 iff `test_failed == 0`. Separate
binaries were considered but rejected: 6+ invocations per manual run, and
`set -e` loops in Makefiles hide downstream failures on the first error.

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
  stated invariant in the Park section); iClarke + SVPWM preserves
  **line-to-line voltages** (`y−b`, `b−g`, `g−y`) — *not* phase sum.
  *Earlier drafts of this brief claimed "three phases sum to 0 after SVPWM
  centering" — that's wrong: min-max injection deliberately adds zero-
  sequence common-mode, which shifts the sum. L2L differences are what
  the bridge actually delivers to the motor and what the injection
  preserves.* SVPWM min-max injection produces the expected ~15% headroom
  vs raw sinusoidal (peak phase-to-common amplitude drops to `√3/2 ≈
  0.866` of the raw sinusoidal peak, freeing that margin for a larger
  αβ on the same rail); `ab_scale_for_pwm` is a plain right-shift by
  `FOC_OUT_SHIFT`.

- **`test_bc.c`** — BC 6-step table parity with `bldcBC.c` (same `(y,b,g)`
  per pos 1..6); `pos ∈ {0, <1, >6}` produces `0, 0, 0`; linear in pwm
  (negating pwm negates every non-zero phase); phase sum is always zero
  (BC drives exactly two phases and floats the third, which is the
  physical property that distinguishes BC from SVPWM).

- **`test_pll.c`** — steady forward rotation (pos sequence `1→2→3→4→5→6→1`
  with fixed dwell between edges) produces positive velocity and
  monotonically increasing angle; reverse (`1→6→5→4→3→2→1`) gives negative
  velocity; direction-aware snap lands at the correct sector boundary
  (forward → lower, reverse → upper) — this was explicitly the subtle bug
  that cost bring-up time last round; velocity saturates at
  `±FOC_PLL_VEL_CLAMP`; first-call seeding (`prev_pos == 0`) sets angle to
  sector centre, velocity to zero, and returns `kind == PLL_EDGE_SEED`;
  glitches (pos delta of ±2, ±3) leave velocity and `prev_pos` unchanged
  and return `kind == PLL_EDGE_GLITCH` (distinct from the seed case);
  valid edges return `kind == PLL_EDGE_VALID` with the dwell/sector pair
  populated; `pos == 0` still advances the accumulator and increments
  `sector_dwell` but doesn't update `prev_pos`; `sector_dwell` saturates
  at `UINT16_MAX` after >65535 calls with no edge (no wrap to 0 — the
  next edge then produces near-zero velocity, not a spike).

- **`test_pi.c`** — integrator output grows by `err >> Ki_shift` per step
  for constant error (raw accumulator grows by `err` every tick, shift
  applied on read); no dead zone for `|err| < 2^ki_shift` (the
  full-precision-accumulator property — after `2^ki_shift/err` ticks the
  output increments by 1); alternating `+err, -err` for equal counts
  returns the integrator exactly to its start (locks in `signed_shift`'s
  half-away-from-zero rounding — naive ASR drifts); clamp saturates
  output at the bound; anti-windup back-calc subtracts `excess >>
  AW_shift` from the integrator's output contribution (raw-accumulator
  decrement = `(excess >> aw_shift) << ki_shift`); zero-error holds
  output constant; mode-change integrator reset (caller responsibility,
  but test the `integrator_raw = 0` reset-to-zero behaviour);
  `signed_shift` symmetry across sign of shift (positive → right-shift
  with half-away-from-zero rounding, negative → left-shift, zero →
  identity); `ema_step` settles to `sample << shift` for constant input
  within the expected time constant (`~2^shift` samples), holds DC, and
  attenuates a unit step by ~`1/2^shift` per sample.

- **`test_dtc.c`** — `dtc_term(0) == 0`; monotonic through zero
  (non-decreasing on a ramp across `±FOC_DTC_MAX × 2^FOC_DTC_SHIFT`);
  saturates at `±FOC_DTC_MAX` well before int16 overflow; sign
  preservation (confirms soft-limiter doesn't invert). `dtc_apply` with
  balanced phases yields a zero-sum correction vector **only when every
  phase is in the pre-clip linear region** (`|i| ≤ FOC_DTC_MAX × 2^FOC_DTC_SHIFT`
  = 120 LSB ≈ 0.94 A). Above that, the clamp breaks linearity and the sum
  is nonzero by design. *Earlier drafts claimed zero-sum as a general
  invariant; it isn't.* A companion test exercises the clipped-phase case
  and asserts the specific non-zero sum to lock in the current clip
  constants — so if `FOC_DTC_MAX` moves, the test has to move with it
  (not a silent change).

- **`test_hall_cal.c`** — synthetic dwell histograms through the update
  math: uniform input leaves `sector_width_q8` at 15360 each; a skewed
  input (e.g. one sector 1.2× nominal, rest compensating) converges
  toward that distribution over multiple window cycles; the EMA
  attenuates single-window injected noise by `2^FOC_HALL_LEARN_EMA_SHIFT`
  exactly (post-update delta = measured-delta / 2^shift); `ema_shift = 0`
  snaps widths to measured in one step (no smoothing); `Σ sector_width_q8`
  stays **near** `(360 << 8)` after repeated updates, but drifts by tens
  of counts over many iterations — arith right-shift on negative deltas
  rounds toward floor, which introduces a systematic bias that
  accumulates. Tolerance in the test is ±50 counts (0.05% of the sum),
  not the "6-entry rounding" figure an earlier draft suggested. The
  learner doesn't self-renormalise; if long-term drift ever matters in
  practice, a `Σ = 360<<8` pass at window close would fix it cheaply.
  A zero `learn_sum` leaves the table untouched (the `total == 0` guard
  in `hall_widths_update`), matching the effect of the caller-side reset
  path after a gate violation.

### What host tests do NOT cover

- ISR timing / jitter / budget (bench only)
- DMA ordering, ADC valley sampling (bench only)
- `volatile` interactions and memory ordering (bench only)
- Actual hardware offsets, angle alignment to the physical rotor, mode
  transitions under real load (bench only)

Those belong in **Verification** below.

Fast feedback loop: `make test` in under 1 second; run it before every flash.
Current suite: 44 tests across 7 files, all passing under `-Werror -fwrapv
-fsanitize=undefined,address -Wdouble-promotion -Werror=float-conversion`.
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
| VLT forward ↔ reverse | With DTC on and the velocity-proportional θ lead in place (step 6), best-Id stick positions agree to within ~2–3° across directions. Residual is dead-time bias + ω·Lq·Iq cross-coupling. Larger gap (≥10°) → direction-aware hall snap or `lead_q8` sign/scale is wrong. |
| TRQ at throttle=0 | Quiet, near-zero current draw |
| TRQ mode-change | No kick when entering/leaving (integrators reset) |
| TRQ with bad angle offset | Current capped at ~`foc_trq_v_max`-driven stall; won't brown out the supply |
| DTC disabled, TRQ cruise | Baseline ripple + slight zero-crossing harmonics on Id/Iq |
| DTC enabled, TRQ cruise | 6×electrical ripple ↓, zero-crossing harmonics ↓; no visible chatter |
| Hall learner converges | After ~10 s of steady cruise >100 RPM mech, `sector_width_q8[]` deviates from 15360 per sector but `Σ = 360°<<8` within rounding |
| Learner frozen (`foc_hall_learn_en=0`) | Table stays at boot values indefinitely |
| Hall learner + DTC both on | Compound: cleaner Id/Iq than either alone; re-tune `FOC_ANGLE_OFFSET_BASE` once both are enabled |
| Steer trim live in VLT | Rotate the steer stick in VLT; `trim:<…>` in RTT tracks stick position with ~64 ms lag, `Id avg` moves with it. Park the stick where `Id avg` sits near zero with wheel accelerating. |
| Stick centered → zero trim | Centre the steer stick; `trim:<…>` in RTT settles to ~0 within ~200 ms (several LPF time constants). θ returns to pure `electrical_angle + FOC_ANGLE_OFFSET_BASE`. |
| Bake-in round trip | Note the value of `trim:` at the best stick position; add `trim/256` degrees to `FOC_ANGLE_OFFSET_BASE` in source and re-flash. Verify stick-centered `Id avg` is now near zero. |

Failure fingerprints:

- Id/Iq oscillating sinusoidally → Park matrix is the wrong sign variant (transpose it).
- VLT slower than BC at same throttle → scaling shift is too large; check math around 0.866·|Vq| / 2^FOC_OUT_SHIFT reaching ~±1125.
- Forward/reverse asymmetry on ideal trim, ≥10° → direction-aware hall snap missing, *or* `lead_q8` has the wrong sign or is missing entirely (a mis-scaled lead looks like an asymmetry that grows linearly with speed).
- Motor brown-outs on TRQ engage → `foc_trq_v_max` too high, or angle offset so wrong the Id PI can't compensate. Lower `foc_trq_v_max` first, then fix alignment.
- Id/Iq worse with DTC enabled → `FOC_DTC_MAX` too high (overcompensation, sign-flipped zero-crossing harmonics). Halve it, retest. If still worse, confirm DTC was off during the angle-offset tuning pass.
- Hall-learner widths drift to bizarre values (one sector ≫ others, or sum deviates far from `360°<<8`) → steady-state gate too loose. Tighten `FOC_HALL_LEARN_PWM_TOL` / `FOC_HALL_LEARN_VEL_TOL` first, then look for direction reversals or hall glitches inside the window — both `pos == 0` and non-adjacent `pos` jumps (the latter should surface as `edge.kind == PLL_EDGE_GLITCH` and reset the window; if widths still drift, verify the glitch-triggered reset path is actually wired).
- Learner converges cleanly on the bench but degrades riding performance → the bench test speed was below 100 RPM mech quantization threshold despite passing the gate; raise `FOC_HALL_LEARN_MIN_VEL`.
- Steer stick moved but `trim:<…>` in the RTT log doesn't change → `steer` global isn't being updated. Check the joystick is armed and the UART packet is flowing (`remoteUart.c` writes `steer` in the same ISR that writes `wState`). Watch raw `steer` via OpenOCD `mdw` directly to distinguish joystick-side vs firmware-side.
- Trim moves but `Id avg` doesn't track → θ read in the Park transform doesn't match the θ read in the inverse-Park. Both must come from the same `state.theta` computed in step 6. Grep `bldc_get_pwm` for any second `electrical_angle` composition.
- Wheel only ever decelerates when the stick is moved, in both directions → `FOC_ANGLE_OFFSET_BASE` sits in the brake-trough basin (the accelerate peak is 180° away). Add 180° to `FOC_ANGLE_OFFSET_BASE` in source and re-flash.
- Optimal steer-stick position drifts slightly between different speeds in the same direction → expected. Residual is `ω·Lq·Iq` cross-coupling (Vd=0 in VLT → steady-state `Id_true = (ω·Lq·Iq)/R`) plus a small Park-side rotation from using apply-θ for both transforms (step 6). Both direction-symmetric, both scale with speed. Pick a mid-speed cruise as your canonical dial-in point.
- Optimal steer-stick position differs between forward and reverse by **≤~3°** at the same speed → acceptable implementation slop. All clean analytical sources (ω·Lq·Iq, Park rotation, DTC-corrected dead-time) are direction-symmetric, so the ≤~3° is really "dead-time residual, hall-learner convergence, `lead_q8` timing precision, current-sensor channel mismatch" combined. Phase 2 TRQ Id PI absorbs it at ride time. If the gap is ≥10°, see the direction-aware-hall-snap / `lead_q8` bullet above.
- θ tracks fine at low/medium RPM but drifts and Id/Iq go ragged at high RPM → hall edge slew limited; PLL is hitting `±2` deltas and bailing on the glitch path, dead-reckoning θ from the last believable edge until the next adjacent transition. Confirm by watching `pos` in RTT — gaps where pos jumps non-adjacently around the high-RPM symptom. Mitigation: hall filtering in `bldc.c` (outside this brief's scope), or accept it as a high-RPM edge.
- TRQ barely moves the motor, PI outputs pinned at `±foc_trq_v_max`, `Iq` far from `iq_target`, throttle has little effect, direction doesn't flip cleanly across zero → VLT alignment is worse than VLT alone reveals. TRQ is a closed-loop **amplifier** of angle error: a 3–5° alignment residual that's benign in VLT saturates the Iq PI and collapses torque production. Don't try to tune TRQ gains against this — go back and re-dial `FOC_ANGLE_OFFSET_BASE` (with DTC in its final state) until VLT runs clean, then TRQ will come along.

---

## Non-goals

Defer (don't implement now, but leave the structure amenable):

- Mode hysteresis (OPEN↔FOC thresholds at 240/480 RPM in the reference)
- Iq protection limiter with speed-dependent `Vq_max_M1[]` LUT
- Fast-capture RAM buffer
- Adaptive angle-offset learning
- Saving calibrated offsets to flash
- Field weakening
