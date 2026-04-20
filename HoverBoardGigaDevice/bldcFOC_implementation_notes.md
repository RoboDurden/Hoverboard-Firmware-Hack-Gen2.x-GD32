# bldcFOC implementation notes

Companion to `bldcFOC_greenfield_brief.md`. The brief is the design
spec; this file captures what we found and decided while actually
implementing and bringing up Phases 1 and 2 on the bench — deviations
from the brief, empirical tuning values, motor electrical estimates,
and behavioral quirks worth remembering.

---

## Deviations from the brief

### `signed_shift` — reworked to proper half-away-from-zero

The brief's inline code block had a bias-direction bug: for negative
`v` it added `-2^(s-1)` before ASR, which over-rounds *exact
multiples* (e.g. `signed_shift(-1000, 3)` returned `-126` instead of
the correct `-125`) and broke the `+err` / `-err` symmetry that
`pi_step`'s integrator depends on to avoid drift. The brief's own
claim "rounds half-away-from-zero" is inconsistent with the formula
it shows.

Replaced with an explicit abs/sign split:

```c
int32_t half = (int32_t)1 << (s - 1);
return (v >= 0) ?  ((v + half) >> s)
                : -(((-v) + half) >> s);
```

`test_pi.c::test_signed_shift_roundtrip_symmetry` locks this in.

### `foc_sample(int pwm, int pos)` — extended signature

The brief specifies `foc_sample(pos)` but the hall-width learner's
steady-state gate needs `pwm` (`|pwm − learn_pwm_ref| < tol`).
Cleaner to pass it explicitly than to stash it in a file-local static
just before the call.

### `FOC_DTC_MAX` — literal mirror, not `#define`-derived

The brief wants `#define FOC_DTC_MAX (DEAD_TIME / 2)` with `DEAD_TIME`
pulled from `Inc/defines.h`. `bldcFOC_core.c` must be host-compilable
without the MCU headers, so `FOC_DTC_MAX` is defined as the literal
`30` in `bldcFOC_core.h`. `bldcFOC.c` asserts `#if DEAD_TIME != 60
#error …` so a `DEAD_TIME` change on the target side can't silently
diverge from the mirror.

### `FOC_PLL_ANCHOR_Q8` — literal, not computed

Brief's expression
`((int32_t)sector_centre_deg[0] << 8) - FOC_PLL_NOMINAL_WIDTH_Q8/2`
isn't a C compile-time constant (array element lookup doesn't qualify).
Defined as `#define FOC_PLL_ANCHOR_Q8 38400` with a derivation comment.
`sector_centre_deg[]` lives as `static const` in `bldcFOC_core.c`.

### Hall debounce in `bldc.c` — brief's "don't modify" broken

The brief explicitly puts `bldc.c` off-limits, but the brief's own
high-RPM failure fingerprint ("θ drifts and Id/Iq go ragged at high
RPM → hall edge slew limited, PLL bails on glitch path") was exactly
what we hit past ~511 RPM mech. Root cause is `bldc.c:238`'s unfiltered
hall read. Added a 2-read temporal filter (`hall` only updates when
two consecutive ISR reads agree) — 1 ISR of worst-case latency,
~5% of a hall sector at 511 RPM, well inside PLL tolerance. Commit
`494278f`.

---

## Empirically tuned constants

| Constant | Brief default | Current | Source |
|----------|:---:|:---:|---|
| `FOC_ANGLE_OFFSET_BASE` | 125° | **156°** | Bake-in from steer-trim on this motor (+31°). |
| `FOC_IQ_MAX` | 500 (~3.9 A) | **1500** (~11.7 A) | Continuous-safe for hub-motor copper loss. |
| `foc_trq_v_max` | 3000 | **6000** | ~8 V peak phase-to-common; ~3.8 A PSU at stall. |

Forward/reverse and speed-dependent residuals after tuning (at
`FOC_ANGLE_OFFSET_BASE=156`, DTC off, learner converged):

| pwm (throttle) | trim (Q8) | trim (°) |
|:---:|:---:|:---:|
| −247 | 722 | +2.82 |
| +254 | 587 | +2.29 |
| +729 | 440 | +1.72 |

- Fwd/rev gap: ~0.5° at similar |speed| — tight (brief's tolerance is ≤3°).
- Drift with speed (~1.1° from 254 → 729): matches the brief's
  `ω·Lq·Iq/R` cross-coupling prediction (direction-symmetric,
  speed-proportional). TRQ's Id PI absorbs this closed-loop at ride
  time; no source change needed.

---

## Motor electrical characteristics (layout 2-1-20, this specific motor)

Back-derived from bench observations:

- **Phase winding resistance**: ~0.3 Ω/phase. Derived from
  `foc_trq_v_max=6000 → V_applied_peak ≈ 7.8 V`, stall PSU draw 3.8 A,
  SVPWM ~29% modulation depth → phase current ~13 A, two phases in
  series → `2R ≈ 0.6 Ω`.
- **Bus-to-phase current ratio at stall**: ~1:3.4 (matches SVPWM duty).
- **Hall sector widths**: learner converges after ~10 s steady cruise
  above ~100 RPM mech; values drift from nominal 15360 by a few
  hundred Q8 units per sector — manufacturing variance.

---

## Thermal guidance (rule of thumb, no firmware protection)

**There is no in-firmware thermal protection.** No temp sensor read,
no I²t integrator, no foldback. Rider discretion applies.

| Regime | Safe phase current |
|---|---|
| Continuous cruise | 8–12 A RMS |
| Short-term peak (<~few s) | 25–40 A |
| Long sustained heavy load | ≤15 A — exceed and motor reaches "boiling hot" territory (seen last summer riding a chair under load on an incline) |

Sealed-hub geometry has poor airflow; thermal time constant is
several minutes. Brief peaks are survivable, long hauls are not.

Current settings (`FOC_IQ_MAX=1500`, `foc_trq_v_max=6000`) keep
continuous target in the safe zone and stall current at ~13 A
(survivable briefly). Raising `foc_trq_v_max` further (10000 →
~21 A stall, 14000 → ~30 A stall) is a short-bursts lever, not a
continuous one.

Minimal future thermal protection: I²t integrator on Iq (`integrator
+= iq² − cruise²; if > threshold, reduce iq_target`). ~15 lines,
catches the chair-on-incline case without any sensor. Out of current
phase scope.

---

## Disarm behavior (discovered during Phase 2 bring-up)

Throttle→0 while the joystick remains connected **does not disarm**
the motor. `bldc.c:214` treats only `bldc_enable==RESET`,
`timedOut==SET`, and DC overcurrent as disarm triggers. Throttle-to-
zero just sets `iBldcInput=0`, which the PWM input low-pass
(`FILTER_SHIFT=14`, ~1 s τ) decays toward zero. During the decay,
TRQ is still actively regulating `iq_target` (which is also decaying
toward zero), so hand-loading the wheel causes the Iq PI to push back
via Vq. Surfaces as "wheel speeds back up when I slow it by hand
during coast". Not a bug — it's the soft-brake filter's intended
behaviour — but worth knowing when interpreting bench sessions.

Real disarm (hardware PWM off) requires joystick timeout or
`STATE_Disable` (bit 6 of `wState`) set.

---

## RTT log note — augmented θ offset isn't a separate field

`ang:<θ>` is the fully-composed theta (electrical_angle + BASE + trim
+ lead — varies with rotor position). `trim:<q8>` is the steer
contribution alone in Q8 degrees. The sum
`FOC_ANGLE_OFFSET_BASE + trim/256` isn't emitted directly; for the
bake-in you just divide the `trim:` field by 256 and add those degrees
to the source constant.
