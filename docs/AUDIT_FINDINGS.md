# AUDIT_FINDINGS.md — V1 Closure Audit, Phase A

**Date:** 2026-07-06 · **Branch:** `audit/v1-closure` · **Auditor:** Claude Code (Fable 5)
**Scope:** Adversarial cross-module audit per the V1-closure handoff. Read-only pass —
no source edits. Every constant cited below was read from the working tree in this
session (commit `6bb78b0`).

**Label key:** `Observation` = read directly from code. `Interpretation` = reasoned
from code. `Speculation` = needs hardware/library verification.
**Severity:** CRITICAL / HIGH / MED / LOW. **Fix size:** S / M / L.

**Repo-vs-handoff discrepancies (protocol rule 3 — code wins):**
- All companion docs live at **repo root**, not `docs/` (`CLAUDE.md`, `ARCHITECTURE.md`,
  `CURRENT_FOCUS.md`, `GAIT_DESIGN.md`, `CODING_STANDARD.md`, `STEERING_PLAN.md`).
  This file is created at `docs/AUDIT_FINDINGS.md` as the handoff demands. Note
  `.gitignore` ignores `*.md` globally, so no doc (including this one) is tracked.
- Several handoff-referenced docs do not exist in the tree
  (`POST_CONTACT_ANKLE_RELEASE_CONTEXT.md`, `FOOT_IMU_PHASE2_PLAN.md`, etc.).
- Two pre-approved Phase B fixes are **already satisfied by code** (B4) or **based on a
  false premise** (B5) — see A-22 and A-23.

---

## 0. Headline answers to the handoff's open questions

| Question | Answer | Evidence |
|---|---|---|
| Does anything submit `ANKLE_PITCH` at `SOURCE_BALANCE`? (gate for ankle release) | **No.** `SOURCE_BALANCE` submits exactly two channels: `IDX_TORSO_PITCH`, `IDX_TORSO_ROLL` (TorsoStabilizer). | `torso_stabilizer.cpp:76,91`; no other `submit(SOURCE_BALANCE, …)` exists in the tree |
| Rollover flag #1: do `sideIdx()` and `static_cast<uint8_t>(Side)` agree? | **Yes — confirmed.** `sideIdx(s)` *is* `static_cast<size_t>(s)`; `side.h:39-40` static_asserts LEFT=0/RIGHT=1. `_rolloverSide` (cast) indexes `_lastAnkleCmdDeg[]`/`_footEst[]` (written via `sideIdx`). Same numeric value by construction. | `side.h:34-45`, `gait_controller.cpp:1551,2086` |
| Is `postContactBodyPitchPeak` neutral-corrected? (B4) | **Already implemented.** `pitchCorr = torso.pitch - _cfg.torsoPitchNeutralRad` at accumulation. The doc-listed flag #2 is stale. | `gait_controller.cpp:198` |
| Is `INV_OMEGA0` stale (computed at 250 mm)? (B5) | **No.** `GAIT_CAPTURE_INV_OMEGA0_S = 0.1498` = √(0.220/9.81) = 0.14975 — **correct for `H_COM_MM = 220`**. What *is* stale is the comment block in `robot_geometry.h` ("Measured: ~25 cm", "ω₀ ≈ 6.26 rad/s at h = 250 mm"). | `project_wide_defs.h:294-299`, `robot_geometry.h:49-60` |
| Is PCA9685 pulse math 50 Hz-only or frequency-parametric? (gates B6) | **Frequency-parametric already.** `degToPulse()` outputs µs; the single hardware write is `_pwm.writeMicroseconds()`, which converts µs→counts from the chip's live prescale. Changing `SD_PWM_FREQ_HZ` alone keeps pulse widths correct. Resolution: 20000 µs/4096 ≈ **4.88 µs/count at 50 Hz** (≈0.66° on a 270°/2000 µs servo) vs 3333 µs/4096 ≈ **0.81 µs/count at 300 Hz** (≈0.11°). Hard ceiling: `SD_USMAX` (2500 µs) must fit inside the frame → ≤ ~333 Hz with margin (at 400 Hz the max pulse equals the whole period). | `servo_driver.cpp:29-52` (Observation); Adafruit `writeMicroseconds` internals (Interpretation) |
| Is F3 (foot-contact lift gate) wired? | **Yes — implemented AND default-on**, contradicting every doc. `main.cpp` builds a `FootContactSnapshot` per tick and passes it to `gaitController.update()`; default `GaitConfig::liftGateMode = LiftGateMode::STANCE_FLAT` consumes `isFootFlat` as the lift gate. | `main.cpp:409-425,443`, `gait_controller.h:161`, `gait_controller.cpp:1869-1931` |

---

## 1. Findings — ranked by severity × fix-size

### A-01 | control/comm boundary | HIGH | class 1 | Observation
**Full-struct config RMW between the AsyncTCP task and the 400 Hz main task — the complete enumeration (input to fix B1).**

The WiFi (async_tcp) task parses `CMD:*` and writes config that the main task reads
every tick. Classification of **every** cross-task config path found:

| Struct | WiFi-task writer(s) | Write style | Main-task reader | Classification |
|---|---|---|---|---|
| `BalanceConfig` | `BALANCE_ON/OFF`, `PITCH_ON/OFF`, `ROLL_ON/OFF` (WebComm.cpp:1310-1360), `BALANCE_TUNE` (:1722), `RECOVERY_TUNE` (:1862), `RESETP` (:1210) | `getConfig()`→modify→`setConfig()` (full ~180 B struct copy) | `BalanceController::update()`/`computeCorrection()` every tick | **Racy (full RMW)** |
| `TorsoConfig` | `TORSO_PITCH/ROLL_ON/OFF` (:1364-1395), `TORSO_TUNE` (:1815) | full RMW | `TorsoStabilizer::update()` every tick | **Racy (full RMW)** |
| `GaitConfig` | `GAIT_TUNE` (:2228), `TOUCHDOWN_TUNE` (:2145), `COMPLIANCE_TUNE` (:2195); also `tareToTorso()` writes two `_cfg` fields directly from the WiFi task (gait_controller.cpp:2631-2632) | full RMW (struct ~60 fields, ~260 B) | `GaitController::update()` + every helper, every tick | **Racy (full RMW)** — torn read can violate cross-field invariants (`stanceHeightMm`/`stepHeightMm` pair, `heelStrikeGatePhase < touchdownGatePhase`) for a tick |
| `WeightShiftConfig` | `WEIGHT_SHIFT_TUNE` (:1423) | full RMW | `WeightShift::update()`, `GaitController::_comShiftTilt()`/`_futureStanceTilt()` (reads `getConfig().ankle_shift_deg` per tick) | **Racy (full RMW)** |
| `FootImpactConfig` | `FOOT_IMPACT_TUNE` (:1517) → `setConfig()` | full RMW (clamped in `_clampConfig`) | `FootImpactDetector::_updateSide()` every tick | **Racy (full RMW)**, low blast radius |
| `SwingFootLevelerConfig` | `SWING_LEVELER_TUNE` (:1586) | full RMW | `SwingFootLeveler::update()` every tick | **Racy (full RMW)** |
| `FootContactConfig` | `FOOT_CONTACT_TUNE` (:1471) | **in-place per-field stores** through pointer (no copy-back) | `main.cpp` snapshot build + gait gate | **Safe-ish** (each field a single aligned 32-bit store; no lost-update since WiFi is sole writer) |
| `SteeringConfig` | `STEER` / `STEER_TUNE` (:2071-2135) | **targeted atomic setters** (`setTurnCmd`, `setEnabled`, `setYawStepDeg`, `setTurnSlewPerSec`, `setSweepWindow` with defensive re-clamp in `yawLawDeg`) | `SteeringController::update()` | **Safe-ish (by design — the model B1 should generalize)** |
| `FilterConfig` (StateEstimator) | none today | — | estimator | **No race**, but a latent trap: `setConfig()` would overwrite the calibration-computed `gyro_bias_*` with the caller's copy (state_estimator.h:149, biases live in `_cfg`). Any future `FILTER_TUNE` must not use full-struct set. |
| `WalkingInstrumentation` | `WALKING_INSTR:reset/enable` (:1551) | `reset()` / `setEnabled()` mutate state read/written by main-task `update()` | telemetry only | **Racy, telemetry-only** |
| `FootContactConfig.loaded_pitch_sign` etc. | (covered above) | | | |

Additional detail — the classic "lost update" variant is *mostly* retired on
`BalanceConfig`: after the consolidation refactor no per-tick main-task writer pushes
into `BalanceConfig` any more (WeightShift's `setAnkleRollBias` path was removed;
`ankle_roll_bias_l/r_deg`, `swing_leveler_roll_bias_*` and their setters are now
**vestigial** — only `oe_clear()` and `resetOutputState()` write them). The live
hazards are (a) torn multi-field reads, (b) enable flags flipping between
`BalanceController::update()`'s transition detector and Gate 1, (c) a WiFi-task RMW
overwriting a concurrent main-task `resetOutputState()` field write.

- **Consequence:** one-tick inconsistent gains/thresholds; in the worst torn-read case
  a cross-field invariant is violated for one tick (bounded downstream by output
  clamps and joint limits). Matches the documented "don't tune while armed" workaround.
- **Proposed fix:** B1 command staging exactly as specified — parsers write into a
  staging slot + generation counter; main task applies at the top of the 400 Hz gate
  (before `weightShift.update()`, which is the first config consumer of the tick).
  One generic mechanism (e.g. a `Staged<T>` double-buffer template with a `pending`
  flag written last) reused by all seven racy structs. Keep Steering's targeted
  setters and `FootContactConfig`'s in-place writes unchanged. Size **M**.
- **Regression risk:** low — bit-identical when no tuning occurs mid-run; apply point
  must be verified to precede *all* config readers (only `oe_loop()` and
  `servoController.update()` run before the gate, and neither reads these structs).

### A-02 | control/comm boundary | HIGH | class 1 | Observation
**WiFi task drives controller state machines directly, not just config.**

Beyond config structs, the WS parser calls, on the async_tcp task:
`_gaitCtrl->start()/stop()/singleStep()/nextPose()/triggerRecovery()/tareToTorso()/submitStancePose()`,
`_weightShift->trigger()/forceCenterImmediate()`, `_balCtrl->resetOutputState()`,
`_servoCtrl->captureSessionNeutral()/clearSessionNeutral()/setSessionTrimDeg()/loadPose()/saveCurrentPose()/applyNamedMovement()/setJointSpeed()/setTransmissionGain()`,
and `oe_estop()/oe_clear()` (which themselves call `servoController.resetToNeutral()`,
`gaitController.start(IDLE)`, `weightShift.forceCenterImmediate()` — see A-03).
Each mutates multi-field state that `update()` on the main task may be reading
mid-tick (e.g. `triggerRecovery` rewrites `_state.mode`, `_rcvSub`, `_rcvStep*`
while `update()` could be between its mode check and `_runRecoveryStep`).

- **Consequence:** rare single-tick mixed-state execution (e.g. one tick built from
  half-old, half-new FSM state). No unbounded output — every path still ends in
  IK soft limits + joint clamps — but it defeats the "one consistent state per tick"
  assumption the FSM code is written against.
- **Proposed fix:** fold *commands* (not just config) into the B1 staging mechanism:
  parser enqueues an event (start/stop/trigger/tare/…), main task drains the queue at
  the tick boundary. Size **M** (shares the B1 mechanism). Document `ESTOP` as the one
  deliberate exception (it must act immediately — see A-03).
- **Regression risk:** low-moderate — command latency grows by ≤1 tick (2.5 ms),
  imperceptible; e-stop must NOT be staged.

### A-03 | safety path | MED | class 1/6 | Observation + Interpretation
**`oe_estop()`/`oe_clear()` run heavy multi-module sequences on the WiFi task.**

`oe_estop()` (main.cpp:169) does `resetToNeutral()` (16 immediate PCA9685 writes on
Wire0) *before* raising OE, and `oe_clear()` (main.cpp:181) additionally resets
balance/torso/WS/gait/leveler/impact state — all callable from `CMD:ESTOP`/`CLEAR_ESTOP`
on the async task while the main task is mid-tick on the same modules and the same
I²C bus. Arduino-ESP32's `TwoWire` serializes individual transactions, so bus frames
don't interleave, but `JointModel::_currentPulse/_targetPulse` writes race with
`JointModel::update()` on the main task, and `oe_clear()`'s `gaitController.start(IDLE)`
is the A-02 hazard on the safety path itself. Mitigations already present: OE is HIGH
during the estop window (outputs physically disabled), `MotionManager::flush()` drains
slots while estopped, and `_oe_estopped` is `volatile`.

- **Consequence:** torn smooth-stepper state during estop entry/clear; a stale
  `_currentPulse` could cause one out-of-sequence servo write at re-enable — bounded
  by `clampPulse` and joint limits. Low probability, safety-adjacent.
- **Proposed fix:** keep OE pin action immediate on the WiFi task (latency matters);
  move the *state reset* work (`resetToNeutral`, controller resets, `start(IDLE)`)
  into the main task, triggered by a staged "estop-entered/cleared" event. Size **M**.
- **Regression risk:** moderate — touches the e-stop sequence; per the handoff scope
  ("OE/e-stop paths read-only") this is **documented, not fixed** in this pass.

### A-04 | motion_manager | HIGH | class 1 | Observation
**`MotionManager::_slots` is written by both tasks; `submit()` sets `occupied` first.**

WiFi task writes slots via slider messages (`WebComm.cpp:2739`), `RESETP` (:1231), and
the `IK_*` commands (:2647-2707); the main task writes via all controllers and reads/
clears in `flush()`. `submit()` (motion_manager.cpp:61-66) stores
`occupied=true` **before** `angleDeg` — if `flush()` preempts between those stores it
dispatches a half-written command: for `SOURCE_UI` the frame is *absolute servo
degrees*, so a torn `angleDeg=0.0` means **0° absolute** → the joint slams to its
clamped anatomical limit (e.g. ch0 right-ankle-roll to −25°).

- **Consequence:** rare single-joint jerk to a limit on a UI interaction racing the
  tick. Window is a handful of instructions at 240 MHz — rare but real, and the
  failure is a physical motion.
- **Proposed fix (S, behavior-preserving):** in `submit()`, write
  `angleDeg`/`source`/`priority` first and `occupied=true` **last** (single aligned
  bool store as the publish flag). This shrinks the torn window to "command missed
  this tick" instead of "garbage dispatched". Full closure comes from routing WS
  submissions through the B1 staging queue. **Phase C candidate** (CRITICAL-leaning
  HIGH, size S, single module, no interface/default change).
- **Regression risk:** near-zero — same values written, different order.

### A-05 | balance_controller | HIGH | class 4 | Observation
**`BalanceConfig::pitch_setpoint_rad = 10.0f` — ten radians (~573°) as the default pitch setpoint.**

`balance_controller.h:34` (`// rad. WS: "setpoint"`). Used directly:
`error = predictedPitch - _cfg.pitch_setpoint_rad` (balance_controller.cpp:277).
Dormant only because `pitch_enabled` defaults `false`. The moment an operator sends
`PITCH_ON`/`BALANCE_ON` without first re-tuning `setpoint`, error ≈ −10 rad →
deadband trivially exceeded → full gain boost → `u` saturates at
`−max_correction_deg` → `dx` rails to the +25 mm clamp and stays there. The
`BALANCE_TUNE:setpoint` key is also **unclamped** at parse time (WebComm.cpp:1751),
so the bad value round-trips through the UI slider sync (`sp=` echo) as truth.
Torso's analogous default is `0.0f` (torso_stabilizer.h:46), supporting the
interpretation that 10.0 is an entry error (degrees typed into a rad field, or a
stray tuning session persisted by reflash).

- **Consequence:** arming pitch balance with defaults commands a persistent saturated
  forward/backward task-space push — a stability event, not a subtle bias.
- **Proposed fix:** default `0.0f` + parse-time clamp (`±0.5 rad` mirrors the tare
  clamp). Size **S** — but this **changes a shipped default**, which the handoff scope
  reserves for human approval. **Documented, not fixed; flagged for decision.**
- **Regression risk:** none at runtime today (field unused while disabled); if the
  operator relies on the current value deliberately, changing it alters their armed
  behavior — hence the human gate.

### A-06 | imu (torso) | HIGH | class 5 | Observation + Speculation
**Torso BMI160 has no per-read failure detection — `valid=true` forever after init.**

`IMU_update()` (imu.cpp:72-91) calls `BMI160.getMotion6(...)` and unconditionally sets
`_latestData.valid = true`. No I²C error status is checked (Speculation: the BMI160Gen
API may not surface one — needs library check). Contrast the foot driver, where any
NACK/short read → `valid=false` (lsm6dsv_driver.cpp:134-161) and every consumer gates.
If the BMI160 or Wire0 degrades mid-gait, the estimator integrates stale/garbage
counts as valid data and Balance/Torso/gait trim/capture all act on it. The
`_accelSanityWeight` band rejects non-1g accel, but gyro garbage integrates directly.

- **Consequence:** a torso-IMU failure mid-run produces sustained wrong corrections
  (bounded by clamps) instead of the designed tick-skip. This is the largest silent
  sensor-failure path in the system.
- **Proposed fix:** (a) check whatever status BMI160Gen exposes; if none, (b) add a
  cheap plausibility gate (e.g. WHO_AM_I heartbeat at 1 Hz, or all-zero/frozen-sample
  detection over N ticks → `valid=false`). Size **M**. Not local/S → **document only**
  this pass.
- **Regression risk:** a too-aggressive frozen-sample detector could false-trip during
  genuine stillness (gyro deadband makes identical samples plausible) — needs care.

### A-07 | gait_controller / docs | HIGH | class 3/2 | Observation
**F3 lift gate is implemented and default-enabled; four docs say it doesn't exist.**

Default `liftGateMode = STANCE_FLAT` (gait_controller.h:161) makes `isFootFlat()` of
the stance foot a *hard control gate* for every lift (gait_controller.cpp:1904).
`main.cpp:409-425` builds the snapshot and passes it into `update()`. CLAUDE.md,
ARCHITECTURE.md §1/§5 ("telemetry only — not wired into any controller yet"),
CURRENT_FOCUS §1B ("F3 … Do not start until F2 is rock-stable"), and GAIT_DESIGN §2's
lift-gate pseudocode (only progressOK+imuOK) are all stale. Behavior consequence of
the *code* (fail-safe, good): a dead/uncalibrated foot IMU → `stanceFlat=false` →
gate never opens → `waitShiftMaxMs` (1500 ms) aborts to IDLE cleanly. Note also the
control-path predicate uses **zero neutrals** (`isFootFlat(state, cfg)` overload;
main.cpp:416 comment) while telemetry previews the tared version — a mounting offset
larger than the 0.035/0.05 rad flat band would permanently close the gate.
- **Consequence:** doc-driven decisions (e.g. the planned "F3 promotion") are being
  made against a world that already shipped; the zero-neutral flat band is a latent
  tuning trap after any chassis change.
- **Proposed fix:** B7 doc sync (S). The zero-neutral gate observation should be added
  to CURRENT_FOCUS's validation-gate table.
- **Regression risk:** none (docs).

### A-08 | project_wide_defs / defaults | HIGH (trust), LOW (each) | class 3 | Observation
**Systemic default-vs-comment/doc divergence — the shipped defaults are not what any document or adjacent comment says.**

Read from `project_wide_defs.h` / headers this session (value ← what the adjacent
comment or companion doc claims):

| Constant | Shipped | Comment/doc claims |
|---|---|---|
| `GAIT_YIELD_DORSIFLEX_DEG_DEFAULT` | **2.0** (+ `yieldEnabled=true`) | "SHIPS AT 0 — a true no-op"; CURRENT_FOCUS/GAIT_DESIGN: "ankle release **specified, not implemented**" — it is implemented (`_yieldBiasDeg` et al.) **and active by default** on every detected touchdown. See A-09. |
| `GAIT_STRIDE_COMPLETE_FRAC_DEFAULT` | **0.2** | "Ship at 1.0 — flashing must change nothing"; also violates its own guidance "keep ≤ touchdownGatePhase" only by accident (tdGate now 0.45) |
| `GaitConfig::touchdownGatePhase` | **0.45** | header comment "Default 0.7"; multiple other comments cite 0.7 |
| `GaitConfig::liftGateThreshold` | **0.4** | comment "0.70 = 70% shift complete" |
| `GAIT_STABILIZE_MS` | **180** | CURRENT_FOCUS/GAIT_DESIGN: "default 400" |
| `GAIT_LIFT_DWELL_MS_DEFAULT` | **40** | comment "80 ms ≈ 32 ticks" |
| `GAIT_TD_COMPLIANCE_ENABLED_DEFAULT` | **false** (depth 2.0) | docs: "W4 … default ON" (depth 4 mm) |
| `GaitConfig::comLatTrimEnabled` | **false**, `GAIT_COM_LAT_TRIM_GAIN_DEFAULT=0.0` | docs: "W2 … default ON … validated, meaningful improvement" |
| `GAIT_COM_LAT_TRIM_KD_DEFAULT` | **0.01** | comment: "DEFAULT 0.0 — W5 is INERT" |
| `GAIT_LAT_CAPTURE_GAIN_DEFAULT` / `_OFFSET_MAX_MM` | **1.0 / 30** | comments: "DEFAULT GAIN: 0.15", "DEFAULT OFFSET CAP: 15 mm" |
| `GaitConfig::latCaptureSign` | **−1.0** | defs block describes intended-true capture sign as the default |
| `GAIT_SWING_APEX_FRAC_DEFAULT` | **0.2** | comment: "Default lowered 0.5 → 0.15"; GAIT_DESIGN says 0.2 ✓(doc newer) |
| `GAIT_HEELSTRIKE_GATE_PHASE_DEFAULT` / `_RELEASE_MS` / `_DORSIFLEX` | **0.2 / 500 / 3.0** | comments: "Default 0.50 … range 0.45–0.55", "240 ms … range 200–280", "5° … range 5–7" |
| `GAIT_YIELD_ENGAGE_MS` / `_RELEASE_MS` | **10 / 400** | comment: "engage(30)+release … = 200 ms total, well inside a half-cycle" (actual 410 ms ≈ a whole half-cycle at 0.7 Hz — see A-09) |
| `WeightShiftConfig::torso_shift_deg` | **−20.0** | comment: "direction unverified. **0 = disabled**" — the term is live at SOURCE_GAIT during every shift (see A-11) |
| `WeightShiftConfig::lateral_shift_mm` | **−12.5** | comment: "Increased … −10 → −15" |
| `BalanceConfig::Kp` | **8.0** | comment: "Kp reduced from 20.0 → 10.0" |
| `GAIT_PHASE_RATE_HZ` | **0.7** | comment: "Reduced … 0.4 → 0.25" |
| `FilterConfig` sanity comment | inner 0.10/outer 0.20 g | comment says "0.075/0.150 by default" |
| `BALANCE_KP_MAX` rationale | ω_n ≈ 6.3 rad/s | ω₀(220 mm) = 6.68 rad/s |

- **Interpretation:** the operator tunes at runtime and persists by editing values
  before reflash, without updating comments/docs — exactly the anti-pattern
  CODING_STANDARD §13 lists ("Values duplicated in docs and code").
- **Consequence:** every "default-inert / bit-identical" claim in the docs is
  unreliable; A/B baselines described in CURRENT_FOCUS do not match the flashed
  firmware.
- **Proposed fix:** B7 sweeps the *doc* claims; the in-file tuning-note staleness
  should be fixed in the same commit (comment-only). Size **S–M** (breadth). The
  *behavioral* members of this list (A-09's yield, A-11's torso shift) need explicit
  human confirmation of intent.

### A-09 | gait_controller | HIGH | class 2/6 | Observation + Interpretation
**Post-contact ankle yield is live by default, and its 410 ms profile outlasts fast cadences.**

With `yieldEnabled=true` + `yieldDorsiflexDeg=2.0` shipped (A-08), every detected
touchdown arms a 2° dorsiflex release (`_startYield`, gait_controller.cpp:2126).
`yieldEngageMs+yieldReleaseMs = 410 ms`; at the shipped `phaseRateHz=0.7` a half-cycle
is 714 ms minus WAIT_SHIFT/STABILIZING — the release frequently spans into the next
half (by design it survives the swap), and `_startYield`'s early-return counts
`yieldSkipCount` when a new touchdown finds the previous release still active — i.e.
at higher cadence alternate steps silently get **no yield** (confound documented in
code, but the docs think the feature doesn't exist at all). Given the most recent
commit is literally `"Ankle yield"`, this is likely a deliberate in-progress
experiment — but it invalidates the docs' "control bit-identical" claims for the
rollover instrumentation A/B.
- **Consequence:** current walking behavior includes an undocumented active control
  term; any A/B run assuming "release off by default" is confounded.
- **Proposed fix:** human decision — either bless the default (update comments/docs)
  or restore `0.0`. Not auditable further from code. Size **S** either way.
- **Regression risk:** restoring 0 changes flashed walking behavior the operator may
  have tuned around.

### A-10 | webcomm parsers | MED | class 8 | Observation
**Parse-time clamp coverage is inconsistent; `sign` keys act as unbounded gain multipliers.**

Clamped correctly: `Kp/Kd/Kp_r/Kd_r` (vs `BALANCE_KP/KD_MAX`), everything in
`GAIT_TUNE`, `TOUCHDOWN_TUNE`, `COMPLIANCE_TUNE`, `FOOT_CONTACT_TUNE`,
`SWING_LEVELER_TUNE` (incl. envelope-order cross-check), `STEER_TUNE`,
`FOOT_IMPACT_TUNE` (via `_clampConfig`). **Unclamped:** `BALANCE_TUNE:`
`setpoint`, `deadband`, `d_lpf`, `tau`, `rate`, `max`, `sign`, `sp_r`, `max_r`,
`sign_r`; `TORSO_TUNE:` `setpoint`, `sign`, `max`, `sp_r`, `sign_r`, `max_r`,
`rate`, `damping`, `out_max`; `WEIGHT_SHIFT_TUNE:` `setpoint`, `ankle`, `torso`,
`ramp` (negative ramp is later floored at 10 ms in `WeightShift::update`).
Sharp edges: `sign=5` multiplies `u_raw` by 5 **after** the Kp ceiling was enforced —
a documented-ceiling bypass (`u` is still clamped by `max_correction_deg`, but `max`
itself is unclamped, so `sign=5,max=90` legitimately defeats `BALANCE_KP_MAX`'s
intent). `d_lpf>1` makes the derivative IIR divergent. Final backstops that keep all
of this bounded: joint anatomical limits (`JointModel::clampToLimits`) and the
±25 mm task-space clamps.
- **Consequence:** a fat-fingered tune can command aggressive-but-bounded behavior
  the gain ceilings were designed to prevent.
- **Proposed fix:** clamp the listed keys (sign → snap to ±1 like `latCaptureSign`;
  `max`/`out_max` ≤ 30°; `d_lpf`/`tau`/`rate` to sane ranges; WS `ankle` ≤ 25,
  `torso` ≤ 25, `setpoint` ±0.5 rad). Size **S**, single file. **Phase C candidate**
  only if judged CRITICAL — recommended as an early Phase-B-adjacent hardening since
  it is parse-time-only and default-inert.
- **Regression risk:** an operator currently relying on an out-of-range value would
  see it clamped — log lines make that visible.

### A-11 | weight_shift / arbitration | MED | class 2 | Observation
**Torso-roll channel has two writers and a live −20° gait-priority command the comment calls disabled.**

`WeightShift::update()` submits `IDX_TORSO_ROLL = progress × torso_shift_deg` at
`SOURCE_GAIT` whenever `|torso_shift_deg|>0.01 && |progress|>0.01`
(weight_shift.cpp:178-181) — with shipped `torso_shift_deg = −20`, torso roll swings
±20° through every weight shift. `TorsoStabilizer` submits the same channel at
`SOURCE_BALANCE`; arbitration is clean (balance wins when armed), but with torso
balance disabled by default, the −20° swing is the *only* writer and is live. The
field comment says "direction unverified. 0 = disabled". Full arbitration map:

| Channel(s) | SOURCE_UI (0) | SOURCE_GAIT (1) | SOURCE_BALANCE (2) |
|---|---|---|---|
| leg pitch/roll ×10 (ch 0-3,5,6,12-15) | sliders, RESETP | GaitController `_submitFootTargetIK` (every tick when gait/WS/bc active); WebComm `IK_*` (WiFi task) | — |
| hip yaw ch4/ch7 | sliders, RESETP | SteeringController (only when `enabled`; submits 0° parked value every tick incl. IDLE → sliders lose the channel while enabled) | — |
| `IDX_TORSO_ROLL` (8) | sliders, RESETP | **WeightShift** (torso_shift term) | **TorsoStabilizer** (roll) |
| `IDX_TORSO_PITCH` (9) | sliders, RESETP | — | **TorsoStabilizer** (pitch) |
| ch10 (unused), ch11 (torso rotation) | sliders, RESETP | — | — |

Same-priority collisions: gait vs `IK_*` test commands on leg channels (last-write
nondeterministic when gait active — UI warns about balance but not gait); gait vs
steering never collide (disjoint channels).
- **Consequence:** unverified-direction ±20° torso roll during every shift is the
  current flashed behavior; comment misleads the next tuner.
- **Proposed fix:** comment fix in B7 + human confirmation the −20 is intentional.
  Size **S**.

### A-12 | main.cpp remap / docs | MED | class 3/4 | Observation
**Foot-IMU axis remap: comment, code, and docs give three different stories.**

`_remapFootRawToRobotFrame` comment (main.cpp:52-57) derives
`(robot) = (−chip_y, −chip_x, −chip_z)`; the code applies
`x=−ay, y=+ax, z=+az` (a Z-rotation, not the comment's reflection). ARCHITECTURE §5
and CURRENT_FOCUS §1B say the remap is **identity**. The code is what flies; the
comment's own derivation and both docs are stale.
- **Consequence:** anyone re-mounting an IMU and following the comment's matrix will
  install a wrong frame with no error indication (exact hazard the comment warns
  about).
- **Proposed fix:** B7 — rewrite the comment from the code, fix both docs. Size **S**.

### A-13 | leg_ik docs | MED | class 4 | Observation
**`leg_ik.h` sign-convention header contradicts the hardware-corrected implementation.**

Header: "ankle_pitch: + = DORSIFLEXION", "Expected: ankle_roll = −hip_roll"
(leg_ik.h:29-31). Implementation: ankle pitch output **negated** because hardware
+ = plantar flexion (leg_ik.cpp:120-132), ankle roll **= +hip_roll** after the
2026-05-30 hardware verification (leg_ik.cpp:186-198). Several other comment sites
still state `ankle_roll = -hip_roll` (project_wide_defs.h stance-inset block,
gait_controller.cpp `_ankleRollDeg` header, GAIT_DESIGN). CLAUDE.md's own invariant —
"IK angle conventions + direction fields must stay consistent (sign errors =
catastrophic)" — is why this doc rot is dangerous: reverting code to match the stale
header inverts both ankles.
- **Proposed fix:** B7 comment sweep, single commit. Size **S**.

### A-14 | gait_controller | MED | class 6 | Observation + Interpretation
**`WSHIFT_ONLY` mode applies the full fore/aft stride formula.**

`update()` calls `_submitCycleStanceBoth(_halfPhase, bc)` (gait_controller.cpp:2150)
*before* the `WSHIFT_ONLY` branch (:2251), and that branch advances `_halfPhase` and
swaps swing roles. With shipped `stepLengthMm = 37`, the "CoM shift only — no foot
lift" verification mode also strides both feet ±18.5 mm fore/aft and role-swaps them.
(The analogous IDLE+manual-shift case was explicitly fixed — the "BUG FIX
(weight-shift step artefact)" block at :1939 — but only for `mode==IDLE`.)
- **Consequence:** WSHIFT_ONLY no longer isolates the lateral shift; any CoM/ankle
  verification run in this mode is confounded by sagittal stride.
- **Proposed fix:** route WSHIFT_ONLY through `_submitStanceBothLegs` (strideScale 0)
  like the IDLE case, or force `strideScale=0` for this mode. Size **S**, single
  module — but it *changes behavior of a diagnostic mode*, so it is a
  default-behavior change → **documented, human decision** (arguably restoring
  documented behavior).

### A-15 | webcomm / joint model | MED | class 1/2 | Observation
**`LOAD:` (pose load) and `MOVE_NAMED:` bypass MotionManager from the WiFi task.**

`loadPose()` → `setAbsoluteAngle(..., immediate=false)` and `applyNamedMovement()` →
`setJointAngle()` write `JointModel::_targetPulse` directly on the async task
(WebComm.cpp:1266-1292, servo_control.cpp:130-193), violating the CLAUDE.md invariant
"NEVER bypass MotionManager" and racing `JointModel::update()`/`flush()` writes on the
main task. `RESETP`'s no-MotionManager fallback (`setGaitOffset`) is the same class
but unreachable in practice (manager always wired).
- **Consequence:** pose loads ignore priority arbitration (can fight an armed
  controller at hardware level) + torn-target race (float store vs read — bounded).
- **Proposed fix:** route both through `submit(SOURCE_UI, ...)` per joint (pose files
  are absolute-degree, which is exactly SOURCE_UI's frame). Size **S**. **Phase C
  candidate** (local, no default change — behavior when a controller is active
  changes from "fight" to "lose cleanly", which is the documented intent).

### A-16 | webcomm | MED | class 8 | Observation
**Default slider path turns any unrecognized colon-bearing text into a servo command.**

`handleWebSocketMessage` final `else`: any message that isn't `CMD:`/`SAVE:` and
contains `:` parses as `<ch>:<angle>` with `toInt()/toFloat()` (WebComm.cpp:2732-2745)
— garbage like `"foo:bar"` becomes ch0 ← 0.0° absolute (→ −25° anatomical after
clamps). No prefix guard, no channel/angle range check before submit.
- **Proposed fix:** require the channel substring to be numeric (`isDigit` on first
  char) and `ch < NUM_JOINTS` and angle within `[0, 270]` before submitting. Size
  **S**. **Phase C candidate.**
- **Regression risk:** none for the real UI (always sends numeric).

### A-17 | state_estimator → consumers | LOW-MED | class 5 | Observation
**Two consumers use IMU fields without a `valid` gate.**

(a) IMU_ROLL lift-gate predicate `imuState.roll * rollSign >= wsThresholdRad`
(gait_controller.cpp:1865) evaluates on stale-held roll when `valid=false` (matters
only in non-default IMU_ROLL/telemetry). (b) Recovery **chain detection** at
COMPLETE (`_runRecoveryStep`, :2911-2919) compares `imuState.roll/pitch` without
checking `valid` — an IMU dropout at that tick chains (or skips chaining) on stale
tilt. All other consumers checked and gated (balance, torso, trim/capture helpers,
comLateral gate, steering rate injection, foot predicates, walking instrumentation).
- **Proposed fix:** gate chain detection on `imuState.valid` (fail toward lockout,
  not toward a blind second step). Size **S**, local. **Phase C candidate.**

### A-18 | timers | LOW | class 6 | Observation
**Two `millis()` deadline comparisons are not wrap-safe.**

`millis() < _rcvLockoutUntilMs` (gait_controller.cpp:2521) and
`now_ms < pv.refractory_until_ms` (foot_impact.cpp:130) use absolute comparison;
`_trimSuppressUntilMs` correctly uses the signed-difference idiom (:363-364).
Manifest only across the 49.7-day wrap → negligible for this platform, but the
codebase already has the correct idiom to copy.
- **Proposed fix:** signed-difference form. Size **S**.

### A-19 | webcomm / RTOS | MED | class 10-adjacent | Interpretation + Speculation
**5 KiB stack buffer in `broadcastWalkingInstr` on the Arduino loop task.**

`char buf[5120]` (WebComm.cpp:422) plus snprintf/AsyncWebSocket send-path frames on
the loop task (default 8 KiB stack on arduino-esp32 unless raised). `GAIT:` uses
2688 B in a sibling frame (not nested). Margin unverified.
- **Consequence:** stack overflow → corruption/panic when the WALKING packet is at
  max fill. Speculation until measured.
- **Proposed fix:** measure `uxTaskGetStackHighWaterMark(NULL)` on hardware; if thin,
  make the buffer `static` (WebComm is a singleton; broadcasts only run on the loop
  task). Size **S** — but "static" must be justified against the single-caller
  invariant. Validation step, not a blind fix.

### A-20 | lsm6dsv / webcomm | LOW-MED | class 1 | Interpretation
**`FOOT_IMU_PROBE` issues Wire1 transactions from the WiFi task concurrent with 400 Hz foot reads.**

`probe()`/`readWhoAmI()` run on the async task; the repeated-start pair
(`endTransmission(false)` + `requestFrom`) is two calls and may interleave with the
main task's burst reads depending on the core's TwoWire locking granularity
(Speculation on lock scope). Failure mode is benign: corrupted transaction → NACK/
short read → `valid=false` for one tick → consumers gate (verified fail-safe).
- **Proposed fix:** stage the probe request to the main task (B1 mechanism) or accept
  and document. Size **S**.

### A-21 | servo stack | MED | class 4 | Interpretation + Speculation
**`degToPulse` assumes a 270° span for all servos; 180° joints live in a scaled pseudo-frame.**

`SD_ANGLE_RANGE = 270` maps 0–270° → 500–2500 µs for every channel
(servo_driver.cpp:44-52). Hip yaw and ankle pitch are 180° servos
(`rangeDeg=180`, joint_config.cpp:54,63,71,87): if their full travel corresponds to
the same 500–2500 µs, one commanded "servo degree" ≈ 0.667 physical degrees on those
joints. `rangeDeg` is used **only** for speed conversion (joint_model.cpp:196-201 —
explicitly correct there). Neutral/limit calibration is empirical in the same
pseudo-frame, and ankle-pitch `transmissionGain=0.72` (vs 0.60 measured) may fold
part of the scale in — but hip yaw (steering!) has gain 1.0, so `yawStepDeg`
"degrees" are plausibly ~2/3 physical. The steering Stage-1.1 `realizationRatio`
budget (cmd vs measured gyro) will absorb/reveal this on hardware.
- **Consequence:** anatomical-degree commands on 180° joints may be systematically
  scaled ~×0.667; consistent within the system, wrong against the physical world.
- **Proposed fix:** bench measurement (command +20° hip yaw, measure body/servo horn
  angle), then either per-joint µs/deg in `degToPulse` (M, touches the single
  conversion seam) or fold into `transmissionGain`. **Hardware validation item —
  document only.**

### A-22 | robot_geometry / capture | LOW | class 9 | Observation
**B5 premise false; the stale artifacts are comments.**

`INV_OMEGA0 = 0.1498` ✔ for `H_COM_MM=220` (√(0.220/9.81)=0.14975 — 0.02 % off, i.e.
rounded, not stale). Stale text: `robot_geometry.h:49-60` ("Measured: ~25 cm",
"ω₀ = … ~6.26 rad/s at h = 250 mm") and `BALANCE_KP_MAX`'s "ω_n ≈ 6.3 rad/s"
rationale. No compile-time `constexpr sqrtf` exists in gnu++11; options are a
hand-derived comment formula (current pattern, fix the numbers) or a Newton-iteration
`constexpr`. Recommend comment fix only.
- **Proposed fix:** replaces B5 — comment-only commit; **no behavioral change, no
  hardware re-check required** (removes one of the handoff's two queued bench items).
  Size **S**.

### A-23 | rollover instrumentation | LOW | class 2 | Observation
**B3/B4 pre-approved fixes are already satisfied.**

B3: index agreement proven by `side.h:39-40` static_asserts + the explicit invariant
comment at gait_controller.cpp:2081-2083. Remaining Phase B action: optionally add a
`static_assert(sideIdx(Side::RIGHT) == 1 …)` adjacent to `_rolloverSide` for
anchoring (comment-only/compile-time; bit-identical). B4: neutral correction already
implemented (gait_controller.cpp:198,211). Phase B action: none beyond doc sync
(CURRENT_FOCUS §1J flag list is stale).

### A-24 | balance/gait vestigials | LOW | class 2 | Observation
**Dead-but-load-bearing-looking state left by the consolidation refactor.**

`BalanceConfig::ankle_roll_bias_l/r_deg` + `swing_leveler_roll_bias_*` + both setters
+ `BJI_*` enum + `_jointState[BJI_COUNT]` shaping array + `_shapeOutput()` are no
longer on any control path (ankle roll owned by gait at SOURCE_GAIT; `bias_l/r` still
broadcast in `BALANCE:` as echoes; `_shapeOutput` has no caller found).
`WeightShift::setSwingLegMask` machinery is declared unused by its own comment
(weight_shift.cpp:147-154) yet still called every tick from gait. Scope forbids
cleanup — documenting so nobody "re-wires" them believing they act.

### A-25 | webcomm | LOW | class 8 | Observation
**`toFloat()`/`toInt()` return 0 on non-numeric input in every parser.**

A typo like `Kp=1O` silently writes 0 (clamps make it legal). Uniform across all
parsers. Consequence bounded; visibility exists via the echo broadcasts.
Fix (optional): reject pairs whose value substring fails a numeric scan. Size **S**,
breadth M.

### A-26 | steering | LOW | class 2 | Observation
**When steering is enabled, hip-yaw channels are locked to 0° even in IDLE.**

`update()` submits the (slewed-to-zero) yaw at SOURCE_GAIT every tick while enabled,
regardless of `_in.active` (steering_controller.cpp:113-121) — UI sliders on ch4/ch7
are overridden. Deliberate parking behavior per comments; noted as an arbitration
fact (the "uncontested slot the UI can still own" comment applies only when
*disabled*).

### A-27 | gait telemetry | LOW | class 6 | Observation
**`waitShiftTimedOut` is not reset by `start(IDLE)`.**

Set at timeout (gait_controller.cpp:2321), cleared only on the next successful swing
commit (:2335). After a timeout→STOP→IDLE, the flag persists in `GaitState` (and the
`GAIT:` freeze snapshot) until the next run's first gate-open. UI treats it as an
edge, so practical impact is a stale badge. Fix: clear in `start()`. Size **S**.

### A-28 | docs / handoff | LOW | class 3 | Observation
**ARCHITECTURE §3 tick-order text vs `main.cpp` actual.**

Actual per-tick additions not in the doc: foot boot-neutral publish (rising-edge
CALIB_DONE), foot-contact snapshot build (F3 — A-07), `steeringController.setYawRates`
injection, `walkingInstr.update` **after** `flush()`, `TORSO:`/`WALKING:` broadcasts.
Doc step 9 ("submits the WeightShift ankle-roll bias at SOURCE_BALANCE") is false
post-consolidation (ankle roll is composed at SOURCE_GAIT in `_submitFootTargetIK`;
`_submitAnkleRollBias` no longer exists). No true read-before-produce hazard found:
the two cross-module 1-tick stalenesses (swing-leg mask, steering gait-input) are
deliberate and documented in code. B7 material.

### A-29 | mass/dims (B7 input) | — | class 3 | Observation
CLAUDE.md still says "~35–40 cm, ~1.4 kg" — handoff states current is ~1.5 kg
(~2.0 kg with upper body). No code impact (`H_COM_MM` is the only mass-adjacent
constant and is current). B7 doc item. SPI/SA0 references: remaining stale SPI text
is only in *history* blocks (lsm6dsv_driver.h) which are accurate as history;
CODING_STANDARD §14's foot-IMU row is already correct.

---

## 2. Summary table (severity × fix-size ranking)

| ID | Severity | Size | Class | One-liner | Phase B/C disposition |
|---|---|---|---|---|---|
| A-01 | HIGH | M | 1 | Full-struct config RMW across tasks (7 structs enumerated) | **B1** (enumeration complete above) |
| A-04 | HIGH | S | 1 | MotionManager slot torn-write can dispatch 0° absolute | **Phase C** (write-order hardening) + B1 |
| A-05 | HIGH | S | 4 | `pitch_setpoint_rad = 10.0` (10 rad) default | Document — **human decision** (default change) |
| A-09 | HIGH | S | 2/6 | Ankle yield live by default; docs say unimplemented | Document — **human decision** |
| A-06 | HIGH | M | 5 | Torso BMI160 silent-failure path (`valid` always true) | Document; needs library check + bench |
| A-02 | HIGH | M | 1 | WiFi task drives FSMs directly (start/stop/trigger/tare) | Fold into B1 staging design |
| A-07 | HIGH | S | 3/2 | F3 lift gate shipped default-on; 4 docs contradict | **B7** |
| A-08 | HIGH(trust) | S–M | 3 | Systemic default-vs-comment divergence (18 sites) | **B7** + human confirmation on behavioral rows |
| A-10 | MED | S | 8 | Unclamped tune keys; `sign` bypasses gain ceilings | Recommend with B-series (parse-time only) |
| A-11 | MED | S | 2 | WeightShift −20° torso roll at SOURCE_GAIT; comment says disabled | **B7** + human confirmation |
| A-14 | MED | S | 6 | WSHIFT_ONLY applies stride formula | Document — **human decision** |
| A-15 | MED | S | 1/2 | LOAD/MOVE_NAMED bypass MotionManager from WiFi task | **Phase C candidate** |
| A-16 | MED | S | 8 | Garbage colon-text becomes a ch0 servo command | **Phase C candidate** |
| A-12 | MED | S | 3/4 | Foot-IMU remap: comment ≠ code ≠ docs | **B7** |
| A-13 | MED | S | 4 | leg_ik.h sign conventions contradict hardware-corrected code | **B7** |
| A-19 | MED | S | — | 5 KiB stack buffer on loop task | Bench validation item |
| A-21 | MED | M | 4 | 270° pulse math on 180° servos (pseudo-frame) | Bench validation item |
| A-03 | MED | M | 1/6 | estop/clear sequences on WiFi task | Document (scope: e-stop read-only) |
| A-17 | LOW-MED | S | 5 | Recovery chain-detect ignores `valid` | **Phase C candidate** |
| A-20 | LOW-MED | S | 1 | FOOT_IMU_PROBE cross-task Wire1 traffic | Document / fold into B1 |
| A-22 | LOW | S | 9 | B5 premise false; robot_geometry comments stale | **Replaces B5** (comment-only) |
| A-23 | LOW | S | 2 | B3 proven, B4 already implemented | Closes B3/B4 |
| A-18 | LOW | S | 6 | Two non-wrap-safe millis() deadlines | Optional S fix |
| A-25 | LOW | S | 8 | toFloat() silent-zero on typos | Optional |
| A-26 | LOW | — | 2 | Steering enabled locks yaw sliders in IDLE | Note only |
| A-27 | LOW | S | 6 | `waitShiftTimedOut` sticky across stop | Optional S fix |
| A-24 | LOW | — | 2 | Vestigial balance ankle-roll fields/setters | Note only (scope: no cleanup) |
| A-28 | LOW | S | 3 | ARCHITECTURE §3 ordering text stale | **B7** |
| A-29 | LOW | S | 3 | CLAUDE.md mass/dims stale | **B7** |

**Counts:** 0 CRITICAL · 8 HIGH · 9 MED · 12 LOW-or-informational.
No finding met the Phase C bar as originally defined (CRITICAL + S + fully local);
the four "Phase C candidate" rows (A-04, A-15, A-16, A-17) are HIGH/MED + S + local
and are recommended for explicit human promotion.

## 3. Impact on the pre-approved Phase B list

| Item | Status after audit |
|---|---|
| B1 | Proceed as specified. Input enumeration = A-01 table; extend design to cover A-02 command staging; keep Steering/FootContact patterns as-is. |
| B2 | Proceed. Literal sites: `main.cpp:332` (`2500`), `WebComm.cpp:481,1130` (`/400.0`), comment sites listing 400 Hz assumptions. No hidden hardcoded `dt` found in control code (unit tests use `0.0025f` legitimately). |
| B3 | **Already proven** (A-23). Optional: add anchoring static_assert + comment. |
| B4 | **Already implemented** (A-23). No code change; sync CURRENT_FOCUS §1J. |
| B5 | **Premise false** (A-22). Re-scope to comment-only fix in `robot_geometry.h` (+ `BALANCE_KP_MAX` rationale number). Drop the "capture re-check" hardware item. |
| B6 | Simplifies: math already frequency-parametric (headline table). Work = introduce `SERVO_FRAME_HZ` (default 50), route `setPWMFreq`, document ladder + the ≤ ~333 Hz ceiling from `SD_USMAX`. Bit-identical at 50. |
| B7 | Larger than scoped: A-07/08/11/12/13/28/29 all feed it. Recommend splitting doc-sync into: (i) safety-relevant sign/remap comments (A-12, A-13), (ii) architecture/F3 reality (A-07, A-28), (iii) defaults/tuning notes (A-08), (iv) mass/dims (A-29). |

## 4. Verified-clean areas (for the record)

- **Null guards (class 7):** every setter-injected pointer dereference found is
  guarded (`_mm`, `_ws`, `_bal`, `_impact`, `_steering`, `_footEst[]`, `_servo`,
  all WebComm pointers, leveler estimators). No violations.
- **E-stop invariant:** `oe_estop()` callers are exactly BalanceController fall
  detect + `CMD:ESTOP` (grep-verified).
- **Hardware write invariant:** `_pwm.writeMicroseconds` appears only in
  `ServoDriver::setServoPulse`.
- **Foot-IMU failure during gait:** fail-safe by construction — invalid snapshot →
  STANCE_FLAT gate closed → `waitShiftMaxMs` abort to IDLE.
- **`dt` plumbing:** measured `micros()` delta end-to-end; estimator caps dt at 50 ms
  and rejects ≤0; JointModel uses measured ms dt; no controller hardcodes 2.5 ms.
- **Stride/role-swap continuity, touchdown latch lifecycle, H-restore deficit
  ownership, heel-strike/yield timer independence:** internally consistent; the
  role-swap edge cases enumerated in the handoff (latch across `_advanceHalf`,
  `oe_clear` during RECOVERY_STEP, WAIT_SHIFT timeout vs dwell, chain-detect during
  estop) all resolve correctly in code — with the two exceptions recorded as A-17
  (chain-detect `valid`) and A-27 (sticky timeout flag).
- **FootContactSnapshot `leftFlat` vs `flat[]`:** reference-member aliases — the
  gate and telemetry read the same storage (foot_contact.h:119-144).

*End of Phase A record. No source files were modified in Phase A.*

---

# PHASE B/C EXECUTION RECORD — 2026-07-07

**Branch:** `audit/v1-closure` · **Executor:** Claude Code (Fable 5), fresh session per handoff.
`pio run -e esp32s3` passed after every commit. One concern per commit; every
commit message carries a Safety-Audit line and a behavior prediction.

## Commits landed (in order)

| Commit | Item | Summary |
|---|---|---|
| 4887078 | **B1** (A-01) | `StagedConfig<T>` seqlock double-buffer (`include/staged_config.h`); all six racy structs (Balance/Torso/Gait/WeightShift/FootImpact/SwingFootLeveler) staged by parsers, applied by `webComm.applyStagedTuning()` at the top of the 400 Hz gate. `FootImpactDetector::clampConfig` made public static for parse-time echo truth. |
| b083a24 | **B1.2** (A-02) | SPSC command ring (16-deep) drained after config apply: gait start/stop/step/next/recovery-tests, WS triggers, RESETP + STANCE as composites. ESTOP/CLEAR_ESTOP deliberately NOT staged; TORSO_TARE + ServoControl calls documented residuals. |
| 5078156 | **B2** | `IMU_LOOP_HZ` / `IMU_LOOP_INTERVAL_US` in project_wide_defs.h; main.cpp gate + WebComm calib-dt derive from it. Bit-identical at 400. |
| 32472d0 | **B3** (A-23) | Anchoring `static_assert` at the `_rolloverSide` write site (invariant already proven — compile-time only). |
| f0a3b9a | **B5** (A-22) | Re-scoped comment-only: robot_geometry.h H_COM block (25 cm/6.26 → 22 cm/6.68), BALANCE_KP_MAX rationale, capture-block arithmetic. Hardware capture re-check DROPPED (no constant changed). |
| 8c25465 | **B6** | `SERVO_FRAME_HZ` (default 50, bit-identical) + validation ladder + static_assert ceiling (~333 Hz from SD_USMAX). Default untouched per DO-NOT rule. |
| a56cad6 | **B7a** (A-12/A-13) | Sign/remap comment truth: foot-IMU remap comment rewritten from code (Z-rotation, not reflection); leg_ik.h conventions now describe the hardware-corrected solver (+hip_roll ankle roll, plantar-flexion-positive ankle pitch); sibling sites fixed. |
| 959bbc1 | **B7b** (A-08) | 20+ stale default/tuning-note comment sites synced to shipped values. A-05/A-09/A-11 flagged in place as pending human decisions (values untouched). |
| c049a37 | **C1** (A-04) | `MotionManager::submit()` writes fields first, `occupied` last — torn cross-task window now misses a tick instead of dispatching 0° absolute. |
| acd2d22 | **C2** (A-15) | LOAD:/MOVE_NAMED routed through MotionManager (SOURCE_UI / SOURCE_GAIT) via new read-only `readPoseAngles` + shared `namedMovementToJointAngle`. Controllers now win by priority instead of hardware-level fights; estop drain now covers these paths. |
| 95fe076 | **C3** (A-16) | Fallback slider parser validates channel (digit-leading, < NUM_JOINTS) and angle ([0, SD_ANGLE_RANGE]); garbage text no longer becomes a ch0 0° command. |
| 799dc79 | **C4** (A-17) | Recovery chain-detect gated on `imuState.valid`; IMU dropout at COMPLETE fails toward lockout, not a blind second step. |

**B4:** no code change needed (already implemented — A-23); doc sync done in B7c.
**Phase C gate note:** zero findings were CRITICAL, so the strict gate was empty;
A-04/A-15/A-16/A-17 were implemented under the operator's explicit "execute
Phase B/C" instruction (each its own revertible commit).

## B7c doc sync (disk-only — `*.md` is gitignored, cannot be committed)

ARCHITECTURE.md (§1/§2 module truths, §3 tick order incl. step 0 staging +
7a/7b/12a/14a, §4 SOURCE_BALANCE = torso-only + torso-roll dual writer, §5 F3
reality + remap, §7.1–7.3 post-consolidation truths) · CURRENT_FOCUS.md (§1B
F3 shipped + zero-neutral caveat, §1J flags closed + yield ships-active, open
issues RMW→fixed + IMU_LOOP_HZ→done, validation-gate table incl. three human
decisions + B6 ladder) · GAIT_DESIGN.md (lift-gate pseudocode → real
LiftGateMode set, §3.9 yield ships-active, stabilizeMs/strideComplete numbers)
· CLAUDE.md + AGENTS.md (mass/dims 1.5/2.0 kg, staging in the tick paragraph,
module rows, failure-mode row, workstream statuses).

## Human decisions queued (values deliberately untouched)

1. **A-05** — `BalanceConfig::pitch_setpoint_rad = 10.0` rad: change to 0.0 + parse clamp?
2. **A-09** — post-contact yield ships ACTIVE (enabled + 2.0°): bless or restore 0?
3. **A-11** — WeightShift `torso_shift_deg = −20` live every shift, direction unverified: confirm or zero?
4. **A-14** — WSHIFT_ONLY applies the full stride formula: restore documented isolation behavior?

## Hardware/bench items queued

- **B6 frame-rate ladder** (50 → 250/300 Hz) per the `SERVO_FRAME_HZ` block. (The
  second handoff bench item — B5 capture re-check — was dropped: premise false.)
- A-19 stack high-water measurement (`broadcastWalkingInstr` 5 KiB buffer).
- A-21 bench measurement of 180°-servo degree scale (hip yaw / ankle pitch pseudo-frame).
- A-06 torso BMI160 silent-failure detection (library check + plausibility gate; size M).

*End of AUDIT_FINDINGS.md — Phases A, B, and C complete.*
