# STEERING_PLAN.md

> Layer 3.5 steering subsystem for the bipedal humanoid. Authoritative plan for
> adding turning to a robot whose straight-line walking is stable but still
> support-transfer-sensitive. Code is the source of truth; this doc is intent.
>
> LAST CHANGED: 2026-06-12 | Hinz | Stage 1.2 — sweep windowing in yawLawDeg
> (`sweepStartFrac`/`sweepEndFrac`, defaults 0/1 = bit-identical legacy no-op;
> relocates the yaw into the airborne phase — see §4.1) + authority ceiling raised
> (`STEERING_YAW_STEP_MAX_DEG = 30`, `maxYawDeg` 15→16 to avoid endpoint flat-top).
> Cold `STEER_TUNE:sweepStart=/sweepEnd=` (pair-clamped). Telemetry fields unchanged.
> | 2026-06-11 | Hinz | Stage 1.1 — turn sign HARDWARE-VERIFIED and
> baked (STEERING_TURN_SIGN = -1; runtime sign control removed); per-step yaw
> BUDGET characterization telemetry added (IMUState::yawRate surfaced from the
> estimator; SteeringYawBudget integrates commanded vs measured body/stance/swing
> yaw per footfall, pre/post-touchdown split, realization ratio). Instrumentation
> only — law, submissions, gait timing, ownership unchanged.
> | 2026-06-09 | Hinz | Stage 1 IMPLEMENTED — `SteeringController`
> (h/cpp/test) + `// STEERING` defs + GaitController push + WebComm CMD:STEER /
> CMD:STEER_TUNE split + disconnect/e-stop turnCmd=0 safing + UI joystick.
> API resolution: config writes use targeted atomic setters (WebComm runs on the
> AsyncTCP task, update() on the 400 Hz task — see §5 / Failure Modes in CLAUDE.md).

---

## 1. Goal

Add gentle, open-loop, **hip-yaw-based** turning during `GaitMode::RUNNING`,
shipped **inert** (master enable off -> zero submissions -> bit-identical to
current firmware), with **zero regression** to straight-line walking. Turn-in-place,
authority characterization, and yaw-rate closed-loop are later stages.

Priorities, in order: preserve current walking performance -> learning value ->
risk reduction -> turning performance. Turning performance is explicitly last.

---

## 2. Why hip-yaw, not differential stride (decision record)

Both hip-yaw joints (`ch4` right `dir+1`, `ch7` left `dir-1`, +/-50 deg, ~667 deg/s) are
**calibrated and addressable** (`channelFor(side, LegJoint::HIP_YAW)`) but
**undriven by any controller** -- `_submitFootTargetIK` submits only the five
sagittal/frontal joints. The yaw channels are an uncontested `SOURCE_GAIT` slot.

| Axis | Differential stride | **Hip-yaw (chosen)** |
|------|---------------------|----------------------|
| Mechanism of heading change | foot scrub (no joint absorbs rotation) | hip-yaw joint absorbs body->foot rotation (no scrub) |
| Touches `stepLengthMm` (current bottleneck) | **yes** | no |
| Blast radius | capture caps + warped stride + touchdown latch | one new module + 2 joints |
| Debuggability | entangled with stride dynamics | isolated SISO: cmd -> measured yaw |
| Turn-in-place | impossible | natural (`stepLength=0` + yaw) |

Differential stride is **deferred** to a possible Stage 3 authority booster, layered
on only after hip-yaw turning is trusted. Hybrid is the eventual destination, never
the starting point (it confounds two mechanisms -> violates one-variable discipline).

---

## 3. Steering model (user-facing)

* **Command:** `turnCmd in [-1, +1]`, default `0`. `0` = straight; sign = direction
(mapped to physical L/R by `turnSign`, confirmed once on hardware); magnitude =
authority. **State-based** (persistent setpoint, "keep turning until commanded
otherwise"), not event-based.
* **Internal unit:** deg-of-body-yaw per footfall = `turnCmd * yawStepDeg`. This is
what is measured on the floor and is the clean upgrade seam to a future
heading-rate command (`rate / measured cadence = yaw/step`, Stage 2+).
* **Radius command is rejected** as primary: degenerate at turn-in-place, re-couples
to `stepLengthMm`.

**Runtime control vs configuration -- protocol split.** `turnCmd` is *runtime
control* streamed many times per second by the operator; `enabled / yawStepDeg /
turnSign / turnSlewPerSec` are *configuration* set rarely. They travel on separate
wire commands so a streaming joystick never re-transmits tuning:

| Wire command | Carries | Cadence | Lands in |
|--------------|---------|---------|----------|
| `CMD:STEER:<turn>` | `turnCmd` only | hot (<=20 Hz, on-change) | `SteeringConfig.turnCmd` setpoint |
| `CMD:STEER_TUNE:enable=,yawStep=,turnSign=,slew=` | config | cold (operator action) | rest of `SteeringConfig` |

This is a **transport split only** -- both write the *same* `SteeringConfig`; there
is no second owner of steering state and the `SteeringController` is unchanged.
`turnCmd` is clamped to `[-1,+1]` in the `CMD:STEER` parser (the `setTurnCmd` setter).

**IMPLEMENTED:** both paths write through targeted atomic setters
(`setTurnCmd` / `setEnabled` / `setYawStepDeg` / `setTurnSign` / `setTurnSlewPerSec`),
NOT a whole-struct read-modify-write, because WebComm runs on the AsyncTCP task and
`update()` on the 400 Hz control task (the BalanceConfig/TorsoConfig torn-read race).

---

## 4. Control law (verified kinematics)

Frames: world `W` (z-up, CCW+); body `phi`; planted-foot heading `psi` (friction-fixed).
Anatomical `+q` = outward on both legs (left mirrored by `direction=-1`). Joint
relations:

```
Right planted:  psi = phi_R + q_R   =>   dpsi = +dq_R
Left  planted:  psi = phi_L - q_L   =>   dpsi = -dq_L
```

Same `+q` rotates the body opposite ways depending on the stance leg -- the law must
flip sign each half-cycle.

**Per-tick law (identical anatomical command to BOTH legs):**

```
s        = smoothstep(halfPhase) = h*h*(3 - 2h)       # 0->1 over the swing
halfSign = (swingSide == LEFT) ? +1 : -1              # flips body-effect with stance leg
q_anat   = turnEff * yawStepDeg * turnSign * halfSign * (s - 0.5)
q_anat   = clamp(q_anat, +/-maxYawDeg)
submit SOURCE_GAIT, channelFor(LEFT,  HIP_YAW), q_anat
submit SOURCE_GAIT, channelFor(RIGHT, HIP_YAW), q_anat   # direction field handles the mirror
```

(Implemented as the static `SteeringController::yawLawDeg()`; `update()` wraps it with
the enabled gate, e-stop park, slew, submit, and telemetry.)

**Properties (proven):** accumulates `+yawStepDeg*turnCmd` body-yaw per footfall,
monotonic and periodic; zero yaw velocity at touchdown (no rotational plant-scrub);
transient toe-out bounded by `+/-yawStepDeg/2` per foot at double support. **Dropping
`halfSign` yields pure heading oscillation with zero net turn** -- verify the flip
first.

**Persistence / slew (the command-handling core):** `turnCmd` is a setpoint; an
internal `_turnEff` slews toward `(active ? turnCmd : 0)` at `turnSlewPerSec` (the
static `slewToward()`). The law uses `_turnEff`. Because `_turnEff` is continuous and
`halfSign*(s-0.5)` is C1-continuous across half boundaries (`+0.5` both sides, zero
slope both sides), `q_anat` is jerk-free **regardless of when the operator moves the
slider**. Release does not snap back (accumulated heading lives in foot/body headings,
not joint angles); the joints slew to neutral over ~1 step and straight walking
resumes from the new heading.

`active = (mode == RUNNING)`. When `enabled` but not active (IDLE / RECOVERY /
STOPPING) or estopped, `_turnEff` slews to 0 and the law parks the yaw joints at
neutral. When `enabled == false`, the module **submits nothing** (UI keeps the yaw
channels; bit-identical ship).

**Equal-command is a Stage-1 simplification, not the optimum.** A decoupled
stance-rotation / swing-aim law nulls the transient splay; deferred to Stage 3.

### 4.1 Sweep windowing (Stage 1.2)

The smoothstep `s` is run over a configurable phase window `[sweepStartFrac,
sweepEndFrac]` and HELD at its 0/1 endpoints outside it:

```
u = clamp((halfPhase - wStart) / (wEnd - wStart), 0, 1)
s = u*u*(3 - 2u)
```

*Why:* yaw commanded while both feet are grounded does not turn the body — with
two friction-fixed contacts the equal anatomical command resolves into symmetric
toe-out/toe-in **foot splay** (zero net heading) and scrubs the soles. Heading
only changes when one foot is free to rotate about the other. The window relocates
the entire sweep into the interval where the swing foot is reliably **airborne**,
so the authority is spent on rotation instead of scrub. (Equivalently: the budget's
`stanceFootStepDeg` should fall and `realizationRatio` rise as the window is pulled
inside the single-support phase.)

*No-op at defaults:* `wStart=0, wEnd=1` makes `u = halfPhase`, reducing the law
**exactly** to the legacy smoothstep — the shipped default is bit-identical. The
hold regions pin `q` at the same `±A/2` endpoints the legacy law reaches at
`halfPhase` 0/1, so role-swap continuity (`+0.5` both sides), zero end-slope, and
per-footfall `Δq = yawStepDeg·turnEff` are preserved for **any** window.

*Peak-rate caution:* narrowing the window concentrates the same `Δq` into less
time — `qdotPeak = 1.5·yawStepDeg / ((wEnd−wStart)·T_half)`, `T_half =
1/(2·phaseRate)`. Do not combine minimum width, maximum `yawStepDeg`, and high
`phaseRate` in a single trial. Suggested first hardware window: start `0.15..0.30`,
end `0.70..0.85`. Window tuning is **cold** `STEER_TUNE` only (`sweepStart=`,
`sweepEnd=`), jointly clamped (`start∈[0,0.8]`, `end∈[start+0.2,1]`); it is never
coupled to `touchdownGatePhase` or any gait config.

*Authority ceiling (Stage 1.2):* the `yawStep=` parse cap is raised to
`STEERING_YAW_STEP_MAX_DEG = 30°` (was conflated with the 15° output clamp). The
instantaneous clamp `maxYawDeg` was raised 15→16 (≥ ceiling/2) so the `±A/2`
hold endpoints are not flat-topped at full authority; behavior-neutral (no prior
tuned setting reached the old clamp). Shipped `yawStepDeg` default unchanged (2.5).

---

## 5. Architecture & ownership

```
UI joystick --CMD:STEER------> WebComm --setTurnCmd()-----> SteeringConfig
UI sliders  --CMD:STEER_TUNE-> WebComm --setEnabled()/...-> SteeringConfig
GaitController.update() --setGaitInput(active, swingSide, halfPhase)--> SteeringController
main.cpp tick:  ... gait.update() -> steering.update() -> motionManager.flush()
                                        +- submit SOURCE_GAIT ch4/ch7 (uncontested)
```

* **Owns** the yaw law, slew state, config, telemetry, and the two yaw submissions:
`SteeringController` (new L3.5 module; direct analogue of `WeightShift`).
* **Unaware of steering:** `LegIK`, `gait_types.h`/`FootTarget`, `WeightShift`,
`BalanceController`. No task-space change, no IK change.
* **Modified minimally:** `GaitController` (one optional pointer + one push call,
same pattern as `_swingLeveler`/`_impact`); `WebComm` (two parse paths + one
broadcast + disconnect/e-stop safing); `main.cpp` (construct/wire/one update call).
`MotionManager`, `joint_config`, `side.*` **unchanged**.

### 5.1 Operator interface (Stage-1 virtual joystick)

A virtual joystick is added to the existing web UI (inside the gait panel). It is
**operator input only** -- it owns no steering behavior, it only generates `turnCmd`
and streams it on the hot `CMD:STEER` path. The `SteeringController` does not know the
joystick exists.

* **X axis -> `turnCmd in [-1,+1]`.** Y axis ignored at Stage 1 (reserved for a future
gait-speed map; do not wire it now).
* **Center-release -> `turnCmd = 0`.** Spring-to-center on pointer/touch release.
* **Streaming:** send on-change while dragging, rate-limited to <=20 Hz, with a final
`0` on release. The controller's `_turnEff` slew makes exact cadence non-critical,
but the cap keeps the socket quiet.
* **Connection loss -> `turnCmd = 0`** server-side: on WebSocket disconnect (and on
`CMD:ESTOP`), WebComm forces `SteeringConfig.turnCmd = 0` so a dropped client cannot
leave a turn latched. The client also best-effort sends `CMD:STEER:0` on `pagehide`.
* **Coexistence:** lives alongside existing gait controls; the bench config
(`STEER_TUNE`) widgets remain for tuning and characterization.
* **Telemetry:** the `STEER:` packet (`turnCmd`, `turnEff`, `yawCmdDeg`, `active`, plus
config echoes) is displayed near the joystick so the operator sees commanded vs
slewed-effective turn.

---

## 6. Interaction summary

| Subsystem | Direct effect | Indirect risk | Stage-1 handling |
|-----------|---------------|---------------|------------------|
| WeightShift | none (no gate/FSM/timing touched) | yaw reaction torque perturbs lateral roll (no arm counter-swing) | small `yawStepDeg`; smoothstep bounds yaw accel; fall-rate parity gate |
| LegIK / FootTarget | none (yaw not in IK) | ~1-2 mm un-modeled touchdown shift at Stage-1 authority (grows linearly) | negligible now; fold yaw into IK only if >~10-15 deg later |
| Touchdown | none (vertical traj unchanged) | stance scrub if friction < yaw torque | measure cmd-yaw vs body-yaw ratio |
| Capture / compliance / h-restore / foot-flat | none (different axis; sole not tilted) | none | - |

---

## 7. Risks & guards

| Risk | Guard |
|------|-------|
| Wrong turn direction | `turnSign` runtime flip; verify at `yawStepDeg~1 deg` first |
| Missing/incorrect `halfSign` -> oscillation, no net turn | unit test asserts net accumulation + boundary continuity |
| Mid-step command jump -> yaw jerk on stance leg | slew-limited `_turnEff` (C1 output) |
| Yaw reaction torque disturbs lateral transfer | tiny `yawStepDeg`; fall-rate parity gate vs straight baseline |
| Stance-foot scrub (friction < torque) | authority-ratio metric; reduce `yawStepDeg` |
| L/R `txGain` asymmetry -> left != right turn | Stage-2 characterization item, not a blocker |
| Residual yaw at stop / recovery | `active=false -> slew to neutral`; recovery parks yaw at 0 |
| Dropped client leaves a turn latched | server forces `turnCmd=0` on disconnect / e-stop |

---

## 8. Staged roadmap

**Stage 0 / 0.5 -- DONE.** Straight-line walking stable; hip-yaw sign + range
verified on hardware (increasing anatomical q -> outward both legs).

**Stage 1 -- open-loop hip-yaw turning -- IMPLEMENTED (this plan), ships INERT.**
Slew-limited state-based authority command; equal-anatomical law; `RUNNING` only;
`GAIT_STEER_ENABLED_DEFAULT=false`. Operator joystick + `CMD:STEER` / `CMD:STEER_TUNE`
split + disconnect/e-stop safing + unit tests (accumulation, boundary continuity,
slew, double-support hold, clamp, disabled-no-op) landed.
*Exit (hardware, pending):* repeatable arc both directions, no scuff-stall, fall-rate /
`lastReachPct` / `flatFootGuardCount` match the `turnCmd=0` baseline (see §9).

**Stage 1.1 -- sign cleanup + characterization telemetry -- IMPLEMENTED.**
Instrumentation only (law/submissions/timing/ownership unchanged). (a) Turn sign
hardware-verified and baked: `STEERING_TURN_SIGN = -1`, runtime sign control
removed (UI button, `STEER_TUNE:turnSign`, `setTurnSign()` all gone; the field +
`yawLawDeg()` param stay). (b) Per-step yaw BUDGET: `IMUState::yawRate` surfaced
from the estimator (debiased, pre-deadband gz; all three estimators); `setYawRates()`
injects torso/L/R; `SteeringYawBudget` integrates per footfall — `cmd` (DERIVED from
summed submitted joint deltas, model-free) vs `body`/`stance`/`swing` (MEASURED
gyro-Z integrals), pre/post-touchdown split, `realizationRatio = body/cmd`
(guarded), 6-step rolling means; latched on the swing-side flip. `STEER:` packet +
UI panel extended; `touchdownLocked` added to the gait push. Unit tests: cmd
consistency, mid-half slew exactness, role-swap (no spurious delta, stable foot
attribution), post-touchdown partition, disabled-zero.
*Exit (hardware, pending):* §9 gates 1-4 below — drift baseline, gyro-sign confirm,
and the realization-ratio reproduction of the floor observations across
phaseRate / stepHeight / stepLength.

**Stage 2 -- characterization + turn-in-place + rate command.** Add gyro-Z yaw-rate
telemetry; build the `turnCmd -> measured yaw/step -> radius` map at fixed cadence;
add turn-in-place (`stepLength=0` + yaw); add the heading-rate wrapper
(`rate / meanCadenceHz`). Open-loop or yaw-rate-closed only -- **no absolute heading**
(BMI160 is 6-axis, gyro-Z integrates with drift).
*Exit:* documented command->radius map; stable sustained arc; clean turn-in-place.

**Stage 3 -- refinements (optional, may never be needed).** Decoupled
stance-rotation / swing-aim law (null transient splay); `torso_rotation` (ch11)
counter-rotation to cancel yaw reaction torque; differential-stride authority
booster; fold yaw into task-space IK if precision at high authority demands it.

---

## 9. Validation gates (Stage 1)

1. **Build/inert:** `enabled=false` -> ch4/ch7 never submitted; firmware
bit-identical. Unit tests pass (accumulation, boundary continuity, slew,
double-support hold, clamp, disabled-no-op). [`runSteeringControllerTests()`]
2. **Bench (no walking):** enable while IDLE; nudge `turnCmd`; confirm both feet toe
per the law at a frozen phase; no joint clipping; slew visibly ramps.
3. **Direction:** `yawStepDeg~1 deg`, `turnCmd=+1`, walk -> body turns intended way? If
reversed, flip `turnSign`. If legs cross / sole scrubs in single support, the
`halfSign` mapping is wrong (not `turnSign`) -- stop.
4. **Regression:** `turnCmd=0` with `enabled=true`, sustained `RUNNING` -> straight
walking, fall rate, `lastReachPct`, `flatFootGuardCount` match disabled baseline.
5. **Authority:** ramp `yawStepDeg` 1->2->3 deg (1 deg steps); measure body-yaw/footfall.
Exit on repeatable bidirectional arc, no scuff-stall, no balance regression.
6. **Operator control:** joystick X drives turnCmd live; center-release returns to
straight; pulling the client (close tab / kill WiFi) forces turnCmd=0 and the
robot straightens within ~1 step (no latched turn). Verify turnCmd, turnEff, and
yawCmdDeg all converge back toward zero after disconnect/release. `STEER:`
telemetry tracks the joystick and the slew-limited decay.

## 10. Implementation boundaries

DO: new `SteeringController` (h/cpp/test), `// STEERING` block in
`project_wide_defs.h`, one pointer+push in `GaitController`, two parse paths +
broadcast in `WebComm` (`CMD:STEER` hot / `CMD:STEER_TUNE` cold) + connection-loss
`turnCmd=0`, a virtual joystick in the web UI (X->turnCmd, center-release->0),
construct/wire/update in `main.cpp`. Submit identical anatomical yaw to both legs at
`SOURCE_GAIT`. Default everything off / zero.

DO NOT: write `FootTarget`, change `LegIK`, apply per-leg opposite yaw signs, rotate
yaw during double support except via the `s=0` hold, change `MotionManager` /
`joint_config` / `side.*`, or steer outside `RUNNING`.
