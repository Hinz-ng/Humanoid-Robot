# Humanoid Robot

A 40 cm walking robot I designed, 3D-printed and programmed from scratch. It balances, shifts its
weight, steps, and catches itself when pushed, all from its own sensors and a single ESP32-S3.

<!-- Photo or short GIF of the robot standing / stepping / catching a push goes here. -->

## The idea

Most small hobby humanoids play back motions recorded in advance. That works until the floor
tilts or something bumps the robot. This one is built the other way around, closer to how
research bipeds work: it constantly measures how it is tipping, decides where its feet need to
go, and works out the joint angles to get them there.

| | |
|---|---|
| Size | ~40 cm, ~1.5 kg, 3D-printed frame |
| Brain | ESP32-S3 |
| Joints | 15 servos: 6 per leg (hip yaw/roll/pitch, knee, ankle pitch/roll) and 3 in the torso |
| Sensing | IMU in the torso, plus one IMU in each foot |
| Control loop | 400 Hz |
| Interface | the robot hosts its own WiFi and a web control panel, so I can tune it from a phone while it stands |

## How it works

- **Feeling its balance.** Three IMUs, filtered together, tell the robot how its torso and each
  foot are tilting and how fast its body is moving.
- **Staying upright.** A balance controller nudges where the feet are placed to keep the body
  over them, while a separate controller holds the torso level.
- **Walking.** A step cycle shifts weight onto one leg, swings the other, and puts it down. The
  foot IMUs act as a safety check: a foot only lifts once the other one is flat on the ground.
- **Catching itself.** When it gets pushed, it works out which way it is falling and takes a
  recovery step in that direction, then another if one wasn't enough.
- **Turning.** Turning comes from rotating the hips during the step.
- **Moving the joints.** Leg inverse kinematics turns foot positions into joint angles. Every
  module asks for joint commands with a priority, and one arbiter decides what reaches the
  servos. A hardware kill switch cuts power to all servos at once.

## Progress

- ✅ Standing balance and weight shifting
- ✅ Stepping in place
- ✅ Sideways push recovery
- ✅ Foot sensors gating each step
- 🔄 Forward walking and smarter foot placement (capture point)
- 🔄 Forward and backward push recovery
- 🔄 Turning while walking
- ⬜ Upper body

## Documentation

| Doc | What's in it |
|---|---|
| [Architecture](ARCHITECTURE.md) | how the code is layered and what runs in each control tick |
| [Gait design](GAIT_DESIGN.md) | the step cycle, foot placement and push recovery |
| [Steering plan](STEERING_PLAN.md) | how turning works and why it uses hip yaw |
| [Current focus](CURRENT_FOCUS.md) | what I'm working on now and what's waiting on testing |
| [Coding standard](CODING_STANDARD.md) | conventions for the codebase |
| [Audit findings](docs/AUDIT_FINDINGS.md) | a full code review and the fixes that came out of it |

## Building

[PlatformIO](https://platformio.org/) project, Arduino framework.

```bash
pio run -t upload      # firmware
pio run -t uploadfs    # web control panel (data/ -> LittleFS)
```

Join the `HumanoidRobot` WiFi network and open the IP address printed on the serial monitor at boot.

## Repo layout

```
src/, include/   firmware, one module per job (sensors, balance, gait, IK, servos, web)
data/            web control panel served by the robot
lib/             BMI160 IMU driver
```
