# Metal-Arm × LeRobot Integration — Testing Guide

Validate the `metal_follower` / `metal_leader` integration in stages, from "no
hardware" to "full end-to-end". Do them in order: each stage builds on the
previous one, with risk increasing as you go.

- Code: `src/lerobot/motors/metal/`, `src/lerobot/robots/metal_follower/`, `src/lerobot/teleoperators/metal_leader/`
- Design doc: `~/makermods/docs/superpowers/specs/2026-06-24-metal-arm-lerobot-integration-design.md`
- Environment: conda **`MakerMods-lerobot`** (Python 3.12)

```bash
export PY=/home/ethan/miniconda3/envs/MakerMods-lerobot/bin/python
conda activate MakerMods-lerobot
cd ~/makermods/lerobot
```

---

## ⚠️ Understand arm_end_type first (it sets the dimensions)

`MetalSDKInterface(can_id, urdf, arm_end_type, enable_arm)` has four end types
that directly determine the state/action dimensions:

| arm_end_type | End effector | URDF | `GetJointPosition` dim | Gripper usable | Integration state/action dim |
|---|---|---|---|---|---|
| 0 | none | `metal_no_gripper.urdf` | 6 | ❌ | **6** (joint1-6) |
| 1 | gripper | `metal_with_gripper.urdf` | 7 | ✅ | **7** (+gripper) |
| 2 | teaching pendant | `metal_with_gripper.urdf` | 7 | ❌ | **6** (7th value ignored) |
| 3 | gripper + pendant | `metal_with_gripper.urdf` | 7 | ✅ | **7** (+gripper) |

The code handles this automatically: `metal_motors(arm_end_type)` builds the
schema, `sync_read`/`sync_write` add the gripper only when usable, and the URDF
is auto-selected (when `urdf_path` is left empty).

### Misconfiguration is caught for you
- **Config time:** an invalid `arm_end_type` (not 0–3) is rejected by the config
  with a clear error.
- **Connect time:** `MetalMotorsBus.connect()` compares the real
  `GetJointPosition()` length against the configured `arm_end_type`. If you set
  `arm_end_type=1` (gripper) but the arm reports only 6 values, connecting fails
  with: *"the configured end effector does not match the hardware"*. Fix the
  `--robot.arm_end_type` / `--teleop.arm_end_type` value and retry.

### 🔴 Leader and follower must agree on gripper presence
LeRobot recording requires the leader's `action_features` to equal the
follower's. So:

- For **gripper teleoperation** (squeeze the leader gripper → follower gripper
  follows): use **`arm_end_type=1` (or 3) on BOTH arms** → both 7-dim, schemas
  match. ✅
- A **pendant-only leader (type 2, 6-dim)** with a **gripper follower (type 1,
  7-dim)** → schema mismatch, recording fails. Make both sides agree.
- Set it with `--robot.arm_end_type=N` / `--teleop.arm_end_type=N`; keep gripper
  presence identical on both sides.

---

## Stage 0 — No-hardware smoke (run now)

```bash
$PY -m pytest tests/motors/test_metal.py tests/robots/test_metal_follower.py tests/teleoperators/test_metal_leader.py -q
```
**Expect:** `31 passed`. Covers unit conversions, all `arm_end_type` variants,
control modes, schema agreement, safety clamping, misconfiguration detection,
independent joint/gripper velocity ratios, and the side-effect-free `is_connected`.

> These unit tests use an in-process SDK test double (`_MockMetalSDK`) so they
> run in CI without ROS2/hardware. The hardware-facing scripts in
> [`tests/`](./tests/) (`10`–`15`) always talk to the real arm. This file's
> inline hardware snippets and those scripts are equivalent — use whichever.

CLI recognizes the types (no hardware, no ROS2 needed):
```bash
$PY -c "import lerobot.scripts.lerobot_record; \
from lerobot.robots.config import RobotConfig; from lerobot.teleoperators.config import TeleoperatorConfig; \
print('robot ok:', 'metal_follower' in RobotConfig.get_known_choices()); \
print('teleop ok:', 'metal_leader' in TeleoperatorConfig.get_known_choices())"
```
**Expect:** both `True`.

---

## Stage 1 — Hardware connect + verify SDK assumptions (single arm, no motion)

The lowest-risk way to confirm/refute the assumptions the integration relies on.

```bash
source /opt/ros/humble/setup.bash      # metal_sdk links ROS2 C++ libs; must source first
conda activate MakerMods-lerobot
# First time on this machine, teach the adapter map: (cd docs/metal && ./start_can.sh setup)
(cd docs/metal && ./start_can.sh)       # bring up can0 (follower) / can1 (leader)
ip link show can0 && ip link show can1  # confirm both are UP
```

Verify SDK output on the follower (can0) alone:
```bash
$PY - <<'EOF'
from metal_sdk import MetalSDKInterface
from lerobot.motors.metal import default_urdf
END_TYPE = 1                                   # set to your real end type: 0/1/2/3
arm = MetalSDKInterface("can0", default_urdf(END_TYPE), END_TYPE, True)
assert arm.Init(), "Init failed"
import time; time.sleep(1)
pos = arm.GetJointPosition()
print("dim =", len(pos))                        # key #1: type 1 should be 7
print("joint names =", arm.GetJointNames())
print("position (rad) =", [round(x, 3) for x in pos])  # key #2: last value looks like gripper mm (0-80)?
EOF
```
**Checks**
- Dimension matches the table above (type 1 → 7). If not, the connect-time guard
  will already flag it; report so the maintainer can adjust `arm_position_dim`.
- The last value range looks like 0–80 (gripper mm), not radians → gripper is at
  index 6.
- `GetJointNames()` includes a gripper name when expected.

> Always release/disable after verifying (`SetEnableArm(False)` or exit).

Then verify through the lerobot wrapper (real, mock off):
```bash
$PY - <<'EOF'
from lerobot.robots.metal_follower import MetalFollower, MetalFollowerConfig
r = MetalFollower(MetalFollowerConfig(can_id="can0", arm_end_type=1))
r.connect()
print("observation_features:", r.observation_features)
print("one observation:", r.get_observation())
r.disconnect()
EOF
```
**Expect:** seven `jointX.pos` / `gripper.pos` keys, no exception. A wrong
`arm_end_type` raises a clear "does not match the hardware" error here.

---

## Stage 2 — Teleoperation loop (both arms, **hold the leader**)

> ⚠️ Safety: the leader (can1) runs in gravity compensation and **free-falls if
> the process exits abnormally**. The first time, **hold the leader, keep it
> low, clear of obstacles**, and be ready to cut power.

```bash
lerobot-teleoperate \
  --robot.type=metal_follower --robot.can_id=can0 --robot.arm_end_type=1 \
  --teleop.type=metal_leader  --teleop.can_id=can1 --teleop.arm_end_type=1
```
**Checks**
- Move the leader; the follower tracks in real time.
- **Direction matches** (a leader joint rotating one way → follower the same
  way). If reversed, the sign convention needs fixing; note which joint.
- Squeeze the leader gripper → follower gripper opens/closes (types 1/3 only).
- No large jump on the first frame (try `--robot.max_relative_target=5` to clamp).

Passing this proves dual-instance coexistence, conversion direction, and control
modes are all correct.

---

## Stage 3 — Record → replay (validate the LeRobotDataset)

Find camera indices first:
```bash
lerobot-find-cameras                     # note the index for high / left_wrist / right_wrist
```

Record one episode (validate the pipeline with a small amount first):
```bash
lerobot-record \
  --robot.type=metal_follower --robot.can_id=can0 --robot.arm_end_type=1 \
  --robot.cameras='{high: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}' \
  --teleop.type=metal_leader --teleop.can_id=can1 --teleop.arm_end_type=1 \
  --dataset.repo_id=HappyEthan/metal_test --dataset.num_episodes=1 --dataset.push_to_hub=false
```
**Checks:** recording runs without errors, the terminal shows a healthy frame
rate, a local dataset is produced.

Replay (follower reproduces; the leader can be disconnected):
```bash
lerobot-replay \
  --robot.type=metal_follower --robot.can_id=can0 --robot.arm_end_type=1 \
  --dataset.repo_id=HappyEthan/metal_test --dataset.episode=0
```
**Checks:** the follower smoothly reproduces the recorded motion and gripper.

---

## Stage 4 — Train → inference (full loop)

After recording enough data (e.g. ≥50 episodes):
```bash
lerobot-train --dataset.repo_id=HappyEthan/metal_task --policy.type=act \
  --output_dir=outputs/train/metal_act
```
Inference (policy drives the follower; leader optional; follower stays in NRT):
```bash
lerobot-record \
  --robot.type=metal_follower --robot.can_id=can0 --robot.arm_end_type=1 \
  --robot.cameras='{high: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}' \
  --policy.path=outputs/train/metal_act/checkpoints/last/pretrained_model \
  --dataset.repo_id=HappyEthan/metal_eval --dataset.num_episodes=5 --dataset.push_to_hub=false
```

---

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `ImportError: librclcpp.so` | Did not `source /opt/ros/humble/setup.bash` |
| `... does not match the hardware` / `expected >= 6` | `arm_end_type` does not match the real end effector; set it per the table |
| `arm_end_type=1 expects a gripper but ... only 6 values` | No gripper present; use `arm_end_type=0` or change the end effector |
| record reports action/observation dim mismatch | Leader/follower gripper presence differs (see 🔴 above) |
| `Init()` failed / cannot connect | `start_can.sh` not run, wrong URDF path, or can0/can1 swapped |
| Follower moves in reverse | Joint sign convention; negate in the `MetalMotorsBus` conversion (report the joint) |
| Leader drops after Ctrl-C | Inherent gravity-comp risk; hold it the first time, later add a SIGINT hook to switch to NRT then disable torque |

---

## Verified on real hardware (2026-06-29, dual arm)

1. ✅ `GetJointPosition` **dimension** per `arm_end_type` (7 for type 1).
2. ✅ `SetArmJointPosition` (6-elem + velocity_ratio) drives the joints.
3. ✅ **Dual-instance** (can0 follower + can1 leader) coexist in one process.
4. ✅ Conversion **direction** correct (joints track the leader the right way).
5. ✅ Gripper position/angle correct; gripper is mechanically slower than the
   joints, mitigated with a faster `gripper_velocity_ratio` (default 10).
6. ✅ End-to-end teleoperate → record → replay and rerun (`--display_data=true`)
   all working through the official CLI.

Still hardware-specific per machine: camera `/dev/video*` indices (find via
`lerobot-find-cameras`) when recording with cameras for training.
