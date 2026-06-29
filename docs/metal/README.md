# Metal-Arm × LeRobot Integration

Drive the MakerMods metal-arm (a leader/follower pair) directly from LeRobot:
`teleoperate → record → replay → train → eval`, all through the native CLI.

- Robot: `metal_follower` (can0, NRT position control, 7-dim state + cameras)
- Teleoperator: `metal_leader` (can1, gravity compensation)
- No ROS2 node required at runtime; the integration calls `metal_sdk` directly.

This folder is self-contained: the CAN bring-up script lives here and the arm
URDFs are bundled with the Python package, so a fresh clone works without editing
absolute paths.

---

## Prerequisites

1. **conda env `MakerMods-lerobot`** (Python 3.12) with this repo installed
   editable and `metal_sdk` available.
2. **ROS2 Humble runtime libraries** — `metal_sdk`'s `.so` links them. You do not
   run any ROS2 node, but you must source it so the loader finds `librclcpp.so`:
   ```bash
   source /opt/ros/humble/setup.bash
   conda activate MakerMods-lerobot
   ```
3. **Two USB-CAN adapters** (CANable-style) for the leader and follower arms.

---

## Setting up a new machine — do you need ROS2?

**It depends on what the machine does.** `import metal_sdk` is deferred (it only
happens when you `connect()` to real hardware), so ROS2 is needed *only on a
machine that drives the physical arm*.

| Machine's role | Needs ROS2? |
|---|---|
| Training / dataset work / running tests / `lerobot-replay` of recorded data | ❌ No |
| Driving the real arm (`teleoperate`, `record`, on-hardware `eval`) | ✅ Yes |

**Why:** `metal_sdk` → `libmetal_sdk_x64.so` (a vendor precompiled binary) hard-links
25 ROS2 `.so` files (`librclcpp.so`, `libkdl_parser.so`, …). The loader must resolve
them at import time. Sourcing ROS2 only sets `LD_LIBRARY_PATH`; **no ROS node, daemon,
or DDS traffic runs** — you are just loading C++ libraries. The `kdl_parser`/`urdf`
parts do the URDF kinematics and gravity compensation (that is why a URDF is passed).

### Minimal ROS2 install (control machine only)

You do **not** need the full ROS2 desktop. Install just the runtime libraries
(Ubuntu 22.04 + ROS2 Humble; for arm64/Jetson use the matching arm64 packages):

```bash
sudo apt install ros-humble-ros-base \
                 ros-humble-kdl-parser ros-humble-urdf \
                 libnlopt0 libgoogle-glog0v5
```

- `ros-humble-ros-base` — rclcpp/rcl/rmw/rosidl/ament stack (no GUI)
- `ros-humble-kdl-parser`, `ros-humble-urdf` — kinematics / gravity compensation
- `libnlopt0`, `libgoogle-glog0v5` — IK optimisation and logging

**Footprint:** the whole `/opt/ros/humble` for this minimal set is **~110 MB on
disk** (vs ~1 GB+ for `ros-humble-desktop`). After installing, every hardware
session just needs:

```bash
source /opt/ros/humble/setup.bash
conda activate MakerMods-lerobot
```

> Tip: to never type `source` again, add that line to the conda env's
> `activate.d` so `LD_LIBRARY_PATH` is set automatically on `conda activate`.

---

## Step 1 — Bring up the CAN interfaces

The arms appear as `can0` (slave/follower) and `can1` (master/leader). Which
physical adapter maps to which interface is **machine-specific**, so each new
machine teaches it once.

```bash
cd docs/metal
./start_can.sh setup     # one-time: plug in each arm's adapter when prompted
./start_can.sh           # every session: bring up can0 / can1
./start_can.sh status    # inspect state
./start_can.sh stop      # tear down
```

`setup` writes the adapter→interface map to `can_map.conf` (git-ignored,
per-machine). You never edit the script. See [CAN setup details](#can-setup-details).

---

## Step 2 — Run the integration

```bash
# Teleoperate (verify the master/slave loop)
lerobot-teleoperate \
  --robot.type=metal_follower --robot.can_id=can0 --robot.arm_end_type=1 \
  --teleop.type=metal_leader  --teleop.can_id=can1 --teleop.arm_end_type=1

# Record a LeRobotDataset
lerobot-record \
  --robot.type=metal_follower --robot.can_id=can0 --robot.arm_end_type=1 \
  --robot.cameras='{high: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}' \
  --teleop.type=metal_leader --teleop.can_id=can1 --teleop.arm_end_type=1 \
  --dataset.repo_id=<you>/metal_task --dataset.num_episodes=50
```

Full staged validation (mock → hardware → end-to-end) is in
[`TESTING.md`](./TESTING.md).

---

## End effector (`arm_end_type`)

`arm_end_type` sets the state/action dimensions. The code adapts automatically and
**rejects mismatches** (at config time for invalid values, at connect time when
the value does not match the hardware).

| arm_end_type | End effector | Dim | Gripper |
|---|---|---|---|
| 0 | none | 6 | ❌ |
| 1 | gripper | 7 | ✅ |
| 2 | teaching pendant | 6 (7th ignored) | ❌ |
| 3 | gripper + pendant | 7 | ✅ |

**Leader and follower must agree on gripper presence** (both 1, or both 3) so
their `action_features` match for recording. Set with
`--robot.arm_end_type` / `--teleop.arm_end_type`.

---

## URDF

The two arm URDFs are bundled at `src/lerobot/motors/metal/urdf/` and selected
automatically by `arm_end_type` — no path configuration needed on a fresh clone.

To use your own URDFs (e.g. a copy with meshes), either:
- set `export METAL_URDF_DIR=/path/to/your/urdf` (must contain
  `metal_with_gripper.urdf` and `metal_no_gripper.urdf`), or
- pass `--robot.urdf_path=/abs/path.urdf` explicitly.

---

## CAN setup details

`start_can.sh` auto-detects USB-CAN adapters, generates udev rules, and runs
`slcand` to expose SocketCAN interfaces.

- **Per-machine config** `can_map.conf` (generated by `setup`) maps each adapter's
  USB serial to an interface + role and overrides the defaults in the script. Keep
  it out of version control; it is specific to your physical adapters.
- **Roles are deterministic**: `setup` asks you to plug each arm's adapter in
  alone, so master/slave never depend on USB enumeration order.
- **sudo**: `slcand`/udev need root. The script prompts interactively by default;
  set `SUDO_PASS` in `can_map.conf` for unattended use.

```text
# can_map.conf (example, auto-generated — do not commit)
CAN_IFACE["can0"]="slave|<follower-adapter-serial>"
CAN_IFACE["can1"]="master|<leader-adapter-serial>"
# SUDO_PASS="..."   # optional, for unattended sudo
```

> ⚠️ Safety: the leader runs in gravity compensation and free-falls if the process
> exits abnormally. Hold it on first bring-up and keep it low.
