# Metal-Arm × LeRobot 集成 — 测试指南

本指南从「无硬件」到「完整端到端」分阶段验证 `metal_follower` / `metal_leader` 集成。
按顺序做:每一阶段都建立在上一阶段已通过的基础上,风险逐级升高。

- 代码位置:`src/lerobot/motors/metal/`、`src/lerobot/robots/metal_follower/`、`src/lerobot/teleoperators/metal_leader/`
- 设计文档:`~/makermods/docs/superpowers/specs/2026-06-24-metal-arm-lerobot-integration-design.md`
- 环境:conda **`MakerMods-lerobot`**(Python 3.12)
- Python:`/home/ethan/miniconda3/envs/MakerMods-lerobot/bin/python`(下文简称 `$PY`)

```bash
export PY=/home/ethan/miniconda3/envs/MakerMods-lerobot/bin/python
conda activate MakerMods-lerobot
cd ~/makermods/lerobot
```

---

## ⚠️ 先搞清楚 arm_end_type(决定维度与夹爪)

`MetalSDKInterface(can_id, urdf, arm_end_type, enable_arm)` 的 `arm_end_type` 有四种,**直接决定 state/action 维度**:

| arm_end_type | 末端 | URDF | `GetJointPosition` 维度 | 夹爪可用 | 本集成 state/action 维度 |
|---|---|---|---|---|---|
| 0 | 无末端 | `metal_no_gripper.urdf` | 6 | ❌ | **6**(仅 joint1-6) |
| 1 | 夹爪 | `metal_with_gripper.urdf` | 7 | ✅ | **7**(+gripper) |
| 2 | 示教器 | `metal_with_gripper.urdf` | 7 | ❌ | **6**(第7维忽略) |
| 3 | 夹爪+示教器 | `metal_with_gripper.urdf` | 7 | ✅ | **7**(+gripper) |

代码已按此表自动处理:`metal_motors(arm_end_type)` 决定 schema、`sync_read/sync_write` 按夹爪存在性收发、URDF 按类型自动选(留空时)。

### 🔴 主从必须「夹爪存在性一致」
lerobot 录制要求 **leader 的 `action_features` ≡ follower 的 `action_features`**。所以:

- 想要**带夹爪遥操作**(人捏主臂夹爪→从臂夹爪跟随):**主、从都用 `arm_end_type=1`(或 3)** → 双方都 7 维,schema 对齐。✅
- 若**主臂是纯示教器(type 2)**、从臂带夹爪(type 1):主臂 6 维、从臂 7 维 → **schema 不一致,录制会报错**。此时要么从臂也去掉夹爪,要么换带夹爪的主臂。
- 配置时用 `--robot.arm_end_type=N` / `--teleop.arm_end_type=N` 指定,务必让两边的夹爪存在性一致。

---

## 阶段 0 — 无硬件冒烟(现在就能跑)

```bash
$PY -m pytest tests/motors/test_metal.py tests/robots/test_metal_follower.py tests/teleoperators/test_metal_leader.py -q
```
**期望**:`28 passed`。覆盖单位换算、夹爪/示教器/无末端三种 `arm_end_type`、控制模式、schema 一致、安全限幅、`is_connected` 无副作用。

CLI 能识别类型(无需硬件、无需 ROS2):
```bash
$PY -c "import lerobot.scripts.lerobot_record; \
from lerobot.robots.config import RobotConfig; from lerobot.teleoperators.config import TeleoperatorConfig; \
print('robot ok:', 'metal_follower' in RobotConfig.get_known_choices()); \
print('teleop ok:', 'metal_leader' in TeleoperatorConfig.get_known_choices())"
```
**期望**:两个都 `True`。

---

## 阶段 1 — 真机连接 + 验证 SDK 假设(单臂,不运动)

这一步用**最低风险**证实/证伪集成依赖的三个关键假设。

```bash
source /opt/ros/humble/setup.bash      # metal_sdk 链接 ROS2 C++ 库,必须先 source
conda activate MakerMods-lerobot
./start_can.sh                          # 拉起 can0(follower) / can1(leader)
ip link show can0 && ip link show can1  # 确认两路都 UP
```

用从臂(can0)单独验证 SDK 返回:
```bash
$PY - <<'EOF'
from metal_sdk import MetalSDKInterface
from lerobot.motors.metal import default_urdf
END_TYPE = 1                                   # 按你的真实末端改:0/1/2/3
arm = MetalSDKInterface("can0", default_urdf(END_TYPE), END_TYPE, True)
assert arm.Init(), "Init failed"
import time; time.sleep(1)
pos = arm.GetJointPosition()
print("维度 =", len(pos))                       # 关键①: type1 应为 7
print("关节名 =", arm.GetJointNames())
print("位置(弧度) =", [round(x, 3) for x in pos])  # 关键②: 末位像夹爪mm(0-80)吗
EOF
```
**检查点**
- 维度与上表一致(type 1 → 7)。若不符 → 告诉维护者,几行改 `arm_position_dim`。
- 最后一维数值范围像 0–80(mm 夹爪)而非弧度 → 确认夹爪在第 7 位。
- `GetJointNames()` 是否含夹爪名。

> 验证完**务必断电/释放**(`SetEnableArm(False)` 或退出进程)。

接着用 lerobot 封装验证(mock 关闭,真机):
```bash
$PY - <<'EOF'
from lerobot.robots.metal_follower import MetalFollower, MetalFollowerConfig
r = MetalFollower(MetalFollowerConfig(can_id="can0", arm_end_type=1))
r.connect()
print("observation_features:", r.observation_features)
print("一次观测:", r.get_observation())
r.disconnect()
EOF
```
**期望**:打印 7 个 `jointX.pos`/`gripper.pos` 键,无异常。

---

## 阶段 2 — 遥操作闭环(主从都接,**手扶主臂**)

> ⚠️ 安全:主臂(can1)为重力补偿,**进程异常退出时会自由下落**。首次务必**手扶主臂、低姿态、远离障碍**,准备随时断电。

```bash
lerobot-teleoperate \
  --robot.type=metal_follower --robot.can_id=can0 --robot.arm_end_type=1 \
  --teleop.type=metal_leader  --teleop.can_id=can1 --teleop.arm_end_type=1
```
**检查点**
- 推动主臂,从臂实时跟随。
- **方向一致**(主臂某关节正转 → 从臂同向)。若反向 → 单位/符号约定需修正,记录哪个关节。
- 捏主臂夹爪 → 从臂夹爪同步开合(仅 type 1/3)。
- 首帧无大跳变(可加 `--robot.max_relative_target=5` 限幅试)。

通过 = 双实例共存、换算方向、控制模式全部正确。

---

## 阶段 3 — 录制 → 回放(验证 LeRobotDataset)

先找相机索引:
```bash
lerobot-find-cameras                     # 记下 high/left_wrist/right_wrist 的 index
```

录 1 条(先少量验证管线):
```bash
lerobot-record \
  --robot.type=metal_follower --robot.can_id=can0 --robot.arm_end_type=1 \
  --robot.cameras='{high: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}' \
  --teleop.type=metal_leader --teleop.can_id=can1 --teleop.arm_end_type=1 \
  --dataset.repo_id=HappyEthan/metal_test --dataset.num_episodes=1 --dataset.push_to_hub=false
```
**检查点**:录制无报错、终端显示帧率正常、本地生成数据集。

回放(从臂复现,主臂可不接):
```bash
lerobot-replay \
  --robot.type=metal_follower --robot.can_id=can0 --robot.arm_end_type=1 \
  --dataset.repo_id=HappyEthan/metal_test --dataset.episode=0
```
**检查点**:从臂平滑复现录制动作、夹爪开合还原。

---

## 阶段 4 — 训练 → 推理(完整闭环)

录够数据(如 ≥50 条)后:
```bash
lerobot-train --dataset.repo_id=HappyEthan/metal_task --policy.type=act \
  --output_dir=outputs/train/metal_act
```
推理(policy 驱动从臂,主臂可不接,从臂仍走 NRT):
```bash
lerobot-record \
  --robot.type=metal_follower --robot.can_id=can0 --robot.arm_end_type=1 \
  --robot.cameras='{high: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}' \
  --policy.path=outputs/train/metal_act/checkpoints/last/pretrained_model \
  --dataset.repo_id=HappyEthan/metal_eval --dataset.num_episodes=5 --dataset.push_to_hub=false
```

---

## 故障速查

| 现象 | 可能原因 / 处理 |
|---|---|
| `ImportError: librclcpp.so` | 没 `source /opt/ros/humble/setup.bash` |
| `GetJointPosition() returned N values, expected >= 6` | `arm_end_type` 与真实末端不符;按上表设对 |
| `arm_end_type=1 expects a gripper but ... only 6 values` | 实际无夹爪,应改 `arm_end_type=0` 或换末端 |
| 录制报 action/observation 维度不匹配 | 主从 `arm_end_type` 夹爪存在性不一致(见上文 🔴) |
| `Init()` 失败 / 连不上 | `start_can.sh` 没起、URDF 路径错、can0/can1 接反 |
| 从臂方向相反 | 关节符号约定,需在 `MetalMotorsBus` 换算处取反(记录关节号反馈) |
| 主臂 Ctrl-C 后下落 | 重力补偿固有风险;首次手扶,后续可加 SIGINT 钩子先切 NRT 再断力矩 |

---

## 已知未验证项(实现基于头文件/手册推断)

1. `GetJointPosition` 各 `arm_end_type` 的**实际维度**(阶段 1 验证)。
2. `SetArmJointPosition` 的 **6元+velocity_ratio 重载**在 pybind11 下的解析(阶段 2 验证)。
3. **双实例**(can0+can1)同进程稳定性(阶段 2 验证)。
4. 关节/夹爪换算的**方向与零位**(阶段 2 验证)。
5. 相机 `/dev/video*` 实际索引(阶段 3 用 `lerobot-find-cameras`)。
