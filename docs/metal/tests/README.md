# Metal × LeRobot —— 分阶段测试脚本

可直接运行的脚本,用来验证融合是否生效,按**风险递增**排序。请从上往下逐个运行,
每一步通过后再进行下一步。完整参考(故障排查表、训练/评估)见
[`../TESTING.md`](../TESTING.md)。

## 一次性前置准备

```bash
source /opt/ros/humble/setup.bash         # metal_sdk 链接了 ROS2 的 .so
conda activate MakerMods-lerobot
cd ~/makermods/lerobot
export PY=/home/ethan/miniconda3/envs/MakerMods-lerobot/bin/python
```

每条硬件命令都要把 `--end-type` 设成你的真实末端执行器:
`0=无(6 dof)`、`1=夹爪(7 dof)`、`2=示教器`、`3=夹爪+示教器`。

## 各阶段

| 阶段 | 命令 | 验证什么 | 硬件 |
|---|---|---|---|
| 0 | `./docs/metal/tests/00_smoke.sh` | 代码接线、CLI 注册 metal 类型 | 不需要 |
| 1a | `$PY docs/metal/tests/10_sdk_raw.py --can can0 --end-type 1` | SDK 维度/夹爪与 arm_end_type 一致 | follower,不运动 |
| 1b | `$PY docs/metal/tests/11_wrapper_read.py --can can0 --end-type 1` | 封装层能连接 + 读到观测 | follower,不运动 |
| 1c | `$PY docs/metal/tests/12_single_joint.py --can can0 --end-type 1 --joint joint1 --delta 5` | 首次运动 + 方向是否正确 | follower,**小幅运动** |
| 1d | `$PY docs/metal/tests/13_leader_gravity_read.py --can can1 --end-type 1` | leader 重力补偿 + 徒手拖动实时读角 | **仅单 leader 臂** |
| 2 | `$PY docs/metal/tests/14_teleop.py --follower-can can0 --leader-can can1 --end-type 1`(或下方 CLI) | leader→follower 跟随 | 双臂 |
| 3 | `$PY docs/metal/tests/15_record.py --follower-can can0 --leader-can can1 --end-type 1 --repo-id local/metal_record_test`(回放见下) | 录制 LeRobotDataset → 回放 | 双臂(无相机) |

### 无硬件试跑
在碰真机之前先验证脚本逻辑(使用进程内 mock SDK):

```bash
$PY docs/metal/tests/11_wrapper_read.py --mock
$PY docs/metal/tests/12_single_joint.py --mock --delta 5
```

### 任何硬件阶段之前先拉起 CAN

```bash
./docs/metal/start_can.sh setup     # 仅这台机器第一次需要
./docs/metal/start_can.sh           # 每次开机:can0(follower)/ can1(leader)
ip link show can0 && ip link show can1
```

## 阶段 1c —— 需要观察什么
- 移动幅度很小(默认 5°),且机械臂会**自动回到起点**。
- 确认关节朝**预期方向**运动。如果反了,记下是哪个关节 ——
  `MetalMotorsBus` 里的正负号约定需要翻转。
- 拒绝 `|delta| > 15°`;退出时一定会关闭力矩。


## 阶段 2 —— 遥操(扶住 leader!)

> ⚠️ leader 运行在重力补偿模式,**进程异常退出会自由下落**。第一次:
> 扶住它,放低,远离障碍物。

leader 和 follower 必须用**相同的 `arm_end_type`**(夹爪一致)。

```bash
lerobot-teleoperate \
  --robot.type=metal_follower --robot.can_id=can0 --robot.arm_end_type=1 \
  --teleop.type=metal_leader  --teleop.can_id=can1 --teleop.arm_end_type=1
```
加 `--robot.max_relative_target=5` 可限制首帧跳变。

## 阶段 3 —— 录制 → 回放

```bash
lerobot-find-cameras        # 记下相机 index

lerobot-record \
  --robot.type=metal_follower --robot.can_id=can0 --robot.arm_end_type=1 \
  --robot.cameras='{high: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}' \
  --teleop.type=metal_leader --teleop.can_id=can1 --teleop.arm_end_type=1 \
  --dataset.repo_id=HappyEthan/metal_test --dataset.num_episodes=1 --dataset.push_to_hub=false

lerobot-replay \
  --robot.type=metal_follower --robot.can_id=can0 --robot.arm_end_type=1 \
  --dataset.repo_id=HappyEthan/metal_test --dataset.episode=0
```

故障排查 + 训练/评估(阶段 4):见 [`../TESTING.md`](../TESTING.md)。
