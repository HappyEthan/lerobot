# Metal-Arm × LeRobot — CLI 命令速查

日常用官方 CLI 操作 metal 双臂(`metal_follower`=can0 从臂 / `metal_leader`=can1 主臂)。
分阶段调试脚本见 [`tests/README.md`](./tests/README.md),完整说明见 [`README.md`](./README.md)。

---

## 0. 每次开始前

```bash
conda activate MakerMods-lerobot          # 自动配好 ROS 库路径(无需手动 source)
cd ~/makermods/lerobot
./docs/metal/start_can.sh                  # 拉起 can0(从)/ can1(主);首次本机先跑一次 setup
ip -br link show can0 && ip -br link show can1   # 两个都应 UP
```

---

## 1. 遥操 teleoperate

```bash
lerobot-teleoperate \
  --robot.type=metal_follower --robot.can_id=can0 --robot.arm_end_type=1 \
  --robot.max_relative_target=5 \
  --teleop.type=metal_leader  --teleop.can_id=can1 --teleop.arm_end_type=1
```
加可视化:末尾加 `--display_data=true`(本地弹出 rerun 窗口)。

---

## 2. 录制 record

**本地测试(无相机,2 条各 20s,不传 Hub)**:
```bash
lerobot-record \
  --robot.type=metal_follower --robot.can_id=can0 --robot.arm_end_type=1 \
  --robot.max_relative_target=5 \
  --teleop.type=metal_leader --teleop.can_id=can1 --teleop.arm_end_type=1 \
  --dataset.repo_id=local/metal_test --dataset.single_task="pick and place" \
  --dataset.num_episodes=2 --dataset.episode_time_s=20 --dataset.reset_time_s=10 \
  --dataset.push_to_hub=false
```

**正式采集(带相机,50 条,传到你的 HF)**:先 `lerobot-find-cameras` 找 index,然后把相机行加进 `--robot.*` 组:
```bash
  --robot.cameras='{high: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}' \
```
并改 `--dataset.repo_id=<你的HF用户名>/<任务名> --dataset.num_episodes=50 --dataset.push_to_hub=true`。

**录制时键盘控制**:`→` 提前结束并进入下一条 · `←` 重录本条 · `Esc` 停止并保存。
每条之间有 `reset_time_s` 秒复位场景。

---

## 3. 回放 replay

```bash
lerobot-replay --robot.type=metal_follower --robot.can_id=can0 --robot.arm_end_type=1 \
  --robot.max_relative_target=5 \
  --dataset.repo_id=local/metal_test --dataset.episode=0
```
回放只用 follower(主臂可松手放一边);follower 会自己动,**清空周围**。

---

## 4. 训练 train

```bash
lerobot-train --dataset.repo_id=<你>/<任务> --policy.type=act \
  --output_dir=outputs/train/metal_act
```

## 5. 真机推理 eval(策略驱动 follower)

```bash
lerobot-record \
  --robot.type=metal_follower --robot.can_id=can0 --robot.arm_end_type=1 \
  --robot.cameras='{high: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}' \
  --policy.path=outputs/train/metal_act/checkpoints/last/pretrained_model \
  --dataset.repo_id=local/metal_eval --dataset.num_episodes=5 --dataset.push_to_hub=false
```

---

## 6. 可视化 rerun

- **本地**(有桌面):任意命令加 `--display_data=true` → 自动弹窗,显示各关节 `observation.*` / `action.*` 时间序列曲线 + 相机图像。
- **远程**(robot 无桌面,笔记本上看):笔记本 `rerun --serve` 记下端口,robot 端加
  `--display_data=true --display_ip=<笔记本IP> --display_port=<端口>`,相机多再加 `--display_compressed_images=true`。
- 调内存上限:`export LEROBOT_RERUN_MEMORY_LIMIT=25%`。

---

## 常用参数速查

| 参数 | 含义 | 默认 / 建议 |
|---|---|---|
| `--robot.arm_end_type` / `--teleop.arm_end_type` | 末端:0=无(6维) 1=夹爪(7维) 2=示教器(6维有效) 3=夹爪+示教器(7维) | 两臂必须一致 |
| `--robot.max_relative_target` | follower 每周期单关节最大变化(度),防暴冲 | 5(首次更稳用 3) |
| `--robot.gripper_velocity_ratio` | 夹爪速度比 1-10(越大越快) | 10(已默认,机械上限) |
| `--robot.velocity_ratio` | 关节速度比 1-10 | 5 |
| `--display_data` | 开 rerun 可视化(顶层 flag,无前缀) | true 时开 |

## 常见坑

- **`--dataset.push_to_hub` 默认 `true`** → 本地测试务必 `=false`,否则会尝试上传 HF(需登录 + repo_id 用你的 HF 用户名)。
- **`--dataset.single_task` 必填**(任务描述文本)。
- **同 `repo_id` 重录会冲突** → 换名,或删 `~/.cache/huggingface/lerobot/<repo_id>`。
- **`--display_data` 是顶层 flag**,别写成 `--robot.display_data`。
- **SSH 无图形界面**时本地 rerun 弹不出 → 用远程模式。
- **`eval_` 前缀的 repo_id 被保留**(给策略评估用),录数据别用。

---

## 参数怎么选(arm_end_type)

| 末端 | arm_end_type | 维度 | 夹爪可用 |
|---|---|---|---|
| 无末端 | 0 | 6 | ❌ |
| 夹爪 | 1 | 7 | ✅ |
| 示教器 | 2 | 6(第7维忽略) | ❌ |
| 夹爪+示教器 | 3 | 7 | ✅ |

主从必须**夹爪存在性一致**(都 1 或都 3),否则录制时 action/observation 维度不匹配。
配错末端会在连接时报 "does not match the hardware"。
