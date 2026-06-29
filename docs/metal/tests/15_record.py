#!/usr/bin/env python
"""阶段 3 —— 录制一条 episode 到 LeRobotDataset(无相机版),验证 record→replay 链。

用我们的 MetalFollower + MetalLeader 跑一个最小录制循环,直接复用 lerobot 的数据集
与特征工具(hw_to_dataset_features / build_dataset_frame / LeRobotDataset),所以产出
的是**标准 LeRobotDataset**,可被 `lerobot-replay` 直接回放。

每帧:follower.get_observation()(状态)+ leader.get_action()(动作)→ follower.send_action()
→ 写入数据集。本脚本默认**不接相机**(先把状态/动作这条链跑通);相机用官方
`lerobot-record` 加 `--robot.cameras=...` 更合适。

⚠️ 需要两个臂 + 两个适配器(can0=follower / can1=leader)。无硬件用 `--mock` 验证逻辑。
⚠️ 安全:leader 重力补偿,异常退出会自由下坠;录制时扶着点、放低、清空周围。

用法:
  python docs/metal/tests/15_record.py --follower-can can0 --leader-can can1 --end-type 1 \
      --repo-id local/metal_record_test --seconds 10
  # 回放:
  lerobot-replay --robot.type=metal_follower --robot.can_id=can0 --robot.arm_end_type=1 \
      --dataset.repo_id=local/metal_record_test --dataset.episode=0

  python docs/metal/tests/15_record.py --mock --seconds 1 --fps 5 --root /tmp/metal_ds  # 仅验证逻辑
"""

import argparse
import time

from lerobot.datasets import LeRobotDataset
from lerobot.robots.metal_follower import MetalFollower, MetalFollowerConfig
from lerobot.teleoperators.metal_leader import MetalLeader, MetalLeaderConfig
from lerobot.utils.feature_utils import build_dataset_frame, combine_feature_dicts, hw_to_dataset_features

ACTION = "action"
OBS = "observation"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--follower-can", default="can0", help="follower 接口(默认 can0)")
    parser.add_argument("--leader-can", default="can1", help="leader 接口(默认 can1)")
    parser.add_argument("--end-type", type=int, default=1, help="arm_end_type 0/1/2/3(两臂一致)")
    parser.add_argument("--fps", type=int, default=30, help="录制帧率(默认 30)")
    parser.add_argument("--seconds", type=float, default=10.0, help="录制时长秒(默认 10)")
    parser.add_argument("--max-step", type=float, default=5.0, help="follower 每周期单关节限幅(度),0=不限")
    parser.add_argument("--repo-id", default="local/metal_record_test", help="数据集 repo_id(namespace/name)")
    parser.add_argument("--root", default=None, help="数据集根目录(默认 HF 缓存);重复跑请换 repo-id 或 root")
    parser.add_argument("--task", default="metal teleop record test", help="任务描述文本")
    parser.add_argument("--mock", action="store_true", help="使用进程内 mock SDK(无硬件)")
    args = parser.parse_args()

    max_rel = args.max_step if args.max_step > 0 else None

    leader = MetalLeader(MetalLeaderConfig(can_id=args.leader_can, arm_end_type=args.end_type, mock=args.mock))
    follower = MetalFollower(
        MetalFollowerConfig(
            can_id=args.follower_can,
            arm_end_type=args.end_type,
            max_relative_target=max_rel,
            mock=args.mock,
        )
    )

    # 录制要求 leader 动作与 follower 动作同构,否则数据集 action/observation 维度会不一致。
    if leader.action_features != follower.action_features:
        raise SystemExit(
            "leader 与 follower 的 action_features 不一致(检查 --end-type 是否相同、夹爪是否都有):\n"
            f"  leader  : {list(leader.action_features)}\n"
            f"  follower: {list(follower.action_features)}"
        )

    # 用 lerobot 的工具把 hw 特征转成数据集特征(无相机 → use_video=False)。
    features = combine_feature_dicts(
        hw_to_dataset_features(follower.action_features, ACTION, use_video=False),
        hw_to_dataset_features(follower.observation_features, OBS, use_video=False),
    )
    print("数据集特征:")
    for k, v in features.items():
        print(f"  {k}: shape={v['shape']} names={v.get('names')}")

    try:
        dataset = LeRobotDataset.create(
            repo_id=args.repo_id,
            fps=args.fps,
            features=features,
            root=args.root,
            robot_type=follower.name,
            use_videos=False,
        )
    except (FileExistsError, ValueError) as e:
        raise SystemExit(
            f"创建数据集失败({e})。该 repo_id/root 可能已存在;请换 --repo-id 或 --root,或删除旧目录。"
        )
    print(f"\n数据集已创建 → {dataset.root}")

    follower.connect()
    leader.connect()
    print(f"已连接。开始录制 {args.seconds}s @ {args.fps}fps。拖动 leader 演示动作,Ctrl-C 提前停止(先扶住!)\n")

    period = 1.0 / args.fps
    n_frames = 0
    start = time.perf_counter()
    try:
        while time.perf_counter() - start < args.seconds:
            loop_t = time.perf_counter()
            obs = follower.get_observation()  # follower 状态(先读)
            action = leader.get_action()  # leader 动作
            follower.send_action(action)  # 写 follower(内部限幅)

            obs_frame = build_dataset_frame(dataset.features, obs, prefix=OBS)
            action_frame = build_dataset_frame(dataset.features, action, prefix=ACTION)
            dataset.add_frame({**obs_frame, **action_frame, "task": args.task})
            n_frames += 1

            print(f"  录制中… 帧 {n_frames}  t={time.perf_counter() - start:5.1f}s", end="\r", flush=True)
            if args.mock and n_frames >= max(1, int(args.seconds * args.fps)):
                break
            time.sleep(max(0.0, period - (time.perf_counter() - loop_t)))
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C,停止录制。")
    finally:
        print(f"\n断开机械臂… 已录 {n_frames} 帧。")
        follower.disconnect()
        leader.disconnect()

    if n_frames == 0:
        raise SystemExit("没有录到任何帧,未保存 episode。")
    dataset.save_episode()
    print(f"\n✅ episode 已保存。共 {n_frames} 帧 → {dataset.root}")
    print("回放命令:")
    print(
        f"  lerobot-replay --robot.type=metal_follower --robot.can_id={args.follower_can} "
        f"--robot.arm_end_type={args.end_type} --dataset.repo_id={args.repo_id} --dataset.episode=0"
        + (f" --dataset.root={args.root}" if args.root else "")
    )


if __name__ == "__main__":
    main()
