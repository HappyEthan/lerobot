#!/usr/bin/env python
"""阶段 1b —— lerobot 封装层只读检查(单臂,不产生任何运动)。

通过真实的 MetalFollower 对象(和 CLI 用的是同一个类)来确认:封装层能连接、
暴露出预期的观测特征、并返回一帧观测。仍然是只读:不发送任何运动指令。

如果 --end-type 设错,会在这里由 connect() 抛出清晰的“与硬件不匹配”错误 ——
这正是我们想要的安全行为。

前置条件:与 10_sdk_raw.py 相同(source ROS2、start_can.sh)。

用法:
  python docs/metal/tests/11_wrapper_read.py --can can0 --end-type 1
  python docs/metal/tests/11_wrapper_read.py --mock        # 无硬件,仅验证逻辑
"""

import argparse

from lerobot.robots.metal_follower import MetalFollower, MetalFollowerConfig


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--can", default="can0", help="SocketCAN 接口(默认 can0)")
    parser.add_argument("--end-type", type=int, default=1, help="arm_end_type 0/1/2/3")
    parser.add_argument("--mock", action="store_true", help="使用进程内 mock SDK(无硬件)")
    args = parser.parse_args()

    cfg = MetalFollowerConfig(can_id=args.can, arm_end_type=args.end_type, mock=args.mock)
    robot = MetalFollower(cfg)
    robot.connect()
    try:
        feats = robot.observation_features
        obs = robot.get_observation()
        print("观测特征 observation_features:")
        for name, dtype in feats.items():
            print(f"  {name}: {dtype}")
        print("\n一帧观测:")
        for name, value in obs.items():
            shown = round(value, 3) if isinstance(value, float) else value
            print(f"  {name} = {shown}")
        print(f"\n阶段 1b 通过 —— 读到 {len(obs)} 个特征,无报错。")
    finally:
        robot.disconnect()
        print("已断开连接。")


if __name__ == "__main__":
    main()
