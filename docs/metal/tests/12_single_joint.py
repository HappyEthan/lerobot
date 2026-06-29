#!/usr/bin/env python
"""阶段 1c —— 首次运动:受保护的单关节小幅移动(单臂)。

这是第一个会让机械臂动起来的脚本。它通过 MetalMotorsBus(与 CLI 相同的路径)
把某一个关节移动一个很小的角度,然后把机械臂精确地送回起始位置。所有动作都
包在 try/finally 里,确保无论如何机械臂都会归位、退出时关闭力矩。

安全:请扶住机械臂,放低、远离障碍物,随时准备断电。脚本在运动前会要求显式
确认,并拒绝过大的角度变化。

前置条件:source ROS2、conda activate、./docs/metal/start_can.sh。

用法:
  python docs/metal/tests/12_single_joint.py --can can0 --end-type 1 --joint joint1 --delta 5
"""

import argparse
import time

from lerobot.motors.metal import JOINT_NAMES, MetalMotorsBus, default_urdf, metal_motors, validate_arm_end_type

MAX_DELTA_DEG = 15.0  # 首次运动测试,拒绝比这更大的角度


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--can", default="can0", help="SocketCAN 接口(默认 can0)")
    parser.add_argument("--end-type", type=int, default=1, help="arm_end_type 0/1/2/3")
    parser.add_argument("--joint", default="joint1", choices=JOINT_NAMES, help="要小幅移动的关节")
    parser.add_argument("--delta", type=float, default=5.0, help="移动角度(度,默认 5)")
    args = parser.parse_args()

    validate_arm_end_type(args.end_type)
    if abs(args.delta) > MAX_DELTA_DEG:
        raise SystemExit(f"--delta {args.delta} 过大;本测试请保持 |delta| <= {MAX_DELTA_DEG}。")

    bus = MetalMotorsBus(
        args.can,
        metal_motors(args.end_type),
        urdf_path=default_urdf(args.end_type),
        arm_end_type=args.end_type,
    )
    bus.connect()
    try:
        bus.set_control_mode("nrt_joint")
        bus.enable_torque()

        start = bus.sync_read("Present_Position")
        print("当前姿态(度):")
        for name in JOINT_NAMES:
            print(f"  {name} = {start[name]:.2f}")

        target = dict(start)
        target[args.joint] = start[args.joint] + args.delta
        print(f"\n将移动 {args.joint}:{start[args.joint]:.2f} -> {target[args.joint]:.2f} 度,"
              f"随后归位。")

        ans = input("请扶住机械臂。输入 'yes' 开始移动:").strip().lower()
        if ans != "yes":
            print("已被用户取消,未发送任何运动指令。")
            return

        bus.sync_write("Goal_Position", target)
        time.sleep(1.5)
        after = bus.sync_read("Present_Position")
        moved = after[args.joint] - start[args.joint]
        print(f"移动后:{args.joint} = {after[args.joint]:.2f} 度(实际移动 {moved:+.2f},"
              f"指令 {args.delta:+.2f})")
        if abs(moved - args.delta) > 2.0:
            print("[注意] 实际移动量与指令不符 —— 请记录正负号/比例,反馈给维护者。")

        print("\n正在归位到起始姿态...")
        bus.sync_write("Goal_Position", start)
        time.sleep(1.5)
        print("阶段 1c 完成 —— 机械臂已回到起点。")
    finally:
        bus.disconnect()  # 关闭力矩
        print("已断开连接(力矩已关闭)。")


if __name__ == "__main__":
    main()
