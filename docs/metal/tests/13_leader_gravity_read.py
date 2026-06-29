#!/usr/bin/env python
"""阶段 1d(单臂 leader)—— 重力补偿 + 实时读取关节角。

把单个 leader(主臂)设成重力补偿模式,然后循环刷新打印各关节角度,
让你**用手拖动主臂、在终端实时看到关节角变化**,松手时机械臂应停在原地
不下坠(这正是重力补偿要验证的)。单臂能做的最有意义的实机测试。

它本身不发送任何位置指令,只设控制模式 + 读取。

安全:重力补偿下机械臂可被徒手拖动并停在原地;但脚本退出会关闭力矩 ——
**退出(Ctrl-C)前请先扶住机械臂**,否则会自由下坠。第一次请放低、远离障碍物。

前置条件:conda activate MakerMods-lerobot(已自动配好 ROS 库路径)、
          ./docs/metal/start_can.sh 已把对应接口拉起。

用法:
  python docs/metal/tests/13_leader_gravity_read.py --can can1 --end-type 1
  python docs/metal/tests/13_leader_gravity_read.py --mock        # 无硬件,仅验证逻辑
"""

import argparse
import time

from lerobot.motors.metal import (
    JOINT_NAMES,
    MetalMotorsBus,
    arm_has_gripper,
    default_urdf,
    metal_motors,
    validate_arm_end_type,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--can", default="can1", help="SocketCAN 接口(默认 can1 = leader)")
    parser.add_argument("--end-type", type=int, default=1, help="arm_end_type 0/1/2/3")
    parser.add_argument("--hz", type=float, default=10.0, help="刷新频率(默认 10 Hz)")
    parser.add_argument("--mock", action="store_true", help="使用进程内 mock SDK(无硬件)")
    args = parser.parse_args()

    validate_arm_end_type(args.end_type)
    has_gripper = arm_has_gripper(args.end_type)
    period = 1.0 / args.hz if args.hz > 0 else 0.1

    bus = MetalMotorsBus(
        args.can,
        metal_motors(args.end_type),
        urdf_path=default_urdf(args.end_type),
        arm_end_type=args.end_type,
        mock=args.mock,
    )
    bus.connect()
    try:
        bus.set_control_mode("gravity")  # 重力补偿:可徒手拖动,松手停在原地
        print(f"接口 {args.can} 已进入重力补偿模式(arm_end_type={args.end_type})。")
        print("用手拖动主臂,观察下面的关节角实时变化。按 Ctrl-C 退出(退出前先扶住!)\n")

        cols = JOINT_NAMES + (["gripper"] if has_gripper else [])
        i = 0
        while True:
            pose = bus.sync_read("Present_Position")
            parts = []
            for name in cols:
                # 关节单位是度;gripper 是 lerobot 的 0-100 归一开合度(非毫米)
                suffix = "" if name == "gripper" else "°"
                parts.append(f"{name}={pose[name]:7.2f}{suffix}")
            # \r 原地刷新,单行滚动显示
            print("  " + "  ".join(parts), end="\r", flush=True)

            i += 1
            if args.mock and i >= 3:  # mock 模式只跑几次就退出,便于无硬件验证
                print("\n(mock)已读取 3 次,退出。")
                break
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C。")
    finally:
        print("正在禁用机械臂(力矩关闭)—— 请确保已扶稳!")
        bus.disconnect()  # 关闭力矩
        print("已断开连接。")


if __name__ == "__main__":
    main()
