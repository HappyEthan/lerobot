#!/usr/bin/env python
"""阶段 2 —— 遥操闭环(双臂):leader 拖动,follower 实时跟随。

用我们自己的 lerobot 类做一个最小遥操循环(等价于 lerobot-teleoperate 的核心):
  leader(can1,重力补偿) --get_action()--> follower(can0,NRT 位置控制).send_action()

相比直接跑 CLI,这个脚本更透明:逐帧打印 leader 目标角,且默认带**每周期限幅**
(follower 每次最多朝 leader 移动 --max-step 度),所以即使一开始两臂姿态差很多,
follower 也是**平滑 ramp 过去**,不会第一帧暴冲。

⚠️ 需要两个臂 + 两个适配器,且已 `./start_can.sh setup`(分开插两个适配器)让
can0/can1 各对一个。只有一个臂时用 `--mock` 验证逻辑。

⚠️ 安全:leader 处于重力补偿,**进程异常退出会自由下坠**。第一次:扶住 leader、
放低、清空周围,随时准备断电。

用法:
  python docs/metal/tests/14_teleop.py --follower-can can0 --leader-can can1 --end-type 1
  python docs/metal/tests/14_teleop.py --mock --end-type 1     # 无硬件,仅验证逻辑
"""

import argparse
import time

from lerobot.robots.metal_follower import MetalFollower, MetalFollowerConfig
from lerobot.teleoperators.metal_leader import MetalLeader, MetalLeaderConfig


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--follower-can", default="can0", help="follower 接口(默认 can0)")
    parser.add_argument("--leader-can", default="can1", help="leader 接口(默认 can1)")
    parser.add_argument("--end-type", type=int, default=1, help="arm_end_type 0/1/2/3(两臂必须一致)")
    parser.add_argument("--hz", type=float, default=30.0, help="控制频率(默认 30 Hz)")
    parser.add_argument(
        "--max-step",
        type=float,
        default=5.0,
        help="每周期 follower 单关节最大移动量(度),防第一帧暴冲;0=不限幅(危险)",
    )
    parser.add_argument("--gripper-vr", type=int, default=10, help="夹爪速度比 1-10(越大越快,默认 10)")
    parser.add_argument("--mock", action="store_true", help="使用进程内 mock SDK(无硬件)")
    args = parser.parse_args()

    period = 1.0 / args.hz if args.hz > 0 else 0.03
    max_rel = args.max_step if args.max_step > 0 else None

    leader = MetalLeader(MetalLeaderConfig(can_id=args.leader_can, arm_end_type=args.end_type, mock=args.mock))
    follower = MetalFollower(
        MetalFollowerConfig(
            can_id=args.follower_can,
            arm_end_type=args.end_type,
            max_relative_target=max_rel,  # follower 内置安全限幅(每周期相对当前位姿的最大变化)
            gripper_velocity_ratio=args.gripper_vr,  # 夹爪更快,跟上关节
            mock=args.mock,
        )
    )

    # 录制要求 leader 的 action 与 follower 完全同构;不一致(如夹爪存在性不同)直接拦下。
    if leader.action_features != follower.action_features:
        raise SystemExit(
            "leader 与 follower 的 action_features 不一致(检查两边 --end-type 是否相同、夹爪是否都有):\n"
            f"  leader  : {list(leader.action_features)}\n"
            f"  follower: {list(follower.action_features)}"
        )

    print(f"limit/周期 = {max_rel}°(0/None 表示不限幅)  频率 = {args.hz} Hz")
    print("连接中... follower(NRT 位置控制)+ leader(重力补偿)")
    follower.connect()
    leader.connect()
    print("已连接。拖动 leader,follower 跟随。按 Ctrl-C 退出(退出前先扶住 leader!)\n")

    joint_keys = [k for k in follower.action_features if k.startswith("joint")]
    i = 0
    try:
        while True:
            action = leader.get_action()  # leader 当前角(度 / 夹爪 0-100)
            follower.send_action(action)  # 写 follower,内部按 max_relative_target 限幅
            line = "  ".join(f"{k.removesuffix('.pos')}={action[k]:7.2f}" for k in joint_keys)
            grip = action.get("gripper.pos")
            if grip is not None:
                line += f"  grip={grip:6.2f}"
            print("  leader→ " + line, end="\r", flush=True)

            i += 1
            if args.mock and i >= 3:
                print("\n(mock)已循环 3 次,退出。")
                break
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C。")
    finally:
        print("断开 follower(关力矩)与 leader(保持重力补偿,但请扶稳)...")
        follower.disconnect()
        leader.disconnect()
        print("已断开。")


if __name__ == "__main__":
    main()
