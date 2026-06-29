#!/usr/bin/env python
"""阶段 1a —— 直接探测 metal_sdk(单臂,不产生任何运动)。

绕过 lerobot 封装层,直接和 SDK 对话,确认融合所依赖的核心假设:
  - GetJointPosition() 的长度与 arm_end_type 一致(类型 0 为 6,其余为 7)
  - 夹爪值(下标 6)读出来像毫米(0-80),而不是弧度
  - 当应有夹爪时,GetJointNames() 里列出了 gripper

本脚本只“读”,读完即禁用机械臂,绝不发送任何运动指令。

前置条件(见 docs/metal/README.md):
  source /opt/ros/humble/setup.bash      # metal_sdk 链接了 ROS2 的 .so
  conda activate MakerMods-lerobot
  ./docs/metal/start_can.sh              # 拉起 can0 / can1

用法:
  python docs/metal/tests/10_sdk_raw.py --can can0 --end-type 1
"""

import argparse
import time

from lerobot.motors.metal import arm_position_dim, default_urdf, validate_arm_end_type


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--can", default="can0", help="SocketCAN 接口(默认 can0 = follower)")
    parser.add_argument(
        "--end-type",
        type=int,
        default=1,
        help="arm_end_type:0=无, 1=夹爪, 2=示教器, 3=夹爪+示教器",
    )
    args = parser.parse_args()
    validate_arm_end_type(args.end_type)

    # 延迟导入:必须先 `source /opt/ros/humble/setup.bash`。
    from metal_sdk import MetalSDKInterface

    expected_dim = arm_position_dim(args.end_type)
    urdf = default_urdf(args.end_type)
    print(f"can={args.can}  arm_end_type={args.end_type}  期望维度={expected_dim}")
    print(f"urdf={urdf}\n")

    arm = MetalSDKInterface(args.can, urdf, args.end_type, True)
    if not arm.Init():
        raise SystemExit(f"{args.can}:MetalSDKInterface.Init() 失败 —— CAN 总线起来了吗?")
    try:
        time.sleep(1.0)  # 等第一批 CAN 帧到达
        names = arm.GetJointNames()
        pos = arm.GetJointPosition()

        print("GetJointNames()    =", names)
        print("GetJointPosition() =", [round(x, 3) for x in pos])
        print("实际维度           =", len(pos))

        ok = True
        if len(pos) != expected_dim:
            ok = False
            print(f"\n[失败] 维度 {len(pos)} != 期望 {expected_dim}(arm_end_type={args.end_type})。")
            print("       配置的末端执行器与硬件不匹配。")
        else:
            print(f"\n[ OK ] 维度与 arm_end_type={args.end_type} 匹配。")

        if expected_dim == 7:
            gripper = pos[6]
            looks_like_mm = 0.0 <= gripper <= 80.0
            tag = "OK" if looks_like_mm else "注意"
            print(f"[{tag:>4}] 夹爪值(下标 6)= {gripper:.3f} "
                  f"({'像毫米 0-80' if looks_like_mm else '不在 0-80 mm 范围内'})")

        print("\n阶段 1a 通过。" if ok else "\n阶段 1a 失败 —— 继续之前先修正 --end-type。")
    finally:
        arm.SetEnableArm(False)  # 务必释放机械臂
        print("机械臂已禁用(SetEnableArm(False))。")


if __name__ == "__main__":
    main()
