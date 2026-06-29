#!/usr/bin/env bash
# 阶段 0 —— 无硬件冒烟测试。
#
# 运行单元测试套件,并确认 CLI 已识别 metal_follower / metal_leader。
# 不需要 ROS2、不需要 CAN、不需要硬件。请最先跑这个:如果它失败,
# 说明融合代码本身有问题,先别碰硬件。
#
# 用法:
#   conda activate MakerMods-lerobot
#   cd ~/makermods/lerobot
#   ./docs/metal/tests/00_smoke.sh
set -euo pipefail

# 根据脚本所在位置定位仓库根目录,这样在任何目录下都能运行。
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$REPO_ROOT"

PY="${PY:-python}"

echo "== 阶段 0:单元测试 =="
"$PY" -m pytest \
  tests/motors/test_metal.py \
  tests/robots/test_metal_follower.py \
  tests/teleoperators/test_metal_leader.py -q

echo
echo "== 阶段 0:CLI 注册检查 =="
"$PY" - <<'EOF'
import lerobot.scripts.lerobot_record  # noqa: F401  (导入即触发 robot/teleop 注册)
from lerobot.robots.config import RobotConfig
from lerobot.teleoperators.config import TeleoperatorConfig

robot_ok = "metal_follower" in RobotConfig.get_known_choices()
teleop_ok = "metal_leader" in TeleoperatorConfig.get_known_choices()
print("robot  metal_follower 已注册:", robot_ok)
print("teleop metal_leader  已注册:", teleop_ok)
assert robot_ok and teleop_ok, "metal 类型未在 CLI 中注册"
print("\n阶段 0 通过 —— 融合代码接线正确。")
EOF
