#!/usr/bin/env python3
"""将人工编写的 YAML 轨迹编译为双手 4x4 位姿 CSV。"""

from pathlib import Path
import sys


SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from trajectory_csv.compiler import main


if __name__ == "__main__":
    main()
