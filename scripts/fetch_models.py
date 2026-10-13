#!/usr/bin/env python3
import os
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "ros2_ws/src/fwr_vision"))
from fwr_vision.models import ensure_models


if __name__ == "__main__":
    data_home = Path(os.environ.get("XDG_DATA_HOME", Path.home() / ".local/share"))
    target = Path(os.environ.get("FWR_MODELS_DIR", data_home / "four_wheel_robot/models"))
    for path in ensure_models(target):
        print(path)
