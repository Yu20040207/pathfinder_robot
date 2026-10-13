import hashlib
import os
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/fwr_vision"))
from fwr_vision.models import fetch_verified


def test_pinned_detector_can_run_inference():
    import cv2
    import numpy as np
    from fwr_vision.models import load_models

    base = Path(os.environ.get("FWR_MODELS_DIR", Path.home() / ".local/share/four_wheel_robot/models"))
    detector_path = base / "face_detection_yunet_2023mar.onnx"
    recognizer_path = base / "face_recognition_sface_2021dec.onnx"
    if not detector_path.is_file() or not recognizer_path.is_file():
        pytest.skip("Run scripts/fetch_models.py to enable inference smoke")
    detector, recognizer = load_models(detector_path, recognizer_path)
    detector.detect(np.zeros((320, 320, 3), np.uint8))
    feature = recognizer.feature(np.zeros((112, 112, 3), np.uint8))
    assert feature.size > 0


def test_verified_cache_is_reused(tmp_path):
    target = tmp_path / "model.onnx"
    target.write_bytes(b"real model")
    digest = hashlib.sha256(b"real model").hexdigest()
    assert fetch_verified("unused", target, digest) == target


def test_bad_download_does_not_replace_cached_file(tmp_path):
    target = tmp_path / "model.onnx"
    target.write_bytes(b"bad cache")
    digest = hashlib.sha256(b"expected model").hexdigest()
    with pytest.raises(ValueError, match="SHA-256"):
        fetch_verified("data:application/octet-stream;base64,YmFkIGRvd25sb2Fk", target, digest)
    assert target.read_bytes() == b"bad cache"
    assert list(tmp_path.glob("*.tmp")) == []
