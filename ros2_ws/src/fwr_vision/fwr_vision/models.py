"""Pinned OpenCV Zoo models, verified before use."""

import hashlib
import os
from pathlib import Path
import tempfile
from urllib.request import urlopen


COMMIT = "47534e27c9851bb1128ccc0102f1145e27f23f98"
BASE = f"https://media.githubusercontent.com/media/opencv/opencv_zoo/{COMMIT}/models"
MODELS = (
    ("face_detection_yunet/face_detection_yunet_2023mar.onnx", "8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4"),
    ("face_recognition_sface/face_recognition_sface_2021dec.onnx", "0ba9fbfa01b5270c96627c4ef784da859931e02f04419c829e83484087c34e79"),
)


def _digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def fetch_verified(url: str, target: Path, sha256: str) -> Path:
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.is_file() and _digest(target) == sha256:
        return target
    fd, name = tempfile.mkstemp(prefix="download-", suffix=".tmp", dir=target.parent)
    try:
        with os.fdopen(fd, "wb") as output, urlopen(url, timeout=60) as source:
            for block in iter(lambda: source.read(1024 * 1024), b""):
                output.write(block)
        temporary = Path(name)
        if _digest(temporary) != sha256:
            raise ValueError(f"SHA-256 mismatch for {target.name}")
        temporary.replace(target)
    finally:
        Path(name).unlink(missing_ok=True)
    return target


def ensure_models(target_dir: Path) -> tuple[Path, Path]:
    paths = []
    for relative, digest in MODELS:
        name = Path(relative).name
        paths.append(fetch_verified(f"{BASE}/{relative}", target_dir / name, digest))
    return paths[0], paths[1]


def load_models(detector_path: Path, recognizer_path: Path):
    import cv2
    import numpy as np

    for path in (detector_path, recognizer_path):
        if not path.is_file():
            raise FileNotFoundError(f"Missing model {path}; run scripts/fetch_models.py")
    detector = cv2.FaceDetectorYN.create(str(detector_path), "", (320, 320))
    recognizer = cv2.FaceRecognizerSF.create(str(recognizer_path), "")
    try:
        detector.detect(np.zeros((320, 320, 3), dtype=np.uint8))
    except cv2.error as error:
        raise RuntimeError(
            f"YuNet inference failed with OpenCV {cv2.__version__}; use the project virtual environment"
        ) from error
    return detector, recognizer
