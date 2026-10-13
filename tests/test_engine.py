from pathlib import Path
import sys

import numpy as np
import pytest

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "ros2_ws/src/fwr_data"))
sys.path.insert(0, str(root / "ros2_ws/src/fwr_vision"))
from fwr_data.store import Template
from fwr_vision.engine import FaceEngine


class Detector:
    def __init__(self, faces):
        self.faces = faces

    def setInputSize(self, size):
        self.size = size

    def detect(self, image):
        return True, self.faces


class Recognizer:
    def alignCrop(self, image, face):
        return image

    def feature(self, image):
        return np.array([[1.0, 0.0]], dtype=np.float32)


FACE = np.array([[10, 20, 40, 50] + [0] * 11], dtype=np.float32)


def textured_image():
    pattern = (np.indices((100, 100)).sum(axis=0) % 2 * 255).astype(np.uint8)
    return np.repeat(pattern[:, :, None], 3, axis=2)


def test_known_and_unknown_faces():
    engine = FaceEngine(Detector(FACE), Recognizer())
    image = textured_image()
    known = engine.analyze(image, [Template("P1", "A", (1.0, 0.0))])[0]
    assert (known.person_id, known.recognized, known.box) == ("P1", True, (10, 20, 40, 50))
    unknown = engine.analyze(image, [Template("P2", "B", (0.0, 1.0))])[0]
    assert unknown.recognized is False
    assert unknown.person_id == ""


def test_ambiguous_identity_is_unknown():
    engine = FaceEngine(Detector(FACE), Recognizer())
    result = engine.analyze(textured_image(), [
        Template("P1", "A", (1.0, 0.0)), Template("P2", "B", (0.999, 0.001))
    ])[0]
    assert result.recognized is False


def test_enrollment_requires_exactly_one_face():
    engine = FaceEngine(Detector(None), Recognizer())
    with pytest.raises(ValueError, match="one face"):
        engine.extract_one(np.zeros((100, 100, 3), np.uint8))


def test_blurred_face_cannot_be_enrolled_or_recognized():
    engine = FaceEngine(Detector(FACE), Recognizer())
    blurry = np.full((100, 100, 3), 100, np.uint8)
    with pytest.raises(ValueError, match="blurry"):
        engine.extract_one(blurry)
    result = engine.analyze(blurry, [Template("P1", "A", (1.0, 0.0))])[0]
    assert result.recognized is False
