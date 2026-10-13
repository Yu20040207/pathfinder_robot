"""Camera independent face embedding and matching."""

from dataclasses import dataclass
import math
from typing import Sequence

import cv2
import numpy as np

from fwr_data.store import EnrollmentSample, Template


@dataclass(frozen=True)
class FaceResult:
    person_id: str
    display_name: str
    recognized: bool
    score: float
    box: tuple[int, int, int, int]
    distance_m: float | None = None


def _cosine(a: tuple[float, ...], b: tuple[float, ...]) -> float:
    if len(a) != len(b):
        return -1.0
    va, vb = np.asarray(a, dtype=np.float32), np.asarray(b, dtype=np.float32)
    norm = float(np.linalg.norm(va) * np.linalg.norm(vb))
    return float(np.dot(va, vb) / norm) if norm > 0 else -1.0


class FaceEngine:
    def __init__(self, detector, recognizer, threshold: float = 0.363, margin: float = 0.05,
                 blur_variance_min: float = 25.0):
        self.detector = detector
        self.recognizer = recognizer
        self.threshold = threshold
        self.margin = margin
        self.blur_variance_min = blur_variance_min

    def _sharp_enough(self, bgr: np.ndarray, face) -> bool:
        x, y, w, h = (int(round(v)) for v in face[:4])
        crop = bgr[max(0, y):min(bgr.shape[0], y + h), max(0, x):min(bgr.shape[1], x + w)]
        if crop.size == 0:
            return False
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        return float(cv2.Laplacian(gray, cv2.CV_64F).var()) >= self.blur_variance_min

    def _faces(self, bgr: np.ndarray):
        if bgr.ndim != 3 or bgr.shape[2] != 3 or min(bgr.shape[:2]) < 32:
            raise ValueError("Invalid BGR image")
        self.detector.setInputSize((bgr.shape[1], bgr.shape[0]))
        _, faces = self.detector.detect(bgr)
        return [] if faces is None else faces

    def _feature(self, bgr: np.ndarray, face) -> tuple[float, ...]:
        aligned = self.recognizer.alignCrop(bgr, face)
        values = np.asarray(self.recognizer.feature(aligned), dtype=np.float32).flatten()
        if values.size == 0 or not np.all(np.isfinite(values)):
            raise ValueError("Face feature extraction failed")
        return tuple(float(value) for value in values)

    def extract_one(self, bgr: np.ndarray) -> EnrollmentSample:
        faces = self._faces(bgr)
        if len(faces) != 1:
            raise ValueError("Enrollment image must contain exactly one face")
        if faces[0][2] < 30 or faces[0][3] < 30:
            raise ValueError("Face is too small")
        if not self._sharp_enough(bgr, faces[0]):
            raise ValueError("Face image is too blurry")
        feature = self._feature(bgr, faces[0])
        success, encoded = cv2.imencode(".jpg", bgr)
        if not success:
            raise ValueError("Could not encode enrollment image")
        return EnrollmentSample(encoded.tobytes(), feature)

    def analyze(self, bgr: np.ndarray, templates: Sequence[Template]) -> list[FaceResult]:
        output = []
        for face in self._faces(bgr):
            x, y, w, h = (int(round(v)) for v in face[:4])
            box = (x, y, w, h)
            if w < 30 or h < 30:
                continue
            if not self._sharp_enough(bgr, face):
                output.append(FaceResult("", "", False, -1.0, box))
                continue
            feature = self._feature(bgr, face)
            per_person = {}
            for item in templates:
                score = _cosine(feature, item.embedding)
                previous = per_person.get(item.person_id)
                if previous is None or score > previous[0]:
                    per_person[item.person_id] = (score, item.display_name)
            ranked = sorted(((score, pid, name) for pid, (score, name) in per_person.items()), reverse=True)
            score = ranked[0][0] if ranked else -1.0
            recognized = bool(ranked and score >= self.threshold and
                              (len(ranked) == 1 or score - ranked[1][0] >= self.margin))
            output.append(FaceResult(ranked[0][1] if recognized else "",
                                     ranked[0][2] if recognized else "", recognized, score, box))
        return output
