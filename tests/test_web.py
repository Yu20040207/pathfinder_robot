from io import BytesIO
from pathlib import Path
import re
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np

root = Path(__file__).resolve().parents[1]
for package in ("fwr_data", "fwr_vision", "fwr_attendance"):
    sys.path.insert(0, str(root / f"ros2_ws/src/{package}"))

from fwr_data.store import AttendanceStore, EnrollmentSample
from fwr_attendance.web import create_app
from fwr_attendance.enrollment import extract_samples
from fwr_attendance.status import StatusFile


class DemoEngine:
    def extract_one(self, bgr):
        if bgr.mean() < 2:
            raise ValueError("Enrollment image must contain exactly one face")
        return EnrollmentSample(b"jpeg", (1.0, 0.0))


def picture(value=100):
    success, encoded = cv2.imencode(".jpg", np.full((64, 64, 3), value, np.uint8))
    assert success
    return encoded.tobytes()


def token(client):
    page = client.get("/").data.decode()
    return re.search(r'name="csrf_token" value="([a-f0-9]+)"', page).group(1)


def test_enroll_list_delete_and_export(tmp_path):
    store = AttendanceStore(tmp_path / "db.sqlite3")
    client = create_app(store, DemoEngine()).test_client()
    csrf = token(client)
    response = client.post("/people", data={"person_id": "P1", "display_name": "张三",
                                           "csrf_token": csrf,
                                           "images": (BytesIO(picture()), "face.jpg")},
                           content_type="multipart/form-data")
    assert response.status_code == 303
    assert b"P1" in client.get("/").data
    assert client.post("/people", data={"person_id": "P1", "display_name": "Duplicate",
                                             "csrf_token": csrf,
                                             "images": (BytesIO(picture()), "face.jpg")},
                       content_type="multipart/form-data").status_code == 400
    assert client.get("/events.csv").status_code == 200
    assert client.post("/people/P1/delete", data={"csrf_token": csrf}).status_code == 303
    assert store.people() == []


def test_bad_upload_does_not_enroll(tmp_path):
    store = AttendanceStore(tmp_path / "db.sqlite3")
    client = create_app(store, DemoEngine()).test_client()
    response = client.post("/people", data={"person_id": "P2", "display_name": "A",
                                           "csrf_token": token(client),
                                           "images": (BytesIO(b"not an image"), "bad.jpg")},
                           content_type="multipart/form-data")
    assert response.status_code == 400
    assert store.people() == []


def test_cross_origin_form_without_session_token_cannot_mutate(tmp_path):
    store = AttendanceStore(tmp_path / "db.sqlite3")
    client = create_app(store, DemoEngine()).test_client()
    response = client.post("/people", data={"person_id": "P3", "display_name": "A",
                                           "images": (BytesIO(picture()), "face.jpg")},
                           content_type="multipart/form-data")
    assert response.status_code == 403
    assert store.people() == []


def test_compressed_oversize_image_is_rejected_before_decode():
    success, encoded = cv2.imencode(".png", np.zeros((5000, 5000, 3), np.uint8))
    assert success and len(encoded) < 5 * 1024 * 1024
    try:
        extract_samples([encoded.tobytes()], DemoEngine())
        assert False, "oversize image was accepted"
    except ValueError as error:
        assert "dimensions" in str(error)


def test_parallel_uploads_do_not_share_inference_state(tmp_path):
    class SharedEngine(DemoEngine):
        active = 0
        maximum = 0
        lock = threading.Lock()

        def extract_one(self, bgr):
            with self.lock:
                self.active += 1
                self.maximum = max(self.maximum, self.active)
            time.sleep(0.05)
            with self.lock:
                self.active -= 1
            return super().extract_one(bgr)

    engine = SharedEngine()
    app = create_app(AttendanceStore(tmp_path / "db.sqlite3"), engine)
    barrier = threading.Barrier(2)

    def submit(pid):
        client = app.test_client()
        csrf = token(client)
        barrier.wait()
        return client.post("/people", data={"person_id": pid, "display_name": pid,
                                            "csrf_token": csrf,
                                            "images": (BytesIO(picture()), "face.jpg")},
                           content_type="multipart/form-data").status_code

    with ThreadPoolExecutor(max_workers=2) as pool:
        responses = list(pool.map(submit, ("A", "B")))
    assert responses == [303, 303]
    assert engine.maximum == 1


def test_page_displays_persisted_attendance_error(tmp_path):
    store = AttendanceStore(tmp_path / "db.sqlite3")
    StatusFile(tmp_path / "attendance-status.json").set_error("disk write failed")
    page = create_app(store, DemoEngine()).test_client().get("/")
    assert page.status_code == 200
    assert b"disk write failed" in page.data
