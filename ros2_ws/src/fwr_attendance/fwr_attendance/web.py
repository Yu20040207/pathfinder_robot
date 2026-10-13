"""Local only enrollment and attendance browser UI."""

import csv
from datetime import datetime
import hmac
from io import StringIO
import os
from pathlib import Path
import secrets
import sqlite3
import threading

from flask import Flask, Response, abort, redirect, render_template, request, session, url_for

from fwr_data.store import AttendanceStore
from fwr_attendance.enrollment import extract_samples
from fwr_attendance.status import StatusFile
from fwr_vision.engine import FaceEngine
from fwr_vision.models import load_models


def _safe_csv(value: str) -> str:
    return "'" + value if value and value[0] in "=+-@" else value


def create_app(store: AttendanceStore, face_engine) -> Flask:
    app = Flask(__name__)
    inference_lock = threading.Lock()
    status = StatusFile(store.path.parent / "attendance-status.json")
    app.secret_key = secrets.token_bytes(32)
    app.config["MAX_CONTENT_LENGTH"] = 27 * 1024 * 1024

    @app.before_request
    def check_csrf():
        if request.method == "POST":
            expected = session.get("csrf_token", "")
            supplied = request.form.get("csrf_token", "")
            if not expected or not hmac.compare_digest(expected, supplied):
                abort(403)

    def page(error=None, code=200):
        if "csrf_token" not in session:
            session["csrf_token"] = secrets.token_hex(16)
        try:
            events = store.events()
            people = store.people()
        except sqlite3.Error as db_error:
            events, people = [], []
            error = f"Database unavailable: {db_error}"
            code = 503
        local_events = [
            (entry, datetime.fromisoformat(entry.occurred_at).astimezone().strftime("%Y-%m-%d %H:%M:%S %Z"))
            for entry in events
        ]
        return render_template("index.html", people=people, events=local_events,
                               error=error or status.get_error(), csrf_token=session["csrf_token"]), code

    @app.get("/")
    def index():
        return page()

    @app.post("/people")
    def enroll():
        try:
            uploads = request.files.getlist("images")
            raw_images = [item.read() for item in uploads]
            with inference_lock:
                samples = extract_samples(raw_images, face_engine)
            store.enroll(request.form.get("person_id", ""), request.form.get("display_name", ""), samples)
        except ValueError as error:
            return page(str(error), 400)
        except sqlite3.Error as error:
            status.set_error(f"Database unavailable: {error}")
            return page("Database unavailable", 503)
        return redirect(url_for("index"), code=303)

    @app.post("/people/<person_id>/delete")
    def delete_person(person_id):
        try:
            deleted = store.delete_person(person_id)
        except sqlite3.Error as error:
            status.set_error(f"Database unavailable: {error}")
            return page("Database unavailable", 503)
        if not deleted:
            return page("Person not found", 404)
        return redirect(url_for("index"), code=303)

    @app.get("/events.csv")
    def export_events():
        output = StringIO()
        writer = csv.writer(output)
        writer.writerow(["id", "person_id", "display_name", "occurred_at_utc", "score", "device_id"])
        for event in store.events():
            writer.writerow([event.id, _safe_csv(event.person_id), _safe_csv(event.display_name),
                             event.occurred_at, event.score, _safe_csv(event.device_id)])
        return Response("\ufeff" + output.getvalue(), mimetype="text/csv; charset=utf-8",
                        headers={"Content-Disposition": "attachment; filename=attendance.csv"})

    @app.errorhandler(413)
    def too_large(_error):
        return page("Upload is too large", 413)

    return app


def main():
    base = Path(os.environ.get("XDG_DATA_HOME", Path.home() / ".local/share")) / "four_wheel_robot"
    model_dir = Path(os.environ.get("FWR_MODELS_DIR", base / "models"))
    detector, recognizer = load_models(model_dir / "face_detection_yunet_2023mar.onnx",
                                       model_dir / "face_recognition_sface_2021dec.onnx")
    store = AttendanceStore(Path(os.environ.get("FWR_DB_PATH", base / "attendance.sqlite3")))
    create_app(store, FaceEngine(detector, recognizer)).run(host="127.0.0.1", port=8765, debug=False)


if __name__ == "__main__":
    main()
