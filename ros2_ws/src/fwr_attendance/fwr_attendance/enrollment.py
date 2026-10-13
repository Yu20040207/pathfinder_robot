"""Validate upload bytes before adding a person to the database."""

from typing import Sequence
from io import BytesIO

import cv2
import numpy as np
from PIL import Image, UnidentifiedImageError

from fwr_data.store import EnrollmentSample


MAX_IMAGE_BYTES = 5 * 1024 * 1024
MAX_IMAGE_PIXELS = 4_000_000


def extract_samples(images: Sequence[bytes], face_engine) -> list[EnrollmentSample]:
    if not 1 <= len(images) <= 5:
        raise ValueError("Please provide 1 to 5 images")
    samples = []
    for raw in images:
        if not raw or len(raw) > MAX_IMAGE_BYTES:
            raise ValueError("Each image must be 1 byte to 5 MiB")
        if not (raw.startswith(b"\xff\xd8\xff") or raw.startswith(b"\x89PNG\r\n\x1a\n")):
            raise ValueError("Only JPEG and PNG images are accepted")
        try:
            with Image.open(BytesIO(raw)) as image:
                width, height = image.size
                if width <= 0 or height <= 0 or width * height > MAX_IMAGE_PIXELS:
                    raise ValueError("Image dimensions are too large")
                image.verify()
            bgr = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        except (UnidentifiedImageError, OSError, cv2.error, Image.DecompressionBombError) as error:
            raise ValueError("Image could not be decoded") from error
        if bgr is None:
            raise ValueError("Image could not be decoded")
        samples.append(face_engine.extract_one(bgr))
    return samples
