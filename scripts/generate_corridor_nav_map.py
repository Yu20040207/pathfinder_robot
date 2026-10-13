#!/usr/bin/env python3
"""Generate the static occupancy map matching corridor_c1.sdf geometry."""

from pathlib import Path


RESOLUTION = 0.05
ORIGIN_X, ORIGIN_Y = -3.0, -2.0
WIDTH, HEIGHT = 240, 80


def occupied(x: float, y: float) -> bool:
    # The corridor walls' inner faces are at y=+/-1.29 m.
    walls = abs(y) >= 1.29 or x >= 8.84
    obstacle = 2.925 <= x <= 3.475 and -0.725 <= y <= -0.175
    return walls or obstacle


def main() -> None:
    target = Path(__file__).resolve().parents[1] / "ros2_ws/src/fwr_bringup/maps/corridor_nav.pgm"
    target.parent.mkdir(parents=True, exist_ok=True)
    pixels = bytearray()
    # PGM top row corresponds to highest world Y; ROS map origin is bottom left.
    for row in range(HEIGHT - 1, -1, -1):
        y = ORIGIN_Y + (row + 0.5) * RESOLUTION
        for col in range(WIDTH):
            x = ORIGIN_X + (col + 0.5) * RESOLUTION
            pixels.append(0 if occupied(x, y) else 254)
    target.write_bytes(f"P5\n{WIDTH} {HEIGHT}\n255\n".encode() + pixels)
    print(target)


if __name__ == "__main__":
    main()
