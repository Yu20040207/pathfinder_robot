#!/usr/bin/env python3
"""Build a portable rounded chassis mesh and a local-path RViz URDF."""

from pathlib import Path
import math
import struct


ROOT = Path(__file__).resolve().parents[1]
PREVIEW = ROOT / "docs/hardware/preview"
SIZE = (0.52, 0.35, 0.16)
RADIUS = 0.025


def samples(half):
    inner = half - RADIUS
    return sorted(
        set(
            [-inner, inner]
            + [-half + RADIUS * i / 8 for i in range(9)]
            + [inner + RADIUS * i / 8 for i in range(9)]
        )
    )


def rounded(point):
    inner = [half - RADIUS for half in (dimension / 2 for dimension in SIZE)]
    clamped = [max(-bound, min(bound, value)) for value, bound in zip(point, inner)]
    delta = [value - value0 for value, value0 in zip(point, clamped)]
    length = math.sqrt(sum(value * value for value in delta))
    return tuple(value0 + RADIUS * value / length for value0, value in zip(clamped, delta))


def triangles():
    for axis in range(3):
        other = [index for index in range(3) if index != axis]
        u_values = samples(SIZE[other[0]] / 2)
        v_values = samples(SIZE[other[1]] / 2)
        for sign in (-1, 1):
            def vertex(u, v):
                point = [0.0, 0.0, 0.0]
                point[axis] = sign * SIZE[axis] / 2
                point[other[0]] = u
                point[other[1]] = v
                return rounded(point)

            for i in range(len(u_values) - 1):
                for j in range(len(v_values) - 1):
                    a = vertex(u_values[i], v_values[j])
                    b = vertex(u_values[i + 1], v_values[j])
                    c = vertex(u_values[i + 1], v_values[j + 1])
                    d = vertex(u_values[i], v_values[j + 1])
                    yield a, b, c
                    yield a, c, d


def facet(vertices):
    a, b, c = vertices
    ab = [b[k] - a[k] for k in range(3)]
    ac = [c[k] - a[k] for k in range(3)]
    normal = (
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    )
    midpoint = [sum(vertex[k] for vertex in vertices) / 3 for k in range(3)]
    if sum(normal[k] * midpoint[k] for k in range(3)) < 0:
        b, c = c, b
        normal = tuple(-value for value in normal)
    magnitude = math.sqrt(sum(value * value for value in normal))
    normal = tuple(value / magnitude for value in normal)
    return struct.pack("<12fH", *(normal + a + b + c), 0)


def main():
    mesh = PREVIEW / "fwr_chassis_rounded.stl"
    faces = list(triangles())
    with mesh.open("wb") as output:
        output.write(b"FWR rounded chassis preview".ljust(80, b"\0"))
        output.write(struct.pack("<I", len(faces)))
        for face in faces:
            output.write(facet(face))

    template = (PREVIEW / "fwr_chassis_preview.urdf.in").read_text()
    (PREVIEW / "fwr_chassis_preview.urdf").write_text(
        template.replace("__CHASSIS_MESH_URI__", mesh.as_uri())
    )
    print(f"Generated {mesh} ({len(faces)} triangles)")


if __name__ == "__main__":
    main()
