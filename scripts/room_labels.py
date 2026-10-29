#!/usr/bin/env python3
"""Record named room labels from RViz Publish Point clicks.

Usage (after sourcing ROS 2 and the workspace):
  python3 scripts/room_labels.py
  python3 scripts/room_labels.py --output ros2_ws/src/fwr_bringup/maps/rooms.yaml

In RViz:
  1. Set Fixed Frame to `map`.
  2. Select the Publish Point tool.
  3. Click the room center.
  4. Enter the room number, name and optional facing yaw in this terminal.

The YAML is written after every room. Labels are also published as a
transient-local MarkerArray on /room_labels so RViz keeps them visible.
"""

from __future__ import annotations

import argparse
import math
import queue
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


DEFAULT_OUTPUT = Path(__file__).resolve().parents[1] / (
    "ros2_ws/src/fwr_bringup/maps/rooms.yaml"
)


class RoomLabels(Node):
    def __init__(self, output: Path, frame: str, markers_topic: str, clicked_topic: str):
        super().__init__("room_labels")
        self.output = output
        self.frame = frame
        self.pending: queue.Queue[tuple[float, float]] = queue.Queue()
        self.rooms: list[dict] = []

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.marker_pub = self.create_publisher(MarkerArray, markers_topic, qos)
        self.create_subscription(PointStamped, clicked_topic, self._on_click, 10)
        self._load()
        self.publish_markers()
        # Republish periodically so RViz panels opened later still receive the
        # transient-local markers.
        self.create_timer(1.0, self.publish_markers)

    def _load(self) -> None:
        if not self.output.exists():
            self.rooms = []
            return
        try:
            data = yaml.safe_load(self.output.read_text(encoding="utf-8")) or {}
        except (OSError, yaml.YAMLError) as error:
            raise SystemExit(f"无法读取 {self.output}: {error}")
        rooms = data.get("rooms", [])
        if not isinstance(rooms, list):
            raise SystemExit(f"{self.output} 的 rooms 必须是列表")
        self.rooms = [room for room in rooms if isinstance(room, dict) and "id" in room]
        print(f"已加载 {len(self.rooms)} 个房间：{self.output}")

    def _save(self) -> None:
        self.output.parent.mkdir(parents=True, exist_ok=True)
        data = {"frame_id": self.frame, "rooms": self.rooms}
        text = yaml.safe_dump(data, allow_unicode=True, sort_keys=False)
        tmp = self.output.with_suffix(self.output.suffix + ".tmp")
        tmp.write_text(text, encoding="utf-8")
        tmp.replace(self.output)

    def _on_click(self, msg: PointStamped) -> None:
        if msg.header.frame_id != self.frame:
            self.get_logger().warning(
                f"忽略 frame_id={msg.header.frame_id!r} 的点击；请把 RViz Fixed Frame 设为 {self.frame}"
            )
            return
        self.pending.put((float(msg.point.x), float(msg.point.y)))

    def publish_markers(self) -> None:
        array = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.frame
        clear.action = Marker.DELETEALL
        array.markers.append(clear)

        for room in self.rooms:
            try:
                room_id = int(room["id"])
                x = float(room["x"])
                y = float(room["y"])
            except (KeyError, TypeError, ValueError):
                continue
            name = str(room.get("name", f"房间{room_id}"))

            dot = Marker()
            dot.header.frame_id = self.frame
            dot.header.stamp = self.get_clock().now().to_msg()
            dot.ns = "room_labels"
            dot.id = room_id * 2
            dot.type = Marker.SPHERE
            dot.action = Marker.ADD
            dot.pose.position.x = x
            dot.pose.position.y = y
            dot.pose.position.z = 0.08
            dot.pose.orientation.w = 1.0
            dot.scale.x = dot.scale.y = dot.scale.z = 0.18
            dot.color.r, dot.color.g, dot.color.b, dot.color.a = 0.1, 0.4, 1.0, 0.7
            array.markers.append(dot)

            text = Marker()
            text.header.frame_id = self.frame
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = "room_labels"
            text.id = room_id * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 0.55
            text.pose.orientation.w = 1.0
            text.scale.z = 0.35
            text.color.r, text.color.g, text.color.b, text.color.a = 0.1, 1.0, 0.2, 1.0
            text.text = f"{room_id} {name}"
            array.markers.append(text)

        self.marker_pub.publish(array)

    def record(self, x: float, y: float) -> None:
        print(f"\n点击坐标：x={x:.3f}  y={y:.3f}  (frame={self.frame})")
        raw_id = input("房间编号（整数，直接回车跳过）: ").strip()
        if not raw_id:
            return
        try:
            room_id = int(raw_id)
        except ValueError:
            print("编号必须是整数，已跳过。")
            return

        existing = next((room for room in self.rooms if int(room.get("id", -1)) == room_id), None)
        default_name = existing.get("name") if existing else f"房间{room_id}"
        raw_name = input(f"房间名称（回车={default_name}）: ").strip()
        name = raw_name or str(default_name)

        raw_yaw = input("朝向 yaw（度，可选，0=地图 +x；回车=0）: ").strip()
        try:
            yaw_deg = float(raw_yaw) if raw_yaw else 0.0
        except ValueError:
            print("yaw 不是数字，按 0 处理。")
            yaw_deg = 0.0

        room = {
            "id": room_id,
            "name": name,
            "x": round(x, 3),
            "y": round(y, 3),
            "yaw": round(math.radians(yaw_deg), 4),
        }
        if existing is not None:
            self.rooms.remove(existing)
            print(f"覆盖编号 {room_id} 的旧标签。")
        self.rooms.append(room)
        self.rooms.sort(key=lambda item: int(item["id"]))
        self._save()
        self.publish_markers()
        print(f"已保存 {room_id} {name} -> {self.output}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--frame", default="map")
    parser.add_argument("--markers-topic", default="/room_labels")
    parser.add_argument("--clicked-topic", default="/clicked_point")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = RoomLabels(args.output, args.frame, args.markers_topic, args.clicked_topic)
    print(__doc__)
    print(f"输出文件：{args.output}")
    print("在 RViz 选择 Publish Point，点击房间中心；Ctrl-C 退出。")
    try:
        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except (KeyboardInterrupt, ExternalShutdownException):
                break
            except Exception:
                # rclpy can raise RCLError while a Ctrl-C tears down the context.
                if rclpy.ok():
                    raise
                break
            try:
                x, y = node.pending.get_nowait()
            except queue.Empty:
                continue
            node.record(x, y)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
