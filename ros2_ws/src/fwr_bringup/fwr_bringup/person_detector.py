"""YOLO11n 人体检测 + 选中目标跟踪（跟随模式用）。

话题
----
订阅 ``/robot/camera/color/image_raw/compressed``  sensor_msgs/CompressedImage
订阅 ``/robot/vision/select``                     std_msgs/String  ``{"id": 3}``／``{"id": -1}``
发布 ``/robot/vision/persons``                    std_msgs/String  JSON（**归一化坐标** 0~1）

JSON 形状::

    {"w": 640, "h": 480, "t": 1795250000.1, "infer_ms": 21.4,
     "persons": [{"id": 1, "x": .31, "y": .42, "w": .12, "h": .38, "s": .87}],
     "sel": 1, "tracking": true, "lost_s": 0.0}

实测（i5-13500H，onnxruntime CPU 4 线程，yolo11n）
------------------------------------------------
| 输入 | 推理 | 上限 |
| --- | --- | --- |
| 320 | 13 ms | 75 fps |
| 416 | 21 ms | 46 fps |
| 640 | 44 ms | 23 fps |

节点按 ``detect_hz``（默认 8）限频，实际占单核 ~17%。

设计要点（踩过坑，别改回去）
------------------------
* **必须用 onnxruntime**：OpenCV 4.10 的 ONNX 解析器在 YOLO11 的 `Concat` 节点上直接
  报 `Incorrect size of input array`，模型加载不起来（2026-11-21 实测）。
* 检测在**独立线程**里跑：解码 + 推理 ~25 ms，放回调里会把 rclpy 执行器堵住，
  图像订阅随后开始丢帧。
* 队列只留 1 帧：宁可丢旧帧也不排队（排队 = 延迟越积越大）。
* 选中目标用 IoU/中心距贪心匹配维持 id（见 `vision_boxes.assign_ids`）：本机 cv2 是
  基础版，没有 `cv2.legacy` 的 CSRT/KCF；做跟随控制时再上真正的视觉跟踪器。
* 坐标一律归一化：手机端画面被服务端缩到 480x360，用像素坐标会错位。
"""

from __future__ import annotations

import json
import os
import queue
import threading
import time

import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from fwr_bringup.vision_boxes import (assign_ids, filter_persons,
                                     parse_pose_output, parse_yolo_output,
                                     pick_selected, to_payload)

MODEL_CANDIDATES = (
    'models/yolo11n.onnx',
    '/home/manbaout/Company_Project/Four-wheel-robot/models/yolo11n.onnx',
)
POSE_CANDIDATES = (
    'models/yolo11n-pose.onnx',
    '/home/manbaout/Company_Project/Four-wheel-robot/models/yolo11n-pose.onnx',
)


def load_session(path: str) -> 'ort.InferenceSession':
    so = ort.SessionOptions()
    so.intra_op_num_threads = 2
    so.inter_op_num_threads = 1
    so.log_severity_level = 3
    # ⚠️ 关键：关掉 ORT 的空转自旋（2026-11-21 实测）。默认 allow_spinning=1 时，
    # 推理线程在两次 run 之间**忙等**，8 Hz 限频也压不住：实测 vision 服务常驻
    # **319% CPU**（3 个核空转），关掉后降到个位数。别改回去。
    try:
        so.add_session_config_entry('session.intra_op.allow_spinning', '0')
        so.add_session_config_entry('session.inter_op.allow_spinning', '0')
    except Exception:                    # 老版本 ORT 没有这个开关
        pass
    return ort.InferenceSession(path, sess_options=so,
                                providers=['CPUExecutionProvider'])


class PersonDetector(Node):
    def __init__(self) -> None:
        super().__init__('person_detector')
        p = self.declare_parameter
        self.model_path = str(p('model_path', '').value) or self._find_model()
        self.input_size = int(p('input_size', 416).value)
        # 2026-11-26 用户报"杂物有时被识别人"：实测蓝筐上的塑料收纳袋在
        # 0.10~0.27 之间跳（阈值 0.35 就在旁边）→ 光照一变就越线。默认提到 0.45。
        self.conf = float(p('conf', 0.45).value)
        self.iou = float(p('iou', 0.45).value)
        # 2026-11-26 用户报"转弯有点跟不上"：跟转过头是延迟问题，不是转速不够
        # （ω 已顶到 0.74 硬限）。检测 8 → 12 Hz，把"看到人转身"的延迟从 125 ms 降到 83 ms。
        # 相机 15 fps 是上限，12 Hz 留了余量；推理 30~38 ms（含姿态校验）也放得下。
        self.detect_hz = float(p('detect_hz', 12.0).value)
        self.person_class = int(p('person_class', 0).value)

        # 姿态二次校验（2026-11-26）：杂物会出现 0.9 分的"关键点"，但姿态框分只有 0.003，
        # 所以规则是"姿态框分 ≥ pose_conf 且 髋/膝/踝 ≥ min_leg_kp"（腿友好，不要求肩部）。
        self.pose_enabled = bool(p('pose_enabled', True).value)
        self.pose_conf = float(p('pose_conf', 0.30).value)
        self.pose_input = int(p('pose_input', 480).value)
        self.pose_kp_conf = float(p('pose_kp_conf', 0.40).value)
        self.pose_min_leg_kp = int(p('pose_min_leg_kp', 1).value)
        self.pose_bypass = float(p('pose_bypass_conf', 0.70).value)
        self.pose_path = str(p('pose_model', '').value) or self._find_model(
            POSE_CANDIDATES)
        self.pose_sess = None
        self.pose_ms = 0.0
        if self.pose_enabled and self.pose_path and os.path.exists(self.pose_path):
            try:
                self.pose_sess = load_session(self.pose_path)
                self.pose_name = self.pose_sess.get_inputs()[0].name
            except Exception as exc:                       # noqa: BLE001
                self.get_logger().warn('姿态模型加载失败（只跑检测）：%s' % exc)
                self.pose_sess = None

        if not self.model_path or not os.path.exists(self.model_path):
            raise SystemExit('找不到 yolo11n.onnx（先下到 models/yolo11n.onnx）')
        self.sess = load_session(self.model_path)
        self.input_name = self.sess.get_inputs()[0].name
        self.get_logger().info(
            '人体检测就绪：%s 输入 %d 阈值 %.2f 上限 %.1f Hz；%s'
            % (self.model_path, self.input_size, self.conf, self.detect_hz,
               ('姿态校验 %s（框分≥%.2f，髋/膝/踝≥%d）'
                % (self.pose_path, self.pose_conf, self.pose_min_leg_kp))
               if self.pose_sess is not None else '姿态校验：未启用'))

        self.pub = self.create_publisher(String, '/robot/vision/persons', 10)
        self.create_subscription(String, '/robot/vision/select', self.on_select, 10)
        self.create_subscription(CompressedImage,
                                 '/robot/camera/color/image_raw/compressed',
                                 self.on_frame, qos_profile_sensor_data)

        self._q: queue.Queue = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

        self.sel_id = -1
        # 选中目标的"最后位置"（像素框）与短暂丢失计时：id 抖动/漏一帧时用它
        # 重新认领，别让页面的绿框忽白（2026-11-22 用户报"选好又变白"）
        self._sel_box = None
        self._sel_miss = 0.0
        self._sel_miss = 0.0          # 选中目标连续丢失的时长
        self._last_detect = 0.0
        self._infer_ms = 0.0

    # ------------------------------------------------------------ 工具
    @staticmethod
    def _find_model(cands=MODEL_CANDIDATES) -> str:
        here = os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.abspath(__file__)))))
        for rel in cands:
            cand = rel if os.path.isabs(rel) else os.path.join(here, rel)
            if os.path.exists(cand):
                return cand
        return ''

    def _detect(self, img) -> list:
        h, w = img.shape[:2]
        blob = cv2.dnn.blobFromImage(img, 1 / 255.0,
                                     (self.input_size, self.input_size),
                                     swapRB=True, crop=False)
        t0 = time.monotonic()
        out = self.sess.run(None, {self.input_name: blob})[0]
        boxes = parse_yolo_output(out, w, h, self.input_size, self.conf,
                                  self.iou, self.person_class)
        if self.pose_sess is not None and boxes:
            pose_rows = []
            try:
                pblob = cv2.dnn.blobFromImage(img, 1 / 255.0,
                                              (self.pose_input, self.pose_input),
                                              swapRB=True, crop=False)
                tp = time.monotonic()
                pout = self.pose_sess.run(None, {self.pose_name: pblob})[0]
                self.pose_ms = (time.monotonic() - tp) * 1000.0
                pose_rows = parse_pose_output(pout, w, h, self.pose_input)
            except Exception as exc:                       # noqa: BLE001
                self.get_logger().warn('姿态推理失败（本帧不做校验）：%s' % exc,
                                       throttle_duration_sec=10.0)
            boxes = filter_persons(boxes, pose_rows, self.pose_bypass,
                                   self.pose_conf, self.pose_min_leg_kp,
                                   self.pose_kp_conf)
        self._infer_ms = (time.monotonic() - t0) * 1000.0
        return boxes

    # ------------------------------------------------------------ 回调
    def on_select(self, msg: String) -> None:
        try:
            data = json.loads(msg.data or '{}')
        except Exception:                        # noqa: BLE001
            return
        new_id = int(data.get('id', -1))
        self.sel_id = new_id
        self._sel_miss = 0.0
        if new_id < 0:
            self.get_logger().info('取消跟随目标')
        else:
            self.get_logger().info('选定跟随目标 id=%d' % new_id)

    def on_frame(self, msg: CompressedImage) -> None:
        now = time.monotonic()
        if self.detect_hz > 0 and (now - self._last_detect) < 1.0 / self.detect_hz:
            return
        self._last_detect = now
        try:                                     # 只留最新一帧
            self._q.get_nowait()
        except queue.Empty:
            pass
        try:
            self._q.put_nowait(bytes(msg.data))
        except queue.Full:
            pass

    # ------------------------------------------------------------ 线程
    def _run(self) -> None:
        prev: list = []                          # [(id, (x, y, w, h, score))]
        last_pub = 0.0
        while not self._stop.is_set():
            try:
                jpeg = self._q.get(timeout=0.3)
            except queue.Empty:
                continue
            img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            boxes = self._detect(img)
            ids = assign_ids(boxes, prev)
            prev = [(pid, b) for pid, b in ids]

            # —— 选中目标的维持：命中 → 更新位置；没命中 → 用上次的框重新认领 ——
            dt = 1.0 / max(1.0, self.detect_hz)
            tracking = False
            sel_box = None
            if self.sel_id >= 0:
                hit = [b for pid, b in ids if pid == self.sel_id]
                if hit:
                    self._sel_box = hit[0]
                    self._sel_miss = 0.0
                    tracking = True
                    sel_box = hit[0]
                else:
                    # ① id 被 assign_ids 换掉了：按"上次的位置"找最近的人接回来
                    new_id = pick_selected(ids, self._sel_box)
                    if new_id is None and len(ids) == 1:
                        # ② 画面里只剩一个人 → 就是他（最常见的单人跟随场景）
                        new_id = ids[0][0]
                    if new_id is not None:
                        if new_id != self.sel_id:
                            self.get_logger().info(
                                '目标 id 重新认领：%d → %d'
                                % (self.sel_id, new_id))
                        self.sel_id = new_id
                        sel_box = next(b for pid, b in ids if pid == new_id)
                        self._sel_box = sel_box
                        self._sel_miss = 0.0
                        tracking = True
                    else:
                        # ③ 真丢了：1 s 内仍把"最后的框"报上去（stale），
                        #    页面画虚框，不会瞬间变白 / 用户不用重新选
                        self._sel_miss += dt
                        if self._sel_box is not None and self._sel_miss < 1.0:
                            sel_box = self._sel_box

            now = time.monotonic()
            if now - last_pub < 0.05:            # 最高 20 Hz
                continue
            last_pub = now
            h, w = img.shape[:2]
            persons = to_payload(ids, w, h)
            if (self.sel_id >= 0 and not tracking and sel_box is not None):
                persons.append({
                    'id': int(self.sel_id),
                    'x': round(float(sel_box[0]) / w, 4),
                    'y': round(float(sel_box[1]) / h, 4),
                    'w': round(float(sel_box[2]) / w, 4),
                    'h': round(float(sel_box[3]) / h, 4),
                    's': round(float(sel_box[4]) if len(sel_box) > 4 else 0.0, 3),
                    'stale': True})
            payload = {'w': w, 'h': h, 't': time.time(),
                       'infer_ms': round(self._infer_ms, 1),
                       'persons': persons,
                       'sel': self.sel_id, 'tracking': tracking,
                       'lost_s': round(self._sel_miss, 2)}
            self.pub.publish(String(data=json.dumps(payload)))

    def destroy_node(self):
        self._stop.set()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PersonDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        finally:
            # ⚠️ 只能关一次：SIGTERM 时 rclpy 的信号处理器已经 rcl_shutdown 过，
            # 再调一次会抛 RCLError（context 已经被关），systemd 记成 failed，
            # 严重时 core-dump（2026-11-24 fwr-vision 每十几秒崩一次就是它）。
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
