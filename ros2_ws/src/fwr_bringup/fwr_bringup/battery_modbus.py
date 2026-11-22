"""JSY-MK-1031 交直流计量模块 → ROS 2 电池状态节点。

硬件接法（模块串在电池正极）：
    电池 P+ ──→ 模块 L
    模块 N  ──→ 驱动板 P+
    电池 P− ──→ 驱动板 P−（不经过模块）
    模块 VCC/GND → 5V/地（与测量侧 3000V 隔离）
    模块 TXD/RXD → USB 转串口模块 → 主控

协议：Modbus-RTU，9600 8N1，默认地址 1。
  * 0x0005 写 1：切换到直流模式（模块出厂默认交流，必须先发）
  * 0x0048 起 9 个寄存器：电压 / 电流 / 功率 / 电能 / 功率因数 / 频率
    电压 = DATA/100 V
    电流 = 32 位 DATA/10000 A
    功率 = 32 位 DATA/10000 W
    电能 = 32 位 DATA/100 kWh
直流模式下功率因数、频率、无功功率恒为 0，本节点不使用。

发布：
  /base/battery              sensor_msgs/BatteryState  1 Hz
  /base/battery_low          std_msgs/Bool             低于 3.4 V/节
  /base/battery_energy_kwh   std_msgs/Float32          累计电能

服务：
  /base/battery_clear_energy std_srvs/Trigger          电能清零
"""
import math
import struct
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from fwr_bringup.battery_charge import ChargeDetector
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger

try:
    import serial
except ImportError:                     # pragma: no cover - 运行环境必备
    serial = None

# 6S 锂电：单体电压 → 剩余电量（%），线性插值
LIPO_CURVE = (
    (4.20, 100.0), (4.10, 90.0), (4.00, 80.0), (3.90, 70.0),
    (3.80, 55.0), (3.70, 40.0), (3.60, 25.0), (3.50, 15.0),
    (3.40, 8.0), (3.30, 3.0), (3.20, 0.0),
)

REG_MODE = 0x0005          # 信号源切换：实测写 2 = 直流（写 1 无效）
REG_MODEL = 0x0001         # 高字节 02H = 直流模块
REG_DATA = 0x0048
DATA_COUNT = 9
REG_ENERGY = 0x004D


def crc16(data: bytes) -> int:
    """Modbus CRC16（多项式 0xA001，初值 0xFFFF）。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def with_crc(frame: bytes) -> bytes:
    crc = crc16(frame)
    return frame + bytes((crc & 0xFF, crc >> 8))


def read_frame(address: int, start: int, count: int) -> bytes:
    return with_crc(struct.pack('>BBHH', address, 0x03, start, count))


def write_frame(address: int, reg: int, value: int) -> bytes:
    # 0x10 写多寄存器：addr func start(2) qty(2) byte_count(1) data(2) crc
    return with_crc(struct.pack('>BBHHBH', address, 0x10, reg, 1, 2, value))


def parse_data_regs(payload: bytes):
    """解析 0x0048 起的 9 个寄存器，返回 (v, i, p, kwh, pf, hz)。"""
    if len(payload) != DATA_COUNT * 2:
        raise ValueError('registers length mismatch')
    regs = struct.unpack('>' + 'H' * DATA_COUNT, payload)
    voltage = regs[0] / 100.0
    current = ((regs[1] << 16) | regs[2]) / 10000.0
    power = ((regs[3] << 16) | regs[4]) / 10000.0
    energy = ((regs[5] << 16) | regs[6]) / 100.0
    pf = regs[7] / 1000.0
    hz = regs[8] / 100.0
    return voltage, current, power, energy, pf, hz


def percent_from_pack(pack_v: float, cells: int) -> float:
    """整包电压 → 剩余百分比（0~1），按单体电压查表插值。"""
    if cells <= 0:
        return float('nan')
    cell = pack_v / cells
    if cell >= LIPO_CURVE[0][0]:
        return 1.0
    if cell <= LIPO_CURVE[-1][0]:
        return 0.0
    for (v_hi, p_hi), (v_lo, p_lo) in zip(LIPO_CURVE, LIPO_CURVE[1:]):
        if v_lo <= cell <= v_hi:
            span = v_hi - v_lo
            ratio = (cell - v_lo) / span if span else 0.0
            return (p_lo + (p_hi - p_lo) * ratio) / 100.0
    return 0.0


class BatteryModbus(Node):
    def __init__(self):
        super().__init__('battery_modbus')
        self.declare_parameter('device', '/dev/ttyUSB2')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('slave_address', 1)
        self.declare_parameter('poll_hz', 1.0)
        self.declare_parameter('cells', 6)
        self.declare_parameter('warn_v_per_cell', 3.40)
        self.declare_parameter('critical_v_per_cell', 3.30)
        self.declare_parameter('mock', False)
        self.declare_parameter('timeout_s', 0.5)
        self.declare_parameter('charging_voltage', 24.0)
        # 2026-11-19：单看电流方向会把「车自己耗电」误判成充电。
        # 实测：充电时 24.73 V / +0.76 A；拔掉充电器后 24.46 V / +0.74 A，
        # 电流几乎一样（+0.74 A ≈ 18 W 正好是主控+雷达的自耗），
        # 只能靠电压门限区分。
        self.declare_parameter('charge_min_voltage', 24.60)
        # 2026-11-22：充电判定改成"电压长窗斜率 + 去抖"，见 battery_charge.py。
        # 旧规则在低 SOC（21.4 V）时每秒在充电/未充电之间翻（用户报"一直跳动"）。
        self.declare_parameter('charge_rise_vps', 8e-5)      # ≈ +0.29 V/h 算在充
        self.declare_parameter('charge_fall_vps', -3e-5)     # ≈ -0.11 V/h 算没充
        self.declare_parameter('charge_debounce_s', 30.0)    # 候选状态要稳定这么久才切
        self.declare_parameter('min_voltage', 5.0)
        self.declare_parameter('max_voltage', 30.0)

        self._device = str(self.get_parameter('device').value)
        self._baud = int(self.get_parameter('baudrate').value)
        self._addr = int(self.get_parameter('slave_address').value)
        self._cells = int(self.get_parameter('cells').value)
        self._mock = bool(self.get_parameter('mock').value)
        self._timeout = float(self.get_parameter('timeout_s').value)
        self._ser = None
        self._lock = threading.Lock()
        self._started = self.get_clock().now()
        self._low = None

        self._pub = self.create_publisher(BatteryState, '/base/battery', 10)
        self._pub_low = self.create_publisher(Bool, '/base/battery_low', 10)
        self._pub_energy = self.create_publisher(
            Float32, '/base/battery_energy_kwh', 10)
        self.create_service(Trigger, '/base/battery_clear_energy',
                            self._on_clear_energy)

        self._zero_count = 0
        self._charging = False
        self._charge = ChargeDetector(
            rise_vps=float(self.get_parameter('charge_rise_vps').value),
            fall_vps=float(self.get_parameter('charge_fall_vps').value),
            top_v=float(self.get_parameter('charge_min_voltage').value),
            debounce_s=float(self.get_parameter('charge_debounce_s').value),
            initial=False)
        self._charge_log_at = 0.0
        if not self._mock:
            self._open()
        period = 1.0 / max(0.1, float(self.get_parameter('poll_hz').value))
        self.create_timer(period, self._poll)
        self.get_logger().info(
            '电池计量节点启动：%s（%s）'
            % ('模拟数据' if self._mock else self._device,
               'mock' if self._mock else f'{self._baud} 8N1 addr={self._addr}'))

    # ---------------- 串口 / Modbus ----------------
    def _open(self) -> None:
        if serial is None:
            raise RuntimeError('缺少 pyserial，请先 pip install pyserial')
        self._close()
        try:
            self._ser = serial.Serial(self._device, self._baud,
                                      timeout=self._timeout)
            self._ser.reset_input_buffer()
            self.get_logger().info(f'已打开 {self._device}')
            self._switch_dc_mode()
        except (OSError, serial.SerialException) as exc:
            self.get_logger().error(f'打开 {self._device} 失败: {exc}')
            self._ser = None

    def _close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:                    # noqa: BLE001
                pass
            self._ser = None

    def _transact(self, frame: bytes, expected: int) -> bytes:
        with self._lock:
            if self._ser is None:
                raise RuntimeError('serial not open')
            self._ser.reset_input_buffer()
            self._ser.write(frame)
            data = self._ser.read(expected)
        if len(data) != expected:
            raise RuntimeError(f'回复长度 {len(data)} != {expected}')
        if crc16(data[:-2]) != (data[-2] | (data[-1] << 8)):
            raise RuntimeError('CRC 校验失败')
        if data[0] != self._addr:
            raise RuntimeError(f'地址不匹配: {data[0]}')
        return data

    def _switch_dc_mode(self) -> None:
        """模块默认交流；实测写 0x0005=2 才会切到直流（手册说写 1，但固件 V01.05 是 2）。"""
        last = None
        for attempt in range(3):
            try:
                self._transact(write_frame(self._addr, REG_MODE, 2), 8)
                time.sleep(0.7)      # 切换后模块会短暂重启，等一下再读
                model = self._transact(
                    read_frame(self._addr, REG_MODEL, 1), 7)
                dc = (model[3] << 8 | model[4]) >> 8 == 0x02
                if dc:
                    self.get_logger().info('已切换直流模式（0x0205）')
                else:
                    self.get_logger().warn('模块型号仍非直流（02H），重试')
                    time.sleep(0.2)
                    continue
                return
            except Exception as exc:             # noqa: BLE001
                last = exc
                time.sleep(0.2)
        self.get_logger().warn(f'切换直流模式失败: {last}')

    def _read_data(self):
        resp = self._transact(
            read_frame(self._addr, REG_DATA, DATA_COUNT),
            5 + DATA_COUNT * 2)
        return parse_data_regs(resp[3:3 + DATA_COUNT * 2])

    # ---------------- 定时轮询 ----------------
    def _poll(self) -> None:
        if self._mock:
            self._publish_mock()
            return
        try:
            if self._ser is None:
                self._open()
            if self._ser is None:
                return
            voltage, current, power, energy, _pf, _hz = self._read_data()
        except Exception as exc:                 # noqa: BLE001
            self.get_logger().warn(f'读取失败: {exc}', throttle_duration_sec=5.0)
            self._close()
            return
        vmin = float(self.get_parameter('min_voltage').value)
        vmax = float(self.get_parameter('max_voltage').value)
        if not (vmin <= voltage <= vmax):
            # 模块偶发返回全 0（切回交流/通讯抖动）：丢弃这次采样并尝试恢复
            self._zero_count += 1
            self.get_logger().warn(
                f'电压异常 {voltage:.2f}V，丢弃本次采样（第 {self._zero_count} 次）',
                throttle_duration_sec=2.0)
            if self._zero_count >= 3:
                self._zero_count = 0
                self.get_logger().warn('连续异常，重新切换直流模式')
                try:
                    self._close()
                    self._open()
                    self._switch_dc_mode()
                except Exception as exc:         # noqa: BLE001
                    self.get_logger().warn(f'恢复失败: {exc}')
            return
        self._zero_count = 0
        self._publish(voltage, current, power, energy)

    def _publish_mock(self) -> None:
        elapsed = (self.get_clock().now() - self._started).nanoseconds * 1e-9
        # 25.2V 起步，每秒掉 0.02V，循环
        voltage = 25.2 - 0.02 * (elapsed % 300.0)
        current = 3.0 + 0.5 * math.sin(elapsed / 5.0)
        self._publish(voltage, current, voltage * current,
                      energy_kwh=elapsed * voltage * current / 3.6e6)

    def _publish(self, voltage: float, current: float, power: float,
                 energy_kwh: float = float('nan')) -> None:
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.voltage = float(voltage)
        msg.current = float(current)      # 实测：充电为正（充电时 +0.7~0.8 A）
        msg.charge = float('nan')
        msg.capacity = float('nan')
        msg.design_capacity = float('nan')
        msg.percentage = percent_from_pack(voltage, self._cells)
        # 充电判定（2026-11-22 重写）：斜率 + CV 顶点兜底 + 去抖。
        # 旧规则三段自相矛盾，低 SOC 时 1 Hz 方波（用户："一直充电中跳动"）。
        now_s = time.monotonic()
        self._charging = self._charge.feed(now_s, voltage, current)
        if now_s - self._charge_log_at >= 10.0:
            self._charge_log_at = now_s
            sl = self._charge.slope
            self.get_logger().info(
                '充电判定：V=%.2f V I=%.2f A 斜率=%s V/s → %s'
                % (voltage, current,
                   'n/a' if sl is None else '%.2e' % sl,
                   '充电中' if self._charging else '未充电'))
        if self._charging:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True
        cell = voltage / self._cells if self._cells else float('nan')
        msg.cell_voltage = [cell] * max(0, self._cells)
        msg.location = 'JSY-MK-1031'
        msg.serial_number = ''
        self._pub.publish(msg)
        self._pub_energy.publish(Float32(data=float(energy_kwh)))
        self._check_thresholds(voltage)

    def _check_thresholds(self, voltage: float) -> None:
        cell = voltage / self._cells if self._cells else float('nan')
        warn = float(self.get_parameter('warn_v_per_cell').value)
        critical = float(self.get_parameter('critical_v_per_cell').value)
        low = bool(cell <= warn)
        if low != self._low:
            self._low = low
            self._pub_low.publish(Bool(data=low))
            if low:
                self.get_logger().warn(
                    f'电量低：整包 {voltage:.2f} V（{cell:.2f} V/节）')
        if cell <= critical:
            self.get_logger().error(
                f'电量临界：整包 {voltage:.2f} V（{cell:.2f} V/节），建议立即返航',
                throttle_duration_sec=30.0)

    def _on_clear_energy(self, _req, response):
        if self._mock:
            self._started = self.get_clock().now()
            response.success = True
            response.message = 'mock 模式已重置'
            return response
        try:
            self._transact(write_frame(self._addr, REG_ENERGY, 0), 8)
            response.success = True
            response.message = '电能已清零'
        except Exception as exc:                 # noqa: BLE001
            response.success = False
            response.message = f'清零失败: {exc}'
        return response

    def destroy_node(self):
        self._close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BatteryModbus()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
