#!/usr/bin/env python3
"""DYP-E08 四合一超声主机：RS485(Modbus RTU) 上台调试工具。

纯标准库实现（termios/select），不依赖 pyserial。
寄存器表依据《DYP-RD-产品规格书(DYP-E08-V2.0)-251127-A0》。

接线（E08 主口，RS485 档）：
    白(1) = B   黄(2) = A   黑(3) = GND   红(4) = VCC 5~24 V
A/B 与转换器丝印相反不会烧，但完全无响应——没回应先对调 A/B。

用法：
    python3 tools/e08_probe.py /dev/ttyUSB0             # 读四通道距离（默认动作）
    python3 tools/e08_probe.py /dev/ttyUSB0 --info      # 版本/波特率/工作模式/角度/量程/降噪
    python3 tools/e08_probe.py /dev/ttyUSB0 --watch     # 连续读四通道，手在探头前晃动
    python3 tools/e08_probe.py /dev/ttyUSB0 --read 0x0214
    python3 tools/e08_probe.py /dev/ttyUSB0 --set 0x0216=0x0003

E08 出厂：主口 9600、地址 0x01、**工作模式 0x0216 默认 0x03 = 轮询**。
轮询是唯一不会让四只探头互相听到的模式，默认值就是对的，别乱改。
0x0214 只读，返回四个接口各自探头的波特率代码——先读它再决定要不要动别的。
"""
import argparse
import os
import select
import sys
import termios
import time
import tty

BAUD_CODES = {
    0x01: 2400, 0x02: 4800, 0x03: 9600, 0x04: 14400, 0x05: 19200,
    0x06: 38400, 0x07: 57600, 0x08: 76800, 0x09: 115200, 0x0A: 128000,
}
WORK_MODES = {0x01: '同时(会串扰)', 0x02: '交叉', 0x03: '轮询(推荐)'}
OUT_MODES = {0x00: '受控', 0x01: '自动'}
VALUE_MODES = {0x00: '处理值', 0x01: '实时值'}
ANGLES = {1: '30°', 2: '40°', 3: '50°', 4: '60°'}
RANGES = {1: '50cm', 2: '150cm', 3: '250cm', 4: '350cm'}

REG = {
    'version': 0x0000,
    'if1': 0x0106, 'if2': 0x0107, 'if3': 0x0108, 'if4': 0x0109,
    'addr': 0x0200, 'baud': 0x0201, 'outmode': 0x0202, 'value': 0x0207,
    'angle': 0x0208, 'portbaud': 0x0214, 'timeout': 0x0215,
    'workmode': 0x0216, 'noise': 0x021A, 'range': 0x021F,
}
NO_DATA = {0xFFFF: 'FFFF 无应答', 0xFFFE: 'FFFE 同频干扰', 0xFFFD: 'FFFD 无目标'}


def crc16(data):
    """Modbus RTU CRC-16，返回 (低字节, 高字节)。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return bytes((crc & 0xFF, crc >> 8))


class Rtu:
    def __init__(self, path, baud, timeout=1.0):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        tty.setraw(self.fd)
        attrs = termios.tcgetattr(self.fd)
        attrs[4] = attrs[5] = getattr(termios, 'B%d' % baud)
        attrs[2] |= termios.CLOCAL | termios.CREAD
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        self.timeout = timeout

    def close(self):
        os.close(self.fd)

    def xfer(self, payload):
        """发一帧（自动补 CRC），等应答，返回去掉 CRC 的原始应答。"""
        termios.tcflush(self.fd, termios.TCIFLUSH)
        os.write(self.fd, payload + crc16(payload))
        deadline = time.monotonic() + self.timeout
        buf = b''
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                break
            # 0x03 响应长度由字节数字段决定；0x06 的批量寄存器应答可能带回多个值，
            # 长度不定，改成“收到数据后再等 30ms 静默”来切帧。
            wait = min(left, 0.03) if buf else left
            if not select.select([self.fd], [], [], wait)[0]:
                if buf:
                    break
                continue
            buf += os.read(self.fd, 256)
            if len(buf) >= 3 and buf[1] == 0x03 and len(buf) >= 5 + buf[2]:
                break
        if len(buf) < 4:
            return b''
        if crc16(buf[:-2]) != buf[-2:]:
            raise ValueError('CRC 错：%s' % buf.hex(' '))
        return buf


def read_regs(rtu, addr, reg, count=1):
    resp = rtu.xfer(bytes((addr, 0x03, reg >> 8, reg & 0xFF, count >> 8, count & 0xFF)))
    if not resp:
        return None
    if resp[1] & 0x80:
        return '异常码 0x%02X' % resp[2]
    return [(resp[3 + 2 * i] << 8) | resp[4 + 2 * i] for i in range(resp[2] // 2)]


def show(values):
    if values is None:
        return '无应答'
    if isinstance(values, str):
        return values
    return ' '.join(f'{v}(0x{v:04X})' for v in values)


def distances(rtu, addr):
    values = read_regs(rtu, addr, REG['if1'], 4)
    if not values or isinstance(values, str):
        print('四通道 -> %s' % show(values))
        return values
    for i, v in enumerate(values, 1):
        note = NO_DATA.get(v, '%d mm' % v)
        print('  接口%d: %5d  0x%04X  %s' % (i, v, v, note))
    return values


def info(rtu, addr):
    baud = read_regs(rtu, addr, REG['baud'])
    print('版本(0x0000)     %s' % show(read_regs(rtu, addr, REG['version'])))
    if isinstance(baud, list) and baud:
        print('主口波特率       %s' % BAUD_CODES.get(baud[0], '未知'))
    for name, reg, table in (
        ('工作模式', REG['workmode'], WORK_MODES),
        ('输出模式', REG['outmode'], OUT_MODES),
        ('值类型', REG['value'], VALUE_MODES),
        ('触发超时', REG['timeout'], None),
    ):
        values = read_regs(rtu, addr, reg)
        if isinstance(values, list) and values:
            label = table.get(values[0], '未知') if table else '%d0ms' % values[0]
            print('%-16s %s' % (name, label))
    for name, reg, table in (
        ('接口角度等级', REG['angle'], ANGLES),
        ('接口量程等级', REG['range'], RANGES),
        ('接口降噪等级', REG['noise'], None),
    ):
        values = read_regs(rtu, addr, reg, 4)
        if isinstance(values, list) and values:
            parts = [NO_DATA.get(v, table.get(v, str(v)) if table else str(v)) for v in values]
            print('%-16s %s' % (name, ' | '.join(parts)))
    pb = read_regs(rtu, addr, REG['portbaud'], 4)
    if isinstance(pb, list) and pb:
        parts = [NO_DATA.get(v, '%d' % BAUD_CODES.get(v, v)) for v in pb]
        print('%-16s %s' % ('接口波特率', ' | '.join(parts)))


def main():
    ap = argparse.ArgumentParser(description='E08 四合一超声主机 RS485 调试')
    ap.add_argument('port')
    ap.add_argument('--baud', type=int, default=9600, help='默认 9600（E08 出厂值）')
    ap.add_argument('--addr', type=lambda s: int(s, 0), default=1, help='从机地址，默认 0x01')
    ap.add_argument('--info', action='store_true', help='打印全部配置寄存器')
    ap.add_argument('--watch', action='store_true', help='连续读四通道距离')
    ap.add_argument('--scan', action='store_true', help='扫 0x0000~0x0220（寄存器已公开，一般不需要）')
    ap.add_argument('--read', type=lambda s: int(s, 0), help='读指定寄存器')
    ap.add_argument('-n', '--count', type=int, default=1)
    ap.add_argument('--set', help='写寄存器，格式 REG=VAL')
    ap.add_argument('--timeout', type=float, default=1.0, help='单帧超时，轮询一轮可能到 0.8s')
    args = ap.parse_args()

    rtu = Rtu(args.port, args.baud, args.timeout)
    try:
        if args.set:
            reg_s, val_s = args.set.split('=')
            reg, val = int(reg_s, 0), int(val_s, 0)
            payload = bytes((args.addr, 0x06, reg >> 8, reg & 0xFF, val >> 8, val & 0xFF))
            print('写 0x%04X = 0x%04X -> %s' % (reg, val, rtu.xfer(payload).hex(' ') or '无应答'))
        elif args.scan:
            for reg in range(0x0000, 0x0220):
                try:
                    values = read_regs(rtu, args.addr, reg, 1)
                except ValueError as exc:
                    print('0x%04X  %s' % (reg, exc))
                    continue
                if isinstance(values, list) and values[0]:
                    print('0x%04X  = %s' % (reg, show(values)))
        elif args.info:
            info(rtu, args.addr)
        elif args.watch:
            print('读四通道（轮询模式一轮约 0.3~0.8s），手在探头前晃动，Ctrl+C 退出')
            while True:
                print('\n%s' % time.strftime('%H:%M:%S'), end=' ')
                distances(rtu, args.addr)
                time.sleep(0.2)
        elif args.read is not None:
            print('0x%04X -> %s' % (args.read, show(read_regs(rtu, args.addr, args.read, args.count))))
        else:
            distances(rtu, args.addr)
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        rtu.close()


if __name__ == '__main__':
    sys.exit(main())
