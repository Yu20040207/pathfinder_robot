"""JSY-MK-1031 Modbus 帧与电压换算的单元测试（无硬件）。"""
import struct

import pytest

from fwr_bringup.battery_modbus import (
    crc16, percent_from_pack, parse_data_regs, read_frame, with_crc,
    write_frame,
)


def test_crc16_matches_manual_example():
    # 手册 4.4.1：01 03 00 48 00 0F 的 CRC 为 85 D8（低字节在前）
    assert crc16(bytes.fromhex('01030048000F')) == 0xD885


def test_read_frame_layout():
    frame = read_frame(1, 0x0048, 9)
    assert frame[:6] == bytes.fromhex('010300480009')
    assert len(frame) == 8
    assert crc16(frame[:6]) == (frame[6] | (frame[7] << 8))


def test_write_frame_layout():
    frame = write_frame(1, 0x0005, 1)
    assert frame[:9] == bytes.fromhex('011000050001020001')
    assert crc16(frame[:9]) == (frame[9] | (frame[10] << 8))


def test_parse_manual_response():
    # 手册示例响应里 0x0048 起的 9 个寄存器
    payload = bytes.fromhex(
        '5AE3'      # 电压 232.67 V
        '000097B2'  # 电流 3.8834 A
        '0000234F'  # 功率 0.9039 W
        '00000000'  # 电能 0
        '03E8'      # 功率因数 1.000
        '1386')     # 频率 49.98 Hz
    v, i, p, kwh, pf, hz = parse_data_regs(payload)
    assert v == pytest.approx(232.67)
    assert i == pytest.approx(3.8834)
    assert p == pytest.approx(0.9039)
    assert kwh == pytest.approx(0.0)
    assert pf == pytest.approx(1.0)
    assert hz == pytest.approx(49.98)


def test_parse_rejects_short_payload():
    with pytest.raises(ValueError):
        parse_data_regs(b'\x00' * 4)


def test_percent_curve():
    assert percent_from_pack(25.2, 6) == pytest.approx(1.0)   # 4.20 V/节
    assert percent_from_pack(24.0, 6) == pytest.approx(0.80)  # 4.00 V/节
    assert percent_from_pack(22.2, 6) == pytest.approx(0.40)  # 3.70 V/节
    assert percent_from_pack(20.4, 6) == pytest.approx(0.08)  # 3.40 V/节
    assert percent_from_pack(19.2, 6) == pytest.approx(0.0)   # 3.20 V/节
    assert percent_from_pack(26.0, 6) == pytest.approx(1.0)   # 超满
    assert percent_from_pack(18.0, 6) == pytest.approx(0.0)   # 过放


def test_percent_interpolates_between_points():
    # 3.75 V/节 在 3.70(40%) 与 3.80(55%) 之间 → 47.5%
    assert percent_from_pack(3.75 * 6, 6) == pytest.approx(0.475)


def test_percent_rejects_zero_cells():
    assert percent_from_pack(24.0, 0) != percent_from_pack(24.0, 0)  # NaN
