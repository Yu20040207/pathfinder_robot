"""Streaming decoder for DM-IMU-L1 USB measurement frames."""

from dataclasses import dataclass
import math
import struct


@dataclass(frozen=True)
class ImuFrame:
    device_id: int
    register: int
    values: tuple[float, ...]


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xFFFF
    return crc


class ImuFrameParser:
    def __init__(self) -> None:
        self._buffer = bytearray()
        self.invalid_frames = 0

    def feed(self, data: bytes) -> list[ImuFrame]:
        self._buffer.extend(data)
        frames = []
        while True:
            start = self._buffer.find(b'\x55\xaa')
            if start < 0:
                self._buffer[:] = self._buffer[-1:] if self._buffer.endswith(b'\x55') else b''
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 4:
                break
            register = self._buffer[3]
            size = 23 if register == 4 else 19 if register in (1, 2, 3) else 0
            if not size:
                self.invalid_frames += 1
                del self._buffer[0]
                continue
            if len(self._buffer) < size:
                break
            packet = bytes(self._buffer[:size])
            if packet[-1] != 0x0A or crc16_ccitt(packet[:-3]) != int.from_bytes(packet[-3:-1], 'little'):
                self.invalid_frames += 1
                del self._buffer[0]
                continue
            count = 4 if register == 4 else 3
            values = struct.unpack('<' + 'f' * count, packet[4:-3])
            if all(math.isfinite(value) for value in values):
                frames.append(ImuFrame(packet[2], register, values))
            else:
                self.invalid_frames += 1
            del self._buffer[:size]
        return frames
