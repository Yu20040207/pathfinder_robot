import struct
import unittest

from fwr_imu.protocol import ImuFrameParser


def frame(register, values, *, device_id=1):
    payload = struct.pack('<' + 'f' * len(values), *values)
    data = bytes((0x55, 0xAA, device_id, register)) + payload
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xFFFF
    return data + struct.pack('<H', crc) + b'\x0a'


class ProtocolTest(unittest.TestCase):
    def test_decodes_measured_acceleration_frame_across_reads(self):
        raw = bytes.fromhex('55 aa 01 01 ec df 37 3e ea 1c c9 40 60 a5 f0 40 4a 6d 0a')
        parser = ImuFrameParser()
        self.assertEqual(parser.feed(raw[:8]), [])
        decoded = parser.feed(raw[8:])
        self.assertEqual(len(decoded), 1)
        self.assertEqual(decoded[0].register, 1)
        self.assertEqual(decoded[0].device_id, 1)
        self.assertAlmostEqual(decoded[0].values[2], 7.520187, places=4)

    def test_resynchronizes_after_corruption_and_rejects_bad_crc(self):
        bad = bytearray(frame(2, (0.1, 0.2, 0.3)))
        bad[7] ^= 0x20
        parser = ImuFrameParser()
        decoded = parser.feed(b'garbage' + bad + frame(2, (1.0, -2.0, 3.0)))
        self.assertEqual([(x.register, x.values) for x in decoded], [(2, (1.0, -2.0, 3.0))])
        self.assertEqual(parser.invalid_frames, 1)

    def test_decodes_quaternion_as_wxyz(self):
        parser = ImuFrameParser()
        decoded = parser.feed(frame(4, (0.9, 0.1, 0.2, 0.3)))
        self.assertEqual(decoded[0].register, 4)
        for observed, expected in zip(decoded[0].values, (0.9, 0.1, 0.2, 0.3)):
            self.assertAlmostEqual(observed, expected)


if __name__ == '__main__':
    unittest.main()
