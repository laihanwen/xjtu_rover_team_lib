import unittest
from Re_control import encode_frame, encode_mapped_frame, load_mapping
from pathlib import Path
from socket_server_new import FrameParser


class MappingTests(unittest.TestCase):
    def test_water_feedback_reverses_heave_and_sway_only(self):
        mapping=load_mapping(Path(__file__).with_name('radiomaster-pocket.json'))
        frame=encode_mapped_frame([1,-1,1,-1,0,0,0,0],[0]*24,mapping)
        self.assertEqual(frame[1:5],bytes([255,0,0,255]))

    def test_captured_pocket_switches_and_disabled_dial(self):
        mapping = load_mapping(Path(__file__).with_name('radiomaster-pocket.json'))
        axes = [0, 0, 0, 0, .94, 1, -1, 0]
        buttons = [0, 1] + [0] * 22
        frame = encode_mapped_frame(axes, buttons, mapping)
        self.assertEqual(frame, bytes([0xA5,127,127,127,127,127,0,2,1,0,0]))
        axes[4] = -1
        self.assertEqual(encode_mapped_frame(axes, buttons, mapping), frame)
        with self.assertRaises(ValueError):
            encode_mapped_frame([0], [0], mapping)

    def test_physical_switches_follow_firmware_wire_order(self):
        frame = encode_frame([0, 0, 0, 0, -0.5, -1, 1], [1, 0, 0])
        self.assertEqual(frame, bytes([0xA5, 127, 127, 127, 127, 0, 2, 0, 1, 0, 0]))
        self.assertEqual(encode_frame([1, -1, 0, 0, -0.1, 0, 0], [0, 1, 0])[1:6],
                         bytes([255, 0, 127, 127, 255]))

    def test_reject_invalid_inputs(self):
        with self.assertRaises(ValueError):
            encode_frame([float("nan")] * 7, [0] * 3)
        with self.assertRaises(ValueError):
            encode_frame([0] * 7, [0] * 3, 1, 1)

    def test_fragmentation_noise_and_latest_frame(self):
        frame = encode_frame([0] * 7, [0] * 3)
        latest = encode_frame([1] * 7, [1] * 3)
        parser = FrameParser()
        self.assertIsNone(parser.push(b"noise" + frame[:5]))
        self.assertEqual(parser.push(frame[5:] + latest), latest)
        self.assertIsNone(parser.push(bytes([0xA5] + [255] * 10)))


if __name__ == "__main__":
    unittest.main()
