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
        mapping['dial_axis'] = None
        axes = [0, 0, 0, 0, .94, -1, 1, 0]
        buttons = [0, 1] + [0] * 22
        frame = encode_mapped_frame(axes, buttons, mapping)
        self.assertEqual(frame, bytes([0xA5,127,127,127,127,127,0,2,1,0,0]))
        axes[4] = -1
        self.assertEqual(encode_mapped_frame(axes, buttons, mapping), frame)
        with self.assertRaises(ValueError):
            encode_mapped_frame([0], [0], mapping)

    def test_verified_dial_marker_and_independent_selector(self):
        mapping = load_mapping(Path(__file__).with_name('radiomaster-pocket.json'))
        mapping['dial_axis'] = 4
        for selector, wire in ((-1, 0), (0, 1), (1, 2)):
            axes = [0,0,0,0,mapping['dial_min'],1,selector,0]
            low = encode_mapped_frame(axes, [0]*24, mapping)
            axes[4] = mapping['dial_max']
            high = encode_mapped_frame(axes, [0]*24, mapping)
            self.assertEqual((low[5],high[5]),(0,255))
            self.assertEqual(low[7],wire)
            self.assertEqual(low[10],1)
            self.assertEqual(low[9],0) # SI does not toggle depth hold

    def test_physical_switches_follow_firmware_wire_order(self):
        frame = encode_frame([0, 0, 0, 0, -0.5, -1, 1], [1, 0, 0])
        self.assertEqual(frame, bytes([0xA5, 127, 127, 127, 127, 0, 2, 0, 1, 0, 0]))
        self.assertEqual(encode_frame([1, -1, 0, 0, -0.1, 0, 0], [0, 1, 0])[1:6],
                         bytes([255, 0, 127, 127, 255]))

    def test_recorded_si_endpoints_and_sc_detents(self):
        mapping = load_mapping(Path(__file__).with_name('radiomaster-pocket.json'))
        self.assertEqual(mapping['dial_axis'],4)
        self.assertEqual(mapping['servo_select_axis'],6)
        self.assertNotEqual(mapping['deadman_axis'],6)
        for dial, expected in ((-.80176,0),(.21136,255),(-1,0),(1,255)):
            frame = encode_mapped_frame([0,0,0,0,dial,1,0,0], [0]*24, mapping)
            self.assertEqual(frame[5],expected)
            self.assertEqual(frame[7],1)
        for detent, expected in ((-1,0),(0,1),(.99997,2)):
            frame = encode_mapped_frame([0,0,0,0,0,1,detent,0], [0]*24, mapping)
            self.assertEqual(frame[7],expected)

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
