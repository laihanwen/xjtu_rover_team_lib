import unittest
from prepare_native_video_replay import sample_indices


class RecordedTimeTests(unittest.TestCase):
    def test_capture_clock_not_nominal_fps(self):
        rows = [dict(frame=i, capture_monotonic_sec=t) for i, t in enumerate([10, 10.01, 10.04, 12, 12.1, 14])]
        self.assertEqual(sample_indices(rows, 2), [0, 3, 5])

    def test_mixed_boot_or_missing_frame_rejected(self):
        for rows in [[dict(frame=0, capture_monotonic_sec=10), dict(frame=1, capture_monotonic_sec=9)],
                     [dict(frame=0, capture_monotonic_sec=10), dict(frame=2, capture_monotonic_sec=11)]]:
            with self.assertRaises(ValueError):
                sample_indices(rows, 1)


if __name__ == '__main__':
    unittest.main()
