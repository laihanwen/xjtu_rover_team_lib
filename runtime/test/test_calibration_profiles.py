"""Offline checks for measured calibration profiles; no hardware or ARM commands."""
import json
from pathlib import Path
import unittest

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]


class UniqueLoader(yaml.SafeLoader):
    pass


def mapping(loader, node):
    result = {}
    for key, value in node.value:
        key = loader.construct_object(key)
        if key in result:
            raise ValueError(f'Duplicate YAML key: {key}')
        result[key] = loader.construct_object(value)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, mapping)


class CalibrationProfiles(unittest.TestCase):
    def test_measured_profiles_and_raw_localization(self):
        reports = ROOT / 'docs/calibration/20261008'
        down = json.loads((reports / 'down/report.json').read_text(encoding='utf-8'))
        front = json.loads((reports / 'front_trial/report.json').read_text(encoding='utf-8'))
        for name in ('runtime.yaml', 'pi-rov.yaml', 'pi-auv-observation.yaml', 'pi-auv-task-one.yaml'):
            with self.subTest(profile=name):
                cfg = yaml.load((ROOT / 'runtime/config' / name).read_text(encoding='utf-8'), Loader=UniqueLoader)
                for section, report in (('camera', down), ('camera_front', front)):
                    camera = cfg[section]
                    self.assertEqual((camera['width'], camera['height']), (320, 240))
                    self.assertEqual((camera['calibration_width'], camera['calibration_height']), (320, 240))
                    self.assertTrue(camera['preview_rectify'])
                    k = np.array(camera['camera_matrix']).reshape(3, 3)
                    d = np.array(camera['distortion_coefficients'])
                    np.testing.assert_array_equal(k, report['camera_matrix'])
                    np.testing.assert_array_equal(d, report['distortion_coefficients'])
                    # Validate finite, full-size cached remap tables for both measured models.
                    a, b = cv2.initUndistortRectifyMap(k, d, None, k, (320, 240), cv2.CV_32FC1)
                    self.assertEqual(a.shape, (240, 320))
                    self.assertTrue(np.isfinite(a).all() and np.isfinite(b).all())
                self.assertEqual(cfg['localization']['camera_matrix'], cfg['camera']['camera_matrix'])
                self.assertEqual(cfg['localization']['distortion'], cfg['camera']['distortion_coefficients'])
                self.assertFalse(cfg['localization']['calibration_verified'])
                self.assertFalse(cfg['motion']['motion_commands_enabled'])
                self.assertFalse(cfg['operation']['auto_arm'])
                self.assertTrue(cfg['camera_front']['enabled'])
                self.assertEqual(cfg['camera_front']['calibration_quality'], 'provisional_unstable_intrinsics')


if __name__ == '__main__':
    unittest.main()
