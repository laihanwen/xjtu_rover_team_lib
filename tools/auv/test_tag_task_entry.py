import unittest
from tag_task_remote import validate_config, task_ready


class EntryTests(unittest.TestCase):
    def config(self):
        return {'mission': {'profile': 'tag_docking'}, 'operation': {'mode': 'debug', 'auto_start': False, 'auto_arm': False}, 'serial': {'device': '/dev/serial0'}}

    def status(self):
        return dict(mission_profile='tag_docking', operation_mode='debug', armed=False,
                    serial=True, status_fresh=True, safe_status=True, depth_sample_fresh=True,
                    recording_ready=True, origin_ready=True, tag_docking={'startup_ready': True})

    def test_debug_config(self):
        validate_config(self.config())

    def test_automatic_operation_rejected(self):
        for key, value in [('mode', 'autonomous'), ('auto_start', True), ('auto_arm', True)]:
            with self.subTest(key=key):
                config = self.config()
                config['operation'][key] = value
                with self.assertRaises(RuntimeError):
                    validate_config(config)

    def test_camera_only_rejected(self):
        config = self.config()
        config['serial']['device'] = ''
        with self.assertRaises(RuntimeError):
            validate_config(config)

    def test_ready(self):
        task_ready(self.status())

    def test_each_missing_gate_rejected(self):
        for key in ('serial', 'status_fresh', 'safe_status', 'depth_sample_fresh', 'recording_ready', 'origin_ready'):
            with self.subTest(key=key):
                status = self.status()
                del status[key]
                with self.assertRaises(RuntimeError):
                    task_ready(status)

    def test_armed_or_unready_rejected(self):
        for key in ('armed', 'startup_ready'):
            status = self.status()
            if key == 'armed':
                status[key] = True
            else:
                status['tag_docking'][key] = False
            with self.assertRaises(RuntimeError):
                task_ready(status)


if __name__ == '__main__':
    unittest.main()
