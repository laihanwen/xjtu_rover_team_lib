"""The obsolete command must reject use before importing hardware libraries."""
import subprocess
import sys
from pathlib import Path
import unittest


class DeprecatedModeEntryTests(unittest.TestCase):
    def test_no_hardware_path_is_reachable(self):
        entry = Path(__file__).with_name('switch_mode.py')
        result = subprocess.run([sys.executable, '-I', str(entry), '--mode', 'auv'],
                                capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 2)
        self.assertTrue(result.stderr)
        self.assertNotIn(b'Traceback', result.stderr)


if __name__ == '__main__':
    unittest.main()
