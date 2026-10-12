import unittest
from prepare_sensor_replay import align


class SensorAlignmentTests(unittest.TestCase):
    def test_same_boot_independent_flags_and_depth_offset(self):
        session=dict(boot_id='boot-a',model=dict(sensor_model_verified=True,imu_signs=[1,1,-1],
                     imu_offsets_deg=[0,0,0],depth_absolute_offset_m=.2,depth_zero_m=99))
        sample=dict(kind='pid',stamp=10,numeric_valid=True,imu_sample_fresh=True,
                    depth_sample_fresh=True,roll_deg=1,pitch_deg=2,yaw_deg=3,depth_m=.4)
        frames=[dict(frame=0,capture_monotonic_sec=10.1,pi_boot_id='boot-a'),
                dict(frame=1,capture_monotonic_sec=10.2,pi_boot_id='boot-a')]
        rows=list(align(frames,[sample],session,'underwater'))
        self.assertTrue(rows[0]['valid']);self.assertAlmostEqual(rows[0]['depth_m'],.6)
        self.assertEqual(rows[0]['yaw_deg'],3) # Runtime applies the measured sign once.
        self.assertFalse(rows[1]['valid'])
        frames[0]['pi_boot_id']='boot-b'
        self.assertFalse(list(align(frames,[sample],session,'underwater'))[0]['valid'])
        frames[0]['pi_boot_id']='boot-a';sample['depth_sample_fresh']=None
        self.assertFalse(list(align(frames,[sample],session,'underwater'))[0]['valid'])

    def test_missing_model_not_filled(self):
        with self.assertRaises(ValueError):list(align([],[],dict(boot_id='a',model={}), 'underwater'))

    def test_duplicate_tick_does_not_refresh_and_reset_rejected(self):
        session=dict(boot_id='a',model=dict(sensor_model_verified=True,imu_signs=[1,1,1],
                    imu_offsets_deg=[0,0,0],depth_absolute_offset_m=0))
        row=dict(kind='pid',stamp=1,mcu_tick_ms=10,mcu_epoch=0,numeric_valid=True,
                 imu_sample_fresh=True,depth_sample_fresh=True,roll_deg=0,pitch_deg=0,yaw_deg=0,depth_m=.4)
        frame=dict(frame=0,capture_monotonic_sec=1.25,pi_boot_id='a')
        self.assertFalse(list(align([frame],[row,dict(row,stamp=1.2)],session,'underwater'))[0]['valid'])
        with self.assertRaises(ValueError):
            list(align([frame],[row,dict(row,stamp=1.2,mcu_epoch=1)],session,'underwater'))


if __name__=='__main__':unittest.main()
