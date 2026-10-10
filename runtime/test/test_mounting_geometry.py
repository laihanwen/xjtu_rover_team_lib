"""Offline geometry checks for the user-confirmed 2026-10-10 camera rig."""
from pathlib import Path
import unittest
import cv2
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[2]


class MountingGeometry(unittest.TestCase):
    def test_optical_axes_and_body_origin(self):
        rig=yaml.safe_load((ROOT/'hardware/camera-mounting-20261010.yaml').read_text(encoding='utf-8'))
        config=yaml.safe_load((ROOT/'runtime/config/pi-auv-task-one.yaml').read_text(encoding='utf-8'))
        # Canonical grid columns go right, rows go backward, normal goes down.
        body_from_grid=np.array([[0,-1,0],[-1,0,0],[0,0,-1]],float)
        world=np.array([[0,0,0],[1.5,0,0],[1.5,1.5,0],[0,1.5,0]],float)
        for role,section in [('front','camera_front'),('down','camera')]:
            rcb=np.array(rig[role]['camera_to_body']).reshape(3,3)
            offset=np.array(rig[role]['position_m'])
            np.testing.assert_allclose(rcb@rcb.T,np.eye(3),atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(rcb),1)
            optical_forward=rcb[:,2]
            if role=='front':
                self.assertGreater(optical_forward[0],0)
                self.assertAlmostEqual(-optical_forward[2]/optical_forward[0],1)
            else:np.testing.assert_allclose(optical_forward,[0,0,-1])
            k=np.array(config[section]['camera_matrix']).reshape(3,3)
            camera_from_grid=rcb.T@body_from_grid
            for body in [np.array([.75,.75,-2.]),np.array([.55,1.1,-1.8])]:
                camera=body+body_from_grid.T@offset
                t=-camera_from_grid@camera
                points=cv2.projectPoints(world,cv2.Rodrigues(camera_from_grid)[0],t,k,None)[0]
                ok,rvec,tvec=cv2.solvePnP(world,points,k,None)
                self.assertTrue(ok)
                rotation=cv2.Rodrigues(rvec)[0]
                recovered=-rotation.T@(tvec.reshape(3)+rcb.T@offset)
                np.testing.assert_allclose(recovered,body,atol=1e-5)
                # Ignoring the measured lever arm gives the camera position,
                # not the required robot position.
                self.assertGreater(np.linalg.norm(-rotation.T@tvec.reshape(3)-body),.05)

    def test_templates_keep_motion_gated(self):
        for name in ['runtime.yaml','pi-rov.yaml','pi-auv-observation.yaml','pi-auv-task-one.yaml','pi-auv-tag-docking.yaml']:
            cfg=yaml.safe_load((ROOT/'runtime/config'/name).read_text(encoding='utf-8'))
            self.assertEqual(cfg['vision'].get('apriltag_camera','down'),'front' if name=='pi-auv-task-one.yaml' else 'down')
            self.assertFalse(cfg['motion']['motion_commands_enabled'])
            self.assertFalse(cfg['operation']['auto_arm'])
            self.assertEqual(cfg['camera_front']['calibration_quality'],'user_verified_20261010')
            np.testing.assert_allclose(cfg['localization']['camera_offset_m'],[.09,.01,-.06])


if __name__=='__main__':unittest.main()
