"""Calibration injection and truth-frame checks against the actual simulator and app."""
import unittest
import cv2
import mujoco
import numpy as np
from app import Lab
from calibration import ControlCalibration, calibration_profiles
from simulation import Simulation


class CalibrationIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim=Simulation()

    @classmethod
    def tearDownClass(cls):
        cls.sim.close()

    def setUp(self):
        self.sim.set_obstacle(False)
        self.sim.reset()
        self.sim.set_target_mode("aruco")

    def test_perturbation_keeps_rendering_truth_and_collision_model_fixed(self):
        sim=self.sim
        rgb,q,K,J,T=sim.image(),sim.data.qpos.copy(),sim.camera_intrinsics(),sim.camera_jacobian(),sim.camera_pose()
        masks=sim.model.geom_contype.copy()
        lab=Lab(sim,auto_start=False,calibration=calibration_profiles()["combined"])
        np.testing.assert_array_equal(sim.image(),rgb)
        np.testing.assert_array_equal(sim.data.qpos,q)
        np.testing.assert_array_equal(sim.camera_intrinsics(),K)
        np.testing.assert_array_equal(sim.camera_jacobian(),J)
        np.testing.assert_array_equal(sim.camera_pose(),T)
        np.testing.assert_array_equal(sim.model.geom_contype,masks)
        self.assertFalse(np.allclose(lab.controller.ibvs.K,K))
        self.assertFalse(np.allclose(lab.control_jacobian(),J))
        self.assertAlmostEqual(lab.controller.ibvs.side_m,sim.config["marker_side_m"]*1.1)

    def test_mount_jacobian_matches_independent_virtual_camera_finite_differences(self):
        sim=self.sim
        c=ControlCalibration(dict(mount_translation_mm=[13,-7,11],mount_rotation_vector_deg=[3,-5,8]))
        transform=np.eye(4);transform[:3,:3]=c.rotation;transform[:3,3]=c.translation
        scratch=mujoco.MjData(sim.model)
        def virtual_pose(q):
            scratch.qpos[:]=q
            mujoco.mj_forward(sim.model,scratch)
            T=np.eye(4)
            T[:3,:3]=scratch.cam_xmat[sim.camera_id].reshape(3,3)@np.diag([1,-1,-1])
            T[:3,3]=scratch.cam_xpos[sim.camera_id]
            return T@transform
        q=sim.data.qpos.copy()
        R=virtual_pose(q)[:3,:3]
        measured=np.zeros((6,6));eps=1e-6
        for joint in range(6):
            step=np.zeros(6);step[joint]=eps
            plus,minus=virtual_pose(q+step),virtual_pose(q-step)
            measured[:3,joint]=R.T@(plus[:3,3]-minus[:3,3])/(2*eps)
            W=((plus[:3,:3]-minus[:3,:3])/(2*eps))@R.T
            measured[3:,joint]=R.T@np.array([W[2,1],W[0,2],W[1,0]])
        np.testing.assert_allclose(c.jacobian(sim.camera_jacobian()),measured,atol=2e-9,rtol=1e-7)
        np.testing.assert_array_equal(sim.data.qpos,q)

    def test_calibration_changes_actual_control_request_for_the_same_pixels(self):
        sim=self.sim
        sim.reset(sim.config["ibvs"]["start_offset_degrees"])
        nominal=Lab(sim,auto_start=False)
        changed=Lab(sim,auto_start=False,calibration=calibration_profiles()["combined"])
        corners=sim.marker_corners(sim.image())
        nominal.controller.start()
        changed.controller.start()
        a=nominal.controller.update(corners,nominal.control_jacobian(),sim.data.qpos,1/30)
        b=changed.controller.update(corners,changed.control_jacobian(),sim.data.qpos,1/30)
        self.assertIn(a.status,('running','joint_limited'))
        self.assertEqual(a.error_px,b.error_px)
        self.assertGreater(np.linalg.norm(a.velocity-b.velocity),1e-5)
        np.testing.assert_array_equal(nominal.reference,changed.reference)

    def test_profile_button_cancels_motion_and_preserves_pose_and_reference(self):
        sim=self.sim;lab=Lab(sim,auto_start=False)
        lab.jog(0,1);lab.advance(.05)
        q=sim.data.qpos.copy();reference=lab.reference.copy()
        lab.draw()
        rect=next(rect for rect,action in lab.buttons if action=="calibration")
        lab.on_mouse(cv2.EVENT_LBUTTONDOWN,rect[0]+3,rect[1]+3,0,None)
        self.assertEqual(lab.calibration_name,"focal-minus-5")
        self.assertFalse(sim.velocity_command.any())
        self.assertFalse(lab.aligning)
        np.testing.assert_array_equal(sim.data.qpos,q)
        np.testing.assert_array_equal(lab.reference,reference)
        np.testing.assert_allclose(lab.controller.ibvs.K[0,0],sim.camera_intrinsics()[0,0]*.95)

    def test_gain_and_target_changes_retain_the_calibration(self):
        sim=self.sim
        lab=Lab(sim,auto_start=False,calibration=calibration_profiles()["combined"])
        expected=lab.control_intrinsics()
        lab.toggle_gain()
        np.testing.assert_array_equal(lab.controller.ibvs.K,expected)
        self.assertAlmostEqual(lab.controller.ibvs.side_m,.264)
        lab.select_perception("natural")
        np.testing.assert_array_equal(lab.controller.ibvs.K,expected)
        self.assertEqual(lab.calibration_name,"combined")
        self.assertAlmostEqual(lab.controller.ibvs.side_m,.264)
        self.assertFalse(sim.velocity_command.any())

    def test_tool_pose_has_documented_origin_axes_and_owns_its_storage(self):
        sim=self.sim;T=sim.tool_pose();body=sim.model.body("tool_roll").id
        np.testing.assert_array_equal(T[:3,3],sim.data.xpos[body])
        np.testing.assert_array_equal(T[:3,:3],sim.data.xmat[body].reshape(3,3))
        T[:3,3]=100
        self.assertLess(np.linalg.norm(sim.tool_pose()[:3,3]),2)


if __name__=="__main__":
    unittest.main()

