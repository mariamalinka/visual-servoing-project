"""Physical contacts, swept geometry, safe actuation and collision-aware search."""
import json
from pathlib import Path
import unittest
from unittest.mock import Mock

import mujoco
import numpy as np

from app import Lab
from collision import CollisionGuard, CollisionPoseError, scale_velocity
from motion_path import MotionPath
from simulation import ROOT, Simulation
from startup_search import StartupSearch


class CollisionGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim = Simulation()

    @classmethod
    def tearDownClass(cls):
        cls.sim.close()

    def setUp(self):
        self.sim.set_obstacle(False,position=self.sim.collision.config["obstacle_position_m"])
        self.sim.reset()

    def test_contacts_enabled_and_unsafe_reset_is_atomic(self):
        sim = self.sim
        robot = np.any(sim.collision.radii > 0,axis=1)
        self.assertTrue(np.all(sim.model.geom_contype[robot] != 0))
        before = sim.data.qpos.copy(),sim.data.time,sim.velocity_command.copy()
        with self.assertRaises(CollisionPoseError):
            sim.reset([0,60,0,0,0,0])
        np.testing.assert_array_equal(sim.data.qpos,before[0])
        self.assertEqual(sim.data.time,before[1])
        np.testing.assert_array_equal(sim.velocity_command,before[2])
        # Direct geometry fixture only: demonstrate that the engine also generates contacts.
        sim.data.qpos[1] += np.deg2rad(60)
        mujoco.mj_forward(sim.model,sim.data)
        self.assertTrue(sim.forbidden_contacts())

    def test_nonadjacent_self_geometry_is_checked(self):
        sim = self.sim
        q = sim.data.qpos.copy()
        # This folded fixture exceeds the normal wrist limit, which rejects it even earlier.
        q[4] = np.pi
        distances,_ = sim.collision.distances(q)
        names = [set(sim.collision.geom_names[i] for i in pair) for pair in sim.collision.pairs]
        index = names.index({"wrist_roll_link","tool_link"})
        self.assertLess(distances[index],0)
        self.assertFalse(sim.collision.pose_clear(q))

    def test_swept_path_rejects_safe_endpoints_with_blocked_middle(self):
        sim = self.sim
        sim.set_obstacle(True)
        goal = sim.data.qpos.copy()
        start = goal+np.deg2rad([35,0,0,0,0,0])
        self.assertTrue(sim.collision.pose_clear(start))
        self.assertTrue(sim.collision.pose_clear(goal))
        self.assertFalse(sim.collision.segment_clear(start,goal))
        self.assertFalse(sim.collision.pose_clear((start+goal)/2))

    def test_detour_has_dense_clearance_and_does_not_change_live_state(self):
        sim = self.sim
        sim.set_obstacle(True)
        goal = sim.data.qpos.copy()
        start = goal+np.deg2rad([35,0,0,0,0,0])
        lo = np.maximum(sim.model.jnt_range[:,0]+.06,np.minimum(start,goal)-np.deg2rad(24))
        hi = np.minimum(sim.model.jnt_range[:,1]-.06,np.maximum(start,goal)+np.deg2rad(24))
        before = sim.data.qpos.copy(),sim.data.qvel.copy(),sim.data.time,sim.data.mocap_pos.copy()
        route = sim.collision.plan_path(start,goal,lo,hi)
        self.assertIsNotNone(route)
        self.assertEqual(len(route),3)
        for a,b in zip([start]+route[:-1],route):
            for fraction in np.linspace(0,1,81):
                distances,_ = sim.collision.distances(a+fraction*(b-a))
                self.assertTrue(np.all(distances >= sim.collision.margins-1e-9))
        np.testing.assert_array_equal(sim.data.qpos,before[0])
        np.testing.assert_array_equal(sim.data.qvel,before[1])
        self.assertEqual(sim.data.time,before[2])
        np.testing.assert_array_equal(sim.data.mocap_pos,before[3])

    def test_blocked_route_respects_allowed_workspace(self):
        sim = self.sim
        sim.set_obstacle(True)
        goal = sim.data.qpos.copy()
        start = goal+np.deg2rad([35,0,0,0,0,0])
        route = sim.collision.plan_path(start,goal,np.minimum(start,goal),np.maximum(start,goal))
        self.assertIsNone(route)

    def test_planning_budget_fails_closed_and_releases_scratch_budget(self):
        sim = self.sim
        sim.set_obstacle(True)
        goal = sim.data.qpos.copy()
        start = goal+np.deg2rad([35,0,0,0,0,0])
        budget = sim.collision.config["max_plan_checks"]
        sim.collision.config["max_plan_checks"] = 1
        try:
            route = sim.collision.plan_path(start,goal,*sim.model.jnt_range.T)
            self.assertIsNone(route)
            self.assertIsNone(sim.collision._remaining_checks)
            self.assertTrue(sim.collision.pose_clear(goal))
            self.assertFalse(sim.velocity_command.any())
        finally:
            sim.collision.config["max_plan_checks"] = budget

    def test_obstacle_update_rejects_overlap_and_reset_preserves_environment(self):
        sim = self.sim
        original = sim.data.mocap_pos.copy()
        with self.assertRaises(CollisionPoseError):
            sim.set_obstacle(True,position=sim.data.geom_xpos[sim.model.geom("camera_case").id])
        np.testing.assert_array_equal(sim.data.mocap_pos,original)
        self.assertFalse(sim.obstacle_enabled)
        position = [.55,.24,.2]
        sim.set_obstacle(True,position=position)
        sim.reset()
        np.testing.assert_array_equal(sim.data.mocap_pos[sim.obstacle_mocap_id],position)
        self.assertTrue(sim.obstacle_enabled)


class CollisionMotionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim = Simulation()

    @classmethod
    def tearDownClass(cls):
        cls.sim.close()

    def setUp(self):
        self.sim.set_obstacle(False,position=self.sim.collision.config["obstacle_position_m"])
        self.sim.reset()

    def assert_clear(self):
        distances,_ = self.sim.collision.distances(self.sim.data.qpos)
        self.assertTrue(np.all(distances >= self.sim.collision.margins-1e-5))
        self.assertFalse(self.sim.forbidden_contacts())

    def test_maximum_speed_approach_brakes_before_floor_and_stays_stopped(self):
        sim = self.sim
        sim.command_velocity(np.array([0,.6,0,0,0,0]))
        for _ in range(1500):
            sim.advance(.002)
            self.assert_clear()
        self.assertIsNotNone(sim.collision_event)
        self.assertIn("floor",sim.collision_event["pair"])
        self.assertFalse(sim.requested_velocity.any())
        self.assertFalse(sim.velocity_command.any())
        sim.advance(1)
        self.assertFalse(sim.velocity_command.any())
        self.assert_clear()

    def test_held_command_is_guarded_after_obstacle_appears(self):
        sim = self.sim
        sim.command_velocity(np.array([.3,0,0,0,0,0]))
        sim.advance(.1)
        sim.set_obstacle(True)
        for _ in range(1500):
            sim.advance(.002)  # No fresh high-level command.
            self.assert_clear()
        self.assertIsNotNone(sim.collision_event)
        self.assertIn("obstacle_box",sim.collision_event["pair"])
        self.assertFalse(sim.velocity_command.any())

    def test_damping_keeps_manual_axis_and_zero_never_creates_motion(self):
        sim = self.sim
        sim.set_obstacle(True)
        sim.reset([5,0,0,0,0,0])
        sim.command_velocity(np.array([.6,0,0,0,0,0]))
        self.assertEqual(np.flatnonzero(sim.velocity_command).tolist(),[0])
        self.assertLess(sim.velocity_command[0],.6)
        sim.command_velocity(np.zeros(6))
        self.assertFalse(sim.velocity_command.any())

    def test_blocked_manual_motion_is_reported_by_app(self):
        sim = self.sim
        lab = Lab(sim,auto_start=False)
        sim.command_velocity(np.array([0,.6,0,0,0,0]))
        lab.pulse_end = 5
        for _ in range(100):
            lab.advance(1/30)
            if lab.alignment_status == "collision_blocked":
                break
        self.assertEqual(lab.alignment_status,"collision_blocked")
        self.assertFalse(lab.aligning)
        self.assertFalse(sim.velocity_command.any())

    def test_lost_view_preserves_a_delivered_view_with_camera_delay(self):
        from camera_timing import load_camera_timing
        sim = self.sim
        sim.set_obstacle(True)
        lab = Lab(sim,auto_start=False,camera_timing=load_camera_timing(.1))
        lab.advance(.25)
        view = lab.controller.last_visible_qpos.copy()
        lab.lost_view()
        np.testing.assert_array_equal(lab.controller.last_visible_qpos,view)
        self.assertFalse(lab.camera.pending)
        self.assertFalse(sim.velocity_command.any())
        lab.align()
        lab.advance(.25)
        self.assertEqual(lab.controller.mode,"waiting")
        self.assertIsNone(lab.controller.startup)

    def test_rejected_lost_view_preset_preserves_the_original_pose(self):
        sim = self.sim
        lab = Lab(sim,auto_start=False)
        sim.reset([8,0,0,0,0,0])
        before = sim.data.qpos.copy()
        lab.controller.config["lost_view_demo_offset_degrees"] = [0,60,0,0,0,0]
        lab.lost_view()
        np.testing.assert_array_equal(sim.data.qpos,before)
        self.assertFalse(lab.aligning)
        self.assertFalse(sim.velocity_command.any())
        self.assertIn("violates collision clearance",lab.message)

    def test_obstacle_button_cancels_motion_and_keeps_pose(self):
        sim = self.sim
        lab = Lab(sim,auto_start=False)
        lab.jog(0,1);lab.advance(.05)
        q = sim.data.qpos.copy()
        lab.toggle_obstacle()
        np.testing.assert_array_equal(sim.data.qpos,q)
        self.assertFalse(sim.velocity_command.any())
        self.assertTrue(sim.obstacle_enabled)


class CollisionPathContractTests(unittest.TestCase):
    def test_uniform_scaling_satisfies_all_closing_constraints_and_allows_retreat(self):
        request = np.array([.4,.2,0,0,0,0])
        gradients = np.array([[-1,0,0,0,0,0],[0,-1,0,0,0,0]])
        lower = np.array([-.1,-.1])
        result = scale_velocity(request,gradients,lower,.6)
        np.testing.assert_allclose(result,request*.25)
        self.assertTrue(np.all(gradients@result >= lower-1e-12))
        np.testing.assert_array_equal(scale_velocity(-request,gradients,lower,.6),-request)
        np.testing.assert_array_equal(scale_velocity(np.zeros(6),gradients,lower,.6),np.zeros(6))

    def test_visible_target_interrupts_search_before_planning(self):
        planner = Mock(side_effect=AssertionError("Detection must brake before route planning"))
        search = StartupSearch(np.zeros(6),np.tile([-2,2],(6,1)),path_planner=planner)
        for _ in range(search.config["confirmation_frames"]):
            sample = search.update(np.ones((4,2)),np.zeros(6),1/30)
            self.assertFalse(sample.velocity.any())
        self.assertEqual(sample.status,"acquired")
        planner.assert_not_called()

    def test_all_blocked_waypoints_finish_with_bounded_zero_motion(self):
        planner = Mock(return_value=None)
        search = StartupSearch(np.zeros(6),np.tile([-2,2],(6,1)),path_planner=planner)
        for _ in range(100):
            sample = search.update(None,np.zeros(6),1/30)
            self.assertFalse(sample.velocity.any())
            if sample.status == "collision_blocked":
                break
        self.assertEqual(sample.status,"collision_blocked")
        self.assertGreater(search.blocked_waypoints,0)
        self.assertLessEqual(planner.call_count,len(search.waypoints))

    def test_path_follower_visits_detour_and_caches_the_current_goal(self):
        q = np.zeros(6);via = q.copy();via[1] = .3
        goal = q.copy();goal[0] = .4
        planner = Mock(return_value=[via.copy(),goal.copy()])
        path = MotionPath(planner)
        lo,hi = np.full(6,-1.),np.full(6,1.)
        np.testing.assert_array_equal(path.target(q,goal,lo,hi,.01),via)
        np.testing.assert_array_equal(path.target(via,goal,lo,hi,.01),goal)
        planner.assert_called_once()

