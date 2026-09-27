"""Follow a checked joint-space path while keeping controller goal semantics."""
from __future__ import annotations
import numpy as np


class MotionPath:
    def __init__(self, planner=None):
        self.planner = planner
        self.reset()

    def reset(self):
        self.goal = None
        self.route = None

    def target(self, qpos, goal, lower, upper, tolerance):
        goal = np.asarray(goal, dtype=float)
        if self.goal is None or not np.array_equal(goal, self.goal):
            self.goal = goal.copy()
            self.route = ([goal.copy()] if self.planner is None else
                          self.planner(qpos, goal, lower, upper))
        if not self.route:
            return None
        while len(self.route) > 1 and np.max(np.abs(qpos-self.route[0])) <= tolerance:
            self.route.pop(0)
        return self.route[0]

