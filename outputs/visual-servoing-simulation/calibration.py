"""Controller calibration assumptions; these never modify rendering or collision geometry."""
from __future__ import annotations
import copy
import json
from pathlib import Path
import cv2
import numpy as np

CONFIG_PATH = Path(__file__).with_name("accuracy_config.json")
DEFAULTS = dict(focal_scale=[1., 1.], principal_offset_px=[0., 0.],
                mount_translation_mm=[0., 0., 0.], mount_rotation_vector_deg=[0., 0., 0.],
                target_size_scale=1.)


def skew(vector):
    x, y, z = vector
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


class ControlCalibration:
    """The assumed camera is at T_actual_assumed = [R_delta, t_delta].

    Errors are fixed in the nominal optical mount frame (right, down, forward).
    Angular errors are a rotation vector, not three sequential Euler rotations.
    """
    def __init__(self, settings=None):
        settings = dict(settings or {})
        unknown = set(settings)-set(DEFAULTS)
        if unknown:
            raise ValueError(f"Unknown calibration settings: {sorted(unknown)}")
        self.settings = copy.deepcopy(DEFAULTS)
        self.settings.update(copy.deepcopy(settings))
        for key, length in (("focal_scale",2), ("principal_offset_px",2),
                            ("mount_translation_mm",3), ("mount_rotation_vector_deg",3)):
            raw = self.settings[key]
            if any(isinstance(x, (bool,np.bool_)) for x in np.asarray(raw,dtype=object).ravel()):
                raise ValueError(f"{key} must contain numbers")
            value = np.asarray(raw, dtype=float)
            if value.shape != (length,) or not np.isfinite(value).all():
                raise ValueError(f"{key} requires {length} finite numbers")
            self.settings[key] = value.tolist()
        if min(self.settings["focal_scale"]) <= 0:
            raise ValueError("Focal scales must be positive")
        scale = self.settings["target_size_scale"]
        if isinstance(scale,bool) or not np.isscalar(scale) or not np.isfinite(scale) or scale <= 0:
            raise ValueError("Target size scale must be positive and finite")
        self.settings["target_size_scale"] = float(scale)
        self.rotation, _ = cv2.Rodrigues(np.deg2rad(self.settings["mount_rotation_vector_deg"]))
        self.translation = np.asarray(self.settings["mount_translation_mm"])/1000

    def intrinsics(self, nominal):
        K = np.asarray(nominal,dtype=float).copy()
        if K.shape != (3,3) or not np.isfinite(K).all() or K[0,0] <= 0 or K[1,1] <= 0:
            raise ValueError("Expected a finite calibrated 3x3 intrinsic matrix")
        K[0,0] *= self.settings["focal_scale"][0]
        K[1,1] *= self.settings["focal_scale"][1]
        K[:2,2] += self.settings["principal_offset_px"]
        return K

    def controller_config(self, config):
        result = copy.deepcopy(config)
        result["marker_side_m"] *= self.settings["target_size_scale"]
        return result

    def jacobian(self, nominal):
        J = np.asarray(nominal,dtype=float)
        if J.shape != (6,6) or not np.isfinite(J).all():
            raise ValueError("Expected a finite 6x6 camera Jacobian")
        if not np.any(self.translation) and not np.any(self.settings["mount_rotation_vector_deg"]):
            return J.copy()
        # Shift the evaluation point once: v_assumed = v_actual + omega x t.
        # Then express both velocity blocks in the assumed optical axes.
        R = self.rotation.T
        return np.vstack((R@(J[:3]-skew(self.translation)@J[3:]), R@J[3:]))


def calibration_profiles():
    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    profiles = {}
    for profile in config["profiles"]:
        name = profile["name"]
        if name in profiles:
            raise ValueError("Calibration profile names must be unique")
        profiles[name] = ControlCalibration(profile["calibration"]).settings
    if "nominal" not in profiles or profiles["nominal"] != ControlCalibration().settings:
        raise ValueError("The nominal calibration profile must have zero errors")
    return profiles

