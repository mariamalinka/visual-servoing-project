"""Persist an alignment image and its calibration, never a robot or target pose."""
from __future__ import annotations

import json
import os
from pathlib import Path
import tempfile
import numpy as np

DEFAULT_REFERENCE = Path(__file__).resolve().parent / "reference" / "goal.npz"


def metadata(config):
    return {key: config[key] for key in
            ("camera_width", "camera_height", "marker_dictionary", "marker_id", "marker_side_m")}


def save_reference(path, rgb, K, config, detector):
    path = Path(path)
    rgb = np.asarray(rgb)
    if rgb.dtype != np.uint8 or rgb.shape != (config["camera_height"],config["camera_width"],3):
        raise ValueError("Reference must be a full-size RGB camera image")
    corners = detector(rgb)
    if corners is None:
        raise ValueError("Reference not saved: the complete marker must be detected")
    K = np.asarray(K, dtype=float)
    if K.shape != (3,3) or not np.isfinite(K).all():
        raise ValueError("Reference calibration must be a finite 3x3 matrix")
    path.parent.mkdir(parents=True, exist_ok=True)
    # Replace one complete file atomically; failed teaching preserves the old goal.
    with tempfile.NamedTemporaryFile(dir=path.parent, suffix=".npz", delete=False) as stream:
        temporary = Path(stream.name)
        try:
            np.savez_compressed(stream, rgb=rgb, K=K,
                                metadata=np.array(json.dumps(metadata(config),sort_keys=True)))
        except BaseException:
            stream.close()
            temporary.unlink(missing_ok=True)
            raise
    try:
        os.replace(temporary,path)
    finally:
        temporary.unlink(missing_ok=True)
    return np.asarray(corners).copy()


def load_reference(path, K, config, detector):
    with np.load(path, allow_pickle=False) as saved:
        if set(saved.files) != {"rgb","K","metadata"}:
            raise ValueError("Reference must contain only an image, calibration and marker settings")
        rgb = saved["rgb"].copy()
        if json.loads(str(saved["metadata"])) != metadata(config):
            raise ValueError("Saved reference marker/camera settings do not match this simulation")
        if saved["K"].shape != (3,3) or not np.allclose(saved["K"],K,rtol=0,atol=1e-8):
            raise ValueError("Saved reference camera calibration does not match")
        if rgb.dtype != np.uint8 or rgb.shape != (config["camera_height"],config["camera_width"],3):
            raise ValueError("Invalid reference image dimensions or type")
    corners = detector(rgb)
    if corners is None:
        raise ValueError("Saved reference does not contain the configured marker")
    return rgb, np.asarray(corners).copy()
