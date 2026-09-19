"""Explicitly teach the supplied natural-picture goal from the demo home view.

This is an asset/setup command, never called by startup search or the app.
Random and cold starts only load the saved image; they do not render home first.
"""
import argparse
import json
from PIL import Image
from simulation import ROOT,Simulation
from perception import NATURAL_REFERENCE,NaturalImagePerception
from reference_image import save_reference

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replace",action="store_true",help="Explicitly replace an existing natural reference")
    args=parser.parse_args()
    if NATURAL_REFERENCE.exists() and not args.replace:
        parser.error("Reference already exists. Use --replace only to restore the supplied demo framing.")
    with Simulation() as sim:
        sim.set_target_mode("natural")
        detector=NaturalImagePerception()
        rgb=sim.image()
        observation=detector.observe(rgb)
        corners=save_reference(NATURAL_REFERENCE,rgb,sim.camera_intrinsics(),
                               detector.reference_config(sim.config),lambda frame:detector.observe(frame).corners)
        Image.fromarray(rgb).save(ROOT/"reference/natural_goal.png")
        print(json.dumps(dict(path=str(NATURAL_REFERENCE),corners=corners.tolist(),inliers=observation.inliers,
                              coverage=observation.coverage,rms=observation.reprojection_rms_px,
                              processing_ms=observation.processing_ms),indent=2))

if __name__=="__main__":
    main()
