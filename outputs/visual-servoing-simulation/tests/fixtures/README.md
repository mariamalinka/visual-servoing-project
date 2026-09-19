# Detection regression fixtures

Unmodified RGB images from the original 200-trial simulation run, seed 20260906.
The default OpenCV 5.0.0 detector failed to decode ID 7 in these images.

- marker-lost-during-motion.png: trial 4, terminal image when detection failed.
- marker-not-detected-at-start.png: trial 8, initial image.
- regression-starts.json: exact joint offsets for those two trials.

The grouping-threshold fix must detect the physical black-bordered marker in both images, and both starting poses must align using the real simulation. The tests also check rejection of blank images, wrong marker IDs and a half-occluded marker.
