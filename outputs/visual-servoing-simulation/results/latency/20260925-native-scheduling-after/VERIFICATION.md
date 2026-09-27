# Verification

The candidate completed 20 SIFT and 20 Learned reused-worker sessions under the
same predeclared plan as the preserved baseline: 120 seconds per session,
offset [3, -3, 4, 3, -2, 2] degrees, 50 ms additional transport delay,
400 ms freshness watchdog and 50 ms control deadline. The two 10,000-uncached
frame targets were met (15,531 SIFT; 10,327 Learned).

- All 299 tests passed before the campaign; [full output](full-tests.txt).
- All 80 before/after raw trace checksums passed the paired audit.
- Model settings, scene, calibration, controller, warm-up, dependency versions
  and campaign plan matched; only realtime scheduling and its new module changed.
- Current source/runtime fingerprint still matches the completed campaign.
- All stops stayed latched; zero unsafe, post-stop-motion or contact ticks.
- Both scheduling API requests were observed without errors in every session.
- Statistical collection ran without concurrent native profiling or heavy analysis.
- Existing project changes and the failed baseline were preserved.

[Machine-readable audit](comparison-audit.json), [completion](completion.json),
[verification](verification.json), and [first/later outcomes](alignment-cohorts.json).

Safety checks passed. **Sustained performance validation failed**: Learned
degradation remains, and SIFT freshness trips increased. A successful runner
exit and passing unit tests do not imply that the timing requirement passed.
