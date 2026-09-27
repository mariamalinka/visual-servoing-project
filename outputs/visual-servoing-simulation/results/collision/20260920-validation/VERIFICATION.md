# Verification

The collision implementation is in the original application directory.

| Check | Result |
|---|---|
| Full regression suite before the final localized Lost view correction | 216 tests passed |
| Final collision suite, including two new Lost view regressions | 18 tests passed |
| Final existing recovery, automatic-start and gain tests | 28 tests passed |
| Scene verification through the root launcher | Passed |
| Final collision experiment: 3 matchers × 3 cases, 100 ms delay | 9/9 converged |
| Forbidden contacts or clearance violations in those cases | None |
| New robustness-runner smoke: alignment and long camera outage | 2/2 expected outcomes |
| Final source, assets and saved reference fingerprints | Match experiment manifest |

The full suite ran before the final change to the Lost view action. That localized
change preserves a previously delivered camera viewpoint and validates the preset
before moving. The 46 focused checks and all nine final experiment cases ran
after it. Recovery experiment cases now invoke the same Lost view action as the UI.

The full suite includes Learned GPU alignment, CPU fallback, camera transport,
controller mathematics, search, perception and cancellation checks. The final
collision suite additionally tests maximum-speed braking before floor contact,
the guard reacting to a newly enabled column during a held command, unsafe
initialization and obstacle placement, swept segments, checked detours,
nonadjacent self geometry and bounded planning failure.

The final nine cases preserved at least 3 mm beyond their applicable collision
margin and remained below 1 px in 30 new current images after stopping.
The long-outage smoke run stopped at the capture-age deadline and stayed stopped
after capture returned.

Evidence: [full regression log](full-regression.txt),
[final collision tests](collision-tests.txt),
[affected existing tests](affected-regression.txt),
[machine-readable verification](verification.json),
[experiment report](REPORT.md), and [source/runtime manifest](manifest.json).

Reproduce from the repository root:

~~~powershell
.\run.cmd --verify
.\run.cmd --collision-study
.\run.cmd --robustness --starts 1 --modes aruco --profiles baseline long-outage
~~~

A fresh full suite now discovers 218 tests, including the two new Lost view
regressions. Published camera and controller studies from earlier revisions have
not been rewritten. Bulk traces, preliminary runs and local backups remain
excluded from Git.
