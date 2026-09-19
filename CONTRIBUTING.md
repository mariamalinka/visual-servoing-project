# Development

Keep application changes in `outputs/visual-servoing-simulation/`. The launchers
at the repository root delegate to that directory.

## Checks

From the root:

~~~powershell
.\outputs\visual-servoing-simulation\.venv\Scripts\python.exe -B tools/check_repository.py
.\run.cmd --verify
~~~

The repository check uses Python's standard library and Git. It checks the files
that would be included by `git add .`: prohibited generated files, oversized
files, Python syntax and relative Markdown links.

The GitHub workflow runs repository checks and the control/depth mathematics tests
on Windows with Python 3.12 and the pinned base requirements. It does not claim
GPU or rendered-integration coverage. Run the full local verification for changes
to perception, scene physics, recovery or controls; learned tests need the optional
runtime and downloaded models.

## Experiments

Run a study before invoking its report-only command on a fresh checkout. The
`latest.json` pointers are local and can contain absolute machine paths.
Historical snapshots in Git include reports, plans, metrics, representative images
and relevant source snapshots. Their bulk traces are kept locally; rerunning a
study generates the traces needed to validate and regenerate its report.

Historical paired studies intentionally check exact runtime and source hashes.
Use the recorded versions when reproducing those comparisons; a different
controller revision should produce a new experiment.

See the [results policy](outputs/visual-servoing-simulation/results/README.md)
before adding experiment output. Do not force-add virtual environments, package
caches, downloaded model weights or ZIP snapshots. Required reference images
and test fixtures remain versioned.

## Dependencies and assets

Keep dependency pins in the application's requirements files. The base runtime
is independent of optional Learned dependencies. Record model provenance and
hashes in `models/manifest.json`; setup downloads the weights.

Keep target-image provenance alongside the assets. Third-party dependencies and
pretrained weights retain their upstream license terms.

## Repository history

The cleanup removes generated files from the next committed tree and preserves
local copies. Existing commits still contain the old environment and cache.
See [repository maintenance](docs/REPOSITORY.md) for the remaining history issue.
