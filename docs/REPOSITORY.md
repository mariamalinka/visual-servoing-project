# Repository layout and maintenance

The application remains in `outputs/visual-servoing-simulation/`. This is the
single working implementation. The root launchers make it accessible without
navigating into that directory or creating a second copy.

The root README introduces the project, `docs/` indexes the detailed guides, and
`tools/check_repository.py` checks the publishable file set.

## Local files

These remain on the developer's disk but are excluded from Git:

| Location | Contents |
|---|---|
| `archive/exports/` | The three historical step ZIP snapshots |
| `archive/repository-cleanup-*/` | Original Git index, pending-change patch and preservation checksums |
| `work/` | Legacy virtual environment, package cache and development scratch output |
| `outputs/visual-servoing-simulation/.venv/` | Current application environment |
| `outputs/visual-servoing-simulation/.cache/` | Dependency cache and runtime repair backups |
| `outputs/visual-servoing-simulation/models/*.pth` | Downloaded pretrained weights |
| Unlisted application results | Local traces, development runs and setup logs |

The ZIP snapshots were moved within this repository directory. Environments,
raw experiments, reference goals and existing source changes were preserved.

Selected report directories remain versioned at their original paths so existing
documentation links and historical baseline inputs remain usable. New experiment
directories are ignored by default; see the
[results allowlist and policy](../outputs/visual-servoing-simulation/results/README.md).

## Existing Git history

Removing a file from the current index does not remove it from older commits.
The existing local `update` commit contains a package-cache blob of
**121,933,498 bytes**, above GitHub's 100 MiB per-file limit. A later cleanup
commit alone cannot remove that blob from the earlier commit.

Before pushing that unpublished history, the oversized blob must be removed from
the commit that introduced it. That is a separate history-editing operation;
this structure cleanup does not amend commits, rewrite history or push changes.
The original index and pending changes are backed up locally.

See [GitHub's guidance on large files and unpushed commits](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github).

## Cleanup validation — 19 September 2026

The prospective committed files were reduced from **22,533 files (795.7 MiB)**
to **619 files (52.9 MiB)**. This describes file contents, not the unchanged size
of the existing Git history or local environments.

All **162 tests** and scene/application checks passed through the root launcher.
Python syntax and relative Markdown links passed the repository check.
Checksums confirmed all **91 protected source, configuration, reference and
fixture files** stayed unchanged; all three archived ZIPs match their original
Git blobs. Detailed local evidence is kept with the cleanup backup.
