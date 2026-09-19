"""Check the prospective Git file set without importing simulation dependencies."""
from __future__ import annotations

import ast
from pathlib import Path
import re
import subprocess
import sys
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]
APP = "outputs/visual-servoing-simulation"
MAX_FILE_BYTES = 50 * 1024 * 1024
MARKDOWN_LINK = re.compile(r"!?\[[^\]\n]*\]\(([^)\n]+)\)")


def git_paths(*args: str) -> set[str]:
    raw = subprocess.check_output(["git", *args, "-z"], cwd=ROOT)
    return {p for p in raw.decode("utf-8").split("\0") if p}


def candidate_paths() -> set[str]:
    tracked = git_paths("ls-files", "--cached")
    deleted = git_paths("ls-files", "--deleted")
    untracked = git_paths("ls-files", "--others", "--exclude-standard")
    return (tracked - deleted) | untracked


def check(paths: set[str]) -> list[str]:
    errors = []
    files = {ROOT / p for p in paths}
    for relative in sorted(paths):
        path = ROOT / relative
        parts = path.relative_to(ROOT).parts
        if (parts[0] in {"work", "archive"}
                or any(p in {".venv", "venv", "__pycache__", ".cache"} for p in parts)
                or path.suffix.lower() in {".zip", ".7z", ".rar", ".whl", ".pyc", ".pth"}
                or (relative.startswith(APP + "/results/") and "traces" in parts)):
            errors.append(f"Generated or machine-local file is included: {relative}")
        if not path.is_file():
            errors.append(f"Missing file: {relative}")
            continue
        if path.stat().st_size > MAX_FILE_BYTES:
            errors.append(f"File exceeds the repository's 50 MiB limit: {relative}")
        if path.suffix == ".py":
            try:
                ast.parse(path.read_text(encoding="utf-8-sig"), filename=relative)
            except (SyntaxError, UnicodeError) as exc:
                errors.append(f"Invalid Python source {relative}: {exc}")
        if path.suffix.lower() != ".md":
            continue
        content = path.read_text(encoding="utf-8-sig")
        content = re.sub(r"(?ms)^(`{3,}|~{3,}).*?^\1[^\n]*$", "", content)
        for match in MARKDOWN_LINK.finditer(content):
            target = match.group(1).strip()
            if target.startswith("<") and ">" in target:
                target = target[1:target.index(">")]
            else:
                target = target.split(' "', 1)[0]
            parsed = urlsplit(target)
            if parsed.scheme or parsed.netloc or not parsed.path:
                continue
            destination = (path.parent / unquote(parsed.path)).resolve()
            if not destination.is_relative_to(ROOT):
                errors.append(f"Link leaves the repository in {relative}: {target}")
                continue
            if destination in files:
                continue
            if destination.is_dir() and any(p.is_relative_to(destination) for p in files):
                continue
            errors.append(f"Link target is not included in Git in {relative}: {target}")
    return errors


def main() -> int:
    paths = candidate_paths()
    errors = check(paths)
    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        print(f"Repository check failed: {len(errors)} problem(s).", file=sys.stderr)
        return 1
    size = sum((ROOT / p).stat().st_size for p in paths)
    print(f"Repository check passed: {len(paths)} files, {size / 1024**2:.1f} MiB.")
    print("Python syntax and relative Markdown file/directory links checked.")
    print("This checks the current file set; it does not remove old Git history.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
