"""Install and validate the optional Windows CPU/GPU matcher in a project venv."""
import argparse
import importlib.metadata
import os
from pathlib import Path
import subprocess
import sys
import sysconfig
import venv

ROOT = Path(__file__).resolve().parent
PROBE = """
import torch, torchvision, lightglue
assert torch.__version__ in ('2.13.0+cpu','2.13.0+cu126'), torch.__version__
assert torchvision.__version__ in ('0.28.0+cpu','0.28.0+cu126'), torchvision.__version__
device = 'cuda' if torch.cuda.is_available() else 'cpu'
assert torch.ones(2,device=device).sum().item() == 2
print('Inference device:',device)
print('Imports and tensor operation OK:', torch.__version__, torchvision.__version__)
"""
INFERENCE_PROBE = """
import cv2
import numpy as np
from learned_perception import LearnedImagePerception
d = LearnedImagePerception()
expected = np.array([[145,90],[475,105],[460,400],[125,380]], np.float32)
transform = cv2.getPerspectiveTransform(d.boundary, expected)
frame = cv2.warpPerspective(d.template_rgb, transform, (640,480), borderValue=(22,22,22))
o = d.observe(frame)
assert o.corners is not None, o.reason
assert np.sqrt(np.mean(np.sum((o.corners-expected)**2, axis=1))) < 2
print('Ready:', d.name, '|', o.inliers, 'inliers')
"""


def prefix_args():
    prefix = Path(sys.prefix).resolve()
    site = Path(sysconfig.get_paths()["purelib"]).resolve()
    if not site.is_relative_to(prefix):
        raise SystemExit("Package directory is outside the selected virtual environment")
    destination = str(prefix)
    if os.name == "nt" and not destination.startswith("\\\\?\\"):
        destination = "\\\\?\\" + destination
    # --target moves a temporary directory with its Windows ACLs into the venv.
    # --prefix writes into this venv directly, inheriting destination permissions,
    # and its extended path handles PyTorch's deeply nested license files.
    return ["--prefix", destination]


def probe_runtime():
    # A version record alone does not prove that Python can read/load the package.
    return subprocess.run([sys.executable, "-B", "-c", PROBE], cwd=ROOT,
                          text=True, capture_output=True)


def runtime_requirements(cuda=False, cpu=False):
    try:
        installed = importlib.metadata.version("torch") or ""
    except importlib.metadata.PackageNotFoundError:
        installed = ""
    if not installed:
        # A Windows long-path uninstall can leave a license-only dist-info
        # directory that shadows the valid runtime in version("torch").
        for distribution in importlib.metadata.distributions():
            if distribution.metadata.get("Name") == "torch" and distribution.version:
                installed = distribution.version
                break
    use_cuda = cuda or (not cpu and "+cu" in installed)
    return ROOT / ("requirements-learned-cuda.txt" if use_cuda else "requirements-learned.txt")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fresh", action="store_true",
                        help="Create .venv and preserve the old work/vservo-venv")
    runtimes=parser.add_mutually_exclusive_group()
    runtimes.add_argument("--cuda",action="store_true",help="Install NVIDIA CUDA acceleration")
    runtimes.add_argument("--cpu",action="store_true",help="Install the CPU-only runtime")
    args = parser.parse_args()
    requirements=runtime_requirements(args.cuda,args.cpu)
    if args.fresh:
        destination = ROOT / ".venv"
        if destination.exists():
            raise SystemExit(f"{destination} already exists; preserved. Run setup-learned.cmd to repair it.")
        venv.EnvBuilder(with_pip=True).create(destination)
        python = destination / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
        subprocess.run([str(python), str(ROOT / "setup_learned.py")] +
                       (["--cuda"] if args.cuda else ["--cpu"] if args.cpu else []), check=True, cwd=ROOT)
        return
    if sys.prefix == sys.base_prefix:
        raise SystemExit("Select the project's virtual environment; run setup-learned.cmd.")
    os.environ.setdefault("PIP_CACHE_DIR", str(ROOT / ".cache/pip"))
    print("Installing into:", sys.executable, flush=True)
    before = probe_runtime()
    if before.returncode:
        print(before.stdout + before.stderr, file=sys.stderr, flush=True)
        if "PermissionError" in before.stderr:
            raise SystemExit("Package files are unreadable. Run setup-learned.cmd --fresh "
                             "to create .venv while preserving the old environment.")
    command = [sys.executable, "-m", "pip", "install", "--disable-pip-version-check",
               "--timeout", "120", *prefix_args()]
    subprocess.run(command + ["-r", str(ROOT / "requirements.txt"),
                              "-r", str(requirements)], check=True)
    after = probe_runtime()
    if after.returncode:
        print(after.stdout + after.stderr, file=sys.stderr, flush=True)
        # Repair readable but incomplete/incompatible installs even if metadata
        # says the pinned versions are already installed.
        subprocess.run(command + ["--force-reinstall", "--no-deps", "-r",
                                  str(requirements)], check=True)
        subprocess.run([sys.executable, "-B", "-c", PROBE], check=True, cwd=ROOT)
    else:
        print(after.stdout, end="", flush=True)
    if args.cuda:
        subprocess.run([sys.executable,"-c",
                        "import torch; assert torch.cuda.is_available(), 'CUDA unavailable; check NVIDIA driver'; "
                        "print(torch.cuda.get_device_name()); print(torch.ones(2,device='cuda').sum().item())"],check=True)
    subprocess.run([sys.executable, "-m", "pip", "check"], check=True)
    subprocess.run([sys.executable, str(ROOT / "download_learned_models.py")], check=True, cwd=ROOT)
    subprocess.run([sys.executable, "-c", INFERENCE_PROBE],
                   check=True, cwd=ROOT)


if __name__ == "__main__":
    main()
