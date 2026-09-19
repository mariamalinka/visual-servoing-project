"""Install the optional Windows CPU matcher in the selected project environment."""
import importlib.metadata
import os
from pathlib import Path
import subprocess
import sys
import sysconfig

ROOT=Path(__file__).resolve().parent
TORCH="https://download.pytorch.org/whl/cpu/torch-2.13.0%2Bcpu-cp312-cp312-win_amd64.whl#sha256=a8b450c1e58e5800e5b4691dac412f8d2d65a1dc3298166f91596603a3531e6f"

def main():
    if sys.prefix==sys.base_prefix:
        raise SystemExit("Select the project's virtual environment; run setup-learned.cmd.")
    os.environ.setdefault("PIP_CACHE_DIR",str(ROOT.parent.parent/"work/pip-cache"))
    command=[sys.executable,"-m","pip","install","--disable-pip-version-check","--timeout","120"]
    try:
        installed=importlib.metadata.version("torch")
    except importlib.metadata.PackageNotFoundError:
        installed=None
    if installed!="2.13.0+cpu":
        # The wheel includes deeply nested license files. Use Windows' extended
        # path form inside this environment rather than changing OS settings.
        site=Path(sysconfig.get_paths()["purelib"]).resolve()
        if not site.is_relative_to(Path(sys.prefix).resolve()):
            raise SystemExit("Package directory is outside the selected virtual environment")
        destination=str(site)
        if os.name=="nt" and not destination.startswith("\\\\?\\"):
            destination="\\\\?\\"+destination
        subprocess.run(command+["--upgrade","--no-deps","--target",destination,TORCH],check=True)
    subprocess.run(command+["-r",str(ROOT/"requirements-learned.txt")],check=True)
    subprocess.run([sys.executable,str(ROOT/"download_learned_models.py")],check=True,cwd=ROOT)
    subprocess.run([sys.executable,"-c",
        "from learned_perception import LearnedImagePerception; d=LearnedImagePerception(); print('Ready:',d.name)"],
        check=True,cwd=ROOT)

if __name__=="__main__":
    main()
