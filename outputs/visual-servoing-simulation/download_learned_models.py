"""Explicit one-time download of checksum-pinned upstream model files."""
import hashlib
import json
import os
from pathlib import Path
import urllib.request
from learned_perception import MODEL_DIR,check_model_files

def main():
    manifest=json.loads((MODEL_DIR/"manifest.json").read_text(encoding="utf-8"))
    for name,row in manifest["weights"].items():
        path=MODEL_DIR/(name+".pth")
        if path.exists() and hashlib.sha256(path.read_bytes()).hexdigest()==row["sha256"]:
            print(f"{name}: already verified",flush=True)
            continue
        temporary=path.with_suffix(".part")
        try:
            print(f"Downloading {name} from its official release...",flush=True)
            with urllib.request.urlopen(row["url"],timeout=120) as source,temporary.open("wb") as target:
                while chunk:=source.read(1024*1024):
                    target.write(chunk)
            if hashlib.sha256(temporary.read_bytes()).hexdigest()!=row["sha256"]:
                raise ValueError(f"Checksum mismatch: {name}; existing model preserved")
            os.replace(temporary,path)
        finally:
            temporary.unlink(missing_ok=True)
    check_model_files()
    print("Both learned models are verified. Runtime matching works offline.")

if __name__=="__main__":
    main()
