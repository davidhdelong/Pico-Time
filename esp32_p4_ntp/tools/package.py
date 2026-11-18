"""Merge the exact IDF flash manifest into a factory image; never flashes."""
import hashlib
import json
import pathlib
import subprocess
import sys

folder = pathlib.Path(sys.argv[1]).resolve()
manifest = json.loads((folder / "flasher_args.json").read_text())
output = folder / "pico_time_p4.factory.bin"
args = [sys.executable, "-m", "esptool", "--chip", "esp32p4", "merge-bin", "-o", str(output)]
args += manifest["write_flash_args"]
for offset, name in sorted(manifest["flash_files"].items(), key=lambda pair: int(pair[0], 16)):
    args += [offset, str(folder / name)]
subprocess.run(args, check=True)
for name in ("pico_time_p4.bin", output.name):
    data = (folder / name).read_bytes()
    print(f"{name}: {len(data)} bytes; SHA-256 {hashlib.sha256(data).hexdigest()}")
