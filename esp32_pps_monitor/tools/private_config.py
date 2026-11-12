"""Generate a private, ignored build header; never print Wi-Fi values."""
import json
import gzip
import os
from pathlib import Path

Import("env")
ssid = os.environ.get("PPS_WIFI_SSID")
password = os.environ.get("PPS_WIFI_PASSWORD")
if not ssid or password is None:
    raise RuntimeError("Set PPS_WIFI_SSID and PPS_WIFI_PASSWORD before building the monitor")
if len(ssid.encode()) > 32 or len(password.encode()) > 64:
    raise RuntimeError("Wi-Fi configuration exceeds ESP-IDF field lengths")
folder = Path(env.subst("$BUILD_DIR")) / "private"
folder.mkdir(parents=True, exist_ok=True)
header = folder / "private_wifi.h"
contents = "#pragma once\n#define PPS_WIFI_SSID " + json.dumps(ssid) + "\n"
contents += "#define PPS_WIFI_PASSWORD " + json.dumps(password) + "\n"
if not header.exists() or header.read_text() != contents:
    header.write_text(contents)
    header.chmod(0o600)
env.Append(CPPPATH=[str(folder.resolve())])
page = Path(env.subst("$PROJECT_DIR")) / "main" / "dashboard.html"
asset = folder / "dashboard_asset.h"
contents = "#pragma once\nstatic const char monitor_dashboard[] = " + json.dumps(page.read_text(), ensure_ascii=False) + ";\n"
compressed = gzip.compress(page.read_bytes(), mtime=0)
contents += "static const unsigned char monitor_dashboard_gzip[] = {" + ",".join(map(str, compressed)) + "};\n"
if not asset.exists() or asset.read_text() != contents:
    asset.write_text(contents)
