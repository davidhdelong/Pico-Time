"""Emit the exact embedded page for host preview/tests; no firmware changes."""
import argparse
import ast
import pathlib
import re
import subprocess

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--variant", choices=("0", "1"), default="1", help="0=L76K, 1=UBX")
args = parser.parse_args()
source = pathlib.Path(__file__).resolve().parent.parent.joinpath("main.c").read_text()
dashboard = source.split("static const char http_dashboard[] =", 1)[1].split(
    "static int rtc_append_health_json", 1)[0]
expanded = subprocess.run(["cc", "-E", "-P", "-x", "c", f"-DGPS_PROTOCOL_UBX={args.variant}", "-"],
                          input=dashboard, capture_output=True, text=True, check=True).stdout
print("".join(ast.literal_eval(s) for s in re.findall(r'"(?:[^"\\]|\\.)*"', expanded)), end="")
