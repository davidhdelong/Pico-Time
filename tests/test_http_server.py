"""Compile the actual main.c HTTP callbacks against an adversarial fake lwIP.

Exercises ownership and callback sequencing, not radio/network performance.
"""
import pathlib
import subprocess
import tempfile

source = pathlib.Path("main.c").read_text()
start = source.index("typedef struct {\n    char request[160];")
end = source.index("static void http_server_init(void)", start)
harness = pathlib.Path("tests/test_http_server.c").read_text().replace(
    "/* ACTUAL_HTTP_IMPLEMENTATION */", source[start:end]
)
with tempfile.TemporaryDirectory(prefix="pico-http-test-") as folder:
    for sanitize in (False, True):
        binary = str(pathlib.Path(folder) / "test_http")
        flags = ["-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-g"] if sanitize else []
        subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", *flags,
                        "-x", "c", "-", "-o", binary], input=harness, text=True, check=True)
        subprocess.run([binary], check=True)
