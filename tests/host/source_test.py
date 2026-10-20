"""Compile production source fragments against explicit host-only dependencies."""
from pathlib import Path
import os
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[2]

def method(source, name):
    start = source.index(name)
    opening = source.index("{", start)
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]

def run(name, production):
    with tempfile.TemporaryDirectory(prefix="nag52-host-") as folder:
        directory = Path(folder)
        (directory / "production.h").write_text(production)
        binary = directory / "test"
        subprocess.run([os.environ.get("CXX", "g++"), "-std=c++17", "-Wall", "-Wextra",
                        "-Werror", "-Wno-unused-parameter", "-fsanitize=address,undefined",
                        "-fno-omit-frame-pointer", "-fno-pie", "-no-pie", "-I", folder,
                        "-I", str(ROOT), str(ROOT / "tests/host" / (name + ".cpp")),
                        "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
