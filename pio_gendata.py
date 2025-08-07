import io
import subprocess

from typing import Tuple

if __name__ == "SCons.Script":
    Import("env")
    env.Execute("$PYTHONEXE -m pip --version")
    env.Execute("$PYTHONEXE -m pip install pyyaml")
    from scripts import generate_data