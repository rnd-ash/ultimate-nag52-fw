from pathlib import Path
import subprocess
import sys
for name in ["test_engine_state.py", "test_tcc_requests.py", "test_tcc_disabled.py", "test_spc_limit.py", "test_calibration.py"]:
    subprocess.run([sys.executable, str(Path(__file__).with_name(name))], check=True)
