from source_test import ROOT, method, run
header = (ROOT / "src/egs_calibration/calibration_structs.h").read_text()
source = (ROOT / "src/egs_calibration/calibration.cpp").read_text()
production = "\n".join(line for line in (header + "\n" + source).splitlines() if not line.startswith("#include"))

run("calibration", production)
