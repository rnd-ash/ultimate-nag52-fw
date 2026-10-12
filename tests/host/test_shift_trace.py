from source_test import ROOT, method, run
header = (ROOT / "src/shift_trace.h").read_text()
geometry = (ROOT / "src/models/vehicle_geometry.h").read_text()
source = (ROOT / "src/shift_trace.cpp").read_text()
production = "\n".join(line for line in (header + "\n" + geometry + "\n" + source).splitlines() if not line.startswith("#include"))
# Compile the actual RLI 0x33 dispatch branch against fake diagnostic responses.
kwp = (ROOT / "src/diag/kwp2000.cpp").read_text()
start = kwp.index("args[0] == RLI_SHIFT_TRACE")
opening = kwp.index("{", start)
block = method(kwp[opening:], "{")
production += "\nvoid request_trace() " + block + "\n"

run("shift_trace", production)
