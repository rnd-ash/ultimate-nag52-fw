from source_test import ROOT, method, run
source = (ROOT / "src/canbus/can_egs51.cpp").read_text()
production = method(source, "CanTorqueData Egs51Can::get_torque_data(")

run("egs51_recovery", production)
