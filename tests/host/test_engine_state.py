from source_test import ROOT, method, run
source = (ROOT / "src/gearbox.cpp").read_text()
production = method(source, "void Gearbox::update_engine_state(")
production += "\n" + method(source, "void Gearbox::set_torque_request(")
a = source.index("        else\n        {\n            // Signal loss does not prove standstill.")
block = method(source[a:], "else")
production += "\nvoid Gearbox::stopped_outputs(bool speeds_valid, bool shifting) " + block[block.index("{"):]
a = source.index("        // The shift task owns active-shift outputs;")
block = method(source[a:], "if (!shifting")
production += "\nvoid Gearbox::idle_outputs(bool shifting) {" + block + "}"
run("engine_state", production)
