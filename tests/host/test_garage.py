from source_test import ROOT, method, run
source = (ROOT / "src/gearbox.cpp").read_text()
block = method(source, "if (is_controllable_gear(curr_target))")
production = "void Gearbox::run() {\nGearboxGear curr_target=target_gear, curr_actual=GearboxGear::Neutral;\n"
production += block + "\nresult=curr_target; (void)curr_actual;\n}\n"

run("garage", production)
