from source_test import ROOT, method, run
source = (ROOT / "src/shifting_algo/s_algo.cpp").read_text()
production = method(source, "int16_t ShiftingAlgorithm::trq_req_reference_torque(")
for cls, file in (("CrossoverShift", "shift_crossover.cpp"), ("ReleasingShift", "shift_release.cpp")):
    source = (ROOT / "src/shifting_algo" / file).read_text()
    start = source.index("    // Output to CAN")
    end = source.index("    return ret;", start)
    production += "\nvoid " + cls + "::send() {\n" + source[start:end] + "}\n"

run("torque_reference", production)
