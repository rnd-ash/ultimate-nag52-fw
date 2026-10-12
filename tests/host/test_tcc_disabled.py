from source_test import ROOT, method, run
source = (ROOT / "src/torque_converter.cpp").read_text()
header = (ROOT / "src/torque_converter.h").read_text()
header = header[header.index("enum class InternalTccState"):header.rindex("#endif")].replace("private:", "public:")
constants = source[source.index("const int16_t rpm_map_x_headers"):source.index("TorqueConverter::TorqueConverter(")]
production = constants + header + "\n" + method(source, "void TorqueConverter::process_open_or_slip_state(")
production += "\n" + method(source, "void TorqueConverter::shift_start(")
production += "\n" + method(source, "void TorqueConverter::shift_end(")

run("tcc_disabled", production)
