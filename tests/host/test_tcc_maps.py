from source_test import ROOT, method, run
source = (ROOT / "src/torque_converter.cpp").read_text()
header = (ROOT / "src/torque_converter.h").read_text()
header = header[header.index("enum class InternalTccState"):header.rindex("#endif")].replace("private:", "public:")
production = header + "\n" + method(source, "TorqueConverter::TorqueConverter(")

run("tcc_maps", production)
