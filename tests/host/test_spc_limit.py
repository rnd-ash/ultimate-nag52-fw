from source_test import ROOT, run
source = (ROOT / "src/shifting_algo/s_algo.cpp").read_text()
a=source.index("                    float scalar = interpolate_float(time, 0.25")
b=source.index("                    ESP_LOGI",a)
production="int adjusted(int old_v, int correction_p, int time, Sid* sid) {"+source[a:b]+" return new_v;}"
run("spc_limit",production)
