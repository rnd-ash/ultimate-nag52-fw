#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <filesystem>
#include <algorithm>

#include "tcu_io/tcu_io_data_source.h"
#include "tcu_io/tcu_io_trace_codec.h"

namespace {

struct ReplayOptions {
    std::string scenario_path;
    std::string input_csv;
    std::string expected_csv;
    std::string output_csv;
    std::string metrics_out_path;
    std::string tolerance_config_path;
    int tolerance_ma = 0;
    int tolerance_pwm = 0;
    int tolerance_mpc_ma = 0;
    int tolerance_spc_ma = 0;
    int tolerance_tcc_pwm = 0;
    bool check_y3 = true;
    bool check_y4 = true;
    bool check_y5 = true;
    bool allow_expected_wildcards = true;
    bool quiet = false;
};

struct ExpectedActuatorFrame {
    TCUIO::TcuIoActuatorFrame values{};
    bool check_mpc = true;
    bool check_spc = true;
    bool check_tcc = true;
    bool check_y3 = true;
    bool check_y4 = true;
    bool check_y5 = true;
};

struct ReplayMetrics {
    size_t compared_frames = 0;
    int mismatches = 0;
    size_t first_mismatch_frame = (size_t)-1;
    int max_mpc_delta = 0;
    int max_spc_delta = 0;
    int max_tcc_delta = 0;
};

int abs_i32(int v) {
    return v < 0 ? -v : v;
}

bool parse_non_negative_int(const std::string& s, int* out) {
    if (out == nullptr || s.empty()) {
        return false;
    }
    int value = 0;
    for (char c : s) {
        if (c < '0' || c > '9') {
            return false;
        }
        if (value > (std::numeric_limits<int>::max() / 10)) {
            return false;
        }
        value = value * 10 + (c - '0');
    }
    *out = value;
    return true;
}

std::string trim_copy(const std::string& s) {
    size_t start = 0;
    while (start < s.size() && (s[start] == ' ' || s[start] == '\t' || s[start] == '\r' || s[start] == '\n')) {
        start++;
    }
    size_t end = s.size();
    while (end > start && (s[end - 1] == ' ' || s[end - 1] == '\t' || s[end - 1] == '\r' || s[end - 1] == '\n')) {
        end--;
    }
    return s.substr(start, end - start);
}

bool parse_bool01(const std::string& s, bool* out) {
    if (out == nullptr) {
        return false;
    }
    if (s == "1") {
        *out = true;
        return true;
    }
    if (s == "0") {
        *out = false;
        return true;
    }
    return false;
}

bool parse_allow_forbid(const std::string& s, bool* out_allow) {
    if (out_allow == nullptr) {
        return false;
    }
    if (s == "allow") {
        *out_allow = true;
        return true;
    }
    if (s == "forbid") {
        *out_allow = false;
        return true;
    }
    return false;
}

bool parse_int32(const std::string& s, int* out) {
    if (out == nullptr || s.empty()) {
        return false;
    }
    int sign = 1;
    size_t i = 0;
    if (s[0] == '-') {
        sign = -1;
        i = 1;
        if (i >= s.size()) {
            return false;
        }
    }
    int value = 0;
    for (; i < s.size(); i++) {
        char c = s[i];
        if (c < '0' || c > '9') {
            return false;
        }
        if (value > (std::numeric_limits<int>::max() / 10)) {
            return false;
        }
        value = value * 10 + (c - '0');
    }
    *out = sign * value;
    return true;
}

std::vector<std::string> split_csv_tokens(const std::string& line) {
    std::vector<std::string> out;
    std::string curr;
    for (char c : line) {
        if (c == ',') {
            out.push_back(trim_copy(curr));
            curr.clear();
        } else {
            curr.push_back(c);
        }
    }
    out.push_back(trim_copy(curr));
    return out;
}

bool starts_with_data_token(const std::string& line) {
    size_t i = 0;
    while (i < line.size() && (line[i] == ' ' || line[i] == '\t')) {
        i++;
    }
    if (i >= line.size()) {
        return false;
    }
    char c = line[i];
    return c == '-' || c == '*' || (c >= '0' && c <= '9');
}

bool load_tolerance_config(const std::string& path, ReplayOptions* opts);

std::string resolve_path_against(const std::string& base_file_path, const std::string& value) {
    if (value.empty()) {
        return value;
    }
    std::filesystem::path p(value);
    if (p.is_absolute()) {
        return p.lexically_normal().string();
    }
    std::filesystem::path base(base_file_path);
    std::filesystem::path parent = base.parent_path();
    return (parent / p).lexically_normal().string();
}

bool load_scenario_manifest(const std::string& path, ReplayOptions* opts) {
    if (opts == nullptr) {
        return false;
    }

    std::ifstream in(path);
    if (!in.is_open()) {
        std::cerr << "Unable to open scenario manifest: " << path << "\n";
        return false;
    }

    std::string line;
    int line_no = 0;
    while (std::getline(in, line)) {
        line_no++;
        std::string t = trim_copy(line);
        if (t.empty() || t[0] == '#') {
            continue;
        }

        size_t eq = t.find('=');
        if (eq == std::string::npos) {
            std::cerr << "Invalid scenario line " << line_no << ": missing '='\n";
            return false;
        }

        std::string key = trim_copy(t.substr(0, eq));
        std::string val = trim_copy(t.substr(eq + 1));
        if (key.empty()) {
            std::cerr << "Invalid scenario line " << line_no << ": empty key\n";
            return false;
        }

        if (key == "input_csv") {
            opts->input_csv = resolve_path_against(path, val);
        } else if (key == "expected_csv") {
            opts->expected_csv = resolve_path_against(path, val);
        } else if (key == "output_csv") {
            opts->output_csv = resolve_path_against(path, val);
        } else if (key == "metrics_out") {
            opts->metrics_out_path = resolve_path_against(path, val);
        } else if (key == "tolerance_config") {
            opts->tolerance_config_path = resolve_path_against(path, val);
            if (!load_tolerance_config(opts->tolerance_config_path, opts)) {
                return false;
            }
        } else if (key == "tolerance") {
            int tol = 0;
            if (!parse_non_negative_int(val, &tol)) {
                std::cerr << "Invalid tolerance value at line " << line_no << "\n";
                return false;
            }
            opts->tolerance_ma = tol;
            opts->tolerance_pwm = tol;
            opts->tolerance_mpc_ma = tol;
            opts->tolerance_spc_ma = tol;
            opts->tolerance_tcc_pwm = tol;
        } else if (key == "tolerance_ma") {
            int tol = 0;
            if (!parse_non_negative_int(val, &tol)) {
                std::cerr << "Invalid tolerance_ma value at line " << line_no << "\n";
                return false;
            }
            opts->tolerance_ma = tol;
            opts->tolerance_mpc_ma = tol;
            opts->tolerance_spc_ma = tol;
        } else if (key == "tolerance_pwm") {
            int tol = 0;
            if (!parse_non_negative_int(val, &tol)) {
                std::cerr << "Invalid tolerance_pwm value at line " << line_no << "\n";
                return false;
            }
            opts->tolerance_pwm = tol;
            opts->tolerance_tcc_pwm = tol;
        } else if (key == "tolerance_mpc_ma") {
            int tol = 0;
            if (!parse_non_negative_int(val, &tol)) {
                std::cerr << "Invalid tolerance_mpc_ma value at line " << line_no << "\n";
                return false;
            }
            opts->tolerance_mpc_ma = tol;
        } else if (key == "tolerance_spc_ma") {
            int tol = 0;
            if (!parse_non_negative_int(val, &tol)) {
                std::cerr << "Invalid tolerance_spc_ma value at line " << line_no << "\n";
                return false;
            }
            opts->tolerance_spc_ma = tol;
        } else if (key == "tolerance_tcc_pwm") {
            int tol = 0;
            if (!parse_non_negative_int(val, &tol)) {
                std::cerr << "Invalid tolerance_tcc_pwm value at line " << line_no << "\n";
                return false;
            }
            opts->tolerance_tcc_pwm = tol;
        } else if (key == "check_y3") {
            if (!parse_bool01(val, &opts->check_y3)) {
                std::cerr << "Invalid check_y3 value at line " << line_no << " (use 0 or 1)\n";
                return false;
            }
        } else if (key == "check_y4") {
            if (!parse_bool01(val, &opts->check_y4)) {
                std::cerr << "Invalid check_y4 value at line " << line_no << " (use 0 or 1)\n";
                return false;
            }
        } else if (key == "check_y5") {
            if (!parse_bool01(val, &opts->check_y5)) {
                std::cerr << "Invalid check_y5 value at line " << line_no << " (use 0 or 1)\n";
                return false;
            }
        } else if (key == "quiet") {
            if (!parse_bool01(val, &opts->quiet)) {
                std::cerr << "Invalid quiet value at line " << line_no << " (use 0 or 1)\n";
                return false;
            }
        } else if (key == "allow_expected_wildcards") {
            if (!parse_bool01(val, &opts->allow_expected_wildcards)) {
                std::cerr << "Invalid allow_expected_wildcards value at line " << line_no << " (use 0 or 1)\n";
                return false;
            }
        } else if (key == "wildcard_policy") {
            if (!parse_allow_forbid(val, &opts->allow_expected_wildcards)) {
                std::cerr << "Invalid wildcard_policy value at line " << line_no << " (use allow or forbid)\n";
                return false;
            }
        } else {
            std::cerr << "Unknown scenario key at line " << line_no << ": " << key << "\n";
            return false;
        }
    }

    return true;
}

bool load_tolerance_config(const std::string& path, ReplayOptions* opts) {
    if (opts == nullptr) {
        return false;
    }
    std::ifstream in(path);
    if (!in.is_open()) {
        std::cerr << "Unable to open tolerance config: " << path << "\n";
        return false;
    }

    std::string line;
    int line_no = 0;
    while (std::getline(in, line)) {
        line_no++;
        std::string t = trim_copy(line);
        if (t.empty() || t[0] == '#') {
            continue;
        }

        size_t eq = t.find('=');
        if (eq == std::string::npos) {
            std::cerr << "Invalid tolerance config line " << line_no << ": missing '='\n";
            return false;
        }

        std::string key = trim_copy(t.substr(0, eq));
        std::string val = trim_copy(t.substr(eq + 1));
        if (key.empty() || val.empty()) {
            std::cerr << "Invalid tolerance config line " << line_no << ": empty key/value\n";
            return false;
        }

        if (key == "tolerance_ma") {
            int tol = 0;
            if (!parse_non_negative_int(val, &tol)) {
                std::cerr << "Invalid tolerance_ma value at line " << line_no << "\n";
                return false;
            }
            opts->tolerance_ma = tol;
            opts->tolerance_mpc_ma = tol;
            opts->tolerance_spc_ma = tol;
        } else if (key == "tolerance_pwm") {
            int tol = 0;
            if (!parse_non_negative_int(val, &tol)) {
                std::cerr << "Invalid tolerance_pwm value at line " << line_no << "\n";
                return false;
            }
            opts->tolerance_pwm = tol;
            opts->tolerance_tcc_pwm = tol;
        } else if (key == "tolerance_mpc_ma") {
            int tol = 0;
            if (!parse_non_negative_int(val, &tol)) {
                std::cerr << "Invalid tolerance_mpc_ma value at line " << line_no << "\n";
                return false;
            }
            opts->tolerance_mpc_ma = tol;
        } else if (key == "tolerance_spc_ma") {
            int tol = 0;
            if (!parse_non_negative_int(val, &tol)) {
                std::cerr << "Invalid tolerance_spc_ma value at line " << line_no << "\n";
                return false;
            }
            opts->tolerance_spc_ma = tol;
        } else if (key == "tolerance_tcc_pwm") {
            int tol = 0;
            if (!parse_non_negative_int(val, &tol)) {
                std::cerr << "Invalid tolerance_tcc_pwm value at line " << line_no << "\n";
                return false;
            }
            opts->tolerance_tcc_pwm = tol;
        } else if (key == "check_y3") {
            if (!parse_bool01(val, &opts->check_y3)) {
                std::cerr << "Invalid check_y3 value at line " << line_no << " (use 0 or 1)\n";
                return false;
            }
        } else if (key == "check_y4") {
            if (!parse_bool01(val, &opts->check_y4)) {
                std::cerr << "Invalid check_y4 value at line " << line_no << " (use 0 or 1)\n";
                return false;
            }
        } else if (key == "check_y5") {
            if (!parse_bool01(val, &opts->check_y5)) {
                std::cerr << "Invalid check_y5 value at line " << line_no << " (use 0 or 1)\n";
                return false;
            }
        } else {
            std::cerr << "Unknown tolerance config key at line " << line_no << ": " << key << "\n";
            return false;
        }
    }
    return true;
}

bool parse_args(int argc, char** argv, ReplayOptions* out) {
    if (out == nullptr || argc < 2) {
        return false;
    }

    std::vector<std::string> positional;
    for (int i = 1; i < argc; i++) {
        std::string token = argv[i];
        if (token == "--tolerance-ma") {
            if (i + 1 >= argc || !parse_non_negative_int(argv[i + 1], &out->tolerance_ma)) {
                std::cerr << "Invalid --tolerance-ma value\n";
                return false;
            }
            out->tolerance_mpc_ma = out->tolerance_ma;
            out->tolerance_spc_ma = out->tolerance_ma;
            i++;
        } else if (token == "--tolerance-pwm") {
            if (i + 1 >= argc || !parse_non_negative_int(argv[i + 1], &out->tolerance_pwm)) {
                std::cerr << "Invalid --tolerance-pwm value\n";
                return false;
            }
            out->tolerance_tcc_pwm = out->tolerance_pwm;
            i++;
        } else if (token == "--tolerance") {
            int tol = 0;
            if (i + 1 >= argc || !parse_non_negative_int(argv[i + 1], &tol)) {
                std::cerr << "Invalid --tolerance value\n";
                return false;
            }
            out->tolerance_ma = tol;
            out->tolerance_pwm = tol;
            out->tolerance_mpc_ma = tol;
            out->tolerance_spc_ma = tol;
            out->tolerance_tcc_pwm = tol;
            i++;
        } else if (token == "--tolerance-config") {
            if (i + 1 >= argc) {
                std::cerr << "Missing --tolerance-config value\n";
                return false;
            }
            out->tolerance_config_path = argv[i + 1];
            if (!load_tolerance_config(out->tolerance_config_path, out)) {
                return false;
            }
            i++;
        } else if (token == "--scenario") {
            if (i + 1 >= argc) {
                std::cerr << "Missing --scenario value\n";
                return false;
            }
            out->scenario_path = argv[i + 1];
            if (!load_scenario_manifest(out->scenario_path, out)) {
                return false;
            }
            i++;
        } else if (token == "--quiet") {
            out->quiet = true;
        } else if (token == "--metrics-out") {
            if (i + 1 >= argc) {
                std::cerr << "Missing --metrics-out value\n";
                return false;
            }
            out->metrics_out_path = argv[i + 1];
            i++;
        } else if (token == "--expected-wildcards") {
            if (i + 1 >= argc) {
                std::cerr << "Missing --expected-wildcards value\n";
                return false;
            }
            if (!parse_allow_forbid(argv[i + 1], &out->allow_expected_wildcards)) {
                std::cerr << "Invalid --expected-wildcards value (use allow or forbid)\n";
                return false;
            }
            i++;
        } else {
            positional.push_back(token);
        }
    }

    if (positional.empty()) {
        if (out->input_csv.empty()) {
            return false;
        }
        return true;
    }
    out->input_csv = positional[0];
    if (positional.size() >= 2) {
        out->expected_csv = positional[1];
    }
    if (positional.size() >= 3) {
        out->output_csv = positional[2];
    }
    if (positional.size() > 3) {
        std::cerr << "Too many positional arguments\n";
        return false;
    }
    return true;
}

bool load_hardware_trace(const std::string& path, std::vector<TCUIO::TcuIoHardwareFrame>* out) {
    if (out == nullptr) {
        return false;
    }
    std::ifstream in(path);
    if (!in.is_open()) {
        std::cerr << "Unable to open input trace: " << path << "\n";
        return false;
    }

    out->clear();
    std::string line;
    int line_no = 0;
    while (std::getline(in, line)) {
        line_no++;
        TCUIO::TcuIoHardwareFrame frame = {};
        if (TCUIO::parse_hardware_frame_csv(line.c_str(), &frame)) {
            out->push_back(frame);
            continue;
        }

        if (starts_with_data_token(line)) {
            std::cerr << "Failed to parse hardware frame line " << line_no << "\n";
            return false;
        }
    }
    return !out->empty();
}

bool parse_expected_actuator_csv_line(const std::string& line, ExpectedActuatorFrame* out, bool allow_wildcards) {
    if (out == nullptr) {
        return false;
    }
    std::vector<std::string> tokens = split_csv_tokens(line);
    if (tokens.size() != 6) {
        return false;
    }

    auto parse_u16_or_wildcard = [](const std::string& token, uint16_t* dest, bool* check) -> bool {
        if (dest == nullptr || check == nullptr) {
            return false;
        }
        if (token == "*") {
            *check = false;
            *dest = 0;
            return true;
        }
        int parsed = 0;
        if (!parse_int32(token, &parsed) || parsed < 0 || parsed > (int)UINT16_MAX) {
            return false;
        }
        *check = true;
        *dest = (uint16_t)parsed;
        return true;
    };

    auto parse_bool_or_wildcard = [](const std::string& token, bool* dest, bool* check) -> bool {
        if (dest == nullptr || check == nullptr) {
            return false;
        }
        if (token == "*") {
            *check = false;
            *dest = false;
            return true;
        }
        bool parsed = false;
        if (!parse_bool01(token, &parsed)) {
            return false;
        }
        *check = true;
        *dest = parsed;
        return true;
    };

    if (!allow_wildcards && std::any_of(tokens.begin(), tokens.end(), [](const std::string& token) { return token == "*"; })) {
        return false;
    }

    if (!parse_u16_or_wildcard(tokens[0], &out->values.mpc_current_target_ma, &out->check_mpc)) {
        return false;
    }
    if (!parse_u16_or_wildcard(tokens[1], &out->values.spc_current_target_ma, &out->check_spc)) {
        return false;
    }
    if (!parse_u16_or_wildcard(tokens[2], &out->values.tcc_pwm_12bit, &out->check_tcc)) {
        return false;
    }
    if (!parse_bool_or_wildcard(tokens[3], &out->values.y3_on, &out->check_y3)) {
        return false;
    }
    if (!parse_bool_or_wildcard(tokens[4], &out->values.y4_on, &out->check_y4)) {
        return false;
    }
    if (!parse_bool_or_wildcard(tokens[5], &out->values.y5_on, &out->check_y5)) {
        return false;
    }
    return true;
}

bool load_actuator_trace(const std::string& path, std::vector<ExpectedActuatorFrame>* out, bool allow_wildcards) {
    if (out == nullptr) {
        return false;
    }
    std::ifstream in(path);
    if (!in.is_open()) {
        std::cerr << "Unable to open expected actuator trace: " << path << "\n";
        return false;
    }

    out->clear();
    std::string line;
    int line_no = 0;
    while (std::getline(in, line)) {
        line_no++;

        ExpectedActuatorFrame frame = {};
        if (parse_expected_actuator_csv_line(line, &frame, allow_wildcards)) {
            out->push_back(frame);
            continue;
        }

        TCUIO::TcuIoActuatorFrame strict_frame = {};
        if (TCUIO::parse_actuator_frame_csv(line.c_str(), &strict_frame)) {
            frame.values = strict_frame;
            out->push_back(frame);
            continue;
        }

        if (starts_with_data_token(line)) {
            std::cerr << "Failed to parse actuator frame line " << line_no << "\n";
            return false;
        }
    }
    return !out->empty();
}

TCUIO::TcuIoActuatorFrame reference_policy(const TCUIO::TcuIoHardwareFrame& in) {
    TCUIO::TcuIoActuatorFrame out = {};

    bool drive_ready = in.parking_lock == 0;
    out.y3_on = drive_ready;
    out.y4_on = drive_ready && in.rpm_n2 > 1400;
    out.y5_on = drive_ready && in.rpm_out > 400;

    if (!drive_ready) {
        out.mpc_current_target_ma = 0;
        out.spc_current_target_ma = 0;
        out.tcc_pwm_12bit = 0;
        return out;
    }

    if (in.rpm_n2 == UINT16_MAX || in.battery_mv == UINT16_MAX) {
        out.mpc_current_target_ma = 0;
        out.spc_current_target_ma = 0;
        out.tcc_pwm_12bit = 0;
        return out;
    }

    out.mpc_current_target_ma = in.rpm_n2 > 1800 ? 850 : 500;
    out.spc_current_target_ma = in.rpm_n3 > 1500 ? 700 : 350;

    if (in.atf_temp_c > 40 && in.rpm_out > 300) {
        out.tcc_pwm_12bit = 1024;
    } else {
        out.tcc_pwm_12bit = 0;
    }
    return out;
}

bool write_actuator_trace(const std::string& path, const std::vector<TCUIO::TcuIoActuatorFrame>& trace) {
    std::ofstream out(path);
    if (!out.is_open()) {
        std::cerr << "Unable to open output trace: " << path << "\n";
        return false;
    }

    char line[192] = {0};
    TCUIO::format_actuator_frame_csv_header(line, sizeof(line));
    out << line << "\n";

    for (const auto& frame : trace) {
        if (TCUIO::format_actuator_frame_csv(frame, line, sizeof(line)) == 0) {
            std::cerr << "Failed to serialize actuator frame\n";
            return false;
        }
        out << line << "\n";
    }
    return true;
}

int compare_trace(
    const std::vector<TCUIO::TcuIoActuatorFrame>& actual,
    const std::vector<ExpectedActuatorFrame>& expected,
    int tolerance_mpc_ma,
    int tolerance_spc_ma,
    int tolerance_tcc_pwm,
    bool check_y3,
    bool check_y4,
    bool check_y5,
    bool quiet,
    ReplayMetrics* metrics
) {
    ReplayMetrics local_metrics;
    ReplayMetrics* effective_metrics = metrics == nullptr ? &local_metrics : metrics;
    if (actual.size() != expected.size()) {
        std::cerr << "Trace size mismatch. actual=" << actual.size() << " expected=" << expected.size() << "\n";
        effective_metrics->compared_frames = actual.size();
        effective_metrics->mismatches = 1;
        effective_metrics->first_mismatch_frame = 0;
        return 1;
    }

    int mismatches = 0;
    for (size_t i = 0; i < actual.size(); i++) {
        const auto& a = actual[i];
        const auto& exp = expected[i];
        const auto& e = exp.values;

        int d_mpc = abs_i32((int)a.mpc_current_target_ma - (int)e.mpc_current_target_ma);
        int d_spc = abs_i32((int)a.spc_current_target_ma - (int)e.spc_current_target_ma);
        int d_tcc = abs_i32((int)a.tcc_pwm_12bit - (int)e.tcc_pwm_12bit);

        if (d_mpc > effective_metrics->max_mpc_delta) {
            effective_metrics->max_mpc_delta = d_mpc;
        }
        if (d_spc > effective_metrics->max_spc_delta) {
            effective_metrics->max_spc_delta = d_spc;
        }
        if (d_tcc > effective_metrics->max_tcc_delta) {
            effective_metrics->max_tcc_delta = d_tcc;
        }

        bool check_mpc_now = exp.check_mpc;
        bool check_spc_now = exp.check_spc;
        bool check_tcc_now = exp.check_tcc;
        bool check_y3_now = check_y3 && exp.check_y3;
        bool check_y4_now = check_y4 && exp.check_y4;
        bool check_y5_now = check_y5 && exp.check_y5;

        bool mpc_ok = !check_mpc_now || (d_mpc <= tolerance_mpc_ma);
        bool spc_ok = !check_spc_now || (d_spc <= tolerance_spc_ma);
        bool tcc_ok = !check_tcc_now || (d_tcc <= tolerance_tcc_pwm);

        bool numeric_ok = mpc_ok && spc_ok && tcc_ok;
        bool digital_ok = (!check_y3_now || (a.y3_on == e.y3_on))
            && (!check_y4_now || (a.y4_on == e.y4_on))
            && (!check_y5_now || (a.y5_on == e.y5_on));

        if (
            !numeric_ok ||
            !digital_ok
        ) {
            mismatches++;
            if (effective_metrics->first_mismatch_frame == (size_t)-1) {
                effective_metrics->first_mismatch_frame = i;
            }
            if (!quiet) {
                std::cerr
                    << "Mismatch at frame " << i
                    << " (mpc " << a.mpc_current_target_ma << " vs " << e.mpc_current_target_ma
                    << ", spc " << a.spc_current_target_ma << " vs " << e.spc_current_target_ma
                    << ", tcc " << a.tcc_pwm_12bit << " vs " << e.tcc_pwm_12bit
                    << ", y3 " << a.y3_on << " vs " << e.y3_on
                    << ", y4 " << a.y4_on << " vs " << e.y4_on
                    << ", y5 " << a.y5_on << " vs " << e.y5_on
                    << ")\n";
            }
        }
    }
    effective_metrics->compared_frames = actual.size();
    effective_metrics->mismatches = mismatches;
    return mismatches;
}

bool write_metrics_summary(const std::string& path, const ReplayMetrics& metrics) {
    std::ofstream out(path);
    if (!out.is_open()) {
        std::cerr << "Unable to open metrics output: " << path << "\n";
        return false;
    }
    out << "frames_compared=" << metrics.compared_frames << "\n";
    out << "mismatches=" << metrics.mismatches << "\n";
    out << "first_mismatch_frame=";
    if (metrics.first_mismatch_frame == (size_t)-1) {
        out << "none\n";
    } else {
        out << metrics.first_mismatch_frame << "\n";
    }
    out << "max_mpc_delta=" << metrics.max_mpc_delta << "\n";
    out << "max_spc_delta=" << metrics.max_spc_delta << "\n";
    out << "max_tcc_delta=" << metrics.max_tcc_delta << "\n";
    return true;
}

} // namespace

int main(int argc, char** argv) {
    ReplayOptions opts;
    if (!parse_args(argc, argv, &opts)) {
        std::cerr << "Usage: host_tcu_io_trace_replay <input_csv> [expected_actuator_csv] [output_actuator_csv] [--scenario <path>] [--tolerance <n>] [--tolerance-ma <n>] [--tolerance-pwm <n>] [--tolerance-config <path>] [--expected-wildcards <allow|forbid>] [--metrics-out <path>] [--quiet]\n";
        return 2;
    }

    std::vector<TCUIO::TcuIoHardwareFrame> input_trace;
    if (!load_hardware_trace(opts.input_csv, &input_trace)) {
        std::cerr << "No input frames loaded\n";
        return 1;
    }

    TCUIO::PlaybackTcuIoDataSource source(input_trace.data(), input_trace.size(), false);
    TCUIO::MockTcuIoActuatorController mock;

    std::vector<TCUIO::TcuIoActuatorFrame> actual_trace;
    actual_trace.reserve(input_trace.size());

    for (size_t i = 0; i < input_trace.size(); i++) {
        TCUIO::TcuIoHardwareFrame hw = {};
        if (!source.read_frame(&hw)) {
            std::cerr << "Failed to read frame from playback source at index " << i << "\n";
            return 1;
        }
        TCUIO::TcuIoActuatorFrame out = reference_policy(hw);
        mock.apply(out);

        TCUIO::TcuIoActuatorFrame last = {};
        if (!mock.get_last(&last)) {
            std::cerr << "Mock controller failed to return last output\n";
            return 1;
        }
        actual_trace.push_back(last);
    }

    if (!opts.output_csv.empty()) {
        if (!write_actuator_trace(opts.output_csv, actual_trace)) {
            return 1;
        }
    }

    if (!opts.expected_csv.empty()) {
        std::vector<ExpectedActuatorFrame> expected_trace;
        if (!load_actuator_trace(opts.expected_csv, &expected_trace, opts.allow_expected_wildcards)) {
            return 1;
        }
        ReplayMetrics metrics;
        int mismatches = compare_trace(
            actual_trace,
            expected_trace,
            opts.tolerance_mpc_ma,
            opts.tolerance_spc_ma,
            opts.tolerance_tcc_pwm,
            opts.check_y3,
            opts.check_y4,
            opts.check_y5,
            opts.quiet,
            &metrics
        );

        if (!opts.metrics_out_path.empty()) {
            if (!write_metrics_summary(opts.metrics_out_path, metrics)) {
                return 1;
            }
        }

        if (!opts.quiet) {
            std::cout
                << "Compared " << metrics.compared_frames << " frame(s), mismatches=" << metrics.mismatches
                << ", max deltas [mpc=" << metrics.max_mpc_delta
                << ", spc=" << metrics.max_spc_delta
                << ", tcc=" << metrics.max_tcc_delta << "]\n";
        }

        if (mismatches != 0) {
            std::cerr << "Replay comparison failed with " << mismatches << " mismatch(es)\n";
            return 1;
        }
    }

    std::cout << "Replay completed for " << actual_trace.size() << " frame(s)\n";
    return 0;
}
