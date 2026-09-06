// Guards the sdkconfig options that the firmware's interrupt handlers rely on
// for cache safety.
//
// WHY THIS TEST EXISTS
// --------------------
// Any flash operation (an NVS write, a map burn, an OTA transfer) disables the
// CPU cache on BOTH cores for its duration. While the cache is off, every
// flash-backed region is unreachable: IROM instructions, DROM read-only data,
// and PSRAM. An interrupt that is still serviced in that window and touches any
// of them takes an immediate "Cache disabled but cached memory region accessed"
// panic and the TCU reboots mid-drive.
//
// The TCC solenoid gptimer ISR hit exactly this. The trap is that
// CONFIG_GPTIMER_ISR_HANDLER_IN_IRAM and CONFIG_GPTIMER_CTRL_FUNC_IN_IRAM look
// like they make an ISR "IRAM safe" but are only performance options -
// CONFIG_GPTIMER_ISR_IRAM_SAFE is the one that keeps the interrupt alive and
// safe with the cache disabled.
//
// inrush_solenoid.cpp carries an #error guard for the same invariant, so a real
// firmware build cannot regress silently. This test catches it earlier and
// without an ESP-IDF toolchain - in particular it catches the case where
// somebody regenerates sdkconfig from menuconfig and quietly drops the option.

#include <cstddef>
#include <fstream>
#include <iostream>
#include <map>
#include <string>

namespace {

int g_failures = 0;

struct RequiredOption {
    const char* key;
    const char* why;
};

const RequiredOption REQUIRED_OPTIONS[] = {
    {
        "CONFIG_GPTIMER_ISR_IRAM_SAFE",
        "src/solenoids/inrush_solenoid.cpp: the TCC solenoid gptimer alarm ISR is "
        "serviced while flash writes have the cache disabled. "
        "CONFIG_GPTIMER_ISR_HANDLER_IN_IRAM is a performance option and is NOT a "
        "substitute for this one."
    },
    {
        "CONFIG_GPTIMER_CTRL_FUNC_IN_IRAM",
        "src/solenoids/inrush_solenoid.cpp: the TCC ISR calls "
        "gptimer_set_alarm_action(), which would otherwise be flash resident."
    },
    {
        "CONFIG_TWAI_ISR_IN_IRAM",
        "src/canbus/can_hal.cpp: the TWAI driver is installed with "
        "ESP_INTR_FLAG_IRAM, so its ISR path must genuinely be in IRAM."
    },
};

std::string trim(const std::string& value) {
    const std::size_t first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return std::string();
    }
    const std::size_t last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, (last - first) + 1u);
}

// Parses an sdkconfig into key -> value.
// "CONFIG_X=y"             becomes  X -> "y"
// "# CONFIG_X is not set"  becomes  X -> ""  (explicitly disabled)
// Anything absent from the file simply has no entry.
bool parse_sdkconfig(const std::string& path, std::map<std::string, std::string>* out) {
    std::ifstream file(path.c_str());
    if (!file.is_open()) {
        return false;
    }
    const std::string not_set_prefix = "# CONFIG_";
    const std::string not_set_suffix = " is not set";
    std::string line;
    while (std::getline(file, line)) {
        const std::string trimmed = trim(line);
        if (trimmed.empty()) {
            continue;
        }
        if (trimmed.compare(0u, not_set_prefix.size(), not_set_prefix) == 0) {
            const std::size_t suffix_at = trimmed.rfind(not_set_suffix);
            if (suffix_at != std::string::npos && suffix_at > 2u) {
                (*out)[trim(trimmed.substr(2u, suffix_at - 2u))] = std::string();
            }
            continue;
        }
        if (trimmed[0] == '#') {
            continue;
        }
        const std::size_t eq_at = trimmed.find('=');
        if (eq_at == std::string::npos) {
            continue;
        }
        (*out)[trim(trimmed.substr(0u, eq_at))] = trim(trimmed.substr(eq_at + 1u));
    }
    return true;
}

void expect_option_enabled(const std::map<std::string, std::string>& config, const RequiredOption& option) {
    const std::map<std::string, std::string>::const_iterator found = config.find(option.key);
    if (found == config.end()) {
        std::cerr << "FAIL: " << option.key << " is missing from sdkconfig\n"
                  << "      Required by: " << option.why << "\n";
        g_failures++;
        return;
    }
    if (found->second != "y") {
        std::cerr << "FAIL: " << option.key << " must be 'y' but is '"
                  << (found->second.empty() ? "not set" : found->second) << "'\n"
                  << "      Required by: " << option.why << "\n";
        g_failures++;
    }
}

// Sanity check on the parser itself, so a silently broken parse cannot make the
// checks above pass by accident.
void test_parser_distinguishes_set_from_not_set(const std::map<std::string, std::string>& config) {
    if (config.empty()) {
        std::cerr << "FAIL: parsed sdkconfig was empty\n";
        g_failures++;
        return;
    }
    const std::map<std::string, std::string>::const_iterator disabled =
        config.find("CONFIG_GPTIMER_ENABLE_DEBUG_LOG");
    if (disabled == config.end()) {
        std::cerr << "FAIL: expected CONFIG_GPTIMER_ENABLE_DEBUG_LOG to be present as a disabled option\n";
        g_failures++;
        return;
    }
    if (!disabled->second.empty()) {
        std::cerr << "FAIL: parser did not treat '# CONFIG_X is not set' as disabled\n";
        g_failures++;
    }
}

} // namespace

int main(int argc, const char* argv[]) {
    if (argc < 2) {
        std::cerr << "usage: host_sdkconfig_isr_safety_tests <path-to-sdkconfig>" << std::endl;
        return 1;
    }

    const std::string path = argv[1];
    std::map<std::string, std::string> config;
    if (!parse_sdkconfig(path, &config)) {
        std::cerr << "FAILED: could not open sdkconfig at " << path << std::endl;
        return 1;
    }

    test_parser_distinguishes_set_from_not_set(config);
    for (const RequiredOption& option : REQUIRED_OPTIONS) {
        expect_option_enabled(config, option);
    }

    if (g_failures == 0) {
        std::cout << "PASS: host_sdkconfig_isr_safety_tests" << std::endl;
        return 0;
    }

    std::cerr << "FAILED: host_sdkconfig_isr_safety_tests failures=" << g_failures << std::endl;
    return 1;
}
