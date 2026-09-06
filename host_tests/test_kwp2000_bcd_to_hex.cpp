#include <cstdint>
#include <iostream>

#include "kwp2000_logic.h"

namespace {

int g_failures = 0;

void expect_eq_u8(const char* name, uint8_t actual, uint8_t expected) {
    if (actual != expected) {
        std::cerr << "FAIL: " << name << " expected=" << (uint16_t)expected << " actual=" << (uint16_t)actual << "\n";
        g_failures++;
    }
}

uint8_t expected_bcd_to_hex_for_codepoint(uint8_t codepoint) {
    if (codepoint >= (uint8_t)'0' && codepoint <= (uint8_t)'9') {
        return (uint8_t)(codepoint - (uint8_t)'0');
    }
    if (codepoint >= (uint8_t)'A' && codepoint <= (uint8_t)'F') {
        return (uint8_t)(codepoint - (uint8_t)'A' + 10u);
    }
    if (codepoint >= (uint8_t)'a' && codepoint <= (uint8_t)'f') {
        return (uint8_t)(codepoint - (uint8_t)'a' + 10u);
    }
    return 0x0Fu;
}

uint8_t expected_dec_to_bcd_for_input(uint8_t value) {
    return (uint8_t)((((uint16_t)value / 10u) * 16u) + ((uint16_t)value % 10u));
}

uint8_t decode_packed_bcd_to_dec(uint8_t packed_bcd) {
    uint8_t tens = (uint8_t)((packed_bcd >> 4) & 0x0Fu);
    uint8_t ones = (uint8_t)(packed_bcd & 0x0Fu);
    return (uint8_t)(tens * 10u + ones);
}

void test_dec_to_bcd_known_values() {
    expect_eq_u8("dec 0", decToBcd(0u), 0x00u);
    expect_eq_u8("dec 9", decToBcd(9u), 0x09u);
    expect_eq_u8("dec 10", decToBcd(10u), 0x10u);
    expect_eq_u8("dec 12", decToBcd(12u), 0x12u);
    expect_eq_u8("dec 45", decToBcd(45u), 0x45u);
    expect_eq_u8("dec 99", decToBcd(99u), 0x99u);
}

void test_dec_to_bcd_roundtrip_for_00_to_99() {
    for (uint16_t i = 0; i <= 99u; i++) {
        uint8_t in = (uint8_t)i;
        uint8_t packed = decToBcd(in);
        uint8_t back = decode_packed_bcd_to_dec(packed);
        if (back != in) {
            std::cerr << "FAIL: decToBcd roundtrip in=" << (uint16_t)in
                      << " packed=" << (uint16_t)packed
                      << " back=" << (uint16_t)back << "\n";
            g_failures++;
        }
    }
}

void test_dec_to_bcd_exhaustive_all_u8_inputs() {
    for (uint16_t i = 0; i <= 0xFFu; i++) {
        uint8_t in = (uint8_t)i;
        uint8_t actual = decToBcd(in);
        uint8_t expected = expected_dec_to_bcd_for_input(in);
        if (actual != expected) {
            std::cerr << "FAIL: decToBcd exhaustive in=" << (uint16_t)in
                      << " expected=" << (uint16_t)expected
                      << " actual=" << (uint16_t)actual << "\n";
            g_failures++;
        }
    }
}

void test_bcd_to_hex_valid_digits() {
    expect_eq_u8("digit 0", bcd_to_hex('0'), 0u);
    expect_eq_u8("digit 5", bcd_to_hex('5'), 5u);
    expect_eq_u8("digit 9", bcd_to_hex('9'), 9u);
}

void test_bcd_to_hex_valid_uppercase() {
    expect_eq_u8("upper A", bcd_to_hex('A'), 10u);
    expect_eq_u8("upper C", bcd_to_hex('C'), 12u);
    expect_eq_u8("upper F", bcd_to_hex('F'), 15u);
}

void test_bcd_to_hex_valid_lowercase() {
    expect_eq_u8("lower a", bcd_to_hex('a'), 10u);
    expect_eq_u8("lower d", bcd_to_hex('d'), 13u);
    expect_eq_u8("lower f", bcd_to_hex('f'), 15u);
}

void test_bcd_to_hex_invalid_boundaries() {
    expect_eq_u8("before digits", bcd_to_hex('/'), 0x0Fu);
    expect_eq_u8("after digits", bcd_to_hex(':'), 0x0Fu);
    expect_eq_u8("before upper", bcd_to_hex('@'), 0x0Fu);
    expect_eq_u8("after upper", bcd_to_hex('G'), 0x0Fu);
    expect_eq_u8("before lower", bcd_to_hex('`'), 0x0Fu);
    expect_eq_u8("after lower", bcd_to_hex('g'), 0x0Fu);
}

void test_bcd_to_hex_exhaustive_ascii_and_extended_bytes() {
    for (uint16_t i = 0; i <= 0xFFu; i++) {
        uint8_t codepoint = (uint8_t)i;
        char input = (char)codepoint;
        uint8_t actual = bcd_to_hex(input);
        uint8_t expected = expected_bcd_to_hex_for_codepoint(codepoint);
        if (actual != expected) {
            std::cerr << "FAIL: exhaustive codepoint=" << (uint16_t)codepoint
                      << " expected=" << (uint16_t)expected
                      << " actual=" << (uint16_t)actual << "\n";
            g_failures++;
        }
    }
}

} // namespace

int main() {
    test_dec_to_bcd_known_values();
    test_dec_to_bcd_roundtrip_for_00_to_99();
    test_dec_to_bcd_exhaustive_all_u8_inputs();
    test_bcd_to_hex_valid_digits();
    test_bcd_to_hex_valid_uppercase();
    test_bcd_to_hex_valid_lowercase();
    test_bcd_to_hex_invalid_boundaries();
    test_bcd_to_hex_exhaustive_ascii_and_extended_bytes();

    if (g_failures == 0) {
        std::cout << "PASS: host_kwp2000_bcd_to_hex_tests" << std::endl;
        return 0;
    }

    std::cerr << "FAILED: host_kwp2000_bcd_to_hex_tests failures=" << g_failures << std::endl;
    return 1;
}