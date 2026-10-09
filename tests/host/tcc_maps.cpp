// Compile the real class and constructor; map allocation/storage are fake.
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <new>
#include <cstdio>
#define ESP_OK 0
#define ESP_LOGE(...) ((void)0)
enum class GearboxGear { First, Second, Third, Fourth, Fifth };
enum class TccClutchStatus { Open };
struct SensorData { int atf_temp = 50; };
struct PressureManager {};
struct AbstractProfile {};
struct { int tcc_max_trq_override = 0; } TCC_CURRENT_SETTINGS;
const char* NVS_KEY_TCC_ADAPT_MAP_1 = "1";
const char* NVS_KEY_TCC_ADAPT_MAP_2 = "2";
const char* NVS_KEY_TCC_ADAPT_MAP_3 = "3";
const char* NVS_KEY_TCC_ADAPT_MAP_4 = "4";
const char* NVS_KEY_TCC_ADAPT_MAP_5 = "5";
const char* NVS_KEY_TCC_SLIP_TARGET_MAP = "6";
const int TCC_ADAPT_MAP_Z_SIZE = 36, TCC_RPM_TARGET_MAP_SIZE = 88;
const int16_t TCC_ADAPT_MAP_X[6] = {}, TCC_ADAPT_MAP_Y[6] = {};
const int16_t TCC_ADAPT_MAP_Z[36] = {}, TCC_RPM_TARGET_MAP[88] = {};
const int16_t rpm_map_x_headers[11] = {}, rpm_map_y_headers[8] = {};
struct StoredMap {
    static inline int attempt = 0, failed_alloc = 0, failed_init = 0, alive = 0;
    int id;
    static void* operator new(size_t n, const std::nothrow_t&) noexcept {
        ++attempt;
        return attempt == failed_alloc ? nullptr : std::malloc(n);
    }
    static void operator delete(void* p) noexcept { std::free(p); }
    static void operator delete(void* p, const std::nothrow_t&) noexcept { std::free(p); }
    template<class... A> StoredMap(A...) : id(attempt) { ++alive; }
    ~StoredMap() { --alive; }
    int init_status() const { return id == failed_init ? 1 : ESP_OK; }
    int get_value(int, int) { return 0; }
    void save_to_eeprom() {}
};
#include "production.h"
int main() {
    for (int alloc = 0; alloc <= 6; ++alloc) {
        for (int init = 0; init <= 6; ++init) {
            StoredMap::attempt = 0; StoredMap::failed_alloc = alloc; StoredMap::failed_init = init;
            TorqueConverter t(300);
            assert(t.init_tables_ok == (alloc == 0 && init == 0));
            StoredMap* maps[] = {t.tcc_adapt_map_d1, t.tcc_adapt_map_d2, t.tcc_adapt_map_d3,
                                t.tcc_adapt_map_d4, t.tcc_adapt_map_d5, t.slip_rpm_target_map};
            for (int i = 0; i < 6; ++i) {
                assert((maps[i] == nullptr) == (i+1 == alloc || i+1 == init));
                delete maps[i];
            }
            assert(StoredMap::alive == 0);
        }
    }
    puts("PASS: six map allocation/init failures, 49 combinations");
}
