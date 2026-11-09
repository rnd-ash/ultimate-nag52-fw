// Real packed calibration layout, checksum, init and reload; allocation/flash are fake.
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
using esp_err_t=int;
constexpr int ESP_OK=0, ESP_ERR_NO_MEM=1, ESP_ERR_INVALID_VERSION=2,
              ESP_ERR_INVALID_SIZE=3, ESP_ERR_INVALID_CRC=4, ESP_ERR_INVALID_ARG=5,
              ESP_ERR_INVALID_STATE=6;
#define ESP_LOGE(...) ((void)0)
int allocations=0; bool fail_alloc=false,fail_read=false;
void* heap_alloc(size_t size) { if(fail_alloc) return nullptr; ++allocations; return std::malloc(size); }
void heap_free(void* ptr) { assert(ptr); --allocations; std::free(ptr); }
#define TCU_HEAP_ALLOC(n) heap_alloc(n)
#define TCU_FREE(p) heap_free(p)
int esp_flash_read(void*,void*,uint32_t,size_t);
#include "production.h"
CalibrationInfo flash{};
int esp_flash_read(void*,void* dest,uint32_t address,size_t size) {
    assert(address==CALIBRATION_START_ADDRESS && size==sizeof(CalibrationInfo));
    if(fail_read) return 99;
    std::memcpy(dest,&flash,size); return ESP_OK;
}
void checksum() { flash.crc=crc(reinterpret_cast<uint8_t*>(&flash)+8,sizeof(flash)-8); }
void valid() {
    flash={};flash.magic=0xDEADBEEF;flash.len=sizeof(flash);
    for(int i=1;i<=5;++i) flash.mech_cal.ratio_table[i]=1000+i;
    checksum();
}
void rejected(int expected) {
    const CalibrationInfo old=*CAL_RAM_PTR;
    for(int i=0;i<20;++i) {
        assert(EGSCal::reload_egs_calibration()==expected);
        assert(std::memcmp(&old,CAL_RAM_PTR,sizeof(old))==0);
        assert(allocations==1);
    }
}
int main() {
    valid();assert(EGSCal::reload_egs_calibration()==ESP_ERR_INVALID_STATE);assert(allocations==0);
    fail_alloc=true;assert(EGSCal::init_egs_calibration()==ESP_ERR_NO_MEM);fail_alloc=false;
    assert(EGSCal::init_egs_calibration()==ESP_OK && allocations==1);
    assert(MECH_PTR==&CAL_RAM_PTR->mech_cal && HYDR_PTR==&CAL_RAM_PTR->hydr_cal);
    flash.magic=0;rejected(ESP_ERR_INVALID_VERSION);
    valid();flash.len=0;rejected(ESP_ERR_INVALID_SIZE);
    valid();++flash.crc;rejected(ESP_ERR_INVALID_CRC);
    for(int gear=1;gear<=5;++gear) { valid();flash.mech_cal.ratio_table[gear]=0;checksum();rejected(ESP_ERR_INVALID_ARG); }
    valid();fail_read=true;rejected(99);fail_read=false;
    fail_alloc=true;rejected(ESP_ERR_NO_MEM);fail_alloc=false;
    // A valid replacement must be judged against its own ratios, even if old RAM is invalid.
    CAL_RAM_PTR->mech_cal.ratio_table[1]=0;
    valid();flash.mech_cal.ratio_table[1]=2000;checksum();
    assert(EGSCal::reload_egs_calibration()==ESP_OK && allocations==1);
    assert(std::memcmp(&flash,CAL_RAM_PTR,sizeof(flash))==0);
    heap_free(CAL_RAM_PTR);CAL_RAM_PTR=nullptr;assert(allocations==0);
    puts("PASS: actual packed layout, wrong-buffer regression, repeated failure cleanup and atomic replacement");
}
