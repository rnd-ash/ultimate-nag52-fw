#ifndef CALIBRATION_STRUCT_H
#define CALIBRATION_STRUCT_H

#include <stdint.h>
#include "esp_err.h"

/**
 * Calibration data from EGS52/53 ported to UN52
 * 
 * !!DANGER DANGER DANGER!!
 * This calibration can only be writeen/read by Read memory/Write memory KWP diagnostic IDs
 * We copy this data to SPIRAM on TCU boot. This way, when we write to this memory, it is possible
 *   to completely erase this memory region without suddenly affecting the TCU. We can issue
 *   a command to the TCU after erase/write flash to load the new calibration
 * 
*/

#define CALIBRATION_MAX_LEN 64*1024 // 64Kb for calibration area
#define CALIBRATION_START_ADDRESS 0x349000 // In Flash (Persistant partition - survives OTA)
#define CALIBRATION_END_ADDRESS CALIBRATION_START_ADDRESS + CALIBRATION_MAX_LEN

#define SHIFT_ARRAY_LEN 8 // Arrays with all 8 shifts (1-2 2-3 3-4 4-5 2-1 3-2 4-3 5-4)

struct ShiftAlgorithmPack{
    // Momentum (Up)
    // X: Output speed (x30)
    // Y: Engine torque (Static) (x5)
    // Z: Momentum (Nm) (x5)
    uint8_t momentum_1_2_x[3];
    uint8_t momentum_2_3_x[3];
    uint8_t momentum_3_4_x[3];
    uint8_t momentum_4_5_x[3];
    uint8_t momentum_1_2_y[2];
    uint8_t momentum_2_3_y[2];
    uint8_t momentum_3_4_y[2];
    uint8_t momentum_4_5_y[2];
    uint8_t momentum_1_2_z[6];
    uint8_t momentum_2_3_z[6];
    uint8_t momentum_3_4_z[6];
    uint8_t momentum_4_5_z[6];
    // Momentum (Down)
    uint8_t momentum_2_1_x[6];
    uint8_t momentum_3_2_x[6];
    uint8_t momentum_4_3_x[6];
    uint8_t momentum_5_4_x[6];
    uint8_t momentum_2_1_y[10];
    uint8_t momentum_3_2_y[10];
    uint8_t momentum_4_3_y[10];
    uint8_t momentum_5_4_y[10];
    uint8_t momentum_2_1_z[60];
    uint8_t momentum_3_2_z[60];
    uint8_t momentum_4_3_z[60];
    uint8_t momentum_5_4_z[60];
    // Torque adder (Up)
    uint8_t trq_adder_1_2_x[6];
    uint8_t trq_adder_2_3_x[6];
    uint8_t trq_adder_3_4_x[6];
    uint8_t trq_adder_4_5_x[6];
    uint8_t trq_adder_1_2_y[8];
    uint8_t trq_adder_2_3_y[8];
    uint8_t trq_adder_3_4_y[8];
    uint8_t trq_adder_4_5_y[8];
    uint8_t trq_adder_1_2_z[48];
    uint8_t trq_adder_2_3_z[48];
    uint8_t trq_adder_3_4_z[48];
    uint8_t trq_adder_4_5_z[48];
    // Torque adder (Down)
    uint8_t trq_adder_2_1_x[3];
    uint8_t trq_adder_3_2_x[3];
    uint8_t trq_adder_4_3_x[3];
    uint8_t trq_adder_5_4_x[3];
    uint8_t trq_adder_2_1_y[4];
    uint8_t trq_adder_3_2_y[4];
    uint8_t trq_adder_4_3_y[4];
    uint8_t trq_adder_5_4_y[4];
    uint8_t trq_adder_2_1_z[12];
    uint8_t trq_adder_3_2_z[12];
    uint8_t trq_adder_4_3_z[12];
    uint8_t trq_adder_5_4_z[12];
} __attribute__ ((packed));

struct HydraulicCalibration{
    uint16_t p_multi_1;
    uint16_t p_multi_other;
    uint16_t lp_reg_spring_pressure;
    uint16_t overlap_circuit_factor_spc[SHIFT_ARRAY_LEN];
    uint16_t overlap_circuit_factor_mpc[SHIFT_ARRAY_LEN];
    int16_t overlap_circuit_spring_pressure[SHIFT_ARRAY_LEN];
    uint16_t shift_reg_spring_pressure;
    uint16_t shift_spc_gain[SHIFT_ARRAY_LEN];
    uint16_t min_mpc_pressure;
    uint8_t unk1;
    uint8_t unk2;
    uint16_t unk3;
    uint16_t unk4;
    uint16_t unk5;
    uint16_t shift_pressure_addr_percent;
    uint16_t inlet_pressure_offset;
    uint16_t inlet_pressure_input_min;
    uint16_t inlet_pressure_input_max;
    uint16_t inlet_pressure_output_min;
    uint16_t inlet_pressure_output_max;
    uint16_t extra_pressure_pump_speed_min;
    uint16_t extra_pressure_pump_speed_max;
    uint16_t extra_pressure_adder_r1_1;
    uint16_t extra_pressure_adder_other_gears;
    uint16_t shift_pressure_factor_percent;
    uint16_t pcs_map_x[7];
    uint16_t pcs_map_y[4];
    uint16_t pcs_map_z[28];
} __attribute__ ((packed));

struct MechanicalCalibration{
    uint8_t gb_ty;
    uint16_t ratio_table[8];
    uint16_t intertia_factor[8];
    uint16_t friction_map[48];
    uint16_t max_torque_on_clutch[4];
    uint16_t max_torque_off_clutch[4];
    uint16_t release_spring_pressure[6];
    uint16_t intertia_torque[8];
    uint8_t strongest_loaded_clutch_idx[8];
    uint16_t turbine_drag[8];
    uint16_t atf_density_minus_50c;
    uint16_t atf_density_drop_per_c;
    uint16_t atf_density_centrifugal_force_factor[3];
} __attribute__ ((packed)) ;

struct TorqueConverterCalibration{
    uint16_t multiplier_map_x[2];
    uint16_t multiplier_map_z[2];
    uint16_t pump_map_x[11];
    uint16_t pump_map_z[11];
} __attribute__ ((packed));

struct CalibrationInfo{
    uint32_t magic;
    uint16_t len;
    uint16_t crc;
    char tcc_cal_name[16];
    TorqueConverterCalibration tcc_cal;
    char mech_cal_name[16];
    MechanicalCalibration mech_cal;
    char hydr_cal_name[16];
    HydraulicCalibration hydr_cal;
    char shift_algo_pack_name[16];
    ShiftAlgorithmPack shift_algo_cal;
} __attribute__ ((packed)) ;
// To check if we overflow
static_assert((sizeof(CalibrationInfo)) < CALIBRATION_MAX_LEN);

extern CalibrationInfo* CAL_RAM_PTR;
extern HydraulicCalibration* HYDR_PTR;
extern MechanicalCalibration* MECH_PTR;
extern TorqueConverterCalibration* TCC_CFG_PTR;
extern ShiftAlgorithmPack* SHIFT_ALGO_CFG_PTR;

namespace EGSCal {
    esp_err_t init_egs_calibration(void);
    esp_err_t reload_egs_calibration(void);
}

#endif