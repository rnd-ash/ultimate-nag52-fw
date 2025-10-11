#ifndef HFM_CAN_H
#define HFM_CAN_H

#include "can_hal.h"
#include "../../lib/hfm/src/HFM.h"
#include "shifter/shifter.h"
#include "stored_table.h"
#include "maps.h"
#include "nvs/all_keys.h"

#include "shifter/shifter_trrs.h"

/// @brief The CAN-layer supports classic Mercedes-Benz cars from the early 1990's (e.g. 124, 202, 129, 140, 210) with a 125kbit/s CAN for HFM coming partially with M104 (L6) and M111 (L4). It does not support the 500kbit/s CAN coming with the M119 (V8) in cars of the same era.
class HfmCan: public EgsBaseCan {
    public:
        HfmCan(const char* name, uint8_t tx_time_ms);

        /**
         * Getters
         */

        // Get the front right wheel data
        uint16_t get_front_right_wheel(const uint32_t expire_time_ms)  override;
        // Get the front left wheel data
        uint16_t get_front_left_wheel(const uint32_t expire_time_ms) override;
        // Get the rear right wheel data
        uint16_t get_rear_right_wheel(const uint32_t expire_time_ms) override;
        // Get the rear left wheel data
        uint16_t get_rear_left_wheel(const uint32_t expire_time_ms) override;
        // Gets engine type
        EngineType get_engine_type(const uint32_t expire_time_ms) override;
        // Returns true if engine is in limp mode
        bool get_engine_is_limp(const uint32_t expire_time_ms) override;
        // Returns true if pedal is kickdown 
         bool get_kickdown(const uint32_t expire_time_ms) override;
        // Returns the pedal percentage. Range 0-250
         uint8_t get_pedal_value(const uint32_t expire_time_ms) override;
        // Gets Torque information
        CanTorqueData get_torque_data(const uint32_t expire_time_ms) override;
        float get_ML(const uint32_t expire_time_ms);
        // Gets engine coolant temperature
        int16_t get_engine_coolant_temp(const uint32_t expire_time_ms) override;
        // Gets engine oil temperature
         int16_t get_engine_oil_temp(const uint32_t expire_time_ms) override;
         // Gets engine charge air temperature
        int16_t get_engine_iat_temp(const uint32_t expire_time_ms) override;
        // Gets engine RPM
         uint16_t get_engine_rpm(const uint32_t expire_time_ms) override;
        // Returns true if engine is cranking
        bool get_is_starting(const uint32_t expire_time_ms) override;

        // 
        bool get_is_brake_pressed(const uint32_t expire_time_ms) override;


    protected:
        void on_rx_frame(uint32_t id,  uint8_t dlc, uint64_t data, const uint32_t timestamp) override;

    private:
        // precalculated cosine values; from 0° till 89.25° in steps of 0.35° as per CAN-definition
        const float cosine[UINT8_MAX + 1u] = {1.F, 0.999981342F, 0.99992537F, 0.999832084F, 0.99970149F, 0.999533591F, 0.999328394F, 0.999085906F, 0.998806137F, 0.998489097F, 0.998134798F, 0.997743253F, 0.997314477F, 0.996848486F, 0.996345296F, 0.995804928F, 0.9952274F, 0.994612735F, 0.993960955F, 0.993272086F, 0.992546152F, 0.99178318F, 0.9909832F, 0.99014624F, 0.989272333F, 0.98836151F, 0.987413807F, 0.986429257F, 0.985407898F, 0.984349769F, 0.983254908F, 0.982123356F, 0.980955155F, 0.97975035F, 0.978508985F, 0.977231106F, 0.975916762F, 0.974566001F, 0.973178873F, 0.97175543F, 0.970295726F, 0.968799815F, 0.967267753F, 0.965699596F, 0.964095404F, 0.962455236F, 0.960779154F, 0.95906722F, 0.957319498F, 0.955536052F, 0.953716951F, 0.951862261F, 0.949972052F, 0.948046394F, 0.946085359F, 0.94408902F, 0.942057453F, 0.939990732F, 0.937888935F, 0.93575214F, 0.933580426F, 0.931373876F, 0.929132572F, 0.926856596F, 0.924546034F, 0.922200972F, 0.919821497F, 0.917407699F, 0.914959668F, 0.912477494F, 0.909961271F, 0.907411092F, 0.904827052F, 0.902209249F, 0.899557779F, 0.896872742F, 0.894154237F, 0.891402366F, 0.888617233F, 0.88579894F, 0.882947593F, 0.880063298F, 0.877146164F, 0.874196298F, 0.871213811F, 0.868198814F, 0.865151421F, 0.862071743F, 0.858959897F, 0.855815998F, 0.852640164F, 0.849432514F, 0.846193166F, 0.842922242F, 0.839619865F, 0.836286156F, 0.832921241F, 0.829525245F, 0.826098294F, 0.822640518F, 0.819152044F, 0.815633003F, 0.812083527F, 0.808503747F, 0.804893797F, 0.801253813F, 0.797583929F, 0.793884283F, 0.790155012F, 0.786396257F, 0.782608157F, 0.778790853F, 0.774944489F, 0.771069207F, 0.767165152F, 0.76323247F, 0.759271307F, 0.755281812F, 0.751264134F, 0.747218421F, 0.743144825F, 0.739043499F, 0.734914595F, 0.730758267F, 0.726574671F, 0.722363962F, 0.718126298F, 0.713861836F, 0.709570737F, 0.705253159F, 0.700909264F, 0.696539215F, 0.692143174F, 0.687721305F, 0.683273774F, 0.678800746F, 0.674302388F, 0.669778868F, 0.665230355F, 0.660657018F, 0.656059029F, 0.651436559F, 0.64678978F, 0.642118865F, 0.63742399F, 0.632705329F, 0.627963058F, 0.623197354F, 0.618408395F, 0.613596361F, 0.608761429F, 0.603903781F, 0.599023599F, 0.594121063F, 0.589196357F, 0.584249666F, 0.579281172F, 0.574291063F, 0.569279523F, 0.564246741F, 0.559192903F, 0.554118199F, 0.549022818F, 0.54390695F, 0.538770785F, 0.533614516F, 0.528438335F, 0.523242435F, 0.518027009F, 0.512792254F, 0.507538363F, 0.502265533F, 0.496973961F, 0.491663844F, 0.48633538F, 0.480988769F, 0.475624209F, 0.470241901F, 0.464842046F, 0.459424845F, 0.4539905F, 0.448539214F, 0.443071191F, 0.437586634F, 0.432085749F, 0.42656874F, 0.421035813F, 0.415487176F, 0.409923034F, 0.404343596F, 0.398749069F, 0.393139663F, 0.387515586F, 0.38187705F, 0.376224263F, 0.370557438F, 0.364876784F, 0.359182516F, 0.353474844F, 0.347753982F, 0.342020143F, 0.336273542F, 0.330514393F, 0.32474291F, 0.318959309F, 0.313163806F, 0.307356618F, 0.30153796F, 0.29570805F, 0.289867106F, 0.284015345F, 0.278152986F, 0.272280247F, 0.266397348F, 0.260504509F, 0.254601948F, 0.248689887F, 0.242768546F, 0.236838146F, 0.230898908F, 0.224951054F, 0.218994806F, 0.213030386F, 0.207058017F, 0.201077921F, 0.195090322F, 0.189095443F, 0.183093508F, 0.17708474F, 0.171069365F, 0.165047606F, 0.159019688F, 0.152985836F, 0.146946276F, 0.140901232F, 0.13485093F, 0.128795597F, 0.122735457F, 0.116670737F, 0.110601664F, 0.104528463F, 0.098451362F, 0.092370587F, 0.086286366F, 0.080198924F, 0.07410849F, 0.068015291F, 0.061919553F, 0.055821505F, 0.049721374F, 0.043619387F, 0.037515773F, 0.031410759F, 0.025304573F, 0.019197442F, 0.013089596F};
        
        //temperature of intake air in K
        const int16_t INTAKE_AIR_TEMPERATURE[256] = {380, 380, 380, 380, 380, 380, 380, 380, 380, 380, 380, 380, 380, 380, 380, 380, 380, 380, 380, 408, 406, 404, 401, 399, 397, 395, 393, 392, 390, 388, 387, 385, 384, 384, 381, 380, 378, 377, 376, 375, 374, 372, 371, 370, 369, 368, 367, 366, 365, 364, 364, 363, 362, 361, 360, 359, 358, 358, 357, 356, 355, 354, 354, 353, 352, 352, 351, 350, 349, 349, 348, 347, 347, 346, 346, 345, 344, 344, 343, 342, 342, 341, 341, 340, 340, 339, 338, 338, 337, 337, 336, 336, 335, 335, 334, 334, 333, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393, 393};

        // temperature of coolant in °C
        const int16_t COOLANT_TEMPERATURE[256] = {70, 70, 70, 70, 70, 70, 130, 125, 120, 117, 113, 110, 107, 103, 100, 98, 96, 94, 92, 90, 89, 87, 86, 84, 83, 81, 80, 79, 78, 77, 76, 74, 73, 72, 71, 70, 69, 68, 68, 67, 66, 65, 64, 63, 63, 62, 61, 60, 59, 59, 58, 58, 57, 56, 56, 55, 54, 54, 53, 53, 52, 51, 51, 50, 50, 49, 49, 48, 48, 47, 47, 46, 46, 45, 45, 44, 44, 43, 43, 42, 42, 41, 41, 40, 40, 39, 39, 38, 38, 38, 37, 37, 36, 36, 35, 35, 35, 34, 34, 33, 33, 33, 32, 32, 31, 31, 30, 30, 30, 29, 29, 29, 28, 28, 28, 27, 27, 26, 26, 26, 25, 25, 25, 24, 24, 24, 23, 23, 23, 22, 22, 21, 21, 21, 20, 20, 20, 19, 19, 19, 18, 18, 18, 17, 17, 17, 16, 16, 16, 15, 15, 15, 14, 14, 13, 13, 13, 12, 12, 12, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 5, 5, 5, 4, 4, 4, 3, 3, 2, 2, 2, 1, 1, 0, 0, 0, -1, -1, -2, -2, -3, -3, -4, -4, -4, -5, -5, -6, -6, -7, -7, -7, -8, -8, -9, -9, -10, -10, -11, -11, -12, -13, -13, -14, -14, -15, -16, -16, -17, -18, -18, -19, -19, -20, -21, -22, -23, -24, -25, -26, -26, -27, -28, -29, -30, -31, -33, -34, -36, -37, -39, -40, 70, 70, 70, 70, 70, 70, 70, 70};

        // TODO: make this configurable
        // number of cylinders [-]
        const uint8_t n_cylinders = 6u;

        // TODO: make this configurable
        // engine displacement [m³]
        const float V_H = 0.003199F;

        const float AIR_MASS_FACTOR = 4.F; // conversion factor for raw-value from CAN

        static const uint8_t M_MAX_LEN = 33u;
        // TODO: make this configurable
        // maximum engine torque for M104 E32 [Nm]
        const uint16_t M_MAX[M_MAX_LEN] = {0u, 61u, 121u, 182u, 242u, 251u, 259u, 268u, 276u, 280u, 284u, 288u, 292u, 299u, 306u, 310u, 309u, 308u, 304u, 300u, 296u, 290u, 282u, 266u, 250u, 235u, 219u, 205u, 190u, 176u, 161u, 147u, 132u};

        // TODO: make this configurable
        // engine speed for torque table [1/min]
        const uint16_t n[M_MAX_LEN] = {0u, 250u, 500u, 750u, 1000u, 1250u, 1500u, 1750u, 2000u, 2250u, 2500u, 2750u, 3000u, 3250u, 3500u, 3750u, 4000u, 4250u, 4500u, 4750u, 5000u, 5250u, 5500u, 5750u, 6000u, 6250u, 6500u, 6750u, 7000u, 7250u, 7500u, 7750u, 8000};

        // volumetric efficiency [Nm * s / kg], assumed constant
        const float c_engine = 221505.69;

        // limit for compatibility with other CAN-layers
        const int32_t PEDAL_VALUE_LIMIT = 250; 

        // contains a maximum value for the throttle [0.35°]
        float max_throttle_value = (float)UINT8_MAX;

        // air density [kg/m³]
        float current_air_pressure = 0.0f;
        // intakte air temperature [K]
        float current_intake_air_temp = 0.0f;

        ECU_HFM hfm_ecu = ECU_HFM();
        
        bool start_enable = false;

        StoredTable *enginemaxtorque = new StoredTable(NVS_KEY_MAP_NAME_HFM_TORQUE_MAP, TORQUE_MAP_SIZE, ENGINE_TORQUE_HEADERS_MAP, TORQUE_MAP_SIZE, ENGINE_TORQUE_MAP);

        uint16_t generateWheelData(const uint32_t expire_time_ms) const;
};

#endif // HFM_CAN_H