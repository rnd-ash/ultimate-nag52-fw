#ifndef HFM_CAN_H
#define HFM_CAN_H

#include "can_hal.h"
#include "../../lib/hfm/src/HFM.h"
#include "shifter/shifter.h"
#include "stored_table.h"
#include "maps.h"

#include "shifter/shifter_trrs.h"

/// @brief The CAN-layer supports classic Mercedes-Benz cars from the early 1990's (e.g. 124, 202, 129, 140, 210) with a 125kbit/s CAN for HFM coming partially with M104 (L6) and M111 (L4). It does not support the 500kbit/s CAN coming with the M119 (V8) in cars of the same era.
class HfmCan: public EgsBaseCan {
    public:
        explicit HfmCan(const char* name, uint8_t tx_time_ms, uint32_t baud);

        /**
         * Getters
         */

        // Get the front right wheel data
        WheelData get_front_right_wheel(const uint32_t expire_time_ms)  override;
        // Get the front left wheel data
        WheelData get_front_left_wheel(const uint32_t expire_time_ms) override;
        // Get the rear right wheel data
        WheelData get_rear_right_wheel(const uint32_t expire_time_ms) override;
        // Get the rear left wheel data
        WheelData get_rear_left_wheel(const uint32_t expire_time_ms) override;
        // Gets shifter position from EWM module
        ShifterPosition get_shifter_position(const uint32_t expire_time_ms) override;
        // Gets engine type
        EngineType get_engine_type(const uint32_t expire_time_ms) override;
        // Returns true if engine is in limp mode
        bool get_engine_is_limp(const uint32_t expire_time_ms) override;
        // Returns true if pedal is kickdown 
         bool get_kickdown(const uint32_t expire_time_ms) override;
        // Returns the pedal percentage. Range 0-250
         uint8_t get_pedal_value(const uint32_t expire_time_ms) override;
        // Gets the current 'static' torque produced by the engine
         int get_static_engine_torque(const uint32_t expire_time_ms) override;
         int get_driver_engine_torque(const uint32_t expire_time_ms) override;
        // Gets the maximum engine torque allowed at this moment by the engine map
         int get_maximum_engine_torque(const uint32_t expire_time_ms) override;
        // Gets the minimum engine torque allowed at this moment by the engine map
         int get_minimum_engine_torque(const uint32_t expire_time_ms) override;
        // Gets the flappy paddle position
         PaddlePosition get_paddle_position(const uint32_t expire_time_ms) override;
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

        ProfileSwitchPos get_shifter_ws_mode(const uint32_t expire_time_ms) override;
        // 
        bool get_is_brake_pressed(const uint32_t expire_time_ms) override;


    protected:
        void on_rx_frame(uint32_t id,  uint8_t dlc, uint64_t data, const uint32_t timestamp) override;
        void on_rx_done(const uint32_t now_ts) override;

    private:
        // precalculated cosine values; from 0° till 89.25° in steps of 0.35° as per CAN-definition
        const float cosine[UINT8_MAX + 1u] = {1.F, 0.999981342F, 0.99992537F, 0.999832084F, 0.99970149F, 0.999533591F, 0.999328394F, 0.999085906F, 0.998806137F, 0.998489097F, 0.998134798F, 0.997743253F, 0.997314477F, 0.996848486F, 0.996345296F, 0.995804928F, 0.9952274F, 0.994612735F, 0.993960955F, 0.993272086F, 0.992546152F, 0.99178318F, 0.9909832F, 0.99014624F, 0.989272333F, 0.98836151F, 0.987413807F, 0.986429257F, 0.985407898F, 0.984349769F, 0.983254908F, 0.982123356F, 0.980955155F, 0.97975035F, 0.978508985F, 0.977231106F, 0.975916762F, 0.974566001F, 0.973178873F, 0.97175543F, 0.970295726F, 0.968799815F, 0.967267753F, 0.965699596F, 0.964095404F, 0.962455236F, 0.960779154F, 0.95906722F, 0.957319498F, 0.955536052F, 0.953716951F, 0.951862261F, 0.949972052F, 0.948046394F, 0.946085359F, 0.94408902F, 0.942057453F, 0.939990732F, 0.937888935F, 0.93575214F, 0.933580426F, 0.931373876F, 0.929132572F, 0.926856596F, 0.924546034F, 0.922200972F, 0.919821497F, 0.917407699F, 0.914959668F, 0.912477494F, 0.909961271F, 0.907411092F, 0.904827052F, 0.902209249F, 0.899557779F, 0.896872742F, 0.894154237F, 0.891402366F, 0.888617233F, 0.88579894F, 0.882947593F, 0.880063298F, 0.877146164F, 0.874196298F, 0.871213811F, 0.868198814F, 0.865151421F, 0.862071743F, 0.858959897F, 0.855815998F, 0.852640164F, 0.849432514F, 0.846193166F, 0.842922242F, 0.839619865F, 0.836286156F, 0.832921241F, 0.829525245F, 0.826098294F, 0.822640518F, 0.819152044F, 0.815633003F, 0.812083527F, 0.808503747F, 0.804893797F, 0.801253813F, 0.797583929F, 0.793884283F, 0.790155012F, 0.786396257F, 0.782608157F, 0.778790853F, 0.774944489F, 0.771069207F, 0.767165152F, 0.76323247F, 0.759271307F, 0.755281812F, 0.751264134F, 0.747218421F, 0.743144825F, 0.739043499F, 0.734914595F, 0.730758267F, 0.726574671F, 0.722363962F, 0.718126298F, 0.713861836F, 0.709570737F, 0.705253159F, 0.700909264F, 0.696539215F, 0.692143174F, 0.687721305F, 0.683273774F, 0.678800746F, 0.674302388F, 0.669778868F, 0.665230355F, 0.660657018F, 0.656059029F, 0.651436559F, 0.64678978F, 0.642118865F, 0.63742399F, 0.632705329F, 0.627963058F, 0.623197354F, 0.618408395F, 0.613596361F, 0.608761429F, 0.603903781F, 0.599023599F, 0.594121063F, 0.589196357F, 0.584249666F, 0.579281172F, 0.574291063F, 0.569279523F, 0.564246741F, 0.559192903F, 0.554118199F, 0.549022818F, 0.54390695F, 0.538770785F, 0.533614516F, 0.528438335F, 0.523242435F, 0.518027009F, 0.512792254F, 0.507538363F, 0.502265533F, 0.496973961F, 0.491663844F, 0.48633538F, 0.480988769F, 0.475624209F, 0.470241901F, 0.464842046F, 0.459424845F, 0.4539905F, 0.448539214F, 0.443071191F, 0.437586634F, 0.432085749F, 0.42656874F, 0.421035813F, 0.415487176F, 0.409923034F, 0.404343596F, 0.398749069F, 0.393139663F, 0.387515586F, 0.38187705F, 0.376224263F, 0.370557438F, 0.364876784F, 0.359182516F, 0.353474844F, 0.347753982F, 0.342020143F, 0.336273542F, 0.330514393F, 0.32474291F, 0.318959309F, 0.313163806F, 0.307356618F, 0.30153796F, 0.29570805F, 0.289867106F, 0.284015345F, 0.278152986F, 0.272280247F, 0.266397348F, 0.260504509F, 0.254601948F, 0.248689887F, 0.242768546F, 0.236838146F, 0.230898908F, 0.224951054F, 0.218994806F, 0.213030386F, 0.207058017F, 0.201077921F, 0.195090322F, 0.189095443F, 0.183093508F, 0.17708474F, 0.171069365F, 0.165047606F, 0.159019688F, 0.152985836F, 0.146946276F, 0.140901232F, 0.13485093F, 0.128795597F, 0.122735457F, 0.116670737F, 0.110601664F, 0.104528463F, 0.098451362F, 0.092370587F, 0.086286366F, 0.080198924F, 0.07410849F, 0.068015291F, 0.061919553F, 0.055821505F, 0.049721374F, 0.043619387F, 0.037515773F, 0.031410759F, 0.025304573F, 0.019197442F, 0.013089596F};
        const float temperature_offset =  -44.F;
        const float temperature_factor = 1.16078431372549F;
        const float air_mass_factor = 4.F;

        ECU_HFM hfm_ecu = ECU_HFM();
        
        bool start_enable = false;

        StoredTable *enginemaxtorque = new StoredTable(MAP_NAME_ENGINE_TORQUE_MAX, TORQUE_MAP_SIZE, ENGINE_TORQUE_HEADERS_MAP, TORQUE_MAP_SIZE, ENGINE_TORQUE_MAP);

        Shifter *shifter = new ShifterTrrs(&can_init_status);      

        WheelData generateWheelData(const uint32_t expire_time_ms) const;
};

#endif // HFM_CAN_H