#ifndef TCU_IO_DATA_SOURCE_H
#define TCU_IO_DATA_SOURCE_H

#include <limits.h>
#include <stddef.h>
#include <stdint.h>

#include "canbus/can_defines.h"
#include "tcu_scaling.h"

namespace TCUIO {

struct TcuIoHardwareFrame {
    uint16_t rpm_n2 = UINT16_MAX;
    uint16_t rpm_n3 = UINT16_MAX;
    uint16_t rpm_out = UINT16_MAX;
    uint16_t battery_mv = UINT16_MAX;
    int atf_temp_c = INT_MAX;
    uint8_t parking_lock = UINT8_MAX;

    wheel_rpm_2x_t wheel_rr_2x_rpm = WheelSpeed::INVALID;
    wheel_rpm_2x_t wheel_rl_2x_rpm = WheelSpeed::INVALID;
    wheel_rpm_2x_t wheel_fr_2x_rpm = WheelSpeed::INVALID;
    wheel_rpm_2x_t wheel_fl_2x_rpm = WheelSpeed::INVALID;

    temp_c_t engine_coolant_temp_c = Temp::INVALID;
    temp_c_t engine_oil_temp_c = Temp::INVALID;
    TransferCaseState transfer_case_state = TransferCaseState::SNA;
};

class ITcuIoDataSource {
public:
    virtual ~ITcuIoDataSource() = default;
    virtual const char* name() const = 0;
    virtual bool read_frame(TcuIoHardwareFrame* out) = 0;
};

class ITcuIoCaptureSink {
public:
    virtual ~ITcuIoCaptureSink() = default;
    virtual void on_frame(const TcuIoHardwareFrame& frame) = 0;
};

struct TcuIoActuatorFrame {
    uint16_t mpc_current_target_ma = 0;
    uint16_t spc_current_target_ma = 0;
    uint16_t tcc_pwm_12bit = 0;
    bool y3_on = false;
    bool y4_on = false;
    bool y5_on = false;
};

class ITcuIoActuatorController {
public:
    virtual ~ITcuIoActuatorController() = default;
    virtual const char* name() const = 0;
    virtual void apply(const TcuIoActuatorFrame& frame) = 0;
    virtual bool get_last(TcuIoActuatorFrame* out) const = 0;
};

class ITcuIoActuatorCaptureSink {
public:
    virtual ~ITcuIoActuatorCaptureSink() = default;
    virtual void on_frame(const TcuIoActuatorFrame& frame) = 0;
};

class MockTcuIoActuatorController final : public ITcuIoActuatorController {
public:
    MockTcuIoActuatorController();

    const char* name() const override;
    void apply(const TcuIoActuatorFrame& frame) override;
    bool get_last(TcuIoActuatorFrame* out) const override;

private:
    TcuIoActuatorFrame last_;
};

class PlaybackTcuIoDataSource final : public ITcuIoDataSource {
public:
    PlaybackTcuIoDataSource(const TcuIoHardwareFrame* frames, size_t frame_count, bool loop);

    const char* name() const override;
    bool read_frame(TcuIoHardwareFrame* out) override;

    void reset();
    size_t frame_count() const;
    size_t next_index() const;

private:
    const TcuIoHardwareFrame* frames_;
    size_t frame_count_;
    size_t next_index_;
    bool loop_;
};

class TcuIoFrameRingBufferCaptureSink final : public ITcuIoCaptureSink {
public:
    TcuIoFrameRingBufferCaptureSink(TcuIoHardwareFrame* buffer, size_t capacity);

    void on_frame(const TcuIoHardwareFrame& frame) override;

    size_t size() const;
    size_t capacity() const;
    size_t samples_written() const;

    bool get_latest(TcuIoHardwareFrame* out) const;
    bool get_at(size_t offset_from_oldest, TcuIoHardwareFrame* out) const;

private:
    TcuIoHardwareFrame* buffer_;
    size_t capacity_;
    size_t head_;
    size_t size_;
    size_t samples_written_;
};

class TcuIoActuatorRingBufferCaptureSink final : public ITcuIoActuatorCaptureSink {
public:
    TcuIoActuatorRingBufferCaptureSink(TcuIoActuatorFrame* buffer, size_t capacity);

    void on_frame(const TcuIoActuatorFrame& frame) override;

    size_t size() const;
    size_t capacity() const;
    size_t samples_written() const;

    bool get_latest(TcuIoActuatorFrame* out) const;
    bool get_at(size_t offset_from_oldest, TcuIoActuatorFrame* out) const;

private:
    TcuIoActuatorFrame* buffer_;
    size_t capacity_;
    size_t head_;
    size_t size_;
    size_t samples_written_;
};

} // namespace TCUIO

#endif