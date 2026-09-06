#include "tcu_io_data_source.h"

namespace TCUIO {

MockTcuIoActuatorController::MockTcuIoActuatorController() : last_() {
}

const char* MockTcuIoActuatorController::name() const {
    return "mock";
}

void MockTcuIoActuatorController::apply(const TcuIoActuatorFrame& frame) {
    last_ = frame;
}

bool MockTcuIoActuatorController::get_last(TcuIoActuatorFrame* out) const {
    if (out == nullptr) {
        return false;
    }
    *out = last_;
    return true;
}

PlaybackTcuIoDataSource::PlaybackTcuIoDataSource(const TcuIoHardwareFrame* frames, size_t frame_count, bool loop)
    : frames_(frames), frame_count_(frame_count), next_index_(0), loop_(loop) {
}

const char* PlaybackTcuIoDataSource::name() const {
    return "playback";
}

bool PlaybackTcuIoDataSource::read_frame(TcuIoHardwareFrame* out) {
    if (out == nullptr || frames_ == nullptr || frame_count_ == 0) {
        return false;
    }

    if (next_index_ >= frame_count_) {
        if (loop_) {
            next_index_ = 0;
        } else {
            *out = frames_[frame_count_ - 1];
            return true;
        }
    }

    *out = frames_[next_index_];
    next_index_++;
    return true;
}

void PlaybackTcuIoDataSource::reset() {
    next_index_ = 0;
}

size_t PlaybackTcuIoDataSource::frame_count() const {
    return frame_count_;
}

size_t PlaybackTcuIoDataSource::next_index() const {
    return next_index_;
}

TcuIoFrameRingBufferCaptureSink::TcuIoFrameRingBufferCaptureSink(TcuIoHardwareFrame* buffer, size_t capacity)
    : buffer_(buffer), capacity_(capacity), head_(0), size_(0), samples_written_(0) {
}

void TcuIoFrameRingBufferCaptureSink::on_frame(const TcuIoHardwareFrame& frame) {
    if (buffer_ == nullptr || capacity_ == 0) {
        return;
    }

    buffer_[head_] = frame;
    head_ = (head_ + 1) % capacity_;
    if (size_ < capacity_) {
        size_++;
    }
    samples_written_++;
}

size_t TcuIoFrameRingBufferCaptureSink::size() const {
    return size_;
}

size_t TcuIoFrameRingBufferCaptureSink::capacity() const {
    return capacity_;
}

size_t TcuIoFrameRingBufferCaptureSink::samples_written() const {
    return samples_written_;
}

bool TcuIoFrameRingBufferCaptureSink::get_latest(TcuIoHardwareFrame* out) const {
    if (out == nullptr || size_ == 0 || buffer_ == nullptr || capacity_ == 0) {
        return false;
    }

    size_t idx = (head_ + capacity_ - 1) % capacity_;
    *out = buffer_[idx];
    return true;
}

bool TcuIoFrameRingBufferCaptureSink::get_at(size_t offset_from_oldest, TcuIoHardwareFrame* out) const {
    if (out == nullptr || buffer_ == nullptr || capacity_ == 0 || offset_from_oldest >= size_) {
        return false;
    }

    size_t oldest_idx = (head_ + capacity_ - size_) % capacity_;
    size_t idx = (oldest_idx + offset_from_oldest) % capacity_;
    *out = buffer_[idx];
    return true;
}

TcuIoActuatorRingBufferCaptureSink::TcuIoActuatorRingBufferCaptureSink(TcuIoActuatorFrame* buffer, size_t capacity)
    : buffer_(buffer), capacity_(capacity), head_(0), size_(0), samples_written_(0) {
}

void TcuIoActuatorRingBufferCaptureSink::on_frame(const TcuIoActuatorFrame& frame) {
    if (buffer_ == nullptr || capacity_ == 0) {
        return;
    }

    buffer_[head_] = frame;
    head_ = (head_ + 1) % capacity_;
    if (size_ < capacity_) {
        size_++;
    }
    samples_written_++;
}

size_t TcuIoActuatorRingBufferCaptureSink::size() const {
    return size_;
}

size_t TcuIoActuatorRingBufferCaptureSink::capacity() const {
    return capacity_;
}

size_t TcuIoActuatorRingBufferCaptureSink::samples_written() const {
    return samples_written_;
}

bool TcuIoActuatorRingBufferCaptureSink::get_latest(TcuIoActuatorFrame* out) const {
    if (out == nullptr || size_ == 0 || buffer_ == nullptr || capacity_ == 0) {
        return false;
    }

    size_t idx = (head_ + capacity_ - 1) % capacity_;
    *out = buffer_[idx];
    return true;
}

bool TcuIoActuatorRingBufferCaptureSink::get_at(size_t offset_from_oldest, TcuIoActuatorFrame* out) const {
    if (out == nullptr || buffer_ == nullptr || capacity_ == 0 || offset_from_oldest >= size_) {
        return false;
    }

    size_t oldest_idx = (head_ + capacity_ - size_) % capacity_;
    size_t idx = (oldest_idx + offset_from_oldest) % capacity_;
    *out = buffer_[idx];
    return true;
}

} // namespace TCUIO